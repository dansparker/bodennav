#include "app/state.hpp"

namespace bodennav::app {

namespace {

bool fresh(const std::optional<Timed>& v, Clock::time_point now, Clock::duration max_age) {
    return v.has_value() && now - v->at <= max_age;
}

}  // namespace

std::vector<can::Frame> frames_for_tick(const NavState& s, unsigned tick, Clock::time_point now, can::Encoder& encoder,
                                        const PublishOptions& o) {
    std::vector<can::Frame> frames;
    const bool loc = fresh(s.localizer, now, o.max_age_nav);
    const bool radial = fresh(s.radial, now, o.max_age_nav);

    if (loc) frames.push_back(encoder.float_value(can::id::kIls1Localizer, static_cast<float>(s.localizer->value)));
    if (loc && fresh(s.glideslope, now, o.max_age_nav)) {
        frames.push_back(encoder.float_value(can::id::kIls1Glideslope, static_cast<float>(s.glideslope->value)));
    }
    if (s.markers.has_value() && now - s.markers->at <= o.max_age_marker) {
        frames.push_back(encoder.uchar_value(can::id::kMarkerBeacons, static_cast<std::uint8_t>(s.markers->value)));
    }
    if (tick % 2 == 0) {
        if (radial) frames.push_back(encoder.float_value(can::id::kVor1Radial, static_cast<float>(s.radial->value)));
        if (fresh(s.adf_bearing, now, o.max_age_adf)) {
            frames.push_back(encoder.float_value(can::id::kAdf1Bearing, static_cast<float>(s.adf_bearing->value)));
        }
    }
    if (tick % 10 == 0) {
        if (s.nav_mhz) frames.push_back(encoder.float_value(can::id::kNav1Frequency, static_cast<float>(*s.nav_mhz)));
        if (s.adf_khz) frames.push_back(encoder.float_value(can::id::kAdf1Frequency, static_cast<float>(*s.adf_khz)));
    }
    if (tick % 20 == 0) {
        if ((loc || radial) && !s.nav_ident.empty()) frames.push_back(encoder.achar4(can::id::kNav1Ident, s.nav_ident));
        const bool adf_heard = s.adf_ident_at.has_value() && now - *s.adf_ident_at <= o.adf_ident_hold;
        if (!s.adf_ident.empty() && (adf_heard || fresh(s.adf_bearing, now, o.max_age_adf))) {
            frames.push_back(encoder.achar4(can::id::kAdf1Ident, s.adf_ident));
        }
    }
    return frames;
}

}  // namespace bodennav::app
