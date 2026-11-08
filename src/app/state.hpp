#pragma once

// What the receivers currently know, and the CAN frames sent from it.

#include "can/canaerospace.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace bodennav::app {

using Clock = std::chrono::steady_clock;

struct Timed {
    double value{0.0};
    Clock::time_point at{};
};

struct NavState {
    std::optional<double> nav_mhz;  // tuned NAV frequency (NAV receiver present)
    std::optional<double> adf_khz;  // tuned ADF frequency (ADF receiver present)
    std::optional<Timed> radial;       // deg magnetic FROM
    std::optional<Timed> localizer;    // deg, + aircraft right
    std::optional<Timed> glideslope;   // deg, + aircraft above
    std::optional<Timed> adf_bearing;  // deg relative, clockwise
    std::optional<Timed> markers;      // bits (marker receiver present)
    std::string nav_ident;
    std::string adf_ident;
    std::optional<Clock::time_point> adf_ident_at;  // last time the ADF identifier was decoded
};

struct PublishOptions {
    Clock::duration max_age_nav{std::chrono::milliseconds(400)};    // VOR/LOC/GS: new value every 100 ms
    Clock::duration max_age_adf{std::chrono::milliseconds(1200)};   // ADF: new value every 500 ms
    Clock::duration max_age_marker{std::chrono::milliseconds(200)};
    Clock::duration adf_ident_hold{std::chrono::seconds(60)};
};

// Ticks at 20 Hz: deviations and markers every tick, radial and ADF bearing at
// 10 Hz, frequencies at 2 Hz, identifiers at 1 Hz. Values are only sent while
// valid (OpenEFIS: no frame for 500 ms = no signal).
[[nodiscard]] std::vector<can::Frame> frames_for_tick(const NavState& state, unsigned tick, Clock::time_point now,
                                                      can::Encoder& encoder, const PublishOptions& options = {});

}  // namespace bodennav::app
