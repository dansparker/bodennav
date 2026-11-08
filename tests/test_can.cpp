#include "app/state.hpp"
#include "can/canaerospace.hpp"
#include "unit.hpp"

#include <algorithm>

using namespace bodennav;

TEST(float_frame_layout) {
    can::Encoder enc(40);
    const auto f = enc.float_value(can::id::kVor1Radial, 123.5f);
    CHECK(f.id == 1126);
    CHECK(f.dlc == 8);
    CHECK(f.data[0] == 40);   // node ID
    CHECK(f.data[1] == 2);    // FLOAT
    CHECK(f.data[2] == 0);    // service code
    CHECK(f.data[3] == 0);    // message code
    CHECK(f.data[4] == 0x42 && f.data[5] == 0xF7 && f.data[6] == 0x00 && f.data[7] == 0x00);  // 123.5f big endian
    const auto g = enc.float_value(can::id::kVor1Radial, 1.0f);
    CHECK(g.data[3] == 1);    // counts per identifier
    const auto h = enc.float_value(can::id::kIls1Localizer, 1.0f);
    CHECK(h.data[3] == 0);
    const auto m = can::decode(g);
    CHECK(m.has_value());
    CHECK(m->as_float().value_or(0.0f) == 1.0f);
}

TEST(message_code_wraps) {
    can::Encoder enc(1);
    can::Frame f;
    for (int i = 0; i < 257; ++i) f = enc.uchar_value(can::id::kMarkerBeacons, 0);
    CHECK(f.data[3] == 0);  // 0..255, then 0 again
    CHECK(f.dlc == 5);
    CHECK(f.data[1] == 10);  // UCHAR
}

TEST(achar4_ident) {
    can::Encoder enc(40);
    const auto f = enc.achar4(can::id::kNav1Ident, "OEV");
    CHECK(f.id == 1502);
    CHECK(f.data[1] == 25);
    CHECK(f.data[4] == 'O' && f.data[5] == 'E' && f.data[6] == 'V' && f.data[7] == 0);
    CHECK(can::decode(f)->as_achar() == "OEV");
}

TEST(identification_service) {
    can::Frame request;
    request.id = can::id::kIdsRequest;
    request.dlc = 4;
    request.data = {0, 0, 0, 7};  // broadcast (node 0), NODATA, service 0, message code 7
    CHECK(can::is_ids_request(*can::decode(request), 40));
    request.data[0] = 41;
    CHECK(!can::is_ids_request(*can::decode(request), 40));
    can::Encoder enc(40);
    const auto r = enc.ids_response(7, 1, 2);
    CHECK(r.id == 129 && r.data[0] == 40 && r.data[1] == 16 && r.data[3] == 7 && r.data[4] == 1 && r.data[5] == 2);
}

TEST(candump_line) {
    can::Encoder enc(40);
    const auto f = enc.float_value(can::id::kVor1Radial, 123.5f);
    CHECK(can::to_candump(f, 1.5, "vcan0") == "(1.500000) vcan0 466#2802000042F70000");
}

TEST(publish_schedule_and_timeouts) {
    using namespace std::chrono_literals;
    app::NavState s;
    const auto now = app::Clock::now();
    s.nav_mhz = 110.30;
    s.localizer = app::Timed{1.0, now - 50ms};
    s.glideslope = app::Timed{-0.2, now - 50ms};
    s.nav_ident = "IVW";
    s.adf_khz = 339.0;
    s.adf_bearing = app::Timed{45.0, now - 2s};  // too old
    can::Encoder enc(40);
    auto ids = [](const std::vector<can::Frame>& frames) {
        std::vector<std::uint32_t> v;
        for (const auto& f : frames) v.push_back(f.id);
        std::sort(v.begin(), v.end());
        return v;
    };
    const auto t0 = ids(app::frames_for_tick(s, 0, now, enc));
    CHECK((t0 == std::vector<std::uint32_t>{1087, 1091, 1104, 1108, 1502}));
    const auto t1 = ids(app::frames_for_tick(s, 1, now, enc));
    CHECK((t1 == std::vector<std::uint32_t>{1087, 1091}));
    // without a localizer no glide path either, and no identifier
    s.localizer.reset();
    const auto t2 = ids(app::frames_for_tick(s, 20, now, enc));
    CHECK((t2 == std::vector<std::uint32_t>{1104, 1108}));
}
