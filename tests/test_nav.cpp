#include "dsp/dsp.hpp"
#include "io/source.hpp"
#include "nav/adf.hpp"
#include "nav/frequency.hpp"
#include "nav/ils.hpp"
#include "nav/marker.hpp"
#include "nav/vor.hpp"
#include "sim/siggen.hpp"
#include "unit.hpp"

#include <cstdio>
#include <set>

using namespace bodennav;

namespace {

sim::Common common(double seconds, double cn0 = 65.0, unsigned seed = 1) {
    sim::Common c;
    c.rate_hz = 32000.0;
    c.seconds = seconds;
    c.cn0_dbhz = cn0;
    c.seed = seed;
    return c;
}

nav::VorDecoder::Options vor_options() {
    nav::VorDecoder::Options o;
    o.noise_bandwidth_hz = 32000.0;  // generator noise is white over the whole band
    return o;
}

nav::IlsDecoder::Options ils_options(nav::IlsKind kind) {
    nav::IlsDecoder::Options o;
    o.kind = kind;
    o.noise_bandwidth_hz = 32000.0;
    return o;
}

}  // namespace

// --- frequencies -------------------------------------------------------------------

TEST(localizer_and_vor_frequencies) {
    CHECK(nav::is_localizer(110.30));
    CHECK(nav::is_localizer(108.15));
    CHECK(nav::is_localizer(111.95));
    CHECK(!nav::is_localizer(110.20));  // VOR
    CHECK(!nav::is_localizer(112.10));  // above the localizer band
    CHECK(!nav::is_localizer(108.00));
    CHECK(nav::is_nav_frequency(117.95));
    CHECK(!nav::is_nav_frequency(118.00));
    CHECK(!nav::is_nav_frequency(110.33));
    CHECK(nav::is_adf_frequency(339.5));
    CHECK(!nav::is_adf_frequency(150.0));
}

TEST(glideslope_pairing) {
    CHECK_NEAR(*nav::glideslope_for(108.10), 334.70, 1e-9);
    CHECK_NEAR(*nav::glideslope_for(110.30), 335.00, 1e-9);
    CHECK_NEAR(*nav::glideslope_for(111.95), 330.95, 1e-9);
    CHECK(!nav::glideslope_for(113.10).has_value());
    std::set<int> seen;
    int count = 0;
    for (int f = 10810; f <= 11195; f += 5) {
        const auto gs = nav::glideslope_for(f / 100.0);
        if (!gs) continue;
        ++count;
        CHECK(*gs >= 329.15 && *gs <= 335.00);
        seen.insert(nav::to_10khz(*gs));
    }
    CHECK(count == 40);
    CHECK(seen.size() == 40);  // every glide path channel used once
}

// --- VOR ---------------------------------------------------------------------------

TEST(vor_radials) {
    for (const double radial : {0.0, 45.0, 123.4, 181.0, 270.0, 359.5}) {
        sim::VorParams p;
        p.radial_deg = radial;
        nav::VorDecoder decoder(vor_options());
        decoder.process(sim::vor(common(2.0), p));
        CHECK(decoder.output().valid);
        CHECK_ANGLE(decoder.output().radial_deg, radial, 0.5);
        CHECK_NEAR(decoder.output().var_depth, 0.30, 0.02);
        CHECK_NEAR(decoder.output().ref_deviation_hz, 480.0, 25.0);
    }
}

TEST(vor_with_signal_frequency_error_and_noise) {
    sim::VorParams p;
    p.radial_deg = 77.0;
    p.signal_hz = 30.3;  // 1 % tolerance of the station
    nav::VorDecoder decoder(vor_options());
    decoder.process(sim::vor(common(3.0, 58.0, 7), p));
    CHECK(decoder.output().valid);
    CHECK_ANGLE(decoder.output().radial_deg, 77.0, 1.5);
}

TEST(vor_ident) {
    sim::VorParams p;
    p.radial_deg = 200.0;
    p.ident = "ABC";
    nav::VorDecoder decoder(vor_options());
    decoder.process(sim::vor(common(12.0), p));
    CHECK(decoder.output().valid);
    CHECK_ANGLE(decoder.output().radial_deg, 200.0, 0.5);
    CHECK(decoder.ident().ident() == "ABC");
}

TEST(vor_invalid_without_signal) {
    nav::VorDecoder decoder(vor_options());
    decoder.process(sim::noise_only(common(3.0)));
    CHECK(!decoder.output().valid);
    // an ILS is no VOR
    nav::VorDecoder decoder2(vor_options());
    sim::IlsParams ils;
    decoder2.process(sim::ils(common(3.0), ils));
    CHECK(!decoder2.output().valid);
}

// --- ILS ---------------------------------------------------------------------------

TEST(localizer_deviation_sign_and_scale) {
    // 150 Hz predominates right of the course: + deviation (OpenEFIS ADR 0007)
    struct Case { double ddm; double deg; };
    for (const Case k : {Case{0.0, 0.0}, Case{-0.0775, 1.25}, Case{0.0775, -1.25}, Case{-0.155, 2.5}}) {
        sim::IlsParams p;
        p.ddm = k.ddm;
        nav::IlsDecoder decoder(ils_options(nav::IlsKind::Localizer));
        decoder.process(sim::ils(common(1.0, 80.0), p));
        CHECK(decoder.output().valid);
        CHECK_NEAR(decoder.output().deviation_deg, k.deg, 0.01);
        CHECK_NEAR(decoder.output().sdm, 0.40, 0.01);
    }
}

TEST(glideslope_deviation_sign_and_scale) {
    // 90 Hz predominates above the glide path: + deviation
    for (const double ddm : {0.0875, -0.0875, 0.0}) {
        sim::IlsParams p;
        p.ddm = ddm;
        p.sdm = 0.80;
        nav::IlsDecoder decoder(ils_options(nav::IlsKind::Glideslope));
        decoder.process(sim::ils(common(1.0, 80.0), p));
        CHECK(decoder.output().valid);
        CHECK_NEAR(decoder.output().deviation_deg, ddm / 0.0875 * 0.35, 0.005);
    }
}

TEST(localizer_ident_and_invalid_cases) {
    sim::IlsParams p;
    p.ident = "IVW";
    nav::IlsDecoder decoder(ils_options(nav::IlsKind::Localizer));
    decoder.process(sim::ils(common(14.0), p));
    CHECK(decoder.output().valid);
    CHECK(decoder.ident().ident() == "IVW");

    nav::IlsDecoder no_signal(ils_options(nav::IlsKind::Localizer));
    no_signal.process(sim::noise_only(common(1.0)));
    CHECK(!no_signal.output().valid);

    sim::IlsParams gs_signal;  // a glide path (SDM 80 %) is not accepted as localizer
    gs_signal.sdm = 0.80;
    nav::IlsDecoder wrong(ils_options(nav::IlsKind::Localizer));
    wrong.process(sim::ils(common(1.0), gs_signal));
    CHECK(!wrong.output().valid);
}

// --- marker ------------------------------------------------------------------------

TEST(marker_beacons) {
    struct Case { sim::MarkerType type; unsigned bit; };
    for (const Case k : {Case{sim::MarkerType::Outer, nav::kOuter}, Case{sim::MarkerType::Middle, nav::kMiddle},
                         Case{sim::MarkerType::Inner, nav::kInner}}) {
        nav::MarkerDecoder::Options o;
        o.noise_bandwidth_hz = 32000.0;
        nav::MarkerDecoder decoder(o);
        const auto samples = sim::marker(common(3.0), k.type);
        int on = 0;
        int other = 0;
        int blocks = 0;
        for (std::size_t i = 0; i + 320 <= samples.size(); i += 320) {
            decoder.process(std::span<const dsp::cf>(samples.data() + i, 320));
            if (decoder.updates() < 20) continue;  // first C/N0 estimate after 0.1 s
            ++blocks;
            if (decoder.bits() & k.bit) ++on;
            if (decoder.bits() & ~k.bit) ++other;
        }
        CHECK(on > blocks / 4);      // keyed: on part of the time
        CHECK(on < blocks);          // ... and off in between (lamp flashes)
        CHECK(other == 0);
    }
    nav::MarkerDecoder::Options o;
    o.noise_bandwidth_hz = 32000.0;
    nav::MarkerDecoder quiet(o);
    quiet.process(sim::noise_only(common(1.0)));
    CHECK(quiet.bits() == 0);
}

// --- ADF ---------------------------------------------------------------------------

TEST(adf_bearings_with_switched_antenna) {
    for (const double bearing : {0.0, 30.0, 90.0, 135.0, 200.0, 271.0, 340.0}) {
        sim::NdbParams p;
        p.bearing_deg = bearing;
        p.clock_error_ppm = 80.0;
        nav::AdfDecoder::Options o;
        nav::AdfDecoder decoder(o);
        decoder.process(sim::ndb(common(3.0, 60.0), p));
        CHECK(decoder.output().bearing_valid);
        CHECK_ANGLE(decoder.output().bearing_deg, bearing, 2.0);
    }
}

TEST(adf_tracks_loop_phase_and_decodes_ident) {
    sim::NdbParams p;
    p.bearing_deg = 222.0;
    p.loop_phase_deg = 70.0;  // installation differs from the configured 90 deg
    p.ident = "OE";
    nav::AdfDecoder::Options o;
    nav::AdfDecoder decoder(o);
    decoder.process(sim::ndb(common(12.0, 60.0, 3), p));
    CHECK(decoder.output().bearing_valid);
    CHECK_ANGLE(decoder.output().bearing_deg, 222.0, 2.0);
    CHECK_NEAR(decoder.output().loop_phase_deg, 70.0, 3.0);
    CHECK(decoder.ident().ident() == "OE");
}

TEST(adf_without_switching_has_no_bearing) {
    sim::NdbParams p;
    p.switched_antenna = false;  // antenna unit failed or not installed
    p.ident = "WG";
    nav::AdfDecoder::Options o;
    nav::AdfDecoder switched(o);
    switched.process(sim::ndb(common(3.0, 60.0), p));
    CHECK(!switched.output().bearing_valid);

    o.antenna = nav::AdfAntenna::None;
    nav::AdfDecoder plain(o);
    const auto samples = sim::ndb(common(12.0, 60.0), p);
    std::string timeline;
    for (std::size_t i = 0; i + 320 <= samples.size(); i += 320) {
        plain.process(std::span<const dsp::cf>(samples.data() + i, 320));
        timeline.push_back(plain.ident().tone_on() ? '#' : '.');
    }
    CHECK(!plain.output().bearing_valid);
    CHECK(plain.ident().ident() == "WG");
    if (plain.ident().ident() != "WG") std::printf("  tone: %s
  depth %.3f
", timeline.c_str(), plain.ident().tone_depth());
}

// --- end to end: SDR stream with offset and 8-bit quantisation -------------------------

TEST(vor_end_to_end_through_cu8_file) {
    sim::Common c;
    c.rate_hz = 256000.0;
    c.seconds = 2.5;
    c.offset_hz = 60000.0;
    c.amplitude = 0.3;
    c.cn0_dbhz = 70.0;
    sim::VorParams p;
    p.radial_deg = 312.0;
    const auto bytes = sim::to_cu8(sim::vor(c, p));
    const char* path = "bodennav_test_vor.cu8";
    std::FILE* f = std::fopen(path, "wb");
    CHECK(f != nullptr);
    if (f == nullptr) return;
    std::fwrite(bytes.data(), 1, bytes.size(), f);
    std::fclose(f);

    io::FileSource source(path, io::FileFormat::CU8, c.rate_hz, false, false);
    CHECK(source.ok());
    dsp::Channelizer::Options co;
    dsp::Channelizer channel(co);
    auto vo = vor_options();
    vo.noise_bandwidth_hz = channel.noise_bandwidth();
    nav::VorDecoder decoder(vo);
    std::vector<dsp::cf> in(8192);
    std::vector<dsp::cf> base;
    while (const std::size_t n = source.read(in)) {
        base.clear();
        channel.process(std::span<const dsp::cf>(in.data(), n), base);
        decoder.process(base);
    }
    std::remove(path);
    CHECK(decoder.output().valid);
    CHECK_ANGLE(decoder.output().radial_deg, 312.0, 0.5);
}
