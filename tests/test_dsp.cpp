#include "dsp/dsp.hpp"
#include "dsp/morse.hpp"
#include "sim/siggen.hpp"
#include "unit.hpp"

using namespace bodennav;

TEST(lowpass_has_unity_dc_gain) {
    const auto taps = dsp::design_lowpass(129, 16000.0, 256000.0);
    double sum = 0.0;
    for (const float t : taps) sum += t;
    CHECK_NEAR(sum, 1.0, 1e-5);
    CHECK(taps.size() == 129);
    CHECK_NEAR(taps.front(), taps.back(), 1e-9);  // symmetric = linear phase
}

TEST(decimator_output_count_and_level) {
    dsp::FirDecimator<float> fir(dsp::design_lowpass(31, 1000.0, 32000.0), 4);
    std::vector<float> in(1000, 1.0f);
    std::vector<float> out;
    fir.process(std::span<const float>(in.data(), 500), out);
    fir.process(std::span<const float>(in.data() + 500, 500), out);
    CHECK(out.size() == (1000 - 31) / 4 + 1);
    CHECK_NEAR(out.back(), 1.0, 1e-5);
}

TEST(tone_amplitude_and_phase) {
    dsp::Tone tone(30.0, 2000.0);
    for (int i = 0; i < 2000; ++i) tone.add(0.5 * std::cos(dsp::kTwoPi * 30.0 * i / 2000.0 + 0.7));
    const auto a = tone.take();
    CHECK_NEAR(std::abs(a), 0.5, 1e-6);
    CHECK_NEAR(std::arg(a), 0.7, 1e-6);
}

TEST(channelizer_moves_channel_to_zero) {
    dsp::Channelizer::Options o;
    dsp::Channelizer ch(o);
    std::vector<dsp::cf> in(25600);
    for (std::size_t i = 0; i < in.size(); ++i) {
        const double ph = dsp::kTwoPi * (o.offset_hz + 1000.0) * static_cast<double>(i) / o.input_rate_hz;
        in[i] = dsp::cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
    }
    std::vector<dsp::cf> out;
    ch.process(in, out);
    CHECK_NEAR(ch.output_rate(), 32000.0, 1e-9);
    CHECK(out.size() > 3000);
    // a tone at +1000 Hz: constant magnitude, phase advancing 2 pi 1000 / 32000 per sample
    const auto z1 = out[2000];
    const auto z2 = out[2001];
    CHECK_NEAR(std::abs(z1), 1.0, 0.01);
    CHECK_NEAR(std::arg(z2 * std::conj(z1)), dsp::kTwoPi * 1000.0 / 32000.0, 1e-3);
    // a signal 40 kHz away is rejected
    dsp::Channelizer ch2(o);
    for (std::size_t i = 0; i < in.size(); ++i) {
        const double ph = dsp::kTwoPi * (o.offset_hz + 40000.0) * static_cast<double>(i) / o.input_rate_hz;
        in[i] = dsp::cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
    }
    out.clear();
    ch2.process(in, out);
    CHECK(std::abs(out[2000]) < 1e-3);
}

TEST(noise_probe_measures_cn0) {
    sim::Common c;
    c.seconds = 2.0;
    c.cn0_dbhz = 60.0;
    sim::IlsParams carrier_only;
    carrier_only.sdm = 0.0;
    const auto samples = sim::ils(c, carrier_only);
    dsp::NoiseProbe probe(c.rate_hz, 12500.0, c.rate_hz);  // white noise over the whole band
    for (const auto z : samples) probe.add(z);
    CHECK_NEAR(probe.take_cn0_dbhz(), 60.0, 1.0);
}

TEST(morse_decoder_confirms_repeated_ident) {
    const sim::MorseKey key("VIE", 0.1, 3.0);
    dsp::MorseDecoder decoder(0.01);
    bool confirmed = false;
    for (int i = 0; i < 1500; ++i) confirmed |= decoder.feed(key.on(i * 0.01));
    CHECK(confirmed);
    CHECK(decoder.ident() == "VIE");
    CHECK(decoder.matches() >= 1);
}

TEST(morse_table_round_trip) {
    for (const char c : std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")) {
        CHECK(dsp::morse_to_char(dsp::char_to_morse(c)) == c);
    }
    CHECK(dsp::morse_to_char("........") == 0);
}

TEST(wrap_angles) {
    CHECK_NEAR(dsp::wrap360(-10.0), 350.0, 1e-9);
    CHECK_NEAR(dsp::wrap360(720.5), 0.5, 1e-9);
    CHECK_NEAR(dsp::wrap180(190.0), -170.0, 1e-9);
}
