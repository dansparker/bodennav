#pragma once

// Conventional VOR (ICAO Annex 10, 3.3): the carrier is amplitude modulated with
// the 30 Hz variable signal (30 %) and with a 9960 Hz subcarrier (30 %) that is
// frequency modulated with the 30 Hz reference (+/-480 Hz). The variable signal
// lags the reference by the magnetic bearing of the aircraft from the station.

#include "dsp/dsp.hpp"
#include "dsp/morse.hpp"

#include <deque>

namespace bodennav::nav {

struct VorOutput {
    bool valid{false};
    double radial_deg{0.0};        // magnetic, FROM the station
    double var_depth{0.0};         // 30 Hz AM depth (nominal 0.30)
    double ref_deviation_hz{0.0};  // 30 Hz FM deviation of the subcarrier (nominal 480 Hz)
    double spread_deg{0.0};        // scatter of the 0.1 s bearings within the window
    double cn0_dbhz{0.0};          // carrier-to-noise density
};

class VorDecoder {
public:
    struct Options {
        double rate_hz{32000.0};           // channel rate (Channelizer output)
        double noise_bandwidth_hz{26000.0};
        double radial_offset_deg{0.0};     // calibration (receiver / installation)
        double min_cn0_dbhz{50.0};
        double max_spread_deg{6.0};
    };

    explicit VorDecoder(const Options& options);
    // Complex baseband, channel at 0 Hz.
    void process(std::span<const dsp::cf> baseband);
    [[nodiscard]] const VorOutput& output() const { return output_; }
    [[nodiscard]] const dsp::IdentDetector& ident() const { return ident_; }
    // Number of evaluated 0.1 s blocks since the start (a new output each block).
    [[nodiscard]] std::size_t updates() const { return updates_; }
    void reset();

private:
    void evaluate_block();

    static constexpr std::size_t kDecimation = 16;  // 32 kHz -> 2 kHz for the 30 Hz signals
    static constexpr std::size_t kWindowBlocks = 10;  // 1 s window, new value every 0.1 s

    Options options_;
    std::size_t block_samples_;
    double low_rate_hz_;
    std::vector<float> envelope_;
    std::vector<dsp::cf> block_;
    dsp::Rotator subcarrier_;
    dsp::FirDecimator<float> var_filter_;
    dsp::FirDecimator<dsp::cf> ref_filter_;
    dsp::Tone var_tone_;
    dsp::Tone ref_tone_;
    dsp::cf previous_{1.0f, 0.0f};
    dsp::NoiseProbe noise_;
    dsp::IdentDetector ident_;
    std::deque<std::pair<dsp::cd, dsp::cd>> window_;  // (variable, reference) per block
    std::deque<double> cn0_;
    std::vector<float> var_out_;
    std::vector<dsp::cf> sub_in_;
    std::vector<dsp::cf> ref_out_;
    VorOutput output_;
    std::size_t updates_{0};
};

}  // namespace bodennav::nav
