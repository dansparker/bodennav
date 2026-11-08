#pragma once

// ILS localizer and glide path (ICAO Annex 10, 3.1): the carrier is amplitude
// modulated with 90 Hz and 150 Hz. The difference in depth of modulation (DDM)
// gives the deviation from the course / glide path:
//   localizer:  150 Hz predominates right of the course (seen on the approach),
//   glide path:  90 Hz predominates above the glide path.
// Nominal sum of depths (SDM): localizer 40 %, glide path 80 %.

#include "dsp/dsp.hpp"
#include "dsp/morse.hpp"

#include <deque>

namespace bodennav::nav {

enum class IlsKind { Localizer, Glideslope };

// Conversion DDM -> deviation angle in the units of OpenEFIS (ADR 0007), so that
// one dot on the display equals one standard dot of a certified indicator:
//   localizer: 1 dot = 0.0775 DDM (75 uA), OpenEFIS 1 dot = 1.25 deg
//   glide path: 1 dot = 0.0875 DDM (75 uA), OpenEFIS 1 dot = 0.35 deg
inline constexpr double kLocalizerDegPerDdm = 1.25 / 0.0775;
inline constexpr double kGlideslopeDegPerDdm = 0.35 / 0.0875;

struct IlsOutput {
    bool valid{false};
    double ddm{0.0};            // depth 90 Hz - depth 150 Hz
    double sdm{0.0};            // depth 90 Hz + depth 150 Hz
    double deviation_deg{0.0};  // localizer: + aircraft right of the course; glide path: + above
    double cn0_dbhz{0.0};
};

class IlsDecoder {
public:
    struct Options {
        IlsKind kind{IlsKind::Localizer};
        double rate_hz{32000.0};
        double noise_bandwidth_hz{26000.0};
        double min_cn0_dbhz{50.0};
        double deg_per_ddm{0.0};  // 0 = standard value for the kind
        int window_blocks{2};     // averaging over 2 x 0.1 s, new value every 0.1 s
    };

    explicit IlsDecoder(const Options& options);
    void process(std::span<const dsp::cf> baseband);
    [[nodiscard]] const IlsOutput& output() const { return output_; }
    [[nodiscard]] const dsp::IdentDetector& ident() const { return ident_; }
    [[nodiscard]] std::size_t updates() const { return updates_; }
    void reset();

private:
    void evaluate_block();

    Options options_;
    std::size_t block_samples_;
    std::vector<dsp::cf> block_;
    dsp::Tone tone90_;
    dsp::Tone tone150_;
    dsp::NoiseProbe noise_;
    dsp::IdentDetector ident_;
    std::deque<std::pair<double, double>> window_;  // depths (90, 150)
    std::deque<double> cn0_;
    IlsOutput output_;
    std::size_t updates_{0};
};

}  // namespace bodennav::nav
