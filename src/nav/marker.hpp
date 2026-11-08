#pragma once

// Marker beacons on 75 MHz (ICAO Annex 10, 3.1.7): 95 % AM with a keyed tone,
// outer 400 Hz (dashes), middle 1300 Hz (dots and dashes), inner 3000 Hz (dots).
// The output follows the keying, so the lamp on the display flashes like on a
// conventional marker receiver.

#include "dsp/dsp.hpp"

#include <array>

namespace bodennav::nav {

enum MarkerBit : unsigned { kOuter = 1u << 0, kMiddle = 1u << 1, kInner = 1u << 2 };

class MarkerDecoder {
public:
    struct Options {
        double rate_hz{32000.0};
        double noise_bandwidth_hz{26000.0};
        double min_cn0_dbhz{45.0};  // sensitivity: higher value = LO sensitivity
        double on_depth{0.40};
        double off_depth{0.20};
    };

    explicit MarkerDecoder(const Options& options);
    void process(std::span<const dsp::cf> baseband);
    // Current keyed state (MarkerBit bits).
    [[nodiscard]] unsigned bits() const { return bits_; }
    [[nodiscard]] double cn0_dbhz() const { return cn0_dbhz_; }
    [[nodiscard]] std::size_t updates() const { return updates_; }
    void reset();

private:
    void evaluate_block();

    Options options_;
    std::size_t block_samples_;
    std::vector<dsp::cf> block_;
    std::array<dsp::Tone, 3> tones_;
    dsp::NoiseProbe noise_;
    unsigned bits_{0};
    double cn0_dbhz_{0.0};
    std::size_t updates_{0};
};

}  // namespace bodennav::nav
