#include "nav/marker.hpp"

#include <cmath>

namespace bodennav::nav {

namespace {
constexpr double kBlockSeconds = 0.01;  // 4 / 13 / 30 periods of the three tones
constexpr std::array<double, 3> kTonesHz{400.0, 1300.0, 3000.0};
}  // namespace

MarkerDecoder::MarkerDecoder(const Options& options)
    : options_(options),
      block_samples_(static_cast<std::size_t>(std::lround(kBlockSeconds * options.rate_hz))),
      tones_{dsp::Tone(kTonesHz[0], options.rate_hz), dsp::Tone(kTonesHz[1], options.rate_hz),
             dsp::Tone(kTonesHz[2], options.rate_hz)},
      // the noise is measured over 0.1 s (more stable than per 10 ms block)
      noise_(options.rate_hz, 12500.0, options.noise_bandwidth_hz) {
    block_.reserve(block_samples_);
}

void MarkerDecoder::reset() {
    block_.clear();
    for (auto& t : tones_) t.reset();
    noise_.take_cn0_dbhz();
    bits_ = 0;
    cn0_dbhz_ = 0.0;
    updates_ = 0;
}

void MarkerDecoder::process(std::span<const dsp::cf> baseband) {
    for (const auto z : baseband) {
        block_.push_back(z);
        if (block_.size() == block_samples_) {
            evaluate_block();
            block_.clear();
        }
    }
}

void MarkerDecoder::evaluate_block() {
    double sum = 0.0;
    for (const auto z : block_) {
        sum += std::abs(z);
        noise_.add(z);
    }
    const double carrier = sum / static_cast<double>(block_.size());
    for (const auto z : block_) {
        const double m = carrier > 0.0 ? std::abs(z) / carrier - 1.0 : 0.0;
        for (auto& t : tones_) t.add(m);
    }
    ++updates_;
    if (updates_ % 10 == 0) {
        const double cn0 = noise_.take_cn0_dbhz();
        cn0_dbhz_ = std::isfinite(cn0) ? cn0 : (cn0 > 0 ? 200.0 : -200.0);
    }
    const bool carrier_ok = cn0_dbhz_ >= options_.min_cn0_dbhz;
    unsigned bits = 0;
    for (std::size_t i = 0; i < tones_.size(); ++i) {
        const double depth = std::abs(tones_[i].take());
        const unsigned bit = 1u << i;
        const bool was_on = (bits_ & bit) != 0;
        const bool on = carrier_ok && (was_on ? depth > options_.off_depth : depth > options_.on_depth);
        if (on) bits |= bit;
    }
    bits_ = bits;
}

}  // namespace bodennav::nav
