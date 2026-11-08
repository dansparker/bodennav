#include "nav/vor.hpp"

#include <cmath>

namespace bodennav::nav {

namespace {

constexpr double kSubcarrierHz = 9960.0;
constexpr double kSignalHz = 30.0;
constexpr double kBlockSeconds = 0.1;     // 3 periods of 30 Hz: DC and harmonics cancel exactly
constexpr std::size_t kFilterTaps = 255;  // same filter for both paths: equal group delay
constexpr double kFilterCutoffHz = 900.0;
constexpr std::size_t kWarmupBlocks = 2;         // filter start-up after reset

dsp::IdentDetector::Options ident_options(double rate_hz) {
    dsp::IdentDetector::Options o;
    o.rate_hz = rate_hz;
    o.tones_hz = {1020.0};
    return o;
}

}  // namespace

VorDecoder::VorDecoder(const Options& options)
    : options_(options),
      block_samples_(static_cast<std::size_t>(std::lround(kBlockSeconds * options.rate_hz))),
      low_rate_hz_(options.rate_hz / static_cast<double>(kDecimation)),
      subcarrier_(kSubcarrierHz, options.rate_hz),
      var_filter_(dsp::design_lowpass(kFilterTaps, kFilterCutoffHz, options.rate_hz), kDecimation),
      ref_filter_(dsp::design_lowpass(kFilterTaps, kFilterCutoffHz, options.rate_hz), kDecimation),
      var_tone_(kSignalHz, options.rate_hz / static_cast<double>(kDecimation)),
      ref_tone_(kSignalHz, options.rate_hz / static_cast<double>(kDecimation)),
      noise_(options.rate_hz, 12500.0, options.noise_bandwidth_hz),
      ident_(ident_options(options.rate_hz)) {
    block_.reserve(block_samples_ * 2);
}

void VorDecoder::reset() {
    block_.clear();
    subcarrier_.reset();
    var_filter_.reset();
    ref_filter_.reset();
    var_tone_.reset();
    ref_tone_.reset();
    previous_ = {1.0f, 0.0f};
    noise_.take_cn0_dbhz();
    ident_.reset();
    window_.clear();
    cn0_.clear();
    output_ = {};
    updates_ = 0;
}

void VorDecoder::process(std::span<const dsp::cf> baseband) {
    for (const auto z : baseband) {
        block_.push_back(z);
        if (block_.size() == block_samples_) {
            evaluate_block();
            block_.clear();
        }
    }
}

void VorDecoder::evaluate_block() {
    const std::size_t n = block_samples_;
    envelope_.resize(n);
    sub_in_.resize(n);
    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const float e = std::abs(block_[i]);
        envelope_[i] = e;
        sum += e;
        noise_.add(block_[i]);
    }
    const double carrier = sum / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double m = carrier > 0.0 ? envelope_[i] / carrier - 1.0 : 0.0;
        envelope_[i] = static_cast<float>(m);
        ident_.add(m);
        const dsp::cd r = std::conj(subcarrier_.next());
        sub_in_[i] = dsp::cf(static_cast<float>(m * r.real()), static_cast<float>(m * r.imag()));
    }

    var_out_.clear();
    ref_out_.clear();
    var_filter_.process(envelope_, var_out_);
    ref_filter_.process(sub_in_, ref_out_);
    for (const float v : var_out_) var_tone_.add(v);
    const double hz_per_rad = low_rate_hz_ / dsp::kTwoPi;
    for (const auto y : ref_out_) {
        const double d = std::arg(y * std::conj(previous_)) * hz_per_rad;  // instantaneous frequency offset [Hz]
        previous_ = y;
        ref_tone_.add(d);
    }
    const dsp::cd var = var_tone_.take();
    const dsp::cd ref = ref_tone_.take();
    const double cn0 = noise_.take_cn0_dbhz();

    ++updates_;
    if (updates_ <= kWarmupBlocks) return;
    window_.emplace_back(var, ref);
    cn0_.push_back(cn0);
    while (window_.size() > kWindowBlocks) window_.pop_front();
    while (cn0_.size() > kWindowBlocks) cn0_.pop_front();

    // The discriminator output belongs to the middle between two samples: half a sample late.
    const double discriminator_delay_deg = 360.0 * kSignalHz * 0.5 / low_rate_hz_;

    dsp::cd product{};
    dsp::cd unit_sum{};
    double var_depth = 0.0;
    double deviation = 0.0;
    for (const auto& [v, r] : window_) {
        const dsp::cd p = r * std::conj(v);  // arg = reference phase - variable phase = bearing
        product += p;
        if (std::abs(p) > 0.0) unit_sum += p / std::abs(p);
        var_depth += std::abs(v);
        deviation += std::abs(r);
    }
    const double count = static_cast<double>(window_.size());
    double cn0_mean = 0.0;
    for (const double c : cn0_) cn0_mean += std::isfinite(c) ? c : (c > 0 ? 200.0 : -200.0);
    cn0_mean /= static_cast<double>(cn0_.size());

    VorOutput out;
    out.var_depth = var_depth / count;
    out.ref_deviation_hz = deviation / count;
    out.cn0_dbhz = cn0_mean;
    const double resultant = std::abs(unit_sum) / count;
    out.spread_deg = resultant >= 1.0 ? 0.0
                     : resultant <= 0.0 ? 180.0
                                        : std::sqrt(-2.0 * std::log(resultant)) * dsp::kRadToDeg;
    out.radial_deg = dsp::wrap360(std::arg(product) * dsp::kRadToDeg + discriminator_delay_deg +
                                  options_.radial_offset_deg);
    out.valid = window_.size() == kWindowBlocks && cn0_mean >= options_.min_cn0_dbhz &&
                out.var_depth > 0.15 && out.var_depth < 0.45 && out.ref_deviation_hz > 300.0 &&
                out.ref_deviation_hz < 700.0 && out.spread_deg <= options_.max_spread_deg;
    output_ = out;
}

}  // namespace bodennav::nav
