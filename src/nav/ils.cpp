#include "nav/ils.hpp"

#include <cmath>

namespace bodennav::nav {

namespace {

constexpr double kBlockSeconds = 0.1;  // 9 periods of 90 Hz, 15 of 150 Hz, 102 of 1020 Hz

dsp::IdentDetector::Options ident_options(double rate_hz) {
    dsp::IdentDetector::Options o;
    o.rate_hz = rate_hz;
    o.tones_hz = {1020.0};
    return o;
}

}  // namespace

IlsDecoder::IlsDecoder(const Options& options)
    : options_(options),
      block_samples_(static_cast<std::size_t>(std::lround(kBlockSeconds * options.rate_hz))),
      tone90_(90.0, options.rate_hz),
      tone150_(150.0, options.rate_hz),
      noise_(options.rate_hz, 12500.0, options.noise_bandwidth_hz),
      ident_(ident_options(options.rate_hz)) {
    if (options_.deg_per_ddm <= 0.0) {
        options_.deg_per_ddm = options_.kind == IlsKind::Localizer ? kLocalizerDegPerDdm : kGlideslopeDegPerDdm;
    }
    if (options_.window_blocks < 1) options_.window_blocks = 1;
    block_.reserve(block_samples_);
}

void IlsDecoder::reset() {
    block_.clear();
    tone90_.reset();
    tone150_.reset();
    (void)noise_.take_cn0_dbhz();  // drop the partial measurement
    ident_.reset();
    window_.clear();
    cn0_.clear();
    output_ = {};
    updates_ = 0;
}

void IlsDecoder::process(std::span<const dsp::cf> baseband) {
    for (const auto z : baseband) {
        block_.push_back(z);
        if (block_.size() == block_samples_) {
            evaluate_block();
            block_.clear();
        }
    }
}

void IlsDecoder::evaluate_block() {
    double sum = 0.0;
    for (const auto z : block_) {
        sum += std::abs(z);
        noise_.add(z);
    }
    const double carrier = sum / static_cast<double>(block_.size());
    for (const auto z : block_) {
        const double m = carrier > 0.0 ? std::abs(z) / carrier - 1.0 : 0.0;
        tone90_.add(m);
        tone150_.add(m);
        if (options_.kind == IlsKind::Localizer) ident_.add(m);
    }
    const double d90 = std::abs(tone90_.take());
    const double d150 = std::abs(tone150_.take());
    const double cn0 = noise_.take_cn0_dbhz();
    ++updates_;

    window_.emplace_back(d90, d150);
    cn0_.push_back(std::isfinite(cn0) ? cn0 : (cn0 > 0 ? 200.0 : -200.0));
    const auto max_size = static_cast<std::size_t>(options_.window_blocks);
    while (window_.size() > max_size) window_.pop_front();
    while (cn0_.size() > max_size) cn0_.pop_front();

    double m90 = 0.0;
    double m150 = 0.0;
    for (const auto& [a, b] : window_) {
        m90 += a;
        m150 += b;
    }
    double cn0_mean = 0.0;
    for (const double c : cn0_) cn0_mean += c;
    const double count = static_cast<double>(window_.size());
    m90 /= count;
    m150 /= count;
    cn0_mean /= count;

    IlsOutput out;
    out.ddm = m90 - m150;
    out.sdm = m90 + m150;
    out.cn0_dbhz = cn0_mean;
    out.deviation_deg = (options_.kind == IlsKind::Localizer ? -out.ddm : out.ddm) * options_.deg_per_ddm;
    const bool sdm_ok = options_.kind == IlsKind::Localizer ? (out.sdm > 0.30 && out.sdm < 0.50)
                                                            : (out.sdm > 0.60 && out.sdm < 0.95);
    out.valid = window_.size() == max_size && cn0_mean >= options_.min_cn0_dbhz && sdm_ok;
    output_ = out;
}

}  // namespace bodennav::nav
