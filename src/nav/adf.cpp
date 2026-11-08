#include "nav/adf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace bodennav::nav {

namespace {

constexpr std::size_t kSlots = 5;

dsp::IdentDetector::Options ident_options(const AdfDecoder::Options& o) {
    dsp::IdentDetector::Options d;
    d.rate_hz = o.rate_hz;
    d.tones_hz = o.ident_tones_hz;
    // reference bins between and above the two possible ident tones
    d.reference_hz = {700.0, 1500.0};
    return d;
}

}  // namespace

AdfDecoder::AdfDecoder(const Options& options)
    : options_(options),
      slot_(std::max<std::size_t>(8, static_cast<std::size_t>(std::lround(options.slot_s * options.rate_hz)))),
      guard_(static_cast<std::size_t>(std::lround(options.guard_s * options.rate_hz))),
      frame_(kSlots * slot_),
      chunk_(static_cast<std::size_t>(std::max(1, options.frames_per_chunk)) * frame_),
      fold_(frame_),
      ident_(ident_options(options)) {
    if (2 * guard_ + 2 > slot_) guard_ = (slot_ - 2) / 2;
    if (options_.window_chunks < 1) options_.window_chunks = 1;
}

void AdfDecoder::reset() {
    buffer_.clear();
    window_.clear();
    ident_.reset();
    output_ = {};
    updates_ = 0;
    plain_block_.clear();
}

void AdfDecoder::process(std::span<const dsp::cf> baseband) {
    if (options_.antenna == AdfAntenna::None) {
        process_plain(baseband);
        return;
    }
    buffer_.insert(buffer_.end(), baseband.begin(), baseband.end());
    while (buffer_.size() >= chunk_ + frame_) process_chunk();
}

// Without a switched antenna only the identifier is decoded (no bearing).
void AdfDecoder::process_plain(std::span<const dsp::cf> baseband) {
    const std::size_t block = slot_;
    for (const auto z : baseband) {
        plain_block_.push_back(z);
        if (plain_block_.size() < block) continue;
        double sum = 0.0;
        for (const auto w : plain_block_) sum += std::abs(w);
        const double carrier = sum / static_cast<double>(block);
        for (const auto w : plain_block_) ident_.add(carrier > 0.0 ? std::abs(w) / carrier - 1.0 : 0.0);
        plain_block_.clear();
    }
    output_.carrier = !ident_.ident().empty();
    output_.bearing_valid = false;
}

void AdfDecoder::process_chunk() {
    const std::size_t frames = chunk_ / frame_;

    // 1. Find the sync slot: fold the power over the frame period, take the quietest slot-long window.
    std::fill(fold_.begin(), fold_.end(), 0.0);
    for (std::size_t i = 0; i < chunk_; ++i) fold_[i % frame_] += std::norm(dsp::cd(buffer_[i].real(), buffer_[i].imag()));
    double window_sum = 0.0;
    for (std::size_t j = 0; j < slot_; ++j) window_sum += fold_[j];
    double best = window_sum;
    std::size_t offset = 0;
    for (std::size_t k = 1; k < frame_; ++k) {
        window_sum += fold_[(k + slot_ - 1) % frame_] - fold_[k - 1];
        if (window_sum < best) {
            best = window_sum;
            offset = k;
        }
    }

    // 2. Samples before the first sync slot: only time for the ident detector.
    for (std::size_t i = 0; i < offset; ++i) ident_.add(0.0, false);

    // 3. Per frame: mean of every slot interior.
    ChunkSums sums;
    for (std::size_t f = 0; f < frames; ++f) {
        const std::size_t start = offset + f * frame_;
        std::array<dsp::cd, kSlots> mean{};
        std::array<double, kSlots> power{};
        std::array<double, kSlots> magnitude{};
        const std::size_t used = slot_ - 2 * guard_;
        for (std::size_t s = 0; s < kSlots; ++s) {
            const std::size_t first = start + s * slot_ + guard_;
            for (std::size_t i = first; i < first + used; ++i) {
                const dsp::cd z(buffer_[i].real(), buffer_[i].imag());
                mean[s] += z;
                power[s] += std::norm(z);
                magnitude[s] += std::abs(z);
            }
            mean[s] /= static_cast<double>(used);
            power[s] /= static_cast<double>(used);
            magnitude[s] /= static_cast<double>(used);
        }
        const dsp::cd sense = 0.25 * (mean[1] + mean[2] + mean[3] + mean[4]);
        const dsp::cd loop_a = 0.5 * (mean[1] - mean[2]);
        const dsp::cd loop_b = 0.5 * (mean[3] - mean[4]);
        sums.u += loop_a * std::conj(sense);
        sums.v += loop_b * std::conj(sense);
        sums.sense += std::norm(sense);
        sums.noise += power[0];
        sums.data += 0.25 * (power[1] + power[2] + power[3] + power[4]);
        ++sums.frames;

        // identifier: AM of each data slot relative to its own carrier (switching steps cancel)
        for (std::size_t s = 0; s < kSlots; ++s) {
            const std::size_t slot_start = start + s * slot_;
            for (std::size_t i = 0; i < slot_; ++i) {
                const bool interior = s > 0 && i >= guard_ && i < slot_ - guard_ && magnitude[s] > 0.0;
                if (interior) ident_.add(std::abs(buffer_[slot_start + i]) / magnitude[s] - 1.0, true);
                else ident_.add(0.0, false);
            }
        }
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset + frames * frame_));

    window_.push_back(sums);
    while (window_.size() > static_cast<std::size_t>(options_.window_chunks)) window_.pop_front();
    evaluate();
}

void AdfDecoder::evaluate() {
    ChunkSums total;
    for (const auto& c : window_) {
        total.u += c.u;
        total.v += c.v;
        total.sense += c.sense;
        total.noise += c.noise;
        total.data += c.data;
        total.frames += c.frames;
    }
    ++updates_;
    AdfOutput out;
    if (total.frames == 0) {
        output_ = out;
        return;
    }
    const double frames = static_cast<double>(total.frames);
    const double sense = total.sense / frames;
    const double noise = total.noise / frames;
    const double data = total.data / frames;
    out.snr_db = noise > 0.0 ? 10.0 * std::log10(std::max(sense, 1e-30) / noise)
                             : std::numeric_limits<double>::infinity();
    out.sync_contrast = data > 0.0 ? noise / data : 1.0;
    out.carrier = out.snr_db >= options_.min_snr_db && out.sync_contrast <= options_.max_sync_contrast;

    // loop phase: u and v share the phase psi -> u^2 + v^2 has the phase 2 psi
    const double configured = options_.loop_phase_deg * dsp::kDegToRad;
    double psi = configured;
    bool psi_ok = true;
    if (options_.track_loop_phase) {
        const dsp::cd w = total.u * total.u + total.v * total.v;
        if (std::abs(w) > 0.0) {
            double estimate = 0.5 * std::arg(w);
            if (std::abs(dsp::wrap180((estimate - configured) * dsp::kRadToDeg)) > 90.0) estimate += dsp::kPi;
            if (std::abs(dsp::wrap180((estimate - configured) * dsp::kRadToDeg)) <= 45.0) psi = estimate;
            else psi_ok = false;
        }
    }
    out.loop_phase_deg = dsp::wrap180(psi * dsp::kRadToDeg);
    const dsp::cd rotate = std::polar(1.0, -psi);
    const dsp::cd a = total.u * rotate;
    const dsp::cd b = total.v * rotate;
    const double in_phase = std::hypot(a.real(), b.real());
    const double orthogonal = std::hypot(a.imag(), b.imag());
    out.loop_ratio = total.sense > 0.0 ? std::hypot(in_phase, orthogonal) / total.sense : 0.0;
    out.quadrature = in_phase > 0.0 ? orthogonal / in_phase : std::numeric_limits<double>::infinity();
    const double fore_aft = options_.invert_fore_aft ? -a.real() : a.real();
    const double lateral = options_.invert_lateral ? -b.real() : b.real();
    out.bearing_deg = dsp::wrap360(std::atan2(lateral, fore_aft) * dsp::kRadToDeg + options_.bearing_offset_deg);
    out.bearing_valid = out.carrier && psi_ok && out.loop_ratio >= options_.min_loop_ratio &&
                        out.quadrature <= options_.max_quadrature &&
                        window_.size() == static_cast<std::size_t>(options_.window_chunks);
    output_ = out;
}

}  // namespace bodennav::nav
