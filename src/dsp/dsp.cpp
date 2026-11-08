#include "dsp/dsp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace bodennav::dsp {

std::vector<float> design_lowpass(std::size_t taps, double cutoff_hz, double rate_hz) {
    if (taps < 1) taps = 1;
    std::vector<double> h(taps);
    const double fc = cutoff_hz / rate_hz;  // cycles per sample
    const double centre = 0.5 * static_cast<double>(taps - 1);
    double sum = 0.0;
    for (std::size_t i = 0; i < taps; ++i) {
        const double t = static_cast<double>(i) - centre;
        const double sinc = (t == 0.0) ? 2.0 * fc : std::sin(kTwoPi * fc * t) / (kPi * t);
        const double x = taps > 1 ? static_cast<double>(i) / static_cast<double>(taps - 1) : 0.5;
        const double window = 0.42 - 0.5 * std::cos(kTwoPi * x) + 0.08 * std::cos(2.0 * kTwoPi * x);
        h[i] = sinc * window;
        sum += h[i];
    }
    std::vector<float> out(taps);
    for (std::size_t i = 0; i < taps; ++i) out[i] = static_cast<float>(h[i] / sum);
    return out;
}

double noise_bandwidth(std::span<const float> taps, double rate_hz) {
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const float t : taps) {
        sum += t;
        sum_sq += static_cast<double>(t) * t;
    }
    return sum == 0.0 ? 0.0 : rate_hz * sum_sq / (sum * sum);
}

double wrap360(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    if (r >= 360.0) r -= 360.0;
    return r;
}

double wrap180(double deg) {
    double r = wrap360(deg + 180.0) - 180.0;
    return r;
}

void Rotator::set(double freq_hz, double rate_hz) {
    step_ = std::polar(1.0, kTwoPi * freq_hz / rate_hz);
    phasor_ = {1.0, 0.0};
    count_ = 0;
}

// --- FirDecimator ---------------------------------------------------------------

template <typename T>
FirDecimator<T>::FirDecimator(std::vector<float> coefficients, std::size_t factor)
    : taps_(std::move(coefficients)), factor_(std::max<std::size_t>(1, factor)) {
    if (taps_.empty()) taps_.push_back(1.0f);
    std::reverse(taps_.begin(), taps_.end());
    reset();
}

template <typename T>
void FirDecimator<T>::reset() {
    buffer_.assign(taps_.size() - 1, T{});
    next_ = taps_.size() - 1;
}

template <typename T>
void FirDecimator<T>::process(std::span<const T> in, std::vector<T>& out) {
    const std::size_t n = taps_.size();
    buffer_.insert(buffer_.end(), in.begin(), in.end());
    std::size_t pos = next_;
    while (pos < buffer_.size()) {
        const T* x = buffer_.data() + (pos + 1 - n);
        T acc{};
        for (std::size_t k = 0; k < n; ++k) acc += x[k] * taps_[k];
        out.push_back(acc);
        pos += factor_;
    }
    const std::size_t keep = n - 1;
    const std::size_t drop = buffer_.size() - keep;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(drop));
    next_ = pos - drop;
}

template class FirDecimator<float>;
template class FirDecimator<cf>;

// --- Tone -----------------------------------------------------------------------

cd Tone::take() {
    const cd amplitude = count_ > 0 ? sum_ * (2.0 / static_cast<double>(count_)) : cd{};
    sum_ = {};
    count_ = 0;
    return amplitude;
}

// --- Channelizer ----------------------------------------------------------------

Channelizer::Channelizer(const Options& options)
    : options_(options),
      mixer_(-options.offset_hz, options.input_rate_hz),
      filter_(design_lowpass(options.taps, options.cutoff_hz, options.input_rate_hz), options.decimation) {
    if (options_.narrow_cutoff_hz > 0.0) {
        narrow_ = FirDecimator<cf>(design_lowpass(options_.narrow_taps, options_.narrow_cutoff_hz, output_rate()), 1);
    }
}

void Channelizer::process(std::span<const cf> in, std::vector<cf>& out) {
    mixed_.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        const cd m = mixer_.next();
        mixed_[i] = in[i] * cf(static_cast<float>(m.real()), static_cast<float>(m.imag()));
    }
    if (options_.narrow_cutoff_hz > 0.0) {
        wide_.clear();
        filter_.process(mixed_, wide_);
        narrow_.process(wide_, out);
    } else {
        filter_.process(mixed_, out);
    }
}

void Channelizer::reset() {
    mixer_.reset();
    filter_.reset();
    if (options_.narrow_cutoff_hz > 0.0) narrow_.reset();
}

double Channelizer::noise_bandwidth() const {
    if (options_.narrow_cutoff_hz > 0.0) return dsp::noise_bandwidth(narrow_.taps(), output_rate());
    return dsp::noise_bandwidth(filter_.taps(), options_.input_rate_hz);
}

// --- NoiseProbe -----------------------------------------------------------------

NoiseProbe::NoiseProbe(double rate_hz, double probe_hz, double noise_bandwidth_hz)
    : rate_hz_(rate_hz), noise_bandwidth_hz_(noise_bandwidth_hz), window_(kLength), upper_(probe_hz, rate_hz), lower_(-probe_hz, rate_hz) {
    for (std::size_t i = 0; i < kLength; ++i) {
        window_[i] = 0.5 - 0.5 * std::cos(kTwoPi * (static_cast<double>(i) + 0.5) / static_cast<double>(kLength));
        window_power_ += window_[i] * window_[i];
    }
}

void NoiseProbe::add(cf z) {
    const cd zd(z.real(), z.imag());
    power_sum_ += std::norm(zd);
    ++power_count_;
    upper_sum_ += window_[index_] * zd * std::conj(upper_.next());
    lower_sum_ += window_[index_] * zd * std::conj(lower_.next());
    if (++index_ == kLength) {
        noise_sum_ += 0.5 * (std::norm(upper_sum_) + std::norm(lower_sum_)) / window_power_;
        ++noise_count_;
        upper_sum_ = {};
        lower_sum_ = {};
        index_ = 0;
    }
}

double NoiseProbe::take_cn0_dbhz() {
    double result = -std::numeric_limits<double>::infinity();
    if (noise_count_ > 0 && power_count_ > 0) {
        const double power = power_sum_ / static_cast<double>(power_count_);
        const double noise_per_sample = noise_sum_ / static_cast<double>(noise_count_);  // white noise variance
        const double n0 = noise_per_sample / rate_hz_;                                  // per Hz
        const double carrier = power - n0 * noise_bandwidth_hz_;
        if (carrier > 0.0) {
            result = n0 > 0.0 ? 10.0 * std::log10(carrier / n0) : std::numeric_limits<double>::infinity();
        }
    }
    noise_sum_ = 0.0;
    noise_count_ = 0;
    power_sum_ = 0.0;
    power_count_ = 0;
    return result;
}

}  // namespace bodennav::dsp
