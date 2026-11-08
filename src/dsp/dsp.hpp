#pragma once

// Basic signal processing blocks: oscillator, FIR design and decimation, tone
// correlation, channel selection and a noise probe. Everything is plain C++20
// without dependencies so it runs on the host, in the CI and on a Raspberry Pi.

#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

namespace bodennav::dsp {

using cf = std::complex<float>;
using cd = std::complex<double>;

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kRadToDeg = 180.0 / std::numbers::pi;
inline constexpr double kDegToRad = std::numbers::pi / 180.0;

// Windowed-sinc low-pass with a Blackman window and unity gain at 0 Hz.
// `cutoff_hz` is the -6 dB point; the transition band is about 5.5 * rate / taps wide.
[[nodiscard]] std::vector<float> design_lowpass(std::size_t taps, double cutoff_hz, double rate_hz);

// Noise-equivalent bandwidth [Hz] of a FIR filter with unity DC gain.
[[nodiscard]] double noise_bandwidth(std::span<const float> taps, double rate_hz);

[[nodiscard]] double wrap360(double deg);  // [0, 360)
[[nodiscard]] double wrap180(double deg);  // [-180, 180)

// Complex oscillator exp(j 2 pi f n / fs). The phase is continuous over all
// calls, so correlations over consecutive blocks stay coherent.
class Rotator {
public:
    Rotator() = default;
    Rotator(double freq_hz, double rate_hz) { set(freq_hz, rate_hz); }
    void set(double freq_hz, double rate_hz);
    // Returns the current phasor and advances by one sample.
    cd next() {
        const cd value = phasor_;
        phasor_ *= step_;
        if (++count_ == 512) {  // keep |phasor| = 1 despite rounding
            phasor_ /= std::abs(phasor_);
            count_ = 0;
        }
        return value;
    }
    void reset() { phasor_ = {1.0, 0.0}; count_ = 0; }

private:
    cd phasor_{1.0, 0.0};
    cd step_{1.0, 0.0};
    int count_{0};
};

// FIR filter with integer decimation (only the kept outputs are computed).
template <typename T>
class FirDecimator {
public:
    FirDecimator() = default;
    FirDecimator(std::vector<float> coefficients, std::size_t factor);
    // Appends the outputs for `in` to `out`.
    void process(std::span<const T> in, std::vector<T>& out);
    void reset();
    [[nodiscard]] std::size_t factor() const { return factor_; }
    [[nodiscard]] std::size_t length() const { return taps_.size(); }
    [[nodiscard]] const std::vector<float>& taps() const { return taps_; }

private:
    std::vector<float> taps_;     // time reversed: taps_[k] weights the k-th oldest sample of the window
    std::vector<T> buffer_;       // the last length()-1 samples, then the new input
    std::size_t factor_{1};
    std::size_t next_{0};         // index in buffer_ of the newest sample of the next output
};

extern template class FirDecimator<float>;
extern template class FirDecimator<cf>;

// Correlates a real signal with exp(-j 2 pi f t). take() returns the complex
// amplitude A exp(j phi) of a component A cos(2 pi f t + phi) over the samples
// added since the last take(). Phases refer to the first sample ever processed.
class Tone {
public:
    Tone() = default;
    Tone(double freq_hz, double rate_hz) : rotator_(freq_hz, rate_hz) {}
    void add(double x) {
        sum_ += x * std::conj(rotator_.next());
        ++count_;
    }
    // Advances the time without using the sample (gated measurements).
    void skip() { rotator_.next(); }
    [[nodiscard]] cd take();
    void reset() { rotator_.reset(); sum_ = {}; count_ = 0; }

private:
    Rotator rotator_;
    cd sum_{};
    std::size_t count_{0};
};

// Selects one channel from the SDR stream: shifts it from `offset_hz` to 0 Hz,
// low-pass filters and decimates. An optional second, narrower filter at the
// output rate is used by the ADF (NDB channels are only 1 kHz apart).
class Channelizer {
public:
    struct Options {
        double input_rate_hz{256000.0};
        double offset_hz{60000.0};      // channel position relative to the tuned centre frequency
        std::size_t decimation{8};
        double cutoff_hz{16000.0};
        std::size_t taps{129};
        double narrow_cutoff_hz{0.0};   // > 0: second filter at the output rate
        std::size_t narrow_taps{65};
    };

    explicit Channelizer(const Options& options);
    void process(std::span<const cf> in, std::vector<cf>& out);
    void reset();
    [[nodiscard]] double output_rate() const { return options_.input_rate_hz / static_cast<double>(options_.decimation); }
    // Noise-equivalent bandwidth of the whole channel filter [Hz].
    [[nodiscard]] double noise_bandwidth() const;
    [[nodiscard]] const Options& options() const { return options_; }

private:
    Options options_;
    Rotator mixer_;
    FirDecimator<cf> filter_;
    FirDecimator<cf> narrow_;
    std::vector<cf> mixed_;
    std::vector<cf> wide_;
};

// Estimates the carrier-to-noise density C/N0 of an AM channel. The noise
// density is measured in two empty bins at +/- `probe_hz` (Hann-windowed
// 64-point DFTs); the carrier power is the mean channel power minus the noise
// in the channel (`noise_bandwidth_hz`, see Channelizer::noise_bandwidth).
class NoiseProbe {
public:
    NoiseProbe(double rate_hz, double probe_hz, double noise_bandwidth_hz);
    void add(cf z);
    // C/N0 [dB-Hz] over the samples since the last take(); -inf without data.
    [[nodiscard]] double take_cn0_dbhz();

private:
    static constexpr std::size_t kLength = 64;
    double rate_hz_;
    double noise_bandwidth_hz_;
    std::vector<double> window_;
    double window_power_{0.0};
    Rotator upper_;
    Rotator lower_;
    cd upper_sum_{};
    cd lower_sum_{};
    std::size_t index_{0};
    double noise_sum_{0.0};
    std::size_t noise_count_{0};
    double power_sum_{0.0};
    std::size_t power_count_{0};
};

}  // namespace bodennav::dsp
