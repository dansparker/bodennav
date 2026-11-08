#include "sim/siggen.hpp"

#include "dsp/morse.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace bodennav::sim {

using dsp::cf;
using dsp::kTwoPi;

MorseKey::MorseKey(const std::string& text, double dot_s, double pause_s) {
    double t = 0.0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const std::string pattern = dsp::char_to_morse(text[i]);
        for (std::size_t j = 0; j < pattern.size(); ++j) {
            const double length = pattern[j] == '.' ? dot_s : 3.0 * dot_s;
            marks_.emplace_back(t, t + length);
            t += length;
            if (j + 1 < pattern.size()) t += dot_s;
        }
        if (i + 1 < text.size()) t += 3.0 * dot_s;
    }
    period_ = t + pause_s;
}

bool MorseKey::on(double t) const {
    if (marks_.empty() || period_ <= 0.0) return false;
    const double phase = std::fmod(t, period_);
    return std::any_of(marks_.begin(), marks_.end(),
                       [phase](const auto& m) { return phase >= m.first && phase < m.second; });
}

namespace {

class Noise {
public:
    Noise(const Common& c) : rng_(c.seed) {
        if (c.cn0_dbhz > 0.0) {
            const double carrier = c.amplitude * c.amplitude;
            const double n0 = carrier / std::pow(10.0, c.cn0_dbhz / 10.0);
            sigma_ = std::sqrt(n0 * c.rate_hz / 2.0);  // per component
        }
    }
    cf next() {
        if (sigma_ <= 0.0) return {};
        return {static_cast<float>(sigma_ * normal_(rng_)), static_cast<float>(sigma_ * normal_(rng_))};
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    double sigma_{0.0};
};

std::size_t sample_count(const Common& c) {
    return static_cast<std::size_t>(std::llround(c.seconds * c.rate_hz));
}

// A (1 + m(t)) exp(j(2 pi offset t + phase)) + noise
std::vector<cf> am(const Common& c, const std::function<double(double)>& modulation) {
    const std::size_t n = sample_count(c);
    std::vector<cf> out(n);
    Noise noise(c);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / c.rate_hz;
        const double envelope = c.amplitude * (1.0 + modulation(t));
        const double phase = kTwoPi * c.offset_hz * t + c.carrier_phase_rad;
        out[i] = cf(static_cast<float>(envelope * std::cos(phase)), static_cast<float>(envelope * std::sin(phase))) +
                 noise.next();
    }
    return out;
}

}  // namespace

std::vector<cf> vor(const Common& c, const VorParams& p) {
    const MorseKey key(p.ident, 0.1, 3.0);
    const double theta = p.radial_deg * dsp::kDegToRad;
    const double beta = p.deviation_hz / p.signal_hz;
    return am(c, [&](double t) {
        const double w = kTwoPi * p.signal_hz * t;
        double m = p.var_depth * std::cos(w - theta);                              // variable lags by the bearing
        m += p.sub_depth * std::cos(kTwoPi * 9960.0 * t + beta * std::sin(w));    // f = 9960 + 480 cos(w t)
        if (!p.ident.empty() && key.on(t)) m += p.ident_depth * std::cos(kTwoPi * 1020.0 * t);
        return m;
    });
}

std::vector<cf> ils(const Common& c, const IlsParams& p) {
    const MorseKey key(p.ident, 0.1, 3.0);
    const double m90 = 0.5 * (p.sdm + p.ddm);
    const double m150 = 0.5 * (p.sdm - p.ddm);
    return am(c, [&](double t) {
        double m = m90 * std::cos(kTwoPi * 90.0 * t) + m150 * std::cos(kTwoPi * 150.0 * t);
        if (!p.ident.empty() && key.on(t)) m += p.ident_depth * std::cos(kTwoPi * 1020.0 * t);
        return m;
    });
}

std::vector<cf> marker(const Common& c, MarkerType type) {
    return am(c, [type](double t) {
        double tone = 400.0;
        bool on = false;
        switch (type) {
            case MarkerType::Outer: {  // dashes, 2 per second
                on = std::fmod(t, 0.5) < 0.3;
                tone = 400.0;
                break;
            }
            case MarkerType::Middle: {  // alternate dots and dashes
                const double ph = std::fmod(t, 0.632);
                on = ph < 0.25 || (ph >= 0.35 && ph < 0.433);
                tone = 1300.0;
                break;
            }
            case MarkerType::Inner: {  // dots, 6 per second
                on = std::fmod(t, 1.0 / 6.0) < 1.0 / 12.0;
                tone = 3000.0;
                break;
            }
        }
        return on ? 0.95 * std::cos(kTwoPi * tone * t) : 0.0;
    });
}

std::vector<cf> ndb(const Common& c, const NdbParams& p) {
    const MorseKey key(p.ident, 0.1, 3.0);
    const std::size_t n = sample_count(c);
    std::vector<cf> out(n);
    Noise noise(c);
    const double beta = p.bearing_deg * dsp::kDegToRad;
    const dsp::cd loop = std::polar(p.loop_ratio, p.loop_phase_deg * dsp::kDegToRad);
    const double clock = 1.0 + p.clock_error_ppm * 1e-6;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / c.rate_hz;
        double modulation = 0.0;
        if (!p.ident.empty() && key.on(t)) modulation = p.ident_depth * std::cos(kTwoPi * p.ident_hz * t);
        const double phase = kTwoPi * c.offset_hz * t + c.carrier_phase_rad;
        const dsp::cd sense = c.amplitude * (1.0 + modulation) * std::polar(1.0, phase);
        dsp::cd z = sense;
        if (p.switched_antenna) {
            const double antenna_time = t * clock + p.start_s;
            const auto slot = static_cast<long long>(std::floor(antenna_time / p.slot_s)) % 5;
            switch (slot) {
                case 0: z = 0.0; break;
                case 1: z = sense * (1.0 + loop * std::cos(beta)); break;
                case 2: z = sense * (1.0 - loop * std::cos(beta)); break;
                case 3: z = sense * (1.0 + loop * std::sin(beta)); break;
                default: z = sense * (1.0 - loop * std::sin(beta)); break;
            }
        }
        out[i] = cf(static_cast<float>(z.real()), static_cast<float>(z.imag())) + noise.next();
    }
    return out;
}

std::vector<cf> noise_only(const Common& c) {
    return am(c, [](double) { return -1.0; });  // no carrier, only the noise of a carrier of `amplitude`
}

std::vector<std::uint8_t> to_cu8(const std::vector<cf>& samples, float scale) {
    std::vector<std::uint8_t> out(samples.size() * 2);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const float re = std::clamp(127.5f + 127.5f * scale * samples[i].real(), 0.0f, 255.0f);
        const float im = std::clamp(127.5f + 127.5f * scale * samples[i].imag(), 0.0f, 255.0f);
        out[2 * i] = static_cast<std::uint8_t>(std::lround(re));
        out[2 * i + 1] = static_cast<std::uint8_t>(std::lround(im));
    }
    return out;
}

}  // namespace bodennav::sim
