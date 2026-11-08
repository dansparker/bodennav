#pragma once

// Test signal generator: complex baseband of VOR, ILS, marker and NDB (with the
// switched loop antenna), optionally shifted by an offset, with white noise.
// Used by the tests and by the bodennav-siggen tool (IQ files for replay).

#include "dsp/dsp.hpp"

#include <random>
#include <string>
#include <vector>

namespace bodennav::sim {

// Morse keying of an identifier, repeated: on(t) is true while the tone is keyed.
class MorseKey {
public:
    MorseKey() = default;
    MorseKey(const std::string& text, double dot_s, double pause_s);
    [[nodiscard]] bool on(double t) const;
    [[nodiscard]] double period() const { return period_; }

private:
    std::vector<std::pair<double, double>> marks_;  // [start, end) within one period
    double period_{0.0};
};

struct Common {
    double rate_hz{32000.0};
    double seconds{10.0};
    double offset_hz{0.0};     // channel position in the generated stream
    double amplitude{0.5};
    double carrier_phase_rad{0.4};
    double cn0_dbhz{0.0};      // 0 = no noise
    unsigned seed{1};
};

struct VorParams {
    double radial_deg{0.0};
    double var_depth{0.30};
    double sub_depth{0.30};
    double deviation_hz{480.0};
    double signal_hz{30.0};        // real stations: within 1 %
    std::string ident;
    double ident_depth{0.10};
};

struct IlsParams {
    double ddm{0.0};            // depth 90 Hz - depth 150 Hz
    double sdm{0.40};
    std::string ident;          // localizer only
    double ident_depth{0.10};
};

enum class MarkerType { Outer, Middle, Inner };

struct NdbParams {
    double bearing_deg{0.0};    // relative to the nose, clockwise
    bool switched_antenna{true};
    double slot_s{0.010};
    double clock_error_ppm{0.0};  // antenna unit clock vs SDR clock
    double start_s{0.0037};       // time of the antenna cycle at t = 0
    double loop_ratio{0.3};
    double loop_phase_deg{90.0};
    std::string ident;
    double ident_hz{1020.0};
    double ident_depth{0.95};
};

[[nodiscard]] std::vector<dsp::cf> vor(const Common& c, const VorParams& p);
[[nodiscard]] std::vector<dsp::cf> ils(const Common& c, const IlsParams& p);
[[nodiscard]] std::vector<dsp::cf> marker(const Common& c, MarkerType type);
[[nodiscard]] std::vector<dsp::cf> ndb(const Common& c, const NdbParams& p);
[[nodiscard]] std::vector<dsp::cf> noise_only(const Common& c);

// Interleaved unsigned 8-bit IQ as written by rtl_sdr (127.5 = 0), clipped.
[[nodiscard]] std::vector<std::uint8_t> to_cu8(const std::vector<dsp::cf>& samples, float scale = 1.0f);

}  // namespace bodennav::sim
