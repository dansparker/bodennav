#pragma once

// ADF with a switched crossed-loop antenna and a single SDR channel.
//
// The antenna unit (see docs/adf-antenne.md) cycles through five slots of equal
// length and adds the two loop signals to the sense (whip) signal with either sign:
//
//   slot 0: nothing connected (sync slot, only receiver noise)
//   slot 1: sense + loop A (fore-aft)      slot 2: sense - loop A
//   slot 3: sense + loop B (athwartships)  slot 4: sense - loop B
//
// With the complex baseband of each slot, S = mean of slots 1-4 is the sense
// signal, A = (Z1 - Z2) / 2 and B = (Z3 - Z4) / 2 the loop signals. Relative to
// the sense signal they are k cos(b) e^{j psi} and k sin(b) e^{j psi} with the
// relative bearing b and the loop phase psi (about 90 deg: the loop voltage is in
// quadrature with the whip voltage). The sense signal resolves the 180 deg
// ambiguity of the loops. Carrier phase and frequency offsets cancel because
// only A conj(S) and B conj(S) are accumulated.

#include "dsp/dsp.hpp"
#include "dsp/morse.hpp"

#include <deque>

namespace bodennav::nav {

enum class AdfAntenna { None, Switched };

struct AdfOutput {
    bool carrier{false};         // station received
    bool bearing_valid{false};
    double bearing_deg{0.0};     // relative to the nose, clockwise 0..360
    double snr_db{0.0};          // sense power / noise power in the sync slot
    double loop_ratio{0.0};      // loop amplitude / sense amplitude (k)
    double quadrature{0.0};      // part orthogonal to the loop phase (0 = ideal)
    double loop_phase_deg{0.0};  // loop phase used for the bearing
    double sync_contrast{0.0};   // power in the sync slot / power in the other slots
};

class AdfDecoder {
public:
    struct Options {
        double rate_hz{32000.0};
        AdfAntenna antenna{AdfAntenna::Switched};
        double slot_s{0.010};
        double guard_s{0.0015};           // samples skipped at both ends of a slot (switching, filter)
        double loop_phase_deg{90.0};
        bool track_loop_phase{true};      // follow the measured loop phase within +/-45 deg
        double bearing_offset_deg{0.0};   // installation calibration
        bool invert_fore_aft{false};      // loop A wired the other way round
        bool invert_lateral{false};       // loop B wired the other way round
        double min_snr_db{10.0};
        double min_loop_ratio{0.05};
        double max_quadrature{0.5};
        double max_sync_contrast{0.25};
        int frames_per_chunk{10};         // a new bearing every chunk (0.5 s)
        int window_chunks{2};             // averaged over 1 s
        std::vector<double> ident_tones_hz{400.0, 1020.0};
    };

    explicit AdfDecoder(const Options& options);
    void process(std::span<const dsp::cf> baseband);
    [[nodiscard]] const AdfOutput& output() const { return output_; }
    [[nodiscard]] const dsp::IdentDetector& ident() const { return ident_; }
    [[nodiscard]] std::size_t updates() const { return updates_; }
    void reset();

private:
    struct ChunkSums {
        dsp::cd u{};           // sum of A conj(S)
        dsp::cd v{};           // sum of B conj(S)
        double sense{0.0};     // sum of |S|^2
        double noise{0.0};     // sum of the mean sync-slot power
        double data{0.0};      // sum of the mean data-slot power
        int frames{0};
    };

    void process_chunk();
    void process_plain(std::span<const dsp::cf> baseband);
    void evaluate();

    Options options_;
    std::size_t slot_;
    std::size_t guard_;
    std::size_t frame_;
    std::size_t chunk_;
    std::vector<dsp::cf> buffer_;
    std::vector<double> fold_;
    std::deque<ChunkSums> window_;
    dsp::IdentDetector ident_;
    AdfOutput output_;
    std::size_t updates_{0};
    // AM-only mode (no switched antenna)
    std::vector<dsp::cf> plain_block_;
};

}  // namespace bodennav::nav
