#pragma once

// Station identifier decoding: a keyed audio tone (1020 Hz for VOR/ILS, 400 or
// 1020 Hz for NDB) is detected in short blocks and the on/off pattern is decoded
// as Morse code. An identifier counts as confirmed when the same word was
// decoded twice in a row.

#include "dsp/dsp.hpp"

#include <string>
#include <vector>

namespace bodennav::dsp {

class MorseDecoder {
public:
    explicit MorseDecoder(double block_s);
    // One detection result per block; true when a new identifier was confirmed.
    bool feed(bool tone);
    [[nodiscard]] const std::string& ident() const { return ident_; }
    [[nodiscard]] double dot_seconds() const { return dot_s_; }
    // Counts every decoded word equal to the confirmed identifier (to see that it is still sent).
    [[nodiscard]] std::size_t matches() const { return matches_; }
    void reset();

private:
    void end_mark(double seconds);
    void finish_letter();
    bool finish_word();

    static constexpr int kDebounce = 2;

    double block_s_;
    bool tone_{false};
    int run_{0};
    int pending_{0};
    double dot_s_{0.12};
    bool letter_done_{true};
    bool word_done_{true};
    bool word_valid_{true};
    std::string symbols_;  // '.' and '-'
    std::string word_;
    std::string last_word_;
    std::string ident_;
    std::size_t matches_{0};
};

// Converts a Morse pattern like ".-" to its character; 0 when unknown.
[[nodiscard]] char morse_to_char(const std::string& pattern);
// Pattern of a character ("" when not in the table).
[[nodiscard]] std::string char_to_morse(char c);

// Detects the identifier tone in the AM modulation signal (envelope / carrier - 1)
// and feeds a MorseDecoder.
class IdentDetector {
public:
    struct Options {
        double rate_hz{32000.0};
        std::vector<double> tones_hz{1020.0};
        double block_s{0.01};
        double min_depth{0.02};       // modulation depth needed for "tone on"
        double noise_ratio{3.0};      // ... and this many times the reference bins
        std::vector<double> reference_hz{1650.0, 2350.0};
        double peak_ratio{0.35};      // ... and this part of the recent peak depth (time constant 5 s)
    };
    explicit IdentDetector(const Options& options);
    // `valid` = false: the sample is not used (e.g. switching slot of the ADF antenna), time still advances.
    void add(double modulation, bool valid = true);
    [[nodiscard]] const std::string& ident() const { return morse_.ident(); }
    [[nodiscard]] std::size_t ident_matches() const { return morse_.matches(); }
    [[nodiscard]] bool tone_on() const { return on_; }
    [[nodiscard]] double tone_depth() const { return depth_; }
    void reset();

private:
    void evaluate();

    Options options_;
    std::size_t block_samples_;
    std::size_t position_{0};
    std::size_t used_{0};
    std::vector<Tone> tones_;
    std::vector<Tone> references_;
    MorseDecoder morse_;
    bool on_{false};
    double depth_{0.0};
    double peak_{0.0};
    double peak_decay_;
};

}  // namespace bodennav::dsp
