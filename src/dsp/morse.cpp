#include "dsp/morse.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <utility>

namespace bodennav::dsp {

namespace {

struct Code {
    char c;
    const char* pattern;
};

constexpr std::array<Code, 36> kCodes{{
    {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},    {'F', "..-."},
    {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"}, {'K', "-.-"},  {'L', ".-.."},
    {'M', "--"},   {'N', "-."},   {'O', "---"},  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},
    {'S', "..."},  {'T', "-"},    {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"},
    {'Y', "-.--"}, {'Z', "--.."}, {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
    {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},
}};

constexpr double kMinMark = 0.025;   // shorter marks are glitches [s]
constexpr double kMaxMark = 1.2;     // longer marks are no Morse (e.g. a continuous tone) [s]
constexpr std::size_t kMaxIdentLength = 4;

}  // namespace

char morse_to_char(const std::string& pattern) {
    for (const auto& code : kCodes) {
        if (pattern == code.pattern) return code.c;
    }
    return 0;
}

std::string char_to_morse(char c) {
    const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    for (const auto& code : kCodes) {
        if (code.c == upper) return code.pattern;
    }
    return {};
}

// --- MorseDecoder ---------------------------------------------------------------

MorseDecoder::MorseDecoder(double block_s) : block_s_(block_s) {}

void MorseDecoder::reset() {
    tone_ = false;
    run_ = 0;
    pending_ = 0;
    dot_s_ = 0.12;
    letter_done_ = true;
    word_done_ = true;
    word_valid_ = true;
    symbols_.clear();
    word_.clear();
    last_word_.clear();
    ident_.clear();
    matches_ = 0;
}

bool MorseDecoder::feed(bool tone) {
    // Debounce: a change counts only after kDebounce equal blocks, so single noise
    // blocks neither split a dash nor interrupt the pause between two identifiers.
    if (tone == tone_) {
        pending_ = 0;
        ++run_;
    } else if (++pending_ < kDebounce) {
        ++run_;  // not confirmed yet: still counted to the current state
    } else {
        const int moved = pending_ - 1;  // blocks already counted to the old state
        if (tone_) end_mark((run_ - moved) * block_s_);
        tone_ = tone;
        run_ = pending_;
        pending_ = 0;
    }
    bool confirmed = false;
    if (!tone_) {
        const double space = run_ * block_s_;
        if (!letter_done_ && space > 2.0 * dot_s_) finish_letter();
        if (!word_done_ && space > 5.0 * dot_s_) confirmed = finish_word();
    }
    return confirmed;
}

void MorseDecoder::end_mark(double seconds) {
    if (seconds < kMinMark) return;
    if (seconds > kMaxMark) {  // not Morse: drop the current word
        symbols_.clear();
        word_.clear();
        word_valid_ = false;
        letter_done_ = true;
        word_done_ = false;
        return;
    }
    const bool dot = seconds < 2.0 * dot_s_;
    symbols_.push_back(dot ? '.' : '-');
    const double unit = dot ? seconds : seconds / 3.0;
    dot_s_ = std::clamp(0.8 * dot_s_ + 0.2 * unit, 0.04, 0.3);
    letter_done_ = false;
    word_done_ = false;
}

void MorseDecoder::finish_letter() {
    letter_done_ = true;
    if (symbols_.empty()) return;
    const char c = morse_to_char(symbols_);
    if (c == 0) word_valid_ = false;
    else word_.push_back(c);
    symbols_.clear();
}

bool MorseDecoder::finish_word() {
    if (!letter_done_) finish_letter();
    word_done_ = true;
    bool confirmed = false;
    if (word_valid_ && !word_.empty() && word_.size() <= kMaxIdentLength) {
        if (word_ == last_word_ && word_ != ident_) {
            ident_ = word_;
            confirmed = true;
        }
        if (word_ == ident_) ++matches_;
        last_word_ = word_;
    } else {
        last_word_.clear();
    }
    word_.clear();
    word_valid_ = true;
    return confirmed;
}

// --- IdentDetector --------------------------------------------------------------

IdentDetector::IdentDetector(const Options& options)
    : options_(options),
      block_samples_(std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(options.block_s * options.rate_hz)))),
      morse_(options.block_s),
      peak_decay_(std::exp(-options.block_s / 5.0)) {
    for (const double f : options_.tones_hz) tones_.emplace_back(f, options_.rate_hz);
    for (const double f : options_.reference_hz) references_.emplace_back(f, options_.rate_hz);
}

void IdentDetector::reset() {
    for (auto& t : tones_) t.reset();
    for (auto& t : references_) t.reset();
    morse_.reset();
    position_ = 0;
    used_ = 0;
    on_ = false;
    depth_ = 0.0;
    peak_ = 0.0;
}

void IdentDetector::add(double modulation, bool valid) {
    if (valid) {
        for (auto& t : tones_) t.add(modulation);
        for (auto& t : references_) t.add(modulation);
        ++used_;
    } else {
        for (auto& t : tones_) t.skip();
        for (auto& t : references_) t.skip();
    }
    if (++position_ == block_samples_) evaluate();
}

void IdentDetector::evaluate() {
    double depth = 0.0;
    for (auto& t : tones_) depth = std::max(depth, std::abs(t.take()));
    double reference = 0.0;
    for (auto& t : references_) reference = std::max(reference, std::abs(t.take()));
    const bool enough = used_ * 2 >= block_samples_;
    position_ = 0;
    used_ = 0;
    if (!enough) {
        morse_.feed(on_);  // keep the timing, hold the last state
        return;
    }
    depth_ = depth;
    // the keyed tone sets its own scale: noise far below the received tone depth is ignored
    peak_ = std::max(depth, peak_ * peak_decay_);
    const double threshold =
        std::max({options_.min_depth, options_.noise_ratio * reference, options_.peak_ratio * peak_});
    // hysteresis: switching off needs the tone to drop to 60 % of the threshold
    on_ = on_ ? depth > 0.6 * threshold : depth > threshold;
    morse_.feed(on_);
}

}  // namespace bodennav::dsp
