#include "io/source.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

namespace bodennav::io {

namespace {

double now_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::size_t sample_size(FileFormat f) {
    switch (f) {
        case FileFormat::CU8: return 2;
        case FileFormat::CS16: return 4;
        case FileFormat::CF32: return 8;
    }
    return 2;
}

}  // namespace

bool parse_file_format(const std::string& text, FileFormat& format) {
    if (text == "cu8") format = FileFormat::CU8;
    else if (text == "cs16") format = FileFormat::CS16;
    else if (text == "cf32") format = FileFormat::CF32;
    else return false;
    return true;
}

FileSource::FileSource(std::string path, FileFormat format, double rate_hz, bool realtime, bool loop)
    : path_(std::move(path)), format_(format), rate_(rate_hz), realtime_(realtime), loop_(loop) {
    file_ = std::fopen(path_.c_str(), "rb");
    started_ = now_seconds();
}

FileSource::~FileSource() {
    if (file_ != nullptr) std::fclose(file_);
}

std::size_t FileSource::read(std::span<dsp::cf> out) {
    if (file_ == nullptr || out.empty()) return 0;
    const std::size_t size = sample_size(format_);
    raw_.resize(out.size() * size);
    std::size_t got = std::fread(raw_.data(), size, out.size(), file_);
    if (got == 0 && loop_) {
        std::rewind(file_);
        got = std::fread(raw_.data(), size, out.size(), file_);
    }
    for (std::size_t i = 0; i < got; ++i) {
        const unsigned char* p = raw_.data() + i * size;
        switch (format_) {
            case FileFormat::CU8:
                out[i] = {(p[0] - 127.5f) / 127.5f, (p[1] - 127.5f) / 127.5f};
                break;
            case FileFormat::CS16: {
                std::int16_t v[2];
                std::memcpy(v, p, sizeof(v));
                out[i] = {static_cast<float>(v[0]) / 32768.0f, static_cast<float>(v[1]) / 32768.0f};
                break;
            }
            case FileFormat::CF32: {
                float v[2];
                std::memcpy(v, p, sizeof(v));
                out[i] = {v[0], v[1]};
                break;
            }
        }
    }
    if (realtime_ && got > 0) {
        produced_ += static_cast<double>(got) / rate_;
        const double wait = started_ + produced_ - now_seconds();
        if (wait > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(wait));
    }
    return got;
}

}  // namespace bodennav::io
