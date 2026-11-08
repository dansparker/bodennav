#pragma once

// Sources of complex samples: SDR via SoapySDR (RTL-SDR, Airspy HF+, SDRplay ...)
// or IQ files (rtl_sdr output) for replay and tests.

#include "dsp/dsp.hpp"

#include <cstdio>
#include <memory>
#include <string>

namespace bodennav::io {

class SampleSource {
public:
    virtual ~SampleSource() = default;
    // Blocks until samples are available; 0 = end of stream or error.
    virtual std::size_t read(std::span<dsp::cf> out) = 0;
    // Tunes the centre frequency [Hz]; false if not possible (files).
    virtual bool tune(double centre_hz) = 0;
    [[nodiscard]] virtual double rate() const = 0;
    [[nodiscard]] virtual std::string description() const = 0;
};

enum class FileFormat { CU8, CS16, CF32 };

// IQ file. With `realtime` reading is paced to the sample rate (replay to a live
// CAN bus); `loop` restarts at the end of the file.
class FileSource final : public SampleSource {
public:
    FileSource(std::string path, FileFormat format, double rate_hz, bool realtime, bool loop);
    ~FileSource() override;
    [[nodiscard]] bool ok() const { return file_ != nullptr; }
    std::size_t read(std::span<dsp::cf> out) override;
    bool tune(double) override { return false; }
    [[nodiscard]] double rate() const override { return rate_; }
    [[nodiscard]] std::string description() const override { return "file " + path_; }

private:
    std::string path_;
    FileFormat format_;
    double rate_;
    bool realtime_;
    bool loop_;
    std::FILE* file_{nullptr};
    std::vector<unsigned char> raw_;
    double started_{0.0};
    double produced_{0.0};
};

struct SdrOptions {
    std::string args;          // SoapySDR device arguments, e.g. "driver=rtlsdr,serial=NAV1"
    double rate_hz{256000.0};
    double gain_db{-1.0};      // < 0: automatic gain
    double ppm{0.0};           // frequency correction
    std::string antenna;       // optional antenna port
};

// nullptr with `error` when SoapySDR is not available or the device cannot be opened.
std::unique_ptr<SampleSource> open_sdr(const SdrOptions& options, std::string& error);

[[nodiscard]] bool parse_file_format(const std::string& text, FileFormat& format);

}  // namespace bodennav::io
