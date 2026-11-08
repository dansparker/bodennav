#include "io/source.hpp"

#ifdef BODENNAV_HAVE_SOAPY
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.h>
#include <SoapySDR/Formats.hpp>

#include <exception>
#endif

namespace bodennav::io {

#ifdef BODENNAV_HAVE_SOAPY

namespace {

class SoapySource final : public SampleSource {
public:
    SoapySource(SoapySDR::Device* device, SoapySDR::Stream* stream, const SdrOptions& options)
        : device_(device), stream_(stream), options_(options) {}
    ~SoapySource() override {
        device_->deactivateStream(stream_);
        device_->closeStream(stream_);
        SoapySDR::Device::unmake(device_);
    }

    std::size_t read(std::span<dsp::cf> out) override {
        void* buffers[] = {out.data()};
        int flags = 0;
        long long time_ns = 0;
        for (int attempt = 0; attempt < 10; ++attempt) {
            const int ret = device_->readStream(stream_, buffers, out.size(), flags, time_ns, 200000);
            if (ret > 0) return static_cast<std::size_t>(ret);
            if (ret != SOAPY_SDR_TIMEOUT && ret != SOAPY_SDR_OVERFLOW) return 0;
        }
        return 0;
    }

    bool tune(double centre_hz) override {
        try {
            device_->setFrequency(SOAPY_SDR_RX, 0, centre_hz);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    [[nodiscard]] double rate() const override { return options_.rate_hz; }
    [[nodiscard]] std::string description() const override { return "SoapySDR " + options_.args; }

private:
    SoapySDR::Device* device_;
    SoapySDR::Stream* stream_;
    SdrOptions options_;
};

}  // namespace

std::unique_ptr<SampleSource> open_sdr(const SdrOptions& options, std::string& error) {
    SoapySDR::Device* device = nullptr;
    try {
        device = SoapySDR::Device::make(options.args);
        if (device == nullptr) {
            error = "no SDR found for '" + options.args + "'";
            return nullptr;
        }
        device->setSampleRate(SOAPY_SDR_RX, 0, options.rate_hz);
        if (!options.antenna.empty()) device->setAntenna(SOAPY_SDR_RX, 0, options.antenna);
        if (options.gain_db < 0.0) {
            device->setGainMode(SOAPY_SDR_RX, 0, true);
        } else {
            device->setGainMode(SOAPY_SDR_RX, 0, false);
            device->setGain(SOAPY_SDR_RX, 0, options.gain_db);
        }
        if (options.ppm != 0.0 && device->hasFrequencyCorrection(SOAPY_SDR_RX, 0)) {
            device->setFrequencyCorrection(SOAPY_SDR_RX, 0, options.ppm);
        }
        SoapySDR::Stream* stream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
        device->activateStream(stream);
        return std::make_unique<SoapySource>(device, stream, options);
    } catch (const std::exception& e) {
        error = std::string("SoapySDR: ") + e.what();
        if (device != nullptr) SoapySDR::Device::unmake(device);
        return nullptr;
    }
}

#else

std::unique_ptr<SampleSource> open_sdr(const SdrOptions& options, std::string& error) {
    error = "built without SoapySDR, cannot open '" + options.args + "'";
    return nullptr;
}

#endif

}  // namespace bodennav::io
