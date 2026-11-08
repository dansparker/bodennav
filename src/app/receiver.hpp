#pragma once

// One SDR (or IQ file) with its decoder, running in its own thread.

#include "app/config.hpp"
#include "app/state.hpp"
#include "dsp/dsp.hpp"
#include "io/source.hpp"
#include "nav/adf.hpp"
#include "nav/ils.hpp"
#include "nav/marker.hpp"
#include "nav/vor.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace bodennav::app {

enum class Role { Nav, Glideslope, Marker, Adf };

struct SharedState {
    std::mutex mutex;
    NavState state;
};

class Receiver {
public:
    Receiver(Role role, std::unique_ptr<io::SampleSource> source, const SourceConfig& source_config,
             const Config& config, SharedState& shared);
    ~Receiver();
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;

    void start();
    void stop();
    // Channel frequency [Hz]; 0 = idle (samples are read but not decoded).
    void tune(double channel_hz) { requested_.store(channel_hz); }
    [[nodiscard]] bool finished() const { return finished_.load(); }

private:
    void run();
    void apply_tune(double channel_hz);
    void publish(Clock::time_point now);
    void clear_state();

    Role role_;
    std::unique_ptr<io::SampleSource> source_;
    SourceConfig source_config_;
    SharedState& shared_;
    dsp::Channelizer channelizer_;
    std::unique_ptr<nav::VorDecoder> vor_;
    std::unique_ptr<nav::IlsDecoder> ils_;
    std::unique_ptr<nav::MarkerDecoder> marker_;
    std::unique_ptr<nav::AdfDecoder> adf_;
    bool localizer_{false};
    double current_{-1.0};
    std::size_t seen_updates_{0};
    std::size_t seen_matches_{0};
    std::atomic<double> requested_{0.0};
    std::atomic<bool> running_{false};
    std::atomic<bool> finished_{false};
    std::thread thread_;
};

}  // namespace bodennav::app
