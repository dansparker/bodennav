#include "app/receiver.hpp"

#include "nav/frequency.hpp"

#include <chrono>
#include <cstdio>

namespace bodennav::app {

namespace {

dsp::Channelizer::Options channel_options(Role role, const SourceConfig& src, double rate) {
    dsp::Channelizer::Options o;
    o.input_rate_hz = rate;
    o.offset_hz = src.offset_hz;
    o.decimation = 8;                 // 256 kHz -> 32 kHz
    o.cutoff_hz = 16000.0 * rate / 256000.0;
    o.taps = 129;
    if (role == Role::Adf) {
        o.narrow_cutoff_hz = 3000.0;  // NDB channels are about 1 kHz apart
        o.narrow_taps = 65;
    }
    return o;
}

}  // namespace

Receiver::Receiver(Role role, std::unique_ptr<io::SampleSource> source, const SourceConfig& source_config,
                   const Config& config, SharedState& shared)
    : role_(role),
      source_(std::move(source)),
      source_config_(source_config),
      shared_(shared),
      channelizer_(channel_options(role, source_config, source_->rate())) {
    const double rate = channelizer_.output_rate();
    const double bandwidth = channelizer_.noise_bandwidth();
    if (role_ == Role::Nav) {
        nav::VorDecoder::Options v;
        v.rate_hz = rate;
        v.noise_bandwidth_hz = bandwidth;
        v.radial_offset_deg = config.vor_offset_deg;
        v.min_cn0_dbhz = source_config.min_cn0_dbhz;
        vor_ = std::make_unique<nav::VorDecoder>(v);
    }
    switch (role_) {
        case Role::Nav:
        case Role::Glideslope: {
            nav::IlsDecoder::Options i;
            i.kind = role_ == Role::Nav ? nav::IlsKind::Localizer : nav::IlsKind::Glideslope;
            i.rate_hz = rate;
            i.noise_bandwidth_hz = bandwidth;
            i.min_cn0_dbhz = source_config.min_cn0_dbhz;
            ils_ = std::make_unique<nav::IlsDecoder>(i);
            break;
        }
        case Role::Marker: {
            nav::MarkerDecoder::Options m;
            m.rate_hz = rate;
            m.noise_bandwidth_hz = bandwidth;
            m.min_cn0_dbhz = source_config.min_cn0_dbhz;
            marker_ = std::make_unique<nav::MarkerDecoder>(m);
            break;
        }
        case Role::Adf: {
            nav::AdfDecoder::Options a = config.adf_options;
            a.rate_hz = rate;
            adf_ = std::make_unique<nav::AdfDecoder>(a);
            break;
        }
    }
}

Receiver::~Receiver() { stop(); }

void Receiver::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread([this] { run(); });
}

void Receiver::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void Receiver::clear_state() {
    std::lock_guard lock(shared_.mutex);
    auto& s = shared_.state;
    switch (role_) {
        case Role::Nav:
            s.radial.reset();
            s.localizer.reset();
            s.nav_ident.clear();
            break;
        case Role::Glideslope: s.glideslope.reset(); break;
        case Role::Marker: s.markers.reset(); break;
        case Role::Adf:
            s.adf_bearing.reset();
            s.adf_ident.clear();
            s.adf_ident_at.reset();
            break;
    }
}

void Receiver::apply_tune(double channel_hz) {
    current_ = channel_hz;
    clear_state();
    channelizer_.reset();
    if (vor_) vor_->reset();
    if (ils_) ils_->reset();
    if (marker_) marker_->reset();
    if (adf_) adf_->reset();
    seen_updates_ = 0;
    seen_matches_ = 0;
    if (channel_hz <= 0.0) return;
    localizer_ = role_ == Role::Nav && nav::is_localizer(channel_hz / 1e6);
    if (!source_->tune(channel_hz - source_config_.offset_hz)) {
        std::fprintf(stderr, "bodennav: %s: not tunable, channel assumed at the configured offset\n",
                     source_->description().c_str());
    }
}

void Receiver::run() {
    std::vector<dsp::cf> input(8192);
    std::vector<dsp::cf> baseband;
    while (running_.load()) {
        const double requested = requested_.load();
        if (requested != current_) apply_tune(requested);
        const std::size_t n = source_->read(input);
        if (n == 0) {
            std::fprintf(stderr, "bodennav: %s: end of stream\n", source_->description().c_str());
            break;
        }
        if (current_ <= 0.0) continue;
        baseband.clear();
        channelizer_.process(std::span<const dsp::cf>(input.data(), n), baseband);
        switch (role_) {
            case Role::Nav:
                if (localizer_) ils_->process(baseband);
                else vor_->process(baseband);
                break;
            case Role::Glideslope: ils_->process(baseband); break;
            case Role::Marker: marker_->process(baseband); break;
            case Role::Adf: adf_->process(baseband); break;
        }
        publish(Clock::now());
    }
    clear_state();
    finished_.store(true);
}

void Receiver::publish(Clock::time_point now) {
    std::lock_guard lock(shared_.mutex);
    auto& s = shared_.state;
    switch (role_) {
        case Role::Nav: {
            if (localizer_) {
                if (ils_->updates() == seen_updates_) return;
                seen_updates_ = ils_->updates();
                const auto& out = ils_->output();
                if (out.valid) s.localizer = Timed{out.deviation_deg, now};
                else s.localizer.reset();
                s.nav_ident = ils_->ident().ident();
            } else {
                if (vor_->updates() == seen_updates_) return;
                seen_updates_ = vor_->updates();
                const auto& out = vor_->output();
                if (out.valid) s.radial = Timed{out.radial_deg, now};
                else s.radial.reset();
                s.nav_ident = vor_->ident().ident();
            }
            break;
        }
        case Role::Glideslope: {
            if (ils_->updates() == seen_updates_) return;
            seen_updates_ = ils_->updates();
            const auto& out = ils_->output();
            if (out.valid) s.glideslope = Timed{out.deviation_deg, now};
            else s.glideslope.reset();
            break;
        }
        case Role::Marker: {
            if (marker_->updates() == seen_updates_) return;
            seen_updates_ = marker_->updates();
            s.markers = Timed{static_cast<double>(marker_->bits()), now};
            break;
        }
        case Role::Adf: {
            const auto matches = adf_->ident().ident_matches();
            if (matches != seen_matches_) {
                seen_matches_ = matches;
                s.adf_ident = adf_->ident().ident();
                s.adf_ident_at = now;
            }
            if (adf_->updates() == seen_updates_) return;
            seen_updates_ = adf_->updates();
            const auto& out = adf_->output();
            if (out.bearing_valid) s.adf_bearing = Timed{out.bearing_deg, now};
            else s.adf_bearing.reset();
            break;
        }
    }
}

}  // namespace bodennav::app
