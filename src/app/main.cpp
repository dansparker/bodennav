// bodennav: VOR/ILS/marker/ADF receiver with low-cost SDRs, output via CANaerospace.

#include "app/config.hpp"
#include "app/receiver.hpp"
#include "app/state.hpp"
#include "can/canaerospace.hpp"
#include "can/socketcan.hpp"
#include "nav/frequency.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

using namespace bodennav;

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop.store(true); }

void usage() {
    std::puts(
        "usage: bodennav -c <config> [--candump] [--interface <can>] [--seconds <n>]\n"
        "  -c, --config     configuration file (see config/bodennav.conf.example)\n"
        "  --candump        print the frames as candump -L lines instead of sending them\n"
        "  --interface      CAN interface (overrides [can] interface)\n"
        "  --seconds        stop after n seconds (tests, replay)");
}

std::unique_ptr<io::SampleSource> open_source(const app::SourceConfig& src, const char* name) {
    std::string error;
    if (src.type == "file") {
        auto file = std::make_unique<io::FileSource>(src.path, src.format, src.sdr.rate_hz, src.realtime, src.loop);
        if (!file->ok()) {
            std::fprintf(stderr, "bodennav: [%s] cannot open %s\n", name, src.path.c_str());
            return nullptr;
        }
        return file;
    }
    auto sdr = io::open_sdr(src.sdr, error);
    if (!sdr) std::fprintf(stderr, "bodennav: [%s] %s\n", name, error.c_str());
    return sdr;
}

double wall_seconds() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path;
    std::string interface_override;
    bool candump = false;
    double seconds = 0.0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
        if (arg == "-c" || arg == "--config") config_path = value();
        else if (arg == "--candump") candump = true;
        else if (arg == "--interface") interface_override = value();
        else if (arg == "--seconds") seconds = std::atof(value().c_str());
        else {
            usage();
            return arg == "-h" || arg == "--help" ? 0 : 2;
        }
    }
    if (config_path.empty()) {
        usage();
        return 2;
    }

    app::Config config;
    std::string error;
    if (!app::load_config(config_path, config, error)) {
        std::fprintf(stderr, "bodennav: %s\n", error.c_str());
        return 1;
    }
    if (!interface_override.empty()) config.can_interface = interface_override;
    app::load_state(config);

    can::SocketCan bus;
    if (!candump && !bus.open(config.can_interface, error)) {
        std::fprintf(stderr, "bodennav: %s\n", error.c_str());
        return 1;
    }

    app::SharedState shared;
    std::unique_ptr<app::Receiver> nav_rx, gs_rx, marker_rx, adf_rx;
    auto make = [&](app::Role role, const app::SourceConfig& src, const char* name) -> std::unique_ptr<app::Receiver> {
        if (!src.present()) return nullptr;
        auto source = open_source(src, name);
        if (!source) return nullptr;
        return std::make_unique<app::Receiver>(role, std::move(source), src, config, shared);
    };
    nav_rx = make(app::Role::Nav, config.nav, "nav");
    gs_rx = make(app::Role::Glideslope, config.gs, "gs");
    marker_rx = make(app::Role::Marker, config.marker, "marker");
    adf_rx = make(app::Role::Adf, config.adf, "adf");
    if (!nav_rx && !gs_rx && !marker_rx && !adf_rx) {
        std::fprintf(stderr, "bodennav: no receiver configured or available\n");
        return 1;
    }

    auto tune_nav = [&](double mhz) {
        if (!nav_rx || !nav::is_nav_frequency(mhz)) return false;
        config.nav_mhz = mhz;
        nav_rx->tune(mhz * 1e6);
        if (gs_rx) {
            const auto gs = nav::glideslope_for(mhz);
            gs_rx->tune(gs ? *gs * 1e6 : 0.0);
        }
        std::lock_guard lock(shared.mutex);
        shared.state.nav_mhz = mhz;
        return true;
    };
    auto tune_adf = [&](double khz) {
        if (!adf_rx || !nav::is_adf_frequency(khz)) return false;
        config.adf_khz = khz;
        adf_rx->tune(khz * 1e3);
        std::lock_guard lock(shared.mutex);
        shared.state.adf_khz = khz;
        return true;
    };
    if (nav_rx && !tune_nav(config.nav_mhz)) std::fprintf(stderr, "bodennav: [nav] no valid frequency_mhz\n");
    if (adf_rx && !tune_adf(config.adf_khz)) std::fprintf(stderr, "bodennav: [adf] no valid frequency_khz\n");
    if (marker_rx) marker_rx->tune(config.marker_mhz * 1e6);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    for (auto* rx : {nav_rx.get(), gs_rx.get(), marker_rx.get(), adf_rx.get()}) {
        if (rx) rx->start();
    }

    can::Encoder encoder(config.node_id);
    auto emit = [&](const can::Frame& f) {
        if (candump) std::printf("%s\n", can::to_candump(f, wall_seconds(), config.can_interface).c_str());
        else bus.send(f);
    };

    // Receive: identification service and tuning requests (proposed IDs 1530/1531).
    std::thread rx_thread;
    std::atomic<double> pending_nav{0.0};
    std::atomic<double> pending_adf{0.0};
    if (!candump) {
        rx_thread = std::thread([&] {
            while (!g_stop.load()) {
                const auto frame = bus.receive(100);
                if (!frame) continue;
                const auto msg = can::decode(*frame);
                if (!msg || msg->node_id == config.node_id) continue;
                if (can::is_ids_request(*msg, config.node_id)) {
                    bus.send(encoder.ids_response(msg->message_code, config.hardware_revision, config.software_revision));
                    continue;
                }
                if (config.display_node_id != 0 && msg->node_id != config.display_node_id) continue;
                if (msg->id == can::id::kNav1TuneRequest) {
                    if (const auto v = msg->as_float()) pending_nav.store(*v);
                } else if (msg->id == can::id::kAdf1TuneRequest) {
                    if (const auto v = msg->as_float()) pending_adf.store(*v);
                }
            }
        });
    }

    const auto start = app::Clock::now();
    auto next = start;
    unsigned tick = 0;
    while (!g_stop.load()) {
        next += std::chrono::milliseconds(50);
        std::this_thread::sleep_until(next);
        const auto now = app::Clock::now();

        if (const double f = pending_nav.exchange(0.0); f > 0.0 && f != config.nav_mhz) {
            if (tune_nav(nav::to_10khz(f) / 100.0)) app::save_state(config);
        }
        if (const double f = pending_adf.exchange(0.0); f > 0.0 && f != config.adf_khz) {
            if (tune_adf(std::round(f * 2.0) / 2.0)) app::save_state(config);
        }

        app::NavState snapshot;
        {
            std::lock_guard lock(shared.mutex);
            snapshot = shared.state;
        }
        for (const auto& f : app::frames_for_tick(snapshot, tick, now, encoder)) emit(f);
        if (candump) std::fflush(stdout);
        ++tick;

        if (seconds > 0.0 && now - start >= std::chrono::duration<double>(seconds)) break;
        bool all_finished = true;
        for (auto* rx : {nav_rx.get(), gs_rx.get(), marker_rx.get(), adf_rx.get()}) {
            if (rx && !rx->finished()) all_finished = false;
        }
        if (all_finished) break;
    }
    g_stop.store(true);
    for (auto* rx : {nav_rx.get(), gs_rx.get(), marker_rx.get(), adf_rx.get()}) {
        if (rx) rx->stop();
    }
    if (rx_thread.joinable()) rx_thread.join();
    return 0;
}
