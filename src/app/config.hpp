#pragma once

// Configuration file (INI style, see config/bodennav.conf.example).

#include "io/source.hpp"
#include "nav/adf.hpp"

#include <cstdint>
#include <string>

namespace bodennav::app {

struct SourceConfig {
    std::string type;  // "soapy", "file" or "" (receiver not present)
    io::SdrOptions sdr;
    std::string path;
    io::FileFormat format{io::FileFormat::CU8};
    bool realtime{true};
    bool loop{true};
    double offset_hz{60000.0};  // channel offset from the tuned centre (keeps the DC spike out of the channel)
    double min_cn0_dbhz{50.0};
    [[nodiscard]] bool present() const { return !type.empty(); }
};

struct Config {
    // CAN
    std::string can_interface{"can0"};
    std::uint8_t node_id{40};
    std::uint8_t display_node_id{0};  // tuning requests only from this node (0 = any other node)
    std::uint8_t hardware_revision{1};
    std::uint8_t software_revision{1};
    // persistence of the selected frequencies
    std::string state_file;
    double nav_mhz{0.0};
    double adf_khz{0.0};
    double marker_mhz{75.0};
    // receivers
    SourceConfig nav;
    SourceConfig gs;
    SourceConfig marker;
    SourceConfig adf;
    double vor_offset_deg{0.0};
    nav::AdfDecoder::Options adf_options;
};

// Reads `path`; false with `error` (file name and line) on a problem.
bool load_config(const std::string& path, Config& config, std::string& error);

// Selected frequencies (state file, "nav_mhz = ..." / "adf_khz = ...").
void load_state(Config& config);
void save_state(const Config& config);

}  // namespace bodennav::app
