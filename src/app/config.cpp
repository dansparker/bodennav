#include "app/config.hpp"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

namespace bodennav::app {

namespace {

std::string trim(const std::string& s) {
    std::size_t a = 0;
    std::size_t b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

bool to_double(const std::string& s, double& out) {
    try {
        std::size_t used = 0;
        out = std::stod(s, &used);
        return used == s.size();
    } catch (...) {
        return false;
    }
}

bool to_bool(const std::string& s, bool& out) {
    if (s == "1" || s == "true" || s == "yes" || s == "on") out = true;
    else if (s == "0" || s == "false" || s == "no" || s == "off") out = false;
    else return false;
    return true;
}

bool to_u8(const std::string& s, std::uint8_t& out) {
    double v = 0.0;
    if (!to_double(s, v) || v < 0.0 || v > 255.0) return false;
    out = static_cast<std::uint8_t>(v);
    return true;
}

bool source_key(SourceConfig& src, const std::string& key, const std::string& value) {
    double d = 0.0;
    if (key == "source") {
        src.type = value == "none" ? "" : value;
        return value == "none" || value == "soapy" || value == "file";
    }
    if (key == "args") { src.sdr.args = value; return true; }
    if (key == "antenna") { src.sdr.antenna = value; return true; }
    if (key == "path") { src.path = value; return true; }
    if (key == "format") return io::parse_file_format(value, src.format);
    if (key == "realtime") return to_bool(value, src.realtime);
    if (key == "loop") return to_bool(value, src.loop);
    if (!to_double(value, d)) return false;
    if (key == "sample_rate") src.sdr.rate_hz = d;
    else if (key == "gain") src.sdr.gain_db = d;
    else if (key == "ppm") src.sdr.ppm = d;
    else if (key == "offset_hz") src.offset_hz = d;
    else if (key == "min_cn0_dbhz") src.min_cn0_dbhz = d;
    else return false;
    return true;
}

}  // namespace

bool load_config(const std::string& path, Config& config, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = path + ": cannot open";
        return false;
    }
    config.marker.min_cn0_dbhz = 45.0;
    config.adf.offset_hz = 20000.0;
    std::string section;
    std::string line;
    int number = 0;
    while (std::getline(in, line)) {
        ++number;
        const auto comment = line.find_first_of("#;");
        if (comment != std::string::npos) line.erase(comment);
        line = trim(line);
        if (line.empty()) continue;
        auto fail = [&](const std::string& what) {
            error = path + ":" + std::to_string(number) + ": " + what;
            return false;
        };
        if (line.front() == '[') {
            if (line.back() != ']') return fail("bad section");
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) return fail("expected key = value");
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        double d = 0.0;
        bool ok = false;
        if (section == "can") {
            if (key == "interface") { config.can_interface = value; ok = true; }
            else if (key == "node_id") ok = to_u8(value, config.node_id) && config.node_id != 0;
            else if (key == "display_node_id") ok = to_u8(value, config.display_node_id);
            else if (key == "hardware_revision") ok = to_u8(value, config.hardware_revision);
        } else if (section == "general") {
            if (key == "state_file") { config.state_file = value; ok = true; }
        } else if (section == "nav") {
            if (key == "frequency_mhz") ok = to_double(value, config.nav_mhz);
            else if (key == "vor_offset_deg") ok = to_double(value, config.vor_offset_deg);
            else ok = source_key(config.nav, key, value);
        } else if (section == "gs") {
            ok = source_key(config.gs, key, value);
        } else if (section == "marker") {
            if (key == "frequency_mhz") ok = to_double(value, config.marker_mhz);
            else ok = source_key(config.marker, key, value);
        } else if (section == "adf") {
            auto& o = config.adf_options;
            if (key == "frequency_khz") ok = to_double(value, config.adf_khz);
            else if (key == "antenna") {
                ok = value == "switched" || value == "none";
                o.antenna = value == "switched" ? nav::AdfAntenna::Switched : nav::AdfAntenna::None;
            } else if (key == "sdr_antenna") { config.adf.sdr.antenna = value; ok = true; }
            else if (key == "track_loop_phase") ok = to_bool(value, o.track_loop_phase);
            else if (key == "invert_fore_aft") ok = to_bool(value, o.invert_fore_aft);
            else if (key == "invert_lateral") ok = to_bool(value, o.invert_lateral);
            else if (key == "slot_ms") { ok = to_double(value, d) && d > 1.0; o.slot_s = d / 1000.0; }
            else if (key == "guard_ms") { ok = to_double(value, d) && d >= 0.0; o.guard_s = d / 1000.0; }
            else if (key == "loop_phase_deg") ok = to_double(value, o.loop_phase_deg);
            else if (key == "bearing_offset_deg") ok = to_double(value, o.bearing_offset_deg);
            else if (key == "min_snr_db") ok = to_double(value, o.min_snr_db);
            else ok = source_key(config.adf, key, value);
        } else {
            return fail("unknown section [" + section + "]");
        }
        if (!ok) return fail("invalid entry '" + key + " = " + value + "'");
    }
    return true;
}

void load_state(Config& config) {
    if (config.state_file.empty()) return;
    std::ifstream in(config.state_file);
    std::string line;
    while (std::getline(in, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        double v = 0.0;
        if (!to_double(trim(line.substr(eq + 1)), v)) continue;
        if (key == "nav_mhz") config.nav_mhz = v;
        else if (key == "adf_khz") config.adf_khz = v;
    }
}

void save_state(const Config& config) {
    if (config.state_file.empty()) return;
    const std::string tmp = config.state_file + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) return;
        out << "nav_mhz = " << config.nav_mhz << "\nadf_khz = " << config.adf_khz << "\n";
    }
    std::rename(tmp.c_str(), config.state_file.c_str());
}

}  // namespace bodennav::app
