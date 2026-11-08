// bodennav-siggen: writes test IQ files (cu8, as rtl_sdr) for replay with
// "source = file", e.g. to test OpenEFIS on vcan0 without a receiver.

#include "sim/siggen.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

using namespace bodennav;

namespace {

void usage() {
    std::puts(
        "usage: bodennav-siggen <vor|loc|gs|outer|middle|inner|ndb> -o <file.cu8> [options]\n"
        "  --radial <deg>     VOR radial (default 0)\n"
        "  --ddm <value>      ILS DDM (90 Hz - 150 Hz), e.g. 0.0775 = 1 dot localizer\n"
        "  --bearing <deg>    NDB relative bearing (switched loop antenna)\n"
        "  --plain            NDB without switched antenna\n"
        "  --ident <text>     identifier in Morse\n"
        "  --seconds <n>      length (default 20)\n"
        "  --rate <hz>        sample rate (default 256000)\n"
        "  --offset <hz>      channel offset from the centre (default 60000, ndb 20000)\n"
        "  --cn0 <dBHz>       carrier to noise density (default 70, 0 = no noise)");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    const std::string kind = argv[1];
    std::string out_path;
    sim::Common c;
    c.rate_hz = 256000.0;
    c.seconds = 20.0;
    c.offset_hz = kind == "ndb" ? 20000.0 : 60000.0;
    c.cn0_dbhz = 70.0;
    c.amplitude = 0.4;  // headroom for the modulation peaks in 8 bit
    sim::VorParams vor;
    sim::IlsParams ils;
    sim::NdbParams ndb;
    std::string ident;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        auto num = [&]() { return i + 1 < argc ? std::atof(argv[++i]) : 0.0; };
        if (arg == "-o") out_path = i + 1 < argc ? argv[++i] : "";
        else if (arg == "--radial") vor.radial_deg = num();
        else if (arg == "--ddm") ils.ddm = num();
        else if (arg == "--bearing") ndb.bearing_deg = num();
        else if (arg == "--plain") ndb.switched_antenna = false;
        else if (arg == "--ident") ident = i + 1 < argc ? argv[++i] : "";
        else if (arg == "--seconds") c.seconds = num();
        else if (arg == "--rate") c.rate_hz = num();
        else if (arg == "--offset") c.offset_hz = num();
        else if (arg == "--cn0") c.cn0_dbhz = num();
        else {
            usage();
            return 2;
        }
    }
    if (out_path.empty()) {
        usage();
        return 2;
    }
    std::vector<dsp::cf> samples;
    if (kind == "vor") {
        vor.ident = ident;
        samples = sim::vor(c, vor);
    } else if (kind == "loc" || kind == "gs") {
        ils.sdm = kind == "loc" ? 0.40 : 0.80;
        if (kind == "loc") ils.ident = ident;
        samples = sim::ils(c, ils);
    } else if (kind == "outer") {
        samples = sim::marker(c, sim::MarkerType::Outer);
    } else if (kind == "middle") {
        samples = sim::marker(c, sim::MarkerType::Middle);
    } else if (kind == "inner") {
        samples = sim::marker(c, sim::MarkerType::Inner);
    } else if (kind == "ndb") {
        ndb.ident = ident;
        samples = sim::ndb(c, ndb);
    } else {
        usage();
        return 2;
    }
    const auto bytes = sim::to_cu8(samples);
    std::FILE* f = std::fopen(out_path.c_str(), "wb");
    if (f == nullptr) {
        std::fprintf(stderr, "cannot write %s\n", out_path.c_str());
        return 1;
    }
    std::fwrite(bytes.data(), 1, bytes.size(), f);
    std::fclose(f);
    std::printf("%s: %zu samples at %.0f Hz, channel at %+.0f Hz\n", out_path.c_str(), samples.size(), c.rate_hz,
                c.offset_hz);
    return 0;
}
