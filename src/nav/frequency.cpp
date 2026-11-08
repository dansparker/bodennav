#include "nav/frequency.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace bodennav::nav {

namespace {

// Localizer / glide path pairing, ICAO Annex 10 Volume I, 3.1.6.1.1 (in 10 kHz units).
constexpr std::array<std::pair<int, int>, 40> kPairs{{
    {10810, 33470}, {10815, 33455}, {10830, 33410}, {10835, 33395}, {10850, 32990},
    {10855, 32975}, {10870, 33050}, {10875, 33035}, {10890, 32930}, {10895, 32915},
    {10910, 33140}, {10915, 33125}, {10930, 33200}, {10935, 33185}, {10950, 33260},
    {10955, 33245}, {10970, 33320}, {10975, 33305}, {10990, 33380}, {10995, 33365},
    {11010, 33440}, {11015, 33425}, {11030, 33500}, {11035, 33485}, {11050, 32960},
    {11055, 32945}, {11070, 33020}, {11075, 33005}, {11090, 33080}, {11095, 33065},
    {11110, 33170}, {11115, 33155}, {11130, 33230}, {11135, 33215}, {11150, 33290},
    {11155, 33275}, {11170, 33350}, {11175, 33335}, {11190, 33110}, {11195, 33095},
}};

}  // namespace

int to_10khz(double mhz) { return static_cast<int>(std::lround(mhz * 100.0)); }

bool is_nav_frequency(double mhz) {
    const int f = to_10khz(mhz);
    if (std::abs(mhz * 100.0 - f) > 0.01) return false;
    return f >= 10800 && f <= 11795 && f % 5 == 0;
}

bool is_localizer(double mhz) {
    if (!is_nav_frequency(mhz)) return false;
    const int f = to_10khz(mhz);
    if (f < 10810 || f > 11195) return false;
    return ((f / 10) % 2) == 1;  // odd 100 kHz digit
}

std::optional<double> glideslope_for(double localizer_mhz) {
    if (!is_localizer(localizer_mhz)) return std::nullopt;
    const int f = to_10khz(localizer_mhz);
    for (const auto& [loc, gs] : kPairs) {
        if (loc == f) return gs / 100.0;
    }
    return std::nullopt;
}

bool is_adf_frequency(double khz) {
    if (khz < 190.0 || khz > 1750.0) return false;
    const double steps = khz * 2.0;
    return std::abs(steps - std::round(steps)) < 0.01;
}

}  // namespace bodennav::nav
