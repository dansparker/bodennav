#pragma once

// VHF NAV and ADF frequency rules (ICAO Annex 10, Volume I).

#include <optional>

namespace bodennav::nav {

// 108.00-117.95 MHz on the 50 kHz grid.
[[nodiscard]] bool is_nav_frequency(double mhz);
// Localizer channels: 108.10-111.95 MHz with an odd 100 kHz digit (108.10, 108.15, 108.30, ...).
[[nodiscard]] bool is_localizer(double mhz);
// Glide path frequency paired with a localizer frequency [MHz]; nullopt for VOR frequencies.
[[nodiscard]] std::optional<double> glideslope_for(double localizer_mhz);
// NDB/ADF range 190-1750 kHz in 0.5 kHz steps.
[[nodiscard]] bool is_adf_frequency(double khz);

// Frequency in units of 10 kHz (rounded), robust against float input such as 110.2999.
[[nodiscard]] int to_10khz(double mhz);

}  // namespace bodennav::nav
