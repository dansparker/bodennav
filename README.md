# bodennav

Empfänger für bodengestützte Funknavigation – **VOR, ILS (Localizer + Gleitpfad), Marker und
ADF/NDB** – mit günstigen SDR-Empfängern (RTL-SDR) auf einem Linux-Rechner (Raspberry Pi).
Die Werte gehen über **CANaerospace** an [OpenEFIS](https://github.com/dansparker/open-efis)
und alle anderen Geräte am Bus, genau mit den Kennungen aus OpenEFIS
[ADR 0007](https://github.com/dansparker/open-efis/blob/main/docs/adr/0007-nav-radio.md).

[![CI](https://github.com/dansparker/bodennav/actions/workflows/ci.yml/badge.svg)](https://github.com/dansparker/bodennav/actions/workflows/ci.yml)

> **Nicht zertifiziert, nicht für IFR-Navigation.** Experimentalgerät für Eigenbauflugzeuge und
> zum Lernen. Bisher nur im Simulator getestet.

## Überblick

```
 VOR/LOC-Antenne ─ RTL-SDR #1 (108–118 MHz) ─┐
 GS-Antenne      ─ RTL-SDR #2 (329–335 MHz) ─┤   Raspberry Pi            CAN 500 kbit/s
 Marker-Antenne  ─ RTL-SDR #3 (75 MHz)      ─┼─► bodennav ─► can0 ────► OpenEFIS, Autopilot …
 ADF-Antenne*    ─ RTL-SDR #4 (190–1750 kHz)─┘   (SoapySDR)   (MCP2515/MCP2518FD)
 * Kreuzrahmen + Hilfsantenne, umgeschaltet (docs/adf-antenne.md)
```

| Empfänger | Signal | Ergebnis | CANaerospace |
|---|---|---|---|
| NAV | VOR: 30 Hz AM gegen 30 Hz FM auf 9960 Hz | Radial (missweisend, FROM) | 1126 |
| NAV | Localizer: DDM 90/150 Hz | Ablage in °, + = rechts vom Kurs | 1087 |
| GS | Gleitpfad: DDM 90/150 Hz, Frequenz automatisch gepaart | Ablage in °, + = über dem Pfad | 1091 |
| Marker | 400/1300/3000 Hz, getastet | O/M/I-Bits (blinken im Takt) | 1501 |
| ADF | Rahmen-/Hilfsantenne, Zeitmultiplex | relative Peilung | 1083 |
| alle | Morse-Kennung (1020 Hz bzw. 400/1020 Hz) | Stationskennung | 1502, 1503 |
| – | eingestellte Frequenzen | MHz / kHz | 1104, 1108 |

Werte werden nur gesendet, solange das Signal gültig ist (Träger/Rausch-Verhältnis,
Modulationsgrade, Stabilität); OpenEFIS zeigt nach 500 ms ohne Wert „kein Signal“.

## Aufbau des Codes

```
src/dsp/    Oszillator, FIR-Dezimierer, Kanalfilter, Tonkorrelator, C/N0-Schätzer, Morse
src/nav/    VorDecoder, IlsDecoder (LOC/GS), MarkerDecoder, AdfDecoder, ILS-Frequenzpaarung
src/can/    CANaerospace-Kodierung (wie OpenEFIS), SocketCAN
src/io/     SDR über SoapySDR, IQ-Dateien (cu8/cs16/cf32) zum Abspielen
src/sim/    Signalgenerator für Tests und bodennav-siggen
src/app/    Konfiguration, Empfänger-Threads, Sendeplan, main
tests/      Unit- und Ende-zu-Ende-Tests (laufen in GitHub Actions)
docs/       Architektur und Empfehlungen, CAN-Schnittstelle, ADF-Antenne
```

C++20 ohne Abhängigkeiten im Kern; nur `bodennav` selbst braucht optional SoapySDR.

## Bauen

```bash
sudo apt install cmake g++ libsoapysdr-dev soapysdr-module-rtlsdr
```

```bash
cmake -S . -B build && cmake --build build -j4
```

```bash
./build/bodennav_tests
```

Für den RTL-SDR Blog V4 den Treiber von rtl-sdr-blog installieren (der Debian-Treiber kennt den
V4 nicht richtig), siehe [docs/architektur.md](docs/architektur.md#software-auf-dem-pi).

## Ausprobieren ohne Hardware

```bash
./build/bodennav-siggen vor -o vor.cu8 --radial 123 --ident ABC --seconds 30
```

```bash
./build/bodennav -c config/replay-vor.conf --candump
```

Ohne `--candump` gehen die Frames auf `can0` bzw. mit `--interface vcan0` auf einen virtuellen
Bus, den OpenEFIS direkt anzeigt.

## Inbetriebnahme (Kurzfassung)

1. SDRs mit eigenen Seriennummern versehen (`rtl_eeprom -d 0 -s NAV00001`).
2. `config/bodennav.conf.example` nach `/etc/bodennav.conf` kopieren und anpassen.
3. CAN-Adapter einrichten: `ip link set can0 up type can bitrate 500000`.
4. In OpenEFIS `"can": { "node_ids": { "nav_radio": 40 } }` und den Kanal
   `data_sources.nav_radio = "canaerospace"` setzen.
5. VOR am Boden gegen ein bekanntes Radial (oder VOT) prüfen, `vor_offset_deg` setzen.
6. ADF: Bodenlauf mit bekanntem NDB, `bearing_offset_deg` und ggf. `invert_*` setzen.

## Dokumentation

* [docs/architektur.md](docs/architektur.md) – Hardware-Empfehlungen, Signalverarbeitung, Grenzen
* [docs/canaerospace.md](docs/canaerospace.md) – Schnittstelle, Vorschlag für Abstimmbefehle
* [docs/adf-antenne.md](docs/adf-antenne.md) – umgeschaltete Kreuzrahmenantenne (Entwurf)

## Lizenz

MIT, siehe [LICENSE](LICENSE).
