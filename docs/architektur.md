# Architektur und Empfehlungen

## Ziel

Ein NAV-Empfänger (VOR/ILS mit Gleitpfad und Markern) und ein ADF für OpenEFIS, aufgebaut aus
handelsüblichen SDR-Sticks. Die Frequenzwahl kann am Empfänger (Konfiguration/Zustandsdatei)
oder vom EFIS über CAN erfolgen (OpenEFIS: Taste SWAP bzw. Feld ADF) (siehe [canaerospace.md](canaerospace.md)).
DME, NAV2 und COM sind nicht Teil des Projekts.

## Warum vier SDRs

Ein RTL-SDR sieht höchstens etwa 2,4 MHz gleichzeitig. Die Signale liegen weit auseinander und
müssen beim ILS-Anflug **gleichzeitig** empfangen werden:

| Signal | Band | Abstand |
|---|---|---|
| Marker | 75 MHz | |
| VOR / Localizer | 108,00–117,95 MHz | |
| Gleitpfad | 328,6–335,4 MHz (fest an die LOC-Frequenz gekoppelt) | |
| NDB | 190–1750 kHz | |

Daher je ein Empfänger pro Band. Ein Zeitmultiplex (ein Stick springt zwischen LOC und GS) wäre
möglich, kostet aber Aktualisierungsrate und Robustheit genau im Endanflug – nicht empfohlen.
Wer nur VOR braucht, baut nur den NAV-Empfänger; jeder Abschnitt in der Konfiguration ist optional.

## Hardware-Empfehlung

| Teil | Empfehlung | ca. Preis | Bemerkung |
|---|---|---|---|
| SDR NAV, GS, Marker | **RTL-SDR Blog V4** (R828D, TCXO 1 ppm) | 3 × 40 € | TCXO wichtig: keine ppm-Kalibrierung, wenig Drift bei Temperatur |
| SDR ADF | RTL-SDR Blog V4 (eingebauter HF-Aufwärtsmischer) | 40 € | Unter 500 kHz ist die Empfindlichkeit zu prüfen; Alternative **Airspy HF+ Discovery** (≈ 170 €, sehr gut auf LW/MW, auch über SoapySDR) |
| Rechner | **Raspberry Pi 4/5** (eigener, nicht der EFIS-Pi) | 60–80 € | Eigenes Gerät = Fehlerisolation; Rechenlast ca. 10–15 % eines Kerns pro Empfänger |
| CAN | **MCP2518FD-HAT** (z. B. Waveshare 2-CH CAN FD HAT) oder MCP2515 | 25 € | wie am EFIS, SocketCAN, 120 Ω Abschluss nur an den Busenden |
| USB | aktiver USB-Hub oder kurze Verlängerungen | | Sticks nicht direkt nebeneinander (Wärme, Übersprechen) |
| Filter | Bandpass je Empfänger: **FM-Sperre 88–108 MHz** vor NAV/Marker, SAW-Bandpass vor GS, MW-Rundfunk-Sperre vor ADF | 10–30 € | RTL-SDRs übersteuern leicht; UKW-Rundfunk liegt direkt unter dem NAV-Band |
| Antennen | NAV: übliche V-/Katzenbart-Antenne (horizontal polarisiert), GS: Faltdipol oder Abgriff an der NAV-Antenne mit Weiche, Marker: Unterseiten-Stab/Schlitz, ADF: siehe [adf-antenne.md](adf-antenne.md) | | Bestehende Flugzeugantennen über Splitter/Diplexer nutzbar |
| Gehäuse | Alu, geschirmt, Stromversorgung 12/24 V → 5 V mit Filter | | Störungen des Pi (Takt, Netzteil) nicht in die Antennen |

### Weitere Empfehlungen

* **Seriennummern** in den RTL-SDR-EEPROMs setzen (`rtl_eeprom -d N -s NAV00001`), damit nach einem
  Neustart nie der GS-Stick am NAV-Eingang hängt. In der Konfiguration wird jeder Stick per
  `serial=` angesprochen.
* **Eigener Pi statt EFIS-Pi:** Ein Hänger im SDR-Treiber (USB) darf das Display nicht treffen.
  Technisch läuft `bodennav` auch auf dem EFIS-Pi (gleiche SocketCAN-Schnittstelle).
* **Systemd-Dienst** mit `Restart=always` und Watchdog; schreibgeschütztes Root-Dateisystem
  (overlayfs) wie beim EFIS-Image, die Zustandsdatei auf eine eigene Partition.
* **Audio der Kennung** (später): ALSA-Ausgang für das Intercom, damit der Pilot die Kennung
  hören kann – Pflicht bei echter Navigation; die dekodierte Kennung auf dem EFIS ist eine Hilfe,
  kein Ersatz.
* **Selbsttest:** Bei VOR-Empfang gegen ein bekanntes Radial am Platz prüfen (oder VOT). Ein
  kleiner Prüfsender mit `bodennav-siggen` + HackRF/PlutoSDR **nur in geschirmter Umgebung bzw.
  per Kabel mit Dämpfungsglied** – Aussendungen im Flugfunkband sind verboten.
* **Rechtliches:** Reiner Empfang ist erlaubt. Für den Einbau in ein Luftfahrzeug gelten die
  Regeln der jeweiligen Kategorie (Experimental/Eigenbau); das Gerät ist nicht zugelassen und
  nicht für IFR gedacht.

## Software auf dem Pi

```bash
sudo apt install cmake g++ libsoapysdr-dev soapysdr-tools
```

RTL-SDR Blog V4: Treiber von rtl-sdr-blog bauen (der librtlsdr aus Debian unterstützt den R828D
des V4 nicht richtig), danach das SoapySDR-Modul `SoapyRTLSDR` aus den Quellen gegen diesen
Treiber bauen. Prüfen mit `SoapySDRUtil --find="driver=rtlsdr"`.

Airspy HF+: `soapysdr-module-airspyhf` (Debian/Raspberry Pi OS), dann `args = driver=airspyhf`.

## Signalverarbeitung

Alle Empfänger laufen mit 256 kS/s. Die Kanalfrequenz liegt 60 kHz (ADF: 20 kHz) neben der
eingestellten Mittenfrequenz, damit der Gleichanteil-Spike des RTL-SDR nicht im Kanal liegt.

```
IQ 256 kS/s ─► Mischer (−Offset) ─► FIR 129 Taps, fc 16 kHz, ↓8 ─► 32 kS/s komplexes Basisband
                                       (ADF zusätzlich FIR 65 Taps, fc 3 kHz)
```

### VOR

* Hüllkurve |z|, normiert auf den Träger je 0,1 s: m(t) = |z| / Träger − 1.
* **Variabel:** m(t) → Tiefpass 900 Hz (255 Taps) → ↓16 → 2 kS/s → Korrelation mit 30 Hz.
* **Referenz:** m(t) · e^(−j2π·9960·t) → **derselbe** Tiefpass (gleiche Gruppenlaufzeit) → ↓16 →
  FM-Diskriminator → Korrelation mit 30 Hz; die halbe Abtastung Verzug des Diskriminators wird
  rechnerisch ausgeglichen (2,7°).
* Radial = Phase(Referenz) − Phase(variabel), je 0,1 s als Produkt r·v* (so heben sich Abweichungen
  der 30-Hz-Frequenz der Station, ±1 %, heraus), gemittelt über 1 s, neuer Wert alle 0,1 s.
* **Gültig**, wenn C/N0 ≥ 50 dB-Hz, AM-Grad 30 Hz 15–45 %, Hub 300–700 Hz und Streuung der
  0,1-s-Werte ≤ 6°.

### Localizer / Gleitpfad

* Modulationsgrad von 90 Hz und 150 Hz je 0,1 s (9 bzw. 15 ganze Perioden), gemittelt über 0,2 s.
* DDM = m90 − m150, SDM = m90 + m150. Gültig mit SDM 30–50 % (LOC) bzw. 60–95 % (GS) und C/N0.
* Ausgabe in Grad, so dass ein Punkt in OpenEFIS (1,25° LOC, 0,35° GS) einem Normpunkt eines
  zugelassenen Anzeigers entspricht (0,0775 bzw. 0,0875 DDM). Vorzeichen nach ADR 0007.
* GS-Frequenz aus der LOC-Frequenz (ICAO Annex 10, Tabelle der 40 Paare, in `nav/frequency.cpp`;
  vor dem ersten Flug gegen die aktuelle Ausgabe des Annex prüfen).

### Marker

* 10-ms-Blöcke, Modulationsgrad von 400/1300/3000 Hz, Schwelle 40 % ein / 20 % aus, plus C/N0
  (Empfindlichkeit HI/LO = `min_cn0_dbhz`). Die Bits folgen der Tastung, die Lampe blinkt.

### Kennung (Morse)

* 1020-Hz-Ton (NDB: 400 oder 1020 Hz) in 10-ms-Blöcken gegen Referenzfrequenzen, Hysterese.
* Punktlänge adaptiv, Buchstaben- und Wortgrenzen nach 2 bzw. 5 Punktlängen. Eine Kennung gilt
  erst, wenn dasselbe Wort **zweimal hintereinander** erkannt wurde.

### ADF

Siehe [adf-antenne.md](adf-antenne.md): Zeitmultiplex mit fünf Schlitzen, kohärente Auswertung
im Basisband, Peilung = atan2(B, A) nach Projektion auf die Rahmenphase ψ; Hilfsantenne löst die
180°-Mehrdeutigkeit. Ohne umgeschaltete Antenne (`antenna = none`) nur Kennung und Frequenz.

## Tests

Die Tests erzeugen alle Signale synthetisch (`src/sim/siggen.cpp`) und prüfen: Radiale über den
ganzen Kreis, 30-Hz-Frequenzfehler, Rauschen, Kennungen, LOC-/GS-Vorzeichen und -Skalen,
Ablehnung falscher Signale (ILS am VOR-Dekoder, GS am LOC-Dekoder, nur Rauschen), Marker,
ADF-Peilungen mit Taktabweichung der Antenneneinheit, nachgeführte Rahmenphase, und einen
Ende-zu-Ende-Lauf über eine cu8-Datei wie vom RTL-SDR. Die CI spielt zusätzlich eine
simulierte VOR-Datei durch das fertige Programm und prüft die CAN-Frames.

**Noch offen:** Validierung mit echten Signalen (IQ-Aufnahmen mit `rtl_sdr` von VOR, ILS und NDB
in der Nähe eines Platzes; die Aufnahmen können direkt als `source = file` abgespielt werden),
Mehrwegeausbreitung/Scalloping, Übersteuerung durch UKW-Rundfunk, Flugerprobung.
