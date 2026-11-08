# CANaerospace-Schnittstelle

Grundlage: CANaerospace V1.7, CAN 2.0A, 500 kbit/s, wie in OpenEFIS
[ADR 0002](https://github.com/dansparker/open-efis/blob/main/docs/adr/0002-canaerospace.md) und
[ADR 0007](https://github.com/dansparker/open-efis/blob/main/docs/adr/0007-nav-radio.md).
Jeder Frame: Node-ID, Datentyp, Service-Code, Message-Code (zählt je Kennung), 0–4 Datenbytes,
Big Endian.

Node-ID des Empfängers: Standard **40** (`[can] node_id`). In OpenEFIS:
`"can": { "node_ids": { "nav_radio": 40 } }` und `"data_sources": { "nav_radio": "canaerospace" }`.

## Gesendete Werte

| ID | Hex | Parameter | Typ | Einheit | Rate | Bedingung |
|---|---|---|---|---|---|---|
| 1087 | 0x43F | ILS #1 localizer deviation | FLOAT | deg, **+ = Flugzeug rechts vom Kurs** | 20 Hz | LOC gültig |
| 1091 | 0x443 | ILS #1 glideslope deviation | FLOAT | deg, **+ = Flugzeug über dem Pfad** | 20 Hz | LOC und GS gültig |
| 1501 | 0x5DD | Marker beacons | UCHAR | Bit 0 Outer, 1 Middle, 2 Inner | 20 Hz | Marker-Empfänger vorhanden |
| 1126 | 0x466 | VOR #1 radial | FLOAT | deg missweisend, FROM | 10 Hz | VOR gültig |
| 1083 | 0x43B | ADF #1 bearing | FLOAT | deg relativ zur Längsachse, im Uhrzeigersinn | 10 Hz (neuer Wert alle 0,5 s) | Peilung gültig |
| 1104 | 0x450 | VOR/ILS #1 frequency | FLOAT | MHz | 2 Hz | immer |
| 1108 | 0x454 | ADF #1 frequency | FLOAT | kHz | 2 Hz | immer |
| 1502 | 0x5DE | NAV #1 identifier | ACHAR4 | Zeichen, mit 0 aufgefüllt | 1 Hz | Kennung erkannt und Signal gültig |
| 1503 | 0x5DF | ADF #1 identifier | ACHAR4 | | 1 Hz | Kennung in den letzten 60 s gehört |

Fehlt ein gültiges Signal, wird die Kennung **nicht** gesendet (ADR 0007: nach 500 ms ohne Wert
gilt das Signal als verloren). Ob ILS oder VOR, erkennt OpenEFIS an 1087 bzw. 1126.

## Identifikationsdienst

Anfrage auf ID 128 (Node-Service-Kanal 0, Service 0) an Node 0 (alle) oder 40 → Antwort auf
ID 129: UCHAR4 (Hardware-Revision, Software-Revision, 0, 0), Message-Code der Anfrage. OpenEFIS
listet den Empfänger dann unter MENU.

## Vorschlag: Frequenzwahl vom EFIS

ADR 0007 lässt die Frequenzwahl am Empfänger, weil CANaerospace keine Stellbefehle hat. Ein
SDR-Empfänger hat aber keine Bedienelemente. Vorschlag (frei verfügbarer Bereich 1500–1799,
in OpenEFIS noch **nicht** umgesetzt):

| ID | Hex | Parameter | Typ | Einheit | Sender |
|---|---|---|---|---|---|
| 1530 | 0x5FA | NAV #1 tune request | FLOAT | MHz (108,00–117,95, 50-kHz-Raster) | Display (`can.own_node_id`) |
| 1531 | 0x5FB | ADF #1 tune request | FLOAT | kHz (190–1750, 0,5-kHz-Raster) | Display |

* Der Empfänger nimmt sie nur von `[can] display_node_id` an (0 = jeder andere Knoten), rundet
  auf das Raster, verwirft ungültige Werte und bestätigt über 1104/1108.
* Das EFIS sendet eine Anfrage beim Umschalten (Standby ↔ aktiv), nicht zyklisch.
* Die zuletzt gewählten Frequenzen speichert der Empfänger (`[general] state_file`).

In OpenEFIS wäre das ein kleiner Zusatz zu ADR 0007 (Senden bei „Swap“ im NAV1-Feld und am ADF).

## Prüfen am Bus

```bash
candump -L can0 | grep -E ' (43F|443|466|43B|450|454|5DD|5DE|5DF)#'
```
