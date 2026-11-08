# ADF-Antenne: umgeschaltete Kreuzrahmenantenne (Entwurf)

> Konzept, **noch nicht gebaut**. Die Auswertung (`src/nav/adf.cpp`) ist im Simulator getestet.

## Prinzip

Ein klassisches ADF misst die Richtung mit zwei gekreuzten Rahmenantennen (Längs- und
Querrahmen) und löst die 180°-Mehrdeutigkeit mit einer ungerichteten Hilfsantenne (Sense).
Statt Goniometer und Modulator im Empfänger schaltet hier eine kleine Antenneneinheit die
Antennen im Zeitmultiplex auf **einen** SDR-Eingang:

| Schlitz | Signal am Empfänger |
|---|---|
| 0 | nichts (Sync-Schlitz: nur Empfängerrauschen) |
| 1 | Sense + Längsrahmen A |
| 2 | Sense − Längsrahmen A |
| 3 | Sense + Querrahmen B |
| 4 | Sense − Querrahmen B |

Standard: 10 ms je Schlitz, 50 ms je Zyklus. Der Empfänger findet den Zyklus selbst (leiser
Sync-Schlitz) und braucht **keine Verbindung** zur Antenneneinheit; Taktabweichungen von einigen
100 ppm werden nachgeführt.

Auswertung im komplexen Basisband je Zyklus: S = Mittel der Schlitze 1–4, A = (Z1 − Z2)/2,
B = (Z3 − Z4)/2. Relativ zum Sense-Signal sind A = k·cos β·e^{jψ}, B = k·sin β·e^{jψ}
(β relative Peilung, ψ Rahmenphase ≈ 90°). Aufsummiert werden A·S* und B·S*: Trägerphase,
Frequenzablage und Pegel fallen heraus. Peilung = atan2(Re(B·e^{−jψ}), Re(A·e^{−jψ})).
ψ wird gemessen und innerhalb ±45° um den konfigurierten Wert nachgeführt.

Gültig nur bei ausreichendem SNR, deutlichem Sync-Schlitz, Rahmenpegel ≥ 5 % des Sense-Pegels
und kleinem Quadraturrest – sonst wird 1083 nicht gesendet.

## Antenneneinheit (Vorschlag)

* **Rahmen:** zwei gekreuzte Ferritstäbe (z. B. 2 × 100 mm, Material 61 oder 77) mit je einer
  Wicklung, in einem flachen Gehäuse an der Rumpfunterseite; alternativ eine fertige
  ADF-Kombiantenne (Rahmen + Sense in einem Gehäuse) mit getrennten Ausgängen.
* **Sense:** kurzer Stab oder kapazitive Fläche, hochohmiger Vorverstärker (JFET/MMIC).
* **Vorzeichenwechsel ±A/±B:** Übertrager mit Mittelanzapfung oder Gegentaktstufe je Rahmen,
  umgeschaltet mit Analogschaltern (z. B. ADG419/ADG1419 oder 74HC4053 bei kleinen Pegeln) bzw.
  PIN-Dioden; Summierung mit dem Sense-Signal, Breitbandverstärker auf 50 Ω, dann Kabel zum SDR.
* **Ablaufsteuerung:** kleiner Mikrocontroller (ATtiny, RP2040, STM32) mit Quarz, 5 Zustände,
  10 ms je Zustand; keine Software-Verbindung zum Empfänger nötig. Stromversorgung über das
  Antennenkabel (Bias-T des RTL-SDR Blog V4, 4,5 V) möglich.
* **Pegel:** Rahmen etwa 0,3 × Sense-Pegel (`loop_ratio` im Simulator); zu großer Rahmenanteil
  ist unkritisch, zu kleiner macht die Peilung verrauscht.

## Einbau und Kalibrierung

1. `loop_phase_deg`: bei Ferritrahmen und Sense ohne Phasenschieber nahe ±90°; mit
   `track_loop_phase = true` wird die gemessene Phase innerhalb ±45° nachgeführt (eine
   Diagnoseausgabe des Messwerts fehlt noch).
2. Flugzeug am Boden auf verschiedene Steuerkurse drehen (oder rollen), Peilung gegen die
   berechnete Richtung zum NDB vergleichen:
   * links/rechts vertauscht (Spiegelung an der Längsachse) → `invert_lateral = true`;
   * vorn/hinten vertauscht (Spiegelung an der Querachse) → `invert_fore_aft = true`;
   * immer um 180° falsch (Sense-Phase verkehrt) → `loop_phase_deg` um 180° ändern;
   * fester Versatz → `bearing_offset_deg`.
3. Quadrantenfehler (Rumpf verzerrt das Feld, typisch ±5–15° mit 2θ-Verlauf) sind noch **nicht**
   korrigiert; eine Korrekturtabelle über der Peilung ist vorgesehen.

## Grenzen

* Dämmerungseffekt, Küsteneffekt, Gewitter und Rundfunkstationen nahe der NDB-Frequenz
  verfälschen wie bei jedem ADF.
* Die Schaltfrequenz (20 Hz Zyklus) erzeugt im NDB-Ton ein leises Brummen; für die Kennung wird
  nur das Innere der Schlitze ausgewertet.
* Ohne Antenneneinheit (`antenna = none`) gibt es nur Frequenz und Kennung, keine Peilung.
