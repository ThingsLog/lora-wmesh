# W-Mesh Node carrier (Z128) — hardware

Single-board carrier for a W-Mesh tier-1 relay / stake node, sized for a **Kradex Z128**
enclosure (105 × 70 × 40 mm, PC/ABS). It hosts the bare **LoRa-E5-HF** module and the
**Seeed L76K GNSS for XIAO** module soldered as modules, plus the GPS power gate,
decoupling, battery/VIN interface and programming/console headers.

All mechanical/electrical data below is from authoritative sources (Seeed LoRa-E5 datasheet
V1.0 Table 1, Seeed OPL KiCad footprints, Kradex Z128 drawing). **One item is provisional —
the mounting-hole positions — see "Enclosure".**

### Status (validated with KiCad 10.0.7 CLI)
- Board **opens and renders** cleanly (`render-top.png`).
- DRC: **no fab-critical errors** (no shorts / clearance / copper / drill). Remaining items are
  the expected **ratsnest** (board is placed but **NOT routed** — route it in KiCad) plus
  cosmetic silk/lib warnings that KiCad clips at plot.
- **Remaining step = routing.** In KiCad: `Route → Router`, or Freerouting
  (`File → Export → Specctra DSN` → freerouting → import `.ses`). GND is a pour on both layers,
  so route the signal/power nets; press `B` to fill zones. Then plot Gerbers.

## Modules hosted

| Ref | Module | Footprint (authoritative) | Source |
|-----|--------|---------------------------|--------|
| U1  | LoRa-E5-HF (STM32WLE5JC + SX126x) | `LoRa-E5.kicad_mod` (SMD-28, 12×12 mm, 1.25 mm pitch) | Seeed OPL lib |
| U2  | L76K GNSS for XIAO (Quectel L76K) | `XIAO-Add-On.kicad_mod` (2×7 THT, 2.54 mm, rows 17.78 mm) | Seeed OPL lib |

Both footprints are bundled in `footprints/wmesh.pretty/`.

## Net / connection table (the actual design)

LoRa-E5 pad numbers are from the Seeed datasheet (Table 1). L76K pads use XIAO silk names.

| Net | LoRa-E5 (U1) | L76K (U2) | Other |
|-----|--------------|-----------|-------|
| **VDD** (always-on rail) | pad1 VCC | — | Q1 source, R1, R2, U3.OUT (via SB1), C3/C4, J3.3V3, J4.3V3 |
| **GPS_VCC** (gated) | — | 3V3 pad | Q1 drain, C1+C2, U2 STBY(D0) tie |
| **PB4_GATE** | pad7 PB4 | — | Q1 gate, R1→VDD |
| **MCU_LPUART_RX** | pad13 PC0 | D7 (GPS TX) | — |
| **MCU_LPUART_TX** | pad12 PC1 | D6 (GPS RX) | — |
| **UART_TX** (console) | pad10 PB6 | — | J3.TX |
| **UART_RX** (console) | pad9 PB7 | — | J3.RX |
| **SWDIO** | pad3 PA13 | — | J4.SWDIO |
| **SWCLK** | pad4 PA14 | — | J4.SWCLK |
| **NRST** | pad17 RST | — | J4.NRST |
| **BOOT** | pad24 PB13 | — | R2→VDD, J5.1 |
| **RF** | pad15 RFIO | — | J6 u.FL signal |
| **GND** | pads 2,14,16,22 | GND pad | everything, J6 shield |

### GPS gate logic
`PB4 = LOW → Q1 (P-FET) ON → GPS powered`. R1 (100k) pulls the gate to VDD so GPS is **OFF
by default** (reset / Hi-Z). **Firmware: set `GPS_GATE_ON = LOW`** in `wmesh_port_stm32wl.cpp`
(currently HIGH for the shield's STANDBY style). GPS STANDBY (D0) is tied to GPS_VCC = runs
whenever powered. Warm vs cold start depends on the L76K module's onboard RTC backup; either
way the early-exit GNSS firmware minimises on-time.

## Power — READ THIS

VBAT connects **directly to VCC, no regulator** (1.8–3.6 V window of STM32WL/L76K).

- **Field (J1):** 3.6 V **LiSOCl₂** EVE ER26500 (C) **in parallel with EVE HLC1520** hybrid
  cap (off-board, wired to J1). The HLC buffers LoRa-TX / GPS-inrush pulses. SB1 = VBAT-direct.
- **Bench (J2 + U3):** a plain **18650 is 4.2 V > 3.6 V abs-max — NEVER on J1.** Feed it (or
  USB 5 V) to **J2**, through the **HT7333 LDO (U3)** → 3.3 V, and set **SB1 = LDO-out**.
- **SB1 selects exactly one source. Never bridge both.**

## Enclosure (Kradex Z128)

- External 105 × 70 × 40 mm; internal height ≈ 32.9 mm (room for a C-cell / 18650 laid flat).
- **PCB outline: 62.65 × 94.45 mm, corner radius 3 mm** (fits the base cavity + side guides).
- **Mounting holes H1–H4: Ø2.8** for the box's Ø2.5 self-tapping posts (posts are Ø6 OD —
  Ø6 copper keep-out around each). Pattern used: columns 52.65 mm apart, rows 71 mm apart
  (2 of the 6 posts top+bottom), centred.
  **⚠ PROVISIONAL** — read off the Kradex drawing, not yet confirmed to the physical box.
  Verify against your Z128 (or the Kradex STEP) and nudge H1–H4 in KiCad before ordering.
- u.FL (J6) + both antennas exit one end wall; drill the Z128 wall for pigtails/SMA.

## Files

- `footprints/wmesh.pretty/` — the two module footprints (authoritative).
- `BOM.csv` — bill of materials with JLC/LCSC codes (verify codes in JLC parts search).
- `schematic.svg` — human-readable schematic for review.
- `wmesh-node-z128.kicad_pro` / `.kicad_pcb` — KiCad 8 board (open, finish routing, DRC, plot).

## Ordering from JLCPCB

1. Open `wmesh-node-z128.kicad_pcb` in **KiCad 8**. Run **DRC**; finish any ratsnest routing.
2. **File → Plot**: Gerbers (F.Cu, B.Cu, F/B.SilkS, F/B.Mask, F/B.Paste, Edge.Cuts),
   Protel extensions off, then **Generate Drill Files** (Excellon, PTH+NPTH merged).
3. Zip the Gerber/drill folder → upload to JLCPCB. Settings: **2-layer, 1.6 mm, HASL
   (or ENIG), 1 oz Cu**. Z128 PCB is 62.65 × 94.45 mm.
4. SMT assembly (optional): Q1, R1/R2, C1–C6, J1/J2, J6, U3. **U1 (LoRa-E5) and U2 (L76K)
   are hand-soldered modules** — exclude from SMT. Export BOM + CPL (pick-and-place) from KiCad.
5. Hand-solder U1, U2, headers J3/J4/J5. Flash U1 via J4 (SWD). Set `GPS_GATE_ON = LOW`.
