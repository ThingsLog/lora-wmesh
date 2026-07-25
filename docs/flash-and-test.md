# W-Mesh Stage 1 — flashing and testing (STM32WLE5 / Wio-E5)

The path from zero to a working pair of nodes, step by step.

## 0. What you need

| # | Item | Note |
|---|------|------|
| 1 | 2–3× Wio-E5 mini (or LoRa-E5 dev board) | STM32WLE5JC, ~10–15 € each |
| 2 | ST-LINK V2/V3 (SWD programmer) | clones work fine |
| 3 | STM32CubeIDE + STM32CubeProgrammer | free, st.com |
| 4 | STM32CubeWL package | contains the SUBGHZ radio driver (`SUBGRF_*`) |
| 5 | USB-UART cable | for the bench console (LPUART1) |
| 6 | Nordic PPK2 | for the energy measurements (step 6) |
| 7 | L76K GNSS breakout | STAKE profile only (step 7b) |

## 1. ⚠️ Unlocking the module (one-time, irreversible)

Factory LoRa-E5 modules ship with an AT firmware and **read-out protection
(RDP level 1)**. The first flash requires lifting the protection, which
**erases the AT firmware forever** — after that the module is yours, but
there is no way back.

1. Wire SWD: `DIO→SWDIO, CLK→SWCLK, GND→GND, 3V3→3V3` (the pads are
   labelled on the back of the Wio-E5 mini).
2. STM32CubeProgrammer → ST-LINK → **Connect** (mode: *Under reset*,
   reset: *Hardware reset*).
3. Tab **OB** (Option Bytes) → *Read Out Protection*: change `BB` to `AA`
   → **Apply**. The chip mass-erases and unlocks.

## 2. Generating the project (CubeMX/CubeIDE)

1. New STM32 project → part **STM32WLE5JC**.
2. Peripherals:
   - **SUBGHZ** (the radio; Activate)
   - **RTC** (Alarm A — the alarm clock behind every wake-up; LSE if the
     board has the crystal)
   - **LPUART1** (115200) — GNSS + bench console
   - **GPIO**: 1 output for the GNSS power gate (MOSFET), 1 for a bench LED
3. Clock: MSI 4 MHz + LSE for the RTC (lowest STOP2 current).
4. Middleware: copy the `SubGHz_Phy` driver from CubeWL (RadioDriver:
   `radio.c/.h`, `radio_driver.c/.h` = the SUBGRF API).
5. Generate → then add to the project:
   - `core/wmesh_core.h`, `core/openlora.h` (include path to `core/`)
   - `firmware/stm32wl/src/main.cpp` (replaces the generated main)
   - implement `core/wmesh_port.h` on top of the generated drivers as
     `firmware/stm32wl/src/wmesh_port.cpp` (the SUBGRF_* mapping is
     commented in the header).
6. Project settings: C++17 (`-std=gnu++17`), `-Os`, LTO optional.

## 3. Flashing

- From CubeIDE: **Run → Debug/Flash** over ST-LINK, or
- from the command line:
  ```
  STM32_Programmer_CLI -c port=SWD -w build/wmesh.elf -v -rst
  ```

## 4. Bench smoke test (2 modules, 30 minutes)

Build the **bench variant** (`-DWMESH_BENCH`): LPUART console enabled,
shortened phases (e.g. PHASE_DUR = {30, 60} s), a window every 5 minutes.

1. **Module A** = leaf, `depth 2`; **module B** = relay, `depth 1`.
2. Open both consoles (115200 8N1).
3. Expected on B during its listening phase:
   `CAD hit → RX 201 B → gradient OK (depth 2>1) → queued (origin=21 seq=0)`
4. Expected on B during its TX phase: its own frames plus the forwarded
   one, `restamp depth=1 ttl=14`.
5. Negative tests: run A with `depth 1` (equal depth) — B must reject:
   `gradient REJECT`. Send a frame twice — `dup suppressed`.

## 5. Monitoring the air (3rd module or SDR)

A third module with a simple RX-dump firmware (or an SDR with a LoRa
decoder) on the same frequency/SF gives an independent view: you will see
the extended preamble, the leaf's frames, and the re-broadcast with the new
depth. For Stage 1 the gateway IS this monitor: raw frames are dumped over
UART to a laptop acting as the base station.

## 6. Energy validation (PPK2)

1. PPK2 in **Source Meter** mode (3.6 V) in place of the module's battery.
2. Profiles to capture:
   - **Sleep floor**: between windows; the target is ~1–2 µA (STOP2 + RTC).
     Hundreds of µA mean something (LPUART, SPI, a GPIO pull) is still
     alive; the bench build is ALWAYS hungrier — only measure the field
     build.
   - **CAD profile**: a train of 2.3 ms peaks @ ~6 mA once per second
     during the listening phase.
   - **TX burst**: 0.594 s @ ~45 mA per frame.
3. Check the 24 h integral against the energy model:
   leaf ≈ 0.03 mAh, relay with 1 child ≈ 0.067 mAh (+GNSS if active).

## 7. Time discipline tests

### 7a. Beacon sync (RELAY/LEAF profiles — the normal path)

1. The third module plays gateway: it transmits the Class B beacon (17 B,
   SF9, 869.525 MHz) — the standard cadence is **every 128 s**; in the
   15-minute window that is ~7 beacons, and the W-Mesh anchor is the first
   one (t=0 of the window). For bench purposes one beacon at start is
   enough (see `wmesh_beacon.h`: `restampBeacon()` builds a valid frame).
2. The node under test: wakes `BEACON_GUARD_S` early, catches the frame,
   `parseBeacon()` → valid, `rtcSetWindowClock()` re-anchors the window
   clock. Expected on the console: `BCN OK t=...`.
3. RELAY: verify the re-broadcast in its own pre-roll slot
   (+`PREROLL_SLOT` s) with the re-stamped time — the SDR/third module must
   see a second valid beacon with time += PREROLL_SLOT and the GwSpecific
   part untouched.
4. Negative test: stop the "gateway" → the node counts `missedBeacons`,
   widens the guard (+`DRIFT_S_PER_DAY` s/day), and after
   `MAX_FREERUN_DAYS` stays silent.

### 7b. GNSS discipline (STAKE profile only)

1. L76K on LPUART1 behind the power-gate GPIO.
2. Run `gnssDisciplineRtc()` once: the gate goes up, NMEA is parsed to a
   valid fix, the RTC is disciplined, the gate drops. Expected: < 60 s cold
   start under open sky, ~0.25 mAh of charge.
3. Check: two consecutive disciplines 24 h apart → the reported drift at
   room temperature must be under ±2 s (in the cold expect up to ~9 s/day
   without temperature compensation — the tuning-fork crystal's parabolic
   temperature curve).

## 8. Field dry-run (24 hours)

The real configuration (PHASE_DUR per the network plan, one window per
day), field build without the console, PPK2 on the relay. The success
criteria: per-tier delivery ratio, duplicate rate, CAD
missed-detection rate, and charge-per-role within the model.

## Common problems

- **No SWD connection**: hold RESET during Connect (*Under reset*).
- **RDP comes back**: some batches need a power cycle after OB Apply.
- **CAD misses frames**: the transmitter's preamble must be LONGER than the
  CAD period (PREAMBLE_SYMS=96 at a 1 s period/SF8: 96×2.048 ms ≈ 197 ms —
  raise the CAD rate or the preamble if you change the SF).
- **High sleep current**: disable debug (`DBGMCU`), set unused GPIOs to
  analog, stop the LPUART clock outside its duty.

## Planning map

The schematic island deployment plan (2 gateways at the BAI base, 3 tier-1
+ 5 tier-2 relays, ~36 GNSS stakes across the glaciers) is in
`livingston-plan.png` in this folder. A planning sketch, not a survey
product — final positions come from the Stage-1 GNSS+RSSI site survey.
