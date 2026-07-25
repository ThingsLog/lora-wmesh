# Building and provisioning

**The key fact: there is ONE binary.** The role (leaf / relay / stake) is
not compiled — it is provisioned into the flash page over the boot console.
Build once, flash every module with the same `.bin`, then let
`tools/enroll.py` turn each module into whatever the map says it is.

## 1. Toolchain (either of)

**Path A — STM32CubeIDE (recommended the first time):** GUI, bundles the
compiler, the CubeMX generator and the flasher. Download from st.com (free
registration).

**Path B — command line:** `brew install --cask gcc-arm-embedded` +
`STM32CubeCLT` (for `STM32_Programmer_CLI`) + the STM32CubeWL package
(HAL + SUBGHZ radio driver) from GitHub: `STMicroelectronics/STM32CubeWL`.

## 2. Generating the base project (once)

CubeMX (standalone or inside CubeIDE):

1. Board/MCU: **STM32WLE5JC** (the LoRa-E5 module).
2. Peripherals:
   - **SUBGHZ** (radio, internal SPI) — activate;
   - **RTC** — activate, Alarm A enabled (the alarm clock behind every wake-up);
   - **LPUART1** 115200 8N1 — the provisioning console + GNSS on stakes;
   - **I2C2** (PB15 SCL / PA15 SDA on the LoRa-E5) — the HDC3020 sensor;
   - **GPIO**: 1 output for the GNSS power gate (stake only), 1 bench LED
     (optional);
   - **NVIC**: RTC alarm, SUBGHZ radio IRQ, LPUART RX.
3. Clock: MSI 16 MHz is plenty; LSE 32.768 kHz for the RTC (the crystal is
   on the module).
4. Project → Toolchain: STM32CubeIDE (path A) or Makefile (path B) → Generate.

## 3. Adding the W-Mesh code

1. Copy `core/*.h` and `firmware/stm32wl/src/*` into `Core/Inc` / `Core/Src`
   of the generated project (or add them to the include path).
2. Delete the generated `main.c` — our `main.cpp` takes over (`.cpp` files
   require C++ enabled: Project Properties → C/C++ Build → enable g++;
   no exceptions/RTTI: `-fno-exceptions -fno-rtti`).
3. Write `Core/Src/wmesh_port.cpp` — the only file that touches hardware.
   The port → HAL/driver mapping:

| `port::` function | CubeWL call |
|---|---|
| `windowClockSec` | RTC calendar − window origin (kept in a backup register) |
| `sleepUntil` | RTC Alarm A + `HAL_PWREx_EnterSTOP2Mode` |
| `rtcSetWindowClock`, `rtcSetUtc`, `timeConfigure` | RTC set + backup register |
| `radioInit` | `SUBGRF_Init`, `SetStandby`, `SetPacketType(LORA)`, `SetModulationParams` (SF8/125k data; SF9 beacon), `SetPacketParams` |
| `radioTransmit` | `SUBGRF_SetPayload` + `SetTx`, IRQ TX_DONE |
| `radioCadOnce` | `SetCadParams(CAD_ON_8_SYMB,...)` + `SetCad`, IRQ CAD_DONE/DETECTED |
| `radioReceiveOne` | `SetRx(timeout)`, IRQ RX_DONE/TIMEOUT |
| `radioSleep` | `SUBGRF_SetSleep` (cold) |
| `consoleAttached` | LPUART poll for a byte N ms after reset |
| `uartReadLine` / `uartWrite` | `HAL_UART_Receive` (line, echoed) / `HAL_UART_Transmit` |
| `configLoad` / `configSave` | the last 2 KB flash page (0x0803F800 on the 256K part): `HAL_FLASH_Unlock` → erase page → program doubleword |
| `gnssDisciplineRtc` | (stake only) L76K gate ON → feed LPUART lines to `core/gnss.h` → RTC ← utcDaySec → gate OFF |
| `readSensorPort` | RAM2 ring buffer, filled every 15 min from the HDC3020 over I2C2 (address 0x44, command 0x2400 single-shot LPM0, 6 B reply with CRC-8 — see `hdc3020.h`); port 0 = T raw, port 1 = RH raw, **last port = battery (mV)** |
| `readBatteryMv` | ADC channel VREFINT: vdd_mV = 3000 × VREFINT_CAL / adc_raw — no divider, no external parts (VDD *is* the battery) |

4. Build → Release (`-Os`). The result: one `wmesh.bin` for every role.

## 4. Flashing (all modules, the same bin)

Over SWD (ST-Link, after the one-time RDP unlock — see
[flash-and-test.md](flash-and-test.md) §1):

```
STM32_Programmer_CLI -c port=SWD -w wmesh.bin 0x08000000 -v -rst
```

## 5. The network planner — from topology to configurations

Hand-assigning slots, channels, sub-slots and whitelists across a fleet is
exactly the kind of bookkeeping that produces field bugs. `tools/plan.py`
does all of it from one JSON file describing the topology and the goals:

```
python3 tools/plan.py examples/livingston.json \
    -o examples/livingston-enroll.sh \
    -t examples/livingston-timeline.png
```

You specify: the window anchor, the node list with parent links, and the
margins. The planner derives: depth (from the parent chain), the channel
(one per tier-1 subtree), the whitelist (ALL descendants — the firmware
filters on the origin id), beacon sub-slots, per-tier slot durations sized
to the heaviest node, and the phase durations from the load × margin.
It then validates what no single node can check about itself — fan-in caps
(14 tier-1 / 10 deeper), whitelist capacity, duty-cycle budget, slot-fits-
phase — and emits (1) one `enroll.py` line per node and (2) a **window
timeline**: every node's beacon, listen and TX slot on one Gantt chart:

![window timeline](../examples/livingston-timeline.png)

## 6. Provisioning — examples (by hand)

### Bench network (Stage 1: 1 relay + 2 leaves, H=2)

```bash
# The relay: tier 1, listens to tier 2's phase, forwards beacon + frames
python3 tools/enroll.py /dev/tty.usbserial-A1 \
    --id 21 --profile relay --depth 1 --phases 300,600 --ports 3 \
    --slot 0 --whitelist 31,32 --window 12:05 --boot

# Leaf 31: tier 2, slot 0 — transmits first in phase 1
python3 tools/enroll.py /dev/tty.usbserial-A2 \
    --id 31 --profile leaf --depth 2 --phases 300,600 --ports 3 \
    --slot 0 --window 12:05 --boot

# Leaf 32: tier 2, slot 1 — 12 s after 31, never simultaneous
python3 tools/enroll.py /dev/tty.usbserial-A3 \
    --id 32 --profile leaf --depth 2 --phases 300,600 --ports 3 \
    --slot 1 --window 12:05 --boot
```

The only things differing between the three commands are id / profile /
depth / slot / whitelist. The binary, the phases and the window are shared
by the whole network.

### Livingston examples (per the map, H=3)

```bash
# R1 — the ridge above the base, tier 1; children: relay R4 (id 22) + env leaves 41,42
# --preroll 5 --psub 0: tier-1 relays R1/R2/R3 re-broadcast the beacon in
# sub-slots 0/1/2 of a 5 s pre-roll interval — in sequence, never colliding
# NOTE the whitelist: it filters on the ORIGIN id, so a relay must admit its
# WHOLE SUBTREE — R4's stakes included — not just its direct children.
python3 tools/enroll.py PORT --id 21 --profile relay --depth 1 \
    --phases 200,300,400 --preroll 5 --psub 0 --slot 0 \
    --whitelist 22,41,42,51,52,53,54,55,56 --window 12:05 --boot

# R4 — Kaliakra ridge, tier 2; children: stakes 51..56
# --ppsub 0: listens for R1's beacon copy (R1 sits in sub-slot 0)
# --freq: R4 is in R1's subtree, so the whole branch stays on R1's channel
python3 tools/enroll.py PORT --id 22 --profile relay --depth 2 \
    --phases 200,300,400 --preroll 5 --psub 0 --ppsub 0 --slot 3 \
    --freq 868100000 --whitelist 51,52,53,54,55,56 --window 12:05 --boot

# Glacier stake 51: tier 3, GNSS is the payload
python3 tools/enroll.py PORT --id 51 --profile stake --depth 3 \
    --phases 200,300,400 --slot 0 --ports 2 --window 12:05 --boot
```

**The relay measures too.** `transmitDuty()` transmits the node's own ports
first, then the foreign frames — a relay with an HDC3020 and `--ports 3` is
a forwarder and a ridge weather station at once, at no extra cost. The
phase arithmetic counts exactly that: "own payloads + everything
forwarded".

**The battery is the last port — on every role.** Sampled on the same
15-min tick (VREFINT, microseconds, no external parts) and carried in the
same window: leaf/relay with HDC3020 → ports=3 (T, RH, mV); stake →
ports=2 (position, mV); sensorless relay → ports=1 (mV only — no node is
mute about its own health). The daily mV profile shows the cold sag and the
LiSOCl₂ end-of-life cliff — the early warning that some ridge needs a visit
next summer.

### Slot assignment rules

- **Within one tier every RELAY gets a unique `--psub`** (beacon sub-slot,
  1 s apart inside the tier's pre-roll interval) and every child sets
  `--ppsub` to its beacon parent's sub-slot — this is what keeps beacon
  copies from two audible parents sequential instead of colliding.
- **Within one tier every node gets a unique `--slot`** — that is the
  entire collision protection inside a phase; `save` refuses a slot that
  does not fit (`slot exceeds own phase`).
- The default `--slotdur` of 12 s covers the heaviest node (4 ports ×
  ~2.4 s of airtime + guard); for single-port stakes it can drop to 4–5 s
  and the phase holds three times as many nodes.
- **One tier-1 subtree = one channel (`--freq`).** W-Mesh does not
  channel-hop: a relay has a single demodulator, so it must know exactly
  where to listen. The whole subtree of a tier-1 relay shares one fixed
  channel (children transmit there, the relay listens and forwards there);
  parallel subtrees go on different channels (R1: 868.1, R2: 868.3,
  R3: 868.5 MHz, ...), and the gateway's concentrator hears all 8 at once.
  ETSI does not require hopping — the duty budget is per sub-band.
- `--window` is **the beacon time** (observed once after the gateway's
  boot), identical across the network — see README, "The window anchor".
- **Stakes never go silent for missed beacons**: the daily GNSS session
  supplies absolute UTC on its own, so a successful session re-arms the
  schedule exactly as a caught beacon would. The `freerun` silence rule
  bites only leaves and relays — the roles whose sole time source is the
  beacon.
- **`--gnssbk 1` (relay/leaf option): GNSS as a backup clock.** Fit the
  stake's L76K circuit (+8 €), powered ONLY on a missed beacon: a fix costs
  ~0.25 mAh once, re-arms the schedule, and a relay then SYNTHESIZES the
  day's beacon for its children (time = last caught beacon + 86400 s per
  elapsed window + hop delta; the frame is MIC-free, so a local copy is
  indistinguishable from a relayed one). One upstream miss no longer
  silences a subtree. Recommended for all ridge relays.

## 7. Verifying after provisioning

```bash
python3 tools/enroll.py PORT --show
```

plus the bench sequence of [flash-and-test.md](flash-and-test.md) §4 and
§7a (smoke test + beacon sync + the negative test with a stopped "gateway").
