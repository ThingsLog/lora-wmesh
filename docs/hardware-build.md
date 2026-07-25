# Building a node — end device and relay

The hardware of both roles is **identical** (and so is the firmware) — a
relay differs only in provisioning and placement: a ridge with visibility,
a higher antenna. The stake adds only the GNSS module.

## 1. Bill of materials (per node)

| # | Component | Note | ~Price |
|---|---|---|---|
| 1 | Wio-E5 mini dev board (SKU 113990939) | STM32WLE5JC — MCU + SX126x-class radio on one die; USB-C console, SWD header | ~20 € (bare Wio-E5 module ~7 € for a custom PCB run) |
| 2 | HDC3020 breakout | T + RH, I²C, ~100 nA sleep | ~5 € |
| 3 | LiSOCl₂ D-cell 3.6 V (e.g. SAFT LS33600, 17–19 Ah) + holder | NOT alkaline — the cold kills it; 70% derating in the budgets | ~15 € |
| 4 | 868 MHz whip antenna 2–3 dBi + IPEX→SMA pigtail | for a relay: a collinear / higher gain is an allowed luxury | ~5–10 € |
| 5 | IP67 enclosure (e.g. 100×68×50) + 2 cable glands | one for the antenna, one blind / for the sensor | ~8 € |
| 6 | Silica gel pack + AWG24 silicone wire | cold-proof insulation | ~2 € |
| — | *(stake; recommended on relays as backup clock, `--gnssbk 1`)* L76K GNSS breakout + passive antenna | + power-gate MOSFET (e.g. AO3401 + 100k); on relays powered only on a missed beacon | ~8 € |
| — | *(tools, one-time)* ST-Link V2 + USB-UART (3.3 V!) | flashing + provisioning | ~10 € |

Total per node: **~40–45 €** (leaf/relay), ~55 € for a stake.

## 2. Wiring

```
LoRa-E5 mini          HDC3020 breakout
  3V3  ────────────────  VDD
  GND  ────────────────  GND
  PB15 ────────────────  SCL      (I2C2)
  PA15 ────────────────  SDA      (I2C2)
                         ADDR → GND (address 0x44)

LoRa-E5 mini          USB-UART (provisioning only, not built in)
  PC1 (LPUART1 TX) ───  RX
  PC0 (LPUART1 RX) ───  TX
  GND  ────────────────  GND

LoRa-E5 mini          ST-Link (flashing only)
  SWDIO/SWCLK/GND/3V3 — standard SWD

Battery: LiSOCl₂ (+) → VBAT/3V3 input, (–) → GND. No regulator — the module
runs 1.8–3.6 V; LiSOCl₂ sits at 3.6 V nominal and declines gently.

(stake only) L76K: VCC ← MOSFET power gate from a GPIO (PB4 suggested),
TX → PC0 (shares LPUART1 with the console — the console lives only at
boot, GNSS only during a fix; they never overlap in time).
```

Notes:
- **The battery is measured with nothing extra**: the ADC's internal
  VREFINT channel reads VDD (= the battery, there is no regulator) — do
  NOT add a divider to an ADC pin; it would only leak current. The battery
  is the last OpenLoRa port on every role (see build-and-provision.md).
- I²C pull-ups: most HDC3020 breakouts carry 10k on board — check; if not,
  4.7k to 3V3 on SDA and SCL.
- Keep the I²C cable short (<10 cm) — the sensor belongs near the enclosure
  wall or in a small external "hat" with a breathable membrane, otherwise
  you measure the temperature of the box, not the air.
- Never bend the antenna pigtail below a 10 mm radius; the antenna exits
  through the gland pointing straight up.

## 2a. GNSS power management (stake only) — the policy

The L76K is NOT low-power at our scale: acquisition ~25 mA, tracking
~22 mA, standby ~300 µA (still 300× our sleep), backup ~10 µA. Therefore:

- **Hard power gate (P-FET), zero µA when off.** No module standby modes —
  even its most frugal mode is unacceptable as a constant load.
- **No backup supply.** Backup (~10 µA = 0.24 mAh/day) would buy a fast
  fix, but the 15-minute averaging session swallows the ~30 s cold start
  (3% overhead) — 0.2 mAh once beats 0.24 mAh every day.
- **The session runs on schedule:** the RTC alarm starts it outside the
  radio phases, 1 fix/s for ~15 min, averaging (metre-level noise → the
  decimetre regime), the position becomes the payload, the gate drops.
- **Stake budget:** ~5–6 mAh/day, 99% of it GNSS → ~7 years on a LiSOCl₂
  D-cell at 70% derating. The radio is noise on this background.

Leaves and relays need no GNSS for normal operation — their time comes
from the beacon. Ridge relays are RECOMMENDED to carry the same gated
circuit as a backup clock (`--gnssbk 1`): powered only on a missed beacon
(~0.25 mAh once), it re-arms the schedule and lets the relay synthesize
the day's beacon for its children — one upstream miss no longer silences
a subtree.

## 3. Flashing (once per module)

1. RDP unlock of the factory LoRa-E5 (one-time, erases the factory AT
   firmware — [flash-and-test.md](flash-and-test.md) §1, the BB→AA
   warning).
2. `STM32_Programmer_CLI -c port=SWD -w wmesh.bin 0x08000000 -v -rst`
   — the single binary from [build-and-provision.md](build-and-provision.md).

## 4. Provisioning (what makes a node a "leaf" or a "relay")

Connect the USB-UART, run `tools/enroll.py`, reset. Examples and slot
rules: [build-and-provision.md](build-and-provision.md) §5. The two
commands that make the DIFFERENCE between the roles:

```
--profile leaf  --depth 2 --slot N                   # end device
--profile relay --depth 1 --slot N --whitelist ...   # relay (+ own data!)
```

A relay with an HDC3020 and `--ports 3` also sends its own T/RH/battery —
it is a forwarder and a weather station at once. The battery (mV) is the
last port on EVERY role — no node is mute about its own health.

## 5. Pre-sealing check (10 minutes)

1. `tools/enroll.py PORT --show` — the configuration matches the plan.
2. Bench beacon test ([flash-and-test.md](flash-and-test.md) §7a): the node
   catches a beacon from the test "gateway" and (if a relay) re-broadcasts it.
3. Sensor read: a `BCN OK` console line + the first measurement in the log.
4. Consumption on a PPK2 if at hand: sleep ≤ 2 µA — anything more means
   something is not sleeping (usually a forgotten debug pin or a UART that
   was never de-initialised).

## 6. Sealing and installation

1. Silica gel inside, battery mechanically fixed (cold + wind = vibration).
2. Close the enclosure checking the gasket; tighten the glands.
3. **Leaf:** 1.5–2 m above expected snow, antenna vertical, roughly towards
   the parent (not critical with LoRa, but not behind metal/rock).
4. **Relay:** on the ridge per the map
   ([livingston-plan.png](livingston-plan.png)), as high as possible; check
   with RSSI at installation: it must hear both the parent and its farthest
   child with ≥10 dB margin above sensitivity.
5. **Stake:** GNSS antenna horizontal, facing up, away from the metal of
   the stake; the enclosure below the antenna.
6. Log: id, role, slot, coordinates, RSSI to the parent, a photo.

## 7. If something does not come up in the field

- A node silent the next day → it missed the beacon: wait one more day
  (acquisition mode listens 6 min and self-recovers) before opening any box.
- The relay hears its children but the gateway does not hear the relay →
  relay antenna/orientation, or the margin to the base was optimism; move
  20–50 m along the ridge.
- Everything silent → check the gateway and the beacon first (SDR/test
  module on 869.525 MHz at 12:0x UTC), only then the nodes.
