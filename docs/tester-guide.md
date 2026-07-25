# Relay node build and test guide

This is the entry point for a tester producing and validating W-Mesh relay
nodes. It walks one board from a sealed bag to an accepted, boxed relay, and
links to the deeper documents instead of repeating them:

- [hardware-build.md](hardware-build.md) — wiring, power, enclosures, BOM
- [build-and-provision.md](build-and-provision.md) — the planner and the enrolment model
- [flash-and-test.md](flash-and-test.md) — toolchains per target
- [protocol-v2-open-relay.md](protocol-v2-open-relay.md) — how the relay serves foreign devices

The field target is the **Wio-E5 / LoRa-E5 (STM32WLE5)** — one die with the
MCU and the SX126x radio, µA-class sleep. The XIAO nRF52840 + Wio-SX1262
stack remains the lab bench target; this guide covers the STM32WL flow.

## 1. What you are building

A relay is a battery node that sleeps almost always, wakes for the daily
window, listens for frames from its children (deeper relays, stakes, leaves),
and forwards them one tier up. It is provisioned once over a serial console;
after that it runs autonomously on a beacon-disciplined clock. A relay works
fine **without** GPS — the GNSS backup clock (`gnssbk`) is an option, not a
requirement.

## 2. Bench inventory

- Wio-E5 mini board (bring-up) or the carrier board from
  `hardware/carrier-z128/` (field build)
- ST-Link programmer (clone is fine — the flash recipe does not need NRST)
  + 4 jumper wires: SWDIO, SWCLK, GND, (RST wire optional, see §6)
- USB-C cable — the mini board's CP2102 gives you the console directly
- Power for bench work: USB, or an 18650 **only through the board's
  5V/VIN LDO input** — a full 18650 is 4.2 V and the direct 3V3 rail has a
  3.6 V absolute maximum. Only a 3.6 V LiSOCl2 cell may drive 3V3 directly
- Multimeter with µA range (sleep-current acceptance)
- A laptop with: PlatformIO (`pio`), OpenOCD, Python 3, a serial terminal
  (115200 8N1), and a clone of this repository

Radio identities (ABP keys, device addresses) and the network-server
credentials live in a `.env` file **outside this repository** — ask the
project lead for it.

## 3. Build the firmware

```bash
cd firmware/stm32wl
pio run -e wio-e5-wmesh
# → .pio/build/wio-e5-wmesh/firmware.bin  (~94 KB)
```

GPS gate polarity is a compile-time choice: `GPS_GATE_ON` must match the
wiring (`HIGH` for the L76K STANDBY pin style, `LOW` for the P-FET VCC gate
of the carrier board). A relay without GPS does not care.

## 4. Flash over SWD

Factory firmware and sleeping W-Mesh firmware both resist a naive
`pio run -t upload`: the board sleeps in STOP2, which gates the debug clock,
and clone ST-Links cannot pulse NRST. The working recipe is scripted in
[tools/flash-stm32wl.tcl](../tools/flash-stm32wl.tcl):

```bash
cd firmware/stm32wl
openocd -f interface/stlink.cfg -f target/stm32wlx.cfg \
  -c "reset_config none separate" \
  -c "set BIN .pio/build/wio-e5-wmesh/firmware.bin" \
  -f ../../tools/flash-stm32wl.tcl
```

While the catcher loop prints, **tap the board's RST button a few times** —
the script catches the core in the brief awake window after reset, performs a
clean soft reset over SWD, writes, verifies, reboots. Success looks like
`>>> verify: ...` with no error followed by `>>> run`.

## 5. First boot and the service console

Connect USB-C and open the terminal at 115200. On reset the banner prints:

```
W-Mesh: any key for console...
```

You have **3 seconds** to press a key; you land in the `service#` prompt.
Inside the console the node never sleeps — take your time. Useful commands:

- `show` — full config as stored (with CRC state)
- `set <key> <value>` / `save` / `boot`
- `bat` — battery millivolts via VREFINT (no divider involved)
- `jrn` — the flash journal: boot count, last event, awake seconds; this is
  your post-mortem tool after a silent night
- `gnss` — run one GNSS session manually (only with GPS wired)

If you miss the 3-second window the node provisions itself from flash and
goes to sleep; tap RST and try again.

## 6. Provision the relay

Never invent values by hand — the planner derives them from the network
description and checks the rules a single node cannot check about itself:

```bash
python3 tools/plan.py examples/livingston.json -o enroll.sh
```

The generated script calls `tools/enroll.py` once per node with its id,
profile, depth, phases, slot, frequency, beacon sub-slots and whitelist, and
sends `set time` **last** (automatically), because everything counts from it.

Rules of thumb for a relay:

- `gnssbk 0` unless a GPS is physically wired and the board has the P-FET
  gate — a relay with `gnssbk 1` and no healthy GPS will burn its budget
- the whitelist must contain **all descendants** of the relay, not only its
  direct children (the firmware filters on the origin id)
- for bring-up a promiscuous whitelist is acceptable; tighten before field

## 7. Bench validation

1. After `boot`, the node prints its config summary and sleeps
2. At the provisioned window time (UTC) it must wake, listen through its
   listening phase and transmit in its slot — watch the serial output, or run
   `tools/bench-console.sh` for a line-buffered capture
3. On the network server, the device identities behind the relay must show
   fresh `lastSeenAt` / incremented `fCntUp` within minutes of the window
4. Next morning, check `jrn`: boot count must not have grown overnight
   (a growing count = reset loop), awake seconds must be window-sized

For a quick cycle, re-provision only `window` and `time` — the rest of the
config survives in flash.

## 8. Power and sleep acceptance

The headline number of the whole design is the sleep current. Measure it
properly or not at all:

- ST-Link disconnected, RST wire removed, USB unplugged
- powered from the cell on the 3V3 rail (3.6 V LiSOCl2 class only)
- multimeter in series, µA range

Target: **≤ 2 µA** for a relay without GPS (hardware-build.md §5.4).
Hundreds of µA means something is awake: GPS module not gated, console
entered and never exited, or a peripheral left on.

## 9. Acceptance checklist, one per node

- [ ] firmware version built from the current repository state
- [ ] flash verified (`>>> verify` clean)
- [ ] console reachable, `show` matches the planner line for this node id
- [ ] `bat` plausible (fresh LiSOCl2 ≈ 3600 mV, USB/LDO ≈ 3300 mV)
- [ ] time set via enrolment, window wake observed once on the bench
- [ ] forwarded frames confirmed on the network server
- [ ] `jrn` clean after an overnight soak (no reset loop, no silent night)
- [ ] sleep current measured and logged
- [ ] node id labelled on the enclosure, row added to the deployment sheet

## 10. Known pitfalls

- **GPS at power-on** — the L76K starts navigating (~25 mA) the moment it
  has power; firmware drives it to standby at boot, but a board that loses
  and regains battery mid-handling can bleed a cell overnight. Battery in →
  provision → verify sleep, in that order
- **GNSS session on real NMEA** — a known crash in the blocking GNSS path is
  under investigation; until it is closed, relays ship with `gnssbk 0` and
  stakes are the only GNSS users
- **USB replug during deep sleep does not enumerate** — tap RST after
  plugging in, then catch the console window
- **18650 on the 3V3 rail destroys the board** — LDO input only
- **Console blocks sleep** — a node left at `service#` will flatline a cell;
  always leave via `boot`
- **Clone ST-Link + sleeping firmware** — use the catcher script; a plain
  upload will report a dead target

## 11. Scaling to a batch

For a production run the loop per board is: flash (§4) → console smoke test
(§5) → provision from the generated enroll script (§6) → one observed window
(§7) → overnight soak → sleep current (§8) → checklist (§9) → box. With two
boards in flight (one soaking, one on the bench) a tester sustains roughly
two accepted relays per day; the soak is the long pole and is deliberate —
the journal after a quiet night is the only test that catches the slow
failure modes.
