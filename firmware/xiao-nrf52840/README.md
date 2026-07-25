# W-Mesh bench node — XIAO nRF52840 + Wio-SX1262

The lab bring-up target for the W-Mesh core: Seeed XIAO nRF52840 with the
Wio-SX1262 LoRa module (kit SKU 102010710). Bare-leaf hardware — no GNSS, no
HDC3020 — so the node's only measurement is its own battery voltage
(`ports 1`, the battery-is-the-last-port rule). The mesh behaviour is
`core/wmesh_node.h`, identical to the field target; only the port layer
(`src/wmesh_port_xiao.cpp`) is board-specific.

Bench-grade by design: millis-derived window clock, FreeRTOS delay sleep,
USB CDC console. Power discipline (STOP2, RTC alarms) belongs to the
STM32WL field target.

## Build and flash

```
pio run                      # firmware.zip (DFU package)
pio run -t upload            # serial DFU; 1200 bps touch resets a RUNNING app
```

If the board is dark (no USB port): double-tap the reset button — it
re-enumerates as the `XIAO_BOOT` bootloader and `pio run -t upload
--upload-port /dev/cu.usbmodemXXX` flashes it directly.

`pio run -e xiao-hello -t upload` flashes a LED + CDC counter instead —
the toolchain sanity check with no W-Mesh code at all.

## Hardware map (kit / standalone Wio-SX1262 for XIAO)

```
SX1262: CS=D4  DIO1=D1  BUSY=D3  RESET=D2  RXEN=D5  (TX via DIO2, TCXO 1.8 V)
SPI:    SCK=D8  MISO=D9  MOSI=D10
VBAT:   P0.31 through the on-board 1M/510k divider, gated by P0.14
```

## Provision and test

```
python3 ../../tools/hiltest.py /dev/cu.usbmodemXXX --id 51 --reset-check
python3 ../../tools/enroll.py  /dev/cu.usbmodemXXX \
    --profile leaf --ports 1 --phases 900 --depth 1 --slot 2 --boot
```

`hiltest.py` is the automated console smoke test (round-trips, validation
rejects, flash save, persistence); `enroll.py` is the real provisioning
flow — `set time` is sent automatically last. Reset the node right after
starting either script; the boot console waits for the host at power-up.

## Monitoring received packets

Two complementary views, both on the USB console (115200):

- **The mesh node traces itself**: every `RX` (with RSSI/SNR) and `TX` frame
  is printed with its window time and a hex dump, plus `ANCHOR` on a caught
  beacon and `SLEEP ->` lines between duties. Open any serial monitor on the
  node's port and watch the window unfold.
- **A dedicated sniffer** (`pio run -e xiao-sniffer -t upload`): continuous
  RX that prints EVERY frame it hears — hex, RSSI/SNR, and the decoded mesh
  header (origin/seq/depth/ttl) or Class B beacon time. Keys: `b` hops to
  the beacon channel (869.525 MHz), `d` back to the branch channel. Flash it
  on one bench board to watch the other two talk.

For the whole bench at once: `tools/bench-console.sh` opens one GNU screen
session with a stacked region per connected board (every `/dev/cu.usbmodem*`),
titled by port. `Ctrl-A Tab` hops between boards, `Ctrl-A H` logs the focused
one to a file, `Ctrl-A \` quits. Free a port (kill its window) before
flashing that board.

## Toolchain pitfalls (learned the hard way)

- The factory bootloader carries SoftDevice S140 **7.3.0** — the app must be
  linked with `nrf52840_s140_v7.ld`. Both come from the Seeed fork of the
  Adafruit core pinned in `platformio.ini`; the stock PlatformIO framework
  package is v6-only and produces a non-booting app.
- `lib_archive = no` is REQUIRED: the core calls `TinyUSB_Device_Init` only
  through a weak reference, and a weak reference never extracts the strong
  definition from an archived library — the app links, runs, and stays
  invisible on USB.

## Power/cable procedure (learned the hard way)

The bench clock lives in RAM: flash keeps the config, keys and FCnt, but a
power gap loses TIME, and for a windowed network time is everything.

- **Battery FIRST, then provision, then unplug USB** — never leave a node
  without both. A node that went dark needs a 30-second re-sync (reconnect,
  RESET, `enroll.py PORT --window HH:MM --boot`).
- A sleeping node does not re-enumerate on USB replug (known bug — wake on
  USBDETECTED is on the fix list): press RESET once after plugging in.
