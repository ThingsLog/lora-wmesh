# Open LoRa message payload v5 — full specification

ThingsLog LPMDL-110X LoRa / LoRaWAN data logger, open data protocol.
This document restates "Open LoRa message payload v5" in full and adds the
**position message (port 14)** extension, validated in the W-Mesh reference
implementation. GNSS conventions are aligned with "Communication protocol
v4" (`GNSS_POINTS = 0x68`), so one server-side decoder serves both
transports.

## Transport and general conventions

- Carried as the FRMPayload of standard LoRaWAN uplinks; the **LoRaWAN
  FPort selects the message type**.
- All multi-byte integers are **big-endian** (network order).
- Every message begins with a **version octet = 5**.
- Differential series: readings 1..n-1 are differentials **against the next
  reading**; the last reading is absolute. To reconstruct, start from the
  last (absolute) reading n and subtract the differential of reading n-1 to
  obtain it, and so on backwards.
- Packet size guidance: keep any message under **51 bytes** (the smallest
  EU868 dwell budget). The logger controls this with `every` /
  `counts_threshold`; if a buffer would exceed the limit it is cleared and
  refilled on the every/record_period pattern. One packet per enabled port.

| FPort | Message |
|---|---|
| 11 | readings (pulse or analog) |
| 12 | alarm |
| 13 | battery |
| 14 | position (extension, proposed) |

## Readings message — port 11

Header (5 octets), then the readings:

| octet_index | description |
|---|---|
| 0 | version: 5 |
| 1 | bits(0-3) sensor_index; bits(4-5) reading type: 0 = pulse differential, 1 = analog; bits(6-7) record period: 0 = minutes, 1 = hours, 2 = days, 3 = seconds |
| 2 | every: if record period is 0, the timing between readings is `every` minutes |
| 3 | number of readings n: bits 15-8 |
| 4 | number of readings n: bits 7-0 |
| 5.. | the readings (see per-type layout below) |

### Pulse readings (type 0)

Readings 1..n-1 are 16-bit differentials; the final absolute counter is
**32-bit**:

| octet_index | description |
|---|---|
| 5 .. 6+2(n-1)-2 | reading_i differentials, int16 BE each |
| 6+2n-3 | reading_n bits 31-24 |
| 6+2n-2 | reading_n bits 23-16 |
| 6+2n-1 | reading_n bits 15-8 |
| 6+2n-0 | reading_n bits 7-0 |

Total length: `5 + 2(n-1) + 4` = `7 + 2n` octets.

### Analog readings (type 1)

All values 16-bit; differentials 1..n-1, the last reading absolute:

| octet_index | description |
|---|---|
| 5 + 2i .. | reading_i, int16 BE (differential for i < n-1, absolute for i = n-1) |

Total length: `5 + 2n` octets. Raw ADC values; conversion to mA:
`I[mA] = ((reading_i(15-8) << 8) + reading_i(7-0)) / 50`.

## Alarm message — port 12

| octet_index | description |
|---|---|
| 0 | version: 5 |
| 1 | bits(4-0) sensor index; bits(7-5) reserved |
| 2 | alarm type: 0 = on/off alarm, 1 = no_zero, 2 = analog low level, 3 = analog high level |
| 3 | value: bits 15-8 |
| 4 | value: bits 7-0 |

## Battery message — port 13

| octet_index | description |
|---|---|
| 0 | version: 5 |
| 1 | battery: bits 15-8 |
| 2 | battery: bits 7-0 |

Battery voltage in millivolts.

## Position message — port 14 (extension, proposed)

One averaged position from a GNSS session (glacier stakes, trackers).
Octets 2..13 are **byte-identical to one `GNSS_POINTS` (0x68) point** of
Communication protocol v4, and the status enum is v4's `GNSS_STATUS_LIST`.

| octet_index | description |
|---|---|
| 0 | version: 5 |
| 1 | bits(0-4) source index; bits(5-7) GNSS status: 0 OK, 1 NO_FIX, 2 TIMEOUT, 3 HW_ERR, 4 DATA_INVALID |
| 2..5 | latitude × 1e7, int32 BE |
| 6..9 | longitude × 1e7, int32 BE |
| 10..13 | altitude in millimetres above sea level, int32 BE |
| 14 | number of fixes averaged into this position (saturates at 255) |
| 15 | HDOP × 10 of the session (saturates at 255) |

Sentinels (identical to v4): latitude or longitude `0xFFFFFFFF` → position
invalid (status says why); altitude `0xFFFFFFFF` → altitude unknown,
position still valid. A backend must treat status ≠ 0 as invalid regardless
of the coordinates. Octets 14-15 qualify an **averaged** point (a stake
integrates a multi-minute session into one daily position, pushing
metre-level receiver scatter toward the decimetre regime); they have no v4
counterpart because v4 points are instantaneous.

## Reference implementation

`core/openlora.h` in github.com/ThingsLog/lora-wmesh — encoders and
reference decoders for all four messages, wire-image unit tests
(`test/host_test.cpp`) and a whole-node simulation over the W-Mesh relay
path (`test/node_sim_test.cpp`). Field-validated 2026-07-27..30: battery
and position messages carried over a two-hop W-Mesh (LoRaWAN ABP,
prefix-stripped at tier 1) through a production Dragino gateway into
ChirpStack v3.
