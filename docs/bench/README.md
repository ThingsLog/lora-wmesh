# Bench validation — 27 Jul 2026, Sofia office

Two XIAO nRF52840 + Wio-SX1262 nodes (bare leaves: no GNSS, no HDC3020),
a Dragino LPS8 gateway and a production ChirpStack v3, all in one room.
Topology: leaf 51 (depth 2, ABP) -> relay 21 (depth 1, ABP) -> gateway.
Bench provisioning: phases 60,60 / preroll 10 / acq 5 (no beacon source,
free-run from `set time`).

Validated end to end, window logs in this directory:
- leaf TX with the extended CAD preamble, under the 3-byte mesh prefix;
- relay CAD catch (RSSI -19..-25 dB at bench range), dedup, prefix strip;
- relay's OWN battery uplink under its own ABP identity (forwarder +
  weather-station role);
- both PHYPayloads parsed, MIC-verified and decrypted by ChirpStack —
  device lastSeenAt matching the relay's TX second.

Two defects found and fixed by this bench (both §IV-relevant):
1. Extended preamble was 96 symbols (a unit slip: 197 ms at SF8) — shorter
   than the 1 s CAD scan period, so the relay could never catch a frame.
   560 symbols (1.15 s) caught intermittently — under 100 ms margin against
   RTOS jitter; 800 symbols (1.64 s) catches reliably.
2. The extended preamble was also used on the final hop, where the
   continuously listening gateway concentrator expects the standard
   8-symbol preamble and does not demodulate 1+ s preambles at all.
   Depth-1 transmissions now use a 12-symbol preamble (the paper's T_std).

## 30 Jul 2026 — STAKE profile + Open LoRa position message

Board 1 restacked with the L76K GNSS module (XIAO + Wio-SX1262 + L76K, all
11 header pins in use). Validated across four windows: 80-89-fix averaging
sessions, clock discipline from the fix on both nodes (relay via gnssbk),
the position going out as the FPort-14 message (final form: 16 B, the core
protocol's GNSS point + session quality), per-frame mesh sequence numbers
(a stake's second port frame no longer dies in the relay's dedup), and a
bounded CAD scan replacing RadioLib's unbounded wait (one wedge observed).

OPEN ITEM: the CAD-to-receive handoff is marginal — one window delivered
2/2 catches (RSSI -19), the next 0/2 with 7 CAD hits. Next bench session:
after a CAD hit, retry reception until the listen phase ends instead of
one 3-second attempt per hit.
