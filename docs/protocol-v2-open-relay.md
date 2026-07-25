# W-Mesh v2 — the open relay

Stage 1 (v1) assumed that every end node runs W-Mesh firmware: the node inserts
the mesh header itself, holds a provisioned slot, and listens to the cascaded
beacon. That made the relay service available only to our own leaves.

v2 opens the relay service to end devices that do not run W-Mesh firmware.
The design rule behind every change: **the relay adapts, the device does not.**
A relay accepts three classes of traffic, from cheapest to most capable:

| Mode | Device requirement | Hardware on relay | Status |
|---|---|---|---|
| **Native** | W-Mesh leaf firmware (v1, unchanged) | current | implemented |
| **Adopted** | plain LoRaWAN, static contract | current | this spec |
| **Negotiated** | WOR-capable stack (TS011-style knock) | current | this spec |

Everything below is uplink-only. See [Non-goals](#non-goals) for why that is a
physical boundary, not a roadmap item.

## 1. What stays untouched

- The LoRaWAN frame is an opaque payload: the MIC is never modified, keys never
  leave the device, the network server validates end to end as if the device
  had hit a gateway directly
- The cascade itself: beacon time discipline, deepest-first phases, gradient
  rule, TTL, duty budgets
- v1 leaves: a frame that arrives already wrapped is forwarded exactly as
  today; the header version bit separates the paths

## 2. Header v2

v1 header (5 B): `[ver|flags] [origin:2] [seq] [ttl|depth]`

v2 keeps the layout and defines two flag bits:

```
bit 7  VER      0 = v1, 1 = v2
bit 6  FOREIGN  frame was wrapped by a relay, not by the device
bit 5  CLASS    0 = native, 1 = adopted/negotiated (admission class)
```

When `FOREIGN` is set the header is followed by the 4-byte **DevAddr** copied
from the wrapped frame's FHDR, and the `origin` field carries a CRC16 of the
DevAddr (fast filter key). Cost: +4 B on adopted frames only, ≈ 2% of the
extended-preamble airtime of a forwarded hop.

### Identity and deduplication

Foreign frames are deduplicated on **(DevAddr, FCnt)** — both are cleartext in
the LoRaWAN FHDR, so no key material is needed anywhere in the mesh. Native
frames keep (origin, seq) as in v1.

## 3. Ingress rules

The first relay that hears an unwrapped LoRaWAN frame on its subtree channel
**wraps it** (inserts the v2 header with `FOREIGN|CLASS`) and queues it for its
own transmit slot. Wrapping happens once; every further hop sees a wrapped
frame.

- During the window the ingress leg runs **continuous RX**, so standard
  8-symbol preambles are received; the extended preamble requirement applies
  only relay-to-relay
- The relay never forwards while a burst is being received — the phase
  discipline already separates listen and transmit, no new mechanism
- Store-and-forward: a foreign burst is absorbed whole, then drained in the
  relay's slot

### Admission control

- Whitelist on **DevAddr ranges / NetID prefixes** (the v1 origin whitelist
  generalised); frames outside it are dropped at ingress
- Native frames drain first; adopted frames fill the remaining duty budget
- Per-ingress quota per window, drop-oldest on overflow — foreign traffic can
  never starve the cascade's own telemetry

## 4. Adopted mode — the static contract

A plain LoRaWAN device is served if its integrator configures it to:

1. transmit on the **subtree channel** (one fixed frequency)
2. at a **fixed SF** (the receiver demodulates one SF at a time)
3. **ABP, unconfirmed** uplinks
4. at a **scheduled start time** inside the window — the start time is the
   device's slot

The reference adopted device is our own production logger: fixed-size packets,
X packets back-to-back (one per port), daily schedule. No firmware change —
only configuration.

### Slot sizing for an X-packet burst

A burst of X packets sent back-to-back occupies one contiguous slot:

```
slot = X * (T_frame + t_gap) + guard
```

with `guard` sized to the device's clock drift between time syncs (adopted
devices do not hear the beacon — budget tens of seconds for a plain RTC). The
window is not the binding resource; the upstream duty budget is:

```
devices per relay ≈ duty_budget / (X * T_ext)
```

| X packets | 1% duty (36 s/h) | polite access (~100 s/h) |
|---|---|---|
| 2 | ~22 | ~60 |
| 4 | ~11 | ~31 |
| 8 | ~5 | ~15 |

(T_ext ≈ 0.8 s, SF8.) These are consistent with the v1 fan-in caps, which were
computed on a multi-port basis; the planner only needs a `ports`/`adopted`
field per foreign device.

### Window announcement

The relayed Class B beacon carries the subtree's `(channel, SF, window
offset)` hint in the 7-byte gateway-specific field. An integrator can read the
schedule off the air; a Class B capable device can even self-synchronise from
the cascaded beacon, which is standard-format by construction.

## 5. Negotiated mode — control channel and appointments

The static contract requires an integrator to pre-configure four values. The
negotiated mode replaces them with one over-the-air handshake, split across
two channels:

- a **control channel** — one well-known frequency per network; relays run a
  slow CAD scan on it around the clock
- per-appointment **data channels** — assigned by the relay in its reply

```
device                       relay
  |--- knock (WOR frame) ----->|   control channel, long preamble
  |<-- appointment ------------|   channel X, slot Y, SF Z [, defer to window]
  |                            |
  |--- data burst ------------>|   data channel, at the appointed slot
```

Properties:

- **One radio serves many channels** — the relay walks between appointments
  instead of listening to the whole spectrum; spectrum coverage by
  time-multiplexing, no concentrator
- **Anytime knocking at μA cost** — the WOR preamble is long by design, so the
  control-channel CAD cycle can be slow (hundreds of ms); the data still
  travels in the window, only the agreement is made anytime
- **AFA by construction** — the data channel is assigned dynamically, which is
  the frequency-agility half of ETSI polite spectrum access
- **Over-the-air enrolment** — the appointment carries exactly the values that
  v1 provisions over the serial console; a field device knocks and is seated

The knock frame format follows the TS011 WOR frame so that devices with a
standard relay-capable stack (LoRa Basics Modem, LoRaMAC-node ≥ relay Tx
support) can knock with factory firmware. Devices without a WOR-capable stack
use the adopted static contract instead; the two modes coexist on one relay.

### Half-duplex note

While the relay receives on a data channel the control channel is deaf. The
knock procedure is retry-based (as WOR is), appointments are short bursts, and
knocks are rare (once per device per enrolment, then only on change), so the
deaf intervals are statistically irrelevant at the fleet sizes the duty budget
allows anyway.

## 6. Transmit path — phase, slot, CAD gate

Every transmitter in the cascade follows one ordered rule:

```
phase → slot → CAD gate → transmit
```

1. **Phase** — deepest-first, separates tiers in time; this is what handles
   hidden terminals across the terrain breaks the mesh exists to bridge
2. **Slot** — the reservation inside the phase (native: beacon-anchored;
   adopted: scheduled start; negotiated: appointment)
3. **CAD gate (LBT)** — immediately before transmitting, one CAD check; busy →
   random backoff of 50–200 ms within the slot's slack, retry; slot exhausted
   → the frames stay queued for the next window (the data is day-buffered:
   delay, not loss; never spill into a neighbour's slot)

The CAD gate reuses the CAD machinery the relay already has. It is mandatory
on the relay-to-relay forward path — the frames there aggregate a whole
subtree and are the most expensive to lose — and recommended on native leaves.
Vanilla adopted devices cannot be made to listen before talking; their
protection is the schedule, which is why slots remain the primary mechanism.

LBT senses at the transmitter, not at the receiver: it cannot see what the
receiving relay hears. It absorbs late neighbours and foreign traffic; it does
not replace the phase/slot structure.

## 7. Regulatory profiles

- **Baseline: 1% duty cycle** — v1 numbers, fan-in caps 14/10, no claims
  beyond EN 300 220 duty limits
- **Option: polite spectrum access (LBT + AFA)** — the CAD gate provides LBT;
  the negotiated mode's dynamic data channels (or a provisioned
  primary/alternate channel pair per subtree, `freqAlt` in the config image)
  provide AFA. If the implementation is confirmed against EN 300 220-1, the
  hourly budget rises from 36 s to ~100 s — roughly ×3 on every capacity
  figure in this document

The compliance reading of EN 300 220-1 (listen time, detection threshold, AFA
behaviour) is an **open item**; no polite-access figure should be claimed in
planning or publications until it is closed.

## 8. Relation to the LoRaWAN TS011 Relay

TS011 (LoRa Alliance, 2022) standardises a battery relay with a WOR
wake-channel and a single hop — a relay cannot sit behind another relay.
W-Mesh v2 is designed as a superset, not an alternative:

| Building block | TS011 | W-Mesh v2 |
|---|---|---|
| Device wake | WOR frame, well-known channel | reused as the negotiated-mode knock |
| Device↔relay leg | standardised, incl OTAA and downlink via RXR | adoptable as-is for devices adjacent to a relay |
| Relay↔relay transport | forbidden (single hop) | the W-Mesh cascade — our contribution |
| Time discipline | always-on CAD | windowed, Class B beacon anchored |
| Devices served per relay | small provisioned table | duty-budget bound (see §4) |

The long-term direction: speak TS011 on the device-facing edge (full standard
service one hop deep, including OTAA), keep W-Mesh as the inter-relay
transport the standard does not have. The device and the network server see
standard LoRaWAN at both ends; the cascade in the middle is invisible to them.

## Non-goals

- **Downlink deeper than one hop** — a Class A RX1 window opens 1 s after the
  uplink; a store-and-forward cascade with phased duty cycling cannot deliver
  into it. Deep nodes are uplink-only by physics
- **OTAA through the cascade** — join-request forwards like any frame, but the
  join-accept cannot return in time; adopted and negotiated devices are ABP
- **Touching the MIC or any key material** — the mesh is a transparent
  carrier; end-to-end security belongs to LoRaWAN
