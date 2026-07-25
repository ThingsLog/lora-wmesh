// W-Mesh node state machine — portable, shared by every firmware target.
// State machine (paper §VI): BOOT -> [console? enrolment dialogue]
//                                 -> SLEEP -> [beacon RX (pre-roll), re-anchor]
//                                 -> [relay: re-broadcast beacon in own slot]
//                                 -> [listen phase: CAD scan/RX/store]
//                                 -> [own phase, own slot: TX own, forward]
//                                 -> SLEEP until next window.
//
// Provisioning: everything about WHEN this node wakes lives in wmesh::Config,
// stored in a flash page and edited over the boot console (press any key on
// the console UART within 3 s of reset; see wmesh_console.h for the dialogue).
// The critical field is cfg.windowUtcSec — the first wake-up after deployment
// is blind (no beacon heard yet) and runs purely on the enrolment-set RTC.
// Livingston network: window 12:00 UTC (15:00 Bulgarian summer time).
//
// This file is deliberately free of vendor calls: everything hardware
// touches goes through port::* (see wmesh_port.h). A firmware target is a
// port implementation plus a two-line main that calls wmesh::nodeMain();
// the host test suite drives the same code through a mock port instead.
// nodeWindowOnce() runs exactly one window, so a simulation can step days.
//
// SPDX-License-Identifier: MIT
#pragma once
#include "wmesh_core.h"
#include "wmesh_beacon.h"
#include "wmesh_config.h"
#include "wmesh_console.h"
#include "openlora.h"
#include "lorawan.h"
#include "gnss.h"
#include "wmesh_port.h"

namespace wmesh {

// The uplink preamble must OUTLAST the listener's CAD scan period, or a
// frame can start and finish between two scans. At SF8/125 kHz a symbol is
// 2.048 ms and the CAD tick is nominally 1 s — but the tick carries jitter
// (RTOS scheduling, the scan itself), so the margin must absorb it: bench
// measurement showed 560 syms (1.15 s, <100 ms margin) misses one window in
// two, while 800 syms (1.64 s, ~600 ms margin) catches reliably. This is
// the paper's T_ext, priced with real jitter rather than nominal numbers.
// (The original 96 was a unit slip: 197 ms at SF8, >1 s only at SF11+.)
// Bench margin update: 800 syms (1.64 s) still produced CAD hits with
// missed receive locks; 1200 syms (2.46 s at SF8) lets 2+ CAD scans land in
// every preamble and leaves a full receive-retry inside the transmission.
constexpr uint16_t NODE_PREAMBLE_SYMS = 1200;

// Everything mutable the state machine carries across windows.
struct NodeRuntime {
  Config       cfg{};
  lorawan::AbpKeys abp{};                  // devAddr == 0 -> raw OpenLoRa mode
  DedupCache   dedup;
  ForwardQueue fwdq;
  uint8_t      mySeq             = 0;
  uint32_t     missedBeacons     = 0;
  bool         acquired          = false;  // ever caught a beacon?
  uint32_t     lastBeaconGps     = 0;      // time field of the last catch
  uint32_t     windowsSinceBeacon = 0;
};

inline bool nodeOriginAllowed(const Config& cfg, uint8_t origin) {
  if (cfg.whitelistLen == 0) return true;   // promiscuous / dual-parenting
  for (uint8_t i = 0; i < cfg.whitelistLen; ++i)
    if (cfg.whitelist[i] == origin) return true;
  return false;
}

// ---- enrolment console (boot-time only) --------------------------------------
inline void nodeEnrolmentConsole(NodeRuntime& rt) {
  char line[160], reply[512];
  port::uartWrite("W-Mesh enrolment console — 'help' for commands\n");
  for (;;) {
    port::uartWrite("service# ");
    if (port::uartReadLine(line, sizeof line) <= 0) continue;
    if (!std::strcmp(line, "bat")) {           // bench diagnostic: live battery
      std::snprintf(reply, sizeof reply, "battery %u mV",
                    (unsigned)port::readBatteryMv());
      port::uartWrite(reply);
      port::uartWrite("\n");
      continue;
    }
    if (!std::strcmp(line, "jrn")) {           // postmortem journal readout
      port::journalText(reply, sizeof reply);
      port::uartWrite(reply);
      port::uartWrite("\n");
      continue;
    }
    const ConsoleResult r = consoleLine(rt.cfg, &rt.abp, line, reply, sizeof reply);
    port::uartWrite(reply);
    port::uartWrite("\n");
    switch (r.action) {
      case ConsoleAction::SAVE: {
        uint8_t img[CFG_IMAGE_LEN];
        configPack(rt.cfg, img);
        if (!port::configSave(img, sizeof img)) port::uartWrite("ERR flash write\n");
        uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
        lorawan::keysPack(rt.abp, kimg);
        if (!port::keysSave(kimg, sizeof kimg)) port::uartWrite("ERR keys write\n");
        break;
      }
      case ConsoleAction::SETTIME:
        port::rtcSetUtc(r.timeEpoch);
        break;
      case ConsoleAction::BOOT:
        return;
      default: break;
    }
  }
}

// ---- beacon duty (pre-roll) --------------------------------------------------
// The parent's re-broadcast lands at prerollSlotStart(depth - 1); the caller
// has already woken us one (possibly widened) guard early. On success the RTC
// window clock is re-anchored to the slot's nominal time — beacon airtime
// (~153 ms at SF9) is absorbed by the guard at this resolution.
inline bool nodeBeaconDuty(NodeRuntime& rt) {
  const Config& cfg = rt.cfg;
  // Same-tier relays re-broadcast in provisioned SUB-slots (1 s apart) inside
  // their tier's pre-roll interval, so beacons from two audible parents arrive
  // in sequence, never in collision; the first valid one wins.
  const uint32_t tParent =
      prerollSlotStart(cfg.depth - 1, cfg.prerollSlotS) + cfg.parentPrerollSub;
  const uint32_t g = widenedGuard(cfg.beaconGuardS, cfg.driftSPerDay,
                                  rt.missedBeacons);
  // Acquisition mode (first boot or after a missed day): the window anchor is
  // the BEACON time, not the infrastructure power-on — the gateway+Starlink
  // take ~4 min to boot and the Class B grid adds up to 128 s. A long listen
  // (cfg.acqListenS, default 6 min) absorbs both; 86400 % 128 == 0, so once
  // observed, the beacon second repeats daily and the tight guard takes over.
  const uint32_t timeoutMs = (rt.acquired && rt.missedBeacons == 0)
                                 ? (2 * g) * 1000 + 500
                                 : cfg.acqListenS * 1000;
  uint8_t b[BEACON_LEN];
  const int len = port::radioReceiveOne(b, sizeof b, timeoutMs);
  const Beacon bc = parseBeacon(b, len > 0 ? static_cast<size_t>(len) : 0);
  if (!bc.valid) { ++rt.missedBeacons; ++rt.windowsSinceBeacon; return false; }
  rt.missedBeacons = 0;
  rt.windowsSinceBeacon = 0;
  rt.lastBeaconGps = bc.timeGps;
  rt.acquired = true;
  port::rtcSetWindowClock(tParent);
  if (cfg.profile == Profile::RELAY) {
    // re-broadcast in OUR pre-roll sub-slot, time field advanced by the exact
    // hop delta; GwSpecific is forwarded verbatim (no MIC to break).
    const uint32_t tOwn =
        prerollSlotStart(cfg.depth, cfg.prerollSlotS) + cfg.prerollSub;
    port::sleepUntil(tOwn);
    restampBeacon(b, bc.timeGps + (tOwn - tParent));
    port::radioTransmit(b, BEACON_LEN, 10 /* Class B short preamble */);
  }
  return true;
}

inline void nodeListenDuty(NodeRuntime& rt, uint32_t listenEndAbs) {
  rt.dedup.clear();
  rt.fwdq.clear();
  while (port::windowClockSec() < listenEndAbs) {
    if (!port::radioCadOnce()) {              // 2.3 ms; §IV: 745x cheaper than RX
      port::sleepUntil(port::windowClockSec()); // ~1 s tick via RTC (coarse)
      continue;
    }
    // CAD fired: the extended preamble is on air RIGHT NOW. One receive
    // attempt can still miss the lock (observed live: CAD hits with zero
    // receptions), so keep trying while the transmission is under way —
    // three back-to-back attempts outlast preamble plus payload.
    uint8_t frame[MAX_FRAME_LEN];
    int len = -1;
    for (int attempt = 0; attempt < 3 && len <= static_cast<int>(MESH_HDR_LEN);
         ++attempt)
      len = port::radioReceiveOne(frame, sizeof frame, 3000);
    if (len <= static_cast<int>(MESH_HDR_LEN)) continue;
    const MeshHeader h = unpackHeader(frame);
    if (!gradientAccepts(rt.cfg.depth, h)) continue;   // §III-C invariant
    if (!nodeOriginAllowed(rt.cfg, h.origin)) continue; // designated parenting
    if (rt.dedup.seenOrRecord(h.origin, h.seq)) continue; // multi-parent copy
    rt.fwdq.push(frame, static_cast<size_t>(len));
  }
}

inline void nodeTransmitDuty(NodeRuntime& rt) {
  const Config& cfg = rt.cfg;
  const bool abp = lorawan::keysValid(rt.abp);
  // Extended preamble exists ONLY to be caught by a duty-cycled relay's CAD
  // scan (§IV). The final hop talks to a continuously listening gateway
  // whose concentrator expects the standard LoRaWAN 8-symbol preamble —
  // a 1+ s preamble there is out of spec and may not demodulate at all.
  const uint16_t pre = (cfg.depth == 1) ? 12 : NODE_PREAMBLE_SYMS;
  // 1) own buffered ports (one frame per port, §III-A), encoded per the
  // published Open LoRa v5 payloads: sensor ports 0..ports-2 go out as
  // analog readings on FPort 11 (sensor_index = port), and the LAST port —
  // the battery, on every role — as the 3-byte battery message on FPort 13.
  // With ABP keys each payload becomes the FRMPayload of a real LoRaWAN
  // uplink — the mesh carries that PHYPayload under the 3-byte prefix, and
  // a depth-1 node sends it bare: its next hop IS the gateway.
  for (uint8_t p = 0; p < cfg.ports; ++p) {
    uint16_t readings[96];
    const uint16_t n = port::readSensorPort(p, readings, 96);
    if (n == 0) continue;
    uint8_t blob[MAX_PAYLOAD];
    uint8_t fport;
    size_t plen;
    if (p == cfg.ports - 1) {                // battery: latest sample only
      plen  = openlora::encodeBattery(readings[n - 1], blob);
      fport = openlora::FPORT_BATTERY;
    } else if (cfg.profile == Profile::STAKE && p == 0 &&
               n == gnss::POS_WORDS) {
      // the day's averaged position, as the native position message
      // (FPort 14 — the W-Mesh extension mirroring the core protocol's
      //  GNSS point: lat/lon x1e7, altitude in millimetres)
      int32_t la, lo; int16_t al; uint16_t nf;
      gnss::unpackPosition(readings, la, lo, al, nf);
      plen  = openlora::encodePosition(0, openlora::GnssStatus::OK, la, lo,
                                       (int32_t)al * 100 /* dm -> mm */, nf,
                                       readings[6] /* hdopX100 word */, blob);
      fport = openlora::FPORT_POSITION;
    } else {                                 // 15-min analog buffer
      plen  = openlora::encodeAnalog(p, openlora::RecordPeriod::MINUTES, 15,
                                     readings, n, blob, sizeof blob);
      fport = openlora::FPORT_READINGS;
    }
    if (plen == 0) continue;
    // The sequence number advances PER FRAME, not per window: the dedup key
    // upstream is (origin, seq), so two port frames sharing one seq would
    // collapse into one at the first relay — the dedup cache is sized for
    // 14 descendants x 4 ports precisely because every port frame carries
    // its own identity. (Found live: a stake's battery frame vanished in
    // the shadow of its position frame.)
    uint8_t frame[MAX_FRAME_LEN];
    if (abp) {
      uint8_t phy[MAX_PAYLOAD];
      const size_t sz = lorawan::buildUplink(rt.abp, fport,
                                             blob, plen, phy, sizeof phy);
      if (sz == 0) continue;
      ++rt.abp.fcntUp;                       // persisted below, once per window
      if (cfg.depth == 1) {
        port::radioTransmit(phy, sz, pre);
      } else {
        packHeader({cfg.nodeId, rt.mySeq, cfg.depth, MAX_TTL}, frame);
        std::memcpy(frame + MESH_HDR_LEN, phy, sz);
        port::radioTransmit(frame, MESH_HDR_LEN + sz, pre);
      }
    } else {
      packHeader({cfg.nodeId, rt.mySeq, cfg.depth, MAX_TTL}, frame);
      std::memcpy(frame + MESH_HDR_LEN, blob, plen);
      port::radioTransmit(frame, MESH_HDR_LEN + plen, pre);
    }
    ++rt.mySeq;
  }
  if (abp) {                                 // FCntUp must survive the day
    uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
    lorawan::keysPack(rt.abp, kimg);
    (void)port::keysSave(kimg, sizeof kimg);
  }
  // 2) stored foreign frames, re-stamped (§III-C).
  // TRANSPARENCY RULE: the encapsulated LoRaWAN PHYPayload is NEVER touched —
  // relays forward ciphertext + MIC + FCnt verbatim. Only the 3-byte mesh
  // prefix is rewritten between tiers, and tier-1 nodes STRIP it before the
  // final hop so a standard gateway parses standard LoRaWAN.
  for (size_t i = 0; i < rt.fwdq.size(); ++i) {
    size_t len; uint8_t frame[MAX_FRAME_LEN];
    std::memcpy(frame, rt.fwdq.frameAt(i, len), len);
    if (cfg.depth == 1) {
      // Last-hop knowledge comes from provisioning alone: depth 1 means, by
      // definition, "my parent is the gateway". No discovery is involved.
      // A prefixed frame that reaches the gateway anyway (misprovisioning or
      // direct overhear of a deep node) is dropped there by LoRaWAN parsing
      // or the MIC check — harmless; an optional gateway shim may instead
      // strip the prefix and turn such receptions into free path diversity.
      port::radioTransmit(frame + MESH_HDR_LEN, len - MESH_HDR_LEN,
                          pre);
    } else {
      restampForForward(frame, cfg.depth);
      port::radioTransmit(frame, len, pre);
    }
  }
}

// ---- boot: load provisioning, run the console, bind the window anchor --------
inline void nodeBoot(NodeRuntime& rt) {
  rt.cfg = configDefaults();   // fall back to defaults on blank/corrupt page
  {
    uint8_t img[CFG_IMAGE_LEN];
    if (port::configLoad(img, sizeof img)) (void)configUnpack(img, rt.cfg);
    uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
    if (port::keysLoad(kimg, sizeof kimg)) (void)lorawan::keysUnpack(kimg, rt.abp);
  }
  // enrolment console: any key on the console UART within 3 s of reset
  if (port::consoleAttached(3000)) nodeEnrolmentConsole(rt);
  port::timeConfigure(rt.cfg.windowUtcSec);   // bind the 12:00 UTC anchor
}

// ---- one full window (wake #1 beacon, #2 listen, #3 own TX slot) -------------
inline void nodeWindowOnce(NodeRuntime& rt) {
  const Config& cfg = rt.cfg;
  const PhasePlan plan = planFor(cfg.depth, cfg.meshH, cfg.phaseDur);
  // uplink phases begin only after every tier has had its beacon slot
  const uint32_t T0 = prerollEnd(cfg.meshH, cfg.prerollSlotS);
  // ---- wake #1: parent's beacon slot, one widened guard early ----
  const uint32_t g = widenedGuard(cfg.beaconGuardS, cfg.driftSPerDay,
                                  rt.missedBeacons);
  const uint32_t tParent =
      prerollSlotStart(cfg.depth - 1, cfg.prerollSlotS) + cfg.parentPrerollSub;
  port::sleepUntil(tParent > g ? tParent - g : 0);
  port::radioInit(cfg.beaconFreqHz);
  bool synced = nodeBeaconDuty(rt);
  // GNSS backup clock (relay/leaf option): powered ONLY on a missed beacon.
  // A fix re-arms the schedule like a caught beacon would, and a relay then
  // SYNTHESIZES the day's beacon for its children — the time field is pure
  // arithmetic (last caught beacon + 86400 s per elapsed window, plus the
  // usual hop delta), and the frame is MIC-free by design, so a locally
  // built copy is indistinguishable from a relayed one.
  if (!synced && cfg.gnssBackup && port::gnssDisciplineRtc()) {
    rt.missedBeacons = 0;
    rt.acquired = true;
    synced = true;
    if (cfg.profile == Profile::RELAY && rt.lastBeaconGps != 0) {
      const uint32_t tOwn =
          prerollSlotStart(cfg.depth, cfg.prerollSlotS) + cfg.prerollSub;
      uint8_t b[BEACON_LEN] = {0};
      restampBeacon(b, rt.lastBeaconGps + 86400u * rt.windowsSinceBeacon
                           + (tOwn - tParent));
      port::sleepUntil(tOwn);
      port::radioTransmit(b, BEACON_LEN, 10);
    }
  }
  if (!synced && rt.missedBeacons > cfg.maxFreerunDays) {
    // too stale to trust our slots — stay silent, try again next window
    port::radioSleep();
    port::sleepUntil(0 /* next window start, RTC 24h alarm */);
    return;
  }
  port::radioInit(cfg.freqHz);
  // STAKE profile: the GNSS averaging session doubles as time discipline —
  // absolute UTC every day, regardless of the beacon. A stake therefore
  // never goes silent for missed beacons: fresh GNSS time re-arms the
  // schedule exactly as a caught beacon would (the beacon stays useful as
  // a cheap "parent is alive" signal, nothing more).
  if (cfg.profile == Profile::STAKE && port::gnssDisciplineRtc()) {
    rt.missedBeacons = 0;
    rt.acquired = true;
  }
  // ---- wake #2: listening phase (relays only have one) ----
  if (plan.listenEnd > plan.listenStart) {
    port::sleepUntil(T0 + plan.listenStart);
    nodeListenDuty(rt, T0 + plan.listenEnd);
  }
  // ---- wake #3: own TX slot inside own phase ----
  port::sleepUntil(T0 + plan.txStart + cfg.txSlotIndex * cfg.txSlotDurS);
  nodeTransmitDuty(rt);
  port::radioSleep();
  port::sleepUntil(0 /* next window start, RTC 24h alarm */);
}

[[noreturn]] inline void nodeMain(NodeRuntime& rt) {
  nodeBoot(rt);
  for (;;) nodeWindowOnce(rt);
}

} // namespace wmesh
