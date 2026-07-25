// Host-side node simulation — the WHOLE window state machine of
// core/wmesh_node.h runs against a scripted mock port, no hardware needed.
// Scenarios: beacon acquisition + re-anchor, leaf battery telemetry, relay
// beacon re-stamp + store-and-forward with dedup/whitelist/gradient filters,
// and the missed-beacon freerun-then-silence policy.
//
// The mock clock is a window-relative second counter: sleepUntil() jumps it,
// a target behind "now" rolls into the next window (exactly the RTC 24 h
// alarm semantics of the real ports). Scripted air frames carry the window
// time at which they become receivable; each is consumed at most once.
//
// Build:  c++ -std=c++17 -I.. node_sim_test.cpp -o node_sim && ./node_sim
// SPDX-License-Identifier: MIT
#include "../core/wmesh_node.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

// ---- scripted world ---------------------------------------------------------
struct AirFrame {
  uint32_t t;                       // window second at which it is on air
  std::vector<uint8_t> bytes;
  bool used = false;
};
struct TxRecord {
  uint32_t t;                       // window second of the transmission
  uint32_t window;                  // window counter at that moment
  std::vector<uint8_t> bytes;
};

static uint32_t g_cur = 0;          // window clock, seconds
static uint32_t g_window = 0;       // wraps counted
static std::vector<AirFrame> g_air;
static std::vector<TxRecord> g_tx;
static std::vector<uint8_t> g_flash;
static std::vector<uint8_t> g_keys;
static uint16_t g_batteryMv = 3600;

static void simReset(uint32_t bootSec) {
  g_cur = bootSec; g_window = 0;
  g_air.clear(); g_tx.clear(); g_flash.clear(); g_keys.clear();
}

// earliest unused frame receivable within [g_cur, g_cur + horizonS]
static AirFrame* nextAir(uint32_t horizonS) {
  AirFrame* best = nullptr;
  for (auto& f : g_air)
    if (!f.used && f.t >= g_cur && f.t <= g_cur + horizonS)
      if (!best || f.t < best->t) best = &f;
  return best;
}

// ---- mock port --------------------------------------------------------------
namespace port {

bool consoleAttached(uint32_t) { return false; }
int  uartReadLine(char*, size_t) { return 0; }
void uartWrite(const char*) {}

bool configLoad(uint8_t* img, size_t len) {
  if (g_flash.size() != len) return false;
  std::memcpy(img, g_flash.data(), len);
  return true;
}
bool configSave(const uint8_t* img, size_t len) {
  g_flash.assign(img, img + len);
  return true;
}

bool keysLoad(uint8_t* img, size_t len) {
  if (g_keys.size() != len) return false;
  std::memcpy(img, g_keys.data(), len);
  return true;
}
bool keysSave(const uint8_t* img, size_t len) {
  g_keys.assign(img, img + len);
  return true;
}

void journalText(char* out, size_t cap) { if (cap) out[0] = 0; }

void rtcSetUtc(uint32_t) {}
void timeConfigure(uint32_t) {}     // the test presets g_cur itself
uint32_t windowClockSec() { return g_cur; }
void rtcSetWindowClock(uint32_t s) { g_cur = s; }
void sleepUntil(uint32_t target) {
  if      (target > g_cur) g_cur = target;
  else if (target == g_cur) g_cur += 1;          // coarse 1 s tick
  else { ++g_window; g_cur = target; }           // RTC 24 h alarm: next window
}
bool gnssDisciplineRtc() { return false; }

void radioInit(uint32_t) {}
void radioSleep() {}
bool radioTransmit(const uint8_t* frame, size_t len, uint16_t) {
  g_tx.push_back({g_cur, g_window, std::vector<uint8_t>(frame, frame + len)});
  return true;
}
bool radioCadOnce() { return nextAir(1) != nullptr; }
int  radioReceiveOne(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  const uint32_t horizon = timeoutMs / 1000u;
  AirFrame* f = nextAir(horizon);
  if (!f) { g_cur += horizon; return -1; }       // listened out the timeout
  f->used = true;
  if (f->t > g_cur) g_cur = f->t;
  const size_t n = f->bytes.size() <= maxLen ? f->bytes.size() : maxLen;
  std::memcpy(buf, f->bytes.data(), n);
  return (int)n;
}

uint16_t readBatteryMv() { return g_batteryMv; }
static bool g_posReady = false;                  // set by the gnss mock below
uint16_t readSensorPort(uint8_t port, uint16_t* out, uint16_t maxN) {
  if (port > 1 || maxN == 0) return 0;           // up to two mock ports
  if (port == 0 && g_posReady && maxN >= gnss::POS_WORDS) {
    // the position words a real stake port would hand over (Livingston-ish)
    const uint32_t la = (uint32_t)(int32_t)-626353908;
    const uint32_t lo = (uint32_t)(int32_t)-603590535;
    out[0]=(uint16_t)(la>>16); out[1]=(uint16_t)la;
    out[2]=(uint16_t)(lo>>16); out[3]=(uint16_t)lo;
    out[4]=1053; out[5]=42; out[6]=180; out[7]=0;
    g_posReady = false;
    return gnss::POS_WORDS;
  }
  out[0] = g_batteryMv;
  return 1;
}

} // namespace port

// ---- helpers ----------------------------------------------------------------
using namespace wmesh;

static std::vector<uint8_t> makeBeacon(uint32_t gpsTime) {
  std::vector<uint8_t> b(BEACON_LEN, 0);
  for (size_t i = 0; i < 7; ++i) b[BCN_GWS_OFF + i] = (uint8_t)(0xA0 + i);
  restampBeacon(b.data(), gpsTime);
  return b;
}

static std::vector<uint8_t> makeDataFrame(uint8_t origin, uint8_t seq,
                                          uint8_t depth, uint16_t reading) {
  std::vector<uint8_t> f(MAX_FRAME_LEN);
  packHeader({origin, seq, depth, MAX_TTL}, f.data());
  const size_t plen = openlora::encodeBattery(reading, f.data() + MESH_HDR_LEN);
  f.resize(MESH_HDR_LEN + plen);
  return f;
}

static void provision(NodeRuntime& rt, const Config& c) {
  uint8_t img[CFG_IMAGE_LEN];
  configPack(c, img);
  g_flash.assign(img, img + CFG_IMAGE_LEN);
  nodeBoot(rt);                                  // loads it back via the port
  assert(rt.cfg.nodeId == c.nodeId);             // flash round-trip took
}

int main() {
  const uint32_t GPS_T = 1400000000u;

  // ===== A: leaf — acquisition catch, re-anchor, battery frame in own slot ===
  {
    simReset(/*bootSec=*/100);                   // enrolment ended mid-window
    Config c = configDefaults();
    c.nodeId = 7; c.profile = Profile::LEAF; c.depth = 1; c.meshH = 1;
    c.ports = 1;                                 // bare leaf: battery only
    c.phaseDur[0] = 900; c.prerollSlotS = 2; c.txSlotIndex = 2; c.txSlotDurS = 12;
    c.beaconGuardS = 1; c.acqListenS = 360;
    assert(configValidate(c) == nullptr);
    NodeRuntime rt;
    provision(rt, c);
    // gateway+Starlink booted late: beacon lands 130 s into the blind window
    g_air.push_back({130, makeBeacon(GPS_T)});
    nodeWindowOnce(rt);
    assert(rt.acquired && rt.missedBeacons == 0 && rt.lastBeaconGps == GPS_T);
    // exactly one transmission: the battery message, in the provisioned slot,
    // at T0 + slot*dur = 1*2 + 2*12 = 26 ON THE RE-ANCHORED clock
    assert(g_tx.size() == 1);
    assert(g_tx[0].t == 26);
    const MeshHeader h = unpackHeader(g_tx[0].bytes.data());
    assert(h.origin == 7 && h.seq == 0 && h.depth == 1 && h.ttl == MAX_TTL);
    uint16_t mvOut = 0;
    assert(openlora::decodeBattery(g_tx[0].bytes.data() + MESH_HDR_LEN,
                                   g_tx[0].bytes.size() - MESH_HDR_LEN, mvOut));
    assert(mvOut == 3600);                       // the battery, in millivolts
    assert(rt.mySeq == 1);                       // per-window sequence advanced
  }

  // ===== B: relay — beacon re-stamp, forward with dedup/whitelist/gradient ===
  {
    simReset(100);
    Config c = configDefaults();
    c.nodeId = 21; c.profile = Profile::RELAY; c.depth = 1; c.meshH = 2;
    c.ports = 1;
    c.phaseDur[0] = 300; c.phaseDur[1] = 600;    // deepest first
    c.prerollSlotS = 2; c.prerollSub = 0; c.parentPrerollSub = 0;
    c.txSlotIndex = 0; c.txSlotDurS = 12; c.beaconGuardS = 1; c.acqListenS = 360;
    c.whitelistLen = 2; c.whitelist[0] = 31; c.whitelist[1] = 32;
    assert(configValidate(c) == nullptr);
    NodeRuntime rt;
    provision(rt, c);
    g_air.push_back({130, makeBeacon(GPS_T)});
    // listen phase is T0..T0+300 = 4..304 (post-anchor). Scheduled after the
    // blind acquisition catch at 130, as their real TX slots would be:
    g_air.push_back({150, makeDataFrame(31, 5, 2, 3210)}); // child -> forward
    g_air.push_back({160, makeDataFrame(31, 5, 2, 3210)}); // dup    -> dedup
    g_air.push_back({170, makeDataFrame(99, 1, 2, 1111)}); // foreign -> reject
    g_air.push_back({180, makeDataFrame(32, 9, 1, 2222)}); // same depth -> reject
    g_air.push_back({190, makeDataFrame(31, 6, 2, 3211)}); // child's 2nd PORT
                                                           // frame (seq+1) is
                                                           // NOT a duplicate
    nodeWindowOnce(rt);
    assert(g_tx.size() == 4);
    // [0] the re-stamped beacon, in OUR pre-roll slot (t = 2), time += hop delta
    assert(g_tx[0].t == 2 && g_tx[0].bytes.size() == BEACON_LEN);
    const Beacon rb = parseBeacon(g_tx[0].bytes.data(), BEACON_LEN);
    assert(rb.valid && rb.timeGps == GPS_T + 2);
    for (size_t i = 0; i < 7; ++i)               // GwSpecific verbatim
      assert(g_tx[0].bytes[BCN_GWS_OFF + i] == (uint8_t)(0xA0 + i));
    // [1] own battery frame at T0 + txStart(300) + 0 = 304, with mesh header
    // (+1: slot 0 starts exactly where the listen phase ends, and the coarse
    //  1 s RTC tick lands the wake-up one second into the slot — harmless
    //  against the 12 s slot pitch, and the same on the real ports)
    assert(g_tx[1].t == 305);
    assert(unpackHeader(g_tx[1].bytes.data()).origin == 21);
    // [2],[3] BOTH child port frames forwarded, PREFIX STRIPPED (last hop) —
    // per-frame sequence numbers keep them distinct in the dedup cache
    const auto child  = makeDataFrame(31, 5, 2, 3210);
    const auto child2 = makeDataFrame(31, 6, 2, 3211);
    assert(g_tx[2].t == 305 && g_tx[3].t == 305);
    assert(g_tx[2].bytes.size() == child.size() - MESH_HDR_LEN);
    assert(std::memcmp(g_tx[2].bytes.data(), child.data() + MESH_HDR_LEN,
                       g_tx[2].bytes.size()) == 0);
    assert(std::memcmp(g_tx[3].bytes.data(), child2.data() + MESH_HDR_LEN,
                       g_tx[3].bytes.size()) == 0);
  }

  // ===== C: deep relay forwards WITH a re-stamped prefix (not last hop) ======
  {
    simReset(100);
    Config c = configDefaults();
    c.nodeId = 41; c.profile = Profile::RELAY; c.depth = 2; c.meshH = 3;
    c.ports = 1;
    c.phaseDur[0] = 200; c.phaseDur[1] = 300; c.phaseDur[2] = 400;
    c.prerollSlotS = 2; c.prerollSub = 0; c.parentPrerollSub = 0;
    c.txSlotIndex = 1; c.txSlotDurS = 12; c.beaconGuardS = 1; c.acqListenS = 360;
    assert(configValidate(c) == nullptr);
    NodeRuntime rt;
    provision(rt, c);
    g_air.push_back({130, makeBeacon(GPS_T)});   // parent relay's re-broadcast
    // T0 = 6; listen (tier-3 TX phase) 6..206; own TX phase starts at 206
    g_air.push_back({150, makeDataFrame(51, 2, 3, 2987)});
    nodeWindowOnce(rt);
    assert(g_tx.size() == 3);                    // beacon + own + forwarded
    // re-broadcast in OUR slot (t = 2*2 = 4); the time field advances by the
    // exact hop delta tOwn - tParent = 4 - 2 = 2 (the parent's re-stamp
    // already carried the earlier hops)
    assert(g_tx[0].t == 4);
    assert(parseBeacon(g_tx[0].bytes.data(), BEACON_LEN).timeGps == GPS_T + 2);
    // forwarded frame keeps the prefix, re-stamped to OUR depth, TTL - 1
    assert(g_tx[2].t == 206 + 1 * 12);
    const MeshHeader fh = unpackHeader(g_tx[2].bytes.data());
    assert(fh.origin == 51 && fh.seq == 2);
    assert(fh.depth == 2 && fh.ttl == MAX_TTL - 1);
  }

  // ===== D: missed beacons — freerun for maxFreerunDays, then silence ========
  {
    simReset(100);
    Config c = configDefaults();
    c.nodeId = 7; c.profile = Profile::LEAF; c.depth = 1; c.meshH = 1;
    c.ports = 1;
    c.phaseDur[0] = 900; c.prerollSlotS = 2; c.txSlotIndex = 2; c.txSlotDurS = 12;
    c.beaconGuardS = 1; c.acqListenS = 5; c.driftSPerDay = 1; c.maxFreerunDays = 3;
    assert(configValidate(c) == nullptr);
    NodeRuntime rt;
    provision(rt, c);
    // no beacon ever: 3 freerun windows still deliver telemetry...
    nodeWindowOnce(rt);
    assert(g_tx.size() == 1 && rt.missedBeacons == 1);
    nodeWindowOnce(rt);
    nodeWindowOnce(rt);
    assert(g_tx.size() == 3 && rt.missedBeacons == 3);
    // ...the 4th crosses maxFreerunDays: the node shuts up (foreign-slot risk)
    nodeWindowOnce(rt);
    assert(g_tx.size() == 3);
    assert(rt.missedBeacons == 4 && !rt.acquired);
    // a beacon the NEXT day revives it: acquisition listen is long again
    g_air.push_back({130, makeBeacon(GPS_T)});
    Config wide = rt.cfg; wide.acqListenS = 360; rt.cfg = wide;
    nodeWindowOnce(rt);
    assert(rt.acquired && rt.missedBeacons == 0);
    assert(g_tx.size() == 4 && g_tx[3].t == 26);
  }

  // ===== E: ABP leaf — LoRaWAN uplink on air, prefix by depth, FCnt persists =
  {
    simReset(100);
    Config c = configDefaults();
    c.nodeId = 7; c.profile = Profile::LEAF; c.depth = 1; c.meshH = 1;
    c.ports = 1;
    c.phaseDur[0] = 900; c.prerollSlotS = 2; c.txSlotIndex = 2; c.txSlotDurS = 12;
    c.beaconGuardS = 1; c.acqListenS = 360;
    NodeRuntime rt;
    lorawan::AbpKeys keys;
    keys.devAddr = 0x26011BDA;
    for (int i = 0; i < 16; ++i) { keys.nwkSKey[i] = (uint8_t)(i + 1); keys.appSKey[i] = (uint8_t)(0xA0 + i); }
    keys.fcntUp = 7;
    uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
    lorawan::keysPack(keys, kimg);
    g_keys.assign(kimg, kimg + sizeof kimg);
    provision(rt, c);
    assert(lorawan::keysValid(rt.abp) && rt.abp.fcntUp == 7);
    g_air.push_back({130, makeBeacon(GPS_T)});
    nodeWindowOnce(rt);
    // depth 1 -> the PHYPayload goes out BARE (next hop is the gateway):
    assert(g_tx.size() == 1 && g_tx[0].t == 26);
    const auto& phy = g_tx[0].bytes;
    assert(phy[0] == 0x40);                          // LoRaWAN MHDR, no prefix
    assert(phy.size() == 13 + openlora::BATTERY_LEN);
    uint8_t pay[64]; size_t pn; uint8_t fp;
    assert(lorawan::openUplink(keys, phy.data(), phy.size(), pay, pn, fp));
    assert(fp == openlora::FPORT_BATTERY);           // 13, per the v5 document
    uint16_t mv2 = 0;
    assert(openlora::decodeBattery(pay, pn, mv2) && mv2 == 3600);
    // FCnt advanced and PERSISTED through the port
    assert(rt.abp.fcntUp == 8);
    lorawan::AbpKeys kk;
    assert(lorawan::keysUnpack(g_keys.data(), kk) && kk.fcntUp == 8);

    // deeper ABP node with TWO ports: both frames under the prefix, and each
    // carries its OWN mesh sequence number (the dedup identity upstream)
    simReset(100);
    Config c2 = c; c2.depth = 2; c2.meshH = 2; c2.ports = 2;
    c2.phaseDur[0] = 300; c2.phaseDur[1] = 600;
    NodeRuntime rt2;
    g_keys.assign(kimg, kimg + sizeof kimg);
    provision(rt2, c2);
    g_air.push_back({130, makeBeacon(GPS_T)});
    nodeWindowOnce(rt2);
    assert(g_tx.size() == 2);
    const MeshHeader mh0 = unpackHeader(g_tx[0].bytes.data());
    const MeshHeader mh1 = unpackHeader(g_tx[1].bytes.data());
    assert(mh0.origin == 7 && mh0.depth == 2);       // prefix present
    assert(mh0.seq == 0 && mh1.seq == 1);            // per-frame sequence
    assert(g_tx[0].bytes[MESH_HDR_LEN] == 0x40);     // PHYPayload underneath
    assert(lorawan::openUplink(keys, g_tx[0].bytes.data() + MESH_HDR_LEN,
                               g_tx[0].bytes.size() - MESH_HDR_LEN, pay, pn, fp));
  }

  // ===== F: STAKE — the day's position goes out as the FPort-14 message =====
  {
    simReset(100);
    Config c = configDefaults();
    c.nodeId = 7; c.profile = Profile::STAKE; c.depth = 1; c.meshH = 1;
    c.ports = 2;                                 // port 0 position, port 1 battery
    c.phaseDur[0] = 900; c.prerollSlotS = 2; c.txSlotIndex = 2; c.txSlotDurS = 12;
    c.beaconGuardS = 1; c.acqListenS = 5;
    assert(configValidate(c) == nullptr);
    NodeRuntime rt;
    lorawan::AbpKeys keys;
    keys.devAddr = 0x26011BDA;
    for (int i = 0; i < 16; ++i) { keys.nwkSKey[i] = (uint8_t)(i + 1); keys.appSKey[i] = (uint8_t)(0xA0 + i); }
    uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
    lorawan::keysPack(keys, kimg);
    g_keys.assign(kimg, kimg + sizeof kimg);
    provision(rt, c);
    port::g_posReady = true;                     // "the GNSS session succeeded"
    nodeWindowOnce(rt);
    assert(g_tx.size() == 2);                    // position + battery
    uint8_t pay[64]; size_t pn; uint8_t fp;
    assert(lorawan::openUplink(keys, g_tx[0].bytes.data(), g_tx[0].bytes.size(),
                               pay, pn, fp));
    assert(fp == openlora::FPORT_POSITION);
    int32_t la, lo, am; uint8_t nf, h10;
    assert(openlora::decodePosition(pay, pn, la, lo, am, nf, h10));
    assert(la == -626353908 && lo == -603590535);  // Livingston survives e2e
    assert(am == 105300 && nf == 42 && h10 == 18); // 1053 dm -> 105300 mm
    assert(lorawan::openUplink(keys, g_tx[1].bytes.data(), g_tx[1].bytes.size(),
                               pay, pn, fp));
    assert(fp == openlora::FPORT_BATTERY);
  }

  std::puts("ALL W-MESH NODE SIMULATION TESTS PASSED");
  return 0;
}
