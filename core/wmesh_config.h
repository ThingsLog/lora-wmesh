// W-Mesh provisioning block — platform-independent.
//
// Everything about WHEN and HOW a node participates lives in one structure,
// stored in a flash page and edited over the boot console (wmesh_console.h).
// The single most important field is windowUtcSec: a freshly deployed node
// has never heard a beacon, so its FIRST wake-up is blind — driven only by
// the RTC set at enrolment and this field. For the Livingston network:
// windowUtcSec = 43200 (12:00 UTC). After the first caught beacon the value
// only anchors the 24 h alarm; the beacon trims everything finer.
//
// Image layout (little-endian, fixed 128 B):
//   [ magic "WMC1" (4) ][ fields ... ][ pad to 126 ][ CRC16 (2) ]
// CRC16 is the same x^16+x^12+x^5+1 (init 0) used for the Class B beacon.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include "wmesh_beacon.h"   // beaconCrc16

namespace wmesh {

constexpr size_t  CFG_IMAGE_LEN = 128;
constexpr uint8_t CFG_MAX_H     = 15;   // depth field is 4 bits
constexpr uint8_t CFG_MAX_CHILDREN = 14; // regulatory fan-in bound (§V)

enum class Profile : uint8_t { LEAF = 0, RELAY = 1, STAKE = 2 };

struct Config {
  uint8_t  nodeId;
  Profile  profile;
  uint8_t  depth;              // 1..meshH
  uint8_t  meshH;              // deepest tier in the network
  uint8_t  ports;              // sensing channels on this node
  uint32_t windowUtcSec;       // window start, seconds after 00:00 UTC
  uint32_t phaseDur[CFG_MAX_H];// per-phase seconds, deepest first, [meshH] used
  uint32_t prerollSlotS;       // beacon cascade slot per tier
  uint8_t  prerollSub;         // relay's sub-slot (s) within its tier's slot:
                               // same-tier relays re-broadcast in sequence,
                               // so a dual-parented child hears two clean
                               // copies, not a collision (first valid wins)
  uint8_t  parentPrerollSub;   // the sub-slot of THIS node's beacon parent
  uint16_t txSlotIndex;        // this node's private slot inside its TX phase
  uint32_t txSlotDurS;         // slot pitch: worst per-node airtime + guard
  uint32_t beaconGuardS;       // wake margin around the parent's beacon slot
  uint32_t acqListenS;         // long listen on first boot / after a missed day
                               // (covers gateway+Starlink boot ~4 min and the
                               //  128 s Class B grid; 86400 % 128 == 0, so a
                               //  once-observed beacon time repeats daily)
  uint32_t driftSPerDay;       // free-run guard widening per missed day
  uint32_t maxFreerunDays;     // beyond this: stay silent (§VIII)
  uint32_t freqHz;             // THE branch channel. W-Mesh does not hop:
                               // a tier-1 relay's whole subtree shares one
                               // fixed channel (children TX here, the relay
                               // listens and forwards here); parallel
                               // subtrees use different channels and the
                               // gateway's concentrator hears all eight.
  uint32_t beaconFreqHz;       // RP002 Class B: 869525000
  uint8_t  whitelist[CFG_MAX_CHILDREN]; // designated children (relays)
  uint8_t  whitelistLen;       // 0 = promiscuous (dual-parenting mode)
  uint8_t  prefixless;         // 1 once per-tier sync words are provisioned
  uint8_t  gnssBackup;         // relay/leaf option: GNSS fitted as a BACKUP
                               // clock, powered ONLY on a missed beacon
                               // (~0.25 mAh per miss-day). The node re-arms
                               // its schedule from the fix, and a relay
                               // synthesizes the day's beacon for its
                               // children — one upstream miss no longer
                               // silences a subtree. (Stakes have GNSS as
                               // payload already; flag invalid there.)
};

inline Config configDefaults() {
  Config c{};
  c.nodeId        = 1;
  c.profile       = Profile::LEAF;
  c.depth         = 1;
  c.meshH         = 1;
  c.ports         = 4;
  c.windowUtcSec  = 43200;          // 12:00 UTC — the Livingston anchor
  c.phaseDur[0]   = 900;
  c.prerollSlotS  = 2;
  c.prerollSub    = 0;
  c.parentPrerollSub = 0;
  c.txSlotIndex   = 0;
  c.txSlotDurS    = 12;
  c.beaconGuardS  = 1;
  c.acqListenS    = 360;            // 6 min: boot jitter + one full 128 s grid
  c.driftSPerDay  = 1;
  c.maxFreerunDays= 3;
  c.freqHz        = 868100000;
  c.beaconFreqHz  = 869525000;
  c.whitelistLen  = 0;
  c.prefixless    = 0;
  c.gnssBackup    = 0;
  return c;
}

// Returns nullptr when valid, else a short reason string (static storage).
inline const char* configValidate(const Config& c) {
  if (c.nodeId == 0)                      return "id must be 1..255";
  if (c.meshH < 1 || c.meshH > CFG_MAX_H) return "h must be 1..15";
  if (c.depth < 1 || c.depth > c.meshH)   return "depth must be 1..h";
  if (c.ports < 1 || c.ports > 4)         return "ports must be 1..4";
  if (c.windowUtcSec >= 86400)            return "window must be < 24:00";
  uint32_t total = 0;
  for (uint8_t i = 0; i < c.meshH; ++i) {
    if (c.phaseDur[i] == 0)               return "phase durations must be > 0";
    total += c.phaseDur[i];
  }
  if (c.prerollSlotS == 0)                return "preroll must be > 0";
  if (c.prerollSub > 15 || c.parentPrerollSub > 15)
                                          return "psub/ppsub must be 0..15";
  if (c.profile == Profile::RELAY && c.prerollSub >= c.prerollSlotS)
                                          return "psub exceeds preroll slot";
  if (c.parentPrerollSub >= c.prerollSlotS)
                                          return "ppsub exceeds preroll slot";
  if (c.txSlotDurS == 0)                  return "slotdur must be > 0";
  if (c.acqListenS == 0)                  return "acq must be > 0";
  // the node's private slot must fit inside its own TX phase
  const uint32_t ownPhase = c.phaseDur[c.meshH - c.depth];
  if ((uint32_t)(c.txSlotIndex + 1) * c.txSlotDurS > ownPhase)
                                          return "slot exceeds own phase";
  if (c.whitelistLen > CFG_MAX_CHILDREN)  return "whitelist exceeds fan-in 14";
  if (c.profile != Profile::RELAY && c.whitelistLen > 0)
                                          return "whitelist is relay-only";
  if (c.gnssBackup && c.profile == Profile::STAKE)
                                          return "stake has GNSS already";
  (void)total;
  return nullptr;
}

// --- pack / unpack ------------------------------------------------------------
namespace detail {
inline void put32(uint8_t* p, uint32_t v) {
  p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
inline uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
} // namespace detail

inline void configPack(const Config& c, uint8_t img[CFG_IMAGE_LEN]) {
  std::memset(img, 0, CFG_IMAGE_LEN);
  std::memcpy(img, "WMC1", 4);
  uint8_t* p = img + 4;
  *p++ = c.nodeId; *p++ = (uint8_t)c.profile; *p++ = c.depth; *p++ = c.meshH;
  *p++ = c.ports;  *p++ = c.whitelistLen;     *p++ = (uint8_t)((c.gnssBackup ? 2 : 0) | (c.prefixless ? 1 : 0));
  *p++ = (uint8_t)((c.prerollSub << 4) | (c.parentPrerollSub & 0x0F));
  detail::put32(p, c.windowUtcSec);  p += 4;
  for (uint8_t i = 0; i < CFG_MAX_H; ++i) { detail::put32(p, c.phaseDur[i]); p += 4; }
  detail::put32(p, c.prerollSlotS);  p += 4;
  detail::put32(p, c.txSlotIndex);   p += 4;
  detail::put32(p, c.txSlotDurS);    p += 4;
  detail::put32(p, c.beaconGuardS);  p += 4;
  detail::put32(p, c.acqListenS);    p += 4;
  detail::put32(p, c.driftSPerDay);  p += 4;
  detail::put32(p, c.maxFreerunDays);p += 4;
  detail::put32(p, c.freqHz);        p += 4;
  detail::put32(p, c.beaconFreqHz);  p += 4;
  std::memcpy(p, c.whitelist, CFG_MAX_CHILDREN); p += CFG_MAX_CHILDREN;
  const uint16_t crc = beaconCrc16(img, CFG_IMAGE_LEN - 2);
  img[CFG_IMAGE_LEN - 2] = (uint8_t)crc;
  img[CFG_IMAGE_LEN - 1] = (uint8_t)(crc >> 8);
}

// Returns false on bad magic/CRC; `c` is untouched then (caller keeps defaults).
inline bool configUnpack(const uint8_t img[CFG_IMAGE_LEN], Config& c) {
  if (std::memcmp(img, "WMC1", 4) != 0) return false;
  const uint16_t stored =
      (uint16_t)img[CFG_IMAGE_LEN - 2] | (uint16_t)img[CFG_IMAGE_LEN - 1] << 8;
  if (stored != beaconCrc16(img, CFG_IMAGE_LEN - 2)) return false;
  const uint8_t* p = img + 4;
  c.nodeId = *p++; c.profile = (Profile)*p++; c.depth = *p++; c.meshH = *p++;
  c.ports = *p++;  c.whitelistLen = *p++;
  c.prefixless = (uint8_t)(*p & 1); c.gnssBackup = (uint8_t)((*p >> 1) & 1); ++p;
  c.prerollSub = (uint8_t)(*p >> 4); c.parentPrerollSub = (uint8_t)(*p & 0x0F); ++p;
  c.windowUtcSec = detail::get32(p);  p += 4;
  for (uint8_t i = 0; i < CFG_MAX_H; ++i) { c.phaseDur[i] = detail::get32(p); p += 4; }
  c.prerollSlotS  = detail::get32(p); p += 4;
  c.txSlotIndex   = (uint16_t)detail::get32(p); p += 4;
  c.txSlotDurS    = detail::get32(p); p += 4;
  c.beaconGuardS  = detail::get32(p); p += 4;
  c.acqListenS    = detail::get32(p); p += 4;
  c.driftSPerDay  = detail::get32(p); p += 4;
  c.maxFreerunDays= detail::get32(p); p += 4;
  c.freqHz        = detail::get32(p); p += 4;
  c.beaconFreqHz  = detail::get32(p); p += 4;
  std::memcpy(c.whitelist, p, CFG_MAX_CHILDREN);
  return true;
}

} // namespace wmesh
