// W-Mesh beacon time discipline — platform-independent.
//
// The cascade's phase boundaries rest on clock alignment, and LoRaWAN already
// ships the primitive: the Class B beacon (TS001 / RP002, EU868) — 17 bytes at
// SF9 on 869.525 MHz, carrying GPS-epoch time, deliberately unencrypted and
// without MIC. That makes it legal to re-broadcast AND to re-stamp: relays
// forward it tier by tier with a corrected time field, and nodes count their
// slots from beacon reception rather than from an absolute clock. Drift then
// only has to hold across the window itself (100 ppm x 900 s = 90 ms), so
// relays and telemetry leaves need no GNSS receiver at all; GNSS remains only
// where position IS the payload (glacier stakes).
//
// PRE-ROLL ORDERING (important): uplink phases run DEEPEST-FIRST, so the
// beacon must reach the deepest tier before phase 1 begins. The window
// therefore opens with a short beacon pre-roll that cascades OUTWARD:
//   t=0                gateway emits the beacon
//   t=1*PREROLL_SLOT   tier-1 relays re-broadcast (re-stamped)
//   t=2*PREROLL_SLOT   tier-2 relays re-broadcast ...
// after H slots every tier holds fresh time and the uplink phases begin.
//
// EU868 beacon layout (17 B):
//   [ RFU 2 ][ Time 4 LE ][ CRC 2 LE ][ GwSpecific 7 ][ CRC 2 LE ]
//   Time = seconds since GPS epoch, mod 2^32.
//   CRC16 polynomial x^16 + x^12 + x^5 + 1 (0x1021), init 0x0000 (TS001).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>

namespace wmesh {

constexpr size_t BEACON_LEN   = 17;
constexpr size_t BCN_TIME_OFF = 2;   // 4 bytes, little-endian GPS seconds
constexpr size_t BCN_CRC1_OFF = 6;   // over bytes [0, 6)
constexpr size_t BCN_GWS_OFF  = 8;   // GwSpecific, opaque to relays
constexpr size_t BCN_CRC2_OFF = 15;  // over bytes [8, 15)

inline uint16_t beaconCrc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0x0000;                         // init 0 per TS001
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(d[i]) << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

struct Beacon {
  bool     valid;
  uint32_t timeGps;   // seconds since GPS epoch (mod 2^32)
};

// Validates the NetCommon CRC only — GwSpecific is opaque to the mesh.
inline Beacon parseBeacon(const uint8_t* b, size_t len) {
  Beacon r{false, 0};
  if (b == nullptr || len != BEACON_LEN) return r;
  const uint16_t stored =
      static_cast<uint16_t>(b[BCN_CRC1_OFF] | (b[BCN_CRC1_OFF + 1] << 8));
  if (stored != beaconCrc16(b, BCN_CRC1_OFF)) return r;
  r.timeGps = static_cast<uint32_t>(b[BCN_TIME_OFF])
            | static_cast<uint32_t>(b[BCN_TIME_OFF + 1]) << 8
            | static_cast<uint32_t>(b[BCN_TIME_OFF + 2]) << 16
            | static_cast<uint32_t>(b[BCN_TIME_OFF + 3]) << 24;
  r.valid = true;
  return r;
}

// Relay re-stamp before re-broadcast: the time field is plain and CRC-only,
// so the relay writes the time of ITS OWN slot (parent time + one pre-roll
// slot) and recomputes the NetCommon CRC. GwSpecific is forwarded verbatim.
inline void restampBeacon(uint8_t* b, uint32_t timeGps) {
  b[BCN_TIME_OFF]     = static_cast<uint8_t>(timeGps);
  b[BCN_TIME_OFF + 1] = static_cast<uint8_t>(timeGps >> 8);
  b[BCN_TIME_OFF + 2] = static_cast<uint8_t>(timeGps >> 16);
  b[BCN_TIME_OFF + 3] = static_cast<uint8_t>(timeGps >> 24);
  const uint16_t crc = beaconCrc16(b, BCN_CRC1_OFF);
  b[BCN_CRC1_OFF]     = static_cast<uint8_t>(crc);
  b[BCN_CRC1_OFF + 1] = static_cast<uint8_t>(crc >> 8);
}

// --- pre-roll arithmetic -----------------------------------------------------
// Slot in which the node at `depth` re-broadcasts (gateway = depth 0, t = 0).
inline uint32_t prerollSlotStart(uint8_t depth, uint32_t slotDurS) {
  return static_cast<uint32_t>(depth) * slotDurS;
}
// The uplink phases begin once every tier has had its re-broadcast slot.
inline uint32_t prerollEnd(uint8_t H, uint32_t slotDurS) {
  return static_cast<uint32_t>(H) * slotDurS;
}

// --- missed-beacon fallback (§VIII: beacon dependency) ------------------------
// A node that misses the beacon free-runs on its last calibration and widens
// its wake-up guard by the residual drift bound per silent day; past a
// configured limit it stays silent rather than transmit into foreign slots.
inline uint32_t widenedGuard(uint32_t baseGuardS, uint32_t driftSPerDay,
                             uint32_t missedDays) {
  return baseGuardS + driftSPerDay * missedDays;
}

} // namespace wmesh
