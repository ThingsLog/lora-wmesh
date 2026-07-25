// Open LoRa message payload v5 — the LPMDL-110X LoRa open data protocol.
// Follows the published document byte for byte:
//   https://iot.thingslog.com/docs/en/LPMDL-110X-LoRa-open-data-protocol.pdf
// so a production ThingsLog decoder parses a W-Mesh node like any LPMDL
// logger. Three message kinds, distinguished by the LoRaWAN FPort:
//
//   FPort 11  readings   [ver 5][flags][every][n:2 BE][readings...]
//             flags: bits 0-3 sensor_index, bits 4-5 reading type
//             (0 = pulse differential, 1 = analog), bits 6-7 record period
//             (0 = minutes, 1 = hours, 2 = days, 3 = seconds).
//             Readings 1..n-1 are differentials AGAINST THE NEXT reading,
//             16-bit BE; the last reading is absolute — 16-bit for analog,
//             32-bit for pulse counters.
//   FPort 12  alarm      [ver 5][sensor][type][value:2 BE]
//   FPort 13  battery    [ver 5][mV:2 BE]
//
// Keep a readings packet under 51 B (the document's guidance): n <= 22 for
// analog. The W-Mesh mesh prefix is transparent to all of this.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>

namespace openlora {

constexpr uint8_t VERSION        = 5;
constexpr uint8_t FPORT_READINGS = 11;
constexpr uint8_t FPORT_ALARM    = 12;
constexpr uint8_t FPORT_BATTERY  = 13;
// W-Mesh EXTENSION (proposed for the next protocol revision): a native
// position message. Old decoders are untouched — a new FPort, same version
// byte. Purpose-built for GNSS stakes whose payload IS the position.
constexpr uint8_t FPORT_POSITION = 14;

constexpr size_t  HDR_LEN      = 5;
constexpr size_t  BATTERY_LEN  = 3;
constexpr size_t  POSITION_LEN = 16;

enum class ReadingType : uint8_t { PULSE = 0, ANALOG = 1 };
enum class RecordPeriod : uint8_t { MINUTES = 0, HOURS = 1, DAYS = 2, SECONDS = 3 };

// ---- battery (FPort 13) -----------------------------------------------------
inline size_t encodeBattery(uint16_t mv, uint8_t out[BATTERY_LEN]) {
  out[0] = VERSION;
  out[1] = static_cast<uint8_t>(mv >> 8);
  out[2] = static_cast<uint8_t>(mv & 0xFF);
  return BATTERY_LEN;
}

inline bool decodeBattery(const uint8_t* in, size_t len, uint16_t& mv) {
  if (len != BATTERY_LEN || in[0] != VERSION) return false;
  mv = static_cast<uint16_t>((in[1] << 8) | in[2]);
  return true;
}

// ---- position (FPort 14, extension) -----------------------------------------
// Mirrors the core Communication-protocol-v4 GNSS conventions so the server
// decoder shares code with the GNSS_POINTS (0x68) path:
//   [0]      version = 5
//   [1]      bits 0-4 source index | bits 5-7 status (the v4 GNSS status
//            enum: 0 OK, 1 NO_FIX, 2 TIMEOUT, 3 HW_ERR, 4 DATA_INVALID)
//   [2..13]  ONE v4 GNSS point: lat*1e7 int32 BE | lon*1e7 int32 BE |
//            alt in MILLIMETRES int32 BE; sentinel 0xFFFFFFFF per field
//            (lat or lon sentinel -> position invalid, alt sentinel ->
//             altitude unknown)
//   [14]     number of fixes averaged into the point (saturates at 255)
//   [15]     HDOP x 10 of the session (saturates at 255)
// One daily averaged position from a stake's GNSS session.
enum class GnssStatus : uint8_t { OK = 0, NO_FIX = 1, TIMEOUT = 2,
                                  HW_ERR = 3, DATA_INVALID = 4 };
constexpr uint32_t GNSS_SENTINEL = 0xFFFFFFFFu;

namespace detail {
inline void put32be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}
inline uint32_t get32be(const uint8_t* p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
         (uint32_t)p[2] << 8 | p[3];
}
} // namespace detail

inline size_t encodePosition(uint8_t sourceIndex, GnssStatus status,
                             int32_t latE7, int32_t lonE7, int32_t altMm,
                             uint16_t nFixes, uint16_t hdopX100,
                             uint8_t out[POSITION_LEN]) {
  out[0] = VERSION;
  out[1] = (uint8_t)((sourceIndex & 0x1F) | ((uint8_t)status << 5));
  detail::put32be(out + 2,  (uint32_t)latE7);
  detail::put32be(out + 6,  (uint32_t)lonE7);
  detail::put32be(out + 10, (uint32_t)altMm);
  out[14] = (uint8_t)(nFixes > 255 ? 255 : nFixes);
  const uint16_t h10 = hdopX100 / 10;
  out[15] = (uint8_t)(h10 > 255 ? 255 : h10);
  return POSITION_LEN;
}

inline bool decodePosition(const uint8_t* in, size_t len, int32_t& latE7,
                           int32_t& lonE7, int32_t& altMm, uint8_t& nFixes,
                           uint8_t& hdopX10, GnssStatus* status = nullptr,
                           uint8_t* sourceIndex = nullptr) {
  if (len != POSITION_LEN || in[0] != VERSION) return false;
  if (sourceIndex) *sourceIndex = in[1] & 0x1F;
  if (status) *status = (GnssStatus)(in[1] >> 5);
  latE7 = (int32_t)detail::get32be(in + 2);
  lonE7 = (int32_t)detail::get32be(in + 6);
  altMm = (int32_t)detail::get32be(in + 10);
  nFixes  = in[14];
  hdopX10 = in[15];
  return true;
}

// ---- analog readings (FPort 11, type 1) -------------------------------------
// n readings, oldest first; 1..n-1 go out as differentials against the next
// reading, the last is absolute 16-bit. Returns 5 + 2n, or 0 if it won't fit.
inline size_t encodeAnalog(uint8_t sensorIndex, RecordPeriod period,
                           uint8_t every, const uint16_t* readings, uint16_t n,
                           uint8_t* out, size_t maxLen) {
  const size_t need = HDR_LEN + 2u * n;
  if (n == 0 || need > maxLen) return 0;
  out[0] = VERSION;
  out[1] = static_cast<uint8_t>(
      (static_cast<uint8_t>(period) << 6) |
      (static_cast<uint8_t>(ReadingType::ANALOG) << 4) | (sensorIndex & 0x0F));
  out[2] = every;
  out[3] = static_cast<uint8_t>(n >> 8);
  out[4] = static_cast<uint8_t>(n & 0xFF);
  size_t o = HDR_LEN;
  for (uint16_t i = 0; i + 1 < n; ++i) {
    const int16_t diff = static_cast<int16_t>(readings[i + 1] - readings[i]);
    out[o++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) >> 8);
    out[o++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) & 0xFF);
  }
  out[o++] = static_cast<uint8_t>(readings[n - 1] >> 8);
  out[o++] = static_cast<uint8_t>(readings[n - 1] & 0xFF);
  return o;
}

// Host-side check / gateway decoder model for the analog message.
inline uint16_t decodeAnalog(const uint8_t* in, size_t len,
                             uint16_t* readings, uint16_t maxN,
                             uint8_t* sensorIndex = nullptr) {
  if (len < HDR_LEN + 2 || in[0] != VERSION) return 0;
  if (((in[1] >> 4) & 0x03) != static_cast<uint8_t>(ReadingType::ANALOG)) return 0;
  const uint16_t n = static_cast<uint16_t>((in[3] << 8) | in[4]);
  if (n == 0 || n > maxN || len < HDR_LEN + 2u * n) return 0;
  if (sensorIndex) *sensorIndex = in[1] & 0x0F;
  size_t o = HDR_LEN + 2u * (n - 1);
  readings[n - 1] = static_cast<uint16_t>((in[o] << 8) | in[o + 1]);
  for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
    const size_t p = HDR_LEN + 2u * static_cast<size_t>(i);
    const int16_t diff = static_cast<int16_t>((in[p] << 8) | in[p + 1]);
    readings[i] = static_cast<uint16_t>(readings[i + 1] - diff);
  }
  return n;
}

// ---- pulse readings (FPort 11, type 0) --------------------------------------
// Differentials are 16-bit; the last (absolute) counter is 32-bit BE.
// Returns 5 + 2(n-1) + 4, or 0 if it won't fit.
inline size_t encodePulse(uint8_t sensorIndex, RecordPeriod period,
                          uint8_t every, const uint32_t* readings, uint16_t n,
                          uint8_t* out, size_t maxLen) {
  const size_t need = HDR_LEN + 2u * (n - 1) + 4u;
  if (n == 0 || need > maxLen) return 0;
  out[0] = VERSION;
  out[1] = static_cast<uint8_t>(
      (static_cast<uint8_t>(period) << 6) |
      (static_cast<uint8_t>(ReadingType::PULSE) << 4) | (sensorIndex & 0x0F));
  out[2] = every;
  out[3] = static_cast<uint8_t>(n >> 8);
  out[4] = static_cast<uint8_t>(n & 0xFF);
  size_t o = HDR_LEN;
  for (uint16_t i = 0; i + 1 < n; ++i) {
    const int16_t diff =
        static_cast<int16_t>(static_cast<int32_t>(readings[i + 1] - readings[i]));
    out[o++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) >> 8);
    out[o++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) & 0xFF);
  }
  const uint32_t last = readings[n - 1];
  out[o++] = static_cast<uint8_t>(last >> 24);
  out[o++] = static_cast<uint8_t>(last >> 16);
  out[o++] = static_cast<uint8_t>(last >> 8);
  out[o++] = static_cast<uint8_t>(last & 0xFF);
  return o;
}

inline uint16_t decodePulse(const uint8_t* in, size_t len,
                            uint32_t* readings, uint16_t maxN,
                            uint8_t* sensorIndex = nullptr) {
  if (len < HDR_LEN + 4 || in[0] != VERSION) return 0;
  if (((in[1] >> 4) & 0x03) != static_cast<uint8_t>(ReadingType::PULSE)) return 0;
  const uint16_t n = static_cast<uint16_t>((in[3] << 8) | in[4]);
  if (n == 0 || n > maxN || len < HDR_LEN + 2u * (n - 1) + 4u) return 0;
  if (sensorIndex) *sensorIndex = in[1] & 0x0F;
  const size_t o = HDR_LEN + 2u * (n - 1);
  readings[n - 1] = (static_cast<uint32_t>(in[o]) << 24) |
                    (static_cast<uint32_t>(in[o + 1]) << 16) |
                    (static_cast<uint32_t>(in[o + 2]) << 8) |
                    static_cast<uint32_t>(in[o + 3]);
  for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
    const size_t p = HDR_LEN + 2u * static_cast<size_t>(i);
    const int16_t diff = static_cast<int16_t>((in[p] << 8) | in[p + 1]);
    readings[i] = readings[i + 1] - static_cast<uint32_t>(diff);
  }
  return n;
}

} // namespace openlora
