// HDC3020 (TI) — temperature + relative humidity over I2C.
// The reference environmental sensor of the deployment: ~100 nA in sleep,
// single-shot measurement on demand — woken by the same RTC that runs the
// schedule, sampled every 15 min into the RAM2 ring buffer, transmitted once
// per day as two OpenLoRa ports (port 0 = T raw, port 1 = RH raw; the server
// applies the datasheet conversions below).
//
// Everything here is platform-independent (host-testable); the only thing
// the port layer supplies is the I2C transfer itself.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>

namespace hdc3020 {

constexpr uint8_t  I2C_ADDR        = 0x44; // ADDR pins 00 (0x45..0x47 otherwise)
// Single-shot trigger, low-power mode 0 (highest accuracy), no clock stretch:
constexpr uint16_t CMD_MEASURE     = 0x2400;
constexpr uint16_t CMD_SOFT_RESET  = 0x30A2;
constexpr uint32_t MEASURE_TIME_MS = 13;   // LPM0 conversion, datasheet max

// Reply: [T msb][T lsb][CRC][RH msb][RH lsb][CRC]
constexpr size_t REPLY_LEN = 6;

// CRC-8, polynomial 0x31, init 0xFF (per datasheet; SHT3x-compatible).
inline uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

struct Sample {
  bool     valid;
  uint16_t rawT;    // port 0 payload word
  uint16_t rawRH;   // port 1 payload word
};

// Validates both CRCs; raw words go into the OpenLoRa buffer unconverted.
inline Sample parseReply(const uint8_t r[REPLY_LEN]) {
  Sample s{false, 0, 0};
  if (crc8(r, 2) != r[2] || crc8(r + 3, 2) != r[5]) return s;
  s.rawT  = static_cast<uint16_t>(r[0] << 8 | r[1]);
  s.rawRH = static_cast<uint16_t>(r[3] << 8 | r[4]);
  s.valid = true;
  return s;
}

// Server-side conversions (datasheet §7.3): kept here as the single source
// of truth; the node itself never needs floating point.
inline float toCelsius(uint16_t raw)  { return -45.0f + 175.0f * raw / 65535.0f; }
inline float toPercentRh(uint16_t raw) { return 100.0f * raw / 65535.0f; }

} // namespace hdc3020
