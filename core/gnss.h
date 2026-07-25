// GNSS averaging session — platform-independent (STAKE profile only).
//
// Policy (docs/hardware-build.md §2a): hard power gate, no backup domain,
// cold start every day (~30 s, budget 60 s), then ~15 min of 1 fix/s
// tracking. This header is everything between the UART and the radio:
// NMEA GGA parsing, fix-quality filtering, streaming average (integer
// arithmetic only — no floating point on the node), and packing the daily
// position into OpenLoRa port words. The port layer just gates power and
// feeds lines in.
//
// Averaging pushes a consumer receiver's metre-level scatter toward the
// decimetre regime (paper §VII); the daily mean is the payload.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>

namespace gnss {

// --- NMEA plumbing -----------------------------------------------------------
// XOR of everything between '$' and '*', compared to the two hex digits after.
inline bool nmeaChecksumOk(const char* s) {
  if (s == nullptr || *s != '$') return false;
  uint8_t x = 0;
  const char* p = s + 1;
  for (; *p && *p != '*'; ++p) x ^= static_cast<uint8_t>(*p);
  if (*p != '*' || !p[1] || !p[2]) return false;
  auto hex = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  const int hi = hex(p[1]), lo = hex(p[2]);
  if (hi < 0 || lo < 0) return false;
  return x == static_cast<uint8_t>(hi << 4 | lo);
}

struct GgaFix {
  bool     valid;
  uint32_t utcDaySec;   // seconds after 00:00 UTC — disciplines the RTC
  int32_t  latE7;       // degrees x 1e-7 (~1 cm), signed (S negative)
  int32_t  lonE7;       // degrees x 1e-7, signed (W negative)
  int16_t  altDm;       // altitude in decimetres
  uint8_t  quality;     // GGA fix quality (0 = none)
  uint8_t  nSats;
  uint16_t hdopX100;
};

namespace detail {
// "ddmm.mmmmm" (lat, degDigits=2) or "dddmm.mmmmm" (lon, degDigits=3) -> E7
inline bool coordE7(const char* f, int degDigits, int32_t& out) {
  if (std::strlen(f) < static_cast<size_t>(degDigits) + 2) return false;
  int32_t deg = 0;
  for (int i = 0; i < degDigits; ++i) {
    if (f[i] < '0' || f[i] > '9') return false;
    deg = deg * 10 + (f[i] - '0');
  }
  // minutes as an integer scaled 1e5 ("38.12345" -> 3812345)
  int64_t min1e5 = 0;
  int digits = 0, seenDot = 0, frac = 0;
  for (const char* p = f + degDigits; *p; ++p) {
    if (*p == '.') { seenDot = 1; continue; }
    if (*p < '0' || *p > '9') return false;
    if (seenDot && frac >= 5) break;      // ignore precision beyond 1e-5 min
    min1e5 = min1e5 * 10 + (*p - '0');
    if (seenDot) ++frac; else ++digits;
  }
  if (digits != 2) return false;
  for (; frac < 5; ++frac) min1e5 *= 10;  // right-pad to exactly 1e5 scale
  // deg fraction = minutes/60; in E7 units: min1e5 * 1e7/(60*1e5) = min1e5*5/3
  out = static_cast<int32_t>(deg * 10000000LL + (min1e5 * 5) / 3);
  return true;
}
} // namespace detail

// Parses a GGA sentence (any talker: $GPGGA/$GNGGA/...). Returns valid=false
// on checksum failure, malformed fields, or quality 0.
inline GgaFix parseGga(const char* s) {
  GgaFix r{};
  if (!nmeaChecksumOk(s)) return r;
  if (std::strlen(s) < 6 || std::strncmp(s + 3, "GGA", 3) != 0) return r;
  // split a working copy on commas
  char buf[128];
  std::strncpy(buf, s, sizeof buf - 1); buf[sizeof buf - 1] = '\0';
  char* star = std::strchr(buf, '*'); if (star) *star = '\0';
  const char* f[15] = {nullptr};
  int n = 0;
  for (char* p = buf; p && n < 15; ) {
    f[n++] = p;
    p = std::strchr(p, ',');
    if (p) *p++ = '\0';
  }
  if (n < 10) return r;
  // f[1] hhmmss.ss  f[2] lat f[3] N/S  f[4] lon f[5] E/W
  // f[6] quality    f[7] nSats        f[8] hdop  f[9] alt (m)
  if (std::strlen(f[1]) < 6) return r;
  r.utcDaySec = static_cast<uint32_t>((f[1][0]-'0')*10 + (f[1][1]-'0')) * 3600
              + static_cast<uint32_t>((f[1][2]-'0')*10 + (f[1][3]-'0')) * 60
              + static_cast<uint32_t>((f[1][4]-'0')*10 + (f[1][5]-'0'));
  if (!detail::coordE7(f[2], 2, r.latE7)) return r;
  if (!detail::coordE7(f[4], 3, r.lonE7)) return r;
  if (f[3][0] == 'S') r.latE7 = -r.latE7;
  if (f[5][0] == 'W') r.lonE7 = -r.lonE7;
  r.quality  = static_cast<uint8_t>(std::atoi(f[6]));
  r.nSats    = static_cast<uint8_t>(std::atoi(f[7]));
  r.hdopX100 = static_cast<uint16_t>(std::atof(f[8]) * 100.0 + 0.5);
  r.altDm    = static_cast<int16_t>(std::atof(f[9]) * 10.0);
  if (r.quality == 0) return r;
  r.valid = true;
  return r;
}

// --- streaming average --------------------------------------------------------
// Integer accumulators only; a 15-min session at 1 fix/s is ~900 samples,
// int64 sums of E7 coordinates cannot overflow for centuries of sessions.
class SessionAvg {
public:
  // Acceptance filter: a usable fix with sane geometry.
  static constexpr uint8_t  MIN_SATS  = 5;
  static constexpr uint16_t MAX_HDOP  = 300;   // 3.00

  void reset() { *this = SessionAvg(); }
  bool add(const GgaFix& fx) {
    if (!fx.valid || fx.nSats < MIN_SATS || fx.hdopX100 > MAX_HDOP) return false;
    latSum_ += fx.latE7; lonSum_ += fx.lonE7; altSum_ += fx.altDm;
    hdopSum_ += fx.hdopX100;
    ++n_;
    return true;
  }
  uint16_t count() const { return n_; }
  int32_t  latE7() const { return n_ ? static_cast<int32_t>(latSum_ / n_) : 0; }
  int32_t  lonE7() const { return n_ ? static_cast<int32_t>(lonSum_ / n_) : 0; }
  int16_t  altDm() const { return n_ ? static_cast<int16_t>(altSum_ / n_) : 0; }
  uint16_t hdopX100() const { return n_ ? static_cast<uint16_t>(hdopSum_ / n_) : 0; }
private:
  int64_t latSum_ = 0, lonSum_ = 0, altSum_ = 0, hdopSum_ = 0;
  uint16_t n_ = 0;
};

// --- position -> OpenLoRa port words -------------------------------------------
// 8 uint16 words = one 6+16 = 22-byte OpenLoRa frame for the position port:
//   [lat hi][lat lo][lon hi][lon lo][altDm][nFixes][hdopX100][spare]
constexpr size_t POS_WORDS = 8;

inline void packPosition(const SessionAvg& a, uint16_t w[POS_WORDS]) {
  const uint32_t lat = static_cast<uint32_t>(a.latE7());
  const uint32_t lon = static_cast<uint32_t>(a.lonE7());
  w[0] = static_cast<uint16_t>(lat >> 16);
  w[1] = static_cast<uint16_t>(lat);
  w[2] = static_cast<uint16_t>(lon >> 16);
  w[3] = static_cast<uint16_t>(lon);
  w[4] = static_cast<uint16_t>(a.altDm());
  w[5] = a.count();
  w[6] = a.hdopX100();
  w[7] = 0;
}
// Server-side reference decoder (also used by the tests).
inline void unpackPosition(const uint16_t w[POS_WORDS],
                           int32_t& latE7, int32_t& lonE7,
                           int16_t& altDm, uint16_t& nFixes) {
  latE7 = static_cast<int32_t>(static_cast<uint32_t>(w[0]) << 16 | w[1]);
  lonE7 = static_cast<int32_t>(static_cast<uint32_t>(w[2]) << 16 | w[3]);
  altDm = static_cast<int16_t>(w[4]);
  nFixes = w[5];
}

} // namespace gnss
