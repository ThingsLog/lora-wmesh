// W-Mesh boot console — platform-independent line parser.
//
// Enrolment workflow (bench, before the node is sealed):
//   1. connect USB-UART to LPUART1, reset the node, press any key within 3 s
//   2. `show`                        — current provisioning
//   3. `set window 12:00`            — THE anchor: window start in UTC
//   4. `set profile relay` / `set depth 1` / `set slot 3` / ...
//   5. `set time <unix-epoch-utc>`   — sets the RTC; do this LAST, right
//                                      before sealing (the first wake-up is
//                                      blind — no beacon heard yet — and runs
//                                      purely on this clock + window)
//   6. `save`                        — validate + write the flash page
//   7. `boot`                        — leave the console, start the schedule
//
// The parser is pure: it edits a Config in RAM and reports what the firmware
// must do (save/set-RTC/boot) via ConsoleAction. No I/O, no vendor calls —
// host-testable to the last branch.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "wmesh_config.h"
#include "lorawan.h"

namespace wmesh {

enum class ConsoleAction : uint8_t { NONE, SAVE, SETTIME, BOOT };

struct ConsoleResult {
  ConsoleAction action;
  uint32_t      timeEpoch;   // valid when action == SETTIME
};

namespace detail {
inline bool parseU32(const char* s, uint32_t& out) {
  if (s == nullptr || *s == '\0') return false;
  char* end = nullptr;
  const unsigned long v = std::strtoul(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = (uint32_t)v;
  return true;
}
// "HH:MM" (UTC) -> seconds after midnight
inline bool parseHhMm(const char* s, uint32_t& out) {
  if (s == nullptr || std::strlen(s) != 5 || s[2] != ':') return false;
  const int h = (s[0]-'0')*10 + (s[1]-'0');
  const int m = (s[3]-'0')*10 + (s[4]-'0');
  if (s[0]<'0'||s[0]>'9'||s[1]<'0'||s[1]>'9'||s[3]<'0'||s[3]>'9'||s[4]<'0'||s[4]>'9')
    return false;
  if (h > 23 || m > 59) return false;
  out = (uint32_t)h * 3600 + (uint32_t)m * 60;
  return true;
}
// exactly 2*n hex chars -> n bytes; returns false otherwise
inline bool parseHexBytes(const char* s, uint8_t* out, size_t n) {
  if (s == nullptr || std::strlen(s) != 2 * n) return false;
  for (size_t i = 0; i < 2 * n; ++i) {
    const char c = s[i];
    uint8_t v;
    if      (c >= '0' && c <= '9') v = (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') v = (uint8_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') v = (uint8_t)(c - 'A' + 10);
    else return false;
    if (i % 2 == 0) out[i / 2] = (uint8_t)(v << 4);
    else            out[i / 2] |= v;
  }
  return true;
}
// "a,b,c" -> array; returns count or -1 on error; "-" clears (count 0)
inline int parseList(const char* s, uint32_t* out, int maxN) {
  if (s == nullptr) return -1;
  if (std::strcmp(s, "-") == 0) return 0;
  int n = 0;
  char buf[128];
  std::strncpy(buf, s, sizeof buf - 1); buf[sizeof buf - 1] = '\0';
  for (char* tok = std::strtok(buf, ","); tok; tok = std::strtok(nullptr, ",")) {
    if (n >= maxN) return -1;
    uint32_t v;
    if (!parseU32(tok, v)) return -1;
    out[n++] = v;
  }
  return n;
}
} // namespace detail

inline void consoleShow(const Config& c, char* out, size_t cap) {
  const char* prof = c.profile == Profile::RELAY ? "relay"
                   : c.profile == Profile::STAKE ? "stake" : "leaf";
  int n = std::snprintf(out, cap,
    "id %u  profile %s  depth %u/%u  ports %u\n"
    "window %02u:%02u UTC (= beacon time, NOT power-on)  preroll %us (sub %u, parent sub %u)\n"
    "slot %u x %us  guard %us (+%us/day, max %u days)  acq %us\n"
    "freq %u  bfreq %u  prefixless %u  gnssbk %u\nphases",
    c.nodeId, prof, c.depth, c.meshH, c.ports,
    (unsigned)(c.windowUtcSec/3600), (unsigned)(c.windowUtcSec%3600/60),
    (unsigned)c.prerollSlotS, c.prerollSub, c.parentPrerollSub,
    c.txSlotIndex, (unsigned)c.txSlotDurS,
    (unsigned)c.beaconGuardS, (unsigned)c.driftSPerDay, (unsigned)c.maxFreerunDays,
    (unsigned)c.acqListenS,
    (unsigned)c.freqHz, (unsigned)c.beaconFreqHz, c.prefixless, c.gnssBackup);
  for (uint8_t i = 0; i < c.meshH && n > 0 && (size_t)n < cap; ++i)
    n += std::snprintf(out + n, cap - n, " %u", (unsigned)c.phaseDur[i]);
  if (n > 0 && (size_t)n < cap) {
    n += std::snprintf(out + n, cap - n, "\nwhitelist");
    if (c.whitelistLen == 0)
      n += std::snprintf(out + n, cap - n, " - (promiscuous)");
    else
      for (uint8_t i = 0; i < c.whitelistLen && (size_t)n < cap; ++i)
        n += std::snprintf(out + n, cap - n, " %u", c.whitelist[i]);
  }
}

// Appended to `show` when a key store is present: the ABP identity in full.
// Bench nodes are provisioned over a wired console — displaying the session
// keys here is the point (the operator must be able to read them back).
inline void consoleShowAbp(const lorawan::AbpKeys& k, char* out, size_t cap) {
  size_t n = std::strlen(out);
  if (!lorawan::keysValid(k)) {
    std::snprintf(out + n, cap - n, "\nabp - (raw OpenLoRa mode)");
    return;
  }
  n += std::snprintf(out + n, cap - n, "\nabp devaddr %08X  fcnt %u\nnwkskey ",
                     (unsigned)k.devAddr, (unsigned)k.fcntUp);
  for (int i = 0; i < 16 && n + 2 < cap; ++i)
    n += std::snprintf(out + n, cap - n, "%02X", k.nwkSKey[i]);
  n += std::snprintf(out + n, cap - n, "\nappskey ");
  for (int i = 0; i < 16 && n + 2 < cap; ++i)
    n += std::snprintf(out + n, cap - n, "%02X", k.appSKey[i]);
}

// Processes one line; edits `c` (and `k` if given); returns the action.
inline ConsoleResult consoleLine(Config& c, lorawan::AbpKeys* k,
                                 const char* line, char* out, size_t cap) {
  ConsoleResult r{ConsoleAction::NONE, 0};
  char buf[160];
  std::strncpy(buf, line ? line : "", sizeof buf - 1); buf[sizeof buf - 1] = '\0';
  char* cmd = std::strtok(buf, " \t\r\n");
  if (!cmd) { out[0] = '\0'; return r; }

  if (!std::strcmp(cmd, "help")) {
    std::snprintf(out, cap,
      "show | save | boot (= exit) | set time <epoch>\n"
      "set id|depth|h|ports|preroll|psub|ppsub|slot|slotdur|guard|acq|drift|freerun|freq|bfreq|prefixless|gnssbk <n>\n"
      "set profile leaf|relay|stake | set window HH:MM | set phases a,b,.. | set whitelist a,b,..|-\n"
      "ABP: set devaddr <hex8>|0 | set nwkskey <hex32> | set appskey <hex32> | set fcnt <n>");
    return r;
  }
  if (!std::strcmp(cmd, "show")) {
    consoleShow(c, out, cap);
    if (k) consoleShowAbp(*k, out, cap);
    return r;
  }
  if (!std::strcmp(cmd, "boot") || !std::strcmp(cmd, "exit")) {
    std::snprintf(out, cap, "booting");
    r.action = ConsoleAction::BOOT;
    return r;
  }
  if (!std::strcmp(cmd, "save")) {
    const char* err = configValidate(c);
    if (err) { std::snprintf(out, cap, "ERR %s", err); return r; }
    std::snprintf(out, cap, "OK saved");
    r.action = ConsoleAction::SAVE;
    return r;
  }
  if (std::strcmp(cmd, "set") != 0) { std::snprintf(out, cap, "ERR unknown cmd (try help)"); return r; }

  const char* key = std::strtok(nullptr, " \t\r\n");
  const char* val = std::strtok(nullptr, " \t\r\n");
  if (!key || !val) { std::snprintf(out, cap, "ERR set <key> <value>"); return r; }
  uint32_t v = 0;

  // ---- ABP session keys (stored beside the config; `save` writes both) ----
  if (!std::strcmp(key, "devaddr") || !std::strcmp(key, "nwkskey") ||
      !std::strcmp(key, "appskey") || !std::strcmp(key, "fcnt")) {
    if (!k) { std::snprintf(out, cap, "ERR no key store on this build"); return r; }
    if (!std::strcmp(key, "devaddr")) {
      uint8_t a[4];
      if (!std::strcmp(val, "0")) { k->devAddr = 0; std::snprintf(out, cap, "OK abp cleared (raw mode)"); return r; }
      if (!detail::parseHexBytes(val, a, 4)) { std::snprintf(out, cap, "ERR 8 hex digits (MSB first)"); return r; }
      k->devAddr = (uint32_t)a[0] << 24 | (uint32_t)a[1] << 16
                 | (uint32_t)a[2] << 8 | a[3];
      std::snprintf(out, cap, "OK devaddr %08X", (unsigned)k->devAddr);
      return r;
    }
    if (!std::strcmp(key, "fcnt")) {
      if (!detail::parseU32(val, v)) { std::snprintf(out, cap, "ERR number expected"); return r; }
      k->fcntUp = v;
      std::snprintf(out, cap, "OK fcnt %u", (unsigned)v);
      return r;
    }
    uint8_t* dst = !std::strcmp(key, "nwkskey") ? k->nwkSKey : k->appSKey;
    if (!detail::parseHexBytes(val, dst, 16)) { std::snprintf(out, cap, "ERR 32 hex digits"); return r; }
    std::snprintf(out, cap, "OK %s set", key);
    return r;
  }

  if (!std::strcmp(key, "time")) {
    if (!detail::parseU32(val, v)) { std::snprintf(out, cap, "ERR epoch seconds expected"); return r; }
    r.action = ConsoleAction::SETTIME; r.timeEpoch = v;
    std::snprintf(out, cap, "OK rtc <- %u", (unsigned)v);
    return r;
  }
  if (!std::strcmp(key, "window")) {
    if (!detail::parseHhMm(val, v)) { std::snprintf(out, cap, "ERR HH:MM expected"); return r; }
    c.windowUtcSec = v;
    std::snprintf(out, cap, "OK window %02u:%02u UTC", (unsigned)(v/3600), (unsigned)(v%3600/60));
    return r;
  }
  if (!std::strcmp(key, "profile")) {
    if      (!std::strcmp(val, "leaf"))  c.profile = Profile::LEAF;
    else if (!std::strcmp(val, "relay")) c.profile = Profile::RELAY;
    else if (!std::strcmp(val, "stake")) c.profile = Profile::STAKE;
    else { std::snprintf(out, cap, "ERR leaf|relay|stake"); return r; }
    std::snprintf(out, cap, "OK profile %s", val);
    return r;
  }
  if (!std::strcmp(key, "phases")) {
    uint32_t list[CFG_MAX_H];
    const int n = detail::parseList(val, list, CFG_MAX_H);
    if (n <= 0) { std::snprintf(out, cap, "ERR comma list of seconds"); return r; }
    for (int i = 0; i < n; ++i) c.phaseDur[i] = list[i];
    c.meshH = (uint8_t)n;   // phase count defines H
    std::snprintf(out, cap, "OK %d phases (h=%d)", n, n);
    return r;
  }
  if (!std::strcmp(key, "whitelist")) {
    uint32_t list[CFG_MAX_CHILDREN];
    const int n = detail::parseList(val, list, CFG_MAX_CHILDREN);
    if (n < 0) { std::snprintf(out, cap, "ERR up to 14 ids, or - to clear"); return r; }
    for (int i = 0; i < n; ++i) c.whitelist[i] = (uint8_t)list[i];
    c.whitelistLen = (uint8_t)n;
    std::snprintf(out, cap, n ? "OK %d children" : "OK promiscuous", n);
    return r;
  }

  if (!detail::parseU32(val, v)) { std::snprintf(out, cap, "ERR number expected"); return r; }
  if      (!std::strcmp(key, "id"))         c.nodeId        = (uint8_t)v;
  else if (!std::strcmp(key, "depth"))      c.depth         = (uint8_t)v;
  else if (!std::strcmp(key, "h"))          c.meshH         = (uint8_t)v;
  else if (!std::strcmp(key, "ports"))      c.ports         = (uint8_t)v;
  else if (!std::strcmp(key, "preroll"))    c.prerollSlotS  = v;
  else if (!std::strcmp(key, "psub"))       c.prerollSub    = (uint8_t)v;
  else if (!std::strcmp(key, "ppsub"))      c.parentPrerollSub = (uint8_t)v;
  else if (!std::strcmp(key, "slot"))       c.txSlotIndex   = (uint16_t)v;
  else if (!std::strcmp(key, "slotdur"))    c.txSlotDurS    = v;
  else if (!std::strcmp(key, "guard"))      c.beaconGuardS  = v;
  else if (!std::strcmp(key, "acq"))        c.acqListenS    = v;
  else if (!std::strcmp(key, "drift"))      c.driftSPerDay  = v;
  else if (!std::strcmp(key, "freerun"))    c.maxFreerunDays= v;
  else if (!std::strcmp(key, "freq"))       c.freqHz        = v;
  else if (!std::strcmp(key, "bfreq"))      c.beaconFreqHz  = v;
  else if (!std::strcmp(key, "prefixless")) c.prefixless    = (uint8_t)(v ? 1 : 0);
  else if (!std::strcmp(key, "gnssbk"))     c.gnssBackup    = (uint8_t)(v ? 1 : 0);
  else { std::snprintf(out, cap, "ERR unknown key (try help)"); return r; }
  std::snprintf(out, cap, "OK %s %u", key, (unsigned)v);
  return r;
}

// Key-store-less convenience overload (host tests, tools).
inline ConsoleResult consoleLine(Config& c, const char* line,
                                 char* out, size_t cap) {
  return consoleLine(c, nullptr, line, out, cap);
}

} // namespace wmesh
