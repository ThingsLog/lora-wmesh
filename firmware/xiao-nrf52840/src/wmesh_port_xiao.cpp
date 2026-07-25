// W-Mesh port layer — Seeed XIAO nRF52840 + Wio-SX1262 bench node.
//
// Implements core/wmesh_port.h over the Arduino core (Seeed fork of the
// Adafruit nRF52 BSP) and RadioLib. This target exists for LAB BRING-UP of
// the mesh logic on real radios: the clock is millis()-derived and sleep is
// a FreeRTOS delay loop, so the power posture is bench-grade by design —
// the deployment-grade STOP2/RTC discipline lives in the STM32WL target.
//
// Bare-leaf hardware: no GNSS, no HDC3020. The node's only measurement is
// its own battery voltage (the "last OpenLoRa port on every role" rule with
// ports = 1), sampled every 15 min into a RAM ring and sent in the window.
//
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // pulls the USB device stack into the link
#include <RadioLib.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "../../../core/wmesh_port.h"
#include "../../../core/gnss.h"

using namespace Adafruit_LittleFS_Namespace;

// ---- Wio-SX1262 for XIAO wiring (standalone SKU 113010003 / kit 102010710) --
// B2B connector; NOT the same map as the ESP32S3 kit's 30-pin module.
static constexpr uint8_t PIN_LORA_CS   = 4;   // D4
static constexpr uint8_t PIN_LORA_DIO1 = 1;   // D1
static constexpr uint8_t PIN_LORA_BUSY = 3;   // D3
static constexpr uint8_t PIN_LORA_RST  = 2;   // D2
static constexpr uint8_t PIN_LORA_RXEN = 5;   // D5; TX side switched by DIO2

// XIAO on-board battery divider (1M / 510k), gated by P0.14.
#ifndef PIN_VBAT
#define PIN_VBAT (32)
#endif
#ifndef VBAT_ENABLE
#define VBAT_ENABLE (14)
#endif

// L76K GNSS module for XIAO (stacked): NMEA on Serial1 (D6/D7, 9600 baud),
// STANDBY on D0 (HIGH = run, LOW = standby). Bench session is shortened;
// the field value is the paper's 15-minute averaging session.
static constexpr uint8_t  PIN_GPS_STANDBY   = 0;
static constexpr uint32_t GNSS_SESSION_MS   = 90u * 1000u;   // field: 900 s
static constexpr uint32_t GNSS_MIN_FIXES    = 20;
static uint16_t s_posWords[gnss::POS_WORDS];
static bool     s_posValid = false;

// Constructed lazily in radioInit() — keeps hardware access (and heap use)
// out of static initialisation entirely.
static Module*  s_module    = nullptr;
static SX1262*  s_radioP    = nullptr;
static bool     s_radioUp   = false;
static uint32_t s_radioFreq = 0;
#define s_radio (*s_radioP)

// ---- 64-bit uptime (millis wraps at 49.7 days) -------------------------------
static uint64_t millis64() {
  static uint32_t hi = 0, last = 0;
  const uint32_t now = millis();
  if (now < last) ++hi;
  last = now;
  return ((uint64_t)hi << 32) | now;
}

// ---- battery ring: one sample every 15 min, sent once per window -------------
static constexpr uint32_t SAMPLE_PERIOD_MS = 900u * 1000u;
static constexpr uint16_t RING_CAP = 96;
static uint16_t s_ring[RING_CAP];
static uint16_t s_ringN = 0;
static uint64_t s_nextSampleMs = 0;

static void samplerTick() {
  if (millis64() < s_nextSampleMs) return;
  s_nextSampleMs = millis64() + SAMPLE_PERIOD_MS;
  const uint16_t mv = port::readBatteryMv();
  if (s_ringN < RING_CAP) s_ring[s_ringN++] = mv;
  else {                                      // ring full: drop the oldest
    memmove(s_ring, s_ring + 1, (RING_CAP - 1) * sizeof(uint16_t));
    s_ring[RING_CAP - 1] = mv;
  }
}

// ---- bench trace: every frame in and out, visible on the USB console --------
static void traceFrame(const char* tag, const uint8_t* d, size_t n) {
  if (!Serial) return;
  Serial.print(tag);
  Serial.print(" w+");
  Serial.print(port::windowClockSec());
  Serial.print("s len ");
  Serial.print((unsigned)n);
  Serial.print(" :");
  char b[4];
  for (size_t i = 0; i < n && i < 40; ++i) {
    snprintf(b, sizeof b, " %02X", d[i]);
    Serial.print(b);
  }
  if (n > 40) Serial.print(" ..");
}

namespace port {

static void wdtFeed();   // defined with the journal/watchdog block below

// ---- enrolment console / provisioning ----------------------------------
bool consoleAttached(uint32_t ms) {
  // Native USB: enumeration plus the operator's terminal need a moment, so
  // the keypress budget restarts when the HOST opens the port, not at reset.
  // Bench policy: allow ≥10 s for the host to show up at all — that is what
  // lets tools/hiltest.py catch the console right after a DFU reboot.
  if (ms < 10000) ms = 10000;
  uint64_t deadline = millis64() + ms;
  bool prompted = false;
  while (millis64() < deadline) {
    wdtFeed();
    if (Serial && !prompted) {
      prompted = true;
      Serial.print("W-Mesh: any key for console...\r\n");
      deadline = millis64() + 5000;
    }
    if (Serial && Serial.available()) return true;
    delay(10);
  }
  return false;
}

int uartReadLine(char* buf, size_t cap) {
  size_t n = 0;
  for (;;) {
    while (!Serial.available()) { wdtFeed(); delay(5); }
    const int c = Serial.read();
    if (c < 0) continue;
    if (c == '\r' || c == '\n') {
      Serial.print("\r\n");
      buf[n] = '\0';
      return (int)n;
    }
    if ((c == 0x08 || c == 0x7F) && n > 0) {   // backspace
      --n;
      Serial.print("\b \b");
      continue;
    }
    if (n + 1 < cap && c >= 0x20 && c < 0x7F) {
      buf[n++] = (char)c;
      Serial.write((uint8_t)c);
    }
  }
}

void uartWrite(const char* s) {
  if (!Serial) return;
  // terminals expect CRLF; the core writes bare \n
  for (const char* p = s; *p; ++p) {
    if (*p == '\n') Serial.write('\r');
    Serial.write((uint8_t)*p);
  }
}

static void ensureFs() {
  static bool fsUp = false;
  if (!fsUp) { InternalFS.begin(); fsUp = true; }
}

static bool blobLoad(const char* path, uint8_t* img, size_t len) {
  ensureFs();
  File f(InternalFS);
  if (!f.open(path, FILE_O_READ)) return false;
  const int n = f.read(img, (uint16_t)len);
  f.close();
  return n == (int)len;
}

static bool blobSave(const char* path, const uint8_t* img, size_t len) {
  ensureFs();
  InternalFS.remove(path);                     // FILE_O_WRITE appends
  File f(InternalFS);
  if (!f.open(path, FILE_O_WRITE)) return false;
  const size_t n = f.write(img, (uint16_t)len);
  f.close();
  return n == len;
}

// ---- crash-surviving journal: who did what, when, across power cycles -----
// 9 bytes in LittleFS: [boots u32][event u8][window sec u32]. Written at the
// few per-window state changes, printed at every boot — the postmortem story
// a silent battery-powered night would otherwise take with it.
struct Journal { uint32_t boots; uint8_t event; uint32_t wsec;
                 uint8_t prevEvent; uint32_t prevWsec; };
static Journal s_jrn{};
static const char* JRN_EVENTS[] = {"?", "BOOT", "WINDOW_START", "WINDOW_DONE",
                                   "SLEEPING", "WOKE"};
enum { JEV_BOOT = 1, JEV_WSTART = 2, JEV_WDONE = 3, JEV_SLEEP = 4,
       JEV_WAKE = 5 };

// Hardware watchdog: a hung loop anywhere reboots the node instead of
// killing it silently (the stake slept through two windows without a single
// crash — a hang in the long tickless sleep). 120 s timeout, kept running
// during sleep; every legitimate wait loop below feeds it.
static void wdtFeed() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }
static void wdtStart() {
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 120u * 32768u;                // 120 s
  NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
  NRF_WDT->TASKS_START = 1;
}

static void journalSave() {
  uint8_t b[14];
  memcpy(b, &s_jrn.boots, 4); b[4] = s_jrn.event; memcpy(b + 5, &s_jrn.wsec, 4);
  b[9] = s_jrn.prevEvent; memcpy(b + 10, &s_jrn.prevWsec, 4);
  blobSave("/wmesh.jrn", b, sizeof b);
}

static void journalEvent(uint8_t ev) {
  s_jrn.event = ev;
  s_jrn.wsec  = port::windowClockSec();
  journalSave();
}

static void journalBoot() {
  static bool done = false;
  if (done) return;
  done = true;
  // The L76K powers up RUNNING (~25 mA) and we used to quiet it only after
  // the first GNSS session — a battery-killer overnight. Standby from the
  // very first port call; harmless no-op on boards without the GPS stack.
  pinMode(PIN_GPS_STANDBY, OUTPUT);
  digitalWrite(PIN_GPS_STANDBY, LOW);
  wdtStart();
  uint8_t b[14];
  if (blobLoad("/wmesh.jrn", b, sizeof b)) {
    memcpy(&s_jrn.boots, b, 4); s_jrn.event = b[4]; memcpy(&s_jrn.wsec, b + 5, 4);
    s_jrn.prevEvent = b[9]; memcpy(&s_jrn.prevWsec, b + 10, 4);
  } else if (blobLoad("/wmesh.jrn", b, 9)) {   // migrate the 9-byte format
    memcpy(&s_jrn.boots, b, 4); s_jrn.event = b[4]; memcpy(&s_jrn.wsec, b + 5, 4);
  }
  // a reboot must never destroy the story: what we found becomes "previous"
  s_jrn.prevEvent = s_jrn.event;
  s_jrn.prevWsec  = s_jrn.wsec;
  if (Serial) {
    Serial.print("JOURNAL: boot #");
    Serial.print(s_jrn.boots + 1);
    Serial.print(", last event ");
    Serial.print(s_jrn.event < 5 ? JRN_EVENTS[s_jrn.event] : "?");
    Serial.print(" at w+");
    Serial.println(s_jrn.wsec);
  }
  ++s_jrn.boots;
  journalEvent(JEV_BOOT);
}

bool configLoad(uint8_t* img, size_t len) {
  journalBoot();                               // first port call after reset
  return blobLoad("/wmesh.cfg", img, len);
}

void journalText(char* out, size_t cap) {
  snprintf(out, cap,
           "boot #%u; before this boot: %s at w+%u; current: %s at w+%u",
           (unsigned)s_jrn.boots,
           s_jrn.prevEvent < 5 ? JRN_EVENTS[s_jrn.prevEvent] : "?",
           (unsigned)s_jrn.prevWsec,
           s_jrn.event < 5 ? JRN_EVENTS[s_jrn.event] : "?",
           (unsigned)s_jrn.wsec);
}
bool configSave(const uint8_t* img, size_t len) { return blobSave("/wmesh.cfg", img, len); }
bool keysLoad(uint8_t* img, size_t len)   { return blobLoad("/wmesh.keys", img, len); }
bool keysSave(const uint8_t* img, size_t len) { return blobSave("/wmesh.keys", img, len); }

// ---- clock / window ---------------------------------------------------
// millis64-derived window clock. Good for a bench: drift only has to hold
// across one window, and re-anchoring from the beacon happens daily anyway.
static uint32_t s_epochRef   = 0;   // UTC epoch at s_epochRefMs
static uint64_t s_epochRefMs = 0;
static uint64_t s_windowStartMs = 0;
static uint32_t s_windowUtcSec  = 43200;  // remembered for GNSS discipline

void rtcSetUtc(uint32_t unixEpochUtc) {
  s_epochRef   = unixEpochUtc;
  s_epochRefMs = millis64();
}

void timeConfigure(uint32_t windowUtcSec) {
  s_windowUtcSec = windowUtcSec;
  const uint64_t now = millis64();
  const uint32_t epochNow =
      s_epochRef + (uint32_t)((now - s_epochRefMs) / 1000u);
  const uint32_t daySec = epochNow % 86400u;
  const uint32_t sinceWindow = (daySec + 86400u - windowUtcSec) % 86400u;
  s_windowStartMs = now - (uint64_t)sinceWindow * 1000u;
}

uint32_t windowClockSec() {
  return (uint32_t)((millis64() - s_windowStartMs) / 1000u);
}

void rtcSetWindowClock(uint32_t secondsFromWindowStart) {
  s_windowStartMs = millis64() - (uint64_t)secondsFromWindowStart * 1000u;
  if (Serial) {
    Serial.print("ANCHOR w+");
    Serial.println(secondsFromWindowStart);
  }
}

void sleepUntil(uint32_t secondsFromWindowStart) {
  const uint32_t now = windowClockSec();
  uint64_t targetMs;
  bool nextWindow = false;
  if (secondsFromWindowStart > now) {
    targetMs = s_windowStartMs + (uint64_t)secondsFromWindowStart * 1000u;
  } else if (secondsFromWindowStart == now) {
    targetMs = millis64() + 1000u;             // coarse 1 s tick
  } else {                                     // behind us -> next window
    targetMs = s_windowStartMs + (86400u + secondsFromWindowStart) * 1000ull;
    nextWindow = true;
  }
  if (Serial && targetMs > millis64() + 5000) { // long sleeps only, not ticks
    Serial.print("SLEEP -> w+");
    Serial.print(secondsFromWindowStart);
    Serial.println(nextWindow ? "s (next window)" : "s");
  }
  if (nextWindow) journalEvent(JEV_SLEEP);
  // Block in LARGE chunks: FreeRTOS tickless idle then holds the nRF52840 in
  // System ON sleep (RTC on LFCLK keeps time) instead of waking every 20 ms.
  // The only scheduled duty inside a sleep is the 15-min battery sample, so
  // each delay() runs to whichever comes first — the sample or the target.
  for (;;) {
    wdtFeed();
    const uint64_t now = millis64();
    if (now >= targetMs) break;
    samplerTick();
    uint64_t next = targetMs;
    if (s_nextSampleMs > now && s_nextSampleMs < next) next = s_nextSampleMs;
    uint64_t chunk = next - millis64();
    // short blocks: a single vTaskDelay never exceeds 60 s, so one misbehaving
    // tickless period cannot cost more than a minute, and the watchdog stays
    // fed (the overnight lost-wakeup was a single multi-hour delay)
    if (chunk > 60000u) chunk = 60000u;
    if (chunk > 0 && chunk < (uint64_t)~0u) delay((uint32_t)chunk);
  }
  samplerTick();
  if (nextWindow) {
    s_windowStartMs += 86400000ull;
    journalEvent(JEV_WAKE);                     // proof the sleep RETURNED
  }
}

// STAKE duty: run the averaging session on the stacked L76K, discipline the
// window clock from the (absolute UTC) fixes, and pack the daily position
// into the port-0 payload words. Returns false when no acceptable fix set
// was collected (indoors, antenna off) — the caller falls back to freerun.
bool gnssDisciplineRtc() {
  pinMode(PIN_GPS_STANDBY, OUTPUT);
  digitalWrite(PIN_GPS_STANDBY, HIGH);         // wake the receiver
  Serial1.begin(9600);
  gnss::SessionAvg avg;
  gnss::GgaFix last{};
  char line[128];
  size_t n = 0;
  uint64_t lastFixAtMs = 0;
  const uint64_t deadline = millis64() + GNSS_SESSION_MS;
  if (Serial) Serial.println("GNSS session start");
  while (millis64() < deadline) {
    wdtFeed();
    while (Serial1.available()) {
      const char c = (char)Serial1.read();
      if (c == '\n' || n + 1 >= sizeof line) {
        line[n] = '\0';
        n = 0;
        const gnss::GgaFix fx = gnss::parseGga(line);
        if (fx.valid && avg.add(fx)) {
          last = fx;
          lastFixAtMs = millis64();
        }
      } else if (c != '\r') {
        line[n++] = c;
      }
    }
    delay(5);
  }
  Serial1.end();
  digitalWrite(PIN_GPS_STANDBY, LOW);          // back to standby
  if (Serial) {
    Serial.print("GNSS session: ");
    Serial.print(avg.count());
    Serial.println(" fixes");
  }
  if (avg.count() < GNSS_MIN_FIXES) return false;
  // absolute time discipline: the last fix carries UTC seconds-of-day, which
  // is all the window arithmetic ever uses (epoch day number is irrelevant)
  const uint32_t sinceFixS = (uint32_t)((millis64() - lastFixAtMs) / 1000u);
  rtcSetUtc(last.utcDaySec + sinceFixS);
  timeConfigure(s_windowUtcSec);
  gnss::packPosition(avg, s_posWords);
  s_posValid = true;
  return true;
}

// ---- radio (RadioLib SX1262) -------------------------------------------
// One modulation profile for both duties (Stage-1 bench simplification):
// SF8 / 125 kHz / CR 4/5, LoRaWAN public sync word, +14 dBm. The Wio-SX1262
// switches its RF path with DIO2 (TX) and the RXEN line (RX); the module's
// TCXO runs from DIO3 at 1.8 V.
void radioInit(uint32_t freqHz) {
  if (s_jrn.event != JEV_WSTART) journalEvent(JEV_WSTART);
  if (s_radioP == nullptr) {
    s_module = new Module(PIN_LORA_CS, PIN_LORA_DIO1,
                          PIN_LORA_RST, PIN_LORA_BUSY);
    s_radioP = new SX1262(s_module);
  }
  if (!s_radioUp) {
    const int16_t st = s_radio.begin(freqHz / 1e6, 125.0, 8, 5,
                                     0x34 /* LoRaWAN sync */, 14, 12, 1.8);
    if (st != RADIOLIB_ERR_NONE) {
      if (Serial) { Serial.print("radio begin err "); Serial.println(st); }
      return;
    }
    s_radio.setDio2AsRfSwitch(true);
    s_radio.setRfSwitchPins(PIN_LORA_RXEN, RADIOLIB_NC);
    s_radio.setCurrentLimit(140);
    s_radioUp = true;
  } else {
    s_radio.standby();
    if (freqHz != s_radioFreq) s_radio.setFrequency(freqHz / 1e6);
  }
  s_radioFreq = freqHz;
}

bool radioTransmit(const uint8_t* frame, size_t len, uint16_t preambleSyms) {
  if (!s_radioUp) return false;
  s_radio.setPreambleLength(preambleSyms);
  const bool ok =
      s_radio.transmit(const_cast<uint8_t*>(frame), len) == RADIOLIB_ERR_NONE;
  traceFrame(ok ? "TX" : "TX FAIL", frame, len);
  if (Serial) Serial.println();
  return ok;
}

static uint32_t s_cadScans = 0, s_cadHits = 0;

bool radioCadOnce() {
  // NOT RadioLib's blocking scanChannel(): its wait-for-IRQ loop has no
  // timeout, and one missed CAD_DONE wedges the node forever (observed live
  // mid-listen-phase). Bounded poll on DIO1 instead, like radioReceiveOne.
  if (!s_radioUp) return false;
  if (s_radio.startChannelScan() != RADIOLIB_ERR_NONE) return false;
  ++s_cadScans;
  bool done = false;
  const uint64_t deadline = millis64() + 200;  // CAD at SF8 is ~3 ms
  while (millis64() < deadline) {
    if (digitalRead(PIN_LORA_DIO1) == HIGH) { done = true; break; }
    delay(1);
  }
  const int16_t res = done ? s_radio.getChannelScanResult()
                           : RADIOLIB_ERR_UNKNOWN;
  s_radio.standby();
  if (res == RADIOLIB_LORA_DETECTED) { ++s_cadHits; return true; }
  return false;
}

int radioReceiveOne(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  if (!s_radioUp) return -1;
  if (s_radio.startReceive() != RADIOLIB_ERR_NONE) return -1;
  const uint64_t deadline = millis64() + timeoutMs;
  int got = -1;
  while (millis64() < deadline) {
    wdtFeed();
    if (digitalRead(PIN_LORA_DIO1) == HIGH) {  // RxDone
      const size_t n = s_radio.getPacketLength();
      if (n > 0 && n <= maxLen &&
          s_radio.readData(buf, n) == RADIOLIB_ERR_NONE) {
        got = (int)n;
        traceFrame("RX", buf, n);
        if (Serial) {
          Serial.print("  RSSI ");
          Serial.print(s_radio.getRSSI());
          Serial.print(" SNR ");
          Serial.println(s_radio.getSNR());
        }
      }
      break;
    }
    delay(2);
  }
  s_radio.standby();
  return got;
}

void radioSleep() {
  if (Serial && s_cadScans) {                  // bench diagnostic per window
    Serial.print("CAD: ");
    Serial.print(s_cadScans);
    Serial.print(" scans, ");
    Serial.print(s_cadHits);
    Serial.println(" hits");
  }
  s_cadScans = s_cadHits = 0;
  if (s_radioUp) s_radio.sleep();
  journalEvent(JEV_WDONE);
}

// ---- persistence / sensors ----------------------------------------------
// Port map on this hardware (the battery is always the LAST port):
//   bare leaf  (ports = 1): port 0 = battery
//   stake      (ports = 2): port 0 = position words from the day's GNSS
//                           session, port 1 = battery
static uint16_t readBatteryRing(uint16_t* out, uint16_t maxN) {
  samplerTick();
  if (s_ringN == 0) {                          // never miss the daily frame
    s_ring[0] = port::readBatteryMv();
    s_ringN = 1;
  }
  const uint16_t n = s_ringN < maxN ? s_ringN : maxN;
  memcpy(out, s_ring + (s_ringN - n), n * sizeof(uint16_t));
  s_ringN = 0;                                 // day's buffer handed over
  return n;
}

uint16_t readSensorPort(uint8_t port, uint16_t* out, uint16_t maxN) {
  if (port == 0 && s_posValid) {               // stake: the day's mean position
    if (maxN < gnss::POS_WORDS) return 0;
    memcpy(out, s_posWords, sizeof s_posWords);
    s_posValid = false;                        // handed over, next session refills
    return gnss::POS_WORDS;
  }
  // Everything else is the battery — including port 0 on a relay whose
  // gnssbk session found no fix: its battery must never go silent for that.
  if (port == 0 || port == 1) return readBatteryRing(out, maxN);
  return 0;
}

uint16_t readBatteryMv() {
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);              // gate the divider on
  delay(1);
  analogReference(AR_INTERNAL);                // 3.6 V full scale
  analogReadResolution(12);
  const uint32_t raw = analogRead(PIN_VBAT);
  pinMode(VBAT_ENABLE, INPUT);                 // gate off, zero standing drain
  const uint32_t pinMv = raw * 3600u / 4095u;
  return (uint16_t)(pinMv * 1510u / 510u);     // undo the 1M/510k divider
}

} // namespace port
