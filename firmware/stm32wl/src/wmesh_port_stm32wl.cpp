// W-Mesh port layer — SeeedStudio Wio-E5 mini (STM32WLE5JC), the field
// target. Implements core/wmesh_port.h over the STM32duino core:
//   radio   RadioLib STM32WLx — the SX126x-class radio on the same die,
//           reached over the internal SUBGHZ SPI; RF switch on PA4/PA5,
//           TCXO fed from PB0-VDD_TCXO at 1.7 V, high-power PA path
//   clock   the hardware RTC on the LSE crystal, running through STOP2 —
//           the window arithmetic reads a real UTC epoch
//   sleep   STM32LowPower deepSleep (STOP2, microamperes) in short chunks
//   storage EEPROM-emulated flash page (config / ABP keys / journal)
//   console USART1 through the board's CP2102 USB-UART bridge
//
// Differences from the XIAO bench port, on purpose:
//   - no GNSS here yet: repeater #1 carries no L76K (gnssbk comes later)
//   - no IWDG during bring-up; added after the first clean window
//
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <RadioLib.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <EEPROM.h>
#include "../../../core/wmesh_port.h"
#include "../../../core/gnss.h"

// BENCH EXPERIMENT parity with the XIAO fleet: hourly windows. All period
// arithmetic lives in this port; field value is 86400 (daily).
static constexpr uint32_t WINDOW_PERIOD_S = 3600;

// ---- GNSS (gnssbk backup clock): L76K on LPUART1, hard power gate ------------
// Wiring per docs/hardware-build.md: module TX -> PC0 (LPUART1 RX), module
// RX -> PC1 (LPUART1 TX); VCC through a high-side P-FET (AO3401 + 100k)
// driven from PB4 — gate LOW turns the receiver ON, HIGH/idle keeps it at
// ZERO microamperes (the XIAO taught us software standby is not a gate).
// If the bench wiring uses the module's STANDBY pin instead of a FET, flip
// GPS_GATE_ON/OFF below (STANDBY: HIGH = run, LOW = standby).
// Bench wiring: the L76K XIAO shield, jumpered over — VCC is hard on 3V3,
// control is the module's STANDBY pin (HIGH = run, LOW = standby, ~300 uA).
// For the field build swap to the P-FET gate (ON = LOW) per hardware-build.md.
static constexpr uint32_t PIN_GPS_GATE = PB4;
static constexpr int      GPS_GATE_ON  = HIGH;   // STANDBY pin: high = run
static constexpr int      GPS_GATE_OFF = LOW;
static constexpr uint32_t GNSS_SESSION_MS = 90u * 1000u;   // field: 900 s
static constexpr uint32_t GNSS_MIN_FIXES  = 20;
static HardwareSerial SerialGps(PC0, PC1);       // (RX, TX)

// ---- radio: RadioLib STM32WLx over the internal SUBGHZ SPI -------------------
static STM32WLx s_radio = new STM32WLx_Module();
// Wio-E5 RF front end: PA4 = RX enable, PA5 = TX enable (HP PA only).
static const uint32_t rfswitch_pins[] =
    {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
    {STM32WLx::MODE_RX,    {HIGH, LOW}},
    {STM32WLx::MODE_TX_HP, {LOW,  HIGH}},
    END_OF_MODE_TABLE,
};
static bool     s_radioUp   = false;
static uint32_t s_radioFreq = 0;
static volatile bool s_radioIrq = false;
static void onRadioIrq() { s_radioIrq = true; }

// ---- RTC-backed clock --------------------------------------------------------
static STM32RTC& s_rtc = STM32RTC::getInstance();
static uint32_t s_windowUtcSec     = 43200;
static uint32_t s_windowStartEpoch = 0;
static bool     s_portUp           = false;

// ---- battery ring: one sample every 15 min, sent once per window -------------
static constexpr uint32_t SAMPLE_PERIOD_S = 900;
static constexpr uint16_t RING_CAP = 96;
static uint16_t s_ring[RING_CAP];
static uint16_t s_ringN = 0;
static uint32_t s_nextSampleEpoch = 0;

static void samplerTick() {
  const uint32_t e = s_rtc.getEpoch();
  if (e < s_nextSampleEpoch) return;
  s_nextSampleEpoch = e + SAMPLE_PERIOD_S;
  const uint16_t mv = port::readBatteryMv();
  if (s_ringN < RING_CAP) s_ring[s_ringN++] = mv;
  else {
    memmove(s_ring, s_ring + 1, (RING_CAP - 1) * sizeof(uint16_t));
    s_ring[RING_CAP - 1] = mv;
  }
}

// ---- bench trace (console is a real UART: always writable) -------------------
static void traceFrame(const char* tag, const uint8_t* d, size_t n) {
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

// ---- one-time init on the first port call ------------------------------------
// ---- crash-surviving journal (EEPROM offset map below) -----------------------
struct Journal { uint32_t boots; uint8_t event; uint32_t wsec;
                 uint8_t prevEvent; uint32_t prevWsec; };
static Journal s_jrn{};
static const char* JRN_EVENTS[] = {"?", "BOOT", "WINDOW_START", "WINDOW_DONE",
                                   "SLEEPING", "WOKE"};
enum { JEV_BOOT = 1, JEV_WSTART = 2, JEV_WDONE = 3, JEV_SLEEP = 4,
       JEV_WAKE = 5 };

// EEPROM-emulated flash page layout (one 2 KB page, buffered writes):
//   [0..127]    wmesh::Config image
//   [192..239]  lorawan ABP key image
//   [256..269]  journal (14 B)
static constexpr uint32_t EE_CFG  = 0;
static constexpr uint32_t EE_KEYS = 192;
static constexpr uint32_t EE_JRN  = 256;

static bool blobLoad(uint32_t off, uint8_t* img, size_t len) {
  eeprom_buffer_fill();
  for (size_t i = 0; i < len; ++i)
    img[i] = eeprom_buffered_read_byte(off + i);
  return true;      // validity is the caller's CRC/magic check
}

static bool blobSave(uint32_t off, const uint8_t* img, size_t len) {
  eeprom_buffer_fill();
  for (size_t i = 0; i < len; ++i)
    eeprom_buffered_write_byte(off + i, img[i]);
  eeprom_buffer_flush();                       // one page erase per save
  return true;
}

static void journalSave() {
  uint8_t b[14];
  memcpy(b, &s_jrn.boots, 4); b[4] = s_jrn.event; memcpy(b + 5, &s_jrn.wsec, 4);
  b[9] = s_jrn.prevEvent; memcpy(b + 10, &s_jrn.prevWsec, 4);
  blobSave(EE_JRN, b, sizeof b);
}

static void journalEvent(uint8_t ev) {
  s_jrn.event = ev;
  s_jrn.wsec  = port::windowClockSec();
  journalSave();
}

static void portInit() {
  if (s_portUp) return;
  s_portUp = true;
  s_rtc.setClockSource(STM32RTC::LSE_CLOCK);   // the 32 kHz crystal
  s_rtc.begin();
  LowPower.begin();
  // journal: preserve what the previous life left, then record this boot
  uint8_t b[14];
  blobLoad(EE_JRN, b, sizeof b);
  memcpy(&s_jrn.boots, b, 4); s_jrn.event = b[4]; memcpy(&s_jrn.wsec, b + 5, 4);
  if (s_jrn.boots == 0xFFFFFFFFu) s_jrn = Journal{};   // fresh chip
  s_jrn.prevEvent = s_jrn.event;
  s_jrn.prevWsec  = s_jrn.wsec;
  ++s_jrn.boots;
  journalEvent(JEV_BOOT);
  // the L76K powers up RUNNING (~25 mA): put it to sleep from the very
  // first moment — it wakes only inside gnssDisciplineRtc()
  pinMode(PIN_GPS_GATE, OUTPUT);
  digitalWrite(PIN_GPS_GATE, GPS_GATE_OFF);
}

void journalText(char* out, size_t cap) {
  snprintf(out, cap,
           "boot #%u; before this boot: %s at w+%u; current: %s at w+%u",
           (unsigned)s_jrn.boots,
           s_jrn.prevEvent < 6 ? JRN_EVENTS[s_jrn.prevEvent] : "?",
           (unsigned)s_jrn.prevWsec,
           s_jrn.event < 6 ? JRN_EVENTS[s_jrn.event] : "?",
           (unsigned)s_jrn.wsec);
}

// ---- enrolment console / provisioning ----------------------------------
bool consoleAttached(uint32_t ms) {
  portInit();
  if (ms < 10000) ms = 10000;                  // bench budget, as on the XIAO
  Serial.print("W-Mesh: any key for console...\r\n");
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available()) return true;
    delay(10);
  }
  return false;
}

int uartReadLine(char* buf, size_t cap) {
  size_t n = 0;
  for (;;) {
    while (!Serial.available()) delay(5);
    const int c = Serial.read();
    if (c < 0) continue;
    if (c == '\r' || c == '\n') {
      Serial.print("\r\n");
      buf[n] = '\0';
      return (int)n;
    }
    if ((c == 0x08 || c == 0x7F) && n > 0) {
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
  for (const char* p = s; *p; ++p) {
    if (*p == '\n') Serial.write('\r');
    Serial.write((uint8_t)*p);
  }
}

bool configLoad(uint8_t* img, size_t len) {
  portInit();
  return blobLoad(EE_CFG, img, len);
}
bool configSave(const uint8_t* img, size_t len) { return blobSave(EE_CFG, img, len); }
bool keysLoad(uint8_t* img, size_t len)  { portInit(); return blobLoad(EE_KEYS, img, len); }
bool keysSave(const uint8_t* img, size_t len) { return blobSave(EE_KEYS, img, len); }

// ---- clock / window ---------------------------------------------------
void rtcSetUtc(uint32_t unixEpochUtc) {
  s_rtc.setEpoch(unixEpochUtc);
}

void timeConfigure(uint32_t windowUtcSec) {
  s_windowUtcSec = windowUtcSec;
  const uint32_t e      = s_rtc.getEpoch();
  const uint32_t anchor = windowUtcSec % WINDOW_PERIOD_S;
  const uint32_t since  =
      (e % WINDOW_PERIOD_S + WINDOW_PERIOD_S - anchor) % WINDOW_PERIOD_S;
  s_windowStartEpoch = e - since;
}

uint32_t windowClockSec() {
  return s_rtc.getEpoch() - s_windowStartEpoch;
}

void rtcSetWindowClock(uint32_t secondsFromWindowStart) {
  s_windowStartEpoch = s_rtc.getEpoch() - secondsFromWindowStart;
}

void sleepUntil(uint32_t secondsFromWindowStart) {
  const uint32_t now = windowClockSec();
  uint32_t targetEpoch;
  bool nextWindow = false;
  if (secondsFromWindowStart > now) {
    targetEpoch = s_windowStartEpoch + secondsFromWindowStart;
  } else if (secondsFromWindowStart == now) {
    targetEpoch = s_rtc.getEpoch() + 1;        // coarse 1 s tick
  } else {
    targetEpoch = s_windowStartEpoch + WINDOW_PERIOD_S + secondsFromWindowStart;
    nextWindow = true;
  }
  if (targetEpoch > s_rtc.getEpoch() + 5) {
    Serial.print("SLEEP -> w+");
    Serial.print(secondsFromWindowStart);
    Serial.println(nextWindow ? "s (next window)" : "s");
    Serial.flush();                            // let the UART drain first
  }
  // STOP2 in short chunks: the RTC keeps time, each wakeup re-aims from the
  // real epoch, and no single sleep can cost more than 25 s if anything in
  // the wake path ever misbehaves (the XIAO lost-wakeup lesson).
  for (;;) {
    const uint32_t e = s_rtc.getEpoch();
    if (e >= targetEpoch) break;
    samplerTick();
    uint32_t chunk = targetEpoch - e;
    if (chunk > 25) chunk = 25;
    LowPower.deepSleep(chunk * 1000);
  }
  samplerTick();
  if (nextWindow) {
    s_windowStartEpoch += WINDOW_PERIOD_S;
    journalEvent(JEV_WAKE);
  }
}

// gnssbk duty: powered ONLY on a missed beacon. Run the averaging session,
// discipline the RTC from the (absolute UTC) fixes, gate the power back off.
// Returns false when no acceptable fix set was collected — the caller falls
// back to freerun on the crystal.
// GNSS time discipline for the duty-cycled window. Design notes (L76K + the
// SeeedStudio XIAO breakout, confirmed against the module docs):
//   * NO PMTK — the L76K ignores PMTK; power state is pins only.
//   * Backup / warm start is HARDWARE: the breakout carries its own small
//     rechargeable cell on V_BCKP that keeps the RTC and ephemeris alive at a
//     few microamperes while VCC is gated off, so the NEXT session is a warm
//     start (seconds) rather than a cold one. We therefore gate VCC fully off
//     between sessions (GPS_GATE_OFF) and let that cell retain the almanac —
//     nothing to wire, nothing to configure, settings are not lost.
//   * EARLY EXIT: the session stops the instant it has enough accepted fixes
//     for a trustworthy clock, so the module is powered only as long as a fix
//     actually takes. A warm start reaches GNSS_MIN_FIXES in seconds, which is
//     where the window-time energy is saved (the module draws ~41 mA while on,
//     so its ON-TIME is the budget — the MCU idling beside it is noise).
//   * NON-BLOCKING at heart: the acquisition is a resumable drain-and-parse
//     step (gnssDrain) so it never busy-spins; the MCU idles between the
//     once-a-second NMEA sentences and the caller stays responsive.
// A decoupling bulk cap (100 uF + 100 nF) at the module's VCC/GND is required
// to ride out the acquisition current spikes — without it they brown the rail.
namespace {
  gnss::SessionAvg s_gnssAvg;
  gnss::GgaFix     s_gnssLast{};
  uint32_t         s_gnssLastFixEpoch = 0;
  char             s_gnssLine[128];
  size_t           s_gnssLineN = 0;
}

// Drain whatever NMEA is buffered and fold GGA fixes into the running average.
// Bounded line buffer; returns immediately when the UART is empty (resumable).
static void gnssDrain() {
  while (SerialGps.available()) {
    const char c = (char)SerialGps.read();
    if (c == '\n' || s_gnssLineN + 1 >= sizeof s_gnssLine) {
      s_gnssLine[s_gnssLineN] = '\0';
      s_gnssLineN = 0;
      const gnss::GgaFix fx = gnss::parseGga(s_gnssLine);
      if (fx.valid && s_gnssAvg.add(fx)) {
        s_gnssLast = fx;
        s_gnssLastFixEpoch = s_rtc.getEpoch();
      }
    } else if (c != '\r') {
      s_gnssLine[s_gnssLineN++] = c;
    }
  }
}

bool gnssDisciplineRtc() {
  pinMode(PIN_GPS_GATE, OUTPUT);
  digitalWrite(PIN_GPS_GATE, GPS_GATE_ON);      // VCC gate / STANDBY -> run
  SerialGps.begin(9600);
  s_gnssAvg.reset();
  s_gnssLast = gnss::GgaFix{};
  s_gnssLastFixEpoch = 0;
  s_gnssLineN = 0;
  Serial.println("GNSS session start");
  const uint32_t deadline = millis() + GNSS_SESSION_MS;
  bool enough = false;
  while ((int32_t)(deadline - millis()) > 0) {
    gnssDrain();
    if (s_gnssAvg.count() >= GNSS_MIN_FIXES) { enough = true; break; }  // warm-start early exit
    delay(10);                                   // idle between once-a-second sentences
  }
  SerialGps.end();
  digitalWrite(PIN_GPS_GATE, GPS_GATE_OFF);      // VCC off; on-board cell keeps ephemeris warm
  // Tri-state the UART lines: with the main VCC gated off but V_BCKP still
  // alive, a driven TX pin would back-power the module's dead domain through
  // its RX protection diode. Hi-Z until the next session re-opens the port.
  pinMode(PC0, INPUT);
  pinMode(PC1, INPUT);
  Serial.print("GNSS session: ");
  Serial.print(s_gnssAvg.count());
  Serial.println(enough ? " fixes (early exit)" : " fixes (timeout)");
  if (!enough) return false;
  // absolute time discipline: seconds-of-day is all the window math uses
  const uint32_t sinceFixS = s_rtc.getEpoch() - s_gnssLastFixEpoch;
  s_rtc.setEpoch(s_gnssLast.utcDaySec + sinceFixS);
  timeConfigure(s_windowUtcSec);
  return true;
}

// ---- radio -------------------------------------------------------------
static uint32_t s_cadScans = 0, s_cadHits = 0;

void radioInit(uint32_t freqHz) {
  if (s_jrn.event != JEV_WSTART) journalEvent(JEV_WSTART);
  if (!s_radioUp) {
    s_radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    const int16_t st = s_radio.begin(freqHz / 1e6, 125.0, 8, 5,
                                     0x34 /* LoRaWAN sync */, 14, 12,
                                     1.7 /* E5 TCXO volts */);
    if (st != RADIOLIB_ERR_NONE) {
      Serial.print("radio begin err ");
      Serial.println(st);
      return;
    }
    s_radio.setDio1Action(onRadioIrq);
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
  Serial.println();
  return ok;
}

bool radioCadOnce() {
  // Bounded CAD, as on the bench: start the scan, wait for the internal
  // radio interrupt (the WL has no external DIO1 pin), never block forever.
  if (!s_radioUp) return false;
  s_radioIrq = false;
  if (s_radio.startChannelScan() != RADIOLIB_ERR_NONE) return false;
  ++s_cadScans;
  const uint32_t t0 = millis();
  while (!s_radioIrq && millis() - t0 < 200) delay(1);
  const int16_t res = s_radioIrq ? s_radio.getChannelScanResult()
                                 : RADIOLIB_ERR_UNKNOWN;
  s_radio.standby();
  if (res == RADIOLIB_LORA_DETECTED) { ++s_cadHits; return true; }
  return false;
}

int radioReceiveOne(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  if (!s_radioUp) return -1;
  s_radioIrq = false;
  if (s_radio.startReceive() != RADIOLIB_ERR_NONE) return -1;
  const uint32_t t0 = millis();
  int got = -1;
  while (millis() - t0 < timeoutMs) {
    if (s_radioIrq) {
      const size_t n = s_radio.getPacketLength();
      if (n > 0 && n <= maxLen &&
          s_radio.readData(buf, n) == RADIOLIB_ERR_NONE) {
        got = (int)n;
        traceFrame("RX", buf, n);
        Serial.print("  RSSI ");
        Serial.print(s_radio.getRSSI());
        Serial.print(" SNR ");
        Serial.println(s_radio.getSNR());
      }
      break;
    }
    delay(2);
  }
  s_radio.standby();
  return got;
}

void radioSleep() {
  if (s_cadScans) {
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

// ---- sensors / battery --------------------------------------------------
uint16_t readSensorPort(uint8_t port, uint16_t* out, uint16_t maxN) {
  if (port > 1 || maxN == 0) return 0;         // relay: battery only
  samplerTick();
  if (s_ringN == 0) {
    s_ring[0] = readBatteryMv();
    s_ringN = 1;
  }
  const uint16_t n = s_ringN < maxN ? s_ringN : maxN;
  memcpy(out, s_ring + (s_ringN - n), n * sizeof(uint16_t));
  s_ringN = 0;
  return n;
}

uint16_t readBatteryMv() {
  // VDD IS the battery (no regulator): the VREFINT channel against the
  // factory calibration word gives VDD directly — zero external parts.
  analogReadResolution(12);
  const uint32_t raw = analogRead(AVREF);
  const uint16_t cal = *reinterpret_cast<const uint16_t*>(0x1FFF75AAu);
  return raw ? (uint16_t)((3000u * cal) / raw) : 0;
}

} // namespace port
