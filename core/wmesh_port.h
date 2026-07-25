// W-Mesh port layer — the ONLY surface the portable core needs.
// One implementation per target:
//   firmware/stm32wl        STM32WLE5 (LoRa-E5): radio over the internal
//                           SUBGHZ SPI via the ST CubeWL driver (SUBGRF_*),
//                           STOP2 between duties (~1 uA), RTC alarm wake-ups
//   firmware/xiao-nrf52840  XIAO nRF52840 + Wio-SX1262 bench node: RadioLib
//                           over SPI, FreeRTOS tick sleep (bench-grade power)
//   test/node_sim_test.cpp  host mock: simulated clock + scripted radio, so
//                           the whole window state machine runs in a unit test
//
// Power posture (field targets): deep sleep between duties, RTC alarm
// wake-ups, GNSS power-gated by GPIO and read only during a fix.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>

namespace port {

// ---- enrolment console / provisioning ----------------------------------
// True if any byte arrives on LPUART1 within `ms` after reset — the operator
// is asking for the boot console (core/wmesh_console.h drives the dialogue).
bool consoleAttached(uint32_t ms);
int  uartReadLine(char* buf, size_t cap);   // blocking, echo, CR/LF-terminated
void uartWrite(const char* s);
// Provisioning page (one flash page, image built by wmesh::configPack).
bool configLoad(uint8_t* img, size_t len);
bool configSave(const uint8_t* img, size_t len);
// LoRaWAN ABP session blob (lorawan::keysPack, 48 B): second page/file.
// Saved by the console AND once per window after TX — FCntUp must survive
// resets, the network side tracks it monotonically.
bool keysLoad(uint8_t* img, size_t len);
bool keysSave(const uint8_t* img, size_t len);
// Human-readable postmortem journal line (boot count, last event before the
// most recent reset, current state) for the console `jrn` command.
void journalText(char* out, size_t cap);
// Console `set time <epoch>`: sets the RTC in UTC. This is the enrolment
// anchor — the first wake-up is blind (no beacon heard yet) and runs purely
// on this clock plus cfg.windowUtcSec (Livingston: 43200 = 12:00 UTC,
// i.e. 15:00 Bulgarian summer time).
void rtcSetUtc(uint32_t unixEpochUtc);
// Bind the window origin so windowClockSec()/sleepUntil() speak window time.
void timeConfigure(uint32_t windowUtcSec);

// ---- clock / window ---------------------------------------------------
// Seconds since window start (RTC-derived); valid only while awake.
uint32_t windowClockSec();
// Program the RTC alarm and enter STOP2; returns on alarm.
void sleepUntil(uint32_t secondsFromWindowStart);
// Re-anchor the window clock from a received beacon: declare "now" to be
// `secondsFromWindowStart` (the known slot time of the parent's re-broadcast).
// This is the ONLY time source relays and telemetry leaves have — and the
// only one they need: drift must hold across a single window (90 ms at
// 100 ppm), not across a day.
void rtcSetWindowClock(uint32_t secondsFromWindowStart);
// Discipline the RTC from a GNSS fix — STAKE PROFILE ONLY. On stakes the
// receiver is already powered for the position payload, so time comes free;
// relays and telemetry leaves carry no GNSS hardware at all.
// Implementation: gate L76K power ON (hard P-FET, no backup domain — cold
// start ~30 s is absorbed by the session), feed LPUART lines through
// core/gnss.h (parseGga -> SessionAvg, ~1 fix/s for ~15 min), apply the
// utcDaySec of a valid fix to the RTC, packPosition() into the position
// port buffer, gate power OFF.
bool gnssDisciplineRtc();

// ---- radio -------------------------------------------------------------
// On STM32WL these are thin wrappers over the ST radio driver:
//   init      -> SUBGRF_Init, SetStandby, SetPacketType(LORA),
//                SetModulationParams(SF8/125k/CR45), SetPacketParams(...)
//   transmit  -> SetPayload + SetTx; extended preamble for CAD catchability
//   cadOnce   -> SetCadParams(CAD_ON_8_SYMB, det_peak, det_min, CAD_ONLY)
//                + SetCad; returns true if activity was detected
//   receiveOne-> SetRx with timeout; returns frame or times out
void  radioInit(uint32_t freqHz);
bool  radioTransmit(const uint8_t* frame, size_t len, uint16_t preambleSyms);
bool  radioCadOnce();
int   radioReceiveOne(uint8_t* buf, size_t maxLen, uint32_t timeoutMs);
void  radioSleep(); // cold sleep between duties (SUBGRF_SetSleep)

// ---- persistence --------------------------------------------------------
// Readings ring buffer lives in RAM2 (retained in STOP2); flash is used
// only by the future multi-packet backlog extension.
// PORT CONVENTION (all profiles): the LAST port (cfg.ports - 1) is the
// battery, sampled at the same 15-min tick as the sensors and sent in the
// same window — e.g. HDC3020 node, ports=3: 0 = T raw, 1 = RH raw, 2 = mV.
uint16_t readSensorPort(uint8_t port, uint16_t* out, uint16_t maxN);
// Battery voltage in millivolts. On STM32WL via the internal VREFINT ADC
// channel — VDD IS the battery (no regulator, no divider, zero sleep current):
//   vdd_mV = 3000 * VREFINT_CAL / adc_raw   (calibration word at 0x1FFF75AA)
// On the XIAO bench node via the on-board VBAT divider (P0.31, gated by P0.14).
// Note for LiSOCl2: the discharge curve is flat until the end-of-life cliff,
// so the value is a cold-sag/passivation/cliff indicator, not a fuel gauge.
uint16_t readBatteryMv();

} // namespace port
