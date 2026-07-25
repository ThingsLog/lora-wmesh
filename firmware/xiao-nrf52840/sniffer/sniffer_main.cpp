// W-Mesh air sniffer / test gateway — continuous RX, prints EVERY LoRa frame
// it hears on the USB console: timestamp, length, RSSI/SNR, hex dump, plus a
// decoded mesh header / Class B beacon when the frame parses as one.
//
// Dedicate one bench board to this while the others run the mesh: it is the
// bench-side view of the paper's listening phase, minus the duty cycling.
// Keys (any time):
//   b  listen on the beacon channel (869.525)     d  back to the branch channel
//   g  toggle GATEWAY mode: emit a valid Class B beacon on the beacon channel
//      every 120 s, then return to listening on the branch channel — a mesh
//      node in its acquisition listen catches it, ANCHORs, and ~26 s later its
//      battery frame shows up here. The bench stand-in for gateway + Starlink.
//
// Build/flash:  pio run -e xiao-sniffer -t upload
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // pulls the USB device stack into the link
#include <RadioLib.h>
#include "../../../core/wmesh_core.h"
#include "../../../core/wmesh_beacon.h"

// Wio-SX1262 for XIAO (kit SKU 102010710) — same map as the mesh port.
static SX1262* radio;
static const float FREQ_DATA_MHZ   = 868.1;
static const float FREQ_BEACON_MHZ = 869.525;
static float freqMHz = FREQ_DATA_MHZ;
static uint32_t nRx = 0;
static bool gwMode = false;
static uint32_t nextBeaconMs = 0;
static const uint32_t BEACON_PERIOD_MS = 120000;

static void tune(float mhz) {
  freqMHz = mhz;
  radio->standby();
  radio->setFrequency(mhz);
  radio->startReceive();
  Serial.print("listening on ");
  Serial.print(mhz, 3);
  Serial.println(" MHz  (SF8 / 125 kHz / CR 4/5, sync 0x34)");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000) delay(10);
  radio = new SX1262(new Module(/*cs*/4, /*dio1*/1, /*rst*/2, /*busy*/3));
  const int16_t st = radio->begin(FREQ_DATA_MHZ, 125.0, 8, 5, 0x34, 14, 12, 1.8);
  if (st != RADIOLIB_ERR_NONE) {
    for (;;) { Serial.print("radio begin err "); Serial.println(st); delay(2000); }
  }
  radio->setDio2AsRfSwitch(true);
  radio->setRfSwitchPins(/*rxEn*/5, RADIOLIB_NC);
  radio->setCurrentLimit(140);
  Serial.println("W-Mesh sniffer");
  tune(FREQ_DATA_MHZ);
}

static void emitBeacon() {
  uint8_t b[wmesh::BEACON_LEN] = {0};
  // the mesh only reads the time field; GPS epoch offset is arbitrary here
  wmesh::restampBeacon(b, 1400000000u + millis() / 1000u);
  radio->standby();
  radio->setFrequency(FREQ_BEACON_MHZ);
  radio->setPreambleLength(10);                // Class B short preamble
  const int16_t st = radio->transmit(b, sizeof b);
  Serial.print("TX BEACON  t+");
  Serial.print(millis() / 1000);
  Serial.println(st == RADIOLIB_ERR_NONE ? "s" : "s  FAILED");
  tune(freqMHz);                               // back to listening
}

void loop() {
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'b') tune(FREQ_BEACON_MHZ);
    if (c == 'd') tune(FREQ_DATA_MHZ);
    if (c == 'g') {
      gwMode = !gwMode;
      nextBeaconMs = millis();                 // first beacon right away
      Serial.println(gwMode ? "GATEWAY mode ON (beacon every 120 s)"
                            : "GATEWAY mode OFF");
    }
  }
  if (gwMode && (int32_t)(millis() - nextBeaconMs) >= 0) {
    nextBeaconMs += BEACON_PERIOD_MS;
    emitBeacon();
  }
  if (digitalRead(/*dio1*/1) != HIGH) { delay(2); return; }

  uint8_t buf[wmesh::MAX_FRAME_LEN];
  const size_t n = radio->getPacketLength();
  const int16_t st = (n > 0 && n <= sizeof buf) ? radio->readData(buf, n)
                                                : RADIOLIB_ERR_UNKNOWN;
  const float rssi = radio->getRSSI(), snr = radio->getSNR();
  radio->startReceive();                       // re-arm before printing

  Serial.print("#");
  Serial.print(++nRx);
  Serial.print("  t+");
  Serial.print(millis() / 1000);
  Serial.print("s  len ");
  Serial.print((unsigned)n);
  Serial.print("  RSSI ");
  Serial.print(rssi, 1);
  Serial.print("  SNR ");
  Serial.print(snr, 1);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("  read err ");
    Serial.println(st);
    return;
  }
  Serial.print("\n  ");
  char b[4];
  for (size_t i = 0; i < n; ++i) {
    snprintf(b, sizeof b, "%02X ", buf[i]);
    Serial.print(b);
  }
  Serial.println();
  // a 17-byte frame that passes the NetCommon CRC is a Class B beacon
  if (n == wmesh::BEACON_LEN) {
    const wmesh::Beacon bc = wmesh::parseBeacon(buf, n);
    if (bc.valid) {
      Serial.print("  BEACON  timeGps ");
      Serial.println(bc.timeGps);
      return;
    }
  }
  // otherwise show it as a mesh frame: 3-byte header + OpenLoRa payload
  if (n > wmesh::MESH_HDR_LEN) {
    const wmesh::MeshHeader h = wmesh::unpackHeader(buf);
    Serial.print("  MESH  origin ");
    Serial.print(h.origin);
    Serial.print("  seq ");
    Serial.print(h.seq);
    Serial.print("  depth ");
    Serial.print(h.depth);
    Serial.print("  ttl ");
    Serial.println(h.ttl);
  }
}
