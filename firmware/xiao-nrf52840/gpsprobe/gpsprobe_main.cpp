// GNSS bring-up probe — L76K module for XIAO (stacked): wakes the receiver
// and mirrors the raw NMEA stream to the USB console, prefixing each GGA
// with the parsed fix from core/gnss.h. No W-Mesh logic, no radio.
// Indoors you still see sentences (empty fix fields) — that alone proves
// the wiring; take the stack near a window for an actual fix.
// Build/flash:  pio run -e xiao-gpsprobe -t upload
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // pulls the USB device stack into the link
#include "../../../core/gnss.h"

static const uint8_t PIN_GPS_STANDBY = 0;   // D0: LOW = standby, HIGH = run
static char line[128];
static size_t n = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000) delay(10);
  pinMode(PIN_GPS_STANDBY, OUTPUT);
  digitalWrite(PIN_GPS_STANDBY, HIGH);      // wake the receiver
  Serial1.begin(9600);                      // L76K default on D6/D7
  Serial.println("GNSS probe: mirroring NMEA from Serial1 (9600)");
}

void loop() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (c == '\n' || n + 1 >= sizeof line) {
      line[n] = '\0';
      if (n > 6) {
        Serial.println(line);
        const gnss::GgaFix fx = gnss::parseGga(line);
        if (fx.valid) {
          Serial.print(">>> FIX sats ");
          Serial.print(fx.nSats);
          Serial.print(" hdop*100 ");
          Serial.print(fx.hdopX100);
          Serial.print(" lat*1e7 ");
          Serial.print(fx.latE7);
          Serial.print(" lon*1e7 ");
          Serial.print(fx.lonE7);
          Serial.print(" alt*10m ");
          Serial.print(fx.altDm);
          Serial.print(" utcDaySec ");
          Serial.println(fx.utcDaySec);
        }
      }
      n = 0;
    } else if (c != '\r') {
      line[n++] = c;
    }
  }
  delay(2);
}
