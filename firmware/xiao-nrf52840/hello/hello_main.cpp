// Toolchain bring-up check: LED heartbeat + USB CDC counter. If this
// enumerates and blinks, the linker layout / SoftDevice / USB stack are fine
// and any failure lies in the W-Mesh application code.
// Build/flash:  pio run -e xiao-hello -t upload
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // pulls the USB device stack into the link

static uint32_t n = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);    // XIAO LEDs are active-low
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(800);
  Serial.print("hello ");
  Serial.println(n++);
}
