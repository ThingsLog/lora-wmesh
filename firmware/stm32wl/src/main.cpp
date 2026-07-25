// W-Mesh field node — Wio-E5 mini (STM32WLE5JC) entry point.
// The entire node behaviour lives in the portable core (core/wmesh_node.h);
// this target contributes only the port layer (wmesh_port_stm32wl.cpp):
// internal SUBGHZ radio via RadioLib, hardware RTC through STOP2 sleep.
// Provision over the boot console on the USB-UART, same dialogue as every
// other target:  set id / set profile relay / ... / set time / save / boot
//
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include "../../../core/wmesh_node.h"

static wmesh::NodeRuntime rt;

void setup() {
  Serial.begin(115200);
  wmesh::nodeMain(rt);   // never returns
}

void loop() {}
