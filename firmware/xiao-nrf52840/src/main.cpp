// W-Mesh bench node — XIAO nRF52840 + Wio-SX1262 entry point.
// The node behaviour is entirely core/wmesh_node.h; this target contributes
// only the port layer (wmesh_port_xiao.cpp). Provision as a bare leaf:
//   set profile leaf / set ports 1 / set depth ... / set time ... / save
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
