// W-Mesh Stage-2 node firmware — STM32WLE5 (LoRa-E5 class) entry point.
//
// The entire node behaviour lives in the portable core (core/wmesh_node.h);
// this target contributes only the port layer. Generate the CubeWL project
// (STM32CubeMX -> LoRa-E5 / STM32WLE5JC, SUBGHZ + RTC + LPUART), then
// implement core/wmesh_port.h over the generated drivers in wmesh_port.cpp.
//
// SPDX-License-Identifier: MIT
#include "../../../core/wmesh_node.h"

static wmesh::NodeRuntime rt;

int main() {
  wmesh::nodeMain(rt);
}
