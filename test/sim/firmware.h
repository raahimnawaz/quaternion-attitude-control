#pragma once
#include <stdint.h>

#include "sim.h"

struct Firmware {
  const char *name;
  void (*setup)();
  void (*loop)();
  float *q_est;  // w, x, y, z
  uint8_t *(*framebuffer)();
  void (*print_fixed4)(float);  // null where the firmware has none
  sim::CpuCycles cpu;
};

extern const Firmware FW_NOW;     // sketch.ino
extern const Firmware FW_NAIVE;   // sketch.ino with UNWINDING_DEMO=1
extern const Firmware FW_LEGACY;  // sketch.ino at the Makefile's BASELINE commit
