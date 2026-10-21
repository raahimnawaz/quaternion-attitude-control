// Each firmware build is the sketch text included verbatim into its own
// namespace. Every header it pulls in is included first, so its own
// #includes are no-ops.
#include "firmware.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Arduino.h"
#include "Wire.h"

namespace fw_now {
#include "../../sketch.ino"
}
#undef UNWINDING_DEMO

namespace fw_naive {
#define UNWINDING_DEMO 1
#include "../../sketch.ino"
}
#undef UNWINDING_DEMO

namespace fw_legacy {
#include LEGACY_SKETCH
}

// Cycle counts from `make -C test before-after` (simavr, ATmega2560 at
// 16 MHz, I2C stubbed to zero time). The current sketch never has a "plain"
// iteration there -- a frame is always in flight -- so plain is estimated: the
// baseline's, less the ~321 cycles the q_est normalisation no longer spends
// (from the per-call bench; the chunk iteration measured 314 less), plus what
// the Mahony integral term added to the chunk iteration (measured, 40221 before).
static constexpr sim::CpuCycles CPU_NOW = {
    36420 - 321 + (42471 - 40221), 36420 - 321 + (42471 - 40221) + (60657 - 42471), 42471, 60657, 83482, 0, 425};
static constexpr sim::CpuCycles CPU_LEGACY = {36420, 0, 0, 0, 0, 233066, 425};

const Firmware FW_NOW = {
    "now", fw_now::setup, fw_now::loop, &fw_now::q_est.w, &fw_now::gyro_bias.x, &fw_now::Ki_imu,
    [] { return fw_now::display.getBuffer(); }, fw_now::print_fixed4, CPU_NOW};

const Firmware FW_NAIVE = {
    "naive", fw_naive::setup, fw_naive::loop, &fw_naive::q_est.w, &fw_naive::gyro_bias.x, &fw_naive::Ki_imu,
    [] { return fw_naive::display.getBuffer(); }, fw_naive::print_fixed4, CPU_NOW};

const Firmware FW_LEGACY = {
    "baseline", fw_legacy::setup, fw_legacy::loop, &fw_legacy::q_est.w, nullptr, nullptr,
    [] { return fw_legacy::display.getBuffer(); }, nullptr, CPU_LEGACY};
