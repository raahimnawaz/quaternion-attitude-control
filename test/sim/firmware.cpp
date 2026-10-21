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
// iteration there -- a frame is always in flight -- so plain is the baseline's
// less the ~321 cycles the q_est normalisation no longer spends (estimated
// from the per-call bench; the chunk iteration measured 314 less).
static constexpr sim::CpuCycles CPU_NOW = {
    36420 - 321, 36420 - 321 + (58452 - 40221), 40221, 58452, 82545, 0, 425};
static constexpr sim::CpuCycles CPU_LEGACY = {36420, 0, 0, 0, 0, 233066, 425};

const Firmware FW_NOW = {
    "now", fw_now::setup, fw_now::loop, &fw_now::q_est.w,
    [] { return fw_now::display.getBuffer(); }, fw_now::print_fixed4, CPU_NOW};

const Firmware FW_NAIVE = {
    "naive", fw_naive::setup, fw_naive::loop, &fw_naive::q_est.w,
    [] { return fw_naive::display.getBuffer(); }, fw_naive::print_fixed4, CPU_NOW};

const Firmware FW_LEGACY = {
    "baseline", fw_legacy::setup, fw_legacy::loop, &fw_legacy::q_est.w,
    [] { return fw_legacy::display.getBuffer(); }, nullptr, CPU_LEGACY};
