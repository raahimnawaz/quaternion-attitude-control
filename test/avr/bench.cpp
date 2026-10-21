// Cycle counts for the firmware's CPU work on an ATmega2560 at 16 MHz, run
// under simavr. Bus time is excluded (see Wire.h); the host SIL adds it back.
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "Wire.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

BenchBus bench_bus;
TwoWire Wire;

struct CountingSerial : public Print {
  uint16_t bytes;
  void begin(unsigned long baud) { Serial.begin(baud); }
  size_t write(uint8_t c) override { bytes++; return Serial.write(c); }
  using Print::write;
};
CountingSerial bench_serial;

#define Serial bench_serial
#include SKETCH
#undef Serial

volatile uint16_t t1_hi;
ISR(TIMER1_OVF_vect) { t1_hi++; }

static uint32_t cycles() {
  uint8_t s = SREG;
  cli();
  uint16_t lo = TCNT1, hi = t1_hi;
  if ((TIFR1 & _BV(TOV1)) && lo < 0x8000) hi++;
  SREG = s;
  return ((uint32_t)hi << 16) | lo;
}

volatile float sink;
static float xs[64];
__attribute__((noinline)) float rs_bits(float x) { return invSqrt(x); }
__attribute__((noinline)) float rs_refined(float x) {  // the two-step variant 73c03f0 used for q_est
  float y = invSqrt(x);
  return y * (1.5f - 0.5f * x * y * y);
}
__attribute__((noinline)) float rs_libm(float x) { return 1.0f / sqrtf(x); }
__attribute__((noinline)) float rs_none(float x) { return x; }
__attribute__((noinline)) float f_sin(float x) { return sin(x - 1.0f); }
__attribute__((noinline)) float f_cos(float x) { return cos(x - 1.0f); }
__attribute__((noinline)) float f_atan2(float x) { return atan2(x - 1.0f, x); }
__attribute__((noinline)) float f_asin(float x) { return asin(x - 1.0f); }
__attribute__((noinline)) float f_mul(float x) { return x * 1.37f; }
__attribute__((noinline)) float f_div(float x) { return 1.37f / x; }

template <float (*Fn)(float)>
static uint32_t per_call() {
  uint32_t t0 = cycles();
  for (uint8_t r = 0; r < 16; r++)
    for (uint8_t i = 0; i < 64; i++) sink = Fn(xs[i]);
  return (cycles() - t0) / 1024;
}

struct Stat {
  uint32_t n, sum, lo, hi;
  void add(uint32_t c) {
    if (!n || c < lo) lo = c;
    if (c > hi) hi = c;
    n++;
    sum += c;
  }
  void print(const char *name) {
    if (!n) return;
    Serial.print(name); Serial.print(' ');
    Serial.print(n); Serial.print(' ');
    Serial.print(sum / n); Serial.print(' ');
    Serial.print(lo); Serial.print(' ');
    Serial.println(hi);
  }
};

int main() {
  init();
  TCCR1A = 0;
  TCCR1B = _BV(CS10);
  TIMSK1 = _BV(TOIE1);
  setup();

  for (uint8_t i = 0; i < 64; i++) xs[i] = 0.9f + 0.2f * i / 63.0f;
  uint32_t base = per_call<rs_none>();
  Serial.print(F("rsqrt_bits ")); Serial.println(per_call<rs_bits>() - base);
  Serial.print(F("rsqrt_bits_2nr ")); Serial.println(per_call<rs_refined>() - base);
  Serial.print(F("rsqrt_libm ")); Serial.println(per_call<rs_libm>() - base);

  Serial.print(F("sin ")); Serial.println(per_call<f_sin>() - base);
  Serial.print(F("cos ")); Serial.println(per_call<f_cos>() - base);
  Serial.print(F("atan2 ")); Serial.println(per_call<f_atan2>() - base);
  Serial.print(F("asin ")); Serial.println(per_call<f_asin>() - base);
  Serial.print(F("fmul ")); Serial.println(per_call<f_mul>() - base);
  Serial.print(F("fdiv ")); Serial.println(per_call<f_div>() - base);

  uint32_t t0 = cycles();
  for (uint8_t i = 0; i < 32; i++) sink = analogRead(A0);
  Serial.print(F("analogRead ")); Serial.println((cycles() - t0) / 32);

  // Category = which UI work the iteration did: none, one chunk, a frame
  // render, each with or without a telemetry line.
  Stat st[6] = {};
  static const char *names[6] = {"plain", "plain+telem", "chunk", "chunk+telem", "frame", "frame+telem"};
  for (uint16_t i = 0; i < 3000; i++) {
    bench_bus = {};
    bench_serial.bytes = 0;
    uint32_t c0 = cycles();
    loop();
    uint32_t c = cycles() - c0;
    uint8_t k = bench_bus.frame_start ? 4 : bench_bus.chunk ? 2 : 0;
    st[k + (bench_serial.bytes > 0)].add(c);
  }
  Serial.println(F("category n mean min max"));
  for (uint8_t k = 0; k < 6; k++) st[k].print(names[k]);
  Serial.flush();

  cli();
  sleep_enable();
  sleep_cpu();
}
