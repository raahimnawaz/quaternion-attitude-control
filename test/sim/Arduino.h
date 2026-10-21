// Just enough of the Arduino AVR core for sketch.ino and the Adafruit
// libraries to build on the host. Time, ADC and serial go through sim.h.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ARDUINO
#define ARDUINO 10808
#endif

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;
typedef enum { LSBFIRST = 0, MSBFIRST = 1 } BitOrder;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)

#define A0 54
#define A1 55
#define A2 56

#define PROGMEM
#define PSTR(s) (s)
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))

class String {
  const char *s_;

 public:
  String(const char *s = "") : s_(s) {}
  const char *c_str() const { return s_; }
  unsigned length() const { return (unsigned)strlen(s_); }
};

template <class T, class U> static inline T min(T a, U b) { return a < (T)b ? a : (T)b; }
template <class T, class U> static inline T max(T a, U b) { return a > (T)b ? a : (T)b; }

uint32_t micros();
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(unsigned us);
int analogRead(uint8_t pin);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
static inline void yield() {}

#include "Print.h"
#include "Stream.h"

class HardwareSerial : public Stream {
 public:
  void begin(unsigned long) {}
  size_t write(uint8_t c) override;
  using Print::write;
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
};
extern HardwareSerial Serial;
