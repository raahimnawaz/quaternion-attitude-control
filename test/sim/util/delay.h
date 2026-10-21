// avr-libc's busy-wait delays. Adafruit_SSD1306.cpp includes this on any
// non-ARM host (on arm64 it's skipped), and its I2C path never calls them.
#pragma once
static inline void _delay_ms(double) {}
static inline void _delay_us(double) {}
