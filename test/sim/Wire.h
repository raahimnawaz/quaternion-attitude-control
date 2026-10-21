// AVR Wire API on top of the simulated bus in sim.h. Same 32-byte buffer,
// same 100 kHz reset in begin() that the real twi_init() does.
#pragma once
#include "Arduino.h"

#define BUFFER_LENGTH 32

class TwoWire : public Stream {
  uint8_t addr_ = 0, tx_[BUFFER_LENGTH], txn_ = 0, rx_[BUFFER_LENGTH], rxn_ = 0, rxi_ = 0;

 public:
  void begin();
  void setClock(uint32_t hz);
  void beginTransmission(uint8_t a) { addr_ = a; txn_ = 0; }
  void beginTransmission(int a) { beginTransmission((uint8_t)a); }
  size_t write(uint8_t b) override {
    if (txn_ >= BUFFER_LENGTH) return 0;
    tx_[txn_++] = b;
    return 1;
  }
  size_t write(int b) { return write((uint8_t)b); }
  size_t write(unsigned int b) { return write((uint8_t)b); }
  size_t write(long b) { return write((uint8_t)b); }
  size_t write(unsigned long b) { return write((uint8_t)b); }
  size_t write(const uint8_t *p, size_t n) override {
    for (size_t i = 0; i < n; i++)
      if (!write(p[i])) return i;
    return n;
  }
  using Print::write;
  uint8_t endTransmission(uint8_t stop = true);
  uint8_t requestFrom(uint8_t a, uint8_t n);
  uint8_t requestFrom(int a, int n) { return requestFrom((uint8_t)a, (uint8_t)n); }
  int available() override { return rxn_ - rxi_; }
  int read() override { return rxi_ < rxn_ ? rx_[rxi_++] : -1; }
  int peek() override { return rxi_ < rxn_ ? rx_[rxi_] : -1; }
  void flush() override {}
};

extern TwoWire Wire;
