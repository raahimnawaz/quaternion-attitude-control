// Zero-time stand-in for the AVR Wire library, so simavr times the firmware's
// CPU work with the I2C bus taken out. Shadows the core's Wire.h via -I.
#pragma once
#include <Arduino.h>
#include <Stream.h>

#define BUFFER_LENGTH 32

struct BenchBus {
  bool frame_start;  // SSD1306 command stream opening with PAGEADDR
  bool chunk;        // SSD1306 data stream
};
extern BenchBus bench_bus;

class TwoWire : public Stream {
  uint8_t addr_, tx_[BUFFER_LENGTH], txn_, rx_[BUFFER_LENGTH], rxn_, rxi_;

 public:
  void begin() {}
  void setClock(uint32_t) {}
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
  uint8_t endTransmission(uint8_t = true) {
    if (addr_ == 0x3C && txn_ > 1) {
      if (tx_[0] == 0x00 && tx_[1] == 0x22) bench_bus.frame_start = true;
      if (tx_[0] == 0x40) bench_bus.chunk = true;
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    static const float imu[6] = {0.01f, -0.02f, 0.99f, 0.001f, 0.002f, -0.001f};
    static const float q_true[4] = {0.9998f, 0.01f, -0.005f, 0.0f};
    rxi_ = 0;
    rxn_ = 0;
    if (a == 0x42 && n == 24) memcpy(rx_, imu, rxn_ = 24);
    if (a == 0x42 && n == 16) memcpy(rx_, q_true, rxn_ = 16);
    return rxn_;
  }
  uint8_t requestFrom(int a, int n) { return requestFrom((uint8_t)a, (uint8_t)n); }
  int available() override { return rxn_ - rxi_; }
  int read() override { return rxi_ < rxn_ ? rx_[rxi_++] : -1; }
  int peek() override { return rxi_ < rxn_ ? rx_[rxi_] : -1; }
  void flush() override {}
};

extern TwoWire Wire;
