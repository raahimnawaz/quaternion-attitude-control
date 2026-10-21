// Type-only stand-in: the display is on I2C, so no SPI path ever runs.
#pragma once
#include "Arduino.h"

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

class SPISettings {
 public:
  SPISettings() {}
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
 public:
  void begin() {}
  void end() {}
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t d) { return d; }
  void transfer(void *, size_t) {}
  uint16_t transfer16(uint16_t d) { return d; }
  void setBitOrder(uint8_t) {}
  void setDataMode(uint8_t) {}
  void setClockDivider(uint8_t) {}
};
extern SPIClass SPI;
