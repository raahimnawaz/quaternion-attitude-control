#pragma once
#include "Print.h"

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  size_t readBytes(char *buf, size_t n) {
    size_t k = 0;
    for (int c; k < n && (c = read()) >= 0;) buf[k++] = (char)c;
    return k;
  }
  size_t readBytes(uint8_t *buf, size_t n) { return readBytes((char *)buf, n); }
};
