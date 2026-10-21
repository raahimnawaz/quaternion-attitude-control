#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

class __FlashStringHelper;

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *b, size_t n) {
    size_t k = 0;
    while (n--) k += write(*b++);
    return k;
  }
  size_t write(const char *s) { return s ? write((const uint8_t *)s, strlen(s)) : 0; }
  size_t write(const char *b, size_t n) { return write((const uint8_t *)b, n); }
  virtual int availableForWrite() { return 0; }
  virtual void flush() {}

  size_t print(const __FlashStringHelper *s) { return write(reinterpret_cast<const char *>(s)); }
  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v, int base = 10) { return print((unsigned long)v, base); }
  size_t print(int v, int base = 10) { return print((long)v, base); }
  size_t print(unsigned int v, int base = 10) { return print((unsigned long)v, base); }
  size_t print(long v, int base = 10) { return fmt(base == 16 ? "%lx" : "%ld", v); }
  size_t print(unsigned long v, int base = 10) { return fmt(base == 16 ? "%lx" : "%lu", v); }
  size_t print(double v, int digits = 2) {
    char b[48];
    snprintf(b, sizeof b, "%.*f", digits, v);
    return write(b);
  }

  size_t println() { return write("\r\n"); }
  template <class T> size_t println(T v) { size_t n = print(v); return n + println(); }
  template <class T> size_t println(T v, int f) { size_t n = print(v, f); return n + println(); }

 private:
  template <class T> size_t fmt(const char *f, T v) {
    char b[32];
    snprintf(b, sizeof b, f, v);
    return write(b);
  }
};
