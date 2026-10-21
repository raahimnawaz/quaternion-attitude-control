// The subset of Wokwi's custom-chip API that chip/chip.chip.c uses, backed by
// the host simulator's clock and bus.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t pin_t;
#define INPUT 0

typedef struct {
  void *user_data;
  uint32_t address;
  pin_t scl;
  pin_t sda;
  bool (*connect)(void *user_data, uint32_t address, bool connect);
  uint8_t (*read)(void *user_data);
  bool (*write)(void *user_data, uint8_t data);
  void (*disconnect)(void *user_data);
  uint32_t reserved[8];
} i2c_config_t;

typedef struct {
  void *user_data;
  void (*callback)(void *user_data);
  uint32_t reserved[8];
} timer_config_t;

pin_t pin_init(const char *name, uint32_t mode);
uint32_t i2c_init(const i2c_config_t *config);
uint32_t timer_init(const timer_config_t *config);
void timer_start(uint32_t timer_id, uint32_t micros, bool repeat);

#ifdef __cplusplus
}
#endif
