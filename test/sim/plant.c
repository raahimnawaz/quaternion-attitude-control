// The real plant, compiled as-is, plus ground-truth accessors for scoring.
// The firmware only ever sees the plant through the I2C registers.
#include "../../chip/chip.chip.c"
#include "plant.h"

void plant_init(unsigned seed) {
  srand(seed);
  chip_init();
}

void plant_truth(void *ud, float q[4], float w[3]) {
  chip_state_t *c = (chip_state_t *)ud;
  memcpy(q, c->q, sizeof c->q);
  memcpy(w, c->w, sizeof c->w);
}

void plant_set_state(void *ud, const float q[4], const float w[3]) {
  chip_state_t *c = (chip_state_t *)ud;
  memcpy(c->q, q, sizeof c->q);
  memcpy(c->w, w, sizeof c->w);
}

void plant_zero_torque(void *ud) {
  memset(((chip_state_t *)ud)->tau, 0, 12);
}

void plant_live_imu(void *ud, uint8_t out[24]) {
  chip_state_t *c = (chip_state_t *)ud;
  memcpy(out, c->accel_out, 12);
  memcpy(out + 12, c->gyro_out, 12);
}
