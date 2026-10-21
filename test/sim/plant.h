#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void plant_init(unsigned seed);
void plant_truth(void *ud, float q[4], float w[3]);
void plant_set_state(void *ud, const float q[4], const float w[3]);
void plant_zero_torque(void *ud);
void plant_set_gyro_bias(void *ud, const float b[3]);
void plant_live_imu(void *ud, uint8_t out[24]);

#ifdef __cplusplus
}
#endif
