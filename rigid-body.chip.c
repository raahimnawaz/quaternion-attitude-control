#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define REG_WHO_AM_I   0x00
#define REG_GYRO_BASE  0x10
#define REG_Q_MEAS     0x20
#define REG_Q_TRUE     0x30
#define REG_W_TRUE     0x40
#define REG_TORQUE     0x50
#define REG_CONTROL    0x60
#define REG_GYRO_SIGMA 0x61

typedef struct {
  float q[4];
  float w[3];
  float tau[3];
  float J[3];
  
  uint8_t current_reg;
  uint8_t wire_buffer[64];
  uint8_t buffer_index;
  uint32_t i2c_dev;
} chip_state_t;

// Euler's Equations
static void get_w_dot(const float w[3], const float J[3], const float tau[3], float w_dot[3]) {
  w_dot[0] = (tau[0] - (J[2] - J[1]) * w[1] * w[2]) / J[0];
  w_dot[1] = (tau[1] - (J[0] - J[2]) * w[2] * w[0]) / J[1];
  w_dot[2] = (tau[2] - (J[1] - J[0]) * w[0] * w[1]) / J[2];
}

static void on_timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  const float dt = 0.001f; 

  // RK4 Integration
  float k1[3], k2[3], k3[3], k4[3], w_temp[3];
  
  get_w_dot(chip->w, chip->J, chip->tau, k1);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + 0.5f * dt * k1[i];
  
  get_w_dot(w_temp, chip->J, chip->tau, k2);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + 0.5f * dt * k2[i];
  
  get_w_dot(w_temp, chip->J, chip->tau, k3);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + dt * k3[i];
  
  get_w_dot(w_temp, chip->J, chip->tau, k4);
  
  for(int i=0; i<3; i++) {
    chip->w[i] += (dt / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);
  }

  // Exponential Map
  float w_norm = sqrt(chip->w[0]*chip->w[0] + chip->w[1]*chip->w[1] + chip->w[2]*chip->w[2]);
  float theta = w_norm * dt / 2.0f;
  float dq[4];
  
  if (theta < 1e-8f) { 
    dq[0] = 1.0f;
    dq[1] = chip->w[0] * dt / 2.0f;
    dq[2] = chip->w[1] * dt / 2.0f;
    dq[3] = chip->w[2] * dt / 2.0f;
  } else {
    float sin_theta = sin(theta);
    dq[0] = cos(theta);
    dq[1] = (chip->w[0] / w_norm) * sin_theta;
    dq[2] = (chip->w[1] / w_norm) * sin_theta;
    dq[3] = (chip->w[2] / w_norm) * sin_theta;
  }

  float qw = chip->q[0], qx = chip->q[1], qy = chip->q[2], qz = chip->q[3];
  float dw = dq[0], dx = dq[1], dy = dq[2], dz = dq[3];

  chip->q[0] = qw*dw - qx*dx - qy*dy - qz*dz;
  chip->q[1] = qw*dx + qx*dw + qy*dz - qz*dy;
  chip->q[2] = qw*dy - qx*dz + qy*dw + qz*dx;
  chip->q[3] = qw*dz + qx*dy - qy*dx + qz*dw;

  // Occasional re-normalization
  static int step = 0;
  if (++step >= 100) {
    step = 0;
    float norm = sqrt(chip->q[0]*chip->q[0] + chip->q[1]*chip->q[1] + chip->q[2]*chip->q[2] + chip->q[3]*chip->q[3]);
    for(int i=0; i<4; i++) chip->q[i] /= norm;
  }
}

// NOTE: uint32_t address to match Wokwi API update
static bool on_i2c_connect(void *user_data, uint32_t address, bool is_write) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->buffer_index = 0;
  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint8_t data = 0;
  uint8_t reg = chip->current_reg;

  if (reg == REG_WHO_AM_I) {
    data = 0x51;
  } 
  else if (reg >= REG_Q_TRUE && reg < REG_Q_TRUE + 16) {
    data = ((uint8_t*)chip->q)[reg - REG_Q_TRUE];
  } 
  else if (reg >= REG_W_TRUE && reg < REG_W_TRUE + 12) {
    data = ((uint8_t*)chip->w)[reg - REG_W_TRUE];
  }

  chip->current_reg++;
  return data;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->buffer_index == 0) {
    chip->current_reg = data;
    chip->buffer_index++;
    return true;
  }
  if (chip->current_reg >= REG_TORQUE && chip->current_reg <= REG_GYRO_SIGMA) {
    if (chip->buffer_index - 1 < sizeof(chip->wire_buffer)) {
      chip->wire_buffer[chip->buffer_index - 1] = data;
      chip->buffer_index++;
    }
  }
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->current_reg >= REG_TORQUE && chip->buffer_index > 1) {
    int payload_len = chip->buffer_index - 1;
    if (chip->current_reg == REG_TORQUE && payload_len == 12) {
      memcpy(chip->tau, chip->wire_buffer, 12);
    }
  }
  chip->buffer_index = 0;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->q[0] = 1.0f; 

  chip->J[0] = 1.0f; 
  chip->J[1] = 2.0f; 
  chip->J[2] = 3.0f; 
  
  // Starting the plant at a slight spin so the controller has to fight it
  chip->w[0] = 0.01f; 
  chip->w[1] = 5.0f;  
  chip->w[2] = 0.0f;

  const i2c_config_t i2c_config = {
    .address = 0x42,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = chip,
  };
  chip->i2c_dev = i2c_init(&i2c_config);

  // Modern Wokwi Timer API
  const timer_config_t timer_config = {
    .callback = on_timer_tick,
    .user_data = chip,
  };
  
  timer_t physics_timer = timer_init(&timer_config);
  timer_start(physics_timer, 1000, true);
}