#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Register map -- keep in sync with blueprint section 3.1.
#define REG_WHO_AM_I   0x00 // R,  1  -> 0x51
#define REG_GYRO       0x10 // R, 12  gyro x,y,z          float32, body frame, rad/s
#define REG_Q_TRUE     0x30 // R, 16  q_true w,x,y,z      GROUND TRUTH -- scoring only
#define REG_W_TRUE     0x40 // R, 12  w_true x,y,z        GROUND TRUTH -- scoring only
#define REG_TORQUE     0x50 // W, 12  torque x,y,z        float32, N.m
#define REG_IMU_DATA   0x70 // R, 24  accel x,y,z then gyro x,y,z -- one atomic read

#define WHO_AM_I_VALUE 0x51

// Everything the master can read, frozen at the instant a read transaction opens.
// See on_i2c_connect for why this exists.
typedef struct {
  float q[4];
  float w[3];
  float accel[3];
  float gyro[3];
} sensor_snapshot_t;

typedef struct {
  float q[4];
  float w[3];
  float tau[3];
  float J[3];

  // Simulated IMU Sensors
  float accel_out[3];
  float gyro_out[3];

  sensor_snapshot_t snap;

  uint8_t current_reg;
  uint8_t wire_buffer[64];
  uint8_t buffer_index;
  uint32_t i2c_dev;
} chip_state_t;

// Random noise generator (-1.0 to 1.0)
static float rand_noise() {
  return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

static void get_w_dot(const float w[3], const float J[3], const float tau[3], float w_dot[3]) {
  w_dot[0] = (tau[0] - (J[2] - J[1]) * w[1] * w[2]) / J[0];
  w_dot[1] = (tau[1] - (J[0] - J[2]) * w[2] * w[0]) / J[1];
  w_dot[2] = (tau[2] - (J[1] - J[0]) * w[0] * w[1]) / J[2];
}

static void on_timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  const float dt = 0.001f; 

  // RK4 Integration (Physics Engine)
  float k1[3], k2[3], k3[3], k4[3], w_temp[3];
  get_w_dot(chip->w, chip->J, chip->tau, k1);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + 0.5f * dt * k1[i];
  get_w_dot(w_temp, chip->J, chip->tau, k2);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + 0.5f * dt * k2[i];
  get_w_dot(w_temp, chip->J, chip->tau, k3);
  for(int i=0; i<3; i++) w_temp[i] = chip->w[i] + dt * k3[i];
  get_w_dot(w_temp, chip->J, chip->tau, k4);
  
  for(int i=0; i<3; i++) chip->w[i] += (dt / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);

  float w_norm = sqrt(chip->w[0]*chip->w[0] + chip->w[1]*chip->w[1] + chip->w[2]*chip->w[2]);
  float theta = w_norm * dt / 2.0f;
  float dq[4];
  
  if (theta < 1e-8f) { 
    dq[0] = 1.0f; dq[1] = chip->w[0] * dt / 2.0f; dq[2] = chip->w[1] * dt / 2.0f; dq[3] = chip->w[2] * dt / 2.0f;
  } else {
    float sin_theta = sin(theta);
    dq[0] = cos(theta); dq[1] = (chip->w[0] / w_norm) * sin_theta; dq[2] = (chip->w[1] / w_norm) * sin_theta; dq[3] = (chip->w[2] / w_norm) * sin_theta;
  }

  float qw = chip->q[0], qx = chip->q[1], qy = chip->q[2], qz = chip->q[3];
  float dw = dq[0], dx = dq[1], dy = dq[2], dz = dq[3];

  chip->q[0] = qw*dw - qx*dx - qy*dy - qz*dz;
  chip->q[1] = qw*dx + qx*dw + qy*dz - qz*dy;
  chip->q[2] = qw*dy - qx*dz + qy*dw + qz*dx;
  chip->q[3] = qw*dz + qx*dy - qy*dx + qz*dw;

  static int step = 0;
  if (++step >= 100) {
    step = 0;
    float norm = sqrt(chip->q[0]*chip->q[0] + chip->q[1]*chip->q[1] + chip->q[2]*chip->q[2] + chip->q[3]*chip->q[3]);
    for(int i=0; i<4; i++) chip->q[i] /= norm;
  }

  // --- M7 IMU SIMULATION ---
  // 1. Calculate gravity vector mapped to the body frame
  float gx = 2.0f * (chip->q[1]*chip->q[3] - chip->q[0]*chip->q[2]);
  float gy = 2.0f * (chip->q[0]*chip->q[1] + chip->q[2]*chip->q[3]);
  float gz = chip->q[0]*chip->q[0] - chip->q[1]*chip->q[1] - chip->q[2]*chip->q[2] + chip->q[3]*chip->q[3];

  // 2. Add high-frequency vibration noise to the accelerometer
  float accel_noise = 0.05f; 
  chip->accel_out[0] = gx + (rand_noise() * accel_noise);
  chip->accel_out[1] = gy + (rand_noise() * accel_noise);
  chip->accel_out[2] = gz + (rand_noise() * accel_noise);

  // 3. Add electrical white noise to the gyroscope
  float gyro_noise = 0.02f;
  chip->gyro_out[0] = chip->w[0] + (rand_noise() * gyro_noise);
  chip->gyro_out[1] = chip->w[1] + (rand_noise() * gyro_noise);
  chip->gyro_out[2] = chip->w[2] + (rand_noise() * gyro_noise);
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool is_write) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->buffer_index = 0;

  // The plant advances every 1 ms. A 24-byte read at 400 kHz occupies the bus for
  // roughly 600 us, so on_timer_tick WILL fire partway through one and the master
  // ends up with the first few bytes from step N and the rest from step N+1.
  // For a quaternion that means a non-unit result assembled from two instants
  // (blueprint trap 6.3); for accel-vs-gyro it means a fusion filter comparing
  // measurements that never coexisted.
  //
  // Latching the whole readable state here makes every transaction atomic with
  // respect to the physics tick. A read transaction opens with a repeated START
  // after the register-pointer write, so this fires immediately before the data.
  if (!is_write) {
    memcpy(chip->snap.q,     chip->q,         sizeof(chip->snap.q));
    memcpy(chip->snap.w,     chip->w,         sizeof(chip->snap.w));
    memcpy(chip->snap.accel, chip->accel_out, sizeof(chip->snap.accel));
    memcpy(chip->snap.gyro,  chip->gyro_out,  sizeof(chip->snap.gyro));
  }
  return true;
}

// Little-endian float32 on the wire; AVR and WASM agree, so a raw byte view is
// the whole serialisation. Blueprint section 1.
static uint8_t byte_of(const float *src, uint8_t offset) {
  return ((const uint8_t *)src)[offset];
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint8_t data = 0;
  uint8_t reg = chip->current_reg;

  if (reg == REG_WHO_AM_I) {
    data = WHO_AM_I_VALUE;
  }
  else if (reg >= REG_GYRO && reg < REG_GYRO + 12) {
    data = byte_of(chip->snap.gyro, reg - REG_GYRO);
  }
  // Ground truth. The controller must never read these -- see blueprint M7 and
  // trap 6.6. They exist so the host can score the estimator against the plant.
  else if (reg >= REG_Q_TRUE && reg < REG_Q_TRUE + 16) {
    data = byte_of(chip->snap.q, reg - REG_Q_TRUE);
  }
  else if (reg >= REG_W_TRUE && reg < REG_W_TRUE + 12) {
    data = byte_of(chip->snap.w, reg - REG_W_TRUE);
  }
  // Combined IMU block: accel (12 bytes) then gyro (12 bytes) = 24 bytes total.
  else if (reg >= REG_IMU_DATA && reg < REG_IMU_DATA + 12) {
    data = byte_of(chip->snap.accel, reg - REG_IMU_DATA);
  }
  else if (reg >= REG_IMU_DATA + 12 && reg < REG_IMU_DATA + 24) {
    data = byte_of(chip->snap.gyro, reg - (REG_IMU_DATA + 12));
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
  if (chip->current_reg == REG_TORQUE) {
    if (chip->buffer_index - 1 < 12) {
      chip->wire_buffer[chip->buffer_index - 1] = data;
      chip->buffer_index++;
    }
  }
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->current_reg == REG_TORQUE && chip->buffer_index == 13) {
    memcpy(chip->tau, chip->wire_buffer, 12);
  }
  chip->buffer_index = 0;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->q[0] = 1.0f; 
  chip->J[0] = 1.0f; chip->J[1] = 2.0f; chip->J[2] = 3.0f; 
  chip->w[0] = 0.0f; chip->w[1] = 0.0f; chip->w[2] = 0.0f;

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

  const timer_config_t timer_config = {
    .callback = on_timer_tick,
    .user_data = chip,
  };
  timer_start(timer_init(&timer_config), 1000, true);
}