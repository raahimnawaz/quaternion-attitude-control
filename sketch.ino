#include <Wire.h>
#include <math.h>
#include "quat.h"

struct Vec3 { float x, y, z; };

// Controller Tuning (J = [1, 2, 3])
// Tuned for wn = 3.0 rad/s, zeta = 0.7
const float Kp[3] = {18.0f, 36.0f, 54.0f}; 
const float Kd[3] = {4.2f,  8.4f,  12.6f}; 

Quat q_cmd;

void setup() {
  Serial.begin(115200);

  // M2 gate. Runs before the bus is touched so a broken quaternion library
  // cannot be mistaken for a broken plant or a broken controller.
  if (quat_selftest()) {
    Serial.println(F("ALL TESTS PASS"));
  } else {
    Serial.println(F("SELFTEST FAILED"));
    while (1) { }  // refuse to close the loop on top of bad math
  }

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C

  delay(100); // Let custom chip boot

  // Command a 90-degree step rotation about the X-axis
  q_cmd = quat_from_axis_angle(1.0f, 0.0f, 0.0f, PI / 2.0f);
  
  Serial.println("t_ms,qw,qx,qy,qz,err_deg,wx,wy,wz,tx,ty,tz,loop_us");
}

void loop() {
  uint32_t t_start = micros();
  
  Quat q_est;
  Vec3 w;

  // 1. Read Truth/Measured States (16 bytes Q + 12 bytes W = 28 bytes)
  // For M3 (Perfect Sensing), we read REG_Q_TRUE (0x30) as our measurement.
  Wire.beginTransmission(0x42);
  Wire.write(0x30); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 28);
  Wire.readBytes((uint8_t*)&q_est, 16);
  Wire.readBytes((uint8_t*)&w, 12);

  // 2. Control Law: q_err = q_est^-1 * q_cmd
  Quat q_est_inv = quat_conj(q_est);
  Quat q_err = quat_mul(q_est_inv, q_cmd);

  // Calculate physical error angle for the plotter
  float err_deg = 2.0f * acos(max(-1.0f, min(1.0f, q_err.w))) * (180.0f / PI);

  // 3. PD Torque Calculation
  // tau = Kp * sign(q_err.w) * q_err.vec - Kd * w
  float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f;
  Vec3 tau;
  tau.x = Kp[0] * sign_w * q_err.x - Kd[0] * w.x;
  tau.y = Kp[1] * sign_w * q_err.y - Kd[1] * w.y;
  tau.z = Kp[2] * sign_w * q_err.z - Kd[2] * w.z;

  // 4. Send Torque Command to Plant (0x50)
  Wire.beginTransmission(0x42);
  Wire.write(0x50);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  uint32_t loop_us = micros() - t_start;

  // 5. Log CSV 
  Serial.print(millis()); Serial.print(",");
  Serial.print(q_est.w, 3); Serial.print(",");
  Serial.print(q_est.x, 3); Serial.print(",");
  Serial.print(q_est.y, 3); Serial.print(",");
  Serial.print(q_est.z, 3); Serial.print(",");
  Serial.print(err_deg, 2); Serial.print(",");
  Serial.print(w.x, 2); Serial.print(",");
  Serial.print(w.y, 2); Serial.print(",");
  Serial.print(w.z, 2); Serial.print(",");
  Serial.print(tau.x, 2); Serial.print(",");
  Serial.print(tau.y, 2); Serial.print(",");
  Serial.print(tau.z, 2); Serial.print(",");
  Serial.println(loop_us);

  // Run loop at approx 100 Hz (10ms period)
  int delay_time = 10 - (loop_us / 1000);
  if (delay_time > 0) delay(delay_time);
}