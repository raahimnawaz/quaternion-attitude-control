#include <Wire.h>
#include <math.h>

// Toggle this! 1 = Naive Controller (Bugged). 0 = Optimized Controller (Fixed).
#define UNWINDING_DEMO 0 

struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; };

Quat quat_mul(const Quat& q, const Quat& p) {
  return {
    q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
    q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
    q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
    q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
  };
}

Quat quat_conj(const Quat& q) {
  return { q.w, -q.x, -q.y, -q.z };
}

Quat quat_from_axis_angle(float ax, float ay, float az, float rad) {
  float half_angle = rad / 2.0f;
  float s = sin(half_angle);
  return { cos(half_angle), ax * s, ay * s, az * s };
}

const float Kp[3] = {18.0f, 36.0f, 54.0f}; 
const float Kd[3] = {4.2f,  8.4f,  12.6f}; 

Quat q_cmd;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  delay(100); 
  
  // Command exactly 181 degrees (Just past the halfway point of the sphere)
  q_cmd = quat_from_axis_angle(1.0f, 0.0f, 0.0f, 181.0f * PI / 180.0f);
  
  Serial.println("t_ms,qw,qx,qy,qz,err_deg,wx,wy,wz,tx,ty,tz,loop_us");
}

void loop() {
  uint32_t t_start = micros();
  Quat q_est;
  Vec3 w;

  Wire.beginTransmission(0x42);
  Wire.write(0x30); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 28);
  Wire.readBytes((uint8_t*)&q_est, 16);
  Wire.readBytes((uint8_t*)&w, 12);

  Quat q_est_inv = quat_conj(q_est);
  Quat q_err = quat_mul(q_est_inv, q_cmd);

  // --- THE UNWINDING LOGIC ---
  #if UNWINDING_DEMO
    // The Bug: The controller is blind to the shortest path.
    float sign_w = 1.0f; 
  #else
    // The Fix: Check if we are on the wrong hemisphere.
    float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f; 
  #endif

  // Calculate the physical error angle strictly for the plotter display
  float err_deg;
  if (sign_w < 0.0f) {
    err_deg = 2.0f * acos(max(-1.0f, min(1.0f, -q_err.w))) * (180.0f / PI);
  } else {
    err_deg = 2.0f * acos(max(-1.0f, min(1.0f, q_err.w))) * (180.0f / PI);
  }

  Vec3 tau;
  tau.x = Kp[0] * sign_w * q_err.x - Kd[0] * w.x;
  tau.y = Kp[1] * sign_w * q_err.y - Kd[1] * w.y;
  tau.z = Kp[2] * sign_w * q_err.z - Kd[2] * w.z;

  Wire.beginTransmission(0x42);
  Wire.write(0x50);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  uint32_t loop_us = micros() - t_start;

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

  int delay_time = 10 - (loop_us / 1000);
  if (delay_time > 0) delay(delay_time);
}