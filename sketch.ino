#include "Wire.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; };

// --- Reciprocal square root ---
// Called twice per control loop: once to normalise the accelerometer vector,
// once to normalise the estimated quaternion.
//
// Which implementation is faster depends entirely on whether the target has a
// hardware FPU, so this is decided at compile time rather than assumed:
//
//   Arduino Mega (ATmega2560) — no FPU. Every float operation is a libgcc
//     call and sqrtf() is a software routine, so trading a square root and a
//     divide for one shift, one subtract and three multiplies is a genuine
//     saving. This is the board the Wokwi prototype runs on.
//
//   ESP32 (Xtensa LX6) and STM32WBA55 (Cortex-M33F) — both have a
//     single-precision FPU. 1.0f/sqrtf(x) is a couple of instructions and
//     correctly rounded, while the bit trick additionally pays to move the
//     value between the integer and float register files. On these parts the
//     "optimisation" is slower AND less accurate.
//
// The custom board in docs/schematic.png is the STM32WBA55, so on the hardware
// this project is actually headed for, the fast path is the plain divide.
#if defined(__AVR__)
static inline float invSqrt(float x) {
  // Bit-level initial guess (Quake III) + one Newton-Raphson step.
  // Worst-case relative error after the single step is ~0.175%.
  //
  // memcpy rather than *(long*)&y: the pointer cast is a strict-aliasing
  // violation, i.e. undefined behaviour that -O2 is entitled to miscompile.
  // Every compiler in use turns this memcpy into the same register move.
  float halfx = 0.5f * x;
  uint32_t i;
  memcpy(&i, &x, sizeof i);
  i = 0x5f3759df - (i >> 1);
  float y;
  memcpy(&y, &i, sizeof y);
  return y * (1.5f - halfx * y * y);
}
#else
static inline float invSqrt(float x) {
  return 1.0f / sqrtf(x);
}
#endif

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

Quat quat_from_euler(float roll, float pitch, float yaw) {
  float cr = cos(roll * 0.5f);
  float sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f);
  float sp = sin(pitch * 0.5f);
  float cy = cos(yaw * 0.5f);
  float sy = sin(yaw * 0.5f);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy
  };
}

const float Kp_base[3] = {18.0f, 36.0f, 54.0f}; 
const float Kd_base[3] = {4.2f,  8.4f,  12.6f}; 

// State Estimation Globals
Quat q_est = {1.0f, 0.0f, 0.0f, 0.0f}; 
uint32_t last_micros = 0;
uint32_t last_oled_millis = 0;
float Kp_imu = 2.5f; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  pinMode(A0, INPUT); 
  pinMode(A1, INPUT); 
  pinMode(A2, INPUT); 
  
  display.clearDisplay();
  display.display();
  delay(100); 
  
  last_micros = micros();
  last_oled_millis = millis();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_micros) / 1000000.0f;
  last_micros = now;

  // --- 1. SENSOR READING (Raw IMU Data) ---
  Vec3 accel, gyro;
  Wire.beginTransmission(0x42);
  Wire.write(0x30); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 24);
  Wire.readBytes((uint8_t*)&accel, 12);
  Wire.readBytes((uint8_t*)&gyro, 12);

  // --- 2. MAHONY SENSOR FUSION FILTER (Optimized) ---
  float accel_sq = accel.x*accel.x + accel.y*accel.y + accel.z*accel.z;
  if (accel_sq > 0.0f) {
    // Fast normalization using multiplication
    float a_norm_inv = invSqrt(accel_sq);
    accel.x *= a_norm_inv; 
    accel.y *= a_norm_inv; 
    accel.z *= a_norm_inv;
    
    Vec3 g_est = {
      2.0f * (q_est.x * q_est.z - q_est.w * q_est.y),
      2.0f * (q_est.w * q_est.x + q_est.y * q_est.z),
      q_est.w * q_est.w - q_est.x * q_est.x - q_est.y * q_est.y + q_est.z * q_est.z
    };

    Vec3 error = {
      accel.y * g_est.z - accel.z * g_est.y,
      accel.z * g_est.x - accel.x * g_est.z,
      accel.x * g_est.y - accel.y * g_est.x
    };

    gyro.x += Kp_imu * error.x;
    gyro.y += Kp_imu * error.y;
    gyro.z += Kp_imu * error.z;
  }

  // Integrate
  Quat q_dot = {
    -0.5f * (q_est.x * gyro.x + q_est.y * gyro.y + q_est.z * gyro.z),
     0.5f * (q_est.w * gyro.x + q_est.y * gyro.z - q_est.z * gyro.y),
     0.5f * (q_est.w * gyro.y - q_est.x * gyro.z + q_est.z * gyro.x),
     0.5f * (q_est.w * gyro.z + q_est.x * gyro.y - q_est.y * gyro.x)
  };

  q_est.w += q_dot.w * dt;
  q_est.x += q_dot.x * dt;
  q_est.y += q_dot.y * dt;
  q_est.z += q_dot.z * dt;

  // Fast quaternion normalization
  float q_norm_inv = invSqrt(q_est.w*q_est.w + q_est.x*q_est.x + q_est.y*q_est.y + q_est.z*q_est.z);
  q_est.w *= q_norm_inv; 
  q_est.x *= q_norm_inv; 
  q_est.y *= q_norm_inv; 
  q_est.z *= q_norm_inv;

  // --- 3. PILOT INPUT ---
  float joy_roll =  ((analogRead(A0) - 512) / 512.0f) * (PI / 4.0f);
  float joy_pitch = ((analogRead(A1) - 512) / 512.0f) * (PI / 4.0f);
  Quat q_cmd = quat_from_euler(joy_roll, joy_pitch, 0.0f);

  float gain_multiplier = 0.1f + (analogRead(A2) / 1023.0f) * 2.9f;

  // --- 4. PD CONTROL LAW ---
  Quat q_est_inv = quat_conj(q_est);
  Quat q_err = quat_mul(q_est_inv, q_cmd);
  float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f; 

  Vec3 tau;
  tau.x = (Kp_base[0] * gain_multiplier) * sign_w * q_err.x - (Kd_base[0] * gain_multiplier) * gyro.x;
  tau.y = (Kp_base[1] * gain_multiplier) * sign_w * q_err.y - (Kd_base[1] * gain_multiplier) * gyro.y;
  tau.z = (Kp_base[2] * gain_multiplier) * sign_w * q_err.z - (Kd_base[2] * gain_multiplier) * gyro.z;

  Wire.beginTransmission(0x42);
  Wire.write(0x50);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  // --- 5. UI LOOP (Decoupled at ~30 FPS) ---
  // The UI no longer blocks the physics calculations
  if (millis() - last_oled_millis >= 33) {
    last_oled_millis = millis();
    
    float roll  = atan2(2.0f * (q_est.w * q_est.x + q_est.y * q_est.z), 1.0f - 2.0f * (q_est.x * q_est.x + q_est.y * q_est.y));
    float pitch = asin(2.0f * (q_est.w * q_est.y - q_est.z * q_est.x));

    display.clearDisplay();
    display.drawFastHLine(64 - 15, 32, 10, SSD1306_WHITE); 
    display.drawFastHLine(64 + 5,  32, 10, SSD1306_WHITE); 
    display.drawPixel(64, 32, SSD1306_WHITE);              

    float pixels_per_rad = 40.0f; 
    float pitch_offset = pitch * pixels_per_rad;
    
    int r = 100; 
    int x0 = 64 - r * cos(roll);
    int y0 = 32 + pitch_offset + r * sin(roll);
    int x1 = 64 + r * cos(roll);
    int y1 = 32 + pitch_offset - r * sin(roll);

    display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
    display.display();
    
    // Print telemetry out for the Python dashboard
    Serial.print(q_est.w, 4); Serial.print(",");
    Serial.print(q_est.x, 4); Serial.print(",");
    Serial.print(q_est.y, 4); Serial.print(",");
    Serial.println(q_est.z, 4);
  }
}