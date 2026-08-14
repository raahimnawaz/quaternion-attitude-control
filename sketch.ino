#include "Wire.h"
#include "math.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

const float Kp[3] = {18.0f, 36.0f, 54.0f}; 
const float Kd[3] = {4.2f,  8.4f,  12.6f}; 

Quat q_cmd;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  // Initialize the OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.display();
  
  delay(100); 
  
  // Command Level Flight
  q_cmd = {1.0f, 0.0f, 0.0f, 0.0f};
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

  float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f; 

  Vec3 tau;
  tau.x = Kp[0] * sign_w * q_err.x - Kd[0] * w.x;
  tau.y = Kp[1] * sign_w * q_err.y - Kd[1] * w.y;
  tau.z = Kp[2] * sign_w * q_err.z - Kd[2] * w.z;

  Wire.beginTransmission(0x42);
  Wire.write(0x50);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  // --- M5: OLED ARTIFICIAL HORIZON ---
  
  // 1. Convert Quaternion to Euler (Roll and Pitch)
  float roll  = atan2(2.0f * (q_est.w * q_est.x + q_est.y * q_est.z), 1.0f - 2.0f * (q_est.x * q_est.x + q_est.y * q_est.y));
  float pitch = asin(2.0f * (q_est.w * q_est.y - q_est.z * q_est.x));

  display.clearDisplay();

  // 2. Draw the static aircraft reticle in the center
  display.drawFastHLine(64 - 15, 32, 10, SSD1306_WHITE); // Left wing
  display.drawFastHLine(64 + 5,  32, 10, SSD1306_WHITE); // Right wing
  display.drawPixel(64, 32, SSD1306_WHITE);              // Nose

  // 3. Calculate the moving horizon line
  float pixels_per_rad = 40.0f; 
  float pitch_offset = pitch * pixels_per_rad;
  
  int r = 100; // Length of the horizon line
  int x0 = 64 - r * cos(roll);
  int y0 = 32 + pitch_offset + r * sin(roll);
  int x1 = 64 + r * cos(roll);
  int y1 = 32 + pitch_offset - r * sin(roll);

  display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  display.display();

  // -----------------------------------

  uint32_t loop_us = micros() - t_start;
  int delay_time = 10 - (loop_us / 1000);
  if (delay_time > 0) delay(delay_time);
}