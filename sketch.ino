#include "Wire.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// The last two arguments are the bus clock during and *after* each display
// transfer. The library's default "after" is 100 kHz, which silently undid
// Wire.setClock(400000) for every plant transaction.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000UL, 400000UL);

// Plant register map -- keep in sync with chip/chip.chip.c.
// These were bare hex literals at three call sites, which is how 0x30 came to mean
// two different things in two different files.
#define PLANT_ADDR   0x42
#define REG_Q_TRUE   0x30 // R, 16  ground truth -- SCORING ONLY, never in the control path
#define REG_TORQUE   0x50 // W, 12
#define REG_IMU_DATA 0x70 // R, 24  accel x,y,z then gyro x,y,z

struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; };

// --- Reciprocal square root ---
// Called once per control loop, to normalise the accelerometer vector -- only
// its direction matters, so one Newton step is accurate enough.
//
// Which implementation is faster depends entirely on whether the target has a
// hardware FPU, so this is decided at compile time rather than assumed:
//
//   Arduino Mega (ATmega2560) -- no FPU. Every float operation is a libgcc
//     call and sqrtf() is a software routine, so trading a square root and a
//     divide for an integer shift and subtract, four multiplies and a float
//     subtract is a saving: 730 cycles against 979 (simavr). This is the
//     board the Wokwi prototype runs on.
//
//   ESP32 (Xtensa LX6) and STM32WBA55 (Cortex-M33F) -- both have a
//     single-precision FPU. 1.0f/sqrtf(x) is a couple of instructions and
//     correctly rounded, while the bit trick additionally pays to move the
//     value between the integer and float register files. On these parts the
//     "optimisation" is slower AND less accurate.
//
// So on any FPU target this firmware moves to, the fast path is the plain divide.
#if defined(__AVR__)
static inline float invSqrt(float x) {
  // Bit-level initial guess (Quake III) + one Newton-Raphson step. The magic
  // constant lands within ~3.4%; one step takes that to a worst-case relative
  // error of ~1.75e-3. Fine for a direction vector, NOT fine for q_est.
  //
  // memcpy rather than *(long*)&y: type-punning through a pointer cast is a
  // strict-aliasing violation -- undefined behaviour that -O2 is entitled to
  // miscompile -- and `long` is not 32 bits everywhere. Every compiler in use
  // turns this memcpy into the same register move.
  float halfx = 0.5f * x;
  uint32_t i;
  memcpy(&i, &x, sizeof i);
  i = 0x5f3759dfUL - (i >> 1);
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

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
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
uint32_t last_frame_millis = 0;
uint32_t last_telemetry_millis = 0;
uint32_t max_dt_us = 0;
float Kp_imu = 2.5f; 

// Mahony integral term: the estimate of the gyro's bias, rad/s, same sign as the
// bias. Linearised, the tilt error obeys s^2 + Kp_imu s + Ki_imu = 0, and
// Ki_imu = Kp_imu^2 / 4 makes that critically damped: a double pole at 1.25 rad/s.
// Without a magnetometer only the bias component perpendicular to gravity is
// observable: level, that is roll and pitch, and yaw is never estimated.
Vec3 gyro_bias = {0.0f, 0.0f, 0.0f};
float Ki_imu = 1.5625f;

// --- OLED streaming ---
// One display() is 1024 bytes plus headers: ~25 ms of bus at 400 kHz, during
// which the loop is stalled. Instead the framebuffer goes out two Wire-buffer
// chunks per control iteration: ~1.5 ms of bus per loop, ~10 FPS.
const uint16_t OLED_BYTES = SCREEN_WIDTH * SCREEN_HEIGHT / 8;
const uint8_t OLED_CHUNK = 31;    // 32-byte AVR Wire buffer, less the 0x40 prefix
const uint8_t OLED_CHUNKS_PER_LOOP = 2;
uint16_t oled_sent = OLED_BYTES;  // == OLED_BYTES: no frame in flight

void oled_start_frame() {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);  // command stream
  Wire.write(SSD1306_PAGEADDR);
  Wire.write(0);
  Wire.write(SCREEN_HEIGHT / 8 - 1);
  Wire.write(SSD1306_COLUMNADDR);
  Wire.write(0);
  Wire.write(SCREEN_WIDTH - 1);
  Wire.endTransmission();
  oled_sent = 0;
}

void oled_send_chunk() {
  uint16_t n = OLED_BYTES - oled_sent;
  if (n > OLED_CHUNK) n = OLED_CHUNK;
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x40);  // data stream
  Wire.write(display.getBuffer() + oled_sent, n);
  Wire.endTransmission();
  oled_sent += n;
}

void render_horizon() {
  float roll  = atan2(2.0f * (q_est.w * q_est.x + q_est.y * q_est.z), 1.0f - 2.0f * (q_est.x * q_est.x + q_est.y * q_est.y));
  // The clamp is load-bearing. Float error in the norm pushes
  // this argument past +/-1 and asin returns NaN, which reaches the display and
  // reads as a controller failure rather than a numerics failure.
  float pitch = asin(clampf(2.0f * (q_est.w * q_est.y - q_est.z * q_est.x), -1.0f, 1.0f));

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
}

// Serial.print(float, 4) costs ~8,800 cycles a value in soft float. Quaternion
// components are in [-1, 1], so format them as fixed point: same text, a
// fraction of the cost.
void print_fixed4(float v) {
  static const int16_t place[4] = {1000, 100, 10, 1};
  int16_t n = lround(v * 10000.0f);
  char s[8], *p = s;
  if (n < 0) { *p++ = '-'; n = -n; }
  *p++ = n >= 10000 ? '1' : '0';
  if (n >= 10000) n -= 10000;
  *p++ = '.';
  for (uint8_t k = 0; k < 4; k++) {  // digits by subtraction: no 16-bit divides
    char c = '0';
    while (n >= place[k]) { n -= place[k]; c++; }
    *p++ = c;
  }
  *p = 0;
  Serial.print(s);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
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
  last_frame_millis = millis();
  last_telemetry_millis = millis();
}

void loop() {
  uint32_t now = micros();

  // --- 1. SENSOR READING (Raw IMU Data) ---
  // One transaction for both sensors. Two transactions would sample two different
  // instants of a 1 kHz plant; the chip additionally latches
  // its state on connect so this read is atomic with respect to the physics tick.
  Vec3 accel, gyro;
  Wire.beginTransmission(PLANT_ADDR);
  Wire.write(REG_IMU_DATA);
  Wire.endTransmission(false);
  if (Wire.requestFrom(PLANT_ADDR, 24) == 24) {
    Wire.readBytes((uint8_t*)&accel, 12);
    Wire.readBytes((uint8_t*)&gyro, 12);
  } else {
    // Short read: readBytes would time out and leave accel/gyro partly filled with
    // stack garbage, which then gets integrated. Skip this cycle instead -- and
    // leave last_micros alone, so the next good sample integrates across the
    // whole gap rather than the estimator silently losing its rotation.
    return;
  }

  uint32_t dt_us = now - last_micros;
  last_micros = now;
  if (dt_us > max_dt_us) max_dt_us = dt_us;
  float dt = dt_us / 1000000.0f;

  // --- 2. MAHONY SENSOR FUSION FILTER (Optimized) ---
  // The filter mutates `gyro` in place into a corrected rate for the integrator:
  // less the bias estimate, plus Kp_imu * (accelerometer residual).
  // The PD law's derivative term needs the raw measurement, not that corrected
  // value -- see the note at the control law. Keep a copy before it is touched.
  //
  // Quantified: the correction adds Kp_imu * (a x v_est) into `gyro`, which is
  // estimator feedback, not angular rate. Routing it into the controller feeds
  // estimator error straight to commanded torque (Kd*Kp_imu = 10.5 against a Kp
  // of 18) and amplifies accelerometer noise into the actuator by 2.5x.
  const Vec3 gyro_raw = gyro;

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

    float ki_dt = Ki_imu * dt;
    gyro_bias.x -= ki_dt * error.x;
    gyro_bias.y -= ki_dt * error.y;
    gyro_bias.z -= ki_dt * error.z;

    gyro.x += Kp_imu * error.x;
    gyro.y += Kp_imu * error.y;
    gyro.z += Kp_imu * error.z;
  }
  gyro.x -= gyro_bias.x;
  gyro.y -= gyro_bias.y;
  gyro.z -= gyro_bias.z;

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

  // Quaternion normalization -- the plain library call, on every target. One
  // Newton step leaves |q| biased by ~1.7e-3, the error that compounds and
  // reaches asin(); a second step fixes that but measures 1,250-1,300 cycles
  // against 979 for the correctly rounded 1/sqrtf (simavr, ATmega2560).
  float q_norm_inv = 1.0f / sqrtf(q_est.w*q_est.w + q_est.x*q_est.x + q_est.y*q_est.y + q_est.z*q_est.z);
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
  // q and -q are the same attitude: take the short way round.
  float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f;

  // Derivative term uses gyro_raw, not the Mahony-corrected gyro. The corrected
  // value carries Kp_imu * (accelerometer residual), so feeding it here would
  // multiply accelerometer noise by Kp_imu * Kd straight into the torque command.
  // It does subtract the bias estimate: that is integrated, so nearly noise-free,
  // and without it the law holds a standing error of Kd * bias / Kp.
  Vec3 tau;
  tau.x = (Kp_base[0] * gain_multiplier) * sign_w * q_err.x - (Kd_base[0] * gain_multiplier) * (gyro_raw.x - gyro_bias.x);
  tau.y = (Kp_base[1] * gain_multiplier) * sign_w * q_err.y - (Kd_base[1] * gain_multiplier) * (gyro_raw.y - gyro_bias.y);
  tau.z = (Kp_base[2] * gain_multiplier) * sign_w * q_err.z - (Kd_base[2] * gain_multiplier) * (gyro_raw.z - gyro_bias.z);

  Wire.beginTransmission(PLANT_ADDR);
  Wire.write(REG_TORQUE);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  // --- 5. UI: stream the framebuffer, new frame every >= 33 ms ---
  bool rendered = false;
  if (oled_sent < OLED_BYTES) {
    for (uint8_t i = 0; i < OLED_CHUNKS_PER_LOOP && oled_sent < OLED_BYTES; i++) oled_send_chunk();
  } else if (millis() - last_frame_millis >= 33) {
    last_frame_millis = millis();
    render_horizon();
    oled_start_frame();
    rendered = true;
  }

  // --- 6. TELEMETRY, ~30 Hz, never in the same iteration as a render ---
  if (!rendered && millis() - last_telemetry_millis >= 33) {
    last_telemetry_millis = millis();

    // --- GROUND TRUTH: SCORING ONLY ---
    // Read here, inside the telemetry block, and never above it. By the time this
    // runs the torque command has already gone out, and q_true lives in a local
    // that dies at the end of this scope -- so there is no name the control path
    // could refer to it by even by accident. M7's whole premise is that the
    // estimator does not get to see truth, and scope enforces that
    // better than a comment asking the next person not to.
    //
    // 16 bytes in one transaction. Four separate reads would assemble a
    // quaternion out of four different instants of a 1 kHz plant.
    Quat q_true = {1.0f, 0.0f, 0.0f, 0.0f};
    bool truth_ok = false;
    Wire.beginTransmission(PLANT_ADDR);
    Wire.write(REG_Q_TRUE);
    Wire.endTransmission(false);
    if (Wire.requestFrom(PLANT_ADDR, 16) == 16) {
      Wire.readBytes((uint8_t*)&q_true, 16);
      truth_ok = true;
    }

    // Telemetry for the Python dashboard.
    // Columns: q_est.w,q_est.x,q_est.y,q_est.z,q_true.w,q_true.x,q_true.y,q_true.z,max_dt_us
    // Raw quaternions rather than a precomputed error angle -- the scoring metric
    // should be changeable without reflashing, and the MCU has better things to do.
    // max_dt_us is the worst loop period since the previous line.
    print_fixed4(q_est.w); Serial.print(',');
    print_fixed4(q_est.x); Serial.print(',');
    print_fixed4(q_est.y); Serial.print(',');
    print_fixed4(q_est.z); Serial.print(',');
    if (truth_ok) {
      print_fixed4(q_true.w); Serial.print(',');
      print_fixed4(q_true.x); Serial.print(',');
      print_fixed4(q_true.y); Serial.print(',');
      print_fixed4(q_true.z);
    } else {
      Serial.print("nan,nan,nan,nan");
    }
    Serial.print(',');
    Serial.println(max_dt_us);
    max_dt_us = 0;
  }
}
