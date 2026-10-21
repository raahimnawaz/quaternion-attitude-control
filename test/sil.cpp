// Software-in-the-loop tests: sketch.ino and chip/chip.chip.c, unmodified, on
// a simulated clock and I2C bus (see sim/sim.h for the timing model).
//
//   sil            run the tests
//   sil --timing   loop-timing table, baseline firmware vs now
//   sil --unwind   path travelled from 181/270/359 deg, naive vs corrected
//   sil --trace now|naive|baseline START_DEG [out.csv]   per-iteration CSV, 10 s
//   sil --bias [out.csv]   bias estimate vs the injected value, Ki on and off
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "sim/firmware.h"
#include "sim/plant.h"
#include "sim/sim.h"

struct Run {
  const Firmware *fw = &FW_NOW;
  double seconds = 5.0;
  int joy_x = 512, joy_y = 512;
  int pot = 317;             // gain multiplier 1.0: wn = 3 rad/s, zeta = 0.7
  double start_deg = 0;      // initial rotation about x, plant and estimator alike
  float w0[3] = {0, 0, 0};
  bool free_spin = false;    // torque payload zeroed on the wire
  int drop_permille = 0;
  bool inject_bias = false;  // 0x60 bit2, written on the bus after setup()
  float bias[3] = {0, 0, 0};
  bool ki_off = false;       // Kp-only Mahony, the filter before the integral term
  unsigned seed = 1;
  const char *trace = nullptr;  // per-iteration CSV
};

struct Result {
  double path_deg, max_err_deg, final_err_deg, settle_s;
  double est_err_max_deg, est_err_final_deg;
  double period_mean_us, period_p50_us, period_p99_us, period_max_us;
  double imu_busy, oled_busy, fps;
  uint32_t iters, frames_done, frame_mismatch, blank_frames;
  uint32_t telemetry_lines, telemetry_bad;
  sim::BusStats bus;
  // Means over the last TAIL_S of the run. Tilt is the angle between two
  // "down" directions in the body frame, so it ignores heading.
  double gyro_offset[3];      // plant gyro minus true rate, whole run
  double tilt_deg;            // true body vs the level command
  double est_tilt_deg;        // estimated vs true vertical
  double yaw_deg;             // true heading at the end (command is 0)
  double bias_est[3];         // firmware's estimate
  double bias_sd[3];          // ... and its noise
  double bias_settle_s;       // last time roll or pitch was off by more than BIAS_TOL
  float bias_at[7][3];        // estimate at BIAS_AT_S
};

static constexpr double TAIL_S = 20.0;
// The estimate's noise, linearised: sigma^2 = S Ki^2 / (2 Kp), with S the
// accelerometer's angle noise (0.05/sqrt(3) rad per sample, every ~5.4 ms):
// 1.5e-3 rad/s. The band is 5 sigma.
static constexpr double BIAS_TOL = 0.0075;  // rad/s
static constexpr double BIAS_AT_S[7] = {0.5, 1, 2, 3, 5, 10, 30};

static double deg(double rad) { return rad * 180.0 / M_PI; }

// Rotation angle of a quaternion, sign-invariant. atan2 rather than acos(|w|):
// acos is ill-conditioned near 1 and turns float rounding into ~0.04 deg of
// phantom rotation per sample.
static double angle_of(double w, double x, double y, double z) {
  return deg(2.0 * atan2(sqrt(x * x + y * y + z * z), fabs(w)));
}

static double angle_between(const float a[4], const float b[4]) {  // angle of conj(a) * b
  double w = (double)a[0] * b[0] + (double)a[1] * b[1] + (double)a[2] * b[2] + (double)a[3] * b[3];
  double x = (double)a[0] * b[1] - (double)a[1] * b[0] - (double)a[2] * b[3] + (double)a[3] * b[2];
  double y = (double)a[0] * b[2] + (double)a[1] * b[3] - (double)a[2] * b[0] - (double)a[3] * b[1];
  double z = (double)a[0] * b[3] - (double)a[1] * b[2] + (double)a[2] * b[1] - (double)a[3] * b[0];
  return angle_of(w, x, y, z);
}

// Body-frame "down" for q (body to world): the third row of R(q).
static void down_of(const float q[4], double g[3]) {
  g[0] = 2.0 * ((double)q[1] * q[3] - (double)q[0] * q[2]);
  g[1] = 2.0 * ((double)q[0] * q[1] + (double)q[2] * q[3]);
  g[2] = (double)q[0] * q[0] - (double)q[1] * q[1] - (double)q[2] * q[2] + (double)q[3] * q[3];
}

static double angle_between_vec(const double a[3], const double b[3]) {
  const double c[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  return deg(atan2(sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
}

static void check_telemetry(const std::string &out, Result &res) {
  size_t pos = 0;
  for (size_t nl; (nl = out.find('\n', pos)) != std::string::npos; pos = nl + 1) {
    std::string line = out.substr(pos, nl - pos);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    float q[4];
    unsigned long gap;
    int fields = sscanf(line.c_str(), "%f,%f,%f,%f,%lu", &q[0], &q[1], &q[2], &q[3], &gap);
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    res.telemetry_lines++;
    if (fields < 4 || fabs(norm - 1.0) > 2e-3) res.telemetry_bad++;
  }
}

static Result simulate(const Run &r) {
  const Firmware &fw = *r.fw;
  Result res = {};
  std::vector<uint8_t> shown(1024);
  sim::reset();
  plant_init(r.seed);
  sim::adc[0] = r.joy_x, sim::adc[1] = r.joy_y, sim::adc[2] = r.pot;
  sim::oled.on_frame_start = [&] { memcpy(shown.data(), fw.framebuffer(), shown.size()); };
  sim::oled.on_frame_done = [&] {
    res.frames_done++;
    if (memcmp(sim::oled.ram, shown.data(), shown.size())) res.frame_mismatch++;
    if (std::all_of(shown.begin(), shown.end(), [](uint8_t b) { return b == 0; })) res.blank_frames++;
  };
  fw.setup();
  res.frames_done = res.frame_mismatch = res.blank_frames = 0;
  const size_t serial_from = sim::serial_out.size();

  const double h = M_PI / 180.0 * r.start_deg / 2.0;
  const float q0[4] = {(float)cos(h), (float)sin(h), 0, 0};
  plant_set_state(sim::plant, q0, r.w0);
  memcpy(fw.q_est, q0, sizeof q0);
  if (r.ki_off) *fw.ki = 0;
  sim::drop_permille = r.drop_permille;
  sim::block_torque = r.free_spin;
  if (r.inject_bias) {
    plant_set_gyro_bias(sim::plant, r.bias);
    const uint8_t ctrl[2] = {0x60, 0x04};  // REG_CONTROL, inject bias
    if (sim::bus_write(sim::IMU_ADDR, ctrl, 2, true)) abort();
  }
  sim::stats = {};

  const uint64_t t0 = sim::now_ns, t1 = t0 + (uint64_t)(r.seconds * 1e9);
  float q_prev[4];
  memcpy(q_prev, q0, sizeof q0);
  std::vector<uint64_t> starts;
  const uint64_t tail_from = t1 - (uint64_t)(std::min(TAIL_S, r.seconds) * 1e9);
  uint32_t n_tail = 0;
  static const double level[3] = {0, 0, 1};
  FILE *trace = r.trace ? fopen(r.trace, "w") : nullptr;
  if (trace) fputs("t_s,err_deg,path_deg,est_err_deg,qw,qx,qy,qz,wx,wy,wz,bias_x,bias_y,bias_z\n", trace);
  const float no_bias[3] = {NAN, NAN, NAN};
  const float *bias_est = fw.gyro_bias ? fw.gyro_bias : no_bias;
  size_t next_at = 0;
  while (sim::now_ns < t1) {
    const uint64_t t_iter = sim::now_ns;
    sim::run_iteration(fw.loop, fw.cpu);
    if (sim::ev.imu_ok) starts.push_back(t_iter);
    float q[4], w[3];
    plant_truth(sim::plant, q, w);
    res.path_deg += angle_between(q_prev, q);
    memcpy(q_prev, q, sizeof q);
    const double err = angle_of(q[0], q[1], q[2], q[3]);  // vs the centred-stick command
    res.max_err_deg = std::max(res.max_err_deg, err);
    res.final_err_deg = err;
    if (err > 2.0) res.settle_s = (sim::now_ns - t0) / 1e9;
    const double est = angle_between(fw.q_est, q);
    res.est_err_max_deg = std::max(res.est_err_max_deg, est);
    res.est_err_final_deg = est;
    res.iters++;
    uint8_t imu[24];
    float gyro[3];
    plant_live_imu(sim::plant, imu);
    memcpy(gyro, imu + 12, sizeof gyro);
    for (int k = 0; k < 3; k++) res.gyro_offset[k] += gyro[k] - w[k];
    const double t = (sim::now_ns - t0) / 1e9;
    if (fabs(bias_est[0] - r.bias[0]) > BIAS_TOL || fabs(bias_est[1] - r.bias[1]) > BIAS_TOL) res.bias_settle_s = t;
    for (; next_at < 7 && t >= BIAS_AT_S[next_at] - 1e-9; next_at++) memcpy(res.bias_at[next_at], bias_est, 12);
    if (t_iter >= tail_from) {
      double down_true[3], down_est[3];
      down_of(q, down_true);
      down_of(fw.q_est, down_est);
      res.tilt_deg += angle_between_vec(down_true, level);
      res.est_tilt_deg += angle_between_vec(down_true, down_est);
      for (int k = 0; k < 3; k++) res.bias_est[k] += bias_est[k], res.bias_sd[k] += bias_est[k] * bias_est[k];
      n_tail++;
    }
    if (trace)
      fprintf(trace, "%.5f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", t, err,
              res.path_deg, est, q[0], q[1], q[2], q[3], w[0], w[1], w[2], bias_est[0], bias_est[1], bias_est[2]);
  }
  if (trace) fclose(trace);
  for (int k = 0; k < 3; k++) res.gyro_offset[k] /= res.iters;
  res.tilt_deg /= n_tail;
  res.est_tilt_deg /= n_tail;
  for (int k = 0; k < 3; k++) {
    res.bias_est[k] /= n_tail;
    res.bias_sd[k] = sqrt(std::max(0.0, res.bias_sd[k] / n_tail - res.bias_est[k] * res.bias_est[k]));
  }
  res.yaw_deg = deg(2.0 * atan2(q_prev[3], q_prev[0]));

  std::vector<double> periods;
  for (size_t i = 1; i < starts.size(); i++) periods.push_back((starts[i] - starts[i - 1]) / 1e3);
  std::sort(periods.begin(), periods.end());
  if (!periods.empty()) {
    double sum = 0;
    for (double p : periods) sum += p;
    res.period_mean_us = sum / periods.size();
    res.period_p50_us = periods[periods.size() / 2];
    res.period_p99_us = periods[std::min(periods.size() - 1, periods.size() * 99 / 100)];
    res.period_max_us = periods.back();
  }
  const double span = (double)(sim::now_ns - t0);
  res.imu_busy = sim::stats.busy_ns_imu / span;
  res.oled_busy = sim::stats.busy_ns_oled / span;
  res.fps = res.frames_done / (span / 1e9);
  res.bus = sim::stats;
  check_telemetry(sim::serial_out.substr(serial_from), res);
  return res;
}

// Each scenario in its own process: every firmware's globals start fresh.
template <class T>
static T isolated(const std::function<T()> &f) {
  int fd[2];
  if (pipe(fd)) abort();
  fflush(stdout);
  pid_t pid = fork();
  if (pid == 0) {
    close(fd[0]);
    T r = f();
    if (write(fd[1], &r, sizeof r) != (ssize_t)sizeof r) _exit(1);
    _exit(0);
  }
  close(fd[1]);
  T r{};
  size_t got = 0;
  for (ssize_t k; got < sizeof r && (k = read(fd[0], (char *)&r + got, sizeof r - got)) > 0;) got += k;
  close(fd[0]);
  int status = 0;
  waitpid(pid, &status, 0);
  if (got != sizeof r || !WIFEXITED(status) || WEXITSTATUS(status)) {
    fprintf(stderr, "scenario process failed\n");
    exit(2);
  }
  return r;
}

static Result run(const Run &r) { return isolated<Result>([&] { return simulate(r); }); }

static int failures = 0;
#define CHECK(cond, ...)                                     \
  do {                                                       \
    bool ok_ = (cond);                                       \
    printf("  %s  ", ok_ ? "ok  " : "FAIL");                 \
    printf(__VA_ARGS__);                                     \
    printf("\n");                                            \
    if (!ok_) failures++;                                    \
  } while (0)

static void test_bus_clock() {
  puts("IMU transactions run at the 400 kHz setup() asks for");
  Run r;
  r.seconds = 1;
  Result now = run(r);
  r.fw = &FW_LEGACY;
  Result old = run(r);
  CHECK(now.bus.imu_reads > 100 && now.bus.imu_reads_400k == now.bus.imu_reads,
        "now: %u/%u reads at 400 kHz", now.bus.imu_reads_400k, now.bus.imu_reads);
  CHECK(old.bus.imu_reads_100k == old.bus.imu_reads,
        "baseline reproduces the reset: %u/%u reads at 100 kHz", old.bus.imu_reads_100k, old.bus.imu_reads);
}

static void test_oled_frames() {
  puts("Streamed OLED frames land intact");
  Run r;
  r.seconds = 2;
  r.joy_x = 700, r.joy_y = 400;
  Result res = run(r);
  CHECK(res.frames_done >= 10, "%u frames completed in 2 s (%.1f FPS)", res.frames_done, res.fps);
  CHECK(res.frame_mismatch == 0, "%u frames differ from the framebuffer they were sent from", res.frame_mismatch);
  CHECK(res.blank_frames == 0, "%u blank frames", res.blank_frames);
}

static void test_loop_period() {
  puts("The control loop never stalls behind the display");
  Run r;
  r.seconds = 3;
  r.joy_x = 700, r.joy_y = 400;
  Result res = run(r);
  CHECK(res.period_max_us < 8000, "worst period %.0f us (mean %.0f, p99 %.0f)",
        res.period_max_us, res.period_mean_us, res.period_p99_us);
}

static void test_unwinding() {
  puts("Sign correction takes the short way from 359 deg; the naive law unwinds");
  Run r;
  r.seconds = 10;
  r.start_deg = 359;
  Result fixed = run(r);
  r.fw = &FW_NAIVE;
  Result naive = run(r);
  CHECK(fixed.path_deg < 10, "corrected: travelled %.1f deg", fixed.path_deg);
  CHECK(naive.path_deg > 300, "naive: travelled %.1f deg", naive.path_deg);
  CHECK(fixed.final_err_deg < 2 && naive.final_err_deg < 2,
        "both settle: %.2f / %.2f deg after 10 s", fixed.final_err_deg, naive.final_err_deg);
}

static void test_dropped_reads() {
  puts("A dropped IMU read does not lose that interval's rotation");
  Run r;
  r.seconds = 4;
  r.free_spin = true;
  r.w0[2] = 0.5f;  // yaw: gravity can't correct it, so lost rotation stays lost
  r.drop_permille = 100;
  Result now = run(r);
  r.fw = &FW_LEGACY;
  Result old = run(r);
  CHECK(now.est_err_final_deg < 1.0, "now: estimate %.2f deg off truth after 4 s at 0.5 rad/s, 10%% drops",
        now.est_err_final_deg);
  printf("        (baseline: %.1f deg)\n", old.est_err_final_deg);
}

static void test_consistent_bursts() {
  puts("Each IMU burst is one sample, even when a plant tick lands mid-read");
  Run r;
  r.seconds = 2;
  r.start_deg = 30;
  Result res = run(r);
  CHECK(res.bus.straddled > 0, "%u of %u reads had a 1 kHz tick land mid-transfer",
        res.bus.straddled, res.bus.imu_reads);
  CHECK(res.bus.consistent == res.bus.imu_reads, "%u of %u reads are one sample, taken as the read opened",
        res.bus.consistent, res.bus.imu_reads);
  printf("        (without the latch %u would have been spliced, worst value off by %.3g)\n",
         res.bus.torn, res.bus.max_tear_err);
}

static void test_telemetry() {
  puts("Fixed-point telemetry prints what Serial.print(x, 4) did");
  int mismatches = isolated<int>([&] {
    int bad = 0;
    for (int i = -10000; i <= 10000; i++) {
      float v = i / 10000.0f + 0.00004f * ((i * 7919) % 3 - 1);
      sim::serial_out.clear();
      FW_NOW.print_fixed4(v);
      char ref[16];
      snprintf(ref, sizeof ref, "%.4f", v);
      if (sim::serial_out != ref && !(strcmp(ref, "-0.0000") == 0 && sim::serial_out == "0.0000")) bad++;
    }
    return bad;
  });
  CHECK(mismatches == 0, "%d of 20001 values in [-1, 1] format differently", mismatches);
  Run r;
  r.seconds = 2;
  r.start_deg = 30;
  Result res = run(r);
  CHECK(res.telemetry_lines >= 50 && res.telemetry_bad == 0, "%u lines in 2 s, %u malformed",
        res.telemetry_lines, res.telemetry_bad);
}

// Blueprint M7 step 3. Roll/pitch bias is observable from the accelerometer;
// yaw bias is not (no magnetometer).
static constexpr float BIAS[3] = {0.05f, -0.03f, 0.04f};  // rad/s, the chip's default

static Run bias_run() {
  Run r;
  r.seconds = 30;
  r.inject_bias = true;
  memcpy(r.bias, BIAS, sizeof BIAS);
  return r;
}

static void test_bias_recovered() {
  puts("The integral term recovers the injected roll/pitch gyro bias; yaw bias is unobservable");
  Run r = bias_run();
  Result res = run(r);
  r.ki_off = true;
  Result kp = run(r);
  double plant_err = 0;
  for (int k = 0; k < 3; k++) plant_err = std::max(plant_err, fabs(res.gyro_offset[k] - BIAS[k]));
  CHECK(plant_err < 1e-3, "0x60 bit2: plant gyro minus true rate averages [%.4f, %.4f, %.4f] rad/s",
        res.gyro_offset[0], res.gyro_offset[1], res.gyro_offset[2]);
  CHECK(fabs(res.bias_est[0] - BIAS[0]) < 1.5e-3 && fabs(res.bias_est[1] - BIAS[1]) < 1.5e-3,
        "roll/pitch estimate [%.4f, %.4f] +/- [%.4f, %.4f] rad/s over the last %.0f s, injected [%.2f, %.2f]",
        res.bias_est[0], res.bias_est[1], res.bias_sd[0], res.bias_sd[1], TAIL_S, BIAS[0], BIAS[1]);
  CHECK(res.bias_settle_s < 5, "within %.4f rad/s from %.2f s on, and for the rest of %.0f s", BIAS_TOL,
        res.bias_settle_s, r.seconds);
  // With no bias at all these are 0.15-0.23 deg over 20 seeds: the accelerometer's noise floor.
  CHECK(res.est_tilt_deg < 0.3 && res.tilt_deg < 0.3,
        "estimated vertical %.2f deg off, body %.2f deg off level (Kp only: %.2f, %.2f)", res.est_tilt_deg,
        res.tilt_deg, kp.est_tilt_deg, kp.tilt_deg);
  // The honest failure. Level, body z is the gravity axis, so the residual
  // a x g_est has no z component to integrate: the yaw estimate stays near 0,
  // the PD law holds the *estimate's* heading, and the body turns at -b_z.
  CHECK(fabs(res.bias_est[2]) < 0.1 * BIAS[2], "yaw NOT recovered: estimate %.4f rad/s, injected %.2f",
        res.bias_est[2], BIAS[2]);
  const double drift = -deg(BIAS[2] * r.seconds);
  CHECK(fabs(res.yaw_deg - drift) < 0.1 * fabs(drift),
        "yaw drifts: heading %.1f deg after %.0f s (-b_z t = %.1f), estimate %.1f deg off truth", res.yaw_deg,
        r.seconds, drift, res.est_err_final_deg);
}

static void report_timing() {
  puts("Loop timing, 5 s, stick deflected (bus is a floor: no TWI ISR time)\n");
  puts("firmware   IMU bus  mean us   p50 us   p99 us   max us   ctrl Hz   OLED FPS  OLED bus  IMU bus");
  for (const Firmware *fw : {&FW_LEGACY, &FW_NOW}) {
    Run r;
    r.fw = fw;
    r.joy_x = 700, r.joy_y = 400;
    Result s = run(r);
    printf("%-9s  %4u kHz %8.0f %8.0f %8.0f %8.0f %9.1f %10.1f %8.1f%% %7.1f%%\n", fw->name,
           s.bus.imu_reads_400k == s.bus.imu_reads ? 400 : 100, s.period_mean_us, s.period_p50_us,
           s.period_p99_us, s.period_max_us, 1e6 / s.period_mean_us, s.fps, 100 * s.oled_busy,
           100 * s.imu_busy);
  }
}

static void report_bias(const char *trace) {
  Run r = bias_run();
  printf("Gyro bias estimate, %.0f s at rest, injected [%.2f, %.2f, %.2f] rad/s\n\n", r.seconds, BIAS[0], BIAS[1],
         BIAS[2]);
  r.trace = trace;
  Result res = run(r);
  r.trace = nullptr;
  r.ki_off = true;
  Result kp = run(r);
  puts("  t s     roll     pitch      yaw");
  for (int i = 0; i < 7; i++)
    printf("%5.1f %8.4f %9.4f %8.4f\n", BIAS_AT_S[i], res.bias_at[i][0], res.bias_at[i][1], res.bias_at[i][2]);
  printf("last %.0f s %6.4f %9.4f %8.4f  mean\n", TAIL_S, res.bias_est[0], res.bias_est[1], res.bias_est[2]);
  printf("%15.4f %9.4f %8.4f  sd\n", res.bias_sd[0], res.bias_sd[1], res.bias_sd[2]);
  printf("\nroll/pitch within %.4f rad/s from %.2f s on\n", BIAS_TOL, res.bias_settle_s);
  printf("\nlast %.0f s       est. vertical off   body off level   heading at %.0f s\n", TAIL_S, r.seconds);
  printf("Kp only         %10.2f deg %13.2f deg %14.1f deg\n", kp.est_tilt_deg, kp.tilt_deg, kp.yaw_deg);
  printf("Kp + Ki         %10.2f deg %13.2f deg %14.1f deg\n", res.est_tilt_deg, res.tilt_deg, res.yaw_deg);
}

static void report_unwinding() {
  puts("Path travelled to settle, 10 s from rest (deg)\n");
  puts("start   naive path  corrected path   naive settle s  corrected settle s");
  for (double a : {181.0, 270.0, 359.0}) {
    Run r;
    r.seconds = 10;
    r.start_deg = a;
    Result fixed = run(r);
    r.fw = &FW_NAIVE;
    Result naive = run(r);
    printf("%5.0f %12.1f %15.1f %16.2f %19.2f\n", a, naive.path_deg, fixed.path_deg, naive.settle_s,
           fixed.settle_s);
  }
}

int main(int argc, char **argv) {
  if (argc > 3 && !strcmp(argv[1], "--trace")) {  // --trace now|naive|baseline START_DEG [out.csv]
    Run r;
    r.fw = !strcmp(argv[2], "naive") ? &FW_NAIVE : !strcmp(argv[2], "baseline") ? &FW_LEGACY : &FW_NOW;
    r.seconds = 10;
    r.start_deg = atof(argv[3]);
    r.trace = argc > 4 ? argv[4] : "trace.csv";
    run(r);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "--timing")) return report_timing(), 0;
  if (argc > 1 && !strcmp(argv[1], "--unwind")) return report_unwinding(), 0;
  if (argc > 1 && !strcmp(argv[1], "--bias")) return report_bias(argc > 2 ? argv[2] : nullptr), 0;
  test_bus_clock();
  test_oled_frames();
  test_loop_period();
  test_unwinding();
  test_dropped_reads();
  test_consistent_bursts();
  test_telemetry();
  test_bias_recovered();
  printf("\n%s\n", failures ? "FAILED" : "all passed");
  return failures ? 1 : 0;
}
