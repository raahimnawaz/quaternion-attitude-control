// Host software-in-the-loop: the real firmware and the real plant on one
// simulated clock and I2C bus.
//
// Timing model (everything else is free):
//   bus   9 bit-times per byte (8 + ACK) plus START/STOP, at whatever clock
//         Wire was last set to -- exact protocol arithmetic, but it leaves
//         out the TWI interrupt's service time per byte, so it's a floor.
//   ADC   104 us per analogRead: 13 ADC clocks at 16 MHz / 128 (datasheet).
//   CPU   per-iteration cycle counts measured on simavr (make avr-bench).
#pragma once
#include <stdint.h>
#include <functional>
#include <string>

namespace sim {

constexpr uint8_t IMU_ADDR = 0x42;
constexpr uint8_t OLED_ADDR = 0x3C;

extern uint64_t now_ns;
void advance(uint64_t ns);
uint64_t ticks();

struct Device {
  virtual ~Device() {}
  virtual bool connect(bool read) = 0;
  virtual uint8_t read() = 0;
  virtual bool write(uint8_t b) = 0;
  virtual void disconnect() = 0;
};

extern uint32_t scl_hz;
uint8_t bus_write(uint8_t addr, const uint8_t *buf, uint8_t n, bool stop);
uint8_t bus_read(uint8_t addr, uint8_t *buf, uint8_t n);

// SSD1306, horizontal addressing, rebuilt from the bytes on the wire.
struct Oled : Device {
  uint8_t ram[1024] = {};
  uint8_t col = 0, page = 0, col_lo = 0, col_hi = 127, page_lo = 0, page_hi = 7;
  bool first = false, data_mode = false;
  uint8_t cmd = 0, args[6] = {}, nargs = 0, args_left = 0, mem_mode = 0;
  uint32_t data_bytes = 0;
  int frames_started = 0, frames_done = 0;
  std::function<void()> on_frame_start, on_frame_done;

  bool connect(bool) override { first = true; return true; }
  uint8_t read() override { return 0; }
  bool write(uint8_t b) override;
  void disconnect() override {}
};
extern Oled oled;

extern void *plant;          // the chip's user_data, for ground truth
extern int adc[3];           // raw A0..A2
extern int drop_permille;    // IMU reads to NACK, per thousand
extern bool block_torque;    // zero the torque payload on the wire
extern std::string serial_out;

// Whole-loop() cycle counts from the AVR bench, by what the iteration did.
// 0 = "never happens for this firmware"; hitting one aborts the run.
struct CpuCycles {
  uint32_t plain, plain_telem, chunk, chunk_telem, frame, frame_telem;
  uint32_t bench_adc;  // simavr's analogRead, subtracted (3 per loop)
};

struct Events { bool imu_ok, frame_start, chunk, serial; };
extern Events ev;

struct BusStats {
  uint32_t imu_reads, imu_reads_100k, imu_reads_400k, straddled, torn, consistent;
  float max_tear_err;
  uint64_t busy_ns_imu, busy_ns_oled;
};
extern BusStats stats;

void reset();
// One loop() call, with its CPU time charged: the control part between the
// IMU read and the torque write, the rest after loop() returns.
void run_iteration(void (*loop)(), const CpuCycles &cpu);

}  // namespace sim
