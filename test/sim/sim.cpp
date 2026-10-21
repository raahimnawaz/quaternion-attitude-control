#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "plant.h"
#include "wokwi-api.h"

namespace sim {

uint64_t now_ns = 0;
uint32_t scl_hz = 100000;
Oled oled;
void *plant = nullptr;
int adc[3] = {512, 512, 512};
int drop_permille = 0;
bool block_torque = false;
std::string serial_out;
Events ev;
BusStats stats;

static constexpr uint64_t ADC_NS = 104000;
static uint64_t n_ticks = 0, pending_ctrl_ns = 0, rng = 0x9E3779B97F4A7C15ull;
static Device *devices[128] = {};

struct Timer {
  void (*cb)(void *);
  void *ud;
  uint64_t period, next;
  bool on, repeat;
};
static std::vector<Timer> timers;

uint64_t ticks() { return n_ticks; }

void advance(uint64_t ns) {
  const uint64_t end = now_ns + ns;
  for (;;) {
    int due = -1;
    for (size_t i = 0; i < timers.size(); i++)
      if (timers[i].on && timers[i].next <= end && (due < 0 || timers[i].next < timers[due].next)) due = (int)i;
    if (due < 0) break;
    Timer &t = timers[due];
    now_ns = t.next;
    t.next += t.period;
    t.on = t.repeat;
    n_ticks++;
    t.cb(t.ud);
  }
  now_ns = end;
}

static uint64_t bit_ns() { return 1000000000ull / scl_hz; }

static bool drop_now() {
  rng ^= rng << 13, rng ^= rng >> 7, rng ^= rng << 17;  // own RNG: the plant's rand() stream stays put
  return (int)(rng % 1000) < drop_permille;
}

static void busy(uint8_t addr, uint64_t ns) {
  if (addr == IMU_ADDR) stats.busy_ns_imu += ns;
  if (addr == OLED_ADDR) stats.busy_ns_oled += ns;
  advance(ns);
}

uint8_t bus_write(uint8_t addr, const uint8_t *buf, uint8_t n, bool stop) {
  uint8_t tx[32];
  memcpy(tx, buf, n);
  if (addr == IMU_ADDR && n == 13 && tx[0] == 0x50) {
    advance(pending_ctrl_ns);
    pending_ctrl_ns = 0;
    if (block_torque) memset(tx + 1, 0, 12);
  }
  const uint64_t b = bit_ns();
  busy(addr, 10 * b);  // START + address/ACK
  Device *d = devices[addr];
  if (!d || !d->connect(false)) {
    busy(addr, b);
    return 2;
  }
  for (uint8_t i = 0; i < n; i++) {
    busy(addr, 9 * b);
    if (!d->write(tx[i])) return 3;
  }
  if (stop) {
    busy(addr, b);
    d->disconnect();
  }
  return 0;
}

uint8_t bus_read(uint8_t addr, uint8_t *buf, uint8_t n) {
  const uint64_t b = bit_ns();
  const bool imu = addr == IMU_ADDR && n == 24 && plant;
  Device *d = devices[addr];
  if (imu && drop_now()) d = nullptr;
  busy(addr, 10 * b);
  if (!d || !d->connect(true)) {
    busy(addr, b);
    return 0;
  }
  // What the pre-latch plant served: each byte from the live registers at
  // the moment it went out.
  uint8_t live[24], unlatched[24], at_start[24];
  const uint64_t t0 = ticks();
  if (imu) plant_live_imu(plant, at_start);
  for (uint8_t i = 0; i < n; i++) {
    if (imu) {
      plant_live_imu(plant, live);
      unlatched[i] = live[i];
    }
    buf[i] = d->read();
    busy(addr, 9 * b);
  }
  busy(addr, b);
  d->disconnect();
  if (imu) {
    ev.imu_ok = true;
    stats.imu_reads++;
    stats.imu_reads_100k += scl_hz == 100000;
    stats.imu_reads_400k += scl_hz == 400000;
    if (ticks() != t0) stats.straddled++;
    if (!memcmp(at_start, buf, 24)) stats.consistent++;
    if (memcmp(unlatched, buf, 24)) {
      stats.torn++;
      float u[6], l[6];
      memcpy(u, unlatched, 24);
      memcpy(l, buf, 24);
      for (int k = 0; k < 6; k++) stats.max_tear_err = fmaxf(stats.max_tear_err, fabsf(u[k] - l[k]));
    }
  }
  return n;
}

static uint8_t ssd1306_argc(uint8_t c) {
  switch (c) {
    case 0x20: case 0x81: case 0x8D: case 0xA8: case 0xD3:
    case 0xD5: case 0xD9: case 0xDA: case 0xDB: return 1;
    case 0x21: case 0x22: case 0xA3: return 2;
    case 0x29: case 0x2A: return 5;
    case 0x26: case 0x27: return 6;
    default: return 0;
  }
}

bool Oled::write(uint8_t b) {
  if (first) {  // control byte; the library only ever uses Co = 0
    first = false;
    data_mode = b & 0x40;
    return true;
  }
  if (data_mode) {
    ev.chunk = true;
    ram[page * 128 + col] = b;
    if (++col > col_hi) {
      col = col_lo;
      if (++page > page_hi) page = page_lo;
    }
    if (++data_bytes == sizeof ram) {
      frames_done++;
      if (on_frame_done) on_frame_done();
    }
    return true;
  }
  if (args_left) {
    args[nargs++] = b;
    if (--args_left) return true;
  } else {
    cmd = b;
    nargs = 0;
    if ((args_left = ssd1306_argc(b))) return true;
  }
  if (cmd == 0x20) mem_mode = args[0];
  if (cmd == 0x21) col_lo = col = args[0] & 127, col_hi = args[1] & 127;
  if (cmd == 0x22) {
    page_lo = page = args[0] & 7, page_hi = args[1] & 7;
    data_bytes = 0;
    frames_started++;
    ev.frame_start = true;
    if (on_frame_start) on_frame_start();
  }
  return true;
}

void reset() {
  now_ns = 0;
  scl_hz = 100000;
  timers.clear();
  memset(devices, 0, sizeof devices);
  oled = Oled();
  devices[OLED_ADDR] = &oled;
  plant = nullptr;
  drop_permille = 0;
  block_torque = false;
  serial_out.clear();
  stats = {};
  n_ticks = pending_ctrl_ns = 0;
}

static uint64_t cycles_ns(int64_t c) { return c > 0 ? (uint64_t)c * 1000 / 16 : 0; }

void run_iteration(void (*loop)(), const CpuCycles &cpu) {
  ev = {};
  pending_ctrl_ns = cycles_ns((int64_t)cpu.plain - 3 * cpu.bench_adc);
  loop();
  if (!ev.imu_ok) {  // early return on a dropped read: a few microseconds
    pending_ctrl_ns = 0;
    return;
  }
  advance(pending_ctrl_ns);
  pending_ctrl_ns = 0;
  uint32_t total = ev.frame_start ? (ev.serial ? cpu.frame_telem : cpu.frame)
                   : ev.chunk     ? (ev.serial ? cpu.chunk_telem : cpu.chunk)
                                  : (ev.serial ? cpu.plain_telem : cpu.plain);
  if (!total) {
    fprintf(stderr, "CPU model has no entry for this iteration (frame=%d chunk=%d serial=%d)\n",
            ev.frame_start, ev.chunk, ev.serial);
    abort();
  }
  advance(cycles_ns((int64_t)total - cpu.plain));
}

}  // namespace sim

// --- Arduino core and Wire, on the simulator --------------------------------

HardwareSerial Serial;
TwoWire Wire;
SPIClass SPI;

size_t HardwareSerial::write(uint8_t c) {
  sim::serial_out.push_back((char)c);
  sim::ev.serial = true;
  return 1;
}

uint32_t micros() { return (uint32_t)(sim::now_ns / 1000); }
uint32_t millis() { return (uint32_t)(sim::now_ns / 1000000); }
void delay(uint32_t ms) { sim::advance(ms * 1000000ull); }
void delayMicroseconds(unsigned us) { sim::advance(us * 1000ull); }
int analogRead(uint8_t pin) {
  sim::advance(sim::ADC_NS);
  return (pin >= A0 && pin <= A2) ? sim::adc[pin - A0] : 0;
}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }

void TwoWire::begin() { sim::scl_hz = 100000; }  // what AVR twi_init() sets
void TwoWire::setClock(uint32_t hz) { sim::scl_hz = hz; }
uint8_t TwoWire::endTransmission(uint8_t stop) { return sim::bus_write(addr_, tx_, txn_, stop); }
uint8_t TwoWire::requestFrom(uint8_t a, uint8_t n) {
  if (n > BUFFER_LENGTH) n = BUFFER_LENGTH;
  rxi_ = 0;
  rxn_ = sim::bus_read(a, rx_, n);
  return rxn_;
}

// --- Wokwi chip API ------------------------------------------------------

namespace {
struct ChipDevice : sim::Device {
  i2c_config_t c;
  bool connect(bool read) override { return c.connect(c.user_data, c.address, read); }
  uint8_t read() override { return c.read(c.user_data); }
  bool write(uint8_t b) override { return c.write(c.user_data, b); }
  void disconnect() override { c.disconnect(c.user_data); }
} chip_dev;
}  // namespace

extern "C" pin_t pin_init(const char *, uint32_t) { return 0; }

extern "C" uint32_t i2c_init(const i2c_config_t *c) {
  chip_dev.c = *c;
  sim::plant = c->user_data;
  sim::devices[c->address] = &chip_dev;
  return 1;
}

extern "C" uint32_t timer_init(const timer_config_t *c) {
  sim::timers.push_back({c->callback, c->user_data, 0, 0, false, false});
  return (uint32_t)sim::timers.size() - 1;
}

extern "C" void timer_start(uint32_t id, uint32_t us, bool repeat) {
  sim::Timer &t = sim::timers[id];
  t.period = us * 1000ull;
  t.next = sim::now_ns + t.period;
  t.on = true;
  t.repeat = repeat;
}
