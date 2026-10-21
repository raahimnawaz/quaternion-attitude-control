# Quaternion Attitude Control

Quaternion-based attitude determination and control for a rigid body in orbit,
running on an ATmega2560 against a physics plant simulated in a custom Wokwi
chip. The MCU never sees the true attitude — it estimates it from a noisy
accelerometer and gyroscope, and closes a PD loop on quaternion error.

The interesting constraint is that the AVR has **no floating-point unit**. Every
`sin`, `sqrt` and quaternion product is a software routine, and the control loop
still has to close at a usable rate.

---

## How it works

```
   ATmega2560  (sketch.ino)                     Custom chip  (chip/chip.chip.c)
  ┌────────────────────────────┐               ┌──────────────────────────────┐
  │ Mahony fusion  accel+gyro  │◄──── I²C ─────│ Euler's equations, RK4       │
  │        ↓                   │   0x42        │ Exponential-map quaternion   │
  │ q_est                      │   400 kHz     │ integration @ 1 kHz          │
  │        ↓                   │               │                              │
  │ PD law on quaternion error │──── τ ───────►│ Noisy accel + gyro out       │
  │        ↓                   │               │ q_true (scoring only)        │
  │ OLED horizon, streamed     │               └──────────────────────────────┘
  └────────────────────────────┘
     joystick → attitude command
     pot      → gain multiplier
```

The plant integrates angular velocity with RK4 and attitude with the exponential
map, which is norm-preserving by construction. The MCU runs a Mahony
complementary filter to fuse the accelerometer's gravity reference against gyro
integration, with an integral term that estimates the gyro's bias `b̂`, then
applies

```
q_err = q_est⁻¹ ⊗ q_cmd
τ     = Kp · sign(q_err.w) · q_err.vec  −  Kd · (ω_gyro − b̂)
```

`sign(q_err.w)` is the unwinding fix. `q` and `−q` are the same rotation, so a
negative scalar part means the controller is about to rotate the long way around;
negating flips it to the short path. It is four characters and it is the whole
point of the naive-vs-optimized comparison below.

### Conventions

Most quaternion bugs are two individually-correct pieces of code disagreeing
about convention. This project pins them:

| Thing | Choice |
|---|---|
| Storage order | `q = [w, x, y, z]` — scalar first |
| Algebra | Hamilton, not JPL |
| Meaning of `q` | rotates a vector **from body frame into world frame** |
| `ω`, `τ` | body frame; rad/s and N·m |
| Wire format | little-endian `float32` |
| Angles on the wire | radians — degrees only for display |

Euler angles are computed for the OLED and **never fed back into control**.

---

## Results

Apart from the one paragraph that quotes the old recorded Wokwi runs ([archived](#archive)), the numbers in
this section come from the software-in-the-loop harness in [`test/`](test/): the real `sketch.ino` and `chip/chip.chip.c`, unmodified,
built with the real Adafruit libraries on a simulated clock and I²C bus, with
per-iteration CPU time measured on a cycle-accurate AVR simulator. What that
does and does not model is spelled out under [Testing](#testing). None of it
has been re-run in Wokwi yet.

### Unwinding: the naive law goes the long way round

Both laws start from rest, 359° from the command, and run for 10 s:

![Attitude error from a 359° start, naive vs sign-corrected](results/unwinding-359.svg)

| start | naive: path travelled | corrected: path travelled | naive settles (< 2°) | corrected settles |
|---:|---:|---:|---:|---:|
| 181° | 193.6° | 192.4° | 2.23 s | 2.32 s |
| 270° | 283.4° | 100.0° | 2.71 s | 1.99 s |
| **359°** | **372.5°** | **4.5°** | **5.58 s** | **0 s** |

Path is integrated from the plant's ground truth, so it runs a few degrees past
the geometric distance (overshoot at ζ = 0.7, plus noise-driven jitter). The
noise comes from the C library's `rand()`, so the figures move by about a degree
between platforms: these are macOS, and CI's Linux run gives 372.4° and 4.3° at
359°.

At 359° the body is physically 1° off. The naive controller swings it through
180° of error and all the way round before it settles; the corrected one nudges
it 1°. At 181° the two are indistinguishable, because the long way is 181° and
the short way 179°. A 2° difference is inside the noise.

That is why the old recorded Wokwi runs ([archived](#archive)) never showed the
effect: they were taken at 181°. Their starting errors, 180.91° and 179.09°,
are one attitude measured two ways. Over the window both runs cover (to 2.74 s)
they travel **238.9° naive and 237.4° corrected**, and their different final
errors (1.24° vs 0.00°) are run length: the naive run stopped at 2.7 s, the
corrected one ran to 5.7 s. Those CSVs also predate the current plant (they
start from a 5 rad/s tumble, the chip now starts at rest), and they are kept
only as a record.

The SIL gets to 359° by starting the plant and the estimator together, as if
the body had been pushed once round. Wokwi has no hook for that yet. It would
go in bit 0 (reset) of the plant's `0x60` control register, which so far
implements only bit 2 (bias). The naive build is
`pio run -e unwinding_demo` (`-DUNWINDING_DEMO=1`).

`sign(q_err.w)` switches discontinuously at 180°, which the literature
(Mayhew, Sanfelice & Teel, 2011) shows can chatter under measurement noise, and
the standard fix is hysteresis. It was checked here and **not** added: from
exactly 180° the corrected law leaves the 170° band 27 ms later than it does
from 178°, and settles at the same moment.

### Gyro bias: roll and pitch recovered, yaw not

Setting bit 2 of `0x60` adds a constant bias of `[0.05, −0.03, 0.04]` rad/s to
the plant's gyro. The filter's integral term estimates it. Its gain is
`Ki_imu = Kp_imu²/4`, which critically damps the tilt error
(`s² + Kp_imu·s + Ki_imu`): it was picked from that, not tuned against the
test. Over 30 s at rest:

| | roll | pitch | yaw |
|---|---:|---:|---:|
| estimate at 1 s | 0.021 | −0.010 | 0.000 |
| at 3 s | 0.041 | −0.025 | 0.000 |
| at 10 s | 0.051 | −0.029 | 0.000 |
| **last 20 s, mean ± sd** | **0.0499 ± 0.0015** | **−0.0301 ± 0.0016** | **0.0000** |
| injected | 0.05 | −0.03 | 0.04 |

*rad/s. SIL, scored against the plant's ground truth.*

Roll and pitch are inside ±0.0075 rad/s from 3.2 s on. That band is 5σ of the
estimate's noise, which the linear model puts at 1.5 × 10⁻³ rad/s and the run
measures at 1.5–1.6 × 10⁻³. Over 20 noise seeds they settle in 2.6–3.3 s, and
the means land within 0.0004 rad/s of the injected values. Those are macOS's
`rand()`; on CI's Linux the same sweep gives 2.5–3.2 s and 0.0005 rad/s, and the
run above gives 0.0495 and −0.0296, settling at 2.8 s.

| last 20 s | estimated vertical off | body off level | heading at 30 s |
|---|---:|---:|---:|
| Kp only (before) | 1.31° | 2.87° | −69.0° |
| Kp + Ki | 0.20° | 0.20° | −69.0° |

0.20° is the accelerometer's noise floor: with no bias at all, the same runs
give 0.15–0.23° over 20 seeds.

**Yaw is not recovered, and the body turns.** Without a magnetometer, only the
part of the bias perpendicular to gravity is observable, and with the body
level that leaves yaw out. The yaw estimate stays at zero, the controller holds
the *estimate's* heading, and the real body rotates at −b_z: −69.0° in 30 s
(−b_z·t = −68.8°). Throughout, the estimate reports that it hasn't moved. The
SIL test asserts this failure along with the recovery, so a change that hides
it fails the build.

The derivative term uses the gyro minus `b̂`. Without that subtraction the PD
law holds a standing error of `Kd·b/Kp`: a one-off build without it sat 1.56°
off level over 20 seeds (modelled: 1.56°). It still never sees the
`Kp_imu`·residual term. With no bias, the integral term costs 0.05° of body
jitter against the Kp-only filter (20-seed average), 0.03° of it from this
subtraction. It also costs ~2,250 cycles a loop (simavr).

`test/build/sil --bias` prints the tables above, and `--bias-seeds 20` prints
the seed sweep.

### The control loop was running at 23 Hz

The diagram used to say 400 kHz and 30 Hz. Neither was true:

| | IMU bus | control rate | worst loop period | OLED |
|---|---:|---:|---:|---:|
| before (`73c03f0`) | 100 kHz | **22.9 Hz** | **45.3 ms** | 21.9 FPS |
| after (`9371b70`) | 400 kHz | **183.8 Hz** | **6.8 ms** | 10.0 FPS |
| now, with the bias estimator | 400 kHz | 178.8 Hz | 6.9 ms | 9.8 FPS |

*5 s with the stick deflected. CPU measured, bus and ADC modelled, see [Testing](#testing).*

Three things were stacked on top of each other:

1. **`display.begin()` reset the bus to 100 kHz.** Adafruit_SSD1306 restores the
   I²C clock to its `clkAfter` constructor argument after every transfer, and
   that defaults to 100 kHz, so `Wire.setClock(400000)` in `setup()` was undone
   one line later. The constructor now gets 400 kHz for both.
2. **`display()` blocked for ~30 ms.** It pushes 1,024 bytes plus headers in one
   call: ~25 ms of bus at 400 kHz, plus ~5 ms of CPU feeding `Wire.write()` (by
   subtraction from the measured UI iteration below). With
   the render and the telemetry, the UI block took longer than its own 33 ms
   timer, so it fired on *every* iteration. The framebuffer now streams two
   31-byte chunks per loop, ~1.5 ms of bus each time. Frames drop to ~10 FPS,
   which is plenty for an artificial horizon.
3. **Telemetry printed floats.** `Serial.print(x, 4)` costs ~9,000 cycles a value
   in soft float, and the line has eight of them. It now formats them as fixed
   point by subtraction: the same text (checked over 20,001 values in
   [-1, 1]) in a quarter of the cycles. It also never lands in the same
   iteration as a render, so the two worst cases don't stack.

Measured on simavr (ATmega2560, 16 MHz), with the bus stubbed out to isolate CPU:

| | cycles | time |
|---|---:|---:|
| `sin` / `cos` | 1,950 / 2,013 | 122 / 126 µs |
| `atan2` / `asin` | 3,108 / 1,308 | 194 / 82 µs |
| float multiply / divide | 145 / 495 | 9 / 31 µs |
| one control iteration, now (2 OLED chunks) | 42,471 | 2.7 ms |
| render iteration, now | 83,482 | 5.2 ms |
| telemetry iteration, now | 60,657 | 3.8 ms |
| UI iteration, before (render + `display()` + telemetry) | 233,066 | 14.6 ms |

Four trig calls build the command quaternion every loop, about 0.5 ms of the
2.7. The compiler had already folded `quat_from_euler`'s `yaw = 0` and merged
the render's repeated `sin(roll)`/`cos(roll)`; hand-hoisting those measured no
change and was dropped.

The firmware now reports this itself: the ninth telemetry column is the worst
loop period since the previous line, in µs. Run it in Wokwi and read that column
to check the model.

### Smaller fixes

- **A dropped IMU read lost its interval's rotation.** `last_micros` advanced
  before the read and the early return skipped the integration, so the
  estimator never saw that span. Spinning at 0.5 rad/s in yaw (which gravity
  can't correct) with 10% of reads dropped, the estimate was 11.8° off truth after
  4 s. With the next good sample integrating across the gap, it's 0.20°, which
  is the noise floor.
- **The plant latch holds.** The chip snapshots its registers when a transaction
  opens. In the SIL, 187 of 360 reads have a 1 kHz tick land mid-transfer, and all
  360 still return one sample. Without the latch 181 would have been spliced
  from two.
- The torque write still used bare `0x42`/`0x50` after the register names were
  introduced. It now uses `PLANT_ADDR`/`REG_TORQUE`.

### Gains

Treating each axis as `J·θ̈ = τ` and using the small-angle result `q_err.vec ≈ θ/2`,
the effective proportional gain is `Kp/2`:

```
ωn = sqrt(Kp / (2·J))          ζ = Kd / (2·sqrt(Kp·J/2))
```

With `J = [1, 2, 3]`, the shipped gains `Kp = [18, 36, 54]` and
`Kd = [4.2, 8.4, 12.6]` give **ωn = 3.0 rad/s and ζ = 0.7 on all three axes**.

The front-panel knob scales `Kp` and `Kd` together by `m ∈ [0.1, 3.0]`. Because
`ζ` depends on `Kd/sqrt(Kp)`, scaling both by `m` scales damping by `√m` — so the
knob sweeps ζ from 0.22 (ringing) to 1.21 (sluggish), passing through 0.7 at
m = 1. The knob is a damping control wearing a gain control's clothing.

---

### `invSqrt`: a small win on the AVR, and one place it was a loss

The Quake III reciprocal square root is worth having *or* worth deleting
depending entirely on whether the part has an FPU:

| target | FPU | verdict |
|---|---|---|
| **Arduino Mega** (ATmega2560), the Wokwi prototype | none | **wins, slightly:** 730 cycles against 979 for `1.0f/sqrtf` (simavr). That's ~16 µs a loop, 0.3% of the period |
| **ESP32** (Xtensa LX6) | yes | **should lose.** `1.0f/sqrtf(x)` is a couple of instructions and correctly rounded, and the trick also pays to move between integer and float register files. *Not measured.* |
| **STM32WBA55** (Cortex-M33F), the archived v1 board | yes | **should lose**, same reason. *Not measured.* |

`sketch.ino` selects on `__AVR__` rather than assuming, and the AVR path
type-puns through `memcpy`, not `*(long*)&y`, which is a strict-aliasing
violation.

One Newton step leaves a relative error of ~1.7e-3. That's fine for the
accelerometer, whose direction is all that matters, but not for `q_est`, which
it would leave biased off unit norm every loop. A second Newton step fixes the
accuracy, but on the AVR it measures **1,250–1,300 cycles (depending on how the
build inlines it): slower than the 979-cycle library call it was meant to beat**,
and still less accurate. So the quaternion now uses `1.0f/sqrtf` on every
target. That takes ~300 cycles off every loop (40,500 → 40,186 per iteration,
measured) and makes the normalisation exact.

---

## Known limitations

The simulation is honest about what it does not model. These are the places where
these results would not survive contact with hardware:

- **Yaw bias is unobservable.** There is no magnetometer, so the filter recovers
  the roll and pitch components of an injected gyro bias but never the yaw
  component, and the body turns at that rate while the estimate holds still
  (see above). The bias is also a constant: there's no temperature drift or
  random walk for the estimator to track.
- **Nothing in Wokwi turns the bias on.** `0x60` bit 2 works, but the firmware
  never writes it, so a Wokwi run has an unbiased gyro. Only the SIL sets it.
  Bits 0, 1 and 3 (reset, freeze, dropout) are still not built.
- **The accelerometer measures gravity and nothing else.** No linear
  acceleration is modelled, so the classic failure mode — a manoeuvring vehicle
  whose accelerometer stops pointing at the ground — is never exercised. This is
  the single largest gap between this sim and a real vehicle.
- **Noise is uniform, not Gaussian, and `rand()` is never seeded.** Every run is
  bit-identical. Good for reproducibility, useless for characterising variance.
- **The MCU Euler-integrates `q` and renormalises.** The plant uses the
  exponential map; the MCU does not. That's the standard Mahony formulation, but
  the difference is unmeasured.
- **`dt` is unclamped.** A stalled loop feeds an arbitrarily large step into the
  integrator. With the worst period now ~7 ms this only matters if the bus hangs.
  Clamping would be wrong too, because it throws the rotation away.
- **The old recorded Wokwi runs are archived, not reproduced.** They predate the
  current plant (a 5 rad/s tumble start; the chip now starts at rest) and were
  taken at 181°, where unwinding can't show. The SIL results above replace them.
- **Nothing here has run on silicon or been re-run in Wokwi.** The SIL's CPU
  costs are measured, but its bus timing is protocol arithmetic, its ADC is the
  datasheet figure, and its plant is the same chip code Wokwi runs.
- **Wokwi does not simulate ADC source impedance**, so the analog input circuit
  is validated only in principle.
- **The SIL's timing is a model.** See [Testing](#testing) for exactly which
  parts are measured.

---

## Running it

Open the project in [Wokwi](https://wokwi.com) — `diagram.json` and `sketch.ino`
must stay in the repository root, which is why the firmware is not tucked into a
subdirectory. The custom plant builds from `chip/chip.chip.c`.

Telemetry streams over serial at 115200 baud, 9 columns at 30 Hz:

```
q_est.w, q_est.x, q_est.y, q_est.z, q_true.w, q_true.x, q_true.y, q_true.z, max_dt_us
```

`max_dt_us` is the worst control-loop period since the previous line.

Attitude error in degrees is `2·acos(|⟨q_est, q_true⟩|)`, computed downstream so
the metric can change without reflashing.

> **`q_true` is for scoring only.** It is read inside the telemetry block, after
> the torque command has gone out, into a local that leaves scope immediately —
> so the control path has no name bound to it. The estimator does not get to see
> the answer.

---

## Testing

```bash
pio run -e megaatmega2560 -e unwinding_demo   # firmware, both laws; also fetches the libraries
make -C test                                  # SIL tests
make -C test before-after                     # CPU cycles on simavr + loop-timing table, 73c03f0 vs now
make -C test plot                             # results/unwinding-359.svg
test/build/sil --bias                         # gyro-bias estimate vs the injected value, Ki on and off
```

CI builds both firmwares and runs the tests and the simavr benchmark on every push.

The SIL compiles `sketch.ino` three times into separate namespaces (as is, with
`UNWINDING_DEMO=1`, and as of `73c03f0` for the before/after), against small
Arduino, Wire and Wokwi-chip shims. The Adafruit libraries are the real ones, so
the bus traffic they generate, including the clock reset, comes from their
code, not from a model of it. Each scenario runs in its own process, scored
against the plant's ground truth, which the firmware never sees.

What's measured and what's modelled:

| | source |
|---|---|
| firmware and plant logic | the real code, unmodified |
| CPU time per loop iteration | **measured**: cycle counts from simavr (`make -C test avr-bench`) for each kind of iteration, charged to the simulated clock |
| I²C time | **modelled**: 9 bit-times per byte plus START/STOP, at whatever clock `Wire` was last set to. Leaves out the TWI interrupt's per-byte service time, so it's a floor |
| `analogRead` | **modelled**: 104 µs, the datasheet's 13 ADC clocks at 16 MHz / 128 (simavr completes it in 26 µs) |
| plant | `chip/chip.chip.c` on a 1 kHz timer, interleaved with the bus byte by byte |

The tests check that IMU traffic runs at 400 kHz, that streamed frames reach the
OLED's RAM intact (rebuilt from the bytes on the wire), that no loop period
exceeds 8 ms, the 359° unwinding result, the dropped-read fix, the plant latch,
that the fixed-point telemetry matches `Serial.print(x, 4)`, and that an
injected gyro bias is recovered on roll and pitch and not on yaw.

## Layout

```
sketch.ino        firmware: fusion, control law, OLED, telemetry
diagram.json      Wokwi wiring
libraries.txt     Wokwi library manifest
platformio.ini    local AVR build: `pio run` (and `-e unwinding_demo` for the naive law)
chip/             custom plant: rigid-body dynamics + simulated IMU
test/             software-in-the-loop harness and the simavr cycle benchmark
results/          the unwinding plot above (`make -C test plot` regenerates it)
```

## Archive

Past results and versions aren't in the tree, but nothing is lost. The tag
[`archive-v1`](https://github.com/raahimnawaz/quaternion-attitude-control/tree/archive-v1)
holds the last full tree:

- `results/`: the recorded 181° Wokwi runs and their plots
- `docs/blueprint.md`: the original design document, with the register map and the traps the code comments used to cite
- `hardware/` and `docs/schematic.png`: the v1 STM32WBA55 schematic and its review. Eight netlist defects, never laid out; don't fabricate it as drawn

```bash
git checkout archive-v1 -- docs hardware results
```
