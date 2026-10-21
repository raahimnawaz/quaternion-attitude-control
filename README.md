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
negating flips it to the short path.

The derivative term uses the measured rate minus `b̂`, never the filter's
corrected rate: that carries `Kp_imu` × the accelerometer residual, which would
feed accelerometer noise straight into torque. The bias gain is
`Ki_imu = Kp_imu²/4`, which critically damps the tilt error.

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

### Timing

Nothing on the control path blocks. The I²C bus runs at 400 kHz, the OLED
framebuffer streams two 31-byte chunks per loop instead of one 25 ms
`display()`, and telemetry is printed as fixed point rather than soft-float
`Serial.print`. On the AVR's soft float, `invSqrt` (the Quake III bit trick, one
Newton step) normalises the accelerometer in 730 cycles against 979 for
`1.0f/sqrtf`. The quaternion uses `1.0f/sqrtf`, because a second Newton step
costs more than the call it replaces.

---

## Results

Measured with a software-in-the-loop harness (the real firmware and plant on a
simulated clock and I²C bus) and a cycle-accurate AVR simulator. Both are in the
[archive](#archive), not in this tree.

| | Result |
|---|---|
| Control rate | 22.9 Hz → **179 Hz**; the worst loop period fell from 45.3 ms to 6.9 ms. The display library had reset the bus to 100 kHz, and a blocking `display()` ran on every iteration |
| Unwinding, from 359° | Without `sign(q_err.w)` the body travels 372°; with it, **4.5°** |
| Gyro bias | Roll and pitch recovered within 0.0004 rad/s in about 3 s (20 noise seeds). **Yaw is not**: without a magnetometer the body turns −69° in 30 s while the estimate says it hasn't moved |
| A dropped IMU read | Estimate 11.8° off truth before the fix, 0.2° after: the next good sample now integrates across the gap |

Bus and ADC timing in the harness are modelled; CPU time is measured. Nothing
here has been re-run in Wokwi or on silicon yet. The ninth telemetry column is
there to check it.

## Known limitations

- **Yaw is unobservable** without a magnetometer, so heading drifts with any
  yaw gyro bias.
- **The accelerometer measures gravity and nothing else.** No linear
  acceleration is modelled, so a manoeuvring vehicle whose accelerometer stops
  pointing at the ground is never exercised.
- **The plant's gyro has no bias.** The firmware estimates one, but a Wokwi run
  gives it nothing to find.
- **The MCU Euler-integrates `q` and renormalises**, while the plant uses the
  exponential map. That's the standard Mahony formulation; the difference is
  unmeasured.
- **`dt` is unclamped.** A stalled bus feeds one large step into the integrator.
  Clamping would be wrong too, because it throws the rotation away.

---

## Running it

Open the project in [Wokwi](https://wokwi.com) — `diagram.json` and `sketch.ino`
must stay in the repository root. The custom plant builds from
`chip/chip.chip.c`. To build the firmware locally:

```bash
pio run
```

Telemetry streams over serial at 115200 baud, 9 columns at 30 Hz:

```
q_est.w, q_est.x, q_est.y, q_est.z, q_true.w, q_true.x, q_true.y, q_true.z, max_dt_us
```

`max_dt_us` is the worst control-loop period since the previous line. Attitude
error in degrees is `2·acos(|⟨q_est, q_true⟩|)`, computed downstream so the
metric can change without reflashing.

> **`q_true` is for scoring only.** It is read inside the telemetry block, after
> the torque command has gone out, into a local that leaves scope immediately —
> so the control path has no name bound to it. The estimator does not get to see
> the answer.

## Layout

```
sketch.ino        firmware: fusion, control law, OLED, telemetry
chip/             custom plant: rigid-body dynamics + simulated IMU
diagram.json      Wokwi wiring
libraries.txt     Wokwi library manifest
platformio.ini    local AVR build
```

## Archive

Past results, versions and tooling aren't in the tree, but nothing is lost:

| Tag | Holds | Restore with |
|---|---|---|
| [`archive-v2`](https://github.com/raahimnawaz/quaternion-attitude-control/tree/archive-v2) | The test harness and simavr cycle benchmark behind every number above, CI, the naive control law, the plant's gyro-bias switch, and the full write-up of each result | `git checkout archive-v2 -- test .github results README.md` |
| [`archive-v1`](https://github.com/raahimnawaz/quaternion-attitude-control/tree/archive-v1) | The recorded 181° Wokwi runs and plots, the original design blueprint, and the v1 STM32WBA55 schematic with its review (eight netlist defects; don't fabricate it as drawn) | `git checkout archive-v1 -- docs hardware results` |
