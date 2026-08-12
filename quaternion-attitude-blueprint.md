# Quaternion Attitude Control on Wokwi — build blueprint

A rigid body you write, a controller you write, and ground truth you own.

**What exists at the end:** a Wokwi custom chip that integrates rigid-body
rotational dynamics, an ATmega2560 running quaternion feedback control against
it, a joystick and sensitivity pot you wire yourself, an artificial horizon on a
128×64 OLED, and CI that runs the whole loop headlessly on every push.

You author two things rather than assembling existing parts: **the plant** (C
compiled to WASM, §M0–M1) and **the control law** (§M3–M6). The pot turns out to
be the most interesting knob in the project — §M6 uses it to locate a stability
boundary that moves depending on how fast your display bus runs.

**Why it's portfolio-grade:** on real hardware you never know the true attitude —
you only have the same sensors the controller has. Here you wrote the plant, so
you know the exact answer and can *quantify* how wrong the estimator is. That is
the claim no amount of physical hardware can make, and it's the reason this is
worth building even though you already have real boards on your résumé.

Deliberately **not** a substitute for `vision_demos` Phase 4. That one needs a
physical actuator and this does not replace it. Different claims — keep them
separate.

---

## 0. Read this first: scope discipline

The build is gated into milestones M0–M8. **Each one ends with something that
runs and a specific thing you should observe.** If a milestone's verification
doesn't reproduce, do not move on — every later bug will be blamed on the wrong
layer.

The single most common way this project dies is writing the controller before
the plant is trustworthy. M1 exists to prevent that.

---

## 1. Conventions — pin these before writing any code

Ninety percent of quaternion bugs are convention mismatches between two pieces of
code that are each individually correct. Write these at the top of every file.

| Thing | Choice |
|---|---|
| Storage order | `q = [w, x, y, z]` — **scalar first** |
| Algebra | **Hamilton** (not JPL) |
| Meaning of `q` | rotates a vector **from body frame into world frame** |
| `ω` | angular velocity in the **body** frame, rad/s |
| Torque `τ` | body frame, N·m |
| Inertia `J` | diagonal, body principal axes, kg·m² |
| Wire format | little-endian `float32` (AVR and WASM agree) |
| Angles on the wire | radians. Degrees only for display. |

> **Trap.** JPL convention (used by some aerospace texts and by Eigen's internal
> storage order) puts the scalar *last* and flips the sign of the product. If you
> copy a formula from a paper, check which convention it's in before trusting it.

---

## 2. The math, written out

### 2.1 Hamilton product

`r = q ⊗ p`:

```
r.w = q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z
r.x = q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y
r.y = q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x
r.z = q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
```

Conjugate: `q* = [w, -x, -y, -z]`. For a **unit** quaternion, `q⁻¹ = q*` — which
is why keeping `‖q‖ = 1` is not cosmetic. If the norm drifts, your "inverse" is
silently not an inverse.

### 2.2 Kinematics

```
q̇ = ½ · q ⊗ [0, ωx, ωy, ωz]
```

This form is correct **only** because `ω` is in the body frame and `q` maps
body→world. World-frame `ω` needs `q̇ = ½ · [0,ω] ⊗ q`. Getting this backwards
produces a plant that looks plausible and rotates about the wrong axes.

### 2.3 Rigid-body dynamics (Euler's equations)

```
ω̇ = J⁻¹ · (τ − ω × (J·ω))
```

For diagonal `J = diag(Jx, Jy, Jz)` this expands to:

```
ω̇x = (τx − (Jz − Jy)·ωy·ωz) / Jx
ω̇y = (τy − (Jx − Jz)·ωz·ωx) / Jy
ω̇z = (τz − (Jy − Jx)·ωx·ωy) / Jz
```

The cross-coupling terms are what make this real physics rather than three
independent integrators. **They are also your free correctness test** — see M1.

### 2.4 Exact quaternion integration

Don't Euler-integrate `q̇` and renormalize. For (approximately) constant `ω` over
a step `Δt`, the exact update is the exponential map:

```
θ = ‖ω‖ · Δt / 2
if θ < 1e-8:  δq = [1, ωx·Δt/2, ωy·Δt/2, ωz·Δt/2]   // small-angle, avoids /0
else:         axis = ω / ‖ω‖
              δq = [cos θ, axis.x·sin θ, axis.y·sin θ, axis.z·sin θ]
q ← q ⊗ δq
```

This is norm-preserving by construction. Renormalize anyway once every N steps to
mop up float error — but you should find the drift is tiny compared to
Euler-integrating, and **measuring that difference is one of your write-up
plots**.

Integrate `ω` with RK4. `ω` is where the stiffness is, `q` is not.

### 2.5 The control law

```
q_err = q_est⁻¹ ⊗ q_cmd            // rotation from current body to desired, in body frame
τ = Kp · sign(q_err.w) · q_err.vec  −  Kd · ω
```

`q_err.vec` is `[x, y, z]` — the axis-times-sin(θ/2) part. It's already in the
body frame, which is the frame your torques act in, which is why this law is so
compact.

**`sign(q_err.w)` is the entire unwinding fix.** Since `cos(θ/2) = q_err.w`, a
negative scalar part means `θ > 180°` — the controller is about to take the long
way around. Because `q` and `−q` are the same rotation, negating flips it to the
short path. Four characters, and M4 is built around demonstrating it.

> **Sign convention warning.** Whether it's `+Kp` or `−Kp` depends on how you
> defined `q_err` (`q_est⁻¹⊗q_cmd` vs `q_cmd⁻¹⊗q_est`). Both appear in the
> literature. **Empirical test:** command a small step. If the error grows
> instead of shrinking, flip the sign. Don't agonise — verify.

### 2.6 Gains

Treat each axis as `J·θ̈ = τ` near the setpoint. The small-angle linearisation of
`q_err.vec ≈ θ/2 · axis` gives you an effective proportional gain of `Kp/2`, so:

```
ωn = sqrt(Kp / (2·J))        natural frequency
ζ  = Kd / (2·sqrt(Kp·J/2))   damping ratio
```

Pick `ωn` ≈ 2–5 rad/s and `ζ` ≈ 0.7, then solve for `Kp`, `Kd`. Start there,
tune after. **Write down what you predicted before you run it**, then compare —
that comparison is worth more in the write-up than a well-tuned response.

### 2.7 Euler angles (display only)

```
roll  = atan2(2(q.w·q.x + q.y·q.z), 1 − 2(q.x² + q.y²))
pitch = asin (clamp(2(q.w·q.y − q.z·q.x), −1, +1))
yaw   = atan2(2(q.w·q.z + q.x·q.y), 1 − 2(q.y² + q.z²))
```

The `clamp` is load-bearing: float error pushes the `asin` argument past ±1 and
you get `NaN`, which propagates into the display and looks like a controller
failure. **These are for the screen only. Never feed Euler angles back into
control** — that's the whole point of M5.

---

## 3. Architecture

```
                         ┌──────────────────────────────┐
                         │ joystick X/Y      → A0, A1   │  the circuit you wire
                         │ potentiometer     → A2       │  on the breadboard
                         │ joystick button   → D2       │
                         └───────────────┬──────────────┘
                                         │ analog in
                                         ▼
┌──────────────────────────┐   I²C @ 0x42   ┌────────────────────────┐
│  custom chip "rigid-body"│ ◄────────────► │   ATmega2560 (Mega)    │
│                          │  gyro (noisy) ►│                        │
│  RK4 @ 1 kHz             │ q_meas (noisy)►│  estimator             │
│  Euler's equations       │ q_true (truth)►│  command shaping        │
│  quaternion state        │ ◄── torque cmd │  quaternion PD          │
│  noise + fault injection │                │  100 Hz control loop    │
└──────────────────────────┘                └───────────┬────────────┘
      the chip you author                               │ I²C @ 0x3C
                                             ┌──────────▼────────────┐
                                             │ SSD1306 128×64        │
                                             │ artificial horizon    │
                                             │ ~10 Hz                │
                                             └───────────────────────┘
```

Two things you build rather than drop in: the **custom chip** (C compiled to
WASM, left) and the **analog input circuit** (right). The joystick commands
attitude, the pot trims sensitivity, and M6 turns that knob into a stability
experiment.

**`q_true` is readable but the controller must never read it.** Only the
reporting/scoring path touches it. Put that rule in a comment at the register
definition, because it is the single thing that makes this project honest — and
it's very tempting to "just peek" when debugging.

### 3.1 Register map (I²C, address `0x42`)

| Reg | Dir | Bytes | Meaning |
|---|---|---|---|
| `0x00` | R | 1 | `WHO_AM_I` → `0x51` |
| `0x10` | R | 12 | gyro x,y,z — float32, body frame, rad/s, **noisy + biased** |
| `0x20` | R | 16 | `q_meas` w,x,y,z — noisy attitude sensor |
| `0x30` | R | 16 | `q_true` w,x,y,z — **ground truth, scoring only** |
| `0x40` | R | 12 | `ω_true` x,y,z |
| `0x50` | W | 12 | torque command x,y,z — float32, N·m |
| `0x60` | W | 1 | control: bit0 reset, bit1 freeze, bit2 inject bias, bit3 inject dropout |
| `0x61` | W | 4 | gyro noise σ (float32, rad/s) |

Multi-byte reads auto-increment the register pointer. Standard pattern: master
writes the register address, then issues a repeated-start read.

> **Trap.** Read all four quaternion floats in **one** I²C transaction. Four
> separate transactions sample four different instants of a plant running at
> 1 kHz, and you get a non-unit quaternion assembled from different moments.
> This is the same "readAltitude calls readPressure again" bug from your BMP180
> sketch, in a new outfit.

### 3.2 Why I²C, and when to abandon it

I²C keeps the wiring simple and matches what you already know. But the OLED is on
the same bus, and **§6.1 is going to bite you**. If it does, move the plant to
SPI and leave the OLED on I²C. Making that call *after measuring* is a better
story than pre-empting it.

### 3.3 The analog input circuit

This is the part you wire yourself rather than author in C.

| Part | Wokwi part | Pin | Range |
|---|---|---|---|
| Joystick X | `wokwi-analog-joystick` | `A0` | 0–1023, centre ≈ 512 |
| Joystick Y | (same part) | `A1` | 0–1023, centre ≈ 512 |
| Potentiometer | `wokwi-potentiometer` | `A2` | 0–1023 |
| Joystick button | (same part, `SW`) | `D2` | active low, needs `INPUT_PULLUP` |

Both are voltage dividers into the ADC. Wire the wipers to the analog pins, ends
to `5V` and `GND`. Put the button on an interrupt-capable pin (`D2`/`D3` on the
Mega) so arming can be edge-triggered rather than polled.

**Two definitions you must keep separate.** Conflating them is the classic
beginner error in this exact project, and M6 is built around the distinction:

- **Stick scaling** — how much *commanded* attitude or rate you get per unit of
  stick deflection. Changes **what you ask for**. Cannot destabilise the loop.
- **Loop gain `Kp`** — how hard the controller works to track that command.
  Changes **how it chases**. Absolutely can destabilise the loop.

Both are "sensitivity" in casual speech. They are completely different animals.

### 3.4 What Wokwi will and will not simulate here

Be precise about this, because claiming a measured effect a simulator cannot
produce is exactly the kind of thing that discredits an otherwise good writeup.

| Effect | Simulated? |
|---|---|
| 10-bit ADC quantisation | **Yes** — `analogRead` returns integers 0–1023 |
| `analogRead` timing cost (~104 µs) | **Yes** |
| Stick centre offset, deadband behaviour | **Yes** — set the part's value |
| Sampling/latency-driven instability | **Yes** — it's your loop and your plant |
| ADC electrical **noise** | **No** — Wokwi's ADC is clean |
| Source-impedance / S-H settling error | **No** — see §6.7 |
| Thermal drift, contact wear | **No** |

So: quantisation effects are legitimate findings. Anything about analog noise has
to be either **injected deliberately in firmware** (and labelled as injected) or
deferred to real hardware. Say which in the README.

---

## 4. Milestones

### M0 — chip skeleton talks

Start from Wokwi's `wokwi-chip-template` (VS Code extension or the GitHub
template). It gives you a Makefile, `chip.json`, and a clang→wasm32 build.

`chip.json`:

```json
{
  "name": "rigid-body",
  "author": "Raahim Nawaz",
  "pins": ["GND", "VCC", "SDA", "SCL"],
  "controls": []
}
```

`chip.c` skeleton:

```c
#include "wokwi-api.h"

typedef struct {
  float q[4], w[3], tau[3];   // state
  float J[3];                 // inertia
  uint8_t reg;                // I2C register pointer
} chip_state_t;

// TODO: i2c_init() with connect/read/write/disconnect handlers
// TODO: timer_init() + timer_start() at 1000 us, repeating
void chip_init(void) { /* ... */ }
```

Check the template's headers for exact struct/callback signatures — treat the
template as the source of truth, not this document.

**Verification gate:** MCU reads `0x00` and prints `0x51`. Nothing else works
yet, and that's fine.

---

### M1 — the plant is trustworthy

Implement RK4 on `ω`, exponential-map update on `q`. No control, no noise.
Torque fixed at zero. Set an initial `ω` and let it run free.

**Three verification gates, all of which must pass:**

1. **Norm holds.** Log `‖q‖ − 1` for 60 s. Should stay below ~1e-6.
2. **Energy conserves.** With `τ = 0`, rotational kinetic energy
   `E = ½(Jx·ωx² + Jy·ωy² + Jz·ωz²)` is constant. Log it. Drift means your
   integrator is wrong.
3. **The Dzhanibekov test.** Set `J = diag(1, 2, 3)` and spin about the
   *intermediate* axis (`ω = [0, 5, 0]`) with a tiny perturbation
   (`ωx = 0.01`). **It must periodically flip end over end.** This is the
   tennis-racket theorem, and it is an extremely sharp test: it only emerges if
   your cross-coupling terms are right. Three independent integrators cannot
   produce it.

Gate 3 is the one that proves you wrote physics rather than plausible-looking
arithmetic. Do not skip it.

---

### M2 — quaternion library on the MCU, with tests

Write `quat.h` / `quat.cpp`:

```cpp
struct Quat { float w, x, y, z; };

Quat  quat_mul(const Quat&, const Quat&);
Quat  quat_conj(const Quat&);
void  quat_normalize(Quat&);
float quat_norm(const Quat&);
Quat  quat_from_axis_angle(float ax, float ay, float az, float rad);
void  quat_to_euler(const Quat&, float& roll, float& pitch, float& yaw);
```

Then a `selftest()` that runs at boot and prints `PASS`/`FAIL` per case. **This
becomes your CI assertion surface in M8**, so make the output machine-readable.

Cases worth having:

- `q ⊗ q* == identity` for a few random `q`
- 90° about X, then 90° about Y — compare against the hand-computed result
- two 180° rotations compose to identity
- `quat_from_axis_angle(0,0,1, π/2)` then `quat_to_euler` gives yaw = 90°
- gimbal-lock guard: pitch at exactly 90° returns finite numbers, not `NaN`

**Verification gate:** `ALL TESTS PASS` on the serial monitor.

---

### M3 — closed loop, perfect sensing

Read `q_meas` with noise σ set to 0. Implement §2.5. Command a 90° step about
one axis.

Log CSV over serial — you'll want to plot this:

```
t_ms,qw,qx,qy,qz,err_deg,wx,wy,wz,tx,ty,tz,loop_us
```

**Verification gate:** error converges monotonically to < 1°. Compare rise time
and overshoot against what §2.6 predicted. Write down both numbers.

---

### M4 — the unwinding demo (the centrepiece)

Delete `sign(q_err.w)`. Command a **179°** step, then a **181°** step. Log both.

**What you should see:** 179° goes the short way. 181° travels ~179° *in the
opposite direction* — arriving at the same place, having taken the long path.

Now restore the sign term and re-run. Both take the short path.

**This is your headline plot.** Two traces, error-vs-time, naive and fixed. It's
a real bug from spacecraft attitude control, it's four characters, and almost
nobody in a junior portfolio can explain it. Keep both firmware paths behind a
`#define UNWINDING_DEMO` so the comparison is reproducible rather than a story
about something you once saw.

---

### M5 — OLED artificial horizon, and gimbal lock on screen

Layout for 128×64, `textSize(1)` (21 chars/line, 8 px/row):

```
ROLL -12.4  PITCH   8.1     <- row 0
┌─────────────────────────┐
│         ────\           │  <- horizon band, rows 1..5
└─────────────────────────┘
q .981 -.108 .071 .000      <- row 6
err 14.2deg  loop  412us    <- row 7
```

Horizon geometry — the robust parametric form (avoids `tan` blowing up):

```
cx = 64, cy = 28                 // band centre
PPD = 0.6                        // px per degree of pitch
hx = cx + PPD*pitch_deg * sin(roll)
hy = cy + PPD*pitch_deg * cos(roll)
dx = cos(roll),  dy = -sin(roll) // unit vector along the horizon
drawLine(hx - 200*dx, hy - 200*dy, hx + 200*dx, hy + 200*dy)
```

Adafruit_GFX clips off-screen endpoints for you, so the ±200 overshoot is fine.

**Skip the sky/ground fill.** A per-pixel inside/outside test is 8192 float
evaluations, which an ATmega2560 cannot afford at any useful rate. Draw the line
plus a short pitch ladder. If you want the fill, do it per-column in fixed point
and accept that it degrades near roll = ±90° — and then *say so* in the README
rather than hiding it.

**The gimbal-lock demo:** command a slow pitch sweep through 90°. Watch roll and
yaw go wild on screen while `err_deg` — computed from the quaternion — stays
smooth and small. **The display breaks and the controller doesn't.** Capture
that as a GIF; it's the most legible argument for quaternions you can make in
five seconds.

---

### M6 — manual control, and the knob that decides stability

Now the loop has a human in it. This is the milestone that makes the project
*demoable* — everything before is a step response, this is something you fly.

**Command shaping.** Read the sticks, turn them into a target. Two modes, exactly
as real flight controllers have:

```
ANGLE mode:  stick → commanded tilt angle   → q_cmd = quat_from_axis_angle(...)
RATE  mode:  stick → commanded body rate    → integrate q_cmd forward each loop
```

Toggle with the joystick button. Angle mode is self-levelling and forgiving;
rate mode holds whatever attitude you leave it in and is much harder to fly.
Feeling that difference is the point.

**The pipeline**, in order:

```
raw = analogRead(A0)                    // 0..1023
x   = (raw - centre) / 512.0            // → roughly -1..+1, centre from boot calibration
if |x| < deadband:  x = 0               // deadband AFTER centring, not before
x   = clamp(x, -1, +1)
x   = expo*x*x*x + (1-expo)*x           // fine near centre, full authority at the stops
cmd = x * stick_scale                   // ← "sensitivity"
```

The expo curve is `out = e·x³ + (1−e)·x`. At `e = 0` it's linear; at `e = 1` it's
pure cubic. It's what makes a stick feel precise near centre without giving up
range, and it costs two multiplies.

**Boot calibration.** Sample the sticks for ~500 ms at startup and store the
centre. Do not assume 512 — nothing centres at exactly 512. Then refuse to arm
if the sticks aren't near centre, which is a real failsafe pattern: it stops the
body snapping to a full-deflection command the instant you arm.

**The experiment — one pot, two meanings.** Build both behind a `#define`:

1. **Pot → `stick_scale`.** Turning it changes how much attitude you command per
   unit of stick. The loop's *stability is unchanged* — only the size of the
   demand changes. Step responses at different pot settings should be
   geometrically similar: same shape, different amplitude.
2. **Pot → `Kp`.** Now you're changing loop gain. Sweep the knob slowly and
   log a step response at each setting. You should see the response walk through
   **sluggish → crisp → overshooting → ringing → divergent.**

**Why divergent, when the maths says it can't be.** A continuous-time PD
controller on a rigid body is stable for *any* positive `Kp`, `Kd`. Your loop is
not continuous. It samples at 100 Hz, and it has delay: the ADC reads, the I²C
round trip, the control computation, and — decisively — the OLED refresh from
§6.1. That delay eats phase margin, and raising `Kp` pushes the crossover
frequency `ωc` up until the fixed delay is worth 180° of phase.

Predict it before you measure it:

```
delay margin ≈ PM / ωc              (PM in radians)
```

Estimate your total loop delay `Td`, find the `ωc` at which `Td·ωc` consumes the
phase margin, then back out the critical `Kp`. Compare against the knob position
where it actually went unstable. **Being wrong here is fine and interesting —
being wrong by 3× tells you which delay term you underestimated.**

**And here is the payoff that ties the whole project together:** run the sweep
twice, once with `Wire.setClock(100000)` and once at `400000`. The critical gain
*moves*. The knob you can safely turn up depends on how fast your display bus is.
That single sentence connects a UI control, classical control theory, and I²C bus
timing — and you measured it.

**Add to the OLED** (row 0 or 7): current mode, pot value as a percentage, and
armed state. You need to see what you're commanding.

**Verification gates:**

- centre calibration holds — sticks released, body does not drift
- angle mode self-levels from a disturbance; rate mode does not
- stick_scale sweep: step responses scale in amplitude, **not** in shape
- `Kp` sweep: you can locate the instability boundary and state it as a number
- the boundary moves between 100 kHz and 400 kHz I²C

---

### M7 — take the truth away

Set `q_meas` noise σ > 0, or stop reading `q_meas` entirely and integrate the
gyro yourself.

1. **Gyro-only.** Integrate `q̇` from the noisy, biased gyro. Watch attitude
   drift. Plot true-vs-estimated error growing without bound. *This is the
   honest failure you want on record.*
2. **Complementary filter.** Blend the fast-but-drifting gyro integration with
   the slow-but-unbiased `q_meas`:
   ```
   q_est = slerp(q_gyro, q_meas, alpha)      // alpha ~ 0.01–0.05
   ```
   Plot the error again. It should bound.
3. **Estimate the bias.** Feed the correction back as a slowly-integrated gyro
   bias term and show the bias estimate converging to the value you injected —
   which you know, because you injected it.

Point 3 is the strongest single result in the project: **you recovered a hidden
parameter and can prove it, because you own the truth.**

---

### M8 — the timing budget

You already know how to talk about this from the `isolcpus`/`SCHED_FIFO` work.
Numbers to produce:

- control loop period: mean, worst case, jitter (log `loop_us` every cycle, keep a running max)
- cost breakdown: ADC reads (3 × ~104 µs), I²C read, quaternion math, control law, I²C write
- what a full OLED refresh costs, and what it does to worst-case jitter
- flash and RAM used (`avr-size`), and headroom remaining

**Expected finding (see §6.1):** the OLED dominates everything.

---

### M9 — CI

`wokwi.toml` at the repo root:

```toml
[wokwi]
version = 1
firmware = "build/firmware.hex"
elf = "build/firmware.elf"
```

`test/selftest.test.yaml`:

```yaml
name: firmware selftest
version: 1
author: Raahim Nawaz
steps:
  - wait-serial: "ALL TESTS PASS"
  - wait-serial: "STEP90 SETTLED"
```

GitHub Actions step:

```yaml
- uses: wokwi/wokwi-ci-action@v1
  with:
    token: ${{ secrets.WOKWI_CLI_TOKEN }}
    path: /
    timeout: 60000
    scenario: test/selftest.test.yaml
```

**Design the asserts into the firmware, not the scenario.** Have the sketch run
its own checks and print one machine-readable line — `STEP90 SETTLED err=0.83
rise=412ms`. The scenario just waits for it. That keeps you independent of which
scenario step types the CLI supports, and it means the same self-test runs on
real hardware later.

Get a free `WOKWI_CLI_TOKEN` from your Wokwi account and add it as a repo secret.

---

## 5. Build order summary

| M | Deliverable | Gate |
|---|---|---|
| 0 | chip skeleton | `WHO_AM_I` reads `0x51` |
| 1 | plant | norm holds, energy conserves, **Dzhanibekov flips** |
| 2 | quat library | `ALL TESTS PASS` |
| 3 | closed loop | 90° step converges, matches predicted `ωn`/`ζ` |
| 4 | **unwinding demo** | 181° goes the wrong way, then doesn't |
| 5 | OLED horizon | gimbal lock visible on screen, controller unaffected |
| 6 | **joystick + pot** | stability boundary located, and it moves with I²C speed |
| 7 | estimator | injected gyro bias recovered |
| 8 | timing budget | jitter + flash/RAM numbers |
| 9 | CI | green badge running actual firmware |

**M0–M4 is already a complete, defensible project — ship there if you stall**
rather than leaving nine milestones half-done.

**M6 is where it becomes demoable.** A recruiter can fly it in a browser, and the
sensitivity sweep is the result that's hardest to get any other way. If you have
the appetite for one more milestone past M4, make it this one — take it before
the estimator.

---

## 6. Traps, with symptoms

### 6.1 The OLED will eat your control loop

The SSD1306 framebuffer is 1024 bytes. Over I²C, each byte is ~9 bits with ACK:

- at **100 kHz** (Arduino default): `1024 × 9 / 100000` ≈ **92 ms per refresh**
- at **400 kHz**: ≈ **23 ms**

Your control loop wants a 10 ms period. **A single blocking `display.display()`
blows six to nine control cycles.**

Fixes, in order of preference:
1. `Wire.setClock(400000)` — free 4×.
2. Refresh at 10 Hz, not every loop.
3. Redraw in horizontal slices across several loop iterations.
4. Move the plant to SPI so at least the two devices aren't contending.

**Do not skip straight to the fix.** Measure the jitter first, then fix it, then
measure again. The before/after is the deliverable — a graph of worst-case loop
time at 100 kHz vs 400 kHz vs sliced refresh is exactly the kind of thing that
gets you asked a follow-up question in an interview.

### 6.2 `float` on an AVR has no hardware behind it

ATmega2560 is 16 MHz with no FPU. A `float` multiply is ~5–10 µs; `sin`/`cos`
are far worse (tens of µs). One exponential-map update needs a `sin`, a `cos` and
a `sqrt`.

Budget it before you optimise. If the loop won't close, options are: small-angle
approximation when `‖ω‖·Δt` is tiny (you need that branch anyway to avoid
dividing by zero), lookup tables, or fixed-point. **Measure first.**

### 6.3 Reading a quaternion in four transactions

Covered in §3.1. Symptom: `‖q‖` occasionally reads 1.03 for no reason, more often
when spinning fast. One transaction, always.

### 6.4 Convention drift between chip and MCU

Symptom: the controller works for rotations about X but drives the wrong way
about Y and Z, or the plant rotates correctly but the horizon tilts backwards.

Cause: `q̇ = ½q⊗[0,ω]` on one side and `½[0,ω]⊗q` on the other, or scalar-first
in the chip and scalar-last on the MCU.

**Cheapest defence:** a wire-format test. Chip exposes a fixed known quaternion
(90° about Z) at a debug register. MCU reads it, converts to Euler, asserts
yaw = 90°. Add it to `selftest()` at M2 and it can never silently break.

### 6.5 `asin` domain error

`quat_to_euler` returns `NaN` at pitch ≈ ±90° without the clamp in §2.7. `NaN`
propagates through the display and every subsequent comparison silently returns
false. Clamp at the source.

### 6.6 Peeking at `q_true`

The temptation at M7, when the estimator drifts, is to "temporarily" feed
`q_true` to the controller to check something else. Do it and you will forget.
Guard it: read `q_true` only inside a `report()` function and never pass it into
`control()`.

### 6.7 ADC source impedance — real, but **not** simulated

The ATmega2560's sample-and-hold wants a source impedance below ~10 kΩ. A 100 kΩ
pot violates that: the S-H capacitor doesn't fully charge in the sample window, so
a reading is contaminated by whichever channel was read *before* it — mux
crosstalk that looks like the joystick and pot are haunted by each other.

Fixes on real hardware: use a 10 kΩ pot, add an op-amp buffer, or read the channel
twice and discard the first.

**Wokwi will not reproduce this.** Its ADC is ideal. Do not claim you measured it.
It belongs in "what this does not show", or in a follow-up on the physical board —
where it would be a genuinely good finding, because it's in the datasheet and
almost nobody hits it deliberately.

### 6.8 Deadband before centring

Applying the deadband to the raw `analogRead` value instead of the centred value
gives you an asymmetric dead zone that moves with the stick's centre offset.
Symptom: the body drifts one way when the stick is released, and the drift
direction flips if you recalibrate. Centre first, deadband second — the order in
§M6's pipeline is deliberate.

### 6.9 Changing `Kp` mid-flight is a step input

Turning the pot while the loop is running changes the gain discontinuously, which
kicks the torque command. Slew-limit the gain, or only latch a new value when the
sticks are centred. Otherwise your instability sweep is measuring the transient
from the knob, not the steady-state stability boundary — and you'll report a
critical gain that's too low.

---

## 7. What to write up

The repo README is the actual deliverable. Structure it around findings, not
features:

1. **The unwinding plot.** Two traces, naive vs `sign(q_err.w)`. Lead with this.
2. **Gimbal-lock GIF.** Euler display breaking while quaternion error stays smooth.
3. **The sensitivity sweep.** Step response vs pot position, for both meanings of
   "sensitivity" — stick scaling changes amplitude only, `Kp` changes shape and
   eventually diverges. Then the kicker: **the critical gain moves when you change
   the I²C clock**, because the display bus is spending your phase margin.
   Predicted critical `Kp` next to measured.
4. **The recovered bias.** Injected value vs estimated, converging.
5. **Timing budget table.** Loop period, jitter, ADC and OLED cost at 100 vs
   400 kHz, flash/RAM used and remaining.
6. **Integrator validation.** Norm drift and energy conservation over 60 s;
   exponential map vs Euler-and-renormalize.
7. **What this does not show.** No physical hardware, no real sensor
   characteristics, no ADC noise or source-impedance effects (§3.4, §6.7), no
   thermal or power behaviour. Say it plainly and point at the repos where you
   *do* have that. The honesty is the differentiator — don't drop it here.

Title it something like *"Quaternion attitude control, and the four characters
that decide which way you rotate."* Not "environmental monitor."

---

## 8. Stretch, only after M8

- **SLERP trajectory following** between commanded orientations rather than steps.
- **Reaction-wheel model** — replace direct torque with wheel speeds, momentum
  storage, and saturation. Introduces wheel desaturation, a genuinely interesting
  real problem.
- **Two servos as a gimbal** in the Wokwi diagram, physically pointing to
  counteract attitude. Legible visual proof in a still screenshot.
- **Fixed-point rewrite** of the hot path, with a before/after timing table.
