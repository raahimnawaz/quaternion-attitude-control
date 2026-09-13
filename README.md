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
  │ OLED horizon @ 30 Hz       │               └──────────────────────────────┘
  └────────────────────────────┘
     joystick → attitude command
     pot      → gain multiplier
```

The plant integrates angular velocity with RK4 and attitude with the exponential
map, which is norm-preserving by construction. The MCU runs a Mahony
complementary filter to fuse the accelerometer's gravity reference against gyro
integration, then applies

```
q_err = q_est⁻¹ ⊗ q_cmd
τ     = Kp · sign(q_err.w) · q_err.vec  −  Kd · ω
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

Both runs start from the same tumble: `ω = [0.01, 5.0, 0.0]` rad/s with the
commanded attitude just under 180° away, which is precisely the case that
separates a controller that unwinds from one that does not.

| | Naive | With `sign(q_err.w)` |
|---|---|---|
| Initial error | 180.91° | 179.09° |
| Final error | 1.24° | 0.00° |
| Time to <5° | ~2148 ms | ~2153 ms |
| Mean loop time | 2000 µs (500 Hz) | 2041 µs (490 Hz) |

**Read that table carefully, because it does not say what it looks like it says.**

The two initial errors are the same physical attitude: 360 − 180.91 = 179.09. The
naive controller measures its error the long way around and commits to rotating
through it; the corrected one takes the short path. That difference is real and
it is visible in the trajectory plots.

What the data does **not** show is the corrected controller settling faster. Both
reach <5° within about 2.15 s, and the corrected one runs marginally slower per
loop. The honest summary is that on this run the unwinding fix improved the
*path* and the *final tracking error*, not the settling time.

That comparison is also weaker than it should be: the naive run ends at 2736 ms,
barely after its own settling point, while the corrected run continues to
5727 ms. The runs are not length-matched, so the naive settling figure rests on
almost its last sample. Regenerating both over a common window is outstanding
work, not a completed result.

![Attitude error over time](results/error-vs-time.svg)

![Quaternion components over time](results/quaternion-vs-time.svg)

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

### `invSqrt` only wins on one of the three targets

The Quake III reciprocal square root is worth having *or* worth deleting
depending entirely on whether the part has an FPU:

| target | FPU | verdict |
|---|---|---|
| **Arduino Mega** (ATmega2560) — the Wokwi prototype | none | **wins.** `sqrtf` is a software routine and every float op is a libgcc call, so trading a root and a divide for a shift, a subtract and three multiplies is a real saving |
| **ESP32** (Xtensa LX6) | yes | **loses.** `1.0f/sqrtf(x)` is a couple of instructions and correctly rounded; the trick also pays to move between integer and float register files |
| **STM32WBA55** (Cortex-M33F) — the custom board | yes | **loses**, same reason |

So the optimisation is right for the board it was prototyped on and wrong for
the board it is headed to. `sketch.ino` now selects on `__AVR__` rather than
assuming. The AVR path also swaps `*(long*)&y` for `memcpy` — the pointer cast
is a strict-aliasing violation, undefined behaviour that `-O2` is entitled to
miscompile, and every compiler lowers the `memcpy` to the same register move.

**This is still unmeasured.** The number it needs is a `micros()` loop around
~10,000 calls of each variant on each board; until that exists the table above
is an architectural argument, not a result.

---

## The board

![Custom attitude-control board, v1 schematic](docs/schematic.png)

An STM32WBA55HEFx (Cortex-M33 + FPU, integrated radio) on 3V3 from an AMS1117
LDO, with the joystick on `PA0`/`PA1`, the gain potentiometer on `PA2`, SWD on
J1, and 4.7 kΩ I²C pull-ups to an off-board IMU on J3.

**v1 is a schematic, and it should not be fabricated as drawn.** The whole
design is 11 components. Tracing the netlist out of the `.kicad_sch` rather than
eyeballing the sheet, in severity order:

1. **`VDD11` is tied to +3V3.** Every supply pin — `VDD`, `VDDA`, `VDDANA`,
   `VDDHPA`, `VDDRF`, `VDDRFPA`, `VDDSMPS` **and `VDD11`** — sits on one net.
   `VDD11` is the 1.1 V core/radio domain, not a supply input. Driving 3.3 V
   into it is a part-killer, and it is consistent with the next item: the SMPS
   that is supposed to *produce* that rail has been no-connected instead.
2. **`VLXSMPS` is no-connect.** The internal SMPS needs its inductor between
   `VLXSMPS` and `VDD11`, or the part must be strapped for LDO mode per the
   datasheet's power-supply scheme. As drawn it is neither, which is how 1
   happened. **Fix these two together — read the datasheet's supply table and
   redraw the power section from it.**
3. **The package is `ST_WLCSP-41_2.98x2.76mm_P0.4mm_Stagger`.** A 41-ball
   wafer-level chip-scale part on 0.4 mm staggered pitch: bare die, no leads.
   That needs HDI with via-in-pad and laser microvias, a stencil and reflow, and
   it cannot be hand-soldered, reworked, or probed. It is also light-sensitive.
   The QFN version of the same silicon is routable on two layers and solderable
   with an iron. Unless there is a size constraint that justifies it, **this one
   choice is what makes the board unbuildable at hobby scale.**
4. **Decoupling is one capacitor.** `C1` (0.1 µF) and `C2` (10 µF) are the only
   caps on the sheet, both at the regulator. Eight supply pins want ~100 nF each
   placed at the ball, plus bulk. On a normal package this is the single most
   common reason a first article is dead on arrival.
5. **No crystal.** `OSC_IN`/`OSC_OUT` are no-connect, so this runs on the
   internal RC — which rules out the radio that is the reason to choose a WBA55.
   Either add the 32 MHz crystal or drop to a cheaper, non-wireless part.
6. **`NRST` floats.** It wants the usual 100 nF to ground.
7. **LDO output cap is light.** The AMS1117 wants ≥10 µF (the datasheet suggests
   22 µF tantalum) to stay stable; 0.1 µF alone is marginal.
8. **The project's root sheet is empty.** `hardware/quat_project.kicad_sch` is a
   blank A4 page, and the real design lives in
   `quat_project_pcb_v1_sch.kicad_sch`, which nothing references — so opening the
   project shows nothing. Every symbol's instance path is also still bound to
   `(project "mini inverter")`, and the title block says the same. The design was
   started by copying another project and never rebound.

**There is no layout.** `hardware/quat_project.kicad_pcb` is an empty board file
— no stackup, no footprints, no traces. Footprints *are* assigned to all 11
symbols, so the netlist would import; nothing has been placed or routed.

➡️ **[`hardware/power-section.md`](hardware/power-section.md)** is the corrected
wiring to redraw from — both supply modes with the exact pin connections and
values, the package change, and what to check before ordering.

## Known limitations

The simulation is honest about what it does not model. These are the places where
these results would not survive contact with hardware:

- **The gyro has no bias.** The plant adds zero-mean noise only. Rejecting slow
  gyro bias is the main reason a Mahony filter exists, and this sim never asks it
  to. The `0x60` control register that would inject bias is specified in the
  blueprint and not implemented.
- **The accelerometer measures gravity and nothing else.** No linear
  acceleration is modelled, so the classic failure mode — a manoeuvring vehicle
  whose accelerometer stops pointing at the ground — is never exercised. This is
  the single largest gap between this sim and a real vehicle.
- **Noise is uniform, not Gaussian, and `rand()` is never seeded.** Every run is
  bit-identical. Good for reproducibility, useless for characterising variance.
- **The MCU Euler-integrates `q` and renormalises.** The plant uses the
  exponential map; the MCU does not, which contradicts §2.4 of the blueprint. It
  is standard Mahony formulation, but the discrepancy is unmeasured.
- **`dt` is unclamped.** A stalled loop feeds an arbitrarily large step into the
  integrator.
- **Results predate the current plant.** The CSVs were recorded when the chip
  started at a 5 rad/s tumble; the current chip starts at rest. They are not
  reproducible from a clean checkout without restoring that initial condition.
- **Wokwi does not simulate ADC source impedance**, so the analog input circuit
  is validated only in principle.
- **No CI.** M9 is unstarted; nothing verifies the quaternion library on push.

---

## Running it

Open the project in [Wokwi](https://wokwi.com) — `diagram.json` and `sketch.ino`
must stay in the repository root, which is why the firmware is not tucked into a
subdirectory. The custom plant builds from `chip/chip.chip.c`.

Telemetry streams over serial at 115200 baud, 8 columns at 30 Hz:

```
q_est.w, q_est.x, q_est.y, q_est.z, q_true.w, q_true.x, q_true.y, q_true.z
```

Attitude error in degrees is `2·acos(|⟨q_est, q_true⟩|)`, computed downstream so
the metric can change without reflashing.

> **`q_true` is for scoring only.** It is read inside the telemetry block, after
> the torque command has gone out, into a local that leaves scope immediately —
> so the control path has no name bound to it. The estimator does not get to see
> the answer.

---

## Layout

```
sketch.ino        firmware: fusion, control law, OLED, telemetry
diagram.json      Wokwi wiring
libraries.txt     Wokwi library manifest
platformio.ini    local AVR build: `pio run -e megaatmega2560`
chip/             custom plant: rigid-body dynamics + simulated IMU
docs/blueprint.md the full design document — maths, register map, milestones
docs/schematic.png v1 schematic as drawn — see "The board" before fabricating
hardware/         KiCad project, plus power-section.md: the corrected wiring to redraw v1 from
results/          recorded runs and plots
```

`docs/blueprint.md` is the primary reference: it derives the maths, fixes the
conventions, specifies the I²C register map, and catalogues the traps this
project hit on the way through.
