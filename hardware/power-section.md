# Corrected power section — STM32WBA55

The v1 sheet ties **every** supply pin, including `VDD11`, to `+3V3`, and leaves
`VLXSMPS` no-connect. That is wrong in both of the part's supported supply
modes. This is what to redraw it as.

**Read this as a wiring skeleton, not as a substitute for the reference
design.** The authority is ST's own schematics:

- **[AN5948 — How to develop RF hardware using STM32WBA MCUs](https://www.st.com/resource/en/application_note/an5948-how-to-develop-rf-hardware-using-stm32wba-mcus-stmicroelectronics.pdf)**
  — reference schematics per package, including the WBA55CG, plus the VDDA
  decoupling and LC-filter figures.
- **[AN6373 — Guidelines for enabling the embedded SMPS on STM32WBA MCUs](https://www.st.com/resource/en/application_note/an6373-guidelines-for-enabling-embedded-smps-on-stm32wba-mcus-stmicroelectronics.pdf)**
  — only needed if you take Option B.
- **[STM32WBA5xxx datasheet](https://www.st.com/resource/en/datasheet/stm32wba52ce.pdf)**
  — the power supply scheme figure is the final word.

The single highest-value move is to **open the NUCLEO-WBA55CG (MB1863)
schematic and copy its power section verbatim.** It is a known-good, ST-verified
layout of exactly this problem. Deriving it by hand is how v1 got here.

---

## First: pick the package

Before any of this matters, get off `ST_WLCSP-41_2.98x2.76mm_P0.4mm_Stagger`.
A 41-ball WLCSP on 0.4 mm staggered pitch needs HDI with via-in-pad and laser
microvias, cannot be hand-soldered or reworked, and is light-sensitive.

**Use `STM32WBA55CG` in UFQFPN48** (7×7 mm, 0.5 mm pitch). Same silicon, routes
on two layers, solders with an iron and a hot plate. Everything below assumes
that part.

---

## Option A — LDO mode (recommended for this board)

No inductor, no SMPS layout constraints, ~10 fewer things to get wrong. The only
cost is idle current, which does not matter on a bench project running off USB.

```
+5V ──┬── C2 10µF ──┬── GND
      │             │
      └── U1 AMS1117-3.3 ────┬── C3 22µF ──┬── C1 100nF ──┬── +3V3
              ADJ/GND ───────┘             │              │
                                          GND            GND

+3V3 ──┬── VDD        + 100nF to GND, at the pin
       ├── VDDA       + 100nF ∥ 1µF to GND, at the pin   (see AN5948 Fig. for the LC option)
       ├── VDDANA     + 100nF to GND, at the pin
       ├── VDDRF      + 100nF to GND, at the pin
       ├── VDDRFPA    + 100nF to GND, at the pin   ← VDD-fed gives up to +10 dBm
       └── VDDHPA     + 100nF to GND, at the pin

VDD11    ── DECOUPLING ONLY: 2 × 2.2µF ∥ 2 × 100nF to VSS.  NEVER to +3V3.
VDDSMPS  ── VSS
VLXSMPS  ── VSS
VSSSMPS  ── VSS
VSS, VSSA, VSSRF ── GND
```

**The three lines that change from v1:** `VDD11` comes off `+3V3` and gets its
own capacitors to ground; `VDDSMPS` moves from `+3V3` to `VSS`; `VLXSMPS` stops
being a no-connect and goes to `VSS`.

## Option B — SMPS mode

Only if you actually need the current saving. Adds a switching node, which wants
a tight loop and a proper ground return — a real layout constraint.

```
VDDSMPS  ── +3V3
VLXSMPS  ── L1 2.2µH ceramic ── VDD11
VDD11    ── 2 × 2.2µF to VSSSMPS   (keep this loop short)
VSSSMPS  ── GND
```

Note the trade: SMPS-fed, the internal PA reaches ~+6 dBm; VDD-fed (Option A's
`VDDRFPA → +3V3`) reaches up to +10 dBm.

---

## The rest of the fix list

| | Fix |
|---|---|
| `NRST` floating | 100 nF to GND |
| No crystal | 32 MHz crystal + load caps on `OSC_IN`/`OSC_OUT` — **required for the radio.** If you are not using BLE, drop to a non-wireless part and save the money |
| LDO output cap 0.1 µF | AMS1117 wants ≥10 µF; 22 µF is the datasheet suggestion. `C3` above |
| Root sheet blank | In KiCad: open `quat_project_pcb_v1_sch.kicad_sch`, *Save As* over `quat_project.kicad_sch`, re-annotate. Fixes the `"mini inverter"` project binding and the title block at the same time |

## Sanity checks before ordering

1. Run **ERC** with the power pins un-suppressed — v1's 24 no-connect markers
   will hide real errors otherwise.
2. Confirm no net named `VDD11` touches `+3V3`. That is the part-killer.
3. Diff your power section against the NUCLEO-WBA55CG schematic pin by pin.
4. Check every supply pin has a 100 nF within a few mm of it *in the layout*,
   not just on the schematic — decoupling that is correct on the sheet and 15 mm
   away on the board is not decoupling.
