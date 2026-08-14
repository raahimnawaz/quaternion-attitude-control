# Quaternion Attitude Control (`quat_project`)

## Overview
This repository contains the firmware, schematics, and validation data for a quaternion-based attitude estimation and control system. Built primarily in C/C++ for embedded systems, it leverages sensor fusion, hardware-level algorithm optimizations, and includes a custom PCB design.

## Key Features
* **Sensor Fusion:** Implements a Mahony filter for accurate attitude estimation.
* **Hardware Optimization:** Utilizes the Quake III fast inverse square root algorithm to maximize performance on resource-constrained hardware.
* **Custom Hardware:** Includes a complete KiCad schematic (`quat_project_pcb_v1_sch.kicad_sch`) for custom PCB deployment.
* **Physical Control Interface:** Integrates potentiometer and joystick inputs for real-time control.
* **Controller Optimization & Validation:** Compares naive vs. optimized controller performance, backed by CSV datasets and SVG visualizations.

## Repository Structure
* **`sketch.ino`**: Main application entry point for the microcontroller.
* **`chip/`**: Contains the core Mahony filter implementation and physical control logic.
* **`rigid-body.chip.c` / `rigid-body.chip.json`**: Rigid body logic and diagram configurations.
* **`quat_project_pcb_v1_sch.kicad_sch`**: KiCad schematic file for the custom printed circuit board.
* **Data & Charts (`*.csv`, `*.svg`)**: Performance validation datasets (naive vs. optimized) and graphical plots tracking Error (degrees) over time and Quaternions over time.
* **`quaternion-attitude-blueprint.md`**: Detailed build blueprint and architectural documentation.

## Tech Stack
* **Languages:** C, C++
* **Tools:** KiCad
* **Domains:** Aerospace, Embedded Systems, Firmware, Sensor Fusion

## Getting Started
1. **Firmware:** Open `sketch.ino` in your preferred IDE (e.g., Arduino IDE, PlatformIO) and flash it to your target microcontroller.
2. **Hardware:** Open `quat_project_pcb_v1_sch.kicad_sch` using KiCad to view or modify the custom PCB schematic.
3. **Documentation:** Refer to `quaternion-attitude-blueprint.md` for detailed build instructions and system architecture.

---
*Developed by [Raahim Nawaz](https://github.com/raahimnawaz)*
