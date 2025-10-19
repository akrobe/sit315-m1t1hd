# SIT315 – Module 1: Real-Time & Embedded Systems  
**Comprehensive Interrupt-Based Sense–Think–Act System (QP1 → QP4 + HD “QP5”)**

This repository contains my progressive builds for the SIT315 Task M1QP. The project evolves from a simple polling loop (QP1) to a fully interrupt-driven, configurable, and supervised system (QP4 + HD extras). Final demo firmware: **QP5_HD_SlowShow**.

---

## Contents

- [Hardware & Pin Map](#hardware--pin-map)  
- [Sketches & What They Demonstrate](#sketches--what-they-demonstrate)  
- [How to Build / Upload](#how-to-build--upload)  
- [Run the Final Demo (QP5)](#run-the-final-demo-qp5)  
- [Serial Menu Cheat Sheet](#serial-menu-cheat-sheet)  
- [Tinkercad Links & Diagrams](#tinkercad-links--diagrams)  
- [Screenshots (for assessment)](#screenshots-for-assessment)  
- [Known Quirks / Troubleshooting](#known-quirks--troubleshooting)  
- [Submission Checklist](#submission-checklist)  
- [Academic Integrity](#academic-integrity)

---

## Hardware & Pin Map

**Board:** Arduino Uno R3 (A000066)

**Sensors / Inputs**
- **A – Arm button**: D2 (INT0, `attachInterrupt`, `INPUT_PULLUP`, momentary to GND)  
- **B – Input 2**: D8 (PCINT0 / PB0, `INPUT_PULLUP`, slide/momentary to GND)  
- **C – Input 3**: D9 (PCINT1 / PB1, `INPUT_PULLUP`, slide/momentary to GND)  
- **Analog**: A0 (capacitive soil sensor or potentiometer center pin)

**Actuators / Outputs**
- **Action LED**: D12 → 220 Ω → LED anode; LED cathode → GND  
- **Heartbeat LED**: D13 (built-in “L” LED)

**Power Rails**
- 5 V → breadboard + rail; GND → breadboard – rail

| Signal | Uno Pin | Notes |
|---|---:|---|
| Arm (A) | D2 | External Interrupt 0 (INT0) |
| B | D8 | **Pin Change Interrupt group** (Port B) |
| C | D9 | **Pin Change Interrupt group** (Port B) |
| LED_ACT | D12 | External LED via 220 Ω |
| LED_HB | D13 | Built-in LED |
| Analog | A0 | Soil sensor/pot center |

> **PCI group note:** **D8 & D9 share the same PCI group (Port B)** which is required by the task (monitor ≥2 pins from same group).

---

## Sketches & What They Demonstrate

Located under `sketches/…`

### QP1_Baseline
- **Polling only** (no interrupts).  
- 1 digital sensor (button on D2, `INPUT_PULLUP`) → LED on D12.  
- Serial prints “pressed/released” + 1 s status line.  
- **Meets QP1**: single sensor + actuator, Sense–Think–Act, Serial output.

### QP2_EXTINT
- ≥2 sensors (A on D2 + B on D8 used).  
- **External interrupt** on D2 with `attachInterrupt` toggles `armed`.  
- Conditional logic (LED acts only when `armed`).  
- **Meets QP2**: `attachInterrupt`, modular logic, non-blocking flow.

### QP3_PCI
- **Pin Change Interrupts** on D8 & D9 (same PCI group).  
- Debounce in main loop; ISR only sets flags/snapshots.  
- **Grouped logic**: Mode 0 (OR) / Mode 1 (AND) toggled by **B→C** or **C→B** combo within a window.  
- Traceable Serial: PCI edges, mode changes, status.  
- **Meets QP3**: multi-sensor, PCI, state management, grouped logic.

### QP4_PCI_Timer
- **PCI + Timer2** periodic tasks → heartbeat (D13) and analog A0 sampling.  
- Clear separation of **event-based** (INT0/PCI) vs **time-based** (Timer2).  
- **Meets QP4**: 3 sensors (A, B, C) + periodic task, interrupt-safe, modular.

### QP5_HD_SlowShow (Final Demo Build)
- QP4 features plus **HD extras**:
  - **EEPROM-backed config** with CRC (threshold, combo window, default mode/armed).
  - **Watchdog (WDT)** + **reset cause logging** (`RST,POR`, `RST,WDT`, etc.).
  - **Structured logs** (CSV-ish prefixes: `EVT`, `TMR`, `ANA`, `STAT`, `CFG`, `RST`).  
  - **Verbosity & live rate control** (`V0/1/2`, `Z<ms>` heartbeat, `Y<ms>` sampling).  
- Designed to produce **slow, human-readable output** for demos/screenshots.  
- (Optional) Gate LED by soil threshold to couple A0 into actuation.

---

## How to Build / Upload

1. **Open** Arduino IDE (Board: **Arduino Uno**, Port: your `/dev/cu.usbmodem…`).  
2. Open the sketch folder you want (e.g., `sketches/QP5_HD_SlowShow/QP5_HD_SlowShow.ino`).  
3. **Upload** (close Serial Monitor during upload).  
4. **Serial Monitor** @ **115200** baud.

> **OnTrack `.cpp`**: copy the contents of your final `.ino` into `TaskM1.cpp` at repo root and add `#include <Arduino.h>` at the top.

---

## Run the Final Demo (QP5)

1) Open Serial Monitor (**115200**).  
2) Set calm output:
V1
Z1000
Y500
- `V1` = normal verbosity (quiet enough for humans)  
- `Z1000` = heartbeat every 1 s (D13 toggles, prints “TMR,BEAT”)  
- `Y500` = A0 sample every 0.5 s (prints “ANA,A0,###”)

3) Show **event vs time**:
- Press **A (D2)** → `EVT,INT0,ARMED,1`  
- Flip **B/C (D8/D9)** → `EVT,PCI,8/9,FALL/RISE`  
- At the same time, observe `TMR,BEAT` and `ANA,A0,###` keep arriving.

4) **Grouped logic**:
- **Mode 0 (OR)**: while armed, any of B/C → LED12 ON.  
- Perform **B→C** within combo window → `EVT,COMBO,B2C,mode=1`.  
- **Mode 1 (AND)**: while armed, both B and C → LED12 ON.

5) **Config & persistence**:
T520
W800
M1
A1
P
- Power cycle the board (unplug/replug USB) → boot prints `CFG,EEPROM,OK` and `CFG,BOOT,…` with saved values.

6) **Watchdog demo**:
X
- Prints `WDT,DEMO,HANG_NOW`, then resets after ~1 s.  
- On boot you’ll see `RST,WDT`.

> If output is still too chatty, use `V0` to quiet, then `V1` just before the screenshot.

---

## Serial Menu Cheat Sheet

| Command | Meaning | Example |
|---|---|---|
| `H` | Help | `H` |
| `S` | Status line + current config echo | `S` |
| `T<0..1023>` | Set soil/analog threshold | `T520` |
| `W<50..2000>` | Set combo window (ms) | `W800` |
| `M0` / `M1` | Set mode (OR / AND) & save to defaults | `M1` |
| `A0` / `A1` | Set default armed on boot | `A1` |
| `P` | Persist config to EEPROM (with CRC) | `P` |
| `R` | Factory reset (defaults + save) | `R` |
| `V0/1/2` | Verbosity (quiet / normal / verbose) | `V1` |
| `Z<100..5000>` | Heartbeat period (ms) | `Z1000` |
| `Y<20..5000>` | Analog sample period (ms) | `Y500` |
| `X` | Deliberate hang (watchdog auto-reset) | `X` |

---

## Tinkercad Links & Diagrams

- **QP1** (polling) circuit link:  
  `<paste QP1 Tinkercad share URL here>`

- **QP3/QP4** (PCI + timer) circuit link:  
  `<paste QP3/QP4 Tinkercad share URL here>`

- Breadboard / schematic images (exported):  
  - `docs/wiring_QP3_QP4.png` *(PCI group D8/D9 highlighted)*

> **Simulation note:** Tinkercad doesn’t fire real Pin Change Interrupts. The QP3/QP4 sim sketches emulate PCI via **polling** but maintain the same external wiring and logic. The hardware sketch uses true PCINT.

---

## Screenshots (for assessment)

Place under `docs/`:

- `serial_QP4.png` – Shows **`TMR,BEAT`** + **`ANA,A0`** + at least one **`EVT,PCI,…`**  
- `serial_QP5_boot.png` – After power cycle: **`RST,POR`**, then **`CFG,EEPROM,OK`**, **`CFG,BOOT,…`**  
- `serial_QP5_events.png` – **`EVT,INT0,ARMED,1`**, **`EVT,PCI,8/9,…`**, and a **`EVT,COMBO,…`** line  
- `serial_QP5_wdt.png` – Shows **`RST,WDT`** after sending `X` (WDT demo)

---

## Known Quirks / Troubleshooting

- **Port / Upload (macOS):** Use **Tools → Board: Arduino Uno** and **Tools → Port: `/dev/cu.usbmodem…`**. Close Serial Monitor when uploading. If stuck, unplug/replug USB and reselect the port.  
- **“.ino.ino” / Mismatched names:** Arduino requires one `.ino` that **matches the folder name**. Example: `sketches/QP5_HD_SlowShow/QP5_HD_SlowShow.ino`.  
- **INPUT_PULLUP wiring:** Buttons/switches go **to GND**. Released = `HIGH`; pressed = `LOW`.  
- **Breadboard rows:** The center trench splits columns **E/F**. The side rails run vertically; confirm continuity with a shorting wire before inserting parts.  
- **Slide switch orientation:** Use the **center pin** to the signal (D8/D9), and one outer pin to **GND**. Leave the other outer pin **unconnected**.  
- **Tinkercad vs Hardware:** Sim uses polling to mimic PCI. On hardware, real PCINT works (fast edges).  
- **Debounce / lockout:** INT0 has a small lockout; PCI debounced in main loop (no `delay()` in ISRs).

---

## Submission Checklist

- [ ] **Private GitHub repo** (add tutor as collaborator).  
- [ ] `sketches/` folder with **QP1 → QP5** sketches.  
- [ ] `TaskM1.cpp` (final firmware copied from `.ino` with `#include <Arduino.h>`).  
- [ ] **Tinkercad links** + **wiring image** in `docs/`.  
- [ ] **Serial screenshots** as listed above (timer + PCI; boot; events; WDT).  
- [ ] **Reflection (300–500 words)** in `docs/QP4_QP5_Reflection.md` describing architecture, interrupts, issues & fixes.  
- [ ] Ready to **demonstrate**: show PCI & Timer events concurrently; explain ISR vs loop separation; persistence & watchdog.

---

## Academic Integrity

This repository is **private** by requirement. All work is individual unless explicitly approved otherwise. Please do not make this repo public; invite your marking tutor as a collaborator to review.

---
