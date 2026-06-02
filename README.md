# Smart Pet Door 🐾

A thermally insulated, electronically actuated pet door that opens only on confirmed animal detection — replacing the permanent thermal bridge of a passive flap with a fully sealed, motor-driven door.

> Conventional flap doors leave a permanent gap in the building envelope and can raise HVAC demand by 6%+ annually. This door seals the opening completely when idle and opens only when a pet actually approaches, then closes and re-seals behind them.

Built on an **ESP32-C6** with dual mmWave radar presence detection, a Time-of-Flight anti-pinch safety curtain, and two silent TMC2130-driven stepper motors. Designed for smart-home integration over Zigbee/Matter.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Features](#features)
- [Hardware](#hardware)
- [GPIO Pin Map](#gpio-pin-map)
- [Repository Layout](#repository-layout)
- [Build & Flash](#build--flash)
- [Configuration](#configuration)
- [Safety System](#safety-system)
- [Access Modes & Connectivity](#access-modes--connectivity-design-target)
- [Roadmap](#roadmap)
- [Authors](#authors)

---

## How It Works

Two **LD2450 24 GHz mmWave radars** watch each side of the threshold (indoor + outdoor). When one detects a target approaching, the controller swings the door open. Once the radar field is clear for a hold delay, the door closes. A downward-facing **VL53L1X ToF sensor** at the gate acts as an invisible safety curtain — any obstruction detected during closing reverses the door (anti-pinch).

The firmware runs a four-state machine:

```
        radar present / timer            reached open angle
CLOSED ───────────────────────► OPENING ───────────────────► OPEN
   ▲                                                            │
   │ reached 0  (or door switch)                 clear + delay  │
   │                                                            ▼
   └──────────────── CLOSING ◄─────────────────────────────────┘
                        │
       ToF obstruction  │  → reverse → OPENING  (anti-pinch)
```

Two NEMA 17 motors are mounted **back-to-back** at the hinge and driven in mirror (motor 2 runs the negated target of motor 1), so the bi-parting panels open together. Drivers stay enabled at standstill so holding torque keeps the door shut against a push, while a reduced hold current (`IHOLD ≈ 0.3 × IRUN`) keeps the motors cool and quiet at rest.

## Features

- **Silent actuation** — TMC2130 StealthChop eliminates the audible whine of A4988/DRV8825-class drivers.
- **Presence detection, not contact** — 24 GHz FMCW radar detects an approaching animal at a distance; multi-target tracking can distinguish direction of travel.
- **Pre-emptive anti-pinch** — ToF curtain halts and reverses the door *before* contact, not after.
- **Secure at rest** — backdrivable drivetrain held closed by motor torque; resists being shoved open.
- **Dual-radar, selectable trigger** — inside-only, outside-only, or a radar-free timed test cycle, set at compile time.
- **Smart-home ready** — ESP32-C6 carries Wi-Fi 6, BLE 5, and an 802.15.4 radio for Zigbee today / Thread + Matter as the upgrade path.

## Hardware

| Subsystem | Part | Notes |
|---|---|---|
| MCU | **ESP32-C6** DevKitM-1 | RISC-V, Wi-Fi 6, BLE 5, 802.15.4 (Zigbee/Thread), Matter-ready |
| Motor driver | **TMC2130** ×2 | StealthChop (silent), StallGuard, SPI config, 1/256 µstep |
| Motor | **NEMA 17** ×2 | 1.8°/step, 0.3–0.6 Nm, direct hinge drive, back-to-back mount |
| Presence | **LD2450** mmWave radar ×2 | 24 GHz FMCW, UART @ 256000 baud, ~5 m range, one per side |
| Safety | **VL53L1X** ToF ×1 | I²C, 940 nm (no radar interference), downward gate mount |
| Power | 12 V SMPS → **LM2596** buck → 5 V | 12 V to drivers, 5 V to MCU + sensors; 1000 µF back-EMF caps per driver |
| Panel | Steel-faced **20 mm PIR foam** + rubber gasket | R ≈ 0.87 m²·K/W; overlapping bi-parting panels with perimeter seal |

A full Bill of Materials (fasteners, structural panels, connectors) and the design rationale for every part choice are in the project documentation.

## GPIO Pin Map

As wired in [`src/main.cpp`](src/main.cpp) for the ESP32-C6:

| Function | GPIO | Bus | Connects to |
|---|---|---|---|
| SPI SCK | 19 | SPI | TMC2130 ×2 — SCK |
| SPI MISO | 12 | SPI | TMC2130 ×2 — SDO |
| SPI MOSI | 20 | SPI | TMC2130 ×2 — SDI |
| Enable (shared, active-low) | 21 | — | TMC2130 ×2 — EN |
| Driver 1 STEP / DIR / CS | 11 / 10 / 18 | SPI/step | Motor driver 1 |
| Driver 2 STEP / DIR / CS | 23 / 22 / 3 | SPI/step | Motor driver 2 |
| ToF SDA / SCL | 6 / 7 | I²C | VL53L1X |
| Radar 1 (inside) RX / TX | 17 / 16 | UART0 | LD2450 #1 |
| Radar 2 (outside) RX / TX | 4 / 5 | UART1 | LD2450 #2 |
| Door switch (reserved) | 1 | — | Disabled until wired (`USE_DOOR_SWITCH 0`) |

> The ESP32-C6 console runs over USB-Serial-JTAG (HWCDC), leaving both hardware UARTs free for the two radars. The door-position switch is reserved on GPIO1 but not yet on the PCB; without it the firmware assumes "shut" at boot (`pos = 0`).

## Repository Layout

```
.
├── platformio.ini          PlatformIO env: esp32-c6-devkitm-1, deps, build flags
├── src/
│   ├── main.cpp            Current firmware — dual-motor door state machine
│   ├── main.old            Earlier single-motor build with StallGuard/DIAG1 stall ISR
│   └── ToF_Code.ino.ref    Standalone VL53L1X polling reference sketch
├── include/                Project headers
├── lib/                    Private libraries
└── test/                   PlatformIO tests
```

## Build & Flash

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

```bash
# Build
pio run

# Flash + open serial monitor (115200 baud)
pio run --target upload
pio device monitor -b 115200
```

The C6 board is **not** in the official `espressif32` platform (Arduino-ESP32 v2.x), so `platformio.ini` pins the [pioarduino](https://github.com/pioarduino/platform-espressif32) fork (Arduino-ESP32 v3.x). Library dependencies (`TMCStepper`, `FastAccelStepper`, `Adafruit VL53L1X`) are resolved automatically on first build.

## Configuration

Key compile-time options at the top of [`src/main.cpp`](src/main.cpp):

| Macro | Default | Purpose |
|---|---|---|
| `RADAR_SEL` | `0` | Trigger source: `0` = timed test cycle (no radar), `1` = inside radar drives door, `2` = outside radar |
| `OPEN_DEG` | `180` | Door open angle in degrees |
| `RMS_CURRENT` | `800` | Motor run current (mA, IRUN) |
| `IHOLD_MULT` | `0.3` | Standstill hold current as a fraction of run current — lower = cooler/quieter, raise if the door can be shoved open |
| `CLOSE_DELAY_MS` | `3000` | Radar-clear hold before the door starts closing |
| `TOF_DETECT_MM` | `350` | ToF distance below which an obstruction is flagged (with a 40 mm blind-spot floor) |
| `USE_DOOR_SWITCH` | `0` | Enable the closed-position switch as homing ground-truth once wired |

`RADAR_SEL 0` is the safe bring-up default: it cycles the door on a timer with no radar hardware attached, so you can validate motion, the ToF anti-pinch reverse, and current/heat before wiring the radars.

## Safety System

The design specifies three tiers of obstruction protection; tiers 1–2 are pre-emptive (act before contact):

| Tier | Sensor | Method | Status |
|---|---|---|---|
| 1 | VL53L1X ToF | Distance poll during close → reverse | ✅ Implemented (anti-pinch reopen) |
| 2 | LD2450 mmWave | Presence-hold keeps door open while occupied | ✅ Implemented |
| 3 | TMC2130 StallGuard | Back-EMF stall readback over SPI | 🔧 Prototyped in `main.old`, not on current PCB |

## Access Modes & Connectivity (design target)

The full design defines four remotely configurable access modes, enforced by direction of travel from the radar pair, plus a Zigbee-reported occupancy count:

| Direction \ Mode | UNLOCKED | IN-ONLY | OUT-ONLY | LOCKED |
|---|:---:|:---:|:---:|:---:|
| Entering (IN) | ✓ | ✓ | ✗ | ✗ |
| Leaving (OUT) | ✓ | ✗ | ✓ | ✗ |

Planned Zigbee surface: a **Door Lock** cluster for mode commands and an **Occupancy Sensing** cluster for the live pet count. Matter-over-Thread is the production upgrade path on the same ESP32-C6 hardware — no board revision required.

> **Note:** the firmware in this repo currently implements the motion + detection + anti-pinch core (the door state machine above). Direction-aware access modes, occupancy counting, and the Zigbee clusters are the documented design target and are not yet wired into `main.cpp`.

## Roadmap

- [ ] Direction-of-travel inference from LD2450 target coordinates
- [ ] UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED access-mode enforcement
- [ ] Zigbee join + Door Lock & Occupancy Sensing clusters
- [ ] Occupancy counting (+1 in / −1 out, floored at 0)
- [ ] Wire the closed-position door switch (GPIO1) for homing ground-truth
- [ ] Integrate StallGuard as the tier-3 reactive stall fallback on the production PCB
- [ ] Matter-over-Thread migration

## Authors

Group 2 — Hanze University of Applied Sciences:

- Yu-I Juan
- Ali Rowshanzadeh
- Ayyan Jayakar
- Andrei Dogari
- Wout de Moel
