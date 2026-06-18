# Smart Pet Door 🐾

A thermally insulated, electronically actuated pet door that opens only on confirmed animal detection — replacing the permanent thermal bridge of a passive flap with a fully sealed, motor-driven door.

> Conventional flap doors leave a permanent gap in the building envelope and can raise HVAC demand by 6%+ annually. This door seals the opening completely when idle and opens only when a pet actually approaches, then closes and re-seals behind them.

Built on an **ESP32-C6** with dual mmWave radar presence detection, a Time-of-Flight anti-pinch safety curtain, and two silent TMC2130-driven stepper motors. Integrates with smart homes over **Zigbee** (Door Lock + Occupancy clusters), with Matter-over-Thread as the upgrade path on the same hardware.

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
- [Access Modes & Connectivity](#access-modes--connectivity)
- [Roadmap](#roadmap)
- [Authors](#authors)

---

## How It Works

A pair of **24 GHz mmWave radars** watch each side of the threshold:

- **Inside — LD2450** (UART0): multi-target list, presence + active-target count.
- **Outside — LD2410** (UART1): single presence/state report.

When a radar detects a target approaching (and the current access mode permits that direction), the controller swings the door open. Once the field is clear for a hold delay, the door closes. A **VL53L1X ToF sensor** at the gate is an invisible safety curtain — any obstruction during closing reverses the door (anti-pinch) — and doubles as the mid-gate "crossing" event used to count pets.

> The design called for two LD2450s, but one arrived dead, so the build pairs one LD2450 with one LD2410. The firmware now decodes **both** UART protocols (LD2450 `AA FF 03 00 … 55 CC`; LD2410 `F4 F3 F2 F1 | len | … | F8 F7 F6 F5`), selected per radar slot.

The door runs a four-state machine:

```
        radar present (mode-gated)        reached open angle
CLOSED ───────────────────────► OPENING ───────────────────► OPEN
   ▲                                                            │
   │ both leaves shut (end-stop or pos 0)        clear + delay  │
   │                                                            ▼
   └──────────────── CLOSING ◄─────────────────────────────────┘
                        │
       ToF obstruction  │  → reverse → OPENING  (anti-pinch)
```

Two NEMA 17 motors are mounted **back-to-back** at the hinge and driven in mirror (leaf 2 runs the negated target of leaf 1), so the bi-parting panels move as one aperture. Each leaf can carry its own end-stop switch (independently enable-able) as the closed ground-truth. Drivers stay enabled at standstill so holding torque keeps the door shut against a push, while a reduced hold current (`IHOLD ≈ 0.3 × IRUN`) keeps the motors cool and quiet at rest.

### Pet counting

A pass is counted only when the full sequence is observed within timeouts: one side's radar → the ToF beam is crossed → the **other** side's radar. Direction sets the running occupancy: outside→inside is `+1`, inside→outside is `−1` (floored at 0). The live count is published over Zigbee (see below). Counting requires the ToF sensor (`USE_TOF 1`).

## Features

- **Silent actuation** — TMC2130 StealthChop eliminates the audible whine of A4988/DRV8825-class drivers.
- **Dual-protocol radar** — LD2450 (multi-target/direction-capable) inside, LD2410 (presence) outside, each with its own frame parser and a staleness guard.
- **Pre-emptive anti-pinch** — ToF curtain halts and reverses the door *before* contact, not after.
- **Secure at rest** — backdrivable drivetrain held closed by motor torque; resists being shoved open.
- **Per-leaf end-stops** — two independent switches (`END_STOP1`/`END_STOP2`), each separately enable-able, used for homing and as the closed ground-truth.
- **Directional pet counting** — radar → ToF → radar pass detection maintains live occupancy.
- **Smart-home native** — Zigbee Door Lock (access mode), Occupancy Sensing (occupied), and Analog Input (pet count) clusters; selectable radio profile at build time.

## Hardware

| Subsystem | Part | Notes |
|---|---|---|
| MCU | **ESP32-C6** DevKitM-1 | RISC-V, Wi-Fi 6, BLE 5, 802.15.4 (Zigbee/Thread), Matter-ready |
| Motor driver | **TMC2130** ×2 | StealthChop (silent), SPI config, 1/16 µstep |
| Motor | **NEMA 17** ×2 | 1.8°/step, 0.3–0.6 Nm, direct hinge drive, back-to-back mount |
| Presence | **LD2450** ×1 + **LD2410** ×1 | 24 GHz FMCW, UART @256000, ~5 m, one per side. LD2410 substituted after an LD2450 arrived dead |
| Safety | **VL53L1X** ToF ×1 | I²C, 940 nm (no radar interference), downward gate mount |
| Power | 12 V SMPS → **LM2596** buck → 5 V | 12 V to drivers, 5 V to MCU + sensors; 1000 µF back-EMF caps per driver |
| Panel | Steel-faced **20 mm PIR foam** + rubber gasket | R ≈ 0.87 m²·K/W; overlapping bi-parting panels with perimeter seal |

A full Bill of Materials and the design rationale for every part choice are in the project documentation.

## GPIO Pin Map

As wired in [`src/main.cpp`](src/main.cpp) for the ESP32-C6 (matches the board wiring table):

| Function | GPIO | Bus | Connects to |
|---|---|---|---|
| SPI SCK / MISO / MOSI | 19 / 12 / 20 | SPI | TMC2130 ×2 |
| Enable (shared, active-low) | 21 | — | TMC2130 ×2 — EN |
| Driver 1 STEP / DIR / CS | 11 / 10 / 18 | SPI/step | Motor driver 1 (leaf 1) |
| Driver 2 STEP / DIR / CS | 23 / 22 / 3 | SPI/step | Motor driver 2 (leaf 2) |
| End-stop 1 / End-stop 2 | 13 / 2 | — | Per-leaf closed switch (`USE_END_STOP1/2`) |
| ToF SDA / SCL / INT | 6 / 7 / 0 | I²C | VL53L1X |
| Radar 1 inside (RX / TX) | 17 / 16 | UART0 | LD2450 |
| Radar 2 outside (RX / TX) | 4 / 5 | UART1 | LD2410 |
| Status NeoPixel | 8 | — | Onboard WS2812 |

> **Console caveat:** the inside radar (LD2450) sits on **UART0 (GPIO16/17)** — the same pins as the CH340 USB-serial console — and the native USB-Serial-JTAG pins (GPIO12/13) are reused for MISO / END_STOP1. So once the inside radar is wired, the CH340 console is unavailable and **Wi-Fi telnet is the live log path**. The CH340 console is only usable when the inside radar is disabled (`USE_RADAR_IN 0`). The status NeoPixel encodes the current stage as a fallback when no log is attached.

## Repository Layout

```
.
├── platformio.ini          Three build envs (debug-wifi / deploy-zigbee / debug-both)
├── partitions_zigbee.csv   Single-app (no-OTA) 4MB layout for the Zigbee build
├── src/
│   ├── main.cpp            Firmware — dual-radar door FSM, pet counting, Zigbee
│   └── ToF_Code.ino.ref    Standalone VL53L1X polling reference sketch
├── include/                Project headers
├── lib/                    Private libraries
└── test/                   PlatformIO tests
```

## Build & Flash

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension). The C6 board is **not** in the official `espressif32` platform (Arduino-ESP32 v2.x), so `platformio.ini` pins the [pioarduino](https://github.com/pioarduino/platform-espressif32) fork (Arduino-ESP32 v3.x). Dependencies (`TMCStepper`, `FastAccelStepper`, `Adafruit VL53L1X`) resolve on first build.

The ESP32-C6 carries both a 2.4 GHz Wi-Fi radio and an 802.15.4 (Zigbee) radio. Pick a **radio profile** by choosing the build env — each sets the `RADIO_MODE` flag:

| Env | `RADIO_MODE` | Radio | Logging | Use |
|---|:---:|---|---|---|
| `debug-wifi` *(default)* | 0 | Wi-Fi only | telnet `:23` (+ CH340 if inside radar off) | bring-up / debugging |
| `deploy-zigbee` | 1 | Zigbee only | USB serial (when inside radar off) | shipping smart-home device |
| `debug-both` | 2 | Wi-Fi + Zigbee (coex) | telnet `:23` | debugging Zigbee with live logs (higher RAM) |

```bash
# Build a profile
pio run -e debug-wifi
pio run -e deploy-zigbee

# Flash + monitor (debug build over CH340, only valid when inside radar disabled)
pio run -e deploy-zigbee -t upload
pio device monitor -b 115200
```

Wi-Fi builds log over telnet: connect to `petdoor.local:23` (or the device IP) — the boot backlog is replayed to a late-joining client. The Zigbee build uses a single-app, **no-OTA** partition (`partitions_zigbee.csv`) because the zboss stack pushes the image past the stock dual-OTA `zigbee.csv` app slot.

## Configuration

Key compile-time options at the top of [`src/main.cpp`](src/main.cpp):

| Macro | Default | Purpose |
|---|---|---|
| `RADIO_MODE` | per-env | `0` Wi-Fi / `1` Zigbee / `2` both — set by the PlatformIO env, not edited by hand |
| `USE_RADAR_IN` / `USE_RADAR_OUT` | `1` / `1` | Enable each radar slot; disabling the inside radar frees UART0 for the CH340 console |
| `TEST_TIMED_CYCLE` | `0` | `1` = ignore radars and cycle the door on timers (mechanical bring-up) |
| `USE_END_STOP1` / `USE_END_STOP2` | `0` / `0` | Enable each per-leaf closed switch once wired |
| `USE_TOF` | `0` | Enable VL53L1X — required for anti-pinch **and** pet counting |
| `OPEN_DEG` | `180` | Door open angle in degrees |
| `RMS_CURRENT` | `800` | Motor run current (mA, IRUN) |
| `IHOLD_MULT` | `0.3` | Standstill hold current as a fraction of run current |
| `CLOSE_DELAY_MS` | `3000` | Radar-clear hold before the door starts closing |
| `TOF_DETECT_MM` | `350` | ToF distance below which an obstruction is flagged (40 mm blind-spot floor) |

`TEST_TIMED_CYCLE 1` is the safe bring-up default for mechanics: it cycles the door on a timer with no radar attached, so you can validate motion, the ToF anti-pinch reverse, and current/heat before wiring the radars.

## Safety System

Three tiers of obstruction protection were specified; tiers 1–2 are pre-emptive (act before contact):

| Tier | Sensor | Method | Status |
|---|---|---|---|
| 1 | VL53L1X ToF | Distance poll during close → reverse | ✅ Implemented (anti-pinch reopen) |
| 2 | mmWave radar | Presence-hold keeps door open while occupied | ✅ Implemented |
| 3 | TMC2130 StallGuard | Back-EMF stall readback over SPI | ❌ Dropped — StallGuard requires SpreadCycle and cannot coexist with the chosen StealthChop mode; ToF covers obstruction instead |

## Access Modes & Connectivity

Four remotely configurable access modes are enforced by direction of travel from the radar pair:

| Direction \ Mode | UNLOCKED | IN-ONLY | OUT-ONLY | LOCKED |
|---|:---:|:---:|:---:|:---:|
| Entering (IN) — outside radar | ✓ | ✓ | ✗ | ✗ |
| Leaving (OUT) — inside radar | ✓ | ✗ | ✓ | ✗ |

The Zigbee surface (built in `deploy-zigbee` / `debug-both`) exposes three endpoints:

- **Door Lock cluster** — operating mode `UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED`. Standard Lock/Unlock maps onto LOCKED/UNLOCKED; a manufacturer-specific enum attribute (`0xF000`) carries the directional IN-ONLY / OUT-ONLY modes the standard cluster has no slot for. Built as a custom endpoint since the Arduino-ESP32 Zigbee library ships no door-lock wrapper.
- **Occupancy Sensing cluster** — `occupied` flag (true while a pet is inside).
- **Analog Input cluster** — live pet count (pets currently inside), reported on change.

### Zigbee implementation notes (the *why*)

- **Uses esp-zigbee-sdk via the framework, not a separate dependency.** Arduino-ESP32 v3.x (the pioarduino fork) *vendors* Espressif's [esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk) — the `esp_zb_*` API plus the precompiled zboss libraries. `#include <Zigbee.h>` is the thin Arduino wrapper on top. The custom door-lock endpoint calls the SDK directly (`esp_zb_door_lock_clusters_create`, `esp_zb_cluster_add_attr`, …); the `ZIGBEE_MODE_ED` build flag links zboss from the framework. No `lib_deps` entry — adding the SDK repo separately would clash with the bundled copy.
- **Custom Door Lock endpoint.** The Arduino wrapper has classes for Occupancy and Analog (used as-is) but **none for Door Lock**, so `ZigbeePetDoorLock` is a hand-built `ZigbeeEP` subclass on cluster `0x0101`.
- **Manufacturer attr `0xF000` for the 4 modes.** Standard ZCL Door Lock is binary (Lock/Unlock, LockState Locked/Unlocked) — no slot for directional IN-ONLY/OUT-ONLY. So LOCKED/UNLOCKED map onto the standard state (any generic controller can read the lock) while the full 4-way mode rides a manufacturer-specific enum.
- **`lib_ignore = Zigbee` in the non-Zigbee env.** The bundled Zigbee lib compiles regardless, but zboss only links with `ZIGBEE_MODE_ED`; without it the build fails at link (`undefined reference to esp_zb_ep_list_create`). Ignoring the lib in `debug-wifi` avoids that.
- **No-OTA partition.** The zboss image (~1.39 MB) overflows the stock dual-OTA `zigbee.csv` app slot (1.31 MB each) on a 4 MB flash. `partitions_zigbee.csv` trades OTA for one 2.5 MB app slot.

Matter-over-Thread is the production upgrade path on the same ESP32-C6 hardware — no board revision required.

## Roadmap

- [x] LD2410 frame parser for the outside radar channel
- [x] UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED access-mode enforcement
- [x] Zigbee join + Door Lock, Occupancy Sensing, and Analog (count) clusters
- [x] Occupancy counting (+1 in / −1 out, floored at 0) via radar → ToF → radar
- [x] Per-leaf closed-position end-stop switches for homing ground-truth
- [ ] Direction-of-travel inference from LD2450 target coordinates (refine count accuracy)
- [ ] OTA support (needs a larger flash or a slimmer Zigbee image to restore dual app slots)
- [ ] Matter-over-Thread migration

## Authors

Group 2 — Hanze University of Applied Sciences:

- Yu-I Juan
- Ali Rowshanzadeh
- Ayyan Jayakar
- Andrei Dogari
- Wout de Moel
