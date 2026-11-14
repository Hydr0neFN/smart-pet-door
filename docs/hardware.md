**English** · [繁體中文](hardware.zh-TW.md)

# Hardware, features and pin map

## Features

- **Silent actuation.** TMC2130 StealthChop removes the audible whine of A4988/DRV8825-class drivers.
- **Dual presence radar.** One **LD2410** on each side (inside and outside), each parsed with a staleness guard. The firmware keeps an LD2450 parser too, so a multi-target LD2450 can go into the inside channel without code changes.
- **Pre-emptive anti-pinch.** The ToF curtain stops and reverses the door *before* contact, not after.
- **Secure at rest.** The backdrivable drivetrain is held closed by motor torque and resists a shove.
- **Per-leaf end-stops.** Two independent switches (`END_STOP1`/`END_STOP2`), each separately enable-able, used for homing and as the closed ground truth.
- **Directional pet counting.** Radar → ToF → radar pass detection keeps a live occupancy count.
- **Smart-home native.** Zigbee Door Lock (access mode), Occupancy Sensing (occupied), and Analog Input (pet count) clusters. The radio profile is selected at build time.

## Pet counting

A pass counts only when the full sequence is seen within the timeouts: one side's radar → the ToF beam is crossed → the **other** side's radar. Direction sets the running occupancy: outside→inside is `+1`, inside→outside is `−1` (floored at 0). The live count is published over Zigbee. Counting needs the ToF sensor (`USE_TOF 1`).

## Parts

| Subsystem | Part | Notes |
|---|---|---|
| MCU | **ESP32-C6** DevKitM-1 | RISC-V, Wi-Fi 6, BLE 5, 802.15.4 (Zigbee/Thread), Matter-ready |
| Motor driver | **TMC2130** ×2 | StealthChop (silent), SPI config, 1/16 µstep |
| Motor | **NEMA 17** ×2 | 1.8°/step, 0.3–0.6 Nm, direct hinge drive, back-to-back mount |
| Presence | **LD2410** ×2 | 24 GHz FMCW, UART @256000, ~5 m, one per side (inside + outside). Firmware also parses LD2450 if one is fitted |
| Safety | **VL53L1X** ToF ×1 | I²C, 940 nm (no radar interference), downward gate mount |
| Power | 12 V SMPS → **LM2596** buck → 5 V | 12 V to drivers, 5 V to MCU + sensors; 1000 µF back-EMF caps per driver |
| Panel | Steel-faced **20 mm PIR foam** + rubber gasket | R ≈ 0.87 m²·K/W; overlapping bi-parting panels with perimeter seal |

A full Bill of Materials and the design rationale for every part choice are in the project documentation.

## Radar slots and the LD2450 option

Both slots run **LD2410**. The firmware keeps a parser for both UART protocols (LD2450 `AA FF 03 00 … 55 CC`; LD2410 `F4 F3 F2 F1 | len | … | F8 F7 F6 F5`) and selects per slot. So a multi-target **LD2450 can go into the inside slot later** with a one-line type change.

## Door motion

Two NEMA 17 motors are mounted **back-to-back** at the hinge and driven in mirror (leaf 2 runs the negated target of leaf 1), so the bi-parting panels move as one aperture. Each leaf can carry its own end-stop switch (independently enable-able) as the closed ground truth. The drivers stay enabled at standstill, so holding torque keeps the door shut against a push. A reduced hold current (`IHOLD ≈ 0.3 × IRUN`) keeps the motors cool and quiet at rest.

## GPIO pin map

As wired in [`src/main.cpp`](../src/main.cpp) for the ESP32-C6 (matches the board wiring table):

| Function | GPIO | Bus | Connects to |
|---|---|---|---|
| SPI SCK / MISO / MOSI | 19 / 12 / 20 | SPI | TMC2130 ×2 |
| Enable (shared, active-low) | 21 | — | TMC2130 ×2 — EN |
| Driver 1 STEP / DIR / CS | 11 / 10 / 18 | SPI/step | Motor driver 1 (leaf 1) |
| Driver 2 STEP / DIR / CS | 23 / 22 / 3 | SPI/step | Motor driver 2 (leaf 2) |
| End-stop 1 / End-stop 2 (`END_STOP1`/`END_STOP2`) | 13 / 2 | — | Per-leaf closed switch (`USE_END_STOP1/2`) |
| ToF SDA / SCL / INT | 6 / 7 / 0 | I²C | VL53L1X |
| Radar 1 inside (RX / TX) | 17 / 16 | UART0 | LD2410 |
| Radar 2 outside (RX / TX) | 4 / 5 | UART1 | LD2410 |
| Status NeoPixel | 8 | — | Onboard WS2812 |

> **Console caveat:** the inside radar (LD2410) sits on **UART0 (GPIO16/17)**, the same pins as the CH340 USB-serial console. The native USB-Serial-JTAG pins (GPIO12/13) are reused for MISO / END_STOP1. So once the inside radar is wired, the CH340 console is unavailable and **Wi-Fi telnet is the live log path**. The CH340 console works only when the inside radar is disabled (`USE_RADAR_IN 0`). Without a log, the status NeoPixel encodes the current stage as a fallback.

`src/ToF_Code.ino.ref` is a separate polling sketch and does not use the firmware pins: it puts I2C on GPIO 19 / 18.
