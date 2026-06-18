# CLAUDE.md — Smart Pet Door (ESP32-C6)

Firmware for a motor-driven, insulated pet door. Single file: [`src/main.cpp`](src/main.cpp).
This file captures the non-obvious constraints; the README is the human-facing overview.

## Build

- PlatformIO, **pioarduino** fork of platform-espressif32 (official espressif32 has no C6).
- CLI on this machine: `C:\Users\Ruanyouyi\.platformio\penv\Scripts\pio.exe`.
- Three envs select a **radio profile** via the `RADIO_MODE` build flag:
  | Env | RADIO_MODE | Radio | Notes |
  |---|---|---|---|
  | `debug-wifi` *(default)* | 0 | Wi-Fi telnet, Zigbee off | `lib_ignore = Zigbee` (see below) |
  | `deploy-zigbee` | 1 | Zigbee ED only | `ZIGBEE_MODE_ED`, custom partition |
  | `debug-both` | 2 | Wi-Fi + Zigbee coex | heavier RAM |
- Build any env: `pio run -e <env>`. All three compile clean as of last change.

## Hardware constraints (the important, non-obvious part)

- **mmWave 1 (inside, LD2450) is on UART0 = the CH340 USB-console pins (GPIO16/17).**
  So the CH340 console is unusable once the inside radar is wired. `LOG_TO_SERIAL`
  auto-disables when `USE_RADAR_IN` is 1. Wi-Fi telnet (`:23`) is the live log path then.
- **Native USB-Serial-JTAG is gone** — its pins (GPIO12/13) are reused for MISO / END_STOP1.
  No native USB console available; don't suggest it.
- C6 has Wi-Fi *and* 802.15.4 (Zigbee). They can coexist (`debug-both`) but it's heavy;
  pick one per build normally.
- **Pin map = the wiring table** (in the README / first big commit). GPIO1, GPIO9, GPIO15 are
  **Not Connected** — never assign them. The original code's bug was the door switch on GPIO1.
- End-stops: leaf 1 = GPIO13, leaf 2 = GPIO2. Independent enables `USE_END_STOP1/2`.
- Two motors are one aperture: leaf 2 always runs the **negated** target of leaf 1 (mirror).

## Radars

- Inside = **LD2450** (UART0), outside = **LD2410** (UART1). Both default **256000** baud.
- Two different frame formats, both parsed in `main.cpp`:
  - LD2450: `AA FF 03 00 | 3×8-byte targets | 55 CC` (30 B). Count non-zero target slots.
  - LD2410: `F4 F3 F2 F1 | len(LE) | payload | F8 F7 F6 F5`; presence = payload target-state byte ≠ 0.
- Presence is latched per frame and only trusted if a frame arrived within `RADAR_STALE_MS`.

## Motor drivers

- TMC2130 in **StealthChop** (`en_pwm_mode(true)`). Quiet is the requirement.
- **StallGuard is intentionally NOT used** — it needs SpreadCycle and cannot coexist with
  StealthChop. Anti-pinch is handled by the ToF instead. Do not re-add StallGuard.

## Zigbee (deploy-zigbee / debug-both only)

- **esp-zigbee-sdk is vendored inside arduino-esp32** (the `esp_zb_*` API + precompiled zboss).
  `<Zigbee.h>` is Espressif's Arduino wrapper on top. We call the SDK directly — no `lib_deps`
  entry; `ZIGBEE_MODE_ED` links zboss from the framework. Don't add the SDK repo separately.
- Arduino-ESP32 ships **no door-lock wrapper** → `ZigbeePetDoorLock` is a custom `ZigbeeEP`
  subclass on the Door Lock cluster (0x0101), built from `esp_zb_door_lock_clusters_create`.
  4 modes (UNLOCKED/IN-ONLY/OUT-ONLY/LOCKED) ride a manufacturer-specific enum attr `0xF000`;
  standard Lock/Unlock maps onto LOCKED/UNLOCKED.
- esp-zigbee-sdk symbol gotchas learned the hard way:
  - Role constant is `ESP_ZB_ZCL_CLUSTER_SERVER_ROLE` (NOT `..._ROLE_SERVER`).
  - There is **no enum for LockState values** — use raw ZCL numbers (1=Locked, 2=Unlocked).
- Occupancy = stock `ZigbeeOccupancySensor`; pet count = stock `ZigbeeAnalog` (analog input).
- **Linker:** the bundled Zigbee core lib links zboss only with `ZIGBEE_MODE_ED`. In the
  non-Zigbee env it must be `lib_ignore = Zigbee`, else link fails on `esp_zb_ep_list_create`.
- **Flash size:** zboss image (~1.39 MB) exceeds the stock `zigbee.csv` dual-OTA app slot
  (1.31 MB each). Use [`partitions_zigbee.csv`](partitions_zigbee.csv) — single-app, **no OTA**,
  2.5 MB app slot, with `zb_storage`/`zb_fct`/`coredump` at stock offsets. (OTA is the tradeoff.)

## This-machine environment gotchas

- **Repo lives in OneDrive** → git reflog append fails: `cannot update the ref 'HEAD':
  unable to append to '.git/logs/HEAD': Invalid argument`. Worked around with local
  `core.logAllRefUpdates=false` + `windows.appendAtomically=false` and deleting `.git/logs`.
  Real fix: move the repo out of OneDrive.
- **SSH to github.com is MITM-intercepted on this network** (banner `SSH-2.0-6279353`, KEX
  failure) on ports 22 and 443 — host key cannot be trusted. **Resolved:** `origin` switched
  to the HTTPS URL, so plain `git push` works (creds via Git Credential Manager). Keep it HTTPS
  while on this network; SSH may work elsewhere.

## Verification limits

No hardware available in-session. Compile checks pass for all envs, but radar framing on real
modules, Zigbee join, and door motion need a flash + bench test before claiming they work.
