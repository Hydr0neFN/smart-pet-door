# CLAUDE.md — Smart Pet Door (ESP32-C6)

Firmware for motor-driven insulated pet door. Single file: [`src/main.cpp`](src/main.cpp).
Captures non-obvious constraints; README = human overview. Human-readable copy: `CLAUDE.original.md`.

## Build

- PlatformIO, **pioarduino** fork of platform-espressif32 (official espressif32 has no C6).
- CLI: `C:\Users\Ruanyouyi\.platformio\penv\Scripts\pio.exe`.
- Three envs pick **radio profile** via `RADIO_MODE`:
  | Env | RADIO_MODE | Radio | Notes |
  |---|---|---|---|
  | `debug-wifi` *(default)* | 0 | Wi-Fi telnet, Zigbee off | `lib_ignore = Zigbee` (see below) |
  | `deploy-zigbee` | 1 | Zigbee ED only | `ZIGBEE_MODE_ED`, custom partition |
  | `debug-both` | 2 | Wi-Fi + Zigbee coex | heavier RAM |
- Build: `pio run -e <env>`. All three compile clean.

## Hardware constraints (non-obvious)

- **mmWave 1 (inside, LD2450) on UART0 = CH340 USB-console pins (GPIO16/17).** Console dead once
  inside radar wired. `LOG_TO_SERIAL` auto-off when `USE_RADAR_IN`=1. Wi-Fi telnet (`:23`) = live log then.
- **Native USB-Serial-JTAG gone** — pins (GPIO12/13) reused for MISO / END_STOP1. No native USB; don't suggest.
- C6 has Wi-Fi *and* 802.15.4 (Zigbee). Coexist (`debug-both`) but heavy; pick one per build.
- **Pin map = wiring table** (README / first big commit). GPIO1, GPIO9, GPIO15 = **Not Connected** —
  never assign. Original bug: door switch on GPIO1.
- End-stops: leaf 1 = GPIO13, leaf 2 = GPIO2. Enables `USE_END_STOP1/2`.
- Two motors = one aperture: leaf 2 runs **negated** target of leaf 1 (mirror).

## Radars

- Inside = **LD2450** (UART0), outside = **LD2410** (UART1). Both **256000** baud.
- Two frame formats, both parsed in `main.cpp`:
  - LD2450: `AA FF 03 00 | 3×8-byte targets | 55 CC` (30 B). Count non-zero target slots.
  - LD2410: `F4 F3 F2 F1 | len(LE) | payload | F8 F7 F6 F5`; presence = payload target-state byte ≠ 0.
- Presence latched per frame, trusted only if frame within `RADAR_STALE_MS`.

## Motor drivers

- TMC2130 **StealthChop** (`en_pwm_mode(true)`). Quiet required.
- **StallGuard NOT used** — needs SpreadCycle, can't coexist with StealthChop. ToF does anti-pinch. Don't re-add.

## Zigbee (deploy-zigbee / debug-both only)

- **esp-zigbee-sdk vendored in arduino-esp32** (`esp_zb_*` API + precompiled zboss). `<Zigbee.h>` =
  Espressif Arduino wrapper. Call SDK directly — no `lib_deps`; `ZIGBEE_MODE_ED` links zboss from
  framework. Don't add SDK repo separately.
- Arduino-ESP32 has **no door-lock wrapper** → `ZigbeePetDoorLock` = custom `ZigbeeEP` subclass on
  Door Lock cluster (0x0101), from `esp_zb_door_lock_clusters_create`. **Security/lock surface only**:
  `LockState` reflects LOCKED-mode.
- **4-mode (UNLOCKED/IN-ONLY/OUT-ONLY/LOCKED) on stock `ZigbeeMultistate`** (Multistate Output,
  indices 0–3) — standard cluster, ZHA/Z2M show writable select, no quirk. (Earlier cut used mfr attr
  `0xF000` on door lock; dropped — generic hubs can't control without converter.) Keep coherent with
  `doorMode`; push lock + multistate together.
- esp-zigbee-sdk symbol gotchas:
  - Role = `ESP_ZB_ZCL_CLUSTER_SERVER_ROLE` (NOT `..._ROLE_SERVER`).
  - **No enum for LockState values** — raw ZCL numbers (1=Locked, 2=Unlocked).
- Occupancy = stock `ZigbeeOccupancySensor`; pet count = stock `ZigbeeAnalog` (analog input).
- **Linker:** bundled Zigbee lib links zboss only with `ZIGBEE_MODE_ED`. Non-Zigbee env needs
  `lib_ignore = Zigbee`, else link fails on `esp_zb_ep_list_create`.
- **Flash:** zboss image (~1.39 MB) exceeds stock `zigbee.csv` dual-OTA slot (1.31 MB each). Use
  [`partitions_zigbee.csv`](partitions_zigbee.csv) — single-app, **no OTA**, 2.5 MB slot,
  `zb_storage`/`zb_fct`/`coredump` at stock offsets. (OTA = tradeoff.)

## This-machine environment gotchas

- **Repo in OneDrive** → git reflog append fails: `cannot update the ref 'HEAD': unable to append to
  '.git/logs/HEAD': Invalid argument`. Fix applied: local `core.logAllRefUpdates=false` +
  `windows.appendAtomically=false`, deleted `.git/logs`. Real fix: move repo out of OneDrive.
- **SSH to github.com MITM-intercepted on this network** (banner `SSH-2.0-6279353`, KEX fail) ports
  22 + 443 — host key untrusted. **Resolved:** `origin` switched to HTTPS, plain `git push` works
  (Git Credential Manager). Keep HTTPS here; SSH may work elsewhere.

## Verification limits

No hardware in-session. Compile passes all envs, but radar framing on real modules, Zigbee join,
door motion need flash + bench test before claiming they work.
