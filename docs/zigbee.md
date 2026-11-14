**English** · [繁體中文](zigbee.zh-TW.md)

# Radio profiles, safety, access modes and Zigbee

## Radio profiles

The ESP32-C6 has a 2.4 GHz Wi-Fi radio and an 802.15.4 (Zigbee) radio. Pick a **radio profile** with the build env. Each env sets the `RADIO_MODE` flag.

| Env | `RADIO_MODE` | Radio | Logging | Use |
|---|:---:|---|---|---|
| `debug-wifi` *(default)* | 0 | Wi-Fi only | telnet `:23` (+ CH340 if inside radar off) | bring-up / debugging |
| `deploy-zigbee` | 1 | Zigbee only | USB serial (when inside radar off) | shipping smart-home device |
| `debug-both` | 2 | Wi-Fi + Zigbee (coex) | telnet `:23` | debugging Zigbee with live logs (higher RAM) |

The Zigbee build uses a single-app, **no-OTA** partition (`partitions_zigbee.csv`). The reason is in the implementation notes below.

## Safety system

Three tiers of obstruction protection were specified. Tiers 1–2 are pre-emptive (they act before contact).

| Tier | Sensor | Method | Status |
|---|---|---|---|
| 1 | VL53L1X ToF | Distance poll during close → reverse | ✅ Implemented (anti-pinch reopen) |
| 2 | mmWave radar | Presence-hold keeps door open while occupied | ✅ Implemented |
| 3 | TMC2130 StallGuard | Back-EMF stall readback over SPI | ❌ Dropped — StallGuard needs SpreadCycle and cannot coexist with the chosen StealthChop mode; ToF covers obstruction instead |

## Access modes

Direction of travel from the radar pair enforces four access modes. All four are set remotely.

| Direction \ Mode | UNLOCKED | IN-ONLY | OUT-ONLY | LOCKED |
|---|:---:|:---:|:---:|:---:|
| Entering (IN) — outside radar | ✓ | ✓ | ✗ | ✗ |
| Leaving (OUT) — inside radar | ✓ | ✗ | ✓ | ✗ |

## Zigbee surface

The Zigbee surface (built in `deploy-zigbee` / `debug-both`) has four endpoints:

- **Door Lock cluster.** The security/lock surface. The device advertises as a Door Lock and the standard `LockState` shows whether the door is in `LOCKED` mode (any generic controller can read it).
- **Multistate Output cluster.** The 4-way access mode `UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED` (indices 0–3). It is a standard cluster, so hubs (ZHA / Zigbee2MQTT) show it as a writable select with no custom quirk.
- **Occupancy Sensing cluster.** The `occupied` flag (true while a pet is inside).
- **Analog Input cluster.** The live pet count (pets currently inside), reported on change.

Both mode surfaces stay in step with the firmware's `doorMode`. Setting `LOCKED` on either one flips the Door Lock to Locked, and any mode change goes to both.

## Zigbee implementation notes (the *why*)

- **Uses esp-zigbee-sdk through the framework, not a separate dependency.** Arduino-ESP32 v3.x (the pioarduino fork) *vendors* Espressif's [esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk): the `esp_zb_*` API plus the precompiled zboss libraries. `#include <Zigbee.h>` is the thin Arduino wrapper on top. The custom door-lock endpoint calls the SDK directly (`esp_zb_door_lock_clusters_create`, …). The `ZIGBEE_MODE_ED` build flag links zboss from the framework. There is no `lib_deps` entry, because a separate copy of the SDK repo would clash with the bundled one.
- **Why Multistate for the mode, not a manufacturer attribute.** Standard ZCL Door Lock is binary (LockState Locked/Unlocked), with no slot for directional IN-ONLY/OUT-ONLY. The first cut carried the 4 modes in a *manufacturer-specific* attribute on the Door Lock cluster, but generic hubs cannot see that without a custom converter/quirk. A **standard Multistate Output** shows natively as a 4-option select, so the modes work out of the box. The Door Lock cluster stays purely as the security/lock advertise and status.
- **Custom Door Lock endpoint; stock everything else.** The Arduino wrapper has classes for Multistate, Occupancy and Analog (all used as-is), but **none for Door Lock**. So `ZigbeePetDoorLock` is a hand-built `ZigbeeEP` subclass on cluster `0x0101`.
- **`lib_ignore = Zigbee` in the non-Zigbee env.** The bundled Zigbee lib compiles regardless, but zboss links only with `ZIGBEE_MODE_ED`. Without it the build fails at link (`undefined reference to esp_zb_ep_list_create`). Ignoring the lib in `debug-wifi` avoids that.
- **No-OTA partition.** The zboss image (~1.39 MB) overflows the stock dual-OTA `zigbee.csv` app slot (1.31 MB each) on a 4 MB flash. `partitions_zigbee.csv` trades OTA for one 2.5 MB app slot.

Matter-over-Thread is the production upgrade path on the same ESP32-C6 hardware. It needs no board revision.

## Motion library

Motion uses **AccelStepper** (software STEP/DIR pulses on any GPIO). FastAccelStepper's hardware step generation left one leaf dead on the ESP32-C6, and AccelStepper has no peripheral-channel limits. The TMC2130s are still configured over SPI for StealthChop.
