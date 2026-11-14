**English** · [繁體中文](README.zh-TW.md)

# Smart Pet Door 🐾

<img width="1917" height="1078" alt="image" src="https://github.com/user-attachments/assets/cc415e5f-6828-416f-af6a-ca0d08f0b3f9" />

A motor-driven pet door: it opens only when a radar confirms an animal, and stays sealed at rest.

## At a glance

| | |
|---|---|
| Problem | A passive flap leaves a permanent gap in the building envelope. HVAC demand can rise by 6%+ annually. |
| What I built | An electronically actuated, thermally insulated pet door. Two mmWave radars watch each side. A ToF safety curtain stops the closing leaf before contact. Two TMC2130-driven stepper motors move the leaves silently. |
| Result | No measured result in the old README. Key defaults: door open angle 180°. Close delay 3000 ms (radar clear). Run current 800 mA, hold current 0.3 × run. |
| Stack | ESP32-C6 DevKitM-1, Arduino-ESP32 v3.x (pioarduino fork) under PlatformIO, Zigbee (Door Lock + Occupancy Sensing + Analog Input) or Wi-Fi telnet logs, AccelStepper, TMCStepper, Adafruit VL53L1X |
| Status | archived |
| My role | Group 2 member (Hanze University of Applied Sciences) |

## How it works

A pair of **24 GHz mmWave radars** (one **LD2410** per side) watches the threshold. When a radar detects a target approaching, and the access mode permits that direction, the controller opens the door. After the field is clear for a hold delay, the door closes.

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

Two NEMA 17 motors sit back-to-back at the hinge and run in mirror, so the bi-parting panels act as one aperture. A **VL53L1X ToF sensor** at the gate is the invisible safety curtain. Any obstruction during closing reverses the door. The same beam gives the mid-gate "crossing" event used to count pets.

## The engineering story

- **Symptom: one leaf did not move.** FastAccelStepper's hardware step generation left one leaf dead on the ESP32-C6.
- **Cause.** Not stated. AccelStepper has no peripheral-channel limits.
- **Fix.** Motion moved to **AccelStepper**: software STEP/DIR pulses on any GPIO, no peripheral-channel limits. The TMC2130s are still set up over SPI for StealthChop.
- **Symptom: a closing door could pinch a pet.**
- **Fix.** The ToF curtain acts before contact, not after. It is tier 1 of the safety plan. Radar presence-hold (tier 2) keeps the door open while the pet is there. Tier 3, StallGuard, was dropped.
- **Symptom: hubs could not see the access mode.** The 4 modes first sat in a manufacturer-specific attribute. Generic hubs cannot read it.
- **Fix.** The modes now use a standard Multistate Output cluster. Hubs show it as a 4-option select.
- **What stays open.** OTA, an optional LD2450 on the inside slot (for target-coordinate direction inference, to refine count accuracy), and the Matter-over-Thread migration.

## Build / run

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension). The C6 board is **not** in the official `espressif32` platform (Arduino-ESP32 v2.x), so `platformio.ini` pins the [pioarduino](https://github.com/pioarduino/platform-espressif32) fork (Arduino-ESP32 v3.x). Dependencies (`TMCStepper`, `AccelStepper`, `Adafruit VL53L1X`) resolve on first build.

Pick a radio profile with the build env. Each env sets the `RADIO_MODE` flag.

```bash
# Build a profile
pio run -e debug-wifi
pio run -e deploy-zigbee

# Flash + monitor (debug build over CH340, only valid when inside radar disabled)
pio run -e deploy-zigbee -t upload
pio device monitor -b 115200
```

Wi-Fi builds log over telnet: connect to `petdoor.local:23` (or the device IP). The boot backlog is replayed to a late-joining client.

## Repo layout

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

## Limits and open items

- **Console.** The inside radar sits on **UART0 (GPIO16/17)**, the same pins as the CH340 USB-serial console. The native USB-Serial-JTAG pins (GPIO12/13) are reused for MISO / END_STOP1. Once the inside radar is wired, the CH340 console is unavailable and **Wi-Fi telnet is the live log path**. The CH340 console works only when the inside radar is disabled (`USE_RADAR_IN 0`).
- **No OTA.** The zboss image (~1.39 MB) overflows the stock dual-OTA `zigbee.csv` app slot (1.31 MB each) on a 4 MB flash. `partitions_zigbee.csv` trades OTA for one 2.5 MB app slot.
- **Count accuracy is refinable.** The optional LD2450 on the inside slot is not built.
- **Safety tier 3 is dropped.** StallGuard needs SpreadCycle and cannot coexist with the chosen StealthChop mode. ToF covers obstruction instead. Tiers 1–2 are implemented: anti-pinch reopen, and presence-hold.
- **Open work.** OTA support (needs a larger flash or a slimmer Zigbee image to restore dual app slots) and the Matter-over-Thread migration.

More detail:

- [Hardware](docs/hardware.md): parts table, features, GPIO and console notes.
- [Configuration](docs/configuration.md): the compile-time options and the bring-up aids.
- [Zigbee, access modes and safety](docs/zigbee.md): the 4 access modes, the Zigbee endpoints, and the implementation notes.
- [Roadmap and authors](docs/roadmap.md): what is done, what is open, and the group credits.

## License

MIT. See [LICENSE](LICENSE).
