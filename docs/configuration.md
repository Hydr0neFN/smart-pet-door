**English** · [繁體中文](configuration.zh-TW.md)

# Configuration

Key compile-time options at the top of [`src/main.cpp`](../src/main.cpp):

| Macro | Default | Purpose |
|---|---|---|
| `RADIO_MODE` | per-env | `0` Wi-Fi / `1` Zigbee / `2` both — set by the PlatformIO env, not edited by hand |
| `USE_RADAR_IN` / `USE_RADAR_OUT` | `1` / `1` | Enable each radar slot; disabling the inside radar frees UART0 for the CH340 console |
| `TEST_TIMED_CYCLE` | derived | `(!USE_RADAR_IN && !USE_RADAR_OUT)` — both radars off ⇒ timed open/close cycle; any radar on ⇒ radar-driven. No separate flag to forget |
| `MOTOR_SELFTEST` | `0` | `1` = jog each leaf at boot, log commanded vs reported position (isolate a dead leaf: wiring/VM vs step generation) |
| `SENSOR_DEBUG` | `0` | `1` = stream raw ToF + radar readings (~4 Hz, incl. `rx`/`good`/`bad` frame counts) and skip the door FSM — sensor bring-up/aiming |
| `USE_END_STOP1` / `USE_END_STOP2` (`END_STOP1`/`END_STOP2`) | `0` / `0` | Enable each per-leaf closed switch once wired |
| `USE_TOF` | `0` | Enable VL53L1X — required for anti-pinch **and** pet counting |
| `OPEN_DEG` | `180` | Door open angle in degrees |
| `RMS_CURRENT` | `800` | Motor run current (mA, IRUN) |
| `IHOLD_MULT` | `0.3` | Standstill hold current as a fraction of run current |
| `CLOSE_DELAY_MS` | `3000` | Radar-clear hold before the door starts closing |
| `TOF_DETECT_MM` | `350` | ToF distance below which an obstruction is flagged (40 mm blind-spot floor) |

Bring-up aids: with both radars disabled the firmware selects a timed open/close cycle (no flag to set). You can then check motion, the ToF anti-pinch reverse, and current/heat before you wire the radars. `MOTOR_SELFTEST` isolates a leaf that does not spin. Note: the TMC2130 reports a valid `version` over SPI even with no 12 V motor supply. `configDriver` warns on charge-pump undervoltage, so a missing VM is not mistaken for a healthy driver. `SENSOR_DEBUG` streams live ToF/radar values for aiming and to confirm frame parsing.
