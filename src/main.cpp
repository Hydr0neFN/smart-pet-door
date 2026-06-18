// ===========================================================================
//  Smart Pet Door — ESP32-C6 (DevKitM-1)
// ---------------------------------------------------------------------------
//  Two mmWave radars detect an animal and drive a double-leaf door:
//    - INSIDE  radar: LD2450 on UART0 (per-target list, presence + count)
//    - OUTSIDE radar: LD2410 on UART1 (single presence/state report)
//  A VL53L1X ToF beam in the doorway gives anti-pinch protection AND the
//  middle "crossing" event used to count pets as they pass through.
//
//  Door motion: two TMC2130 / stepper leaves mounted back-to-back, driven in
//  mirror so they open and close as one aperture. Each leaf may carry its own
//  end-stop switch (independently enable-able) used as the closed ground truth.
//
//  Smart-home control (build env "deploy-zigbee" / "debug-both") exposes:
//    - Door Lock cluster      : operating mode UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED
//    - Occupancy Sensing      : occupied flag (pet present inside)
//    - Analog Input cluster   : live pet count (pets currently inside)
//
//  Radio profile is selected at build time via RADIO_MODE (see platformio.ini):
//    0 = Wi-Fi telnet log, Zigbee off   1 = Zigbee only   2 = both (coex)
// ===========================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// ── Radio profile ──────────────────────────────────────────────────────────
#define RADIO_WIFI    0
#define RADIO_ZIGBEE  1
#define RADIO_BOTH    2
#ifndef RADIO_MODE
#define RADIO_MODE    RADIO_WIFI
#endif
#define RADIO_USES_WIFI    (RADIO_MODE == RADIO_WIFI   || RADIO_MODE == RADIO_BOTH)
#define RADIO_USES_ZIGBEE  (RADIO_MODE == RADIO_ZIGBEE || RADIO_MODE == RADIO_BOTH)

#if RADIO_USES_WIFI
  #include <WiFi.h>
  #include <ESPmDNS.h>
#endif
#if RADIO_USES_ZIGBEE
  #include <Zigbee.h>
#endif

// ===========================================================================
//  Pin map — ESP32-C6 DevKitM-1 (matches the wiring table; pin = header pin#)
// ===========================================================================
// ── SPI bus (shared by both TMC2130 drivers) ──
#define PIN_SCK         19   // pin26  GPIO19  SCK
#define PIN_MISO        12   // pin13  GPIO12  MISO
#define PIN_MOSI        20   // pin25  GPIO20  MOSI
#define PIN_EN          21   // pin24  GPIO21  shared active-low driver enable

// ── Stepper driver 1 / leaf 1 ──
#define PIN_M1_STEP     11   // pin12  GPIO11  MOTOR1_STEP
#define PIN_M1_DIR      10   // pin11  GPIO10  MOTOR1_DIR
#define PIN_M1_CS       18   // pin27  GPIO18  CS_1

// ── Stepper driver 2 / leaf 2 ──
#define PIN_M2_STEP     23   // pin22  GPIO23  MOTOR2_STEP
#define PIN_M2_DIR      22   // pin23  GPIO22  MOTOR2_DIR
#define PIN_M2_CS        3   // pin18  GPIO3   CS_2

// ── Status NeoPixel (onboard WS2812) ──
#define PIN_LED          8   // pin10  GPIO8

// ── End-stop switches (one per leaf; closed = ground truth) ──
#define PIN_END_STOP1   13   // pin14  GPIO13  END_STOP1 (leaf 1)
#define PIN_END_STOP2    2   // pin17  GPIO2   END_STOP2 (leaf 2)

// ── ToF VL53L1X (I2C) ──
#define PIN_I2C_SDA      6   // pin6   GPIO6   I2C_SDA
#define PIN_I2C_SCL      7   // pin7   GPIO7   I2C_SCL
#define PIN_TOF_INT      0   // pin8   GPIO0   TOF_INT (IRQ available; polling used)

// ── mmWave 1 INSIDE  (UART0, shared with CH340 console) ──
#define PIN_R1_RX       17   // pin20  UART_RXD  <- mmWave1 TX
#define PIN_R1_TX       16   // pin19  UART_TXD  -> mmWave1 RX

// ── mmWave 2 OUTSIDE (UART1) ──
#define PIN_R2_RX        4   // pin4   GPIO4   <- mmWave2 TX
#define PIN_R2_TX        5   // pin5   GPIO5   -> mmWave2 RX

// ===========================================================================
//  Feature / behaviour configuration
// ===========================================================================
// ── Radars ──
// Slot roles are fixed by the wiring: inside = LD2450, outside = LD2410.
// Disable a slot if its module is not yet wired. Disabling the inside radar
// frees UART0 so the CH340 USB console (Serial) becomes usable again.
#define USE_RADAR_IN     1            // LD2450 inside  (UART0)
#define USE_RADAR_OUT    1            // LD2410 outside (UART1)
#define RADAR_IN_BAUD    256000       // LD2450 factory default
#define RADAR_OUT_BAUD   256000       // LD2410 factory default
#define RADAR_STALE_MS   1500         // presence ignored if no fresh frame within

// ── Bench test ──
// 1 = ignore radars and cycle the door on timers (mechanical bring-up).
#define TEST_TIMED_CYCLE 0

// ── End-stop switches ──
// Set to 1 once the corresponding leaf switch is wired.
#define USE_END_STOP1    0
#define USE_END_STOP2    0
#define END_STOP_ACTIVE  LOW          // level read when that leaf is shut
#define HOME_TIMEOUT_MS  8000

// ── ToF (VL53L1X) ──
// 0 = sensor absent: skip I2C, anti-pinch and pet counting disabled.
#define USE_TOF          0
#define TOF_BLIND_MM     40           // ignore <40mm blind spot
#define TOF_DETECT_MM    350          // object closer than this = blocked beam

// ── Driver current ──
#define R_SENSE          0.11f
#define RMS_CURRENT      800          // IRUN (moving), driver 1
#define RMS_CURRENT2     800          // IRUN (moving), driver 2
// Backdrivable drivetrain: keep drivers enabled at standstill so the door
// resists being pushed, but cut hold current to kill idle heat. Raise if the
// door can be shoved open.
#define IHOLD_MULT       0.3f

// ── Geometry / motion ──
#define MICROSTEPS       16
#define STEPS_PER_REV    (200 * MICROSTEPS)
#define OPEN_DEG         180
#define OPEN_STEPS       ((int32_t)STEPS_PER_REV * OPEN_DEG / 360)
#define RUN_HZ           800
#define RUN_ACCEL        400
#define HOME_HZ          200
#define CLOSE_DELAY_MS   3000         // radar-clear hold before closing
#define CLOSED_DWELL_MS  2000         // timed test: shut dwell before auto-open
#define OPEN_HOLD_MS     2000         // timed test: open hold before close

// ── Pet-count pass detector timeouts (ms) ──
#define PASS_ARM_TIMEOUT   4000       // approached a side but never crossed beam
#define PASS_CROSS_TIMEOUT 3000       // entered beam but backed out same side
#define PASS_STUCK_TIMEOUT 8000       // lingered in the beam
#define PASS_COOLDOWN_MS   1500       // debounce between counted passes

// ── Wi-Fi telnet log ──
#if RADIO_USES_WIFI
  #define WIFI_SSID "HGN_Laptop"
  #define WIFI_PASS "Juanjy48"
  #define LOG_PORT  23
#endif

// CH340 console (UART0) and Wi-Fi telnet are independent log sinks and coexist:
// NetLog writes to both. The CH340 half is only enabled when the inside radar is
// NOT claiming UART0 — i.e. radar off => CH340 + telnet; radar on => telnet only.
#if !USE_RADAR_IN
  #define LOG_TO_SERIAL 1
#else
  #define LOG_TO_SERIAL 0
#endif

// ===========================================================================
//  Status LED — colour encodes the current stage so a hang freezes on it.
//  white=boot blue=Wi-Fi purple=drivers red=FATAL
//  green=CLOSED yellow=OPENING cyan=OPEN orange=CLOSING (red flash=anti-pinch)
// ===========================================================================
static inline void led(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_LED, r, g, b); }

// ===========================================================================
//  Hardware objects
// ===========================================================================
TMC2130Stepper driver(PIN_M1_CS, R_SENSE);
TMC2130Stepper driver2(PIN_M2_CS, R_SENSE);
FastAccelStepperEngine engine   = FastAccelStepperEngine();
FastAccelStepper      *stepper  = nullptr;
FastAccelStepper      *stepper2 = nullptr;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(-1, -1);
HardwareSerial RadarSerialIn(0);    // UART0 -> mmWave1 (inside, LD2450)
HardwareSerial RadarSerialOut(1);   // UART1 -> mmWave2 (outside, LD2410)

// ===========================================================================
//  Logging — Print sink with a replayable ring buffer.
//  Streams to a telnet client (Wi-Fi builds) and/or the CH340 console.
// ===========================================================================
#if RADIO_USES_WIFI
WiFiServer logServer(LOG_PORT);
WiFiClient logClient;
#endif

class NetLog : public Print {
  static const size_t CAP = 2048;            // backlog bytes (replayed on connect)
  char   ring[CAP];
  size_t head    = 0;
  bool   wrapped = false;
public:
  size_t write(uint8_t b) override {
    ring[head++] = (char)b;
    if (head >= CAP) { head = 0; wrapped = true; }
#if LOG_TO_SERIAL
    Serial.write(b);
#endif
#if RADIO_USES_WIFI
    if (logClient && logClient.connected()) logClient.write(b);
#endif
    return 1;
  }
  size_t write(const uint8_t *p, size_t n) override {
    for (size_t i = 0; i < n; i++) write(p[i]);
    return n;
  }
#if RADIO_USES_WIFI
  void replayBacklog(WiFiClient &c) {
    if (wrapped) c.write((const uint8_t *)ring + head, CAP - head);
    c.write((const uint8_t *)ring, head);
  }
#endif
} Log;

#if RADIO_USES_WIFI
// Accept a single telnet client and replay the boot backlog to it.
static void handleLog() {
  if (logServer.hasClient()) {
    WiFiClient c = logServer.accept();
    if (logClient && logClient.connected()) c.stop();      // one client only
    else { logClient = c; logClient.setNoDelay(true); Log.replayBacklog(logClient); }
  }
}
#else
static inline void handleLog() {}
#endif

// ===========================================================================
//  Radar parsing — supports LD2450 (per-target list) and LD2410 (state report).
//  Both default to 256000 baud. Output of interest is binary presence plus,
//  for the LD2450, an active-target count.
// ---------------------------------------------------------------------------
//  LD2450 frame: AA FF 03 00 | T1(8) T2(8) T3(8) | 55 CC                = 30 B
//  LD2410 frame: F4 F3 F2 F1 | len(2 LE) | payload | F8 F7 F6 F5
//                payload[0]=type(0x02 target) [1]=0xAA [2]=target state
// ===========================================================================
enum RadarType : uint8_t { RADAR_LD2450, RADAR_LD2410 };

// LD2410 streaming parser phases.
enum { RP_HDR, RP_LEN_LO, RP_LEN_HI, RP_DATA, RP_FTR };

struct Radar {
  HardwareSerial *ser  = nullptr;
  const char     *name = "";
  RadarType       type = RADAR_LD2450;

  // streaming parser state (shared fields, used per type)
  uint8_t  buf[40];
  uint8_t  idx     = 0;
  uint8_t  sync    = 0;          // header bytes matched
  uint8_t  ftrSync = 0;          // footer bytes matched (LD2410)
  uint8_t  phase   = RP_HDR;     // LD2410 phase
  uint16_t need    = 0;          // LD2410 payload length
  uint8_t  lenLo   = 0;
  bool     inFrame = false;      // LD2450

  // decoded outputs
  uint8_t  targets = 0;          // active targets (LD2450 0..3, LD2410 0/1)
  bool     present = false;
  uint32_t good = 0, bad = 0;
  uint32_t lastFrameMs = 0;
};

Radar radarIn  = {};   // inside  (LD2450)
Radar radarOut = {};   // outside (LD2410)

static const uint8_t LD2450_HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t LD2410_HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t LD2410_FTR[4] = {0xF8, 0xF7, 0xF6, 0xF5};

// LD2450: count slots whose 8-byte target block is non-zero.
static void parse2450(Radar &r) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t *p = &r.buf[4 + i * 8];
    for (uint8_t k = 0; k < 8; k++) if (p[k]) { n++; break; }
  }
  r.targets = n;
  r.present = (n > 0);
}

static void feed2450(Radar &r, uint8_t b) {
  if (!r.inFrame) {
    if (b == LD2450_HDR[r.sync]) {
      r.buf[r.sync++] = b;
      if (r.sync == 4) { r.inFrame = true; r.idx = 4; r.sync = 0; }
    } else {
      r.sync = (b == LD2450_HDR[0]) ? 1 : 0;
      if (r.sync == 1) r.buf[0] = b;
    }
  } else {
    if (r.idx < sizeof(r.buf)) r.buf[r.idx++] = b;
    if (r.idx == 30) {
      r.inFrame = false; r.idx = 0;
      if (r.buf[28] == 0x55 && r.buf[29] == 0xCC) { parse2450(r); r.good++; }
      else r.bad++;
    }
  }
}

// LD2410: presence = target state byte != 0 (1 moving, 2 stationary, 3 both).
static void parse2410(Radar &r) {
  if (r.need >= 3 && r.buf[1] == 0xAA) {
    uint8_t st = r.buf[2];
    r.present = (st != 0x00);
    r.targets = r.present ? 1 : 0;
  }
}

static void feed2410(Radar &r, uint8_t b) {
  switch (r.phase) {
    case RP_HDR:
      if (b == LD2410_HDR[r.sync]) { if (++r.sync == 4) { r.sync = 0; r.phase = RP_LEN_LO; } }
      else                         { r.sync = (b == LD2410_HDR[0]) ? 1 : 0; }
      break;
    case RP_LEN_LO:
      r.lenLo = b; r.phase = RP_LEN_HI;
      break;
    case RP_LEN_HI:
      r.need = (uint16_t)r.lenLo | ((uint16_t)b << 8);
      if (r.need == 0 || r.need > sizeof(r.buf)) { r.bad++; r.phase = RP_HDR; }
      else                                       { r.idx = 0; r.phase = RP_DATA; }
      break;
    case RP_DATA:
      r.buf[r.idx++] = b;
      if (r.idx >= r.need) { r.ftrSync = 0; r.phase = RP_FTR; }
      break;
    case RP_FTR:
      if (b == LD2410_FTR[r.ftrSync]) {
        if (++r.ftrSync == 4) { parse2410(r); r.good++; r.phase = RP_HDR; }
      } else {
        r.bad++; r.phase = RP_HDR;
        r.sync = (b == LD2410_HDR[0]) ? 1 : 0;   // resync if a header byte slipped in
      }
      break;
  }
}

static void radarFeed(Radar &r, uint8_t b) {
  (r.type == RADAR_LD2450) ? feed2450(r, b) : feed2410(r, b);
}

// Drain the UART; stamp lastFrameMs whenever a valid frame completes.
static void radarPump(Radar &r, uint32_t now) {
  if (!r.ser) return;
  uint32_t before = r.good;
  while (r.ser->available()) radarFeed(r, (uint8_t)r.ser->read());
  if (r.good != before) r.lastFrameMs = now;
}

// Fresh presence: latched state is only trusted if a frame arrived recently.
static bool radarPresent(const Radar &r, uint32_t now) {
  return r.present && (now - r.lastFrameMs < RADAR_STALE_MS);
}

// ===========================================================================
//  Stepper drivers
// ===========================================================================
// StealthChop (quiet) is the chosen mode. StallGuard load/stall detection is
// intentionally NOT used — it requires SpreadCycle and cannot coexist with
// StealthChop. Anti-pinch is handled by the ToF instead.
void configDriver(TMC2130Stepper &d, uint16_t rms, const char *tag) {
  d.begin();
  d.rms_current(rms, IHOLD_MULT);    // IRUN = rms, IHOLD = IHOLD_MULT × rms
  d.microsteps(MICROSTEPS);
  d.en_pwm_mode(true);               // StealthChop
  d.pwm_autoscale(true);

  uint8_t ver = d.version();
  Log.print(tag); Log.print(" version: 0x"); Log.println(ver, HEX);
  if (ver != 0x11) {
    led(60, 0, 0);                   // red: fatal — bad driver version
    Log.print(tag); Log.println(" ERROR: bad version — check wiring/CS!");
    while (1) delay(10);
  }
  Log.print(tag); Log.print(" en_pwm_mode="); Log.print(d.en_pwm_mode());
  Log.print(" autoscale=");          Log.print(d.pwm_autoscale());
  Log.print(" TOFF=");               Log.println(d.toff());
}

// ===========================================================================
//  Door — two mirrored leaves driven as a single aperture.
// ===========================================================================
int32_t doorTarget = 0;

void doorMoveTo(int32_t pos) {
  doorTarget = pos;
  stepper->moveTo(pos);
  stepper2->moveTo(-pos);            // leaf 2 mirrored (back-to-back mount)
}
bool doorRunning()  { return stepper->isRunning() || stepper2->isRunning(); }
bool doorAtTarget() {
  return stepper->getCurrentPosition()  ==  doorTarget &&
         stepper2->getCurrentPosition() == -doorTarget;
}

// A leaf is shut if its (enabled) switch reads closed, else by commanded pos 0.
bool leaf1Shut() {
#if USE_END_STOP1
  return digitalRead(PIN_END_STOP1) == END_STOP_ACTIVE;
#else
  return stepper->getCurrentPosition() == 0;
#endif
}
bool leaf2Shut() {
#if USE_END_STOP2
  return digitalRead(PIN_END_STOP2) == END_STOP_ACTIVE;
#else
  return stepper2->getCurrentPosition() == 0;
#endif
}
bool doorShut() { return leaf1Shut() && leaf2Shut(); }

// Poll ToF; true = something inside the clear path (blocked beam).
bool obstructed() {
#if !USE_TOF
  return false;                      // no sensor: anti-pinch / counting disabled
#else
  static bool last = false;
  if (vl53.dataReady()) {
    int16_t d = vl53.distance();
    vl53.clearInterrupt();
    if (d != -1) last = (d > TOF_BLIND_MM && d < TOF_DETECT_MM);
  }
  return last;
#endif
}

// Home each enabled leaf to its end-stop; otherwise assume shut at pos 0.
void homeDoor() {
#if USE_END_STOP1 || USE_END_STOP2
  Log.println("Homing to end-stop(s)...");
  stepper->setSpeedInHz(HOME_HZ);      stepper2->setSpeedInHz(HOME_HZ);
  stepper->setAcceleration(RUN_ACCEL); stepper2->setAcceleration(RUN_ACCEL);

  bool done1 = true, done2 = true;
  #if USE_END_STOP1
    done1 = leaf1Shut();
    if (!done1) stepper->runBackward();  else stepper->setCurrentPosition(0);
  #else
    stepper->setCurrentPosition(0);
  #endif
  #if USE_END_STOP2
    done2 = leaf2Shut();
    if (!done2) stepper2->runForward(); else stepper2->setCurrentPosition(0);
  #else
    stepper2->setCurrentPosition(0);
  #endif

  uint32_t t0 = millis();
  while (!(done1 && done2) && millis() - t0 < HOME_TIMEOUT_MS) {
  #if USE_END_STOP1
    if (!done1 && leaf1Shut()) { stepper->forceStopAndNewPosition(0);  done1 = true; }
  #endif
  #if USE_END_STOP2
    if (!done2 && leaf2Shut()) { stepper2->forceStopAndNewPosition(0); done2 = true; }
  #endif
    delay(2);
  }
  if (!done1) stepper->forceStopAndNewPosition(0);
  if (!done2) stepper2->forceStopAndNewPosition(0);
  Log.println(done1 && done2 ? "Homed (shut)." : "Home TIMEOUT — forced pos=0.");
#else
  stepper->setCurrentPosition(0); stepper2->setCurrentPosition(0);
  Log.println("No end-stops: assume shut, pos=0.");
#endif
}

// ===========================================================================
//  Door operating mode (driven by the Zigbee Door Lock cluster)
// ===========================================================================
enum DoorMode : uint8_t {
  MODE_UNLOCKED = 0,    // open for either direction
  MODE_IN_ONLY  = 1,    // only let pets in  (outside radar opens)
  MODE_OUT_ONLY = 2,    // only let pets out (inside radar opens)
  MODE_LOCKED   = 3,    // never open; hold shut
};
volatile DoorMode doorMode = MODE_UNLOCKED;

const char *modeName(DoorMode m) {
  switch (m) {
    case MODE_IN_ONLY:  return "IN-ONLY";
    case MODE_OUT_ONLY: return "OUT-ONLY";
    case MODE_LOCKED:   return "LOCKED";
    default:            return "UNLOCKED";
  }
}

// May the door open for the current presence, given the lock mode?
bool openRequested(bool inPresent, bool outPresent) {
  switch (doorMode) {
    case MODE_LOCKED:   return false;
    case MODE_IN_ONLY:  return outPresent;             // pet outside wants in
    case MODE_OUT_ONLY: return inPresent;              // pet inside wants out
    case MODE_UNLOCKED:
    default:            return inPresent || outPresent;
  }
}

// ===========================================================================
//  Pet-count pass detector
//  Valid pass = one side's radar -> ToF beam crossed -> the OTHER side's radar,
//  all within timeouts. Direction adjusts the running occupancy count.
// ===========================================================================
enum PassState : uint8_t { PASS_IDLE, PASS_ARMED, PASS_CROSSING };
enum PassDir   : uint8_t { DIR_NONE, DIR_IN, DIR_OUT };   // IN = outside->inside

struct PetCounter {
  PassState state = PASS_IDLE;
  PassDir   dir   = DIR_NONE;
  uint32_t  tMark = 0;
  uint32_t  cooldownUntil = 0;
  int16_t   inside   = 0;            // pets currently inside (clamped >= 0)
  uint32_t  totalIn  = 0;
  uint32_t  totalOut = 0;
} pets;

static void petReset() { pets.state = PASS_IDLE; pets.dir = DIR_NONE; }

static void petCommit(uint32_t now) {
  if (pets.dir == DIR_IN)  { pets.inside++;                       pets.totalIn++;  }
  else                     { if (pets.inside > 0) pets.inside--;  pets.totalOut++; }
  Log.printf("PET %s -> inside=%d (in=%lu out=%lu)\n",
             pets.dir == DIR_IN ? "IN" : "OUT",
             pets.inside, pets.totalIn, pets.totalOut);
  pets.cooldownUntil = now + PASS_COOLDOWN_MS;
  petReset();
}

// Returns true when the occupancy count changed this tick.
static bool petUpdate(bool inP, bool outP, bool blocked, uint32_t now) {
  int16_t before = pets.inside;
  switch (pets.state) {
    case PASS_IDLE:
      if (now < pets.cooldownUntil) break;
      if      (outP && !inP) { pets.dir = DIR_IN;  pets.state = PASS_ARMED; pets.tMark = now; }
      else if (inP && !outP) { pets.dir = DIR_OUT; pets.state = PASS_ARMED; pets.tMark = now; }
      break;

    case PASS_ARMED:                                   // wait for the beam to break
      if (blocked)                              { pets.state = PASS_CROSSING; pets.tMark = now; }
      else if (now - pets.tMark > PASS_ARM_TIMEOUT) petReset();
      break;

    case PASS_CROSSING:                                // beam broken: wait for far side
      if (!blocked) {
        bool farSeen = (pets.dir == DIR_IN) ? inP : outP;
        if (farSeen)                                   petCommit(now);
        else if (now - pets.tMark > PASS_CROSS_TIMEOUT) petReset();   // backed out
      } else if (now - pets.tMark > PASS_STUCK_TIMEOUT) petReset();   // lingering
      break;
  }
  return pets.inside != before;
}

// ===========================================================================
//  Zigbee — Door Lock (custom EP) + Occupancy + Analog pet count
// ===========================================================================
#if RADIO_USES_ZIGBEE

#define ZB_EP_DOORLOCK   10
#define ZB_EP_OCCUPANCY  11
#define ZB_EP_PETCOUNT   12

// Manufacturer-specific enum on the Door Lock cluster carrying the 4 operating
// modes. Standard Lock/Unlock maps onto LOCKED/UNLOCKED; this attribute adds the
// directional IN-ONLY / OUT-ONLY modes the standard cluster has no slot for.
#define PETDOOR_ATTR_MODE 0xF000

// ZCL Door Lock "LockState" attribute values (the SDK exposes no enum for these).
#define DOORLOCK_STATE_LOCKED   1
#define DOORLOCK_STATE_UNLOCKED 2

// Door Lock endpoint built directly on the Door Lock (0x0101) cluster, since the
// Arduino-ESP32 Zigbee library ships no door-lock wrapper class.
class ZigbeePetDoorLock : public ZigbeeEP {
public:
  explicit ZigbeePetDoorLock(uint8_t endpoint) : ZigbeeEP(endpoint) {
    _device_id = ESP_ZB_HA_DOOR_LOCK_DEVICE_ID;

    esp_zb_door_lock_cfg_t lock_cfg = ESP_ZB_DEFAULT_DOOR_LOCK_CONFIG();
    esp_zb_cluster_list_t *cluster_list = esp_zb_door_lock_clusters_create(&lock_cfg);

    // Add the manufacturer-specific mode enum to the Door Lock server cluster.
    esp_zb_attribute_list_t *lock_cluster = esp_zb_cluster_list_get_cluster(
        cluster_list, ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_add_attr(
        lock_cluster, ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK, PETDOOR_ATTR_MODE,
        ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &_mode);

    _cluster_list = cluster_list;
    _ep_config = {
      .endpoint           = endpoint,
      .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id      = ESP_ZB_HA_DOOR_LOCK_DEVICE_ID,
      .app_device_version = 0,
    };
  }

  void setModeCallback(void (*cb)(uint8_t)) { _modeCb = cb; }

  // Push the current mode (and a coherent lock state) to the coordinator.
  void publishMode(uint8_t mode) {
    _mode = mode;
    uint8_t lockState = (mode == MODE_LOCKED)
        ? DOORLOCK_STATE_LOCKED
        : DOORLOCK_STATE_UNLOCKED;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(_endpoint, ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, PETDOOR_ATTR_MODE, &_mode, false);
    esp_zb_zcl_set_attribute_val(_endpoint, ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID,
        &lockState, false);
    esp_zb_lock_release();
  }

private:
  void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override {
    if (message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK) return;
    if (message->attribute.id == PETDOOR_ATTR_MODE) {
      _mode = *(uint8_t *)message->attribute.data.value;
    } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID) {
      uint8_t st = *(uint8_t *)message->attribute.data.value;
      _mode = (st == DOORLOCK_STATE_LOCKED) ? MODE_LOCKED : MODE_UNLOCKED;
    } else {
      return;
    }
    if (_mode > MODE_LOCKED) _mode = MODE_LOCKED;
    if (_modeCb) _modeCb(_mode);
  }

  void (*_modeCb)(uint8_t) = nullptr;
  uint8_t _mode = MODE_UNLOCKED;
};

ZigbeePetDoorLock     zbLock(ZB_EP_DOORLOCK);
ZigbeeOccupancySensor zbOccupancy(ZB_EP_OCCUPANCY);
ZigbeeAnalog          zbPetCount(ZB_EP_PETCOUNT);

// Coordinator wrote a new mode -> apply locally (runs in the Zigbee task).
void onZigbeeMode(uint8_t mode) {
  doorMode = (DoorMode)mode;
  Log.printf("Zigbee: mode -> %s\n", modeName(doorMode));
}

void zigbeeSetup() {
  zbLock.setModeCallback(onZigbeeMode);

  zbOccupancy.setManufacturerAndModel("DIY", "SmartPetDoor");
  zbOccupancy.setOccupancy(false);

  zbPetCount.addAnalogInput();
  zbPetCount.setAnalogInput(0.0f);
  zbPetCount.setAnalogInputReporting(0, 30, 1.0f);   // report on change, ≤30s

  Zigbee.addEndpoint(&zbLock);
  Zigbee.addEndpoint(&zbOccupancy);
  Zigbee.addEndpoint(&zbPetCount);

  Log.println("Zigbee: starting end device...");
  if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
    led(60, 0, 0);
    Log.println("Zigbee: begin FAILED — rebooting");
    delay(1000);
    ESP.restart();
  }

  uint32_t t0 = millis();
  while (!Zigbee.connected() && millis() - t0 < 30000) { handleLog(); delay(100); }
  Log.println(Zigbee.connected() ? "Zigbee: connected." : "Zigbee: not joined (will retry).");

  zbLock.publishMode(doorMode);
}

// Mirror local state to the coordinator when it changes.
void zigbeePublish() {
  static int16_t  lastInside = -1;
  static DoorMode lastMode   = (DoorMode)0xFF;

  if (pets.inside != lastInside) {
    lastInside = pets.inside;
    zbOccupancy.setOccupancy(pets.inside > 0);
    zbPetCount.setAnalogInput((float)pets.inside);
    zbPetCount.reportAnalogInput();
  }
  if (doorMode != lastMode) {
    lastMode = doorMode;
    zbLock.publishMode(doorMode);
  }
}
#else
static inline void zigbeeSetup()   {}
static inline void zigbeePublish() {}
#endif  // RADIO_USES_ZIGBEE

// ===========================================================================
//  Door state machine
// ===========================================================================
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
DoorState state         = CLOSED;
uint32_t  lastPresentMs = 0;
uint32_t  stateMs       = 0;

void setState(DoorState s) {
  state = s; stateMs = millis();
  switch (s) {
    case CLOSED:  led(0, 40, 0);  break;   // green
    case OPENING: led(40, 30, 0); break;   // yellow
    case OPEN:    led(0, 30, 40); break;   // cyan
    case CLOSING: led(50, 15, 0); break;   // orange
  }
}
const char *stateName(DoorState s) {
  switch (s) {
    case CLOSED:  return "CLOSED";
    case OPENING: return "OPENING";
    case OPEN:    return "OPEN";
    default:      return "CLOSING";
  }
}

// ===========================================================================
//  Setup
// ===========================================================================
void setup() {
  led(20, 20, 20);                   // white: boot start
#if LOG_TO_SERIAL
  Serial.begin(115200);              // CH340 console (only when inside radar off)
#endif
  delay(500);

#if RADIO_USES_WIFI
  led(0, 0, 40);                     // blue: Wi-Fi connecting
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("petdoor");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(100);
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin("petdoor");
    logServer.begin();
    logServer.setNoDelay(true);
    Log.print("Wi-Fi up. telnet ");
    Log.print(WiFi.localIP());
    Log.println(":23  (or petdoor.local:23)");
  } else {
    Log.println("Wi-Fi FAILED — no telnet log this boot");
  }
#endif

  Log.println("\n--- Smart Pet Door (ESP32-C6) ---");
  Log.printf("Radio mode: %d  | radars in=%d out=%d | end-stops %d/%d | ToF %d\n",
             RADIO_MODE, USE_RADAR_IN, USE_RADAR_OUT, USE_END_STOP1, USE_END_STOP2, USE_TOF);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);         // enable drivers (held for holding torque)
#if USE_END_STOP1
  pinMode(PIN_END_STOP1, INPUT_PULLUP);
#endif
#if USE_END_STOP2
  pinMode(PIN_END_STOP2, INPUT_PULLUP);
#endif

  // Stepper drivers
  led(30, 0, 30);                    // purple: configuring drivers
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  configDriver(driver,  RMS_CURRENT,  "D1");
  configDriver(driver2, RMS_CURRENT2, "D2");

  engine.init();
  stepper  = engine.stepperConnectToPin(PIN_M1_STEP);
  stepper2 = engine.stepperConnectToPin(PIN_M2_STEP);
  if (!stepper || !stepper2) {
    led(60, 0, 0); Log.println("ERROR: stepper init!"); while (1) delay(10);
  }
  stepper->setDirectionPin(PIN_M1_DIR);
  stepper2->setDirectionPin(PIN_M2_DIR);

  // ToF
#if USE_TOF
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!vl53.begin(0x29, &Wire)) {
    led(60, 0, 0); Log.println("ERROR: VL53L1X init!"); while (1) delay(10);
  }
  vl53.startRanging();
  vl53.setTimingBudget(50);
  vl53.clearInterrupt();
#else
  Log.println("ToF disabled — anti-pinch and pet counting off");
#endif

  // Radars
  radarIn.ser  = &RadarSerialIn;  radarIn.name  = "IN";  radarIn.type  = RADAR_LD2450;
  radarOut.ser = &RadarSerialOut; radarOut.name = "OUT"; radarOut.type = RADAR_LD2410;
#if USE_RADAR_IN
  RadarSerialIn.begin(RADAR_IN_BAUD,   SERIAL_8N1, PIN_R1_RX, PIN_R1_TX);
#else
  radarIn.ser = nullptr;
#endif
#if USE_RADAR_OUT
  RadarSerialOut.begin(RADAR_OUT_BAUD, SERIAL_8N1, PIN_R2_RX, PIN_R2_TX);
#else
  radarOut.ser = nullptr;
#endif

  homeDoor();
  stepper->setSpeedInHz(RUN_HZ);       stepper2->setSpeedInHz(RUN_HZ);
  stepper->setAcceleration(RUN_ACCEL); stepper2->setAcceleration(RUN_ACCEL);
  setState(CLOSED);

  zigbeeSetup();

#if TEST_TIMED_CYCLE
  Log.println("Ready. TIMED test cycle (radars ignored).");
#else
  Log.println("Ready. Radar-driven.");
#endif
}

// ===========================================================================
//  Main loop
// ===========================================================================
void loop() {
  uint32_t now = millis();
  handleLog();

  radarPump(radarIn,  now);
  radarPump(radarOut, now);

  bool inP    = radarPresent(radarIn,  now);
  bool outP   = radarPresent(radarOut, now);
  bool block  = obstructed();

  // Pet counting (independent of the door FSM; needs the ToF beam).
  petUpdate(inP, outP, block, now);
  zigbeePublish();   // pushes occupancy/count and mode changes when they differ

#if TEST_TIMED_CYCLE
  bool wantOpen = false;
#else
  bool wantOpen = openRequested(inP, outP);
  if (inP || outP) lastPresentMs = now;
#endif

  switch (state) {
    case CLOSED:
#if TEST_TIMED_CYCLE
      if (now - stateMs >= CLOSED_DWELL_MS) {
        Log.println("Timer -> OPEN"); doorMoveTo(OPEN_STEPS); setState(OPENING);
      }
#else
      if (wantOpen) {
        Log.printf("Radar(%s) -> OPEN\n", modeName(doorMode));
        doorMoveTo(OPEN_STEPS); setState(OPENING);
      }
#endif
      break;

    case OPENING:
      if (doorAtTarget()) setState(OPEN);
      break;

    case OPEN:
#if TEST_TIMED_CYCLE
      if (now - stateMs >= OPEN_HOLD_MS) {
        Log.println("Timer -> CLOSING"); doorMoveTo(0); setState(CLOSING);
      }
#else
      // Close when the lock mode forbids opening, or the relevant side is clear.
      if (doorMode == MODE_LOCKED ||
          (!wantOpen && now - lastPresentMs >= CLOSE_DELAY_MS)) {
        Log.println("Clear -> CLOSING"); doorMoveTo(0); setState(CLOSING);
      }
#endif
      break;

    case CLOSING:
      if (obstructed()) {                                  // anti-pinch: reopen
        led(80, 0, 0);
        Log.println("ToF obstruction -> reopen");
        doorMoveTo(OPEN_STEPS); setState(OPENING);
        break;
      }
#if USE_END_STOP1
      if (leaf1Shut() && stepper->isRunning())  stepper->forceStopAndNewPosition(0);
#endif
#if USE_END_STOP2
      if (leaf2Shut() && stepper2->isRunning()) stepper2->forceStopAndNewPosition(0);
#endif
      if (doorShut() || doorAtTarget()) {
        Log.println("Closed."); setState(CLOSED);
      }
      break;
  }

  // 1 Hz debug
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 1000) {
    Log.printf("[%s|%s] IN=%u(bad%lu) OUT=%u(bad%lu) inside=%d\n",
               stateName(state), modeName(doorMode),
               radarIn.targets, radarIn.bad, radarOut.targets, radarOut.bad,
               pets.inside);
    radarIn.bad = radarOut.bad = 0;
    lastDbg = now;
  }
}
