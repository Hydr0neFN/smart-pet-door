//Silence — automatic animal door
//  LD2450 radar detects animal  -> open (90°)
//  radar clear + delay          -> close
//  door switch trips (shut)     -> CLOSED (ground truth)
//  ToF sees obstruction closing -> reverse / reopen (anti-pinch)
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// ── SPI / enable (shared) ────────────────────────────────
#define SCK_PIN     18
#define MISO_PIN    19
#define MOSI_PIN    23
#define EN_PIN      27          // shared active-low enable

// Driver 1
#define STEP_PIN    25
#define DIR_PIN     26
#define CS_PIN       5

// Driver 2
#define STEP2_PIN   14
#define DIR2_PIN    13
#define CS2_PIN     22

// ── Door switch ──────────────────────────────────────────
// GPIO34 input-only — EXTERNAL pull-up: 3V3 ─R─ D34, switch shorts D34→GND.
// Door open / switch released → HIGH. Door shut actuates switch → LOW.
#define DOOR_PIN    34
#define DOOR_SHUT   HIGH         // pin level when door physically closed

// ── ToF (VL53L1X) ────────────────────────────────────────
#define TOF_SDA     21
#define TOF_SCL     33
// Sensor 38 cm (380 mm) above clear surface, aimed down. Object shortens read.
#define TOF_BLIND_MM     40     // ignore <40mm blindspot
#define TOF_DETECT_MM   350     // <350mm (and >blind) = obstruction in doorway

// ── LD2450 radar (HardwareSerial UART2) ──────────────────
#define RADAR_RX    35          // sensor TX → ESP RX (GPIO35 input-only: OK for RX; 34 = door)
#define RADAR_TX    32          // ESP TX → sensor RX (config only)
// Factory default 256000. If this module was set to 115200 (mmWave project NVM),
// change to 115200. Watch the `bad:`/fps debug line if no detection.
#define RADAR_BAUD  256000

// ── Driver config ────────────────────────────────────────
#define R_SENSE     0.11f
#define RMS_CURRENT  800
#define RMS_CURRENT2 800

// ── Geometry / motion ────────────────────────────────────
#define MICROSTEPS  16
#define STEPS_PER_REV (200 * MICROSTEPS)
#define OPEN_DEG    180                              // door open angle (degrees)
#define OPEN_STEPS  ((int32_t)STEPS_PER_REV * OPEN_DEG / 360)  // open position
#define RUN_HZ      800
#define RUN_ACCEL   400
#define HOME_HZ     200                   // slow homing creep
#define CLOSE_DELAY_MS 3000               // radar-clear hold before closing

// ── Mode: radar trigger vs timed test cycle ──────────────
#define USE_RADAR        0                // 0 = timed auto open/close (radar off)
#define USE_DOOR_SWITCH  1                // 1 = switch stops the close at shut (LOW)
                                          //     + boot homing. Needs switch wired so
                                          //     door-open idles HIGH (else false shut).
#define CLOSED_DWELL_MS 2000              // wait shut this long, then auto-open
#define OPEN_HOLD_MS    2000              // stay open this long, then close

// ── Objects ──────────────────────────────────────────────
TMC2130Stepper driver(CS_PIN, R_SENSE);
TMC2130Stepper driver2(CS2_PIN, R_SENSE);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper  = NULL;
FastAccelStepper *stepper2 = NULL;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(-1, -1);
HardwareSerial RadarSerial(2);            // UART2, remapped to RADAR_RX/TX

// ===========================================================================
//  LD2450 frame parser (ported from mmWave project; HW-UART, presence only).
//  Frame: AA FF 03 00 | T1(8) T2(8) T3(8) | 55 CC = 30 bytes.
//  Target 8B: xLo xHi yLo yHi vLo vHi resLo resHi. All-zero slot = no target.
// ===========================================================================
struct Radar2450 {
  uint8_t  buf[30];
  uint8_t  idx, hdr;
  bool     inFrame;
  uint8_t  activeCount;     // valid targets in last frame
  uint32_t good, bad;       // frame stats (debug)
};
Radar2450 radar = {};

static const uint8_t LD45_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};

static void parse2450(Radar2450 &r) {
  r.activeCount = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t *p = &r.buf[4 + i * 8];
    // any non-zero byte in the slot = a tracked target
    bool valid = false;
    for (uint8_t k = 0; k < 8; k++) if (p[k]) { valid = true; break; }
    if (valid) r.activeCount++;
  }
}

static void feed2450(Radar2450 &r, uint8_t b) {
  if (!r.inFrame) {
    if (b == LD45_HEADER[r.hdr]) {
      r.buf[r.hdr++] = b;
      if (r.hdr == 4) { r.inFrame = true; r.idx = 4; r.hdr = 0; }
    } else {
      r.hdr = (b == LD45_HEADER[0]) ? 1 : 0;
      if (r.hdr == 1) r.buf[0] = b;
    }
  } else {
    r.buf[r.idx++] = b;
    if (r.idx == 30) {
      r.inFrame = false; r.idx = 0;
      if (r.buf[28] == 0x55 && r.buf[29] == 0xCC) { parse2450(r); r.good++; }
      else r.bad++;
    }
  }
}

static void pumpRadar() {
  while (RadarSerial.available()) feed2450(radar, (uint8_t)RadarSerial.read());
}

// ===========================================================================
//  Driver setup (StealthChop only — NO boot creep; would rotate the door).
//  PWM_AUTOSCALE converges over the first real door moves.
// ===========================================================================
void configDriver(TMC2130Stepper &d, uint16_t rms, const char *tag) {
  d.begin();
  d.rms_current(rms);
  d.microsteps(MICROSTEPS);
  d.en_pwm_mode(true);     // StealthChop
  d.pwm_autoscale(true);

  uint8_t ver = d.version();
  Serial.print(tag); Serial.print(" version: 0x"); Serial.println(ver, HEX);
  if (ver != 0x11) {
    Serial.print(tag); Serial.println(" ERROR: bad version — check wiring/CS!");
    while (1);
  }
  Serial.print(tag); Serial.print(" en_pwm_mode="); Serial.print(d.en_pwm_mode());
  Serial.print(" autoscale=");                      Serial.print(d.pwm_autoscale());
  Serial.print(" TOFF=");                           Serial.println(d.toff());
}

// Move door to absolute position (D2 mirrored: back-to-back mount).
// pos = OPEN_STEPS → open(90°); pos = 0 → closed.
int32_t doorTarget = 0;
void doorMoveTo(int32_t pos) {
  doorTarget = pos;
  stepper->moveTo(pos);
  stepper2->moveTo(-pos);
}
bool doorRunning() { return stepper->isRunning() || stepper2->isRunning(); }
// Arrived = position equals commanded target. Race-proof: at the moment a move
// is issued, currentPos != target, so this can't false-fire before motion runs.
bool doorAtTarget() {
  return stepper->getCurrentPosition() == doorTarget &&
         stepper2->getCurrentPosition() == -doorTarget;
}

// Poll ToF; true = obstruction shorter than clear 380mm path.
bool obstructed() {
  static bool last = false;
  if (vl53.dataReady()) {
    int16_t d = vl53.distance();
    vl53.clearInterrupt();
    if (d != -1) last = (d > TOF_BLIND_MM && d < TOF_DETECT_MM);
  }
  return last;
}

// Drive close-direction slowly until the door switch reports shut → zero ref.
void homeDoor() {
#if USE_DOOR_SWITCH
  Serial.println("Homing to door switch...");
  if (digitalRead(DOOR_PIN) == DOOR_SHUT) {
    stepper->setCurrentPosition(0); stepper2->setCurrentPosition(0);
    Serial.println("Already shut. pos=0.");
    return;
  }
  stepper->setSpeedInHz(HOME_HZ);  stepper2->setSpeedInHz(HOME_HZ);
  stepper->setAcceleration(RUN_ACCEL); stepper2->setAcceleration(RUN_ACCEL);
  stepper->runBackward();  stepper2->runForward();   // close direction (mirrored)
  while (digitalRead(DOOR_PIN) != DOOR_SHUT) delay(2);
  stepper->forceStopAndNewPosition(0);
  stepper2->forceStopAndNewPosition(0);
  Serial.println("Homed. pos=0 (shut).");
#else
  // No switch: trust door is shut at boot, set zero reference.
  stepper->setCurrentPosition(0); stepper2->setCurrentPosition(0);
  Serial.println("No switch: assume shut, pos=0.");
#endif
}

// ── Door state machine ───────────────────────────────────
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
DoorState state = CLOSED;
uint32_t  lastPresentMs = 0;
uint32_t  stateMs = 0;          // millis() when current state was entered

void setState(DoorState s) { state = s; stateMs = millis(); }

const char *stateName(DoorState s) {
  switch (s) { case CLOSED: return "CLOSED"; case OPENING: return "OPENING";
               case OPEN: return "OPEN"; default: return "CLOSING"; }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Auto door: dual TMC2130 + LD2450 + ToF ---");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);          // enable drivers
  pinMode(DOOR_PIN, INPUT);           // GPIO34 input-only; external pull-up on board

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  configDriver(driver,  RMS_CURRENT,  "D1");
  configDriver(driver2, RMS_CURRENT2, "D2");

  engine.init();
  stepper  = engine.stepperConnectToPin(STEP_PIN);
  stepper2 = engine.stepperConnectToPin(STEP2_PIN);
  if (!stepper || !stepper2) { Serial.println("ERROR: stepper init!"); while (1); }
  stepper->setDirectionPin(DIR_PIN);
  stepper2->setDirectionPin(DIR2_PIN);

  // ToF
  Wire.begin(TOF_SDA, TOF_SCL);
  if (!vl53.begin(0x29, &Wire)) { Serial.println("ERROR: VL53L1X init!"); while (1) delay(10); }
  vl53.startRanging();
  vl53.setTimingBudget(50);
  vl53.clearInterrupt();

  // Radar
#if USE_RADAR
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
#endif

  // Home, then set run speed for door travel.
  homeDoor();
  stepper->setSpeedInHz(RUN_HZ);  stepper2->setSpeedInHz(RUN_HZ);
  stepper->setAcceleration(RUN_ACCEL); stepper2->setAcceleration(RUN_ACCEL);
  setState(CLOSED);

#if USE_RADAR
  Serial.println("Ready. Waiting for radar.");
#else
  Serial.println("Ready. Timed cycle (radar off).");
#endif
}

void loop() {
  uint32_t now = millis();
#if USE_RADAR
  pumpRadar();                                  // drain UART every pass
  bool present = (radar.activeCount > 0);
  if (present) lastPresentMs = now;
#endif

  switch (state) {
    case CLOSED:
#if USE_RADAR
      if (present) { Serial.println("Radar → OPEN"); doorMoveTo(OPEN_STEPS); setState(OPENING); }
#else
      if (now - stateMs >= CLOSED_DWELL_MS) { Serial.println("Timer → OPEN"); doorMoveTo(OPEN_STEPS); setState(OPENING); }
#endif
      break;

    case OPENING:
      if (doorAtTarget()) setState(OPEN);
      break;

    case OPEN:
#if USE_RADAR
      if (!present && (now - lastPresentMs >= CLOSE_DELAY_MS)) {
        Serial.println("Clear → CLOSING"); doorMoveTo(0); setState(CLOSING);
      }
#else
      if (now - stateMs >= OPEN_HOLD_MS) {
        Serial.println("Timer → CLOSING"); doorMoveTo(0); setState(CLOSING);
      }
#endif
      break;

    case CLOSING:
      if (obstructed()) {                       // anti-pinch: reverse, reopen
        Serial.println("⚠️ ToF obstruction → reopen"); doorMoveTo(OPEN_STEPS); setState(OPENING);
#if USE_DOOR_SWITCH
      } else if (digitalRead(DOOR_PIN) == DOOR_SHUT) {
        stepper->forceStopAndNewPosition(0); stepper2->forceStopAndNewPosition(0);
        Serial.println("Switch shut → CLOSED"); setState(CLOSED);
#endif
      } else if (doorAtTarget()) {              // reached commanded 0
        Serial.println("Closed (position)"); setState(CLOSED);
      }
      break;
  }

  // 1 Hz debug
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 1000) {
#if USE_RADAR
    float fps = radar.good * 1000.0f / (now - lastDbg);
    Serial.printf("[%s] tgt=%u door=%d fps=%.0f bad=%u\n",
                  stateName(state), radar.activeCount, digitalRead(DOOR_PIN), fps, radar.bad);
    radar.good = 0; radar.bad = 0;
#else
    Serial.printf("[%s] door=%d\n", stateName(state), digitalRead(DOOR_PIN));
#endif
    lastDbg = now;
  }
}
