//Silence — automatic animal door (ESP32-C6, smart-home target)
//  mmWave LD2450 detects animal -> open (OPEN_DEG)
//  radar clear + delay          -> close
//  door switch (when wired)     -> CLOSED (ground truth)
//  ToF obstruction while closing-> reverse / reopen (anti-pinch)
//
//  Two radars: inside (mmWave1, UART0) + outside (mmWave2, UART1).
//  RADAR_SEL picks which drives the door (0=timed test, 1=inside, 2=outside).
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// ── SPI / enable (shared) — C6 pinout ────────────────────
#define SCK_PIN     19
#define MISO_PIN    12
#define MOSI_PIN    20
#define EN_PIN      21          // shared active-low enable (both drivers)

// Driver 1 (MOTOR1)
#define STEP_PIN    11
#define DIR_PIN     10
#define CS_PIN      18

// Driver 2 (MOTOR2)
#define STEP2_PIN   23
#define DIR2_PIN    22
#define CS2_PIN      3

// ── Door switch ──────────────────────────────────────────
// Not on the C6 schematic yet. GPIO1 reserved for it. Disabled until wired.
#define USE_DOOR_SWITCH 0
#define DOOR_PIN    1           // placeholder (unused while USE_DOOR_SWITCH 0)
#define DOOR_SHUT   LOW

// ── ToF (VL53L1X) ────────────────────────────────────────
#define TOF_SDA     6
#define TOF_SCL     7
#define TOF_INT     0           // sensor IRQ available; polling used (unused here)
#define TOF_BLIND_MM     40     // ignore <40mm blindspot
#define TOF_DETECT_MM   350     // 38cm mount: <350mm (and >blind) = obstruction

// ── Radars (two LD2450, HardwareSerial) ──────────────────
// 0 = timed test cycle (no radar). 1 = inside drives door. 2 = outside.
#define RADAR_SEL   0
#define RADAR_BAUD  256000      // factory; if a module set to 115200, change here
#define R1_RX       17          // mmWave1 (inside)  TX → ESP RX (UART0)
#define R1_TX       16          // mmWave1 (inside)  RX ← ESP TX
#define R2_RX        4          // mmWave2 (outside) TX → ESP RX (UART1)
#define R2_TX        5          // mmWave2 (outside) RX ← ESP TX

// ── Driver current ───────────────────────────────────────
#define R_SENSE     0.11f
#define RMS_CURRENT  800        // IRUN (moving)
#define RMS_CURRENT2 800
// Backdrivable drivetrain: keep drivers ENABLED at standstill so the door
// resists being pushed open (security), but cut the standstill hold current to
// kill the idle heat / 0.5A draw. Hold = IHOLD_MULT × RMS. Lower = cooler but
// weaker hold; raise if the door can be shoved open.
#define IHOLD_MULT  0.3f

// ── Geometry / motion ────────────────────────────────────
#define MICROSTEPS  16
#define STEPS_PER_REV (200 * MICROSTEPS)
#define OPEN_DEG    180                              // door open angle (degrees)
#define OPEN_STEPS  ((int32_t)STEPS_PER_REV * OPEN_DEG / 360)
#define RUN_HZ      800
#define RUN_ACCEL   400
#define HOME_HZ     200
#define CLOSE_DELAY_MS  3000    // radar-clear hold before closing
#define CLOSED_DWELL_MS 2000    // timed mode: shut dwell before auto-open
#define OPEN_HOLD_MS    2000    // timed mode: open hold before close

// ── Objects ──────────────────────────────────────────────
TMC2130Stepper driver(CS_PIN, R_SENSE);
TMC2130Stepper driver2(CS2_PIN, R_SENSE);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper  = NULL;
FastAccelStepper *stepper2 = NULL;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(-1, -1);
HardwareSerial RadarSerialIn(0);          // UART0 → mmWave1 (inside)
HardwareSerial RadarSerialOut(1);         // UART1 → mmWave2 (outside)

// ===========================================================================
//  LD2450 frame parser (HW-UART, presence only).
//  Frame: AA FF 03 00 | T1(8) T2(8) T3(8) | 55 CC = 30 bytes. Zero slot = none.
// ===========================================================================
struct Radar2450 {
  HardwareSerial *ser;
  const char *name;
  uint8_t  buf[30];
  uint8_t  idx, hdr;
  bool     inFrame;
  uint8_t  activeCount;
  uint32_t good, bad;
};
Radar2450 radarIn  = {};
Radar2450 radarOut = {};

static const uint8_t LD45_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};

static void parse2450(Radar2450 &r) {
  r.activeCount = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t *p = &r.buf[4 + i * 8];
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

static void pumpRadar(Radar2450 &r) {
  while (r.ser->available()) feed2450(r, (uint8_t)r.ser->read());
}

// ===========================================================================
//  Driver setup. StealthChop, NO boot creep (would rotate door). Low IHOLD.
// ===========================================================================
void configDriver(TMC2130Stepper &d, uint16_t rms, const char *tag) {
  d.begin();
  d.rms_current(rms, IHOLD_MULT);   // IRUN = rms, IHOLD = IHOLD_MULT × rms
  d.microsteps(MICROSTEPS);
  d.en_pwm_mode(true);              // StealthChop
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
int32_t doorTarget = 0;
void doorMoveTo(int32_t pos) {
  doorTarget = pos;
  stepper->moveTo(pos);
  stepper2->moveTo(-pos);
}
bool doorRunning() { return stepper->isRunning() || stepper2->isRunning(); }
// Arrived = position == commanded target. Race-proof (start pos != target).
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

// TODO(optional): clamp/stall detection (StallGuard2 or external shunt) — not on
// the current PCB. Hook point: sample load while CLOSING, halt+reopen on spike.

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
  stepper->setCurrentPosition(0); stepper2->setCurrentPosition(0);
  Serial.println("No switch: assume shut, pos=0.");
#endif
}

// ── Door state machine ───────────────────────────────────
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
DoorState state = CLOSED;
uint32_t  lastPresentMs = 0;
uint32_t  stateMs = 0;

void setState(DoorState s) { state = s; stateMs = millis(); }
const char *stateName(DoorState s) {
  switch (s) { case CLOSED: return "CLOSED"; case OPENING: return "OPENING";
               case OPEN: return "OPEN"; default: return "CLOSING"; }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Auto door (ESP32-C6): dual TMC2130 + LD2450 x2 + ToF ---");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);          // enable drivers (stay enabled: holding torque)
#if USE_DOOR_SWITCH
  pinMode(DOOR_PIN, INPUT);
#endif

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

  // Radars
  radarIn.ser  = &RadarSerialIn;  radarIn.name  = "IN";
  radarOut.ser = &RadarSerialOut; radarOut.name = "OUT";
#if RADAR_SEL > 0
  RadarSerialIn.begin(RADAR_BAUD,  SERIAL_8N1, R1_RX, R1_TX);
  RadarSerialOut.begin(RADAR_BAUD, SERIAL_8N1, R2_RX, R2_TX);
#endif

  homeDoor();
  stepper->setSpeedInHz(RUN_HZ);  stepper2->setSpeedInHz(RUN_HZ);
  stepper->setAcceleration(RUN_ACCEL); stepper2->setAcceleration(RUN_ACCEL);
  setState(CLOSED);

#if RADAR_SEL == 1
  Serial.println("Ready. Trigger: INSIDE radar.");
#elif RADAR_SEL == 2
  Serial.println("Ready. Trigger: OUTSIDE radar.");
#else
  Serial.println("Ready. Timed cycle (radars off).");
#endif
}

void loop() {
  uint32_t now = millis();

#if RADAR_SEL > 0
  pumpRadar(radarIn);
  pumpRadar(radarOut);
  #if RADAR_SEL == 1
    bool present = (radarIn.activeCount > 0);
  #else
    bool present = (radarOut.activeCount > 0);
  #endif
  if (present) lastPresentMs = now;
#endif

  switch (state) {
    case CLOSED:
#if RADAR_SEL > 0
      if (present) { Serial.println("Radar → OPEN"); doorMoveTo(OPEN_STEPS); setState(OPENING); }
#else
      if (now - stateMs >= CLOSED_DWELL_MS) { Serial.println("Timer → OPEN"); doorMoveTo(OPEN_STEPS); setState(OPENING); }
#endif
      break;

    case OPENING:
      if (doorAtTarget()) setState(OPEN);
      break;

    case OPEN:
#if RADAR_SEL > 0
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
#if RADAR_SEL > 0
    Serial.printf("[%s] IN=%u(bad%u) OUT=%u(bad%u)\n", stateName(state),
                  radarIn.activeCount, radarIn.bad, radarOut.activeCount, radarOut.bad);
    radarIn.good = radarIn.bad = radarOut.good = radarOut.bad = 0;
#else
    Serial.printf("[%s] timed\n", stateName(state));
#endif
    lastDbg = now;
  }
}
