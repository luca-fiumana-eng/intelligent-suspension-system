/*****************************************************************************
 *  Main_Control_Logic.ino
 *  HTS interlock controller - Intelligent Suspension System, 3/4-wheel scooters
 *
 *  Target   : Arduino Mega 2560 (5 V)
 *  Author   : Luca Fiumana
 *  Revision : v2.0 (rewrite of the prototype sketch; see CHANGES at the end)
 *
 *  WHAT THIS FIRMWARE DOES
 *  -----------------------
 *  A DC motor (through an H-bridge) moves the HTS mechanism between two
 *  mechanical positions, OPEN and CLOSED. Two limit switches report the
 *  position. The rider requests a change with a handlebar push-button, but
 *  the firmware only allows it when the vehicle is slow and not accelerating
 *  hard. If the vehicle goes faster than VEL_LIM_KMH, or accelerates harder
 *  than ACC_LIM_MS2, the HTS is driven back to OPEN automatically, and the
 *  button is ignored until the condition has cleared.
 *
 *      lockAllowed = (speed < VEL_LIM_KMH) AND (acceleration < ACC_LIM_MS2)
 *
 *      lockAllowed  | HTS position | Result
 *      -------------+--------------+------------------------------------------
 *      true         | OPEN/CLOSED  | button toggles the position
 *      false        | OPEN         | stays OPEN, button ignored
 *      false        | not OPEN     | motor drives HTS to OPEN ("release")
 *
 *  SAFETY PHILOSOPHY
 *  -----------------
 *  - The release direction is always the safe one: any doubt -> go to OPEN.
 *  - Every motor movement has a time limit (MOVE_TIMEOUT_MS). If the limit
 *    switches do not confirm the target in time, the motor is switched off
 *    and the controller latches a FAULT (audible alarm). A FAULT is cleared
 *    only by power-cycling the vehicle.
 *  - Both limit switches pressed at once is physically impossible: it means
 *    a wiring or switch failure, and also latches a FAULT.
 *  - The main loop never blocks (no delay(), no waiting loops), so wheel
 *    pulses are never missed while the motor is moving or the buzzer sounds.
 *
 *  PIN MAP (matches the electrical schematic)
 *  ------------------------------------------
 *    D2   buzzer                 D10  H-bridge input A (motor)
 *    D3   rider push-button      D11  H-bridge input B (motor)
 *    D4   limit switch OPEN      A10  wheel-speed sensor (conditioned)
 *    D5   limit switch CLOSED
 *    All digital inputs use INPUT_PULLUP: pressed / active = LOW.
 *
 *  THINGS TO VERIFY ON THE REAL HARDWARE (marked "VERIFY" in the code)
 *  -------------------------------------------------------------------
 *  1. SENSOR_GAIN and the comparator thresholds, with an oscilloscope on A10.
 *  2. The motor direction mapping (see MotorCmd below).
 *  3. WHEEL_RADIUS_M, measured with the tyre loaded and at working pressure.
 *****************************************************************************/

#include <Arduino.h>

#define FW_VERSION "2.0"

// Set to 0 to silence the periodic telemetry line on the serial port.
#define ENABLE_TELEMETRY 1

// ---------------------------------------------------------------------------
//  Notes for the buzzer (same values as the standard Arduino pitches.h, so
//  the sketch is self-contained and does not need that extra file).
// ---------------------------------------------------------------------------
constexpr uint16_t NOTE_G4  = 392;
constexpr uint16_t NOTE_AS4 = 466;
constexpr uint16_t NOTE_C5  = 523;

// ---------------------------------------------------------------------------
//  Pins
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_BUZZER       = 2;
constexpr uint8_t PIN_BUTTON       = 3;   // rider push-button (pressed = LOW)
constexpr uint8_t PIN_LIMIT_OPEN   = 4;   // pressed (LOW) = HTS fully OPEN
constexpr uint8_t PIN_LIMIT_CLOSED = 5;   // pressed (LOW) = HTS fully CLOSED
constexpr uint8_t PIN_MOTOR_A      = 10;  // H-bridge input A
constexpr uint8_t PIN_MOTOR_B      = 11;  // H-bridge input B
constexpr uint8_t PIN_WHEEL_SENSOR = A10; // conditioned Hall-sensor signal

// ---------------------------------------------------------------------------
//  Vehicle and sensor parameters
// ---------------------------------------------------------------------------
constexpr float   WHEEL_RADIUS_M  = 0.26f;  // rolling radius [m]       VERIFY
constexpr uint8_t TONE_WHEEL_SLOTS = 50;    // teeth on the phonic wheel
constexpr float   ARC_PER_SLOT_M  =         // distance rolled per tooth [m]
    2.0f * PI * WHEEL_RADIUS_M / TONE_WHEEL_SLOTS;

// Analog front end. The voltage seen on A10 is scaled back to the sensor
// signal with SENSOR_GAIN (value inherited from the prototype).        VERIFY
// Note: one ADC count equals about 49 mV at the sensor with a gain of 10, so
// the signal swing must be much larger than the hysteresis band below.
constexpr float ADC_VREF_V   = 5.0f;
constexpr float SENSOR_GAIN  = 10.0f;
// Comparator with hysteresis (Schmitt trigger) centred on the 1.8 V threshold
// of the prototype. The old comment said the signal swings 0.8-1.5 V, which
// is below 1.8 V: check the real swing and re-centre the thresholds.   VERIFY
constexpr float V_THR_HIGH = 1.9f;          // rising edge above this
constexpr float V_THR_LOW  = 1.7f;          // re-arm below this
constexpr int   ADC_THR_HIGH =
    (int)((V_THR_HIGH / (ADC_VREF_V * SENSOR_GAIN)) * 1023.0f + 0.5f);
constexpr int   ADC_THR_LOW  =
    (int)((V_THR_LOW  / (ADC_VREF_V * SENSOR_GAIN)) * 1023.0f + 0.5f);
static_assert(ADC_THR_HIGH > ADC_THR_LOW, "Hysteresis band collapsed");

// ---------------------------------------------------------------------------
//  Interlock thresholds (same values as the prototype)
// ---------------------------------------------------------------------------
constexpr float    VEL_LIM_KMH     = 7.0f;   // above this the HTS must be OPEN
constexpr float    ACC_LIM_MS2     = 3.0f;   // at/above this the HTS must be OPEN
constexpr uint32_t MOVE_TIMEOUT_MS = 5000;   // max time to reach a target position

// ---------------------------------------------------------------------------
//  Timing and filtering
// ---------------------------------------------------------------------------
constexpr uint32_t SPEED_PERIOD_MS   = 100;   // speed/acceleration update rate
constexpr uint8_t  EDGE_WINDOW       = 16;    // ring buffer of tooth edges (max edges per estimate)
static_assert((EDGE_WINDOW & (EDGE_WINDOW - 1)) == 0, "EDGE_WINDOW must be a power of 2");
constexpr uint32_t SPEED_WINDOW_US   = 100000;// edges within the last 100 ms are used (at least 2)
constexpr uint32_t MIN_EDGE_US       = 500;   // edges closer than this are noise
constexpr uint32_t EDGE_STALE_US     = 250000;// no edge for 250 ms -> speed = 0
constexpr uint32_t LIMIT_DEBOUNCE_MS = 5;
constexpr uint32_t BUTTON_DEBOUNCE_MS= 30;
constexpr uint32_t CONFLICT_HOLD_MS  = 20;    // both limits pressed this long -> FAULT
constexpr uint32_t MOTOR_DEADTIME_MS = 50;    // pause when reversing the motor
constexpr uint32_t TELEMETRY_MS      = 200;
constexpr unsigned long SERIAL_BAUD  = 115200;

// ===========================================================================
//  Types
// ===========================================================================
enum class Pos   : uint8_t { OPEN, CLOSED, MID, CONFLICT };
enum class State : uint8_t { IDLE, MOVING, RELEASING, FAULT };
enum class Fault : uint8_t { NONE, LIMIT_CONFLICT, MOVE_TIMEOUT, RELEASE_TIMEOUT };

// Motor commands. Direction mapping is inherited from the prototype:
//   FORWARD = A high / B low : used for every rider-commanded movement
//   REVERSE = A low  / B high: used for the automatic release
// VERIFY that this matches the mechanism (e.g. a continuous-rotation cam).
enum class MotorCmd : uint8_t { STOP, FORWARD, REVERSE };

// ===========================================================================
//  Debounced digital input (active LOW)
// ===========================================================================
struct DebouncedInput {
  uint8_t  pin;
  uint32_t debounceMs;
  bool     stable   = false;   // true = active (pin LOW)
  bool     lastRaw  = false;
  uint32_t changedAt = 0;

  void init(uint8_t p, uint32_t ms) {
    pin = p; debounceMs = ms;
    stable = lastRaw = (digitalRead(pin) == LOW);
    changedAt = millis();
  }
  void update(uint32_t now) {
    bool raw = (digitalRead(pin) == LOW);
    if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
    else if (raw != stable && (uint32_t)(now - changedAt) >= debounceMs) stable = raw;
  }
};

// ===========================================================================
//  Motor driver: never reverses without passing through STOP + dead time
// ===========================================================================
struct Motor {
  MotorCmd applied   = MotorCmd::STOP;
  MotorCmd requested = MotorCmd::STOP;
  MotorCmd lastDir   = MotorCmd::STOP;
  uint32_t stoppedAt = 0;

  void request(MotorCmd c) { requested = c; }

  // Immediate stop, bypasses everything (used on FAULT).
  void emergencyStop() { requested = MotorCmd::STOP; apply(MotorCmd::STOP, millis()); }

  void update(uint32_t now) {
    if (requested == applied) return;
    if (requested == MotorCmd::STOP) { apply(MotorCmd::STOP, now); return; }
    if (applied != MotorCmd::STOP)   { apply(MotorCmd::STOP, now); return; }  // reversing: stop first
    if (lastDir != MotorCmd::STOP && requested != lastDir &&
        (uint32_t)(now - stoppedAt) < MOTOR_DEADTIME_MS) return;              // wait out the dead time
    apply(requested, now);
  }

private:
  void apply(MotorCmd c, uint32_t now) {
    // Both outputs are always released before one is raised.
    digitalWrite(PIN_MOTOR_A, LOW);
    digitalWrite(PIN_MOTOR_B, LOW);
    if (c == MotorCmd::FORWARD) digitalWrite(PIN_MOTOR_A, HIGH);
    if (c == MotorCmd::REVERSE) digitalWrite(PIN_MOTOR_B, HIGH);
    if (c == MotorCmd::STOP) { if (applied != MotorCmd::STOP) { lastDir = applied; stoppedAt = now; } }
    applied = c;
  }
};

// ===========================================================================
//  Non-blocking buzzer: plays a list of (frequency, duration) steps
// ===========================================================================
struct BeepStep { uint16_t freq; uint16_t ms; };   // freq 0 = silence

const BeepStep PAT_START[] = { { NOTE_G4, 200 } };                           // command accepted
const BeepStep PAT_DONE[]  = { { NOTE_AS4, 100 }, { 0, 100 }, { NOTE_AS4, 100 } }; // target reached
const BeepStep PAT_FAULT[] = { { NOTE_C5, 90 }, { 0, 210 } };                // repeating alarm

struct Beeper {
  const BeepStep* pat = nullptr;
  uint8_t  len = 0, idx = 0;
  bool     repeat = false, active = false;
  uint32_t stepStart = 0;

  void start(const BeepStep* p, uint8_t n, bool rep, uint32_t now) {
    pat = p; len = n; idx = 0; repeat = rep; active = true;
    beginStep(now);
  }
  void update(uint32_t now) {
    if (!active || (uint32_t)(now - stepStart) < pat[idx].ms) return;
    if (++idx >= len) {
      if (!repeat) { active = false; noTone(PIN_BUZZER); return; }
      idx = 0;
    }
    beginStep(now);
  }
private:
  void beginStep(uint32_t now) {
    stepStart = now;
    if (pat[idx].freq) tone(PIN_BUZZER, pat[idx].freq); else noTone(PIN_BUZZER);
  }
};

// ===========================================================================
//  Global state
// ===========================================================================
DebouncedInput btn, limOpen, limClosed;
Motor  motor;
Beeper beeper;

State  state  = State::IDLE;
Fault  fault  = Fault::NONE;
Pos    target = Pos::OPEN;        // valid while state == MOVING
uint32_t stateSince = 0;          // time the current movement started

// Wheel-speed measurement
bool     wheelHigh = false;
uint32_t edgeTimesUs[EDGE_WINDOW];
uint8_t  edgeHead = 0, edgeCount = 0;
uint32_t lastEdgeUs = 0;

// Kinematics (updated every SPEED_PERIOD_MS)
float    vHist[3] = { 0, 0, 0 };  // [0] = newest, speed in m/s
uint32_t tHist[3] = { 0, 0, 0 };  // timestamps [ms]
uint8_t  histCount = 0;
float    speedMs = 0, speedKmh = 0, accMs2 = 0;
bool     speedValid = false;      // false for the first ~300 ms after power-up
uint32_t lastKinematicsMs = 0;

bool     buttonArmed = true;      // re-armed when the button has been released
uint32_t conflictSince = 0;
bool     conflictActive = false;
uint32_t lastTelemetryMs = 0;

// ===========================================================================
//  Wheel-speed measurement
// ===========================================================================

// Called on every loop pass (~every 0.15 ms). Detects rising edges of the
// phonic-wheel signal with hysteresis and time-stamps them.
void sampleWheelSensor() {
  int adc = analogRead(PIN_WHEEL_SENSOR);
  if (!wheelHigh) {
    if (adc >= ADC_THR_HIGH) {
      wheelHigh = true;
      uint32_t t = micros();
      if ((uint32_t)(t - lastEdgeUs) >= MIN_EDGE_US) {   // reject glitches
        edgeTimesUs[edgeHead] = t;
        edgeHead = (edgeHead + 1) & (EDGE_WINDOW - 1);
        if (edgeCount < EDGE_WINDOW) edgeCount++;
        lastEdgeUs = t;
      }
    }
  } else if (adc <= ADC_THR_LOW) {
    wheelHigh = false;
  }
}

// Speed from the time between the oldest and newest edge inside the last
// SPEED_WINDOW_US ("reciprocal counting"). Unlike counting pulses in a fixed
// window, the resolution does not depend on the window length: one pulse in
// 100 ms would be a 1.2 km/h step, while this estimate is accurate to a
// fraction of a percent. The window is limited in TIME, not in number of
// teeth: a fixed number of teeth would span a long, shrinking interval at low
// speed and distort the acceleration estimate while the vehicle speeds up.
float computeSpeedMs() {
  if (edgeCount == 0) return 0.0f;
  uint32_t nowUs = micros();
  uint32_t newest = edgeTimesUs[(edgeHead - 1) & (EDGE_WINDOW - 1)];
  uint32_t sinceNewest = nowUs - newest;

  if (sinceNewest > EDGE_STALE_US) {   // wheel stopped: forget old edges
    edgeCount = 0;                     // (also avoids micros() wrap-around artefacts)
    return 0.0f;
  }
  if (edgeCount < 2) return 0.0f;      // need two edges to measure a period

  // Walk back from the newest edge while the edges are inside the window.
  // The two newest edges are always used, so even a slow wheel gives a value.
  uint8_t  n = 1;
  uint32_t oldest = newest;
  while (n < edgeCount) {
    uint32_t t = edgeTimesUs[(edgeHead - 1 - n) & (EDGE_WINDOW - 1)];
    if (n >= 2 && (uint32_t)(newest - t) > SPEED_WINDOW_US) break;
    oldest = t;
    n++;
  }
  float span_s = (float)(newest - oldest) * 1e-6f;
  float v = (float)(n - 1) * ARC_PER_SLOT_M / span_s;

  // When the wheel slows down, the time since the last tooth already tells us
  // the speed cannot be higher than one tooth per that interval.
  if (sinceNewest > 0) {
    float vBound = ARC_PER_SLOT_M / ((float)sinceNewest * 1e-6f);
    if (v > vBound) v = vBound;
  }
  return v;
}

// Speed and acceleration at a fixed rate. Acceleration is taken over 2 samples
// (200 ms) to reduce noise.
void updateKinematics(uint32_t now) {
  lastKinematicsMs = now;
  float v = computeSpeedMs();

  vHist[2] = vHist[1]; vHist[1] = vHist[0]; vHist[0] = v;
  tHist[2] = tHist[1]; tHist[1] = tHist[0]; tHist[0] = now;
  if (histCount < 3) histCount++;

  speedMs  = v;
  speedKmh = v * 3.6f;
  if (histCount >= 3) {
    float dt_s = (float)(tHist[0] - tHist[2]) * 1e-3f;
    accMs2 = (vHist[0] - vHist[2]) / dt_s;
    speedValid = true;
  }
}

// ===========================================================================
//  Helpers
// ===========================================================================
Pos readPosition() {
  bool o = limOpen.stable, c = limClosed.stable;
  if (o && c) return Pos::CONFLICT;
  if (o)      return Pos::OPEN;
  if (c)      return Pos::CLOSED;
  return Pos::MID;                      // between the two limit switches
}

const __FlashStringHelper* posName(Pos p) {
  switch (p) {
    case Pos::OPEN:   return F("OPEN");
    case Pos::CLOSED: return F("CLOSED");
    case Pos::MID:    return F("MID");
    default:          return F("CONFLICT");
  }
}
const __FlashStringHelper* stateName(State s) {
  switch (s) {
    case State::IDLE:      return F("IDLE");
    case State::MOVING:    return F("MOVING");
    case State::RELEASING: return F("RELEASING");
    default:               return F("FAULT");
  }
}
const __FlashStringHelper* faultName(Fault f) {
  switch (f) {
    case Fault::LIMIT_CONFLICT:  return F("both limit switches pressed");
    case Fault::MOVE_TIMEOUT:    return F("timeout while moving");
    case Fault::RELEASE_TIMEOUT: return F("timeout while releasing");
    default:                     return F("none");
  }
}

// Plays a pattern unless the controller is in FAULT (the alarm has priority).
void beep(const BeepStep* p, uint8_t n, uint32_t now) {
  if (state != State::FAULT) beeper.start(p, n, false, now);
}

// Latches a FAULT: motor off, alarm on. Cleared only by a power cycle.
void enterFault(Fault f, uint32_t now) {
  motor.emergencyStop();
  state = State::FAULT;
  fault = f;
  beeper.start(PAT_FAULT, sizeof(PAT_FAULT) / sizeof(PAT_FAULT[0]), true, now);
  Serial.print(F("FAULT: ")); Serial.println(faultName(f));
}

// The two interlock conditions, valid only once the speed estimate is ready.
bool releaseRequired() { return speedValid && (speedKmh >= VEL_LIM_KMH || accMs2 >= ACC_LIM_MS2); }
bool buttonEnabled()   { return speedValid && (speedKmh <  VEL_LIM_KMH && accMs2 <  ACC_LIM_MS2); }

void startRelease(uint32_t now) {
  state = State::RELEASING;
  stateSince = now;
  motor.request(MotorCmd::REVERSE);
  beep(PAT_START, 1, now);
  Serial.println(F("Interlock: releasing HTS (speed/acceleration limit)"));
}

void startMove(Pos to, uint32_t now) {
  state = State::MOVING;
  target = to;
  stateSince = now;
  motor.request(MotorCmd::FORWARD);
  beep(PAT_START, 1, now);
  Serial.println(to == Pos::OPEN ? F("HTS opening") : F("HTS closing"));
}

void finishMovement(uint32_t now, Pos reached) {
  motor.request(MotorCmd::STOP);
  state = State::IDLE;
  beep(PAT_DONE, sizeof(PAT_DONE) / sizeof(PAT_DONE[0]), now);
  Serial.print(F("HTS ")); Serial.println(posName(reached));
}

// ===========================================================================
//  State machine
// ===========================================================================
void runStateMachine(uint32_t now) {
  if (state == State::FAULT) return;

  Pos pos = readPosition();

  // Both limit switches pressed is impossible: wiring or switch failure.
  if (pos == Pos::CONFLICT) {
    if (!conflictActive) { conflictActive = true; conflictSince = now; }
    if ((uint32_t)(now - conflictSince) >= CONFLICT_HOLD_MS) enterFault(Fault::LIMIT_CONFLICT, now);
    return;
  }
  conflictActive = false;

  // Button: act on the press, then wait for release (no ping-pong if held).
  bool buttonPressedEvent = false;
  if (btn.stable) { if (buttonArmed) { buttonPressedEvent = true; buttonArmed = false; } }
  else buttonArmed = true;

  switch (state) {
    case State::IDLE:
      if (releaseRequired() && pos != Pos::OPEN) {
        startRelease(now);
      } else if (buttonPressedEvent && buttonEnabled() &&
                 (pos == Pos::OPEN || pos == Pos::CLOSED)) {
        startMove(pos == Pos::OPEN ? Pos::CLOSED : Pos::OPEN, now);
      }
      break;

    case State::MOVING:
      if (releaseRequired()) { startRelease(now); break; }    // safety overrides the rider
      if (pos == target)     { finishMovement(now, pos); break; }
      if ((uint32_t)(now - stateSince) > MOVE_TIMEOUT_MS) enterFault(Fault::MOVE_TIMEOUT, now);
      break;

    case State::RELEASING:
      if (pos == Pos::OPEN)  { finishMovement(now, pos); break; }
      if ((uint32_t)(now - stateSince) > MOVE_TIMEOUT_MS) enterFault(Fault::RELEASE_TIMEOUT, now);
      break;

    default: break;
  }
}

// ===========================================================================
//  Telemetry (one line every TELEMETRY_MS, CSV: ms, km/h, m/s2, position, state)
// ===========================================================================
void telemetry(uint32_t now) {
#if ENABLE_TELEMETRY
  if ((uint32_t)(now - lastTelemetryMs) < TELEMETRY_MS) return;
  lastTelemetryMs = now;
  Serial.print(now);                 Serial.print(',');
  Serial.print(speedKmh, 1);         Serial.print(',');
  Serial.print(accMs2, 2);           Serial.print(',');
  Serial.print(posName(readPosition())); Serial.print(',');
  Serial.println(stateName(state));
#endif
}

// ===========================================================================
//  Arduino entry points
// ===========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Outputs are set LOW *before* becoming outputs so the motor never twitches.
  digitalWrite(PIN_MOTOR_A, LOW);
  digitalWrite(PIN_MOTOR_B, LOW);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LIMIT_OPEN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_CLOSED, INPUT_PULLUP);
  btn.init(PIN_BUTTON, BUTTON_DEBOUNCE_MS);
  limOpen.init(PIN_LIMIT_OPEN, LIMIT_DEBOUNCE_MS);
  limClosed.init(PIN_LIMIT_CLOSED, LIMIT_DEBOUNCE_MS);

  uint32_t now = millis();
  lastKinematicsMs = now;
  Serial.print(F("HTS controller v")); Serial.println(F(FW_VERSION));

  // Power-up: decide what to do from the limit switches alone.
  Pos pos = readPosition();
  Serial.print(F("Initial HTS position: ")); Serial.println(posName(pos));
  if (pos == Pos::CONFLICT) {
    enterFault(Fault::LIMIT_CONFLICT, now);
  } else if (pos == Pos::MID) {
    startMove(Pos::OPEN, now);          // stopped half-way: finish the movement to OPEN
  }
  // OPEN or CLOSED: stay as found; the interlock acts once the speed is known.
}

void loop() {
  uint32_t now = millis();

  sampleWheelSensor();                                   // every pass: never miss a tooth
  if ((uint32_t)(now - lastKinematicsMs) >= SPEED_PERIOD_MS) updateKinematics(now);

  btn.update(now);
  limOpen.update(now);
  limClosed.update(now);

  runStateMachine(now);
  motor.update(now);
  beeper.update(now);
  telemetry(now);
}

/*****************************************************************************
 *  CHANGES FROM THE PROTOTYPE SKETCH
 *  ---------------------------------
 *  Behaviour kept: thresholds (7 km/h, 3 m/s2), 5 s movement timeout, motor
 *  directions, power-up rules, beep meanings.
 *
 *  Fixed
 *   - Allarme() called itself forever: stack overflow and an unpredictable
 *     reset. Now a latched FAULT with a non-blocking alarm.
 *   - delay() and waiting while-loops froze the loop for up to 5 s, so wheel
 *     pulses were lost exactly when the interlock was acting. Now everything
 *     is a state machine driven by millis().
 *   - The state message was printed on every loop pass; at 9600 baud the
 *     serial buffer fills and print() blocks, slowing the loop to a few tens
 *     of passes per second - far too slow to follow a 60 Hz tooth signal.
 *     Now one message per event plus rate-limited telemetry, at 115200 baud.
 *   - Speed came from counting pulses in a 100 ms window: one pulse is
 *     1.18 km/h, and the difference between two windows is 3.27 m/s2, above
 *     the 3 m/s2 limit. A single pulse of jitter could therefore release the
 *     HTS. Now speed uses edge timestamps (reciprocal counting) and
 *     acceleration is computed over 200 ms.
 *   - The comparator had no hysteresis: noise around the threshold produced
 *     extra pulses. Added a Schmitt trigger and a minimum edge spacing.
 *   - The button was level-triggered: holding it re-triggered the movement.
 *     Now one action per press.
 *   - Duplicated 25-line blocks for "speed above limit" and "acceleration
 *     above limit" merged into a single rule.
 *   - Motor could be reversed instantly. Added a stop + 50 ms dead time.
 *   - Strings in RAM (String class) replaced with F() flash strings.
 *
 *  Added
 *   - Plausibility gate at power-up: until the speed estimate is valid
 *     (about 300 ms) the button is disabled.
 *   - Telemetry CSV line for bench and road logging.
 *
 *  Known limitations / next steps (see the project dossier)
 *   - Speed comes from a single sensor: a dead sensor reads as 0 km/h and
 *     would allow the lock at speed. Add a signal plausibility check and a
 *     second speed source (e.g. the IMU).
 *   - Stall is detected only by timeout; a motor current measurement would
 *     detect it in milliseconds.
 *   - A hardware watchdog is not enabled: some Mega 2560 bootloaders do not
 *     clear it after a reset and can boot-loop. Enable it only after testing
 *     on the actual board.
 *****************************************************************************/
