// ============================================================
// EA3121 Robotics Lab 02
// Activity 1 - High Performance Line Following Robot
//
// ESP32 + MD0482 5-Bit IR Sensor + TB6612FNG
//
// TRACK:
// Black line approximately 5 cm wide
//
// IMPORTANT:
// Perfect centered pattern = 10001  (middle 3 sensors on black)
//
// Sensor:
// 0 = BLACK
// 1 = WHITE
//
// ------------------------------------------------------------
// CONTROL IMPROVEMENTS
// ------------------------------------------------------------
// 1. Full PID (I term added) with anti-windup + conditional
//    integration, instead of PD only.
// 2. Error is low-pass filtered BEFORE the derivative is taken.
//    Digital sensors give a stair-stepped error; differentiating
//    it raw produced 1-tick spikes. Smooth error -> smooth D.
// 3. Sensor readings pass a 3-sample majority filter (kills
//    single-tick glitches from reflections / stray light).
// 4. Continuous speed profile (speed falls smoothly with error)
//    replaces the stepped speed table.
// 5. Motor outputs are slew-rate limited -> no torque steps.
// 6. No "perfect centre fast path". Zeroing the PID state every
//    time the robot crossed centre threw away the D term at the
//    exact moment it was needed, which caused weaving.
// 7. Smooth extra-gain curve on large error instead of a
//    hard-coded outer-sensor kick.
//
// ------------------------------------------------------------
// END OF MAZE -> U-TURN
// ------------------------------------------------------------
// The end marker is a large filled BLACK BOX, not a thin bar.
// A thin junction bar is black only briefly; the box stays black
// for a sustained distance. That is the whole difference.
//
//   1. ALL BLACK (00000) appears          -> start advancing
//   2. still all black after 250 ms       -> confirmed END BOX
//   3. keep driving forward across it
//   4. ALL WHITE (11111) = off the far edge
//                                         -> spin in place NOW
//   5. spin until ALL BLACK (00000) again -> box reacquired,
//      the sensors re-enter it in reverse order (~180 deg)
//   6. drive forward across the box until the line reappears
//                                         -> resume PID follow
//
// If all black clears again BEFORE the 250 ms confirm, it was an
// ordinary junction and the robot just drives straight through.
// ============================================================


// ============================================================
// SENSOR CONNECTIONS (unchanged)
// ============================================================

#define S0_PIN 13   // Far Left   (sensor1)
#define S1_PIN 14   // Left       (sensor2)
#define S2_PIN 25   // Centre     (sensor3)
#define S3_PIN 26   // Right      (sensor4)
#define S4_PIN 27   // Far Right  (sensor5)


// ============================================================
// TB6612FNG CONNECTIONS (unchanged)
// ============================================================

#define STBY_PIN 21

// Left Motor - Motor A
#define PWMA_PIN 22
#define AIN1_PIN 16
#define AIN2_PIN 17

// Right Motor - Motor B
#define PWMB_PIN 23
#define BIN1_PIN 18
#define BIN2_PIN 19


// ============================================================
// SENSOR PATTERN CONSTANTS
// ============================================================

#define PATTERN_ALL_WHITE 0b11111   // no black anywhere
#define PATTERN_ALL_BLACK 0b00000   // sitting on a wide black area
#define PATTERN_CENTERED  0b10001   // perfectly centred on a 5 cm line


// ============================================================
// SPEED SETTINGS
// ============================================================

// Cruise speed on a straight
int STRAIGHT_SPEED = 200;

// Speed floor when the line is at the extreme edge of the array.
// Speed is interpolated smoothly between these two.
int MIN_TURN_SPEED = 140;

// Speed while driving across a black junction / the end box
int JUNCTION_SPEED = 190;

// Pivot speed while hunting for a lost line
int RECOVERY_SPEED = 200;

// How far the inner wheel is allowed to reverse on a hard corner.
// Negative = it may pivot. Less negative = gentler, more stable.
int INNER_WHEEL_MIN = -140;


// ============================================================
// PID CONTROLLER
// ============================================================

float Kp = 42.0;     // correction per unit error
float Ki = 20.0;     // correction per (error * second)   -- start at 0
float Kd = 0.25;     // correction per (error / second)

// error is in "sensor positions", range -4.0 .. +4.0.
// Kd acts on a TRUE time derivative, not a per-tick delta, so its
// numeric value is much smaller than a per-tick Kd would be.

// Maximum total controller correction
float MAX_CORRECTION = 300.0;

// Anti-windup: hard cap on the I contribution alone
float MAX_I_CORRECTION = 60.0;

// Integral only accumulates when |error| is small enough that the
// robot is genuinely tracking (not mid-corner). Stops the I term
// from winding up through every turn.
float I_ACTIVE_ERROR = 1.5;

// Extra proportional authority once the line reaches the outer
// sensors. gain = 1 + SLOPE * (|error| - KNEE), clamped at 1 below
// the knee.
float EXTRA_GAIN_KNEE  = 2.0;
float EXTRA_GAIN_SLOPE = 0.25;


// ============================================================
// FILTERING
// ============================================================

// Error low-pass. Higher = smoother but more lag.
// At 1 kHz, 0.90 gives roughly a 10 ms time constant.
float ERROR_FILTER_ALPHA = 0.90;

// Extra light smoothing on the computed derivative
float DERIV_FILTER_ALPHA = 0.70;


// ============================================================
// CONTROL LOOP
// ============================================================

// 1000 microseconds = 1 kHz controller
const unsigned long CONTROL_PERIOD_US = 1000;
unsigned long previousControlTime = 0;

// Motor slew rate limit, PWM counts per control tick.
// 12/tick at 1 kHz = 0 -> full scale in ~21 ms. Fast enough to be
// invisible to the PID, slow enough to kill output steps.
const int MAX_PWM_STEP_PER_TICK = 12;

int appliedLeftSpeed  = 0;
int appliedRightSpeed = 0;


// ============================================================
// CONTROLLER STATE
// ============================================================

float filteredError     = 0.0;
float prevFilteredError = 0.0;
float filteredDerivative = 0.0;
float integralTerm      = 0.0;   // stored as the I *contribution*
float lastValidError    = 0.0;


// ============================================================
// END OF MAZE / U-TURN TUNING
// ============================================================

// All black must persist this long to count as the END BOX rather
// than an ordinary junction bar. Raise it if a wide junction is
// being mistaken for the end box; lower it if the end box is
// small or the robot is slow.
const unsigned long END_BOX_CONFIRM_MS = 250;

// Safety: if all black somehow never clears, U-turn anyway.
const unsigned long BOX_ADVANCE_MAX_MS = 2500;

// Short brake before spinning. Rotating while still rolling
// forward is the usual reason a U-turn lands at the wrong angle.
// Set to 0 for a fully immediate spin.
const unsigned long UT_BRAKE_MS = 60;

const int UT_SPIN_SPEED = 200;

// Sensors are ignored for this long after the spin starts, so a
// clipped corner of the box cannot end the turn at ~90 degrees.
// Raise if the turn stops short, lower if it overshoots past 180.
const unsigned long UT_SPIN_IGNORE_MS = 120;

// All 5 sensors must read black for this many ticks (1 tick = 1 ms)
// before the spin is accepted as complete.
const int UT_BLACK_LOCK_TICKS = 15;

const unsigned long UT_SPIN_TIMEOUT_MS = 1500;
const int           UT_MAX_ATTEMPTS    = 3;

// Driving back across the box after the spin
const int           UT_EXIT_SPEED      = 170;
const unsigned long UT_EXIT_TIMEOUT_MS = 1500;
const int           UT_EXIT_LINE_TICKS = 20;   // ms of real line before resuming

// Ignore end-box detection for this long after finishing a U-turn
const unsigned long UT_COOLDOWN_MS = 900;


// ============================================================
// LINE-LOSS TUNING (all white while following)
// ============================================================

const unsigned long LOST_COAST_MS  = 60;   // bridge small gaps by coasting
const unsigned long LOST_SEARCH_MS = 400;  // pivot search window, then give up


// ============================================================
// STATE MACHINE
// ============================================================

enum RobotState {
  ST_FOLLOW,
  ST_SEARCH,        // line lost (all white) - coast then pivot
  ST_BOX_ADVANCE,   // on a black area - junction or end box?
  ST_UT_BRAKE,
  ST_UT_SPIN,       // spin until all black again
  ST_UT_EXIT,       // drive back across the box onto the line
  ST_STOPPED
};

RobotState robotState = ST_FOLLOW;
unsigned long stateStart = 0;

bool endBoxConfirmed = false;
unsigned long uturnCooldownUntil = 0;

int  uturnDir = 1;          // +1 = spin right (clockwise), -1 = spin left
int  uturnAttempt = 0;
int  blackLockTicks = 0;
int  exitLineTicks = 0;


// ============================================================
// SENSOR GLITCH FILTER
// ============================================================

uint8_t patternHistory[3] = { PATTERN_ALL_WHITE, PATTERN_ALL_WHITE, PATTERN_ALL_WHITE };
uint8_t patternHistoryIdx = 0;


// ============================================================
// DEBUG
// ============================================================

#define DEBUG_MODE 1

unsigned long previousDebugTime = 0;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void debugOutput(uint8_t pattern, float error, int leftSpeed, int rightSpeed);
void enterState(RobotState s);


// ============================================================
// MOTOR CONTROL
// ============================================================

void setLeftMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Forward
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, speed);
  }
  else if (speed < 0) {
    // Reverse
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, -speed);
  }
  else {
    // Short brake
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, 255);
  }
}

// ============================================================
// RIGHT MOTOR
// ============================================================

void setRightMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Forward
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, speed);
  }
  else if (speed < 0) {
    // Reverse
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, -speed);
  }
  else {
    // Short brake
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, 255);
  }
}

// ============================================================
// DRIVE BOTH MOTORS - SLEW LIMITED
//
// Ramps the commanded speed instead of stepping to it. Removes the
// torque jolt that made the old code twitch on every sensor change.
// ============================================================

void driveMotors(int leftSpeed, int rightSpeed) {

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  appliedLeftSpeed  += constrain(leftSpeed  - appliedLeftSpeed,
                                 -MAX_PWM_STEP_PER_TICK, MAX_PWM_STEP_PER_TICK);
  appliedRightSpeed += constrain(rightSpeed - appliedRightSpeed,
                                 -MAX_PWM_STEP_PER_TICK, MAX_PWM_STEP_PER_TICK);

  setLeftMotor(appliedLeftSpeed);
  setRightMotor(appliedRightSpeed);
}

// ============================================================
// DRIVE BOTH MOTORS - IMMEDIATE (no ramp)
//
// Used for braking, where the whole point is to stop now.
// ============================================================

void driveMotorsInstant(int leftSpeed, int rightSpeed) {
  appliedLeftSpeed  = constrain(leftSpeed,  -255, 255);
  appliedRightSpeed = constrain(rightSpeed, -255, 255);
  setLeftMotor(appliedLeftSpeed);
  setRightMotor(appliedRightSpeed);
}

// ============================================================
// STOP
// ============================================================

void stopMotors() {
  driveMotorsInstant(0, 0);
}

// ============================================================
// READ 5 SENSOR PATTERN (raw)
// ============================================================

uint8_t readSensorPatternRaw() {
  uint8_t pattern = 0;

  pattern |= digitalRead(S0_PIN) << 4;
  pattern |= digitalRead(S1_PIN) << 3;
  pattern |= digitalRead(S2_PIN) << 2;
  pattern |= digitalRead(S3_PIN) << 1;
  pattern |= digitalRead(S4_PIN);

  return pattern;
}

// ============================================================
// READ 5 SENSOR PATTERN (3-sample majority per channel)
//
// A sensor has to agree with itself on 2 of the last 3 ticks to
// change state. At 1 kHz that is 3 ms of lag and it removes the
// single-tick dropouts that used to slam the derivative term.
// ============================================================

uint8_t readSensorPattern() {

  patternHistory[patternHistoryIdx] = readSensorPatternRaw();
  patternHistoryIdx = (patternHistoryIdx + 1) % 3;

  uint8_t filtered = 0;

  for (int bit = 0; bit < 5; bit++) {
    int votes = ((patternHistory[0] >> bit) & 1)
              + ((patternHistory[1] >> bit) & 1)
              + ((patternHistory[2] >> bit) & 1);

    if (votes >= 2) filtered |= (1 << bit);
  }

  return filtered;
}

// ============================================================
// CALCULATE LINE ERROR
//
// Weighted centroid of the black sensors. Range -4.0 .. +4.0.
// Negative = line is to the LEFT, positive = line is to the RIGHT.
//
// The perfect-centre pattern 10001 already produces exactly 0.0
// here ((-2 + 0 + 2) / 3), so it needs no special case.
// ============================================================

bool calculateLineError(uint8_t pattern, float &error, int &blackCount) {

  int b0 = !((pattern >> 4) & 1);
  int b1 = !((pattern >> 3) & 1);
  int b2 = !((pattern >> 2) & 1);
  int b3 = !((pattern >> 1) & 1);
  int b4 = !(pattern & 1);

  blackCount = b0 + b1 + b2 + b3 + b4;

  // No black detected
  if (blackCount == 0) {
    return false;
  }

  // WEIGHTED POSITION
  float weightedSum = (b0 * -4.0) + (b1 * -2.0) + (b2 * 0.0) + (b3 * 2.0) + (b4 * 4.0);
  error = weightedSum / blackCount;

  return true;
}

// ============================================================
// DYNAMIC BASE SPEED
//
// Continuous instead of stepped: full speed on centre, falling
// linearly to MIN_TURN_SPEED when the line is at the outer sensor.
// ============================================================

int calculateBaseSpeed(float error, float derivative) {

  float t = fabs(error) / 4.0;
  t = constrain(t, 0.0, 1.0);

  float base = STRAIGHT_SPEED - (STRAIGHT_SPEED - MIN_TURN_SPEED) * t;

  // Additional slow-down while the line is sweeping quickly across
  // the array - i.e. a corner is arriving.
  float dSlow = fabs(derivative) * 0.10;
  base -= constrain(dSlow, 0.0, 40.0);

  return (int)constrain(base, (float)MIN_TURN_SPEED - 40.0, (float)STRAIGHT_SPEED);
}

// ============================================================
// PID RESET
// ============================================================

void resetController() {
  filteredError      = 0.0;
  prevFilteredError  = 0.0;
  filteredDerivative = 0.0;
  integralTerm       = 0.0;
}

// ============================================================
// STATE HELPER
// ============================================================

void enterState(RobotState s) {
  robotState = s;
  stateStart = millis();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Sensor pins
  pinMode(S0_PIN, INPUT);
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);

  // Motor pins
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);

  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY_PIN, HIGH);

  stopMotors();
  delay(500);

  previousControlTime = micros();
  enterState(ST_FOLLOW);

  Serial.println("\n======================================");
  Serial.println("LINE FOLLOWER - PID + END BOX U-TURN");
  Serial.println("======================================");
}

// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void loop() {
  unsigned long now = micros();

  unsigned long elapsedUs = now - previousControlTime;
  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }
  previousControlTime = now;

  // Real dt, clamped so a hiccup cannot blow up I or D
  float dt = constrain(elapsedUs / 1000000.0, 0.0005, 0.005);

  unsigned long nowMs = millis();

  uint8_t pattern = readSensorPattern();

  bool allWhite = (pattern == PATTERN_ALL_WHITE);
  bool allBlack = (pattern == PATTERN_ALL_BLACK);

  float error = 0.0;
  int blackCount = 0;
  bool lineDetected = calculateLineError(pattern, error, blackCount);

  // A usable line = some black, but not the whole array
  bool lineVisible = lineDetected && !allBlack;

  switch (robotState) {

    // --------------------------------------------------------
    // NORMAL LINE FOLLOWING - PID
    // --------------------------------------------------------
    case ST_FOLLOW: {

      // Line completely lost
      if (allWhite) {
        enterState(ST_SEARCH);
        break;
      }

      // Hit a black area. Could be a junction bar or the end box -
      // ST_BOX_ADVANCE drives forward and finds out which.
      if (allBlack) {
        if (nowMs >= uturnCooldownUntil) {
          endBoxConfirmed = false;
          enterState(ST_BOX_ADVANCE);
        }
        else {
          // Just finished a U-turn, still near the box: push through
          driveMotors(JUNCTION_SPEED, JUNCTION_SPEED);
          debugOutput(pattern, 0.0, appliedLeftSpeed, appliedRightSpeed);
        }
        break;
      }

      lastValidError = error;

      // --- filter the error FIRST, then differentiate it ---
      filteredError = (ERROR_FILTER_ALPHA * filteredError)
                    + ((1.0 - ERROR_FILTER_ALPHA) * error);

      float rawDerivative = (filteredError - prevFilteredError) / dt;
      prevFilteredError = filteredError;

      filteredDerivative = (DERIV_FILTER_ALPHA * filteredDerivative)
                         + ((1.0 - DERIV_FILTER_ALPHA) * rawDerivative);

      // --- integral, with conditional integration + anti-windup ---
      if (fabs(filteredError) < I_ACTIVE_ERROR) {
        integralTerm += Ki * filteredError * dt;
        integralTerm = constrain(integralTerm, -MAX_I_CORRECTION, MAX_I_CORRECTION);
      }
      else {
        // Bleed off while cornering rather than snapping to zero
        integralTerm *= 0.995;
      }

      // --- extra authority at the edges of the array ---
      float extraGain = 1.0;
      if (fabs(filteredError) > EXTRA_GAIN_KNEE) {
        extraGain += EXTRA_GAIN_SLOPE * (fabs(filteredError) - EXTRA_GAIN_KNEE);
      }

      // --- PID ---
      float correction = (Kp * filteredError * extraGain)
                       + integralTerm
                       + (Kd * filteredDerivative);

      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

      int baseSpeed = calculateBaseSpeed(filteredError, filteredDerivative);

      int leftSpeed  = baseSpeed + (int)correction;
      int rightSpeed = baseSpeed - (int)correction;

      // Allow the inner wheel to back up on a hard corner, but only
      // so far - full counter-rotation at speed makes it spin out.
      leftSpeed  = constrain(leftSpeed,  INNER_WHEEL_MIN, 255);
      rightSpeed = constrain(rightSpeed, INNER_WHEEL_MIN, 255);

      driveMotors(leftSpeed, rightSpeed);
      debugOutput(pattern, filteredError, appliedLeftSpeed, appliedRightSpeed);
      break;
    }

    // --------------------------------------------------------
    // LINE LOST (all white) - coast, then pivot toward last side
    // --------------------------------------------------------
    case ST_SEARCH: {

      if (lineDetected) {
        // Reacquired. Do not reset the controller - keeping the
        // error history makes the re-entry smooth.
        enterState(ST_FOLLOW);
        break;
      }

      unsigned long lost = nowMs - stateStart;

      if (lost < LOST_COAST_MS) {
        // Small gap in the tape, or a glossy patch: keep going
        // straight-ish on the last correction.
        int nudge = (int)(Kp * lastValidError * 0.5);
        driveMotors(MIN_TURN_SPEED + nudge, MIN_TURN_SPEED - nudge);
      }
      else if (lost < LOST_SEARCH_MS) {
        // Pivot back toward whichever side the line left on
        if (lastValidError < -0.1) {
          driveMotors(-RECOVERY_SPEED, RECOVERY_SPEED);
        }
        else {
          driveMotors(RECOVERY_SPEED, -RECOVERY_SPEED);
        }
      }
      else {
        stopMotors();
        enterState(ST_STOPPED);
      }

      debugOutput(pattern, 99.0, appliedLeftSpeed, appliedRightSpeed);
      break;
    }

    // --------------------------------------------------------
    // ON A BLACK AREA - junction bar, or the END BOX?
    //
    // Drive straight across it and let the exit decide:
    //   line reappears before 250 ms  -> ordinary junction
    //   still black after 250 ms      -> END BOX confirmed
    //   all white after confirm       -> off the far edge, U-TURN
    // --------------------------------------------------------
    case ST_BOX_ADVANCE: {

      unsigned long inBlack = nowMs - stateStart;

      if (allBlack && inBlack >= END_BOX_CONFIRM_MS) {
        endBoxConfirmed = true;
      }

      // Reached the far edge of the black area
      if (allWhite) {
        if (endBoxConfirmed) {
          // THIS IS THE END OF THE MAZE - turn around
          uturnDir = (lastValidError < -0.1) ? -1 : 1;
          uturnAttempt = 0;
          blackLockTicks = 0;
          resetController();
          enterState(ST_UT_BRAKE);
#if DEBUG_MODE
          Serial.print("\n>>> END OF MAZE - U-TURN ");
          Serial.println(uturnDir > 0 ? "RIGHT" : "LEFT");
#endif
        }
        else {
          // A thin bar with nothing straight ahead (turn-only
          // junction). Hunt for the line instead of turning around.
          enterState(ST_SEARCH);
        }
        break;
      }

      // Line came back. Only an ordinary junction if the box was
      // never confirmed - once confirmed, a partial pattern is just
      // the ragged edge of the box, so keep advancing.
      if (lineVisible && !endBoxConfirmed) {
        enterState(ST_FOLLOW);
        break;
      }

      // Safety net: black that never ends
      if (inBlack >= BOX_ADVANCE_MAX_MS) {
        endBoxConfirmed = true;
        uturnDir = 1;
        uturnAttempt = 0;
        blackLockTicks = 0;
        resetController();
        enterState(ST_UT_BRAKE);
        break;
      }

      // Drive straight across, holding a damped version of the last
      // known error so a crossing does not knock it off course.
      int correction = (int)(Kp * lastValidError * 0.5);
      driveMotors(JUNCTION_SPEED + correction, JUNCTION_SPEED - correction);
      debugOutput(pattern, lastValidError * 0.5, appliedLeftSpeed, appliedRightSpeed);
      break;
    }

    // --------------------------------------------------------
    // U-TURN 1/3: brake
    //
    // Momentum is what makes a spin-in-place come out crooked.
    // Set UT_BRAKE_MS to 0 for a fully immediate spin.
    // --------------------------------------------------------
    case ST_UT_BRAKE: {
      stopMotors();
      if (nowMs - stateStart >= UT_BRAKE_MS) {
        blackLockTicks = 0;
        enterState(ST_UT_SPIN);
      }
      debugOutput(pattern, 0.0, 0, 0);
      break;
    }

    // --------------------------------------------------------
    // U-TURN 2/3: spin in place until ALL BLACK again
    //
    // The robot is just past the far edge of the box. Rotating on
    // the spot brings the sensor bar back around onto the box - the
    // sensors re-enter it in reverse order, and all 5 read black at
    // roughly 180 degrees.
    // --------------------------------------------------------
    case ST_UT_SPIN: {

      driveMotors(UT_SPIN_SPEED * uturnDir, -UT_SPIN_SPEED * uturnDir);

      unsigned long spinning = nowMs - stateStart;

      if (spinning >= UT_SPIN_IGNORE_MS) {

        if (allBlack) {
          blackLockTicks++;
        }
        else {
          blackLockTicks = 0;
        }

        if (blackLockTicks >= UT_BLACK_LOCK_TICKS) {
          exitLineTicks = 0;
          enterState(ST_UT_EXIT);
#if DEBUG_MODE
          Serial.println(">>> BOX REACQUIRED - crossing back");
#endif
          break;
        }
      }

      if (spinning >= UT_SPIN_TIMEOUT_MS) {
        uturnAttempt++;

        if (uturnAttempt >= UT_MAX_ATTEMPTS) {
          stopMotors();
          enterState(ST_STOPPED);
#if DEBUG_MODE
          Serial.println(">>> U-TURN FAILED - STOPPED");
#endif
        }
        else {
          // Sweep back the other way
          uturnDir = -uturnDir;
          blackLockTicks = 0;
          enterState(ST_UT_SPIN);
#if DEBUG_MODE
          Serial.println(">>> U-TURN RETRY - reversing sweep");
#endif
        }
      }

      debugOutput(pattern, 0.0, appliedLeftSpeed, appliedRightSpeed);
      break;
    }

    // --------------------------------------------------------
    // U-TURN 3/3: drive back across the box onto the line
    // --------------------------------------------------------
    case ST_UT_EXIT: {

      driveMotors(UT_EXIT_SPEED, UT_EXIT_SPEED);

      if (lineVisible) {
        exitLineTicks++;
      }
      else {
        exitLineTicks = 0;
      }

      if (exitLineTicks >= UT_EXIT_LINE_TICKS) {
        resetController();
        lastValidError = 0.0;
        endBoxConfirmed = false;
        uturnCooldownUntil = nowMs + UT_COOLDOWN_MS;
        enterState(ST_FOLLOW);
#if DEBUG_MODE
        Serial.println(">>> U-TURN COMPLETE\n");
#endif
        break;
      }

      // Drove clean off the box without finding the line
      if (allWhite || (nowMs - stateStart >= UT_EXIT_TIMEOUT_MS)) {
        uturnCooldownUntil = nowMs + UT_COOLDOWN_MS;
        enterState(ST_SEARCH);
      }

      debugOutput(pattern, 0.0, appliedLeftSpeed, appliedRightSpeed);
      break;
    }

    // --------------------------------------------------------
    // STOPPED - stays here until reset
    // --------------------------------------------------------
    case ST_STOPPED: {
      stopMotors();
      debugOutput(pattern, 0.0, 0, 0);
      break;
    }
  }
}

// ============================================================
// DEBUG OUTPUT
// ============================================================

const char* stateName() {
  switch (robotState) {
    case ST_FOLLOW:      return "FOLLOW";
    case ST_SEARCH:      return "SEARCH";
    case ST_BOX_ADVANCE: return "BLACK-ADV";
    case ST_UT_BRAKE:    return "UT-BRAKE";
    case ST_UT_SPIN:     return "UT-SPIN";
    case ST_UT_EXIT:     return "UT-EXIT";
    case ST_STOPPED:     return "STOPPED";
  }
  return "?";
}

void debugOutput(uint8_t pattern, float error, int leftSpeed, int rightSpeed) {
#if DEBUG_MODE
  if (millis() - previousDebugTime < 100) return;
  previousDebugTime = millis();

  for (int bit = 4; bit >= 0; bit--) {
    Serial.print((pattern >> bit) & 1);
  }

  Serial.print(" | ");
  Serial.print(stateName());
  Serial.print(" | Err: ");
  Serial.print(error, 2);
  Serial.print(" | I: ");
  Serial.print(integralTerm, 1);
  Serial.print(" | D: ");
  Serial.print(filteredDerivative, 1);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" | R: ");
  Serial.println(rightSpeed);
#endif
}
