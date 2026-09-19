// ============================================================
// EA3121 Robotics Lab 02
// Activity 2 - Obstacle Avoiding Robot
//
// ESP32 + ST GY-VL53L0XV2 Time-of-Flight laser + TB6612FNG
//
// ------------------------------------------------------------
// WIRING
// ------------------------------------------------------------
// MOTOR DRIVER - identical to day2/day2.ino, nothing to rewire:
//
//   STBY -> 21
//   PWMA -> 22   AIN1 -> 16   AIN2 -> 17     (LEFT  motor)
//   PWMB -> 23   BIN1 -> 18   BIN2 -> 19     (RIGHT motor)
//
// NEW WIRING - GY-VL53L0XV2 breakout (4 wires):
//
//   VIN  -> 5V   (the breakout has its own 2.8 V LDO + level
//                 shifters, so 5 V or 3V3 both work. 5 V is the
//                 steadier rail while the motors are pulling.)
//   GND  -> GND  (must be common with the motor driver ground)
//   SDA  -> GPIO 32
//   SCL  -> GPIO 33
//
//   XSHUT -> GPIO 4   (OPTIONAL - only if USE_XSHUT is set to 1
//                      below. It hardware-resets the sensor at
//                      boot, which cures the "not detected"
//                      hang after a messy reset.)
//   GPIO1 (the sensor's interrupt pad) -> leave unconnected.
//
// Pull-ups are already on the breakout - do not add your own.
//
// WHY NOT THE DEFAULT I2C PINS: the ESP32's default SDA/SCL are
// GPIO 21/22, and day2.ino has those on STBY and PWMA. Wire.begin()
// is therefore called with the alternative pair 32/33.
//
// FREED UP: the 5 IR line sensor pins (13, 14, 25, 26, 27) are not
// used in this activity - the MD0482 array can stay plugged in or
// come off, it makes no difference.
//
// ------------------------------------------------------------
// LIBRARY
// ------------------------------------------------------------
// Library Manager -> "VL53L0X" by Pololu.
//
// ------------------------------------------------------------
// HOW IT AVOIDS
// ------------------------------------------------------------
// The sensor is fixed, pointing straight forward - there is no
// servo - so the robot cannot see sideways while driving. It has
// to turn its whole body to look. That is the entire state machine:
//
//   CRUISE   drive forward, speed scaled by how much room is left
//   BRAKE    obstacle inside STOP_DISTANCE -> stop dead
//   BACKUP   reverse a little so the sensor is back in useful range
//            (the VL53L0X cannot measure closer than ~3 cm)
//   LOOK_L   pivot left, remember the best clearance seen
//   LOOK_R   pivot right past centre, remember the best clearance
//   COMMIT   pivot back to whichever side was more open, then go
//   ESCAPE   both sides blocked, or stuck repeatedly -> 180 spin
//
// The looks are timed pivots, so no encoders or IMU are needed.
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>


// ============================================================
// TB6612FNG CONNECTIONS (unchanged from day2.ino)
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
// VL53L0X CONNECTIONS (new)
// ============================================================

#define I2C_SDA_PIN 32
#define I2C_SCL_PIN 33

// Set to 1 only if you wire XSHUT to the pin below.
#define USE_XSHUT   0
#define XSHUT_PIN   4

VL53L0X sensor;


// ============================================================
// SENSOR TIMING
// ============================================================

// Time the sensor spends integrating one measurement.
// 20 ms is the fast preset: shorter range, less noise-immunity,
// but the loop reacts sooner. 33 ms is the library default.
const uint32_t TIMING_BUDGET_US = 20000;

// How often a new reading is pulled. Must be >= timing budget.
const unsigned long SENSOR_PERIOD_MS = 25;

// Anything past this is "open space" as far as steering cares.
// The VL53L0X gets unreliable on dark/angled surfaces beyond ~1.2 m.
const int MAX_USEFUL_MM = 1500;

// Below this the reading is physically meaningless (sensor blind
// zone) - treated as "something is right against the nose".
const int MIN_VALID_MM = 30;


// ============================================================
// DISTANCE THRESHOLDS
// ============================================================

// Obstacle this close -> stop and start avoiding.
const int STOP_DISTANCE_MM = 220;

// Start easing off the throttle from here down.
const int SLOW_DISTANCE_MM = 650;

// A direction counts as "open enough to drive into" only if it
// measures at least this far. Deliberately larger than
// STOP_DISTANCE_MM so the robot does not turn into a gap it will
// immediately have to back out of again.
const int CLEAR_DISTANCE_MM = 450;


// ============================================================
// SPEED SETTINGS
// ============================================================

int CRUISE_SPEED  = 200;   // full speed, open floor ahead
int CREEP_SPEED   = 120;   // speed just before STOP_DISTANCE
int BACKUP_SPEED  = 160;
int PIVOT_SPEED   = 170;   // spin-in-place speed while looking


// ============================================================
// MANOEUVRE TIMING
//
// Tune these to your robot: PIVOT_90_MS is the single most
// important one. Raise it if the robot under-turns, lower it if
// it swings too far. Everything else is derived from it.
// ============================================================

const unsigned long BRAKE_MS      = 120;   // settle before reversing
const unsigned long BACKUP_MS     = 350;   // reverse out of the blind zone
const unsigned long PIVOT_90_MS   = 420;   // ~90 deg spin in place
const unsigned long SETTLE_MS     = 90;    // pause so the ToF sees a still scene

// A look sweeps roughly 60 deg to one side.
const unsigned long LOOK_MS       = (PIVOT_90_MS * 2) / 3;

// Escape = roughly a 180.
const unsigned long ESCAPE_SPIN_MS = PIVOT_90_MS * 2;


// ============================================================
// STUCK DETECTION
//
// If the robot hits something again within STUCK_WINDOW_MS of
// finishing an avoid, that avoid did not work. STUCK_LIMIT of
// those in a row -> stop guessing and do a full 180.
// ============================================================

const unsigned long STUCK_WINDOW_MS = 2500;
const int           STUCK_LIMIT     = 3;

int           consecutiveHits   = 0;
unsigned long lastAvoidEndMs    = 0;


// ============================================================
// MOTOR SLEW LIMIT
//
// Same idea as the line follower: ramp the PWM instead of
// stepping it, so the robot does not lurch and skid every time
// the state machine changes its mind.
// ============================================================

const unsigned long CONTROL_PERIOD_MS   = 5;
const int           MAX_PWM_STEP_PER_TICK = 10;

unsigned long previousControlMs = 0;

int appliedLeftSpeed  = 0;
int appliedRightSpeed = 0;


// ============================================================
// DISTANCE FILTER
//
// 3-sample median. A median (not an average) is the right tool
// here: the VL53L0X's failure mode is the occasional wild single
// reading, and a median deletes it outright instead of smearing
// it across the next few samples.
// ============================================================

int distanceHistory[3] = { MAX_USEFUL_MM, MAX_USEFUL_MM, MAX_USEFUL_MM };
int distanceHistoryIdx = 0;

int currentDistance = MAX_USEFUL_MM;   // filtered, always valid

unsigned long previousSensorMs = 0;


// ============================================================
// STATE MACHINE
// ============================================================

enum RobotState {
  ST_CRUISE,      // driving forward
  ST_BRAKE,       // obstacle found, stopping
  ST_BACKUP,      // reversing out of the sensor blind zone
  ST_LOOK_LEFT,   // pivoting left, sampling clearance
  ST_LOOK_RIGHT,  // pivoting right through centre, sampling
  ST_COMMIT,      // pivoting onto the chosen heading
  ST_ESCAPE       // nowhere to go - 180 and try again
};

RobotState robotState = ST_CRUISE;
unsigned long stateStart = 0;

int bestLeftMm  = 0;
int bestRightMm = 0;

int commitDir = 1;                     // +1 = right, -1 = left
unsigned long commitDurationMs = 0;

// Direction of the next escape spin. Alternated on every escape so
// the robot cannot rock endlessly between the same two headings.
int escapeDir = 1;


// ============================================================
// DEBUG
// ============================================================

#define DEBUG_MODE 1
unsigned long previousDebugMs = 0;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void enterState(RobotState s);
void debugOutput();


// ============================================================
// LEFT MOTOR
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
// SPIN IN PLACE
//
// dir = +1 spins RIGHT (clockwise), dir = -1 spins LEFT.
// ============================================================

void pivot(int dir) {
  driveMotors(PIVOT_SPEED * dir, -PIVOT_SPEED * dir);
}


// ============================================================
// READ THE ToF SENSOR
//
// Returns a usable millimetre value, always. Out-of-range,
// timed-out and blind-zone readings are folded into the two
// extremes rather than being handed upwards as errors, so the
// steering code never has to special-case them:
//
//   timeout / no target   -> MAX_USEFUL_MM  (nothing out there)
//   below the blind zone  -> MIN_VALID_MM   (something touching)
// ============================================================

int readDistanceRaw() {

  int mm = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    return MAX_USEFUL_MM;
  }

  // 8190/8191 are the library's "no target in range" returns.
  if (mm >= 8000) {
    return MAX_USEFUL_MM;
  }

  if (mm < MIN_VALID_MM) {
    return MIN_VALID_MM;
  }

  if (mm > MAX_USEFUL_MM) {
    return MAX_USEFUL_MM;
  }

  return mm;
}


// ============================================================
// MEDIAN-OF-3 FILTER
// ============================================================

int readDistanceFiltered() {

  distanceHistory[distanceHistoryIdx] = readDistanceRaw();
  distanceHistoryIdx = (distanceHistoryIdx + 1) % 3;

  int a = distanceHistory[0];
  int b = distanceHistory[1];
  int c = distanceHistory[2];

  return max(min(a, b), min(max(a, b), c));
}


// ============================================================
// PRIME THE FILTER
//
// Fills all three slots so the first real decision is not made
// against two leftover initial values.
// ============================================================

void primeDistanceFilter() {
  for (int i = 0; i < 3; i++) {
    distanceHistory[i] = readDistanceRaw();
    delay(SENSOR_PERIOD_MS);
  }
  currentDistance = readDistanceFiltered();
}


// ============================================================
// FORWARD SPEED PROFILE
//
// Full speed with open floor ahead, tapering down to CREEP_SPEED
// as the obstacle reaches STOP_DISTANCE_MM. Continuous, so there
// is no visible step as the robot closes on a wall.
// ============================================================

int calculateCruiseSpeed(int distance) {

  if (distance >= SLOW_DISTANCE_MM) {
    return CRUISE_SPEED;
  }

  if (distance <= STOP_DISTANCE_MM) {
    return CREEP_SPEED;
  }

  float t = (float)(distance - STOP_DISTANCE_MM)
          / (float)(SLOW_DISTANCE_MM - STOP_DISTANCE_MM);

  return (int)(CREEP_SPEED + (CRUISE_SPEED - CREEP_SPEED) * t);
}


// ============================================================
// STATE HELPER
// ============================================================

void enterState(RobotState s) {
  robotState = s;
  stateStart = millis();
}


// ============================================================
// ENTER THE ESCAPE SPIN
// ============================================================

void enterEscape() {
  escapeDir = -escapeDir;
  enterState(ST_ESCAPE);
}


// ============================================================
// BEGIN AN AVOIDANCE MANOEUVRE
//
// Also decides whether this hit means the robot is stuck: hits
// that arrive soon after the previous avoid finished are counted,
// and enough of them in a row promotes the manoeuvre to a 180.
// ============================================================

void beginAvoid() {

  unsigned long nowMs = millis();

  if (lastAvoidEndMs != 0 && (nowMs - lastAvoidEndMs) < STUCK_WINDOW_MS) {
    consecutiveHits++;
  }
  else {
    consecutiveHits = 1;
  }

  enterState(ST_BRAKE);
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // ----- Motor driver -----
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);   // wake the TB6612FNG
  stopMotors();

  // ----- ToF sensor -----
#if USE_XSHUT
  // Hold the sensor in reset, then release it, so a half-configured
  // sensor left over from the previous run starts clean.
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(20);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);          // 400 kHz - the VL53L0X is happy here

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("ERROR: VL53L0X not detected.");
    Serial.println("Check SDA=32, SCL=33, GND shared, VIN powered.");
    while (1) {
      stopMotors();
      delay(200);
    }
  }

  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  sensor.startContinuous();

  primeDistanceFilter();

  Serial.println("\n======================================");
  Serial.println("OBSTACLE AVOIDER - VL53L0X ToF");
  Serial.print  ("Stop at   : "); Serial.print(STOP_DISTANCE_MM);  Serial.println(" mm");
  Serial.print  ("Slow from : "); Serial.print(SLOW_DISTANCE_MM);  Serial.println(" mm");
  Serial.print  ("Clear if  : "); Serial.print(CLEAR_DISTANCE_MM); Serial.println(" mm");
  Serial.println("======================================");

  previousControlMs = millis();
  previousSensorMs  = millis();
  enterState(ST_CRUISE);

  delay(1500);   // hands clear
}


// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void loop() {

  unsigned long nowMs = millis();

  // --------------------------------------------------------
  // SENSOR - fixed rate, independent of the control tick
  // --------------------------------------------------------
  if (nowMs - previousSensorMs >= SENSOR_PERIOD_MS) {
    previousSensorMs = nowMs;
    currentDistance = readDistanceFiltered();
  }

  // --------------------------------------------------------
  // CONTROL TICK
  // --------------------------------------------------------
  if (nowMs - previousControlMs < CONTROL_PERIOD_MS) {
    return;
  }
  previousControlMs = nowMs;

  unsigned long inState = nowMs - stateStart;

  switch (robotState) {

    // ------------------------------------------------------
    // CRUISE - open floor ahead
    // ------------------------------------------------------
    case ST_CRUISE: {

      if (currentDistance <= STOP_DISTANCE_MM) {
        beginAvoid();
        break;
      }

      int speed = calculateCruiseSpeed(currentDistance);
      driveMotors(speed, speed);
      break;
    }

    // ------------------------------------------------------
    // BRAKE - kill the momentum before reversing
    // ------------------------------------------------------
    case ST_BRAKE: {

      stopMotors();

      if (inState >= BRAKE_MS) {
        // Enough bad luck in a row means the simple left/right
        // choice is not working here. Turn around instead.
        if (consecutiveHits >= STUCK_LIMIT) {
          consecutiveHits = 0;
          enterEscape();
        }
        else {
          enterState(ST_BACKUP);
        }
      }
      break;
    }

    // ------------------------------------------------------
    // BACKUP
    //
    // Reversing is not just for clearance: at 22 cm the obstacle
    // is close to the ToF's useful minimum, and a wall filling
    // the field of view reads erratically. Backing off restores
    // a trustworthy measurement before the robot has to choose
    // a direction from one.
    // ------------------------------------------------------
    case ST_BACKUP: {

      driveMotors(-BACKUP_SPEED, -BACKUP_SPEED);

      if (inState >= BACKUP_MS) {
        bestLeftMm  = 0;
        bestRightMm = 0;
        enterState(ST_LOOK_LEFT);
      }
      break;
    }

    // ------------------------------------------------------
    // LOOK LEFT
    //
    // Pivot left and keep the FURTHEST reading seen during the
    // sweep. Furthest, not average: the robot is hunting for a
    // gap, and one clear bearing through a doorway is worth
    // more than a good average across a wall.
    // ------------------------------------------------------
    case ST_LOOK_LEFT: {

      // Hold still briefly at the start so the first samples are
      // not motion-blurred by the pivot.
      if (inState < SETTLE_MS) {
        stopMotors();
        break;
      }

      pivot(-1);

      if (currentDistance > bestLeftMm) {
        bestLeftMm = currentDistance;
      }

      if (inState >= SETTLE_MS + LOOK_MS) {
        enterState(ST_LOOK_RIGHT);
      }
      break;
    }

    // ------------------------------------------------------
    // LOOK RIGHT
    //
    // Sweeps back through the original heading and out to the
    // right, so it covers twice the arc of the left look. Only
    // the second half - once the robot is past centre - counts
    // as "right", otherwise the left side's clearance would be
    // credited to the right.
    // ------------------------------------------------------
    case ST_LOOK_RIGHT: {

      pivot(+1);

      if (inState > LOOK_MS && currentDistance > bestRightMm) {
        bestRightMm = currentDistance;
      }

      if (inState >= LOOK_MS * 2) {

        // The robot is now sitting LOOK_MS worth of rotation to
        // the right of its original heading.
        bool leftClear  = (bestLeftMm  >= CLEAR_DISTANCE_MM);
        bool rightClear = (bestRightMm >= CLEAR_DISTANCE_MM);

        if (!leftClear && !rightClear) {
          // Boxed in - both sweeps found a wall.
          enterEscape();
          break;
        }

        if (bestRightMm >= bestLeftMm) {
          // Already facing right; nothing more to turn.
          commitDir        = +1;
          commitDurationMs = 0;
        }
        else {
          // Swing back left, past centre, onto the left heading.
          commitDir        = -1;
          commitDurationMs = LOOK_MS * 2;
        }

#if DEBUG_MODE
        Serial.print(">>> LOOK  L=");
        Serial.print(bestLeftMm);
        Serial.print(" mm  R=");
        Serial.print(bestRightMm);
        Serial.print(" mm  -> going ");
        Serial.println(commitDir > 0 ? "RIGHT" : "LEFT");
#endif

        enterState(ST_COMMIT);
      }
      break;
    }

    // ------------------------------------------------------
    // COMMIT - turn onto the chosen heading and resume
    // ------------------------------------------------------
    case ST_COMMIT: {

      if (inState < commitDurationMs) {
        pivot(commitDir);
        break;
      }

      lastAvoidEndMs = nowMs;
      enterState(ST_CRUISE);
      break;
    }

    // ------------------------------------------------------
    // ESCAPE - dead end, or the same obstacle keeps winning
    // ------------------------------------------------------
    case ST_ESCAPE: {

      pivot(escapeDir);

      // Bail out early if a clear path opens up mid-spin - no
      // point completing a 180 into a wall when the gap is here.
      if (inState > PIVOT_90_MS / 2 && currentDistance >= SLOW_DISTANCE_MM) {
        consecutiveHits = 0;
        lastAvoidEndMs  = nowMs;
        enterState(ST_CRUISE);
        break;
      }

      if (inState >= ESCAPE_SPIN_MS) {
        consecutiveHits = 0;
        lastAvoidEndMs  = nowMs;
        enterState(ST_CRUISE);
      }
      break;
    }
  }

  debugOutput();
}


// ============================================================
// DEBUG OUTPUT
// ============================================================

const char* stateName() {
  switch (robotState) {
    case ST_CRUISE:     return "CRUISE";
    case ST_BRAKE:      return "BRAKE";
    case ST_BACKUP:     return "BACKUP";
    case ST_LOOK_LEFT:  return "LOOK-L";
    case ST_LOOK_RIGHT: return "LOOK-R";
    case ST_COMMIT:     return "COMMIT";
    case ST_ESCAPE:     return "ESCAPE";
  }
  return "?";
}

void debugOutput() {
#if DEBUG_MODE
  if (millis() - previousDebugMs < 100) return;
  previousDebugMs = millis();

  Serial.print(stateName());
  Serial.print(" | Dist: ");
  Serial.print(currentDistance);
  Serial.print(" mm | L: ");
  Serial.print(appliedLeftSpeed);
  Serial.print(" | R: ");
  Serial.print(appliedRightSpeed);
  Serial.print(" | hits: ");
  Serial.println(consecutiveHits);
#endif
}
