// ============================================================
// EA3121 Robotics Lab 02
// Activity 4 - Wall Following Robot (40 cm standoff)
//
// ESP32 + ST GY-VL53L0XV2 Time-of-Flight laser + TB6612FNG
//
// ------------------------------------------------------------
// WIRING - IDENTICAL TO ACTIVITIES 2 AND 3, NOTHING TO REWIRE
// ------------------------------------------------------------
//   STBY -> 21
//   PWMA -> 22   AIN1 -> 16   AIN2 -> 17     (LEFT  motor)
//   PWMB -> 23   BIN1 -> 18   BIN2 -> 19     (RIGHT motor)
//
//   VL53L0X   VIN -> 5V   GND -> GND
//             SDA -> GPIO 32
//             SCL -> GPIO 33
//
// NO NEW PINS. Only the sensor's AIM changes - see below.
//
// ------------------------------------------------------------
// WHERE TO POINT THE SENSOR  <-- THE ONE MECHANICAL DECISION
// ------------------------------------------------------------
// One fixed ToF cannot watch the wall and the path ahead at the
// same time, so the mounting angle decides what the robot can do.
// Set SENSOR_MOUNT_DEG below to match how you actually bolt it:
//
//   45 deg  (RECOMMENDED) - halfway between straight ahead and
//           the wall. The beam still measures the wall, and an
//           approaching corner shortens the range well before
//           the bumper reaches it, so the robot can turn in
//           time. It also self-corrects for heading: pointing
//           away from the wall lengthens the range immediately,
//           which acts like a free derivative term.
//
//   90 deg  (straight at the wall) - the textbook arrangement
//           and slightly more accurate on the standoff, but the
//           robot is BLIND AHEAD. It will follow a wall neatly
//           and then drive straight into the one in front of it.
//           Usable only if the course has no inner corners.
//
// Geometry, for a robot running parallel to the wall:
//
//   perpendicular distance = measured range * sin(mount angle)
//
// so a 45 deg mount reads 566 mm when the robot is the target
// 400 mm off the wall. The code does this conversion, and every
// distance below is a real perpendicular distance in mm.
//
// ------------------------------------------------------------
// LIBRARY
// ------------------------------------------------------------
// Library Manager -> "VL53L0X" by Pololu.
//
// ------------------------------------------------------------
// HOW IT FOLLOWS
// ------------------------------------------------------------
// PD control on the standoff error, plus a small state machine
// for the three things that break a plain controller:
//
//   FOLLOW        PD steering, speed backs off as error grows
//   INNER CORNER  wall ahead -> pivot away until it clears
//   OUTER CORNER  wall vanished -> curve around the edge
//   LOST          nothing anywhere -> hunt, then give up safely
//
// There is no integral term. On a wall follower the standing
// error comes from the robot's heading, not from a force it has
// to hold against, and D handles that. An I term here mostly
// winds up during corners and then overshoots on the way out.
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>


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
// VL53L0X CONNECTIONS (unchanged)
// ============================================================

#define I2C_SDA_PIN 32
#define I2C_SCL_PIN 33

#define USE_XSHUT   0
#define XSHUT_PIN   4

VL53L0X sensor;


// ============================================================
// OPTIONAL EXTRAS (same as Activity 3)
// ============================================================

#define USE_START_BUTTON 1
#define BUTTON_PIN       15     // momentary to GND, internal pull-up

#define STATUS_LED_PIN   2      // on-board LED, no wiring


// ============================================================
// GEOMETRY
// ============================================================

// Which side the wall is on: +1 = RIGHT, -1 = LEFT.
const int WALL_SIDE = +1;

// How far the sensor is rotated from straight ahead, toward the
// wall. 45 recommended, 90 = pointed straight at the wall.
const float SENSOR_MOUNT_DEG = 45.0;

// range -> perpendicular distance
const float PERP_FACTOR = sin(SENSOR_MOUNT_DEG * PI / 180.0);


// ============================================================
// THE TARGET
// ============================================================

// 40 cm from the wall, as specified.
const float TARGET_MM = 400.0;

// Errors smaller than this are treated as zero. Without it the
// robot chases every millimetre of sensor noise and visibly
// hunts left and right down a straight wall.
const float DEAD_BAND_MM = 15.0;


// ============================================================
// SENSOR LIMITS
// ============================================================

// Beyond this a reading is not trustworthy enough to steer on,
// and is treated as "no wall there".
const int MAX_RANGE_MM = 1600;

// Below the sensor's blind zone the number means nothing.
const int MIN_RANGE_MM = 30;

// ~30 Hz. Fast enough to steer on, and each measurement still
// gets a reasonable integration time.
const uint32_t TIMING_BUDGET_US = 33000;

// Control period. Matched to the sensor rate - running the
// controller faster than measurements arrive just differentiates
// the same number repeatedly and manufactures noise in D.
const unsigned long CONTROL_PERIOD_MS = 33;


// ============================================================
// PD CONTROLLER
//
// Error is in millimetres, output is in PWM counts.
//
// Kp = 0.30 means being 100 mm off the target produces 30 counts
// of steering. Start here; raise until the robot corners
// crisply, back off when it starts weaving on a straight.
// ============================================================

float Kp = 0.30;
float Kd = 0.12;

// Ceiling on the steering term, so a wild reading cannot spin
// the robot on the spot.
const float MAX_CORRECTION = 90.0;

// The derivative of a ToF signal is spiky. This smooths it.
const float DERIV_ALPHA = 0.70;


// ============================================================
// SPEED
// ============================================================

int CRUISE_SPEED = 175;   // on a straight, tracking well
int MIN_SPEED    = 120;   // when the error is large

// Error at which the speed has fallen all the way to MIN_SPEED.
const float SPEED_TAPER_MM = 250.0;

// Pivot speed for corner manoeuvres.
int PIVOT_SPEED = 165;

// The inner wheel may reverse this far on a hard correction.
const int INNER_WHEEL_MIN = -90;


// ============================================================
// CORNERS
// ============================================================

// A 45 deg mount sees an approaching front wall as a collapsing
// range. Below this perpendicular-equivalent distance, treat it
// as an inner corner and turn away from the wall.
//
// With a 90 deg mount this can never trigger - the beam is
// parallel to the wall ahead. That is the blind-ahead warning
// from the header, in code.
const float INNER_CORNER_MM = 260.0;

// Too close to keep steering out of - back off hard.
const float EMERGENCY_MM = 150.0;

// Consecutive no-return readings before the wall counts as
// genuinely gone rather than a dark patch or a bad angle.
const int LOST_READINGS = 4;

// Outer corner: curve toward the wall for up to this long while
// trying to pick it up again.
const unsigned long OUTER_TURN_MAX_MS = 2500;

// If the wall is still missing after this, stop.
const unsigned long LOST_GIVEUP_MS = 6000;

// Inner-corner pivot safety limit.
const unsigned long INNER_TURN_MAX_MS = 2000;


// ============================================================
// FILTERING
// ============================================================

// Light low-pass on the perpendicular distance. Heavier
// filtering than this adds lag that the D term then fights.
const float DIST_ALPHA = 0.45;

int  rangeHistory[3] = { 0, 0, 0 };
int  rangeHistoryIdx = 0;
bool historyPrimed   = false;

float filteredDistance = 0.0;
bool  filterPrimed     = false;


// ============================================================
// CONTROLLER STATE
// ============================================================

float previousError     = 0.0;
float filteredDerivative = 0.0;

unsigned long previousControlMs = 0;

int appliedLeft  = 0;
int appliedRight = 0;

int lostCount = 0;

bool sensorReady = false;
bool running     = false;   // false = parked, waiting for 's'

// Set false if GPIO 15 reads LOW at boot - see handleInput().
bool buttonUsable = true;

// Parked-state reminder, so a robot waiting for 's' cannot be
// mistaken for a robot that has crashed.
unsigned long previousIdleMsgMs = 0;


// ============================================================
// STATE MACHINE
// ============================================================

enum FollowState {
  ST_FOLLOW,
  ST_INNER_CORNER,   // wall ahead, pivot away from it
  ST_OUTER_CORNER,   // wall ended, curve around it
  ST_LOST,           // no wall anywhere
  ST_STOPPED
};

FollowState state = ST_FOLLOW;
unsigned long stateStart = 0;

const char* stateName() {
  switch (state) {
    case ST_FOLLOW:       return "FOLLOW";
    case ST_INNER_CORNER: return "INNER";
    case ST_OUTER_CORNER: return "OUTER";
    case ST_LOST:         return "LOST";
    case ST_STOPPED:      return "STOPPED";
  }
  return "?";
}

void enterState(FollowState s) {
  state = s;
  stateStart = millis();
}


// ============================================================
// DEBUG
// ============================================================

#define DEBUG_MODE 1
unsigned long previousDebugMs = 0;


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
// DRIVE / STOP
// ============================================================

void driveMotors(int leftSpeed, int rightSpeed) {
  appliedLeft  = constrain(leftSpeed,  -255, 255);
  appliedRight = constrain(rightSpeed, -255, 255);
  setLeftMotor(appliedLeft);
  setRightMotor(appliedRight);
}

void stopMotors() {
  driveMotors(0, 0);
}


// ============================================================
// STEER
//
// One place that turns a signed steering value into wheel
// speeds. Positive steer = turn RIGHT.
// ============================================================

void steer(int baseSpeed, float steerAmount) {

  int left  = baseSpeed + (int)steerAmount;
  int right = baseSpeed - (int)steerAmount;

  left  = constrain(left,  INNER_WHEEL_MIN, 255);
  right = constrain(right, INNER_WHEEL_MIN, 255);

  driveMotors(left, right);
}


// ============================================================
// PIVOT - dir +1 spins RIGHT, -1 spins LEFT
// ============================================================

void pivot(int dir) {
  driveMotors(PIVOT_SPEED * dir, -PIVOT_SPEED * dir);
}


// ============================================================
// READ THE WALL DISTANCE
//
// Returns the PERPENDICULAR distance to the wall in mm, or -1
// for "no wall in range". Everything downstream works in
// perpendicular millimetres, so the mounting angle stops
// mattering after this function.
// ============================================================

float readWallDistance() {

  int mm = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) return -1.0;
  if (mm >= 8000)               return -1.0;   // no target
  if (mm < MIN_RANGE_MM)        return -1.0;
  if (mm > MAX_RANGE_MM)        return -1.0;

  // ----- median of the last 3 raw ranges -----
  //
  // The VL53L0X fails by throwing the occasional wild value, and
  // a median deletes those outright. An average would smear each
  // one across the next several samples instead.
  rangeHistory[rangeHistoryIdx] = mm;
  rangeHistoryIdx = (rangeHistoryIdx + 1) % 3;

  if (!historyPrimed) {
    rangeHistory[0] = rangeHistory[1] = rangeHistory[2] = mm;
    historyPrimed = true;
  }

  int a = rangeHistory[0];
  int b = rangeHistory[1];
  int c = rangeHistory[2];
  int median = max(min(a, b), min(max(a, b), c));

  // ----- range -> perpendicular distance -----
  return median * PERP_FACTOR;
}


// ============================================================
// RESET THE CONTROLLER
// ============================================================

void resetController() {
  previousError      = 0.0;
  filteredDerivative = 0.0;
  filterPrimed       = false;
  historyPrimed      = false;
  lostCount          = 0;
}


// ============================================================
// SPEED PROFILE
//
// Full speed when tracking well, tapering to MIN_SPEED as the
// error grows. Slowing down while correcting is what keeps the
// robot from overshooting into the wall it is trying to hold.
// ============================================================

int speedForError(float error) {

  float t = fabs(error) / SPEED_TAPER_MM;
  t = constrain(t, 0.0, 1.0);

  return (int)(CRUISE_SPEED - (CRUISE_SPEED - MIN_SPEED) * t);
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n\n=== BOOT ===");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

#if USE_START_BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(10);   // let the pull-up settle before reading

  // A button wired open reads HIGH here. LOW means it is tied to
  // ground - a held button, a wiring mistake, or something else
  // on GPIO 15 - and the robot would read that as an endless
  // press. Disable it rather than let it jam the loop.
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonUsable = false;
    Serial.println("WARNING: GPIO 15 is LOW at boot - start button disabled.");
    Serial.println("         Use 's' on the Serial Monitor instead.");
  }
#endif

  // Motor driver held disabled until everything else is ready
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);
  stopMotors();

  // ----- ToF sensor -----
#if USE_XSHUT
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(20);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  sensor.setTimeout(500);

  for (int attempt = 1; attempt <= 3 && !sensorReady; attempt++) {
    if (sensor.init()) {
      sensorReady = true;
      break;
    }
    Serial.print("VL53L0X not detected (attempt ");
    Serial.print(attempt);
    Serial.println("/3)");
    delay(300);
  }

  if (sensorReady) {
    sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
    sensor.startContinuous();
    Serial.println("VL53L0X: ready");
  }
  else {
    // Not fatal: the serial console stays alive so the wiring can
    // be fixed and retried with 'i', no reflash needed.
    Serial.println("VL53L0X: FAILED - check SDA=32, SCL=33, GND shared, VIN powered");
    Serial.println("           fix the wiring, then send 'i' to retry");
  }

  Serial.println("\n======================================");
  Serial.println("WALL FOLLOWER");
  Serial.print  ("  Wall on the   : ");
  Serial.println(WALL_SIDE > 0 ? "RIGHT" : "LEFT");
  Serial.print  ("  Sensor mount  : ");
  Serial.print(SENSOR_MOUNT_DEG, 0);
  Serial.println(" deg from forward");
  Serial.print  ("  Target        : ");
  Serial.print(TARGET_MM, 0);
  Serial.println(" mm perpendicular");
  Serial.print  ("  Reads as      : ");
  Serial.print(TARGET_MM / PERP_FACTOR, 0);
  Serial.println(" mm of raw range when parallel");

  if (SENSOR_MOUNT_DEG > 80.0) {
    Serial.println("  WARNING: at ~90 deg the robot is blind ahead and");
    Serial.println("           cannot detect inner corners. Use 45 deg.");
  }

  Serial.println("--------------------------------------");
  Serial.println("  s = start/stop    i = re-init sensor");
  Serial.println("  m = motor self test (no sensor involved)");
#if USE_START_BUTTON
  Serial.println("  or press the button on GPIO 15");
#endif
  Serial.println("======================================");
  Serial.println("Parked. Send 's' to start following.");

  previousControlMs = millis();
  enterState(ST_FOLLOW);
}


// ============================================================
// START / STOP
// ============================================================

void startFollowing() {

  if (!sensorReady) {
    Serial.println("Cannot start: VL53L0X not initialised. Send 'i'.");
    return;
  }

  resetController();
  enterState(ST_FOLLOW);
  running = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.println("\n>>> FOLLOWING");
}

void stopFollowing(const char* why) {
  running = false;
  stopMotors();
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.print(">>> STOPPED: ");
  Serial.println(why);
}


// ============================================================
// RETRY THE SENSOR - send 'i'
// ============================================================

void retrySensorInit() {

  Serial.println("Re-initialising VL53L0X...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  sensor.setTimeout(500);

  if (sensor.init()) {
    sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
    sensor.startContinuous();
    sensorReady = true;
    Serial.println("VL53L0X: ready - send 's' to start");
  }
  else {
    sensorReady = false;
    Serial.println("VL53L0X: still not responding.");
    Serial.println("  SDA=32  SCL=33  GND shared with the ESP32  VIN powered");
  }
}


// ============================================================
// MOTOR SELF TEST - send 'm'
//
// Drives the motors directly, with no sensor and no state
// machine involved. If the wheels turn here but the robot does
// nothing when following, the problem is upstream of the motors
// (sensor, state, or never started). If they do not turn here
// either, it is power or wiring.
// ============================================================

void motorSelfTest() {

  Serial.println("\n--- MOTOR SELF TEST ---");
  Serial.println("(wheels off the ground, please)");

  digitalWrite(STBY_PIN, HIGH);   // in case something left it low
  delay(500);

  Serial.println("  LEFT forward");
  driveMotors(160, 0);
  delay(800);
  stopMotors();
  delay(400);

  Serial.println("  LEFT reverse");
  driveMotors(-160, 0);
  delay(800);
  stopMotors();
  delay(400);

  Serial.println("  RIGHT forward");
  driveMotors(0, 160);
  delay(800);
  stopMotors();
  delay(400);

  Serial.println("  RIGHT reverse");
  driveMotors(0, -160);
  delay(800);
  stopMotors();
  delay(400);

  Serial.println("  BOTH forward");
  driveMotors(160, 160);
  delay(1000);
  stopMotors();

  Serial.println("--- test done ---");
  Serial.println("  No movement at all -> power or TB6612 wiring.");
  Serial.println("  One motor only     -> that motor's AIN/BIN pins.");
  Serial.println("  Both fine          -> send 's' to start following.\n");
}


// ============================================================
// DEBUG OUTPUT
// ============================================================

void debugOutput(float distance, float error, float derivative, float steerAmount) {
#if DEBUG_MODE
  if (millis() - previousDebugMs < 100) return;
  previousDebugMs = millis();

  Serial.print(stateName());
  Serial.print(" | D: ");
  if (distance > 0) Serial.print(distance, 0);
  else              Serial.print("----");
  Serial.print(" mm | Err: ");
  Serial.print(error, 0);
  Serial.print(" | dE: ");
  Serial.print(derivative, 0);
  Serial.print(" | Steer: ");
  Serial.print(steerAmount, 0);
  Serial.print(" | L: ");
  Serial.print(appliedLeft);
  Serial.print(" | R: ");
  Serial.println(appliedRight);
#endif
}


// ============================================================
// SERIAL / BUTTON INPUT
// ============================================================

void handleInput() {

  if (Serial.available()) {
    char c = Serial.read();

    if (c == 's' || c == 'S') {
      if (running) stopFollowing("commanded");
      else         startFollowing();
    }
    else if (c == 'i' || c == 'I') {
      retrySensorInit();
    }
    else if (c == 'm' || c == 'M') {
      if (running) stopFollowing("motor test");
      motorSelfTest();
    }
  }

#if USE_START_BUTTON
  // buttonUsable is false when GPIO 15 read LOW at boot, which
  // means it is wired to ground (or to nothing that pulls it up).
  // Without this guard the release-wait below never returns and
  // the robot sits there looking dead.
  if (buttonUsable && digitalRead(BUTTON_PIN) == LOW) {
    delay(30);                                   // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {

      // Bounded wait: a stuck-low pin must not hang the loop.
      unsigned long waitStart = millis();
      while (digitalRead(BUTTON_PIN) == LOW &&
             millis() - waitStart < 2000) {
        delay(10);
      }

      if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button on GPIO 15 is stuck LOW - ignoring it.");
        Serial.println("  Unwire it, or set USE_START_BUTTON to 0.");
        buttonUsable = false;
      }
      else if (running) stopFollowing("button");
      else              startFollowing();
    }
  }
#endif
}


// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void loop() {

  handleInput();

  if (!running) {
    stopMotors();

    // Say so every 3 s. A parked robot and a crashed robot look
    // identical from across the room; this tells them apart, and
    // says why it will not start if the sensor is the reason.
    if (millis() - previousIdleMsgMs >= 3000) {
      previousIdleMsgMs = millis();
      if (sensorReady) {
        Serial.println("PARKED - send 's' (or press the button) to start following");
      }
      else {
        Serial.println("PARKED - sensor not ready. 'i' retries it, 'm' tests the motors");
      }
    }

    delay(10);
    return;
  }

  // ----- fixed-rate control tick -----
  unsigned long nowMs = millis();

  if (nowMs - previousControlMs < CONTROL_PERIOD_MS) {
    return;
  }

  float dt = (nowMs - previousControlMs) / 1000.0;
  dt = constrain(dt, 0.010, 0.200);
  previousControlMs = nowMs;

  unsigned long inState = nowMs - stateStart;

  // ----- measure -----
  float distance = readWallDistance();
  bool  haveWall = (distance > 0);

  if (haveWall) {
    lostCount = 0;

    if (!filterPrimed) {
      filteredDistance = distance;
      filterPrimed = true;
    }
    else {
      filteredDistance = (DIST_ALPHA * filteredDistance)
                       + ((1.0 - DIST_ALPHA) * distance);
    }
  }
  else {
    lostCount++;
  }

  switch (state) {

    // --------------------------------------------------------
    // FOLLOW - PD on the standoff error
    // --------------------------------------------------------
    case ST_FOLLOW: {

      // Wall gone for several readings in a row: an outer corner,
      // a doorway, or the end of the wall.
      if (lostCount >= LOST_READINGS) {
        enterState(ST_OUTER_CORNER);
        Serial.println(">>> wall lost - curving around the corner");
        break;
      }

      if (!haveWall) {
        // A single dropout. Hold the last command rather than
        // reacting - one bad reading is not news.
        break;
      }

      // Something is close enough ahead that steering will not
      // clear it. Only reachable with an angled mount.
      if (filteredDistance < INNER_CORNER_MM) {
        enterState(ST_INNER_CORNER);
        Serial.println(">>> inner corner - turning away");
        break;
      }

      // ----- error -----
      float error = filteredDistance - TARGET_MM;   // + = too far

      if (fabs(error) < DEAD_BAND_MM) {
        error = 0.0;
      }

      // ----- derivative -----
      float rawDerivative = (error - previousError) / dt;
      previousError = error;

      filteredDerivative = (DERIV_ALPHA * filteredDerivative)
                         + ((1.0 - DERIV_ALPHA) * rawDerivative);

      // ----- PD -----
      float correction = (Kp * error) + (Kd * filteredDerivative);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

      // Too far from the wall -> steer toward it. WALL_SIDE turns
      // that into a direction: +1 (right wall) means steer right.
      float steerAmount = WALL_SIDE * correction;

      // ----- emergency: about to scrape the wall -----
      if (filteredDistance < EMERGENCY_MM) {
        pivot(-WALL_SIDE);          // spin away from the wall
        debugOutput(distance, error, filteredDerivative, 0);
        break;
      }

      steer(speedForError(error), steerAmount);
      debugOutput(distance, error, filteredDerivative, steerAmount);
      break;
    }

    // --------------------------------------------------------
    // INNER CORNER - wall ahead
    //
    // Pivot away from the wall until the range opens back up
    // past the target. Pivoting rather than curving because
    // there is, by definition, no room ahead to curve into.
    // --------------------------------------------------------
    case ST_INNER_CORNER: {

      pivot(-WALL_SIDE);

      // Turned far enough that the wall is off to the side again
      if (haveWall && filteredDistance > TARGET_MM * 0.9) {
        resetController();
        enterState(ST_FOLLOW);
        Serial.println(">>> corner cleared");
        break;
      }

      // Pivoted past the corner entirely and lost the wall
      if (lostCount >= LOST_READINGS) {
        resetController();
        enterState(ST_FOLLOW);
        break;
      }

      if (inState >= INNER_TURN_MAX_MS) {
        enterState(ST_LOST);
      }

      debugOutput(distance, 0, 0, 0);
      break;
    }

    // --------------------------------------------------------
    // OUTER CORNER - the wall ended
    //
    // Drive forward while curving toward where the wall was. A
    // curve, not a pivot: the corner is behind the sensor by the
    // time it disappears, so the robot has to travel a little
    // before the new face comes into view.
    // --------------------------------------------------------
    case ST_OUTER_CORNER: {

      if (haveWall) {
        // Only accept the wall back once it is at a plausible
        // distance, otherwise a glancing reflection off the floor
        // pulls the robot back out of the turn too early.
        if (filteredDistance < TARGET_MM * 2.0) {
          resetController();
          enterState(ST_FOLLOW);
          Serial.println(">>> wall reacquired");
          break;
        }
      }

      // Gentle curve toward the wall side
      steer(MIN_SPEED, WALL_SIDE * 55.0);

      if (inState >= OUTER_TURN_MAX_MS) {
        enterState(ST_LOST);
        Serial.println(">>> no wall after the corner");
      }

      debugOutput(distance, 0, 0, 55.0);
      break;
    }

    // --------------------------------------------------------
    // LOST - nothing to follow
    // --------------------------------------------------------
    case ST_LOST: {

      if (haveWall && filteredDistance < MAX_RANGE_MM * PERP_FACTOR) {
        resetController();
        enterState(ST_FOLLOW);
        Serial.println(">>> wall found again");
        break;
      }

      // Creep forward with a slight bias toward the wall side.
      steer(MIN_SPEED, WALL_SIDE * 30.0);

      if (inState >= LOST_GIVEUP_MS) {
        stopFollowing("no wall found");
        enterState(ST_STOPPED);
      }

      debugOutput(distance, 0, 0, 30.0);
      break;
    }

    // --------------------------------------------------------
    // STOPPED
    // --------------------------------------------------------
    case ST_STOPPED: {
      stopMotors();
      break;
    }
  }
}
