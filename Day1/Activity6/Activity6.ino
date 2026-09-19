// Differential-drive odometry with closed-loop distance commands over Bluetooth.
//
// Two N20 gearmotors + quadrature encoders, driven by a TB6612FNG on an ESP32.
// Encoder ticks are decoded in x4 quadrature by interrupt, integrated into a
// dead-reckoned pose (X, Y, theta), and used to close the loop on distance so
// that "F 10" drives exactly 10 cm forward.
//
// The link is Bluetooth Classic SPP, which is what the Android app "Arduino
// Bluetooth Controller" speaks. Pair the board in Android's Bluetooth settings
// first, then connect to it from the app.
//
// Commands (case insensitive, one per line):
//   F <cm>    drive forward that many centimetres      e.g.  F 10
//   B <cm>    drive backward that many centimetres     e.g.  B 25.5
//   L <deg>   spin left in place by that many degrees  e.g.  L 90
//   R <deg>   spin right in place by that many degrees
//   S         stop immediately
//   P         print the current pose
//   Z         zero the pose and encoder counts
//   0-9       speed, 0 = slowest, 9 = fastest (applies to the next move)
//
// A bare F, B, L or R with no number moves one default step, so the app's
// button/controller mode works with plain single-character bindings.

#include <BluetoothSerial.h>
#include <math.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth Classic is not available on this board. Select a plain ESP32.
#endif

// Name shown in Android's Bluetooth pairing list
#define DEVICE_NAME "24SEA052_Robo"

// Step size used when a direction command arrives without a number.
const float DEFAULT_STEP_CM  = 10.0;
const float DEFAULT_TURN_DEG = 90.0;

// ---------------------------------------------------------------- wiring ---

// Motor A wiring (left wheel)
const int AIN1 = 12;
const int AIN2 = 14;
const int PWMA = 4;

// Motor B wiring (right wheel)
const int BIN1 = 26;
const int BIN2 = 27;
const int PWMB = 16;

// Encoder channels. These four pins are interrupt capable, have usable
// internal pull-ups, and are not strapping or flash pins on the ESP32.
const int LEFT_ENC_A  = 32;
const int LEFT_ENC_B  = 33;
const int RIGHT_ENC_A = 18;
const int RIGHT_ENC_B = 19;

// The two motors face opposite ways on the chassis, so "forward" is a
// different rotation for each. Flip these if a wheel spins the wrong way.
const bool LEFT_FORWARD_CW  = true;
const bool RIGHT_FORWARD_CW = false;

// Set true if a wheel's count goes down while it drives the robot forward
// (the A/B channels are swapped relative to the other side).
const bool LEFT_ENC_INVERT  = false;
const bool RIGHT_ENC_INVERT = true;

// ----------------------------------------------------- robot calibration ---

// Measure these on your own robot, they set the accuracy of everything below.
const float WHEEL_DIAMETER_MM = 46.2;   // rubber-on-rubber, measure loaded
const float WHEEL_BASE_MM     = 140.0;   // distance between the wheel contact patches
const float ENCODER_PPR       = 204.0;    // pulses per motor-shaft revolution, one channel
const float GEAR_RATIO        = 30.0;   // motor turns per wheel turn

const float COUNTS_PER_WHEEL_REV = ENCODER_PPR * 4.0 * GEAR_RATIO;   // x4 quadrature
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_WHEEL_REV;

// ------------------------------------------------------ control tuning -----

const int MIN_PWM = 90;                 // N20s stall below roughly a third duty
const int MAX_PWM = 255;
const float RAMP_DOWN_MM  = 30.0;       // start easing off this far from target
const unsigned long RAMP_UP_MS = 250;   // soft start, avoids wheel slip
const float KP_STRAIGHT = 0.6;          // corrects left/right count mismatch
const int MAX_CORRECTION = 60;
const unsigned long CONTROL_PERIOD_MS = 20;    // 50 Hz odometry and control
const unsigned long TELEMETRY_PERIOD_MS = 250;
const unsigned long SETTLE_MS = 150;    // let the robot coast to rest before reporting
const unsigned long STALL_TIMEOUT_MS = 600;    // no encoder movement while driving

// --------------------------------------------------------------- state -----

// Encoder counts, written only by the ISRs. A 32-bit aligned read is atomic on
// the ESP32, so the control loop can sample them without disabling interrupts.
volatile long leftCount  = 0;
volatile long rightCount = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;

// Quadrature transition table, indexed by (previous state << 2) | new state.
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Pose in millimetres and radians, integrated from the encoder counts.
float poseX = 0.0;
float poseY = 0.0;
float poseTheta = 0.0;
long lastOdomLeft = 0;
long lastOdomRight = 0;

int speedLevel = 5;

enum MotionState { IDLE, DRIVING, SETTLING };
MotionState motionState = IDLE;

int moveLeftDir = 0;                    // +1 / -1, wheel direction for this move
int moveRightDir = 0;
float moveTargetMM = 0.0;               // arc length each wheel must cover
long moveStartLeft = 0;
long moveStartRight = 0;
const char *moveLabel = "";
float moveRequest = 0.0;                // what the user asked for, cm or degrees
bool moveIsTurn = false;

unsigned long moveStartMs = 0;
unsigned long moveTimeoutMs = 0;
unsigned long settleStartMs = 0;
unsigned long lastProgressMs = 0;
float lastProgressMM = 0.0;
unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;

BluetoothSerial SerialBT;
bool phoneConnected = false;

// Commands arrive as a line, but not every terminal app appends a newline, so
// a short idle gap also ends the line.
char inputBuffer[24];
size_t inputLength = 0;
unsigned long lastInputMs = 0;
const unsigned long INPUT_IDLE_MS = 150;

// ------------------------------------------------------------ encoders -----

void IRAM_ATTR leftEncoderISR() {
  uint8_t state = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  leftCount += QUAD_TABLE[(leftPrevState << 2) | state];
  leftPrevState = state;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  rightCount += QUAD_TABLE[(rightPrevState << 2) | state];
  rightPrevState = state;
}

// Counts with the sign convention "positive means this wheel rolled forward".
long leftTicks() {
  return LEFT_ENC_INVERT ? -leftCount : leftCount;
}

long rightTicks() {
  return RIGHT_ENC_INVERT ? -rightCount : rightCount;
}

// -------------------------------------------------------------- motors -----

// direction: 1 = forward, -1 = backward, 0 = stop
void leftMotor(int direction, int speed) {
  bool clockwise = (direction > 0) ? LEFT_FORWARD_CW : !LEFT_FORWARD_CW;
  digitalWrite(AIN1, direction == 0 ? LOW : (clockwise ? HIGH : LOW));
  digitalWrite(AIN2, direction == 0 ? LOW : (clockwise ? LOW : HIGH));
  analogWrite(PWMA, direction == 0 ? 0 : speed);
}

void rightMotor(int direction, int speed) {
  bool clockwise = (direction > 0) ? RIGHT_FORWARD_CW : !RIGHT_FORWARD_CW;
  digitalWrite(BIN1, direction == 0 ? LOW : (clockwise ? HIGH : LOW));
  digitalWrite(BIN2, direction == 0 ? LOW : (clockwise ? LOW : HIGH));
  analogWrite(PWMB, direction == 0 ? 0 : speed);
}

void stopMotors() {
  leftMotor(0, 0);
  rightMotor(0, 0);
}

int pwmForLevel(int level) {
  return MIN_PWM + (MAX_PWM - MIN_PWM) * level / 9;
}

// ------------------------------------------------------------ reporting ----

void reply(const char *message) {
  Serial.println(message);
  if (SerialBT.hasClient()) {
    // The app splits its terminal on newlines, so end every message with one.
    SerialBT.println(message);
  }
}

void reportPose() {
  char status[96];
  snprintf(status, sizeof(status), "X=%.1fcm Y=%.1fcm TH=%.1fdeg",
           poseX / 10.0, poseY / 10.0, poseTheta * 180.0 / PI);
  reply(status);
}

// ------------------------------------------------------------ odometry -----

// Standard differential-drive dead reckoning, evaluated at the midpoint of the
// heading change so that arcs are followed more closely than a straight-line
// approximation would allow.
void updateOdometry() {
  long left = leftTicks();
  long right = rightTicks();

  float dLeft  = (left - lastOdomLeft) * MM_PER_COUNT;
  float dRight = (right - lastOdomRight) * MM_PER_COUNT;
  lastOdomLeft = left;
  lastOdomRight = right;

  if (dLeft == 0.0 && dRight == 0.0) {
    return;
  }

  float dCentre = (dLeft + dRight) / 2.0;
  float dTheta  = (dRight - dLeft) / WHEEL_BASE_MM;

  poseX += dCentre * cos(poseTheta + dTheta / 2.0);
  poseY += dCentre * sin(poseTheta + dTheta / 2.0);
  poseTheta += dTheta;

  // Keep theta in (-pi, pi]
  while (poseTheta > PI)  poseTheta -= 2.0 * PI;
  while (poseTheta <= -PI) poseTheta += 2.0 * PI;
}

void resetPose() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
  lastOdomLeft = 0;
  lastOdomRight = 0;
  poseX = 0.0;
  poseY = 0.0;
  poseTheta = 0.0;
}

// ------------------------------------------------------- motion control ----

// How far along the commanded move each wheel has come, in millimetres.
float moveProgressMM() {
  long left = (leftTicks() - moveStartLeft) * moveLeftDir;
  long right = (rightTicks() - moveStartRight) * moveRightDir;
  return ((left + right) / 2.0) * MM_PER_COUNT;
}

void abortMove(const char *reason) {
  stopMotors();
  motionState = IDLE;
  reply(reason);
  reportPose();
}

void finishMove() {
  stopMotors();
  motionState = SETTLING;
  settleStartMs = millis();
}

void startMove(const char *label, int leftDir, int rightDir, float perWheelMM,
               bool isTurn, float request) {
  moveLabel = label;
  moveLeftDir = leftDir;
  moveRightDir = rightDir;
  moveTargetMM = perWheelMM;
  moveIsTurn = isTurn;
  moveRequest = request;
  moveStartLeft = leftTicks();
  moveStartRight = rightTicks();
  moveStartMs = millis();
  lastProgressMs = moveStartMs;
  lastProgressMM = 0.0;
  // Generous allowance so a slow crawl is not mistaken for a fault; a genuine
  // stall is caught much sooner by the no-movement check.
  moveTimeoutMs = 3000 + (unsigned long)(perWheelMM * 40.0);
  motionState = DRIVING;

  char status[96];
  snprintf(status, sizeof(status), "%s %.1f%s @ speed %d",
           label, request, isTurn ? "deg" : "cm", speedLevel);
  reply(status);
}

void commandDistance(char direction, float centimetres) {
  if (centimetres <= 0.0) {
    reply("Distance must be > 0");
    return;
  }
  int dir = (direction == 'F') ? 1 : -1;
  startMove(dir > 0 ? "Forward" : "Backward", dir, dir, centimetres * 10.0,
            false, centimetres);
}

void commandTurn(char direction, float degrees) {
  if (degrees <= 0.0) {
    reply("Angle must be > 0");
    return;
  }
  // Spinning in place: each wheel travels along a circle of radius WHEEL_BASE/2.
  float arcMM = (degrees * PI / 180.0) * (WHEEL_BASE_MM / 2.0);
  if (direction == 'L') {
    startMove("Left", -1, 1, arcMM, true, degrees);
  } else {
    startMove("Right", 1, -1, arcMM, true, degrees);
  }
}

void runMotionControl() {
  if (motionState == SETTLING) {
    if (millis() - settleStartMs >= SETTLE_MS) {
      motionState = IDLE;
      char status[96];
      snprintf(status, sizeof(status), "%s done: %.1f%s travelled %.1fcm",
               moveLabel, moveRequest, moveIsTurn ? "deg" : "cm",
               moveProgressMM() / 10.0);
      reply(status);
      reportPose();
    }
    return;
  }

  if (motionState != DRIVING) {
    return;
  }

  unsigned long now = millis();
  float progress = moveProgressMM();
  float remaining = moveTargetMM - progress;

  if (remaining <= 0.0) {
    finishMove();
    return;
  }

  if (now - moveStartMs > moveTimeoutMs) {
    abortMove("Timeout, move aborted");
    return;
  }

  if (fabs(progress - lastProgressMM) > 0.5) {
    lastProgressMM = progress;
    lastProgressMs = now;
  } else if (now - lastProgressMs > STALL_TIMEOUT_MS) {
    abortMove("Stalled, move aborted (check encoders and battery)");
    return;
  }

  int base = pwmForLevel(speedLevel);

  // Soft start, so the wheels do not slip and lose distance at the very start.
  unsigned long elapsed = now - moveStartMs;
  if (elapsed < RAMP_UP_MS) {
    base = MIN_PWM + (int)((base - MIN_PWM) * (float)elapsed / (float)RAMP_UP_MS);
  }

  // Ease off near the target so the robot stops on the mark instead of past it.
  if (remaining < RAMP_DOWN_MM) {
    int eased = MIN_PWM + (int)((base - MIN_PWM) * (remaining / RAMP_DOWN_MM));
    base = min(base, eased);
  }
  base = constrain(base, MIN_PWM, MAX_PWM);

  // Proportional correction on the mismatch between the two wheels: whichever
  // wheel is ahead gets slowed down. This keeps straight lines straight and
  // turns centred on the robot.
  long leftProgress = (leftTicks() - moveStartLeft) * moveLeftDir;
  long rightProgress = (rightTicks() - moveStartRight) * moveRightDir;
  int correction = (int)(KP_STRAIGHT * (leftProgress - rightProgress));
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int leftPwm  = constrain(base - correction, MIN_PWM, MAX_PWM);
  int rightPwm = constrain(base + correction, MIN_PWM, MAX_PWM);

  leftMotor(moveLeftDir, leftPwm);
  rightMotor(moveRightDir, rightPwm);

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    char status[96];
    snprintf(status, sizeof(status), "  %.1f/%.1fcm X=%.1f Y=%.1f TH=%.0f",
             progress / 10.0, moveTargetMM / 10.0,
             poseX / 10.0, poseY / 10.0, poseTheta * 180.0 / PI);
    reply(status);
  }
}

// ------------------------------------------------------- command parsing ---

void executeLine(char *line) {
  // Find the command letter.
  char *p = line;
  while (*p == ' ' || *p == '\t') p++;
  if (*p == '\0') return;

  char command = *p++;
  if (command >= 'a' && command <= 'z') command -= 32;

  while (*p == ' ' || *p == '\t' || *p == ',' || *p == '=') p++;
  bool hasValue = (*p != '\0');
  float value = hasValue ? atof(p) : 0.0;

  // A bare single digit sets the speed.
  if (command >= '0' && command <= '9' && !hasValue) {
    speedLevel = command - '0';
    char status[48];
    snprintf(status, sizeof(status), "Speed %d (PWM %d)", speedLevel, pwmForLevel(speedLevel));
    reply(status);
    return;
  }

  switch (command) {
    case 'F':
    case 'B':
      if (motionState != IDLE) {
        reply("Busy, send S first");
        return;
      }
      commandDistance(command, hasValue ? value : DEFAULT_STEP_CM);
      break;

    case 'L':
    case 'R':
      if (motionState != IDLE) {
        reply("Busy, send S first");
        return;
      }
      commandTurn(command, hasValue ? value : DEFAULT_TURN_DEG);
      break;

    case 'S':
      stopMotors();
      motionState = IDLE;
      reply("Stopped");
      reportPose();
      break;

    case 'P':
      reportPose();
      break;

    case 'Z':
      if (motionState != IDLE) {
        reply("Busy, send S first");
        return;
      }
      resetPose();
      reply("Pose reset");
      reportPose();
      break;

    default:
      reply("Commands: F<cm> B<cm> L<deg> R<deg> S P Z 0-9");
      break;
  }
}

void flushInput() {
  if (inputLength == 0) return;
  inputBuffer[inputLength] = '\0';
  inputLength = 0;
  executeLine(inputBuffer);
}

void feedInput(char c) {
  lastInputMs = millis();
  if (c == '\n' || c == '\r' || c == ';') {
    flushInput();
    return;
  }
  if (inputLength < sizeof(inputBuffer) - 1) {
    inputBuffer[inputLength++] = c;
  }
}

// ------------------------------------------------------- link monitoring ---

// Watches the SPP link so the robot never keeps driving after the phone drops
// out of range or the app disconnects.
void updateConnection() {
  bool connected = SerialBT.hasClient();
  if (connected == phoneConnected) {
    return;
  }
  phoneConnected = connected;

  if (connected) {
    Serial.println("Phone connected");
    reply("Ready. F<cm> B<cm> L<deg> R<deg> S P Z 0-9");
    return;
  }

  Serial.println("Phone disconnected, motors stopped");
  stopMotors();
  motionState = IDLE;
  inputLength = 0;                // drop any half-received command
}

// ----------------------------------------------------------------- setup ---

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  stopMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  leftPrevState  = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  rightPrevState = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  // Both edges of both channels: full x4 quadrature resolution.
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightEncoderISR, CHANGE);

  resetPose();

  SerialBT.begin(DEVICE_NAME);    // Bluetooth Classic SPP, slave mode

  Serial.printf("%.4f mm per count, %.0f counts per wheel revolution\n",
                MM_PER_COUNT, COUNTS_PER_WHEEL_REV);
  Serial.println("Discoverable as " DEVICE_NAME ", waiting for a phone...");
}

void loop() {
  updateConnection();

  while (SerialBT.available()) {
    feedInput((char)SerialBT.read());
  }

  // The same parser serves the USB serial monitor, for testing without a phone.
  if (Serial.available()) {
    feedInput((char)Serial.read());
  }

  // Terminal apps that do not append a newline are handled by the idle gap.
  if (inputLength > 0 && millis() - lastInputMs > INPUT_IDLE_MS) {
    flushInput();
  }

  unsigned long now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    updateOdometry();
    runMotionControl();
  }
}
