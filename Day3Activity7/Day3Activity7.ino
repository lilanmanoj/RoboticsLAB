/*
 * Day3Activity7.ino
 * Activity 7 - Command Robot Velocity  (lab PDF p.28)
 *
 * Commands the robot as a whole - linear velocity v and angular velocity w -
 * and lets the differential-drive kinematics work out what each wheel must do:
 *
 *     vR = v + (L/2) w
 *     vL = v - (L/2) w
 *     w_wheel = v_wheel / r
 *     RPM = w_wheel * 60 / (2 pi)
 *
 * The resulting RPM values are the setpoints for the Activity 6 wheel
 * controllers. Encoder feedback is then run back through the forward
 * kinematics to report the v and w actually achieved, which is the
 * verification step the activity asks for.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"
#include "VelocityPI.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

// Saturation limits. V_WHEEL_MAX comes from Activity 1: the wheels reached
// about 400 RPM, which at a 43 mm wheel is ~0.90 m/s. Keep a margin so the
// controllers retain authority to correct.
const float V_WHEEL_MAX = 0.60f;   // m/s per wheel

VelocityPI ctrl[2];
float kp = 0.35f;
float ki = 2.50f;

float cmdV = 0.0f;   // desired robot linear velocity, m/s
float cmdW = 0.0f;   // desired robot angular velocity, rad/s

float wheelSetMps[2] = {0.0f, 0.0f};
float wheelSetRpm[2] = {0.0f, 0.0f};
bool saturating = false;
bool logging = false;

char cmdBuf[32];
uint8_t cmdLen = 0;

/*
 * Inverse kinematics with proportional saturation.
 *
 * When a demand exceeds what the wheels can deliver, both wheel speeds are
 * scaled by the SAME factor rather than clipped individually. Clipping would
 * change the ratio between them, which changes w - the robot would turn by a
 * different amount than commanded. Scaling preserves the path and only slows
 * the robot along it.
 */
void computeWheelSetpoints() {
  const float half = WHEEL_BASE_M / 2.0f;
  float vR = cmdV + half * cmdW;
  float vL = cmdV - half * cmdW;

  const float peak = max(fabsf(vL), fabsf(vR));
  saturating = peak > V_WHEEL_MAX;
  if (saturating && peak > 0.0f) {
    const float scale = V_WHEEL_MAX / peak;
    vL *= scale;
    vR *= scale;
  }

  wheelSetMps[MOTOR_LEFT] = vL;
  wheelSetMps[MOTOR_RIGHT] = vR;
  wheelSetRpm[MOTOR_LEFT] = mpsToRpm(vL);
  wheelSetRpm[MOTOR_RIGHT] = mpsToRpm(vR);
}

// Forward kinematics on the measured wheel speeds (PDF p.30).
float measuredV() {
  return (wheel[MOTOR_RIGHT].mps + wheel[MOTOR_LEFT].mps) / 2.0f;
}

float measuredW() {
  return (wheel[MOTOR_RIGHT].mps - wheel[MOTOR_LEFT].mps) / WHEEL_BASE_M;
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  v<m/s>   desired linear velocity,  e.g. v0.20");
  Serial.println("  w<rad/s> desired angular velocity, e.g. w0.8 (+ = left turn)");
  Serial.println("  c<v>,<w> both at once, e.g. c0.20,0.8");
  Serial.println("  k<Kp>    proportional gain");
  Serial.println("  i<Ki>    integral gain");
  Serial.println("  s        stop");
  Serial.println("  g        toggle CSV logging");
  Serial.println("  h        this help");
  Serial.println();
}

void printCsvHeader() {
  Serial.println("t_ms,cmd_v,cmd_w,set_mps_l,set_mps_r,set_rpm_l,set_rpm_r,"
                 "rpm_l,rpm_r,mps_l,mps_r,meas_v,meas_w,pwm_l,pwm_r");
}

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'v': cmdV = arg; break;
    case 'w': cmdW = arg; break;
    case 'c': {
      const char *comma = strchr(cmd, ',');
      cmdV = arg;
      cmdW = comma ? atof(comma + 1) : 0.0f;
      break;
    }
    case 'k':
      kp = max(0.0f, arg);
      ctrl[0].kp = ctrl[1].kp = kp;
      break;
    case 'i':
      ki = max(0.0f, arg);
      ctrl[0].ki = ctrl[1].ki = ki;
      break;
    case 's':
      cmdV = 0.0f;
      cmdW = 0.0f;
      piReset(ctrl[0]);
      piReset(ctrl[1]);
      motorsCoast();
      break;
    case 'g':
      logging = !logging;
      Serial.println();
      if (logging) printCsvHeader();
      break;
    default: printHelp(); break;
  }
  computeWheelSetpoints();
}

void pollSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void display() {
  Serial.printf("\033[6A");
  Serial.printf(" commanded  v %6.3f m/s   w %6.3f rad/s   %s\033[K\n",
                cmdV, cmdW, saturating ? "[SATURATED - scaled]" : "");
  Serial.printf(" measured   v %6.3f m/s   w %6.3f rad/s\033[K\n",
                measuredV(), measuredW());
  Serial.printf(" wheel |  set m/s  set rpm  act rpm  err rpm |   pwm\033[K\n");
  Serial.printf(" ------+-----------------------------------+------\033[K\n");
  Serial.printf(" LEFT  |%9.3f%9.1f%9.1f%9.1f |%6d\033[K\n",
                wheelSetMps[MOTOR_LEFT], wheelSetRpm[MOTOR_LEFT],
                wheel[MOTOR_LEFT].rpmFilt, ctrl[MOTOR_LEFT].error,
                ctrl[MOTOR_LEFT].output);
  Serial.printf(" RIGHT |%9.3f%9.1f%9.1f%9.1f |%6d\033[K\n",
                wheelSetMps[MOTOR_RIGHT], wheelSetRpm[MOTOR_RIGHT],
                wheel[MOTOR_RIGHT].rpmFilt, ctrl[MOTOR_RIGHT].error,
                ctrl[MOTOR_RIGHT].output);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  piInit(ctrl[MOTOR_LEFT], kp, ki);
  piInit(ctrl[MOTOR_RIGHT], kp, ki);

  Serial.println();
  Serial.println("=== Activity 7 - Command Robot Velocity ===");
  Serial.println("vR = v + (L/2)w   vL = v - (L/2)w   RPM = (v/r)*60/(2pi)");
  Serial.printf("wheel base L = %.3f m, wheel radius r = %.4f m\n",
                WHEEL_BASE_M, WHEEL_RADIUS_M);
  Serial.printf("per-wheel limit %.2f m/s (%.0f rpm)\n",
                V_WHEEL_MAX, mpsToRpm(V_WHEEL_MAX));
  Serial.println("MEASURE L and the wheel diameter - both are assumptions.");
  printHelp();
  Serial.println("\n\n\n\n\n");
}

void loop() {
  static uint32_t nextControl = 0;
  static uint32_t nextDisplay = 0;
  const uint32_t now = millis();

  pollSerial();

  if ((int32_t)(now - nextControl) >= 0) {
    nextControl = (nextControl == 0 ? now : nextControl) + CONTROL_INTERVAL_MS;
    const float dt = CONTROL_INTERVAL_MS / 1000.0f;

    robotSample(FILTER_ALPHA);

    for (int i = 0; i < 2; i++) {
      const MotorId m = (MotorId)i;
      if (fabsf(wheelSetRpm[m]) < 0.5f) {
        setMotorPwm(m, 0);
        piReset(ctrl[m]);
        ctrl[m].error = -wheel[m].rpmFilt;
      } else {
        setMotorPwm(m, piUpdate(ctrl[m], m, wheelSetRpm[m],
                                wheel[m].rpmFilt, dt));
      }
    }

    if (logging) {
      Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,"
                    "%.4f,%.4f,%.4f,%.4f,%d,%d\n",
                    now, cmdV, cmdW,
                    wheelSetMps[MOTOR_LEFT], wheelSetMps[MOTOR_RIGHT],
                    wheelSetRpm[MOTOR_LEFT], wheelSetRpm[MOTOR_RIGHT],
                    wheel[MOTOR_LEFT].rpmFilt, wheel[MOTOR_RIGHT].rpmFilt,
                    wheel[MOTOR_LEFT].mps, wheel[MOTOR_RIGHT].mps,
                    measuredV(), measuredW(),
                    ctrl[MOTOR_LEFT].output, ctrl[MOTOR_RIGHT].output);
    }
  }

  if (!logging && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    display();
  }
}
