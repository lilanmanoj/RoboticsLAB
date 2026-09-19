/*
 * Day3Activity4.ino
 * Activity 4 - PI Velocity Control  (lab PDF p.25)
 *
 *     u(t) = Kp e(t) + Ki integral( e(t) dt )
 *
 * Adds the integral term that Activity 3 lacked, so the steady-state velocity
 * error is driven to zero instead of merely being made small. Press 'i' to
 * zero Ki at runtime and watch the error reappear - that back-to-back
 * comparison is what the activity asks for.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz control loop
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

float kp = 0.35f;
float ki = 2.50f;      // PWM per (RPM*second)
bool useFeedforward = true;

float setpointRpm = 0.0f;
float integral[2] = {0.0f, 0.0f};
float lastError[2] = {0.0f, 0.0f};
int lastOutput[2] = {0, 0};
bool logging = false;

char cmdBuf[24];
uint8_t cmdLen = 0;

void resetIntegrators() {
  integral[0] = 0.0f;
  integral[1] = 0.0f;
}

/*
 * u(t) = Kp e(t) + Ki * integral(e dt), on top of the Activity 1 feedforward.
 *
 * The integrator is frozen whenever the output is already at a rail and the
 * error would push it further in. Without that guard, asking for a speed the
 * motor cannot reach lets the integral grow without limit, and the controller
 * then ignores the setpoint coming back down until it unwinds. That is
 * integral windup, and it is the one thing that will bite you while tuning.
 */
int controlStep(MotorId m, float dt) {
  const float e = setpointRpm - wheel[m].rpmFilt;
  lastError[m] = e;

  const float ff = useFeedforward ? (float)feedforwardPwm(m, setpointRpm) : 0.0f;
  const float candidate = ff + kp * e + ki * (integral[m] + e * dt);

  const bool saturated = (candidate > 255.0f && e > 0.0f) ||
                         (candidate < -255.0f && e < 0.0f);
  if (!saturated) {
    integral[m] += e * dt;
  }

  const float u = ff + kp * e + ki * integral[m];
  lastOutput[m] = constrain((int)lroundf(u), -255, 255);
  return lastOutput[m];
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  v<rpm>   desired wheel velocity, e.g. v150");
  Serial.println("  k<Kp>    proportional gain, e.g. k0.5");
  Serial.println("  i<Ki>    integral gain, e.g. i2.5   (i0 = pure P)");
  Serial.println("  w        toggle feedforward");
  Serial.println("  z        zero the integrators");
  Serial.println("  s        stop");
  Serial.println("  g        toggle CSV logging of every control cycle");
  Serial.println("  h        this help");
  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println("=== Activity 4 - PI Velocity Control ===");
  Serial.println("u(t) = Kp*e(t) + Ki*integral(e dt)");
  Serial.printf("control loop %u ms (%.0f Hz), start Kp=%.2f Ki=%.2f\n",
                CONTROL_INTERVAL_MS, 1000.0f / CONTROL_INTERVAL_MS, kp, ki);
  Serial.println("Compare against Activity 3 by setting i0 (pure P).");
  Serial.println("Lift the chassis so the wheels spin free.");
}

void printCsvHeader() {
  Serial.println("t_ms,kp,ki,setpoint_rpm,rpm_l,err_l,int_l,pwm_l,"
                 "rpm_r,err_r,int_r,pwm_r");
}

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'v':
      setpointRpm = constrain(arg, -RPM_MAX, RPM_MAX);
      break;
    case 'k':
      kp = max(0.0f, arg);
      Serial.printf("\nKp = %.3f\n", kp);
      break;
    case 'i':
      ki = max(0.0f, arg);
      resetIntegrators();
      Serial.printf("\nKi = %.3f (integrators zeroed)\n", ki);
      break;
    case 'w':
      useFeedforward = !useFeedforward;
      resetIntegrators();
      Serial.printf("\nfeedforward %s\n", useFeedforward ? "ON" : "OFF");
      break;
    case 'z':
      resetIntegrators();
      break;
    case 's':
      setpointRpm = 0.0f;
      resetIntegrators();
      motorsCoast();
      break;
    case 'g':
      logging = !logging;
      Serial.println();
      if (logging) printCsvHeader();
      break;
    default:
      printHelp();
      break;
  }
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
  Serial.printf("\033[4A");
  Serial.printf(" Kp %.3f  Ki %.3f  ff %-3s  setpoint %6.1f rpm\033[K\n",
                kp, ki, useFeedforward ? "ON" : "OFF", setpointRpm);
  Serial.printf(" wheel |  set rpm  act rpm  err rpm   integral |   pwm\033[K\n");
  Serial.printf(" LEFT  |%9.1f%9.1f%9.1f%11.2f |%6d\033[K\n",
                setpointRpm, wheel[MOTOR_LEFT].rpmFilt, lastError[MOTOR_LEFT],
                integral[MOTOR_LEFT], lastOutput[MOTOR_LEFT]);
  Serial.printf(" RIGHT |%9.1f%9.1f%9.1f%11.2f |%6d\033[K\n",
                setpointRpm, wheel[MOTOR_RIGHT].rpmFilt, lastError[MOTOR_RIGHT],
                integral[MOTOR_RIGHT], lastOutput[MOTOR_RIGHT]);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  printBanner();
  printHelp();
  Serial.println("\n\n\n");
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

    if (setpointRpm == 0.0f) {
      motorsCoast();
      resetIntegrators();
      lastOutput[0] = lastOutput[1] = 0;
      lastError[0] = -wheel[0].rpmFilt;
      lastError[1] = -wheel[1].rpmFilt;
    } else {
      setMotorPwm(MOTOR_LEFT, controlStep(MOTOR_LEFT, dt));
      setMotorPwm(MOTOR_RIGHT, controlStep(MOTOR_RIGHT, dt));
    }

    if (logging) {
      Serial.printf("%lu,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%d\n",
                    now, kp, ki, setpointRpm,
                    wheel[MOTOR_LEFT].rpmFilt, lastError[MOTOR_LEFT],
                    integral[MOTOR_LEFT], lastOutput[MOTOR_LEFT],
                    wheel[MOTOR_RIGHT].rpmFilt, lastError[MOTOR_RIGHT],
                    integral[MOTOR_RIGHT], lastOutput[MOTOR_RIGHT]);
    }
  }

  if (!logging && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    display();
  }
}
