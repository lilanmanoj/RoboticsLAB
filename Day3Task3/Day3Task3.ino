/*
 * Day3Task3.ino
 * Task 3 - Convert Wheel Velocity to RPM  (lab PDF p.34)
 *
 *     w_wheel = v_wheel / r
 *     RPM     = w_wheel * 60 / (2 pi)
 *
 * Converts the vL and vR from Task 2 into the RPM setpoints that feed the
 * closed-loop wheel velocity controllers of Activities 4-6.
 *
 * Pure computation - no motors, no encoders.
 *
 * Board: ESP32-S3-DevKitC-1 (runs on any Arduino target)
 */

#include <Arduino.h>

// ASSUMPTION: 43 mm wheels. Not recorded anywhere in the repository - measure
// yours. Every RPM figure below scales inversely with the radius.
float wheelDiameterM = 0.043f;

// Highest wheel speed reached during Activity 1, used only as a sanity check
// on whether the requested setpoint is actually achievable.
const float RPM_MAX = 400.0f;

// Defaults are the Task 2 answer for the lab-sheet numbers.
float vL = 0.1912f;   // m/s
float vR = 0.3088f;   // m/s

char cmdBuf[48];
uint8_t cmdLen = 0;

void solveOne(const char *name, float vWheel, float r) {
  const float omega = vWheel / r;                    // rad/s
  const float rpm = omega * 60.0f / (2.0f * PI);

  Serial.printf("%-6s v = %8.4f m/s\n", name, vWheel);
  Serial.printf("       w_wheel = v/r      = %8.4f / %.5f = %8.4f rad/s\n",
                vWheel, r, omega);
  Serial.printf("       RPM = w*60/(2pi)   = %8.4f * 9.5493      = %8.2f rpm%s\n",
                omega, rpm,
                fabsf(rpm) > RPM_MAX ? "   [ABOVE MEASURED MAX]" : "");
}

void solve() {
  const float r = wheelDiameterM / 2.0f;

  Serial.println();
  Serial.println("--------------------------------------------------------");
  Serial.printf("wheel diameter %.4f m  ->  radius r = %.5f m\n",
                wheelDiameterM, r);
  Serial.printf("circumference  %.5f m per revolution\n", PI * wheelDiameterM);
  Serial.println("--------------------------------------------------------");
  solveOne("LEFT", vL, r);
  Serial.println();
  solveOne("RIGHT", vR, r);
  Serial.println("--------------------------------------------------------");

  const float rpmL = (vL / r) * 60.0f / (2.0f * PI);
  const float rpmR = (vR / r) * 60.0f / (2.0f * PI);
  Serial.printf("ANSWER: RPM_L = %.2f rpm, RPM_R = %.2f rpm\n", rpmL, rpmR);
  Serial.println();
  Serial.println("These are the setpoints for the wheel velocity controllers");
  Serial.println("(Activity 4 single wheel, Activity 6 independent wheels).");
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  c<vL>,<vR>  set wheel velocities, e.g. c0.1912,0.3088");
  Serial.println("  d<m>        set wheel diameter,   e.g. d0.043");
  Serial.println("  n<rpmL>,<rpmR>  inverse: RPM -> m/s");
  Serial.println("  r           recompute and print");
  Serial.println("  h           this help");
  Serial.println();
}

int parseFloats(const char *s, float *out, int maxN) {
  int n = 0;
  const char *p = s;
  while (n < maxN && *p) {
    out[n++] = atof(p);
    const char *comma = strchr(p, ',');
    if (!comma) break;
    p = comma + 1;
  }
  return n;
}

void handleCommand(const char *cmd) {
  float vals[2];
  const float r = wheelDiameterM / 2.0f;
  switch (cmd[0]) {
    case 'c':
      if (parseFloats(cmd + 1, vals, 2) == 2) {
        vL = vals[0];
        vR = vals[1];
        solve();
      } else {
        Serial.println("need two values: c<vL>,<vR>");
      }
      break;
    case 'd':
      wheelDiameterM = max(0.001f, (float)atof(cmd + 1));
      solve();
      break;
    case 'n':
      if (parseFloats(cmd + 1, vals, 2) == 2) {
        // RPM -> rad/s -> m/s, the reverse of the task
        vL = vals[0] * 2.0f * PI / 60.0f * r;
        vR = vals[1] * 2.0f * PI / 60.0f * r;
        Serial.printf("\n%.2f rpm -> %.4f m/s,  %.2f rpm -> %.4f m/s\n",
                      vals[0], vL, vals[1], vR);
        solve();
      } else {
        Serial.println("need two values: n<rpmL>,<rpmR>");
      }
      break;
    case 'r': solve(); break;
    default: printHelp(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Task 3 - Convert Wheel Velocity to RPM ===");
  Serial.println("Worked with the Task 2 answer as input:");
  solve();
  printHelp();
}

void loop() {
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
