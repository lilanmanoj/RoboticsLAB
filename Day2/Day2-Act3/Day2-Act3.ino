// ============================================================
// EA3121 Robotics Lab 02
// Activity 3 - 2D Environment Mapping (ToF LIDAR)
//
// ESP32 + ST GY-VL53L0XV2 Time-of-Flight laser + TB6612FNG
// + Wi-Fi upload to a dockerised map server
//
// ------------------------------------------------------------
// WIRING - IDENTICAL TO ACTIVITY 2, NOTHING TO REWIRE
// ------------------------------------------------------------
//   STBY -> 21
//   PWMA -> 22   AIN1 -> 16   AIN2 -> 17     (LEFT  motor)
//   PWMB -> 23   BIN1 -> 18   BIN2 -> 19     (RIGHT motor)
//
//   VL53L0X   VIN -> 5V   GND -> GND
//             SDA -> GPIO 32
//             SCL -> GPIO 33
//             XSHUT -> GPIO 4   (optional, USE_XSHUT below)
//
// NEW PINS FOR THIS ACTIVITY: none are required. Wi-Fi needs no
// pins, and the sensor and motors are unchanged.
//
// ------------------------------------------------------------
// NETWORK - THE ROBOT JOINS AN EXISTING WI-FI
// ------------------------------------------------------------
//   SSID     : GA25LM
//   Password : A1B2C3
//
// The laptop running the map container must be on the SAME
// network. The robot POSTs its scan points to it.
//
// FINDING THE SERVER: the laptop's address is handed out by that
// network's DHCP and changes between sessions, so hardcoding it
// goes stale. The robot resolves the laptop by NAME over mDNS
// (hp-pavilion.local) and then confirms the container is really
// listening with GET /api/health - a name that resolves is not
// proof that anything is serving on it.
//
// Set SERVER_HOST_IP below to skip mDNS and pin an address.
//
// Two OPTIONAL extras are supported:
//
//   START BUTTON -> GPIO 15 to GND, momentary, no resistor
//                   (internal pull-up is enabled in software)
//                   Press = run another scan.
//                   NOTE: GPIO 15 is an ESP32 strapping pin and
//                   must be HIGH at boot. An idle-open button is
//                   fine - just do not hold it down while the
//                   board resets.
//                   Set USE_START_BUTTON to 0 to skip the wiring.
//
//   STATUS LED   -> GPIO 2, already on the DevKit board, no
//                   wiring at all.
//                   blinking  = connecting to Wi-Fi
//                   solid on  = scanning
//                   off       = idle / waiting
//
// If you prefer no button, type 's' + Enter in the Serial Monitor
// to trigger a scan instead.
//
// ------------------------------------------------------------
// LIBRARY
// ------------------------------------------------------------
// Library Manager -> "VL53L0X" by Pololu. Everything else
// (WiFi, HTTPClient) ships with the ESP32 core.
//
// ------------------------------------------------------------
// HOW THE MAP IS BUILT
// ------------------------------------------------------------
// The sensor is bolted facing forward, so the robot spins its own
// body to sweep the beam - the car is the turret.
//
// The scan is STEPPED, not a continuous spin:
//
//   pivot a small step -> stop -> let the chassis settle ->
//   take 5 readings and keep the median -> repeat x72
//
// Stopping to measure is what makes the map readable. During a
// continuous spin each 20-200 ms measurement is smeared across
// several degrees of rotation, and every edge in the room comes
// out as a diagonal smudge. Stopped, the beam is pointed at one
// bearing for the whole integration, so walls come out straight.
//
// Bearing comes from the step counter, not from the clock:
// step k is at k * (360 / SCAN_STEPS) degrees. There are no
// encoders, so the ONE thing that has to be calibrated is
// SPIN_360_MS - how long a full turn takes at PIVOT_SPEED.
//
// Points are streamed to the server in batches as the scan
// progresses, so the map fills in live in the browser.
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>


// ============================================================
// WI-FI
// ============================================================

const char* WIFI_SSID     = "GA25LM";
const char* WIFI_PASSWORD = "A1B2C3D4";

// How long to wait at boot before giving up and scanning offline.
// Not connecting is not a failure: the map still comes out on
// Serial, and the robot keeps retrying in the background.
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;


// ============================================================
// MAP SERVER
//
// Two ways to find it, tried in this order:
//
//   1. SERVER_HOST_IP  - if you fill it in, it is used as-is.
//   2. mDNS            - resolve SERVER_MDNS_NAME + ".local"
//
// mDNS is the default because the laptop's IP is a DHCP lease and
// changes between sessions, while its hostname does not. On Linux
// avahi-daemon answers these queries (already running on
// hp-pavilion); macOS answers out of the box; on Windows it needs
// Bonjour installed - if it is not, just fill in SERVER_HOST_IP.
// ============================================================

const char* SERVER_HOST_IP   = "10.31.81.226";   // "" = use mDNS
const char* SERVER_MDNS_NAME = "hp-pavilion";   // no ".local" suffix
const int   SERVER_PORT      = 8080;

// mDNS query timeout per attempt.
const uint32_t MDNS_TIMEOUT_MS = 2000;

// Health-check timeout when confirming a resolved address.
const uint16_t DISCOVER_TIMEOUT_MS = 1000;

// Resolved server address, "" until found.
String serverHost = "";

// Give up on a POST after this long so a missing server cannot
// stall the scan. The robot keeps mapping and printing to Serial
// either way - the upload is a bonus, not a dependency.
const uint16_t HTTP_TIMEOUT_MS = 3000;

// Consecutive upload failures before the robot assumes the laptop
// moved (or reconnected on a different lease) and re-discovers.
const int MAX_POST_FAILURES = 3;

int postFailures = 0;


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

// False until init() succeeds. A missing sensor stops the robot
// scanning, but it must not stop the access point or the serial
// console - those are how you find out what is wrong.
bool sensorReady = false;


// ============================================================
// OPTIONAL EXTRAS
// ============================================================

#define USE_START_BUTTON 1
#define BUTTON_PIN       15

#define STATUS_LED_PIN   2


// ============================================================
// SENSOR CONFIGURATION - LONG RANGE PRESET
//
// The robot is stationary at every measurement, so there is time
// to spend on accuracy. A 200 ms budget with a relaxed signal
// rate limit and longer VCSEL pulses pushes the useful range out
// to roughly 2 m, which is what makes a room-sized map possible.
// The cost is 200 ms per point - a 72 point scan takes about a
// minute including the pivots, and that is fine for mapping.
// ============================================================

const uint32_t TIMING_BUDGET_US   = 200000;
const float    SIGNAL_RATE_LIMIT  = 0.10;   // MCPS, default 0.25

// Anything beyond this is treated as "no return" and left off the
// map rather than drawn as a false wall at max range.
const int MAX_RANGE_MM = 2000;

// Below the sensor's blind zone the reading means nothing.
const int MIN_RANGE_MM = 30;

// Readings per bearing. The median of these is what gets stored.
const int SAMPLES_PER_STEP = 5;


// ============================================================
// SCAN GEOMETRY
// ============================================================

// 72 steps x 5 degrees = 360 degrees.
const int   SCAN_STEPS = 72;
const float STEP_DEG   = 360.0 / SCAN_STEPS;


// ============================================================
// MOTION - THE ONE THING YOU MUST CALIBRATE
//
// Time for one full 360 degree spin in place at PIVOT_SPEED.
// Measure it: mark the floor, run the CALIBRATE routine (send
// 'c' in the Serial Monitor), watch where it stops, adjust.
//
// If the map comes out stretched - the far wall appears twice, or
// the scan closes short of a full circle - this number is wrong.
// Too small = the map over-rotates, too large = it under-rotates.
// ============================================================

int           PIVOT_SPEED  = 170;
unsigned long SPIN_360_MS  = 3000;

// Derived: how long to drive the pivot for one step.
unsigned long stepPivotMs() {
  return SPIN_360_MS / SCAN_STEPS;
}

// The chassis keeps rolling for a moment after the PWM stops, and
// the ToF needs a still target. This pause is deliberately longer
// than it looks like it needs to be.
const unsigned long SETTLE_MS = 250;

// Brief full-power kick before each step. Below a certain PWM the
// motors will not break static friction at all, so a short kick
// gets the chassis moving and the remainder of the step runs at
// PIVOT_SPEED. Without this the small steps are wildly uneven.
const int           KICK_SPEED = 235;
const unsigned long KICK_MS    = 40;


// ============================================================
// SCAN STORAGE
//
// 72 points is small enough to hold the whole scan in RAM, which
// means a failed upload never costs data - the scan is complete
// on the robot and can be dumped to Serial or re-sent.
// ============================================================

int scanDistanceMm[SCAN_STEPS];   // 0 = no return at that bearing

// Points are uploaded in batches this size. Small enough to build
// the JSON in a String without fragmenting the heap, large enough
// that the scan is not dominated by HTTP round trips.
const int BATCH_SIZE = 8;

uint32_t scanId = 0;


// ============================================================
// AUTO-REPEAT
//
// 0 = scan once at boot, then wait for a button press or 's'.
// Set to e.g. 30000 to re-scan every 30 s unattended.
// ============================================================

const unsigned long AUTO_RESCAN_MS = 0;

unsigned long lastScanEndMs = 0;


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
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void stopMotors() {
  driveMotors(0, 0);
}


// ============================================================
// PIVOT ONE SCAN STEP
//
// Spins CLOCKWISE (to the right), which is why bearings increase
// clockwise on the map: 0 = straight ahead, 90 = robot's right.
// ============================================================

void pivotOneStep() {

  unsigned long total = stepPivotMs();

  // Kick through static friction
  driveMotors(KICK_SPEED, -KICK_SPEED);
  delay(min(KICK_MS, total));

  if (total > KICK_MS) {
    driveMotors(PIVOT_SPEED, -PIVOT_SPEED);
    delay(total - KICK_MS);
  }

  stopMotors();
}


// ============================================================
// ONE RANGE READING
//
// Returns 0 for "no usable return" rather than a number, so a
// missing wall never becomes a phantom one on the map.
// ============================================================

int readRangeOnce() {

  int mm = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) return 0;
  if (mm >= 8000)               return 0;   // library "out of range"
  if (mm < MIN_RANGE_MM)        return 0;
  if (mm > MAX_RANGE_MM)        return 0;

  return mm;
}


// ============================================================
// MEASURE ONE BEARING
//
// Median of SAMPLES_PER_STEP readings. A median rather than a
// mean because the VL53L0X's error mode is the occasional wild
// outlier - a mean would drag every point toward it, a median
// throws it away. No-return samples are excluded first, and the
// bearing only counts as a hit if most samples agreed there was
// something there.
// ============================================================

int measureBearing() {

  int samples[SAMPLES_PER_STEP];
  int n = 0;

  for (int i = 0; i < SAMPLES_PER_STEP; i++) {
    int mm = readRangeOnce();
    if (mm > 0) {
      samples[n++] = mm;
    }
  }

  // Fewer than half the samples saw anything -> open space
  if (n <= SAMPLES_PER_STEP / 2) {
    return 0;
  }

  // Insertion sort - n is at most 5
  for (int i = 1; i < n; i++) {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  return samples[n / 2];
}


// ============================================================
// HEALTH CHECK
//
// Confirms something is actually serving the map API at this
// address, rather than merely existing on the network.
// ============================================================

bool serverAnswersAt(const String& host) {

  String url = "http://" + host + ":" + String(SERVER_PORT) + "/api/health";

  HTTPClient http;
  http.setConnectTimeout(DISCOVER_TIMEOUT_MS);
  http.setTimeout(DISCOVER_TIMEOUT_MS);

  if (!http.begin(url)) {
    return false;
  }

  int code = http.GET();
  http.end();

  return code == 200;
}


// ============================================================
// FIND THE MAP SERVER
//
// Manual address if one is configured, otherwise resolve the
// laptop's hostname over mDNS. Either way the result is verified
// with the health check before it is cached - a name that
// resolves is not proof that the container is running.
// ============================================================

bool discoverServer() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Discovery: not on Wi-Fi yet");
    return false;
  }

  // ----- 1. configured address -----
  if (strlen(SERVER_HOST_IP) > 0) {
    Serial.print("Discovery: checking ");
    Serial.print(SERVER_HOST_IP);
    Serial.print(" ... ");

    if (serverAnswersAt(SERVER_HOST_IP)) {
      serverHost = SERVER_HOST_IP;
      postFailures = 0;
      Serial.println("map server found");
      return true;
    }

    Serial.println("no answer");
    Serial.println("  Is the container up?  docker compose up -d");
    return false;
  }

  // ----- 2. mDNS -----
  Serial.print("Discovery: resolving ");
  Serial.print(SERVER_MDNS_NAME);
  Serial.print(".local ... ");

  IPAddress ip = MDNS.queryHost(SERVER_MDNS_NAME, MDNS_TIMEOUT_MS);

  if (ip == IPAddress((uint32_t)0)) {
    Serial.println("no answer");
    Serial.println("  The laptop must be on GA25LM and running avahi/Bonjour.");
    Serial.println("  Or set SERVER_HOST_IP to its address and re-flash.");
    return false;
  }

  Serial.print(ip);
  Serial.print(" ... ");

  // Sanity check: the laptop runs Docker, and avahi happily
  // publishes the docker0 / br-* bridge addresses (172.x.x.x)
  // alongside the real wireless one. Those are unroutable from
  // here, so if the resolved address is not on the robot's own
  // subnet, say so plainly rather than leaving a bare timeout to
  // be interpreted.
  uint32_t resolved = (uint32_t)ip;
  uint32_t own      = (uint32_t)WiFi.localIP();
  uint32_t mask     = (uint32_t)WiFi.subnetMask();

  if ((resolved & mask) != (own & mask)) {
    Serial.println();
    Serial.println("  WARNING: that address is not on this robot's subnet.");
    Serial.println("  It is probably a docker bridge address, not the laptop's Wi-Fi IP.");
    Serial.println("  Fix: add 'deny-interfaces=docker0' to /etc/avahi/avahi-daemon.conf");
    Serial.println("       and restart avahi - or set SERVER_HOST_IP in this sketch.");
    Serial.print("  Trying it anyway ... ");
  }

  String candidate = ip.toString();

  if (!serverAnswersAt(candidate)) {
    Serial.println("host is up but the map server is not answering");
    Serial.println("  Is the container up?  docker compose up -d");
    Serial.println("  Firewall on port 8080?");
    return false;
  }

  serverHost = candidate;
  postFailures = 0;
  Serial.println("map server found");
  return true;
}


// ============================================================
// HTTP POST
//
// Returns true on any 2xx. Failures are reported and ignored -
// the scan continues regardless.
// ============================================================

bool postJson(const String& path, const String& payload) {

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  if (serverHost.length() == 0 && !discoverServer()) {
    return false;
  }

  String url = "http://" + serverHost + ":" + String(SERVER_PORT) + path;

  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);

  if (!http.begin(url)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");

  int code = http.POST(payload);
  http.end();

  if (code >= 200 && code < 300) {
    postFailures = 0;
    return true;
  }

  Serial.print("POST ");
  Serial.print(path);
  Serial.print(" failed: ");
  Serial.println(code);   // negative = could not reach the server

  // The laptop may have rejoined on a different lease. Forget the
  // cached address so the next upload probes for it again.
  if (++postFailures >= MAX_POST_FAILURES) {
    Serial.println("Server unreachable - will re-discover");
    serverHost = "";
    postFailures = 0;
  }

  return false;
}


// ============================================================
// UPLOAD: SCAN START
// ============================================================

void uploadScanStart() {

  String payload = "{\"scan\":" + String(scanId)
                 + ",\"steps\":" + String(SCAN_STEPS)
                 + ",\"step_deg\":" + String(STEP_DEG, 2)
                 + ",\"max_range_mm\":" + String(MAX_RANGE_MM)
                 + "}";

  postJson("/api/scan/start", payload);
}


// ============================================================
// UPLOAD: A BATCH OF POINTS
//
// [[bearing_deg, distance_mm], ...] - compact on purpose, since
// this goes out over Wi-Fi 9 times per scan.
// ============================================================

void uploadBatch(int fromStep, int toStep) {

  String payload = "{\"scan\":" + String(scanId) + ",\"points\":[";

  for (int k = fromStep; k < toStep; k++) {
    if (k > fromStep) payload += ",";
    payload += "[" + String(k * STEP_DEG, 1) + "," + String(scanDistanceMm[k]) + "]";
  }

  payload += "]}";

  postJson("/api/scan/points", payload);
}


// ============================================================
// UPLOAD: SCAN END
// ============================================================

void uploadScanEnd() {
  postJson("/api/scan/end", "{\"scan\":" + String(scanId) + "}");
}


// ============================================================
// SERIAL DUMP
//
// The map also comes out here, so the activity still works with
// no server, no Wi-Fi, and no network at all. Paste-friendly CSV.
// ============================================================

void dumpScanToSerial() {

  Serial.println("\n--- SCAN RESULT (bearing_deg, distance_mm, x_mm, y_mm) ---");

  int hits = 0;

  for (int k = 0; k < SCAN_STEPS; k++) {

    float bearing = k * STEP_DEG;
    int   d       = scanDistanceMm[k];

    Serial.print(bearing, 1);
    Serial.print(",");
    Serial.print(d);

    if (d > 0) {
      hits++;
      float rad = radians(bearing);
      // x = right of the robot, y = ahead of the robot
      Serial.print(",");
      Serial.print(d * sin(rad), 0);
      Serial.print(",");
      Serial.print(d * cos(rad), 0);
    }
    else {
      Serial.print(",,");   // open bearing, no point
    }

    Serial.println();
  }

  Serial.print("--- ");
  Serial.print(hits);
  Serial.print(" / ");
  Serial.print(SCAN_STEPS);
  Serial.println(" bearings returned a hit ---\n");
}


// ============================================================
// RUN ONE FULL 360 SCAN
// ============================================================

void runScan() {

  if (!sensorReady) {
    Serial.println("Cannot scan: VL53L0X not initialised. Send 'i' after fixing the wiring.");
    return;
  }

  scanId = millis();

  digitalWrite(STATUS_LED_PIN, HIGH);

  Serial.println("\n======================================");
  Serial.print("SCAN ");
  Serial.print(scanId);
  Serial.print(" - ");
  Serial.print(SCAN_STEPS);
  Serial.print(" bearings, ");
  Serial.print(STEP_DEG, 1);
  Serial.println(" deg apart");
  Serial.println("======================================");

  uploadScanStart();

  int batchStart = 0;

  for (int k = 0; k < SCAN_STEPS; k++) {

    // Let the chassis stop rocking before the beam integrates
    delay(SETTLE_MS);

    scanDistanceMm[k] = measureBearing();

    Serial.print("  ");
    Serial.print(k * STEP_DEG, 1);
    Serial.print(" deg -> ");
    if (scanDistanceMm[k] > 0) {
      Serial.print(scanDistanceMm[k]);
      Serial.println(" mm");
    }
    else {
      Serial.println("open");
    }

    // Stream the map out as it is built
    if ((k + 1) - batchStart >= BATCH_SIZE || k == SCAN_STEPS - 1) {
      uploadBatch(batchStart, k + 1);
      batchStart = k + 1;
    }

    // No pivot after the final bearing - the circle is closed
    if (k < SCAN_STEPS - 1) {
      pivotOneStep();
    }
  }

  stopMotors();
  uploadScanEnd();
  dumpScanToSerial();

  if (serverHost.length() > 0) {
    Serial.print("Map: http://");
    Serial.print(serverHost);
    Serial.print(":");
    Serial.println(SERVER_PORT);
  }
  else {
    Serial.println("Map: not uploaded - no server found on GA25LM");
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  lastScanEndMs = millis();
}


// ============================================================
// CALIBRATION HELPER - send 'c'
//
// Spins for exactly SPIN_360_MS at PIVOT_SPEED. Mark the floor,
// run it, and see how far past (or short of) the mark the robot
// lands. Adjust SPIN_360_MS and repeat until it comes back to
// the mark. Everything about the map's shape depends on this.
// ============================================================

void runSpinCalibration() {

  Serial.println("\n--- CALIBRATION: one continuous 360 ---");
  Serial.print("SPIN_360_MS = ");
  Serial.println(SPIN_360_MS);
  Serial.println("Mark the floor and watch where it stops.");

  delay(1500);

  driveMotors(KICK_SPEED, -KICK_SPEED);
  delay(KICK_MS);
  driveMotors(PIVOT_SPEED, -PIVOT_SPEED);
  delay(SPIN_360_MS - KICK_MS);
  stopMotors();

  Serial.println("--- done. Over-rotated? lower SPIN_360_MS.");
  Serial.println("---       Under-rotated? raise it.\n");
}


// ============================================================
// JOIN THE WI-FI
//
// Non-fatal by design: if the network is down the robot still
// scans and still reports over Serial. A mapping robot that
// refuses to map because a laptop is missing is a worse robot.
// ============================================================

void connectWiFi() {

  // Do not let a stale mode or a half-configured radio from the
  // previous sketch linger - start from a known state.
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // ----- POWER, NOT PERFORMANCE -----
  //
  // Both of these settings trade throughput for a gentler current
  // draw, because this robot browns out rather than runs short of
  // bandwidth. A scan uploads about 2 KB total; there is nothing
  // here that needs a fast link.
  //
  // TX power: full output is +19.5 dBm and pulls the biggest
  // current spike the board ever sees - which is precisely what
  // trips the brownout detector at boot. +11 dBm is roughly a
  // third of the radiated power and still ample across a room.
  // Raise it only if the link will not hold, and only once the
  // supply is known good.
  WiFi.setTxPower(WIFI_POWER_11dBm);

  // Modem sleep left ENABLED (the default). It lets the radio idle
  // between beacons instead of holding the PA up continuously.
  // Turning it off costs average current for latency this robot
  // does not care about.
  WiFi.setSleep(true);

  Serial.print("Wi-Fi: connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(250);
    Serial.print(".");
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi: NOT CONNECTED - scanning offline, results on Serial only");
    Serial.println("  Check the SSID/password, and that GA25LM is 2.4 GHz.");
    Serial.println("  Send 'w' to retry.");
    return;
  }

  Serial.println("--- WI-FI CONNECTED ---");
  Serial.print("  SSID     : ");  Serial.println(WiFi.SSID());
  Serial.print("  Robot IP : ");  Serial.println(WiFi.localIP());
  Serial.print("  Gateway  : ");  Serial.println(WiFi.gatewayIP());
  Serial.print("  Signal   : ");  Serial.print(WiFi.RSSI()); Serial.println(" dBm");

  // Needed before MDNS.queryHost() can be used. The robot itself is
  // reachable as tof-robot.local, which is handy when you want to
  // ping it to check it is alive.
  if (!MDNS.begin("tof-robot")) {
    Serial.println("  mDNS: failed to start (name lookup will not work)");
  }

  discoverServer();
}


// ============================================================
// WI-FI STATUS - send 'w'
//
// The first thing to check when scans are not reaching the map.
// Reconnects automatically if the link has dropped.
// ============================================================

void printWiFiStatus() {

  bool connected = (WiFi.status() == WL_CONNECTED);

  Serial.println("\n--- WI-FI STATUS ---");
  Serial.print("  State      : ");
  Serial.println(connected ? "connected" : "NOT connected");
  Serial.print("  SSID       : ");
  Serial.println(WIFI_SSID);

  if (connected) {
    Serial.print("  Robot IP   : ");  Serial.println(WiFi.localIP());
    Serial.print("  Gateway    : ");  Serial.println(WiFi.gatewayIP());
    Serial.print("  Signal     : ");  Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }

  Serial.print("  Map server : ");
  Serial.println(serverHost.length() ? serverHost : "not found yet");

  if (!connected) {
    Serial.println("  -> reconnecting now");
    connectWiFi();
  }
  Serial.println();
}


// ============================================================
// RETRY THE SENSOR - send 'i'
//
// Fix the I2C wiring with the board still powered, send 'i', and
// carry on. No reflash, no reset.
// ============================================================

void retrySensorInit() {

  Serial.println("Re-initialising VL53L0X...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  sensor.setTimeout(500);

  if (sensor.init()) {
    sensor.setSignalRateLimit(SIGNAL_RATE_LIMIT);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
    sensorReady = true;
    Serial.println("VL53L0X: ready - send 's' to scan");
  }
  else {
    sensorReady = false;
    Serial.println("VL53L0X: still not responding.");
    Serial.println("  SDA=32  SCL=33  GND shared with the ESP32  VIN powered");
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);          // let the USB serial attach before the banner

  Serial.println("\n\n=== BOOT ===");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

#if USE_START_BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif

  // ----- Motor driver DISABLED first -----
  //
  // Only STBY is touched here. Holding it LOW puts the TB6612FNG's
  // outputs in high-Z, so the wheels cannot twitch while the rest of
  // setup runs. Full motor init happens after the radio is up.
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  // ----- Network FIRST -----
  //
  // Wi-Fi is brought up before the sensor deliberately. Anything
  // below this line can fail - a loose I2C wire, a dead sensor -
  // and the network still comes up, so the robot stays
  // diagnosable. Doing it the other way round means one unplugged
  // sensor wire silently costs you the Wi-Fi as well.
  connectWiFi();

  // ----- Motor driver -----
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

  // Three tries: a sensor that browned out with the motors often
  // answers on the second attempt.
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
    // Long-range preset - see the note by TIMING_BUDGET_US
    sensor.setSignalRateLimit(SIGNAL_RATE_LIMIT);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
    Serial.println("VL53L0X: ready");
  }
  else {
    // Not fatal any more. The AP stays up, Serial stays responsive,
    // and 'i' retries the sensor once the wiring is fixed - no
    // reflash needed.
    Serial.println("VL53L0X: FAILED - check SDA=32, SCL=33, GND shared, VIN powered");
    Serial.println("           fix the wiring, then send 'i' to retry");
  }

  Serial.println("\n======================================");
  Serial.println("2D ToF MAPPER");
  Serial.println("  s = scan     c = calibrate spin");
  Serial.println("  d = re-find the map server");
  Serial.println("  w = wi-fi status   i = re-init sensor");
#if USE_START_BUTTON
  Serial.println("  or press the button on GPIO 15");
#endif
  Serial.println("======================================");

  if (sensorReady) {
    delay(2000);   // hands clear
    runScan();     // first map straight away
  }
}


// ============================================================
// MAIN LOOP - idle until something asks for another scan
// ============================================================

void loop() {

  stopMotors();

  // ----- Serial trigger -----
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 's' || c == 'S') {
      runScan();
    }
    else if (c == 'c' || c == 'C') {
      runSpinCalibration();
    }
    else if (c == 'd' || c == 'D') {
      serverHost = "";
      discoverServer();
    }
    else if (c == 'w' || c == 'W') {
      printWiFiStatus();
    }
    else if (c == 'i' || c == 'I') {
      retrySensorInit();
    }
  }

#if USE_START_BUTTON
  // ----- Button trigger -----
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(30);                                // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      while (digitalRead(BUTTON_PIN) == LOW) {  // wait for release
        delay(10);
      }
      runScan();
    }
  }
#endif

  // ----- Unattended re-scan -----
  if (AUTO_RESCAN_MS > 0 && millis() - lastScanEndMs >= AUTO_RESCAN_MS) {
    runScan();
  }

  delay(20);
}
