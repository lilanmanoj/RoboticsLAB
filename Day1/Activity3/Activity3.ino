// Wireless robot navigation over Bluetooth Low Energy.
//
// Two N20 motors driven by a TB6612FNG dual motor driver, commanded from a
// phone running a BLE terminal app (iOS: "Bluetooth Terminal").
// iOS cannot use Bluetooth Classic serial, so this advertises the Nordic UART
// Service (NUS), which BLE terminal apps treat as a serial link.
//
// Commands (single characters, upper or lower case):
//   F - forward      B - backward
//   L - turn left    R - turn right
//   S - stop         0-9 - speed, 0 = slowest, 9 = fastest

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Name shown in the phone's scan list
#define DEVICE_NAME "24SEA052_Robo"

// Nordic UART Service: RX is what the phone writes to, TX is what we notify on
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Motor A wiring (left wheel)
const int AIN1 = 12;
const int AIN2 = 14;
const int PWMA = 4;

// Motor B wiring (right wheel)
const int BIN1 = 26;
const int BIN2 = 27;
const int PWMB = 16;

// The two motors face opposite ways on the chassis, so "forward" is a
// different rotation for each. Flip these if a wheel spins the wrong way.
const bool LEFT_FORWARD_CW  = true;
const bool RIGHT_FORWARD_CW = false;

// N20 motors stall below roughly a third duty cycle, so speed 0 is not 0 PWM.
const int MIN_PWM = 90;
const int MAX_PWM = 255;

int speedLevel = 5;               // 0-9, set by the digit commands
char currentCommand = 'S';        // last movement command
BLECharacteristic *txCharacteristic = nullptr;
bool phoneConnected = false;

int pwmForLevel(int level) {
  return MIN_PWM + (MAX_PWM - MIN_PWM) * level / 9;
}

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

// Re-applies the current command, so a new speed takes effect while moving.
void drive() {
  int speed = pwmForLevel(speedLevel);

  switch (currentCommand) {
    case 'F':
      leftMotor(1, speed);
      rightMotor(1, speed);
      break;
    case 'B':
      leftMotor(-1, speed);
      rightMotor(-1, speed);
      break;
    case 'L':                     // spin in place, left wheel back
      leftMotor(-1, speed);
      rightMotor(1, speed);
      break;
    case 'R':                     // spin in place, right wheel back
      leftMotor(1, speed);
      rightMotor(-1, speed);
      break;
    default:                      // 'S' and anything unrecognised
      leftMotor(0, 0);
      rightMotor(0, 0);
      break;
  }
}

// Sends a line back to the phone's terminal, if one is listening.
void reply(const char *message) {
  Serial.println(message);
  if (phoneConnected && txCharacteristic != nullptr) {
    txCharacteristic->setValue((uint8_t *)message, strlen(message));
    txCharacteristic->notify();
  }
}

void handleCommand(char c) {
  char status[48];

  if (c >= '0' && c <= '9') {
    speedLevel = c - '0';
    drive();                      // apply the new speed immediately
    snprintf(status, sizeof(status), "Speed %d (PWM %d)", speedLevel, pwmForLevel(speedLevel));
    reply(status);
    return;
  }

  if (c >= 'a' && c <= 'z') {
    c -= 32;                      // accept lower case
  }

  const char *name;
  switch (c) {
    case 'F': name = "Forward";  break;
    case 'B': name = "Backward"; break;
    case 'L': name = "Left";     break;
    case 'R': name = "Right";    break;
    case 'S': name = "Stop";     break;
    default:  return;             // ignore newlines and stray characters
  }

  currentCommand = c;
  drive();
  snprintf(status, sizeof(status), "%s @ speed %d", name, speedLevel);
  reply(status);
}

void stopMotors() {
  currentCommand = 'S';
  drive();
}

class ConnectionCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    phoneConnected = true;
    Serial.println("Phone connected");
  }

  void onDisconnect(BLEServer *server) override {
    phoneConnected = false;
    stopMotors();                 // never keep driving after losing the link
    Serial.println("Phone disconnected, motors stopped");
    server->startAdvertising();   // let the phone reconnect
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    // getData()/getLength() are used instead of getValue() because the return
    // type of getValue() differs between ESP32 core 2.x and 3.x.
    uint8_t *data = characteristic->getData();
    size_t length = characteristic->getLength();
    for (size_t i = 0; i < length; i++) {
      handleCommand((char)data[i]);
    }
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  stopMotors();

  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ConnectionCallbacks());

  BLEService *service = server->createService(NUS_SERVICE_UUID);

  BLECharacteristic *rx = service->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new CommandCallbacks());

  txCharacteristic = service->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Advertising as " DEVICE_NAME ", waiting for a phone...");
}

void loop() {
  // Commands arrive on the BLE task, so the main loop only mirrors the USB
  // serial monitor for testing without a phone.
  if (Serial.available()) {
    handleCommand((char)Serial.read());
  }
}
