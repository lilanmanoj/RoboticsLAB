// Two N20 motors driven by a TB6612FNG dual motor driver.
// Motor A wiring
const int AIN1 = 12;
const int AIN2 = 14;
const int PWMA = 4;

// Motor B wiring
const int BIN1 = 26;
const int BIN2 = 27;
const int PWMB = 16;

void motorA(bool clockwise, int speed) {
  digitalWrite(AIN1, clockwise ? HIGH : LOW);
  digitalWrite(AIN2, clockwise ? LOW : HIGH);
  analogWrite(PWMA, speed);
}

void motorB(bool clockwise, int speed) {
  digitalWrite(BIN1, clockwise ? HIGH : LOW);
  digitalWrite(BIN2, clockwise ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void loop() {
  int i = 0;
  // Both motors clockwise, full speed
  for (i = 10; i < 255; i++) {
    motorA(true, i);
    motorB(true, i);
    delay(50);
  }

  // Both motors counterclockwise, full speed
  for (i = 255; i > 10; i--) {
    motorA(false, i);
    motorB(false, i);
    delay(50);
  }
}
