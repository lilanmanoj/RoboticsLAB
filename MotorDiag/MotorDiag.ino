/*
 * MotorDiag.ino
 * Interactive bench diagnostic for the TB6612FNG + N20 drivetrain.
 *
 * Type single-line commands over Serial (115200) to drive one channel at a
 * time and watch both encoders. Use it to find out WHICH stage is broken when
 * a motor refuses to turn: the LEDC/PWM path, the direction pins, the driver
 * channel, the wiring, or the motor itself.
 *
 * Board: ESP32-S3-DevKitC-1   Pin map: repository README.md
 */

#include <Arduino.h>

// Pairing verified on the bench: channel A turns the encoder on GPIO 36/37,
// channel B turns the encoder on GPIO 38/39 - crossed relative to ../README.md.

// Motor A - N20 Motor Left (Starboard Side)
#define AIN1 10
#define AIN2 11
#define PWMA 4
#define ENC_L_C1 36
#define ENC_L_C2 37

// Motor B - N20 Motor Right (Port Side)
#define BIN1 47
#define BIN2 48
#define PWMB 5
#define ENC_R_C1 38
#define ENC_R_C2 39

#define STBY 1

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

void IRAM_ATTR leftEncoderISR() {
  uint8_t s = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  leftCount += QUAD_TABLE[(leftState << 2) | s];
  leftState = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);
  rightCount += QUAD_TABLE[(rightState << 2) | s];
  rightState = s;
}

bool streamEncoders = false;
unsigned long lastStream = 0;

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  a<pwm>  drive LEFT  (motor A) forward, e.g. a200");
  Serial.println("  A<pwm>  drive LEFT  (motor A) reverse");
  Serial.println("  b<pwm>  drive RIGHT (motor B) forward, e.g. b200");
  Serial.println("  B<pwm>  drive RIGHT (motor B) reverse");
  Serial.println("  s       stop both motors (coast)");
  Serial.println("  z       DC test RIGHT: digitalWrite(PWMB,HIGH), bypasses PWM");
  Serial.println("  Z       DC test LEFT:  digitalWrite(PWMA,HIGH), bypasses PWM");
  Serial.println("  p       print static pin levels for multimeter probing");
  Serial.println("  e       toggle live encoder count streaming");
  Serial.println("  r       zero both encoder counts");
  Serial.println("  h       this help");
  Serial.println();
}

void stopAll() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  Serial.println("-> both motors coasting");
}

void drive(bool motorA, bool forward, int pwm) {
  const int in1 = motorA ? AIN1 : BIN1;
  const int in2 = motorA ? AIN2 : BIN2;
  const int pwmPin = motorA ? PWMA : PWMB;

  pwm = constrain(pwm, 0, 255);
  digitalWrite(in1, forward ? LOW : HIGH);
  digitalWrite(in2, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwm);

  Serial.print("-> motor ");
  Serial.print(motorA ? "A/LEFT " : "B/RIGHT ");
  Serial.print(forward ? "FWD" : "REV");
  Serial.print(" pwm=");
  Serial.print(pwm);
  Serial.print("  STBY=");
  Serial.println(digitalRead(STBY));
}

// Full-on DC drive with digitalWrite instead of analogWrite. If the motor
// turns here but not under a<pwm>/b<pwm>, the LEDC/PWM path is the problem,
// not the driver or the motor.
void dcTest(bool motorA) {
  const int in1 = motorA ? AIN1 : BIN1;
  const int in2 = motorA ? AIN2 : BIN2;
  const int pwmPin = motorA ? PWMA : PWMB;

  analogWrite(pwmPin, 0);   // release LEDC before taking the pin over
  pinMode(pwmPin, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(pwmPin, HIGH);

  Serial.print("-> DC test on motor ");
  Serial.print(motorA ? "A/LEFT" : "B/RIGHT");
  Serial.println(": PWM pin held HIGH for 2 s (no PWM involved)");
  delay(2000);
  digitalWrite(pwmPin, LOW);
  Serial.println("-> DC test done");
}

// Holds a known static pattern so every driver input can be checked with a
// multimeter against GND. Expected: STBY 3.3V, BIN1 0V, BIN2 3.3V, PWMB 3.3V.
void pinLevelTest() {
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  pinMode(PWMB, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(PWMB, HIGH);

  Serial.println("-> static levels held for 15 s. Measure against GND:");
  Serial.println("     STBY (GPIO1)  should read ~3.3 V");
  Serial.println("     BIN1 (GPIO47) should read ~0 V");
  Serial.println("     BIN2 (GPIO48) should read ~3.3 V");
  Serial.println("     PWMB (GPIO5)  should read ~3.3 V");
  Serial.println("   then measure the TB6612FNG B1/B2 motor OUTPUT pins:");
  Serial.println("     one should sit near VM, the other near 0 V");
  Serial.println("   and check VM and VCC on the driver itself.");
  delay(15000);
  digitalWrite(PWMB, LOW);
  Serial.println("-> pin level test done");
}

void printCounts() {
  noInterrupts();
  int32_t l = leftCount;
  int32_t r = rightCount;
  interrupts();
  Serial.print("enc  L=");
  Serial.print(l);
  Serial.print("  R=");
  Serial.println(r);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }
  const char c = cmd.charAt(0);
  const int arg = cmd.substring(1).toInt();

  switch (c) {
    case 'a': drive(true, true, arg ? arg : 200); break;
    case 'A': drive(true, false, arg ? arg : 200); break;
    case 'b': drive(false, true, arg ? arg : 200); break;
    case 'B': drive(false, false, arg ? arg : 200); break;
    case 's': stopAll(); break;
    case 'z': dcTest(false); break;
    case 'Z': dcTest(true); break;
    case 'p': pinLevelTest(); break;
    case 'e':
      streamEncoders = !streamEncoders;
      Serial.println(streamEncoders ? "-> encoder streaming ON"
                                    : "-> encoder streaming OFF");
      break;
    case 'r':
      noInterrupts();
      leftCount = 0;
      rightCount = 0;
      interrupts();
      Serial.println("-> counts zeroed");
      break;
    default: printHelp(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

  leftState = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  rightState = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), rightEncoderISR, CHANGE);

  digitalWrite(STBY, HIGH);  // driver enabled for the whole session
  stopAll();

  Serial.println();
  Serial.println("MotorDiag ready - TB6612FNG / N20 bench test");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }
  if (streamEncoders && millis() - lastStream >= 250) {
    lastStream = millis();
    printCounts();
  }
}
