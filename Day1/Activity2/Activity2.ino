// Encoder test
#define ENCODER_A 12
#define ENCODER_B 14

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
}

void loop() {
  Serial.print("A: ");
  Serial.print(digitalRead(ENCODER_A));
  Serial.print(" | B: ");
  Serial.println(digitalRead(ENCODER_B));
  delay(100);
}