/*
 * Filename: StepperMotorControl.ino
 * Description: 
 * Author: Maximelio Finch
 * Date: 4/11/2025
 */
const int stepPin = 9;  // Pin connected to STEP
const int dirPin = 10;   // Pin connected to DIR
const int enPin = 11;    // Pin connected to ENABLE

int i = 3200;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Enable the driver
  digitalWrite(enPin, LOW);

  // Set direction
  digitalWrite(dirPin, HIGH);
}

void loop() {
  // Rotate one full revolution 
  for (int j = 0; j < i; j++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // 1 ms HIGH
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000); // 1 ms LOW
  }
  i = 3200;
  delay(1000); // Wait 1 second before next rotation
}