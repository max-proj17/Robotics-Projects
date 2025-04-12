/*
 * Filename: StepperMotorControl.ino
 * Description: Arduino sketch to control a stepper motor using microstepping. Initial Test.
 * Author: Maximelio Finch
 * Date: 4/10/2025
 */
const int stepPin = 3;  // Pin connected to STEP
const int dirPin = 4;   // Pin connected to DIR
const int enPin = 2;    // Pin connected to ENABLE
const int ms1Pin = 7;   // Pin connected to MS1
const int ms2Pin = 6;   // Pin connected to MS2
const int ms3Pin = 5;   // Pin connected to MS3

int i = 0;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);
  pinMode(ms3Pin, OUTPUT);

  // Set microstepping to full step
  setMicrosteppingMode(16);

  // Enable the driver
  digitalWrite(enPin, LOW); // LOW to enable

  // Set direction
  digitalWrite(dirPin, HIGH);
}

void loop() {
  // Rotate one full revolution (200 steps for 1.8° stepper)
  for (int j = 0; j < i; j++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // 1 ms HIGH
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000); // 1 ms LOW
  }
  delay(1000); // Wait 1 second before next rotation
}


void setMicrosteppingMode(int mode) {
  switch (mode) {
    case 1: // Full step
      i = 200;
      digitalWrite(ms1Pin, LOW);
      digitalWrite(ms2Pin, LOW);
      digitalWrite(ms3Pin, LOW);
      break;
    case 2: // Half step
      i = 400;
      digitalWrite(ms1Pin, HIGH);
      digitalWrite(ms2Pin, LOW);
      digitalWrite(ms3Pin, LOW);
      break;
    case 4: // Quarter step
      i = 800;
      digitalWrite(ms1Pin, LOW);
      digitalWrite(ms2Pin, HIGH);
      digitalWrite(ms3Pin, LOW);
      break;
    case 8: // Eighth step
      i = 1600;
      digitalWrite(ms1Pin, HIGH);
      digitalWrite(ms2Pin, HIGH);
      digitalWrite(ms3Pin, LOW);
      break;
    case 16: // Sixteenth step
      i = 3200;
      digitalWrite(ms1Pin, HIGH);
      digitalWrite(ms2Pin, HIGH);
      digitalWrite(ms3Pin, HIGH);
      break;
    default:
      // set to full step by default
      i = 200;
      digitalWrite(ms1Pin, LOW);
      digitalWrite(ms2Pin, LOW);
      digitalWrite(ms3Pin, LOW);
      break;
  }
}