/*
 * Simple Stepper + AS5600 Encoder Test with FreeRTOS Tasks
 * - motorTask: drives the stepper
 * - encoderTask: continuously reads & plots encoder angle
 * - Barebones encoder math, no filtering, no zeroing
 */

#include <Arduino.h>

// ----------------------------
// Pins
// ----------------------------
const int driver_dir_pin = 3;
const int driver_step_pin = 8;
const int driver_enable_pin = 46;
const int encoder_out_pin = 12;
const int encoder_dir_pin = 13;
const int driver_ms1_pin = 7;   // Pin connected to MS1
const int driver_ms2_pin = 6;   // Pin connected to MS2
const int driver_ms3_pin = 5;   // Pin connected to MS3
int i = 3200;

// ----------------------------
// Settings
// ----------------------------
const int stepsPerRevolution = 3200;
const int ADCRESOLUTION = 4095;
#define VREF 3.3  // ESP32 default voltage reference  
#define ALPHA 0.3 // Digital LPF strength (higher = smoother, but slower)

// ----------------------------
// Encoder: barebones read
// ----------------------------
float readEncoderDeg() {
  int adc = analogRead(encoder_out_pin);

  // Map raw ADC 0–4095 -> 0–360 degrees
  float voltage = (adc / (float)ADCRESOLUTION) * VREF;
  float angleDegrees = (voltage / VREF) * 360.0;
  static float filteredAngle = angleDegrees;

  // Digital LPF to further remove ADC read noise
  filteredAngle = ALPHA * filteredAngle + (1 - ALPHA) * angleDegrees;

  return filteredAngle;  // no zero offset, no wrap logic, just direct mapping
}

// ----------------------------
// Serial plot helper (fixed 0–360 range)
// ----------------------------
void plotAngle(float angle) {
  Serial.print(0.0);     Serial.print(",");  // lower bound
  Serial.print(360.0);   Serial.print(",");  // upper bound
  Serial.println(angle);                     // encoder trace
}

// ----------------------------
// FreeRTOS Tasks
// ----------------------------

void motorTask(void *parameter) {

  for (;;) {
    // Rotate one full revolution (200 steps for 1.8° stepper)
    for (int j = 0; j < i; j++) {
      digitalWrite(driver_step_pin, HIGH);
      delayMicroseconds(500); // 1 ms HIGH
      digitalWrite(driver_step_pin, LOW);
      delayMicroseconds(500); // 1 ms LOW
    }
    delay(1000); // Wait 1 second before next rotation

    vTaskDelay(pdMS_TO_TICKS(1)); // small yield
  }
}

void encoderTask(void *parameter) {
  for (;;) {
    float angle = readEncoderDeg();
    plotAngle(angle);

    // Adjust the rate if you want more/less samples
    vTaskDelay(pdMS_TO_TICKS(2)); // ~100 Hz
  }
}

// ----------------------------
// Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(driver_dir_pin, OUTPUT);
  pinMode(driver_step_pin, OUTPUT);
  pinMode(driver_enable_pin, OUTPUT);
  pinMode(encoder_out_pin, OUTPUT);
  pinMode(encoder_dir_pin, OUTPUT);

  digitalWrite(driver_ms1_pin, HIGH);
  digitalWrite(driver_ms2_pin, HIGH);
  digitalWrite(driver_ms3_pin, HIGH);

  // Enable the driver
  digitalWrite(driver_enable_pin, LOW); // LOW to enable
  // Set direction
  digitalWrite(driver_dir_pin, HIGH);
  digitalWrite(encoder_dir_pin, HIGH);

  analogReadResolution(12);

  // Create tasks: put them on different cores if you like
  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    2048,
    NULL,
    1,
    NULL,
    1      // core 1
  );

  xTaskCreatePinnedToCore(
    encoderTask,
    "EncoderTask",
    2048,
    NULL,
    1,
    NULL,
    0      // core 0
  );
}

// ----------------------------
// Loop (unused with FreeRTOS tasks)
// ----------------------------
void loop() {
  // nothing here; tasks are doing the work
}
