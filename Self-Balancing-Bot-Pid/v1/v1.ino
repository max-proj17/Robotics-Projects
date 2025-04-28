/*
 * Filename: StepperMotorControl.ino
 * Description: 
 * Author: Maximelio Finch
 * Date: 4/11/2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// m1:    m2:    imu: good
// Motor 1
#define M1_STEPPIN 8
#define M1_DIRPIN 3
#define M1_ENPIN 46
#define M1_AS5600ADCPIN 9

// Motor 2  issue! with adc
#define M2_STEPPIN 13
#define M2_DIRPIN 11
#define M2_ENPIN 12
#define M2_AS5600ADCPIN 10

#define ADCRESOLUTION 4095 // 12-bit ADC so max values it can produce: 2^12 = 4096
#define VREF 3.3
#define ALPHA 0.5 // Encoder filter: Digital LPF strength (higher = smoother, but slower)
#define GYRO_ALPHA 0.7 // Gyro LPF
#define MPU_ADDRESS 0x68
#define PIN_SCL 15
#define PIN_SDA 16

int stepsPerCycle = 3200;

// Filtered encoder angles
float enc1Filtered = 0, enc2Filtered = 0;
float enc1Offset = 0, enc2Offset = 0; // for zeroing

// IMU
MPU9250_asukiaaa imu;
float filteredGyroX = 0, filteredGyroY = 0, filteredGyroZ = 0;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;

// Task handles
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t imuTaskHandle = nullptr;

void sensorTask(void *pvParameters) {
  for (;;) {
    int adc1 = analogRead(M1_AS5600ADCPIN);
    int adc2 = analogRead(M2_AS5600ADCPIN);
    float v1 = (adc1 / (float)ADCRESOLUTION) * VREF;
    float v2 = (adc2 / (float)ADCRESOLUTION) * VREF;

    float angle1 = (v1 / VREF) * 360.0;
    float angle2 = (v2 / VREF) * 360.0;

    static float f1 = angle1, f2 = angle2;
    f1 = ALPHA * f1 + (1 - ALPHA) * angle1;
    f2 = ALPHA * f2 + (1 - ALPHA) * angle2;

    enc1Filtered = f1 - enc1Offset;
    enc2Filtered = f2 - enc2Offset;

    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
  }
}

void imuTask(void *parameter) {
  unsigned long lastTime = millis();
  for (;;) {
    imu.accelUpdate();
    imu.gyroUpdate();
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float gx = imu.gyroX();
    float gy = imu.gyroY();
    float gz = imu.gyroZ();

    filteredGyroX = GYRO_ALPHA * filteredGyroX + (1 - GYRO_ALPHA) * gx;
    filteredGyroY = GYRO_ALPHA * filteredGyroY + (1 - GYRO_ALPHA) * gy;
    filteredGyroZ = GYRO_ALPHA * filteredGyroZ + (1 - GYRO_ALPHA) * gz;

    angleX += filteredGyroX * dt;
    angleY += filteredGyroY * dt;
    angleZ += filteredGyroZ * dt;

    // Serial Output for plotting
    Serial.print(0.0, 5); Serial.print(",");         // Lower bound reference
    Serial.print(360.0, 5); Serial.print(",");       // Upper bound reference

    Serial.print(angleY, 2); Serial.print(",");      // IMU pitch angle
    Serial.print(enc1Filtered, 2); Serial.print(",");// Encoder 1 angle
    Serial.println(enc2Filtered, 2);                 // Encoder 2 angle

    vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz
  }
}

void motorTask(void *pvParameters) {
  for (;;) {
    for (int j = 0; j < stepsPerCycle; j++) {
      digitalWrite(M1_STEPPIN, HIGH);
      digitalWrite(M2_STEPPIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(M1_STEPPIN, LOW);
      digitalWrite(M2_STEPPIN, LOW);
      delayMicroseconds(1000);
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
  }
}

// void monitorTask(void *pvParameters) {
//   for (;;) {
//     UBaseType_t sensorStack = uxTaskGetStackHighWaterMark(sensorTaskHandle);
//     UBaseType_t motorStack = uxTaskGetStackHighWaterMark(motorTaskHandle);
//     Serial.print("Sensor Task Stack High Water Mark: ");
//     Serial.print(sensorStack);
//     Serial.println(" bytes");
//     Serial.print("Motor Task Stack High Water Mark: ");
//     Serial.print(motorStack);
//     Serial.println(" bytes");
//     vTaskDelay(pdMS_TO_TICKS(10000));
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(M1_STEPPIN, OUTPUT); pinMode(M1_DIRPIN, OUTPUT); pinMode(M1_ENPIN, OUTPUT);
  pinMode(M2_STEPPIN, OUTPUT); pinMode(M2_DIRPIN, OUTPUT); pinMode(M2_ENPIN, OUTPUT);
  digitalWrite(M1_ENPIN, LOW); digitalWrite(M1_DIRPIN, HIGH);
  digitalWrite(M2_ENPIN, LOW); digitalWrite(M2_DIRPIN, HIGH);

  analogReadResolution(12);
  Wire.begin(PIN_SDA, PIN_SCL);
  imu.setWire(&Wire);
  imu.beginAccel(); imu.beginGyro();

  Serial.println("Hold robot upright to calibrate zero...");

  // Allow sensors to settle
  delay(2000);
  imu.accelUpdate(); imu.gyroUpdate();
  angleY = 0.0;  // IMU is upright at start

  // Read encoder baseline
  int adc1 = analogRead(M1_AS5600ADCPIN);
  int adc2 = analogRead(M2_AS5600ADCPIN);
  enc1Offset = ((adc1 / (float)ADCRESOLUTION) * 360.0);
  enc2Offset = ((adc2 / (float)ADCRESOLUTION) * 360.0);

  Serial.println("Zeroed. Starting tasks...");

  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 1536, NULL, 2, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 1, &imuTaskHandle, 1);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 1280, NULL, 1, &motorTaskHandle, 1);
  // xTaskCreatePinnedToCore(monitorTask, "Monitor Task", 2048, NULL, 1, NULL, 0);
}

void loop() {

}
