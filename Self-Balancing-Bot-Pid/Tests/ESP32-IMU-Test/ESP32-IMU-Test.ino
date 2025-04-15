#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// Only i2c, no interrupts

// Pin Def
#define PIN_SCL    15
#define PIN_SDA    16
#define PIN_INT    3
#define GYRO_ALPHA 0.7
float filteredGyroX = 0, filteredGyroY = 0, filteredGyroZ = 0;

#define MPU_ADDRESS 0x68  // ADO low = 0x68


MPU9250_asukiaaa imu;
TaskHandle_t imuTaskHandle = nullptr;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;


void imuTask(void *parameter) {
  unsigned long lastTime = millis();
  while (true) {
    imu.accelUpdate();
    imu.gyroUpdate();

    // Delta time
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Raw gyro readings
    float gx = imu.gyroX();
    float gy = imu.gyroY();
    float gz = imu.gyroZ();

    // Apply digital LPF
    filteredGyroX = GYRO_ALPHA * filteredGyroX + (1 - GYRO_ALPHA) * gx;
    filteredGyroY = GYRO_ALPHA * filteredGyroY + (1 - GYRO_ALPHA) * gy;
    filteredGyroZ = GYRO_ALPHA * filteredGyroZ + (1 - GYRO_ALPHA) * gz;

    // Integration to get angle (in degrees)
    angleX += filteredGyroX * dt;
    angleY += filteredGyroY * dt;
    angleZ += filteredGyroZ * dt;



    // Serial.print(imu.accelX()); Serial.print('\t');
    // Serial.print(imu.accelY()); Serial.print('\t');
    // // Serial.print(imu.accelZ()); Serial.print('\t');
    // Serial.print(filteredGyroX); Serial.print('\t');
    // Serial.print(filteredGyroY); Serial.print('\t');
    // Serial.print(filteredGyroZ); Serial.print('\t');
    // Serial.print(100);    Serial.print('\t');
    // Serial.println(-100);
    // Output angles
    Serial.print(angleX); Serial.print('\t');
    Serial.print(angleY); Serial.print('\t');
    Serial.print(angleZ); Serial.print('\t');
    Serial.print(0);    Serial.print('\t');
    Serial.println(360);


    vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz
  }
}

void setup() {
  Serial.begin(115200);
  // Serial.println("AccelX\tAccelY\tAccelZ\tGyroX\tGyroY\tGyroZ");
  delay(1000);

  Wire.begin(PIN_SDA, PIN_SCL);
  imu.setWire(&Wire);
  imu.beginAccel();
  imu.beginGyro();

  Serial.println("MPU9250 initialized (Accel + Gyro only).");

  xTaskCreatePinnedToCore(
    imuTask,
    "IMU Task",
    4096,
    NULL,
    1,
    &imuTaskHandle,
    1
  );
}


void loop() {
  //vTaskDelay(pdMS_TO_TICKS(1000)); 
}
