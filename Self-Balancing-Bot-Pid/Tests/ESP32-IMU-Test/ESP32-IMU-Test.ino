#include <Arduino.h>
#include <SPI.h>
#include <SparkFunMPU9250-DMP.h>

// MPU9250 SPI setup
#define PIN_CS   7
#define PIN_SCK  15 // SCL
#define PIN_MISO 8 // ADO
#define PIN_MOSI 16 // SDA
#define PIN_INT  3  // INT pin from MPU9250

MPU9250_DMP imu;
SemaphoreHandle_t imuSemaphore;

// ISR: Called when MPU-9250 triggers an interrupt
void IRAM_ATTR imuISR() {
  // Tracks if a task was awoken in the ISR
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(imuSemaphore, &xHigherPriorityTaskWoken);

  // If the samephore awakens a higher prioirty task,
  // execute that task after this one finishes
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void sensorTask(void *pvParameters) {
  for (;;) {
    // Wait for interrupt to signal new data
    if (xSemaphoreTake(imuSemaphore, portMAX_DELAY) == pdTRUE) {
      if (imu.fifoAvailable()) {
        if (imu.dmpUpdateFifo() == INV_SUCCESS) {
          // Use only pitch/roll from gyro + accel (no magnetometer)
          Serial.print("Pitch: ");   // Angles in degrees
          Serial.print(imu.pitch, 2);
          Serial.print(" | Roll: ");
          Serial.print(imu.roll, 2);
          Serial.print(" | GyroX: "); 
          Serial.print(imu.calcGyro(imu.gx), 2); // Angular Speed
          Serial.print(" | GyroY: ");
          Serial.println(imu.calcGyro(imu.gy), 2);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Create binary semaphore
  imuSemaphore = xSemaphoreCreateBinary();

  // Initialize SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  if (imu.begin(SPI, PIN_CS) != INV_SUCCESS) {
    Serial.println("MPU9250 connection failed!");
    while (1);
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  // sets full scale range of sensors
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  // Sets Low Pass Filter to block high frequency noise
  imu.setLPF(5);
  imu.setSampleRate(50);

  imu.enableFifo(true);
  // Sensor Fusion: Fuse only gyrocope and accelerometer
  // DMP_FEATURE_GYRO_CAL: DMP will track and adjust for small drifts in gyroscope readings
  // DMP_FEATURE_6X_LP_QUAT: Sensor will output a four-part number used to describe 3-d rotations/orientations
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10);

  // Attach interrupt from MPU
  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), imuISR, RISING);

  // Start sensor task
  xTaskCreatePinnedToCore(sensorTask, "IMU Sensor Task", 4096, NULL, 2, NULL, 1);
}

void loop() {

}
