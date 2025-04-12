/*
 * Filename: StepperMotorControl.ino
 * Description: 
 * Author: Maximelio Finch
 * Date: 4/11/2025
 */
const int STEPPIN = 11;  // Pin connected to STEP
const int DIRPIN = 9;   // Pin connected to DIR
const int ENPIN = 10;    // Pin connected to ENABLE
const int AS5600ADCPIN = 12;
const int ADCRESOLUTION = 4095; // 12-bit ADC so max values it can produce: 2^12 = 4096
const int VREF = 3.3;  // ESP32 default voltage reference  
const int ALPHA = 0.3; // Digital LPF strength (higher = smoother, but slower)

int i = 3200;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

void sensorTask(void *pvParameters) {
  for (;;) {
    int adcValue = analogRead(AS5600ADCPIN);
    float voltage = (adcValue / (float)ADCRESOLUTION) * VREF;
    float angleDegrees = (voltage / VREF) * 360.0;
    static float filteredAngle = angleDegrees;
    filteredAngle = ALPHA * filteredAngle + (1 - ALPHA) * angleDegrees;
    Serial.print(0);           // Minimum Y-axis value
    Serial.print(",");
    Serial.print(360);         // Maximum Y-axis value
    Serial.print(",");
    Serial.println(filteredAngle, 1);
    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
  }
}

void motorTask(void *pvParameters) {
  for (;;) {
    for (int j = 0; j < i; j++) {
      digitalWrite(STEPPIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(STEPPIN, LOW);
      delayMicroseconds(1000);
    }
    i = 3200;
    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
  }
}

void monitorTask(void *pvParameters) {
  for (;;) {
    UBaseType_t sensorStack = uxTaskGetStackHighWaterMark(sensorTaskHandle);
    UBaseType_t motorStack = uxTaskGetStackHighWaterMark(motorTaskHandle);
    Serial.print("Sensor Task Stack High Water Mark: ");
    Serial.print(sensorStack);
    Serial.println(" bytes");
    Serial.print("Motor Task Stack High Water Mark: ");
    Serial.print(motorStack);
    Serial.println(" bytes");
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(ENPIN, OUTPUT);
  digitalWrite(ENPIN, LOW);
  digitalWrite(DIRPIN, HIGH);
  analogReadResolution(12);
  delay(1000);

  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 1536, NULL, 2, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 1280, NULL, 1, &motorTaskHandle, 1);
  // xTaskCreatePinnedToCore(monitorTask, "Monitor Task", 2048, NULL, 1, NULL, 0);
}

void loop() {

}
