#define AS5600ADCPIN 12
#define ADCRESOLUTION 4095 // 12-bit ADC so max values it can produce: 2^12 = 4096
#define VREF 3.3  // ESP32 default voltage reference  
#define ALPHA 0.3 // Digital LPF strength (higher = smoother, but slower)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set 12-bit resolution (0–4095)
  delay(1000);
}

void loop() {
  int adcValue = analogRead(AS5600ADCPIN);
  // (adcValue / (float)adcResolution) gives the percentage of full scale the input voltage is.
  // multiplying by vRef converts that percentage into an actual voltage value (0V to vRef range)
  float voltage = (adcValue / (float)ADCRESOLUTION) * VREF;

  // Map voltage to angle (AS5600 outputs 0–Vref for 0–360 degrees)
  // (voltage / vRef) gives us the percentage of the full-scale angle (0–1)
  // multiplying by 360 converts that percentage into an angle in degrees (0° to 360°)
  float angleDegrees = (voltage / VREF) * 360.0;
  static float filteredAngle = angleDegrees;

// Digital LPF to further remove ADC read noise
  filteredAngle = ALPHA * filteredAngle + (1 - ALPHA) * angleDegrees;
  Serial.print(0.0);     Serial.print(",");   // Lower bound
  Serial.print(360.0);   Serial.print(",");   // Upper bound
  Serial.println(filteredAngle);

  // Serial.print("ADC Value: ");
  // Serial.print(adcValue);
  // Serial.print(" | Voltage: ");
  // Serial.print(voltage, 3);
  // Serial.print(" V | Angle: ");
  //Serial.println(angleDegrees, 1);
  // Serial.println("°");

  delay(20);
}