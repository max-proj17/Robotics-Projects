

const int STEPPIN = 11;  // Pin connected to STEP
const int DIRPIN = 9;   // Pin connected to DIR
const int ENPIN = 10;    // Pin connected to ENABLE
const int AS5600ADCPIN = 12;
const int ADCRESOLUTION = 4095; // 12-bit ADC so max values it can produce: 2^12 = 4096
const int VREF = 3.3;  // ESP32 default voltage reference  
const int ALPHA = 0.5; // Digital LPF strength (higher = smoother, but slower)


// Pin Def
#define PIN_SCL    15
#define PIN_SDA    16
#define PIN_INT    3
#define GYRO_ALPHA 0.7
float filteredGyroX = 0, filteredGyroY = 0, filteredGyroZ = 0;

#define MPU_ADDRESS 0x68  // ADO low = 0x68
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
