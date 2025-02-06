#include <Wire.h>

#define RLED 4
#define GLED 11
#define GYRO_SENS 32.8 // Sensitivity scale factor for +/-1000 deg/s
#define STATIONARY_TIME 5000 // 5 seconds

int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ, gyroMag;
float biasX = 0, biasY = 0, biasZ = 0;
float thresholdValue; // Corrected threshold
unsigned long stationaryStart = 0;
bool isStationary = false;

void setup() {
  Serial.begin(9600);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  digitalWrite(RLED, LOW);
  digitalWrite(GLED, HIGH);
  Wire.begin();
  setupMPU();
  calibrateGyro();
  threshold();  // Call threshold after calibration
  setFilterBandwidth(4);
}

void loop() {
  recordGyroRegisters();
  printData();
  detectMotion();
  delay(100);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0b00010000);
  Wire.endTransmission(); 
}

void calibrateGyro() {
  int samples = 200;
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    recordGyroRegisters();
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(5);
  }
  biasX = sumX / samples;
  biasY = sumY / samples;
  biasZ = sumZ / samples;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while(Wire.available() < 6);
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
  processGyroData();
}

void processGyroData() {
  rotX = (gyroX - biasX) / GYRO_SENS;
  rotY = (gyroY - biasY) / GYRO_SENS; 
  rotZ = (gyroZ - biasZ) / GYRO_SENS;
  gyroMag = sqrt(rotX * rotX + rotY * rotY + rotZ * rotZ);
}

void detectMotion() {
  if (gyroMag > (thresholdValue - 5) && gyroMag < (thresholdValue + 2)) {  
    if (!isStationary) {
      stationaryStart = millis();
      isStationary = true;
    }
    if (millis() - stationaryStart >= STATIONARY_TIME) {
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
    }
  } else {
    isStationary = false;
    digitalWrite(RLED, LOW);
    digitalWrite(GLED, HIGH);
  }
}

void threshold() {
  float sum = 0;
  int samples = 50; // Increased sample size for better stability
  for (int j = 0; j < samples; j++) {
    recordGyroRegisters();
    sum += gyroMag;
    delay(5);  // Reduced delay for faster averaging
  }
  thresholdValue = (sum / samples);  
}

void printData() {
  Serial.print(" | Mag="); Serial.print(gyroMag);
  Serial.print(" | Threshold="); Serial.println(thresholdValue);
}

void setFilterBandwidth(uint8_t bandwidth) {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1A);
  Wire.write(bandwidth);
  Wire.endTransmission();
}
