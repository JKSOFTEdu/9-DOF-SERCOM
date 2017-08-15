/*
  Uses:
  The library for the LSM9DS0 Accelerometer and magnentometer/compass
  by Adafruit that was written by Kevin Townsend for Adafruit Industries.  

  Adaptations by J.A. Korten / JKSOFT Educational (c) 2017.
  Aug. 15 2017

  BSD license, all text above must be included in any redistribution
*/

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

#include <Adafruit_LSM9DS0.h>

// We have connected the sensor to D3 / D4 (PA08 / PA09)
// Tip! See the tutorial by Lady Ada on SERCOM for SAMD21

TwoWire sensorTWI(&sercom2, 4, 3);

const boolean showDebugMsg = true;

boolean sAccMagDetected = false;
boolean sGyroDetected = false;

Adafruit_LSM9DS0 lsm;


void setupSensors () {
  setupTWI();
  lsm = Adafruit_LSM9DS0(&sensorTWI, 1000);  // Use I2C, ID #1000
}

void setup() {
  // put your setup code here, to run once:
  //setupTWI();
  setupSensors();
  Serial.println("Checking sensors...");
  if (check9DOFSensors()) {
    Serial.println("Sensors OK...");
  } else {
    Serial.println("Sensor Issue...");
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //check9DOFSensors();
  Serial.print(".");
  if(sAccMagDetected && sGyroDetected) readSensors();
}

void readSensors(){
  lsm.read();
  //lsm.readAccel(); // (see the inside of the library, works neat!)

  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);        Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);      Serial.println(" ");
  Serial.print("Temp: "); Serial.print((int)lsm.temperature);    Serial.println(" ");
  delay(200);

}

