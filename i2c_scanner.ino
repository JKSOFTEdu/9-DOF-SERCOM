/*
  Uses:
  The library for the LSM9DS0 Accelerometer and magnentometer/compass
  by Adafruit that was written by Kevin Townsend for Adafruit Industries.  

  Adaptations by J.A. Korten / JKSOFT Educational (c) 2017.
  Aug. 15 2017

  BSD license, all text above must be included in any redistribution
*/


#define sensorGyroAddr   0x6B
#define sensorAccMagAddr 0x1D

void setupTWI()
{
  Serial.begin(115200);
  sensorTWI.begin();
  //Wire.begin();

  // Assign pins 4 & 3 to SERCOM functionality
  pinPeripheral(4, PIO_SERCOM_ALT);
  pinPeripheral(3, PIO_SERCOM_ALT);
  if (showDebugMsg) {
    delay(2000);
    Serial.println("RobotPatient Sensors - Ready to detect!");
  }
}


void detectTWI()
{
  byte error, address;
  int nDevices;
  if (showDebugMsg) {
    Serial.println("Scanning bus 1: PA08/PA09 on SERCOM2");
  }
  // alt pins to avoid inference with serial!
  digitalWrite(12, HIGH);

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    digitalWrite(12, LOW);
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    sensorTWI.beginTransmission(address);
    error = sensorTWI.endTransmission();

    if (error == 0)
    {
      digitalWrite(12, HIGH);
      if (showDebugMsg) {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");
      }

      nDevices++;
    }
    else if (error == 4)
    {
      if (showDebugMsg) {
        Serial.print("Unknow error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }
  if (showDebugMsg) {
    if (nDevices == 0)
      Serial.println("No I2C devices found on bus 1\n");
    else
      Serial.println("done\n");

    Serial.println("Scanning bus 2: PA22/PA23 on SERCOM3");
  }
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    digitalWrite(12, LOW);
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      digitalWrite(12, HIGH);
      if (showDebugMsg) {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");
      }
      nDevices++;
    }
    else if (error == 4)
    {
      if (showDebugMsg) {
        Serial.print("Unknow error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }
  if (showDebugMsg) {
    if (nDevices == 0) {
      Serial.println("No I2C devices found on bus 2\n");
    } else {
      Serial.println("done\n");
    }
  }

  delay(5000);           // wait 5 seconds for next scan
}

boolean check9DOFSensors() {
  sensorTWI.beginTransmission(0x1D); // Acc / Mag
  int error0 = sensorTWI.endTransmission();

  sensorTWI.beginTransmission(0x6B); // Gyro
  int error1 = sensorTWI.endTransmission();

  sAccMagDetected = (error0 == 0);
  sGyroDetected   = (error1 == 0);
  return sAccMagDetected && sGyroDetected;
}
