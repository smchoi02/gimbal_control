// Standalone BMP581 read test for OpenRB-150.
// Serial Monitor: 115200 baud. The test tries I2C addresses 0x46 then 0x47.

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

BMP581 bmp;
bool present = false;
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  present = bmp.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) == BMP5_OK ||
            bmp.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) == BMP5_OK;
  Serial.println(F("# BMP581 standalone read test"));
  Serial.println(present ? F("# OK") : F("# FAIL: check I2C wiring/address"));
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;

  bmp5_sensor_data sample = {0, 0};
  const bool valid = present && bmp.getSensorData(&sample) == BMP5_OK &&
                     sample.pressure > 0.0f;
  Serial.print(F("present/valid="));
  Serial.print(present ? 1 : 0);
  Serial.print('/');
  Serial.print(valid ? 1 : 0);
  Serial.print(F(" pressure_pa="));
  Serial.print(valid ? sample.pressure : 0.0f, 1);
  Serial.print(F(" temperature_c="));
  Serial.println(valid ? sample.temperature : 0.0f, 2);
}
