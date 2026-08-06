// BMP581 단독 읽기 시험 (OpenRB-150 I2C: D11=SDA, D12=SCL).
// Serial Monitor: 115200 baud

#include <Arduino.h>
#include <Wire.h>

#include "../../src/sensors/bmp581_sensor.h"
#include "../../src/sensors/bmp581_sensor.cpp"

constexpr uint8_t BMP581_ADDRESS = 0x47;  // 센서가 0x46이면 변경.

Bmp581Sensor barometer(Wire);
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("# BMP581 read test"));
  if (!barometer.begin(BMP581_ADDRESS)) {
    Serial.println(F("# FAIL: BMP581 not found; check I2C address 0x46/0x47"));
  } else {
    Serial.print(F("# OK chip ID=0x"));
    Serial.println(barometer.chipId(), HEX);
  }
}

void loop() {
  const uint32_t now = millis();
  barometer.poll(now);

  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;

  const BarometerSample& s = barometer.sample();
  Serial.print(F("present/valid="));
  Serial.print(barometer.present());
  Serial.print('/');
  Serial.print(s.valid);
  Serial.print(F(" pressure_pa="));
  Serial.print(s.pressurePa, 1);
  Serial.print(F(" temperature_c="));
  Serial.println(s.temperatureC, 2);
}
