// BNO085 단독 읽기 시험 (OpenRB-150 I2C: D11=SDA, D12=SCL).
// Serial Monitor: 115200 baud

#include <Arduino.h>
#include <Wire.h>

#include "../../src/sensors/bno085_sensor.h"
#include "../../src/sensors/bno085_sensor.cpp"

constexpr uint8_t BNO085_ADDRESS = 0x4A;  // ADR high이면 0x4B로 변경.

Bno085Sensor imu(Wire);
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("# BNO085 read test"));
  if (!imu.begin(BNO085_ADDRESS, 20, true)) {
    Serial.println(F("# FAIL: BNO085 not found or report enable failed"));
  }
}

void loop() {
  const uint32_t now = millis();
  imu.poll(now);

  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;

  const AttitudeSample& s = imu.sample();
  Serial.print(F("present/report/valid="));
  Serial.print(imu.present());
  Serial.print('/');
  Serial.print(imu.reportEnabled());
  Serial.print('/');
  Serial.print(s.valid);
  Serial.print(F(" yaw/pitch/roll="));
  Serial.print(s.yawDeg, 1);
  Serial.print('/');
  Serial.print(s.pitchDeg, 1);
  Serial.print('/');
  Serial.print(s.rollDeg, 1);
  Serial.print(F(" accuracy="));
  Serial.println(s.accuracy);
}
