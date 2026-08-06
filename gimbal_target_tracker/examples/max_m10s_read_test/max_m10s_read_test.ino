// MAX-M10S 단독 읽기 시험 (I2C address 0x42).
// NAV-PVT를 10 Hz로 요청하고 유효한 fix를 시리얼에 출력한다.

#include <Arduino.h>
#include <Wire.h>

#include "../../src/sensors/max_m10s_sensor.h"
#include "../../src/sensors/max_m10s_sensor.cpp"

constexpr uint8_t MAX_M10S_ADDRESS = 0x42;

MaxM10sSensor gps(Wire);
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("# MAX-M10S read test"));
  if (!gps.begin(MAX_M10S_ADDRESS)) {
    Serial.println(F("# FAIL: MAX-M10S not found at I2C 0x42"));
  } else {
    Serial.println(F("# OK: waiting for NAV-PVT and GNSS fix"));
  }
}

void loop() {
  const uint32_t now = millis();
  gps.poll(now);

  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;

  const GpsFix& f = gps.fix();
  Serial.print(F("present/valid="));
  Serial.print(gps.present());
  Serial.print('/');
  Serial.print(f.valid);
  Serial.print(F(" fix/sv="));
  Serial.print(f.fixType);
  Serial.print('/');
  Serial.print(f.numSv);
  Serial.print(F(" lat/lon="));
  Serial.print(static_cast<double>(f.latI7) * 1.0e-7, 7);
  Serial.print('/');
  Serial.print(static_cast<double>(f.lonI7) * 1.0e-7, 7);
  Serial.print(F(" hMSL_m="));
  Serial.println(static_cast<float>(f.hMslMm) * 0.001f, 2);
}
