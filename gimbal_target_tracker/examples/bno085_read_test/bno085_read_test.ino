// Standalone BNO085 read test for OpenRB-150.
// Serial Monitor: 115200 baud. Change address to 0x4B if ADR is high.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <math.h>

constexpr uint8_t BNO085_ADDRESS = 0x4A;
BNO08x imu;
bool present = false;
uint32_t lastPrintMs = 0;
float yawDeg = 0.0f, pitchDeg = 0.0f, rollDeg = 0.0f;
uint8_t accuracy = 0;
bool haveSample = false;

void updateEuler(float w, float x, float y, float z) {
  const float yaw = atan2f(2.0f * (w * z + x * y),
                           1.0f - 2.0f * (y * y + z * z));
  float sinePitch = 2.0f * (w * y - z * x);
  if (sinePitch > 1.0f) sinePitch = 1.0f;
  if (sinePitch < -1.0f) sinePitch = -1.0f;
  const float pitch = asinf(sinePitch);
  const float roll = atan2f(2.0f * (w * x + y * z),
                            1.0f - 2.0f * (x * x + y * y));
  constexpr float RAD_TO_DEG = 57.2957795131f;
  yawDeg = yaw * RAD_TO_DEG;
  pitchDeg = pitch * RAD_TO_DEG;
  rollDeg = roll * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  present = imu.begin(BNO085_ADDRESS, Wire);
  if (present) present = imu.enableGameRotationVector(20);
  Serial.println(F("# BNO085 standalone read test"));
  Serial.println(present ? F("# OK: Game Rotation Vector enabled")
                         : F("# FAIL: check I2C address/wiring"));
}

void loop() {
  if (present && imu.getSensorEvent() &&
      imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    updateEuler(imu.getGameQuatReal(), imu.getGameQuatI(),
                imu.getGameQuatJ(), imu.getGameQuatK());
    accuracy = 3;
    haveSample = true;
  }

  const uint32_t now = millis();
  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;
  Serial.print(F("present/valid="));
  Serial.print(present ? 1 : 0);
  Serial.print('/');
  Serial.print(haveSample ? 1 : 0);
  Serial.print(F(" yaw/pitch/roll="));
  Serial.print(yawDeg, 1);
  Serial.print('/');
  Serial.print(pitchDeg, 1);
  Serial.print('/');
  Serial.print(rollDeg, 1);
  Serial.print(F(" accuracy="));
  Serial.println(accuracy);
}
