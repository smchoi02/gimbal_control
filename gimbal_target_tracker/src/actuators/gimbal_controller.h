#pragma once

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include "../common/types.h"
#include "../config/system_config.h"

class GimbalController {
 public:
  GimbalController(HardwareSerial& serial, int directionPin);

  bool begin();
  void command(float yawDeg, float pitchDeg, float dtSeconds);
  void setTorque(bool enabled);
  // Capture the current encoder positions as RAM-only gimbal 0-degree poses.
  // This does not write DYNAMIXEL EEPROM or alter hardware offsets.
  bool captureSoftZero();
  void pollFeedback();

  const GimbalState& state() const { return state_; }

 private:
  Dynamixel2Arduino dxl_;
  GimbalState state_;
  float filteredYawDeg_ = 0.0f;
  float filteredPitchDeg_ = 0.0f;
  float yawRateDegS_ = 0.0f;
  float pitchRateDegS_ = 0.0f;
  float yawZeroDeg_ = cfg::DXL_CENTER_DEG;
  float pitchZeroDeg_ = cfg::DXL_CENTER_DEG;

  static float wrapDeg(float angle);
  static float servoDeg(float angle);
  static float clampValue(float value, float low, float high);
  static float moveToward(float value, float target, float maxDelta);
  float smoothAxis(float target, float current, float* rateDegS,
                   float dtSeconds, bool circular, bool* limited);
};
