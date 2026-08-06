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
  void stow(float dtSeconds) { command(0.0f, 0.0f, dtSeconds); }
  void setTorque(bool enabled);
  // Make the axes' current physical positions the gimbal 0-degree pose.
  bool calibrateZero();
  void pollFeedback();

  const GimbalState& state() const { return state_; }
  // Servo angles as the DYNAMIXEL reports them, before the zero offset is
  // removed. These are what you align the horns against when mounting.
  float yawRawDeg() const { return yawRawDeg_; }
  float pitchRawDeg() const { return pitchRawDeg_; }
  float yawZeroDeg() const { return yawZeroDeg_; }
  float pitchZeroDeg() const { return pitchZeroDeg_; }

 private:
  Dynamixel2Arduino dxl_;
  GimbalState state_;
  float filteredYawDeg_ = 0.0f;
  float filteredPitchDeg_ = 0.0f;
  float yawZeroDeg_ = cfg::DXL_CENTER_DEG;
  float pitchZeroDeg_ = cfg::DXL_CENTER_DEG;
  float yawRawDeg_ = 0.0f;
  float pitchRawDeg_ = 0.0f;

  static float wrapDeg(float angle);
  static float servoDeg(float angle);
  static float clampValue(float value, float low, float high);
  float rateLimit(float target, float current, float maxStep, bool* limited);
};
