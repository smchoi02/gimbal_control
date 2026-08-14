#pragma once

#include <stdint.h>

enum class TrackMode : uint8_t {
  STOW = 0,
  TRACK = 1,
  HOLD_LAST_DIRECTION = 2,
  FAULT = 3
};

struct AttitudeSample {
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // body -> NED, w/x/y/z
  float yawDeg = 0.0f;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  // Body-axis angular velocity derived from successive BNO085 quaternions.
  float angularRateDps[3] = {};
  uint8_t accuracy = 0;
  uint32_t timestampMs = 0;
  bool valid = false;
};

struct BarometerSample {
  float pressurePa = 0.0f;
  float temperatureC = 0.0f;
  uint32_t timestampMs = 0;
  bool valid = false;
};

struct GpsFix {
  uint32_t iTowMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  int32_t hMslMm = 0;
  int32_t velNMmS = 0;
  int32_t velEMmS = 0;
  int32_t velDMmS = 0;
  uint8_t fixType = 0;
  uint8_t numSv = 0;
  uint32_t timestampMs = 0;
  bool valid = false;
};

struct RemoteTargetSample {
  uint8_t sequence = 0;
  uint32_t senderTimeMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  float aglM = 0.0f;
  float velNMps = 0.0f;
  float velEMps = 0.0f;
  float velDMps = 0.0f;
  uint8_t imuFlags = 0;
  uint8_t quatAccuracy = 0;
  uint16_t imuAgeMs = 0;
  float accelMps2[3] = {};
  float gyroDps[3] = {};
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  uint8_t fixType = 0;
  uint32_t timestampMs = 0;
  bool valid = false;
};

struct RelativeTarget {
  float northM = 0.0f;
  float eastM = 0.0f;
  float downM = 0.0f;
  float horizontalM = 0.0f;
  float rangeM = 0.0f;
  float yawDeg = 0.0f;
  float pitchDeg = 0.0f;
  bool valid = false;
};

struct GimbalState {
  float yawCommandDeg = 0.0f;
  float pitchCommandDeg = 0.0f;
  float yawPresentDeg = 0.0f;
  float pitchPresentDeg = 0.0f;
  int16_t yawCurrentRaw = 0;
  int16_t pitchCurrentRaw = 0;
  int8_t yawTemperatureC = 0;
  int8_t pitchTemperatureC = 0;
  bool limitActive = false;
  bool torqueOn = false;
  bool healthy = false;
  bool yawOnline = false;
  bool pitchOnline = false;
};
