#pragma once

#include <stdint.h>

enum class TrackMode : uint8_t {
  BOOT = 0,
  WAIT_FOR_DATA = 1,
  ALIGNING = 2,
  TRACKING = 3,
  SAFE_HOLD = 4,
  FAULT = 5
};

// SAFE_HOLD preserves the last valid line of sight with live IMU compensation;
// it never means "go home".
enum class TrackingBlocker : uint8_t {
  NONE = 0,
  GIMBAL_FAULT,
  ALIGNMENT_IN_PROGRESS,
  ALIGNMENT_UNSTABLE,
  BARO_REFERENCE_NOT_READY,
  BARO_BIAS_NOT_READY,
  REMOTE_PACKET_STALE,
  REMOTE_GPS_INVALID,
  LOCAL_GPS_INVALID,
  LOCAL_BARO_INVALID,
  IMU_INVALID,
  RANGE_TOO_SMALL,
  GEOMETRY_INVALID
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
  // Altitude above this payload's own pressure reference. It is not MSL.
  float aglM = 0.0f;
  float referencePressurePa = 0.0f;
  uint32_t timestampMs = 0;
  bool referenceValid = false;
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
  uint8_t month = 0;  // UTC month from UBX NAV-PVT; zero when unavailable.
  uint8_t day = 0;    // UTC day from UBX NAV-PVT; zero when unavailable.
  uint32_t timestampMs = 0;
  bool dateValid = false;
  bool valid = false;
};

struct RemoteTargetSample {
  uint8_t sequence = 0;
  // UBX NAV-PVT iTOW from the rocket. Freshness always uses timestampMs,
  // the local E22 receive time, rather than this sender clock.
  uint32_t senderITowMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  float aglM = 0.0f;
  float velNMps = 0.0f;
  float velEMps = 0.0f;
  float velDMps = 0.0f;
  // final_test byte 3: UBX NAV-PVT fixType, not satellite count.
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
  // Continuous equivalents retained across the +/-180 degree display seam.
  // Motor commands remain single-turn unless the hardware mode is changed.
  float yawCommandUnwrappedDeg = 0.0f;
  float yawPresentDeg = 0.0f;
  float pitchPresentDeg = 0.0f;
  float yawPresentUnwrappedDeg = 0.0f;
  int16_t yawCurrentRaw = 0;
  int16_t pitchCurrentRaw = 0;
  int8_t yawTemperatureC = 0;
  int8_t pitchTemperatureC = 0;
  uint32_t lastFeedbackUs = 0;
  uint32_t maxFeedbackUs = 0;
  uint32_t slowFeedbackCount = 0;
  uint32_t lastControlIntervalUs = 0;
  uint32_t maxControlIntervalUs = 0;
  uint32_t controlOverrunCount = 0;
  bool limitActive = false;
  bool torqueOn = false;
  bool healthy = false;
  bool yawOnline = false;
  bool pitchOnline = false;
};
