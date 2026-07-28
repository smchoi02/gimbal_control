#pragma once

#include <math.h>
#include <stdint.h>

#include "../common/types.h"
#include "../config/system_config.h"
#include "attitude_math.h"

namespace target_geometry {

constexpr float METERS_PER_I7_LAT = 0.0111320f;
constexpr float BARO_EXPONENT = 0.190263f;
constexpr float BARO_SCALE_M = 44330.77f;

// Positive result means the remote transmitter is above this payload.
inline float relativeAltitudeM(float localPressurePa, float remotePressurePa) {
  if (localPressurePa <= 0.0f || remotePressurePa <= 0.0f) return 0.0f;
  return BARO_SCALE_M *
         (1.0f - powf(remotePressurePa / localPressurePa, BARO_EXPONENT));
}

inline bool relativeNed(const GpsFix& local, const BarometerSample& localBaro,
                        const RemoteTargetSample& remote, float ned[3]) {
  if (!local.valid || !localBaro.valid || !remote.valid) return false;

  const int64_t dLatI7 = static_cast<int64_t>(remote.latI7) - local.latI7;
  const int64_t dLonI7 = static_cast<int64_t>(remote.lonI7) - local.lonI7;
  const double localLatDeg = static_cast<double>(local.latI7) * 1.0e-7;
  const float cosLat =
      cosf(static_cast<float>(localLatDeg) * attitude::DEG_TO_RAD_F);

  ned[0] = static_cast<float>(dLatI7) * METERS_PER_I7_LAT;
  ned[1] = static_cast<float>(dLonI7) * METERS_PER_I7_LAT * cosLat;
  const float remoteAboveM =
      relativeAltitudeM(localBaro.pressurePa, remote.pressurePa);
  if (!isfinite(remoteAboveM) ||
      fabsf(remoteAboveM) > cfg::MAX_ABS_RELATIVE_ALT_M) {
    return false;
  }
  ned[2] = -remoteAboveM;  // NED down-positive.
  return true;
}

inline bool pointingAngles(const float ned[3], const float q[4],
                           RelativeTarget* result) {
  const float horizontal = sqrtf(ned[0] * ned[0] + ned[1] * ned[1]);
  const float range =
      sqrtf(horizontal * horizontal + ned[2] * ned[2]);
  if (!isfinite(range) || range < cfg::MIN_TARGET_RANGE_M) return false;

  float body[3];
  attitude::nedToBody(ned, q, body);
  attitude::bodyToGimbal(body, &result->yawDeg, &result->pitchDeg);
  result->northM = ned[0];
  result->eastM = ned[1];
  result->downM = ned[2];
  result->horizontalM = horizontal;
  result->rangeM = range;
  result->valid = isfinite(result->yawDeg) && isfinite(result->pitchDeg);
  return result->valid;
}

inline void normalizeDirection(const float input[3], float output[3]) {
  const float n = sqrtf(input[0] * input[0] + input[1] * input[1] +
                        input[2] * input[2]);
  if (n > 1.0e-6f) {
    output[0] = input[0] / n;
    output[1] = input[1] / n;
    output[2] = input[2] / n;
  }
}

}  // namespace target_geometry
