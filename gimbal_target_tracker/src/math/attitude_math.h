#pragma once

#include <math.h>

namespace attitude {

// The _F suffix avoids collisions with Arduino core DEG_TO_RAD/RAD_TO_DEG
// preprocessor macros.
constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
constexpr float RAD_TO_DEG_F = 57.29577951308232f;

inline void normalizeQuaternion(float q[4]) {
  const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] +
                           q[2] * q[2] + q[3] * q[3]);
  if (norm > 1.0e-9f) {
    for (uint8_t i = 0; i < 4; ++i) q[i] /= norm;
  }
}

// Returns reference^-1 * current. Both inputs describe body -> world, so the
// result describes the current body orientation in the reference body frame.
inline void relativeToReference(const float reference[4], const float current[4],
                                float out[4]) {
  const float rw = reference[0], rx = -reference[1];
  const float ry = -reference[2], rz = -reference[3];
  const float cw = current[0], cx = current[1];
  const float cy = current[2], cz = current[3];
  out[0] = rw * cw - rx * cx - ry * cy - rz * cz;
  out[1] = rw * cx + rx * cw + ry * cz - rz * cy;
  out[2] = rw * cy - rx * cz + ry * cw + rz * cx;
  out[3] = rw * cz + rx * cy - ry * cx + rz * cw;
  normalizeQuaternion(out);
}

inline void quaternionToBodyToNed(const float q[4], float r[3][3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  r[0][0] = 1 - 2 * (y * y + z * z);
  r[0][1] = 2 * (x * y - w * z);
  r[0][2] = 2 * (x * z + w * y);
  r[1][0] = 2 * (x * y + w * z);
  r[1][1] = 1 - 2 * (x * x + z * z);
  r[1][2] = 2 * (y * z - w * x);
  r[2][0] = 2 * (x * z - w * y);
  r[2][1] = 2 * (y * z + w * x);
  r[2][2] = 1 - 2 * (x * x + y * y);
}

inline void nedToBody(const float ned[3], const float q[4], float body[3]) {
  float r[3][3];
  quaternionToBodyToNed(q, r);
  body[0] = r[0][0] * ned[0] + r[1][0] * ned[1] + r[2][0] * ned[2];
  body[1] = r[0][1] * ned[0] + r[1][1] * ned[1] + r[2][1] * ned[2];
  body[2] = r[0][2] * ned[0] + r[1][2] * ned[1] + r[2][2] * ned[2];
}

inline void bodyToGimbal(const float body[3], float* yawDeg, float* pitchDeg) {
  *yawDeg = atan2f(body[1], body[0]) * RAD_TO_DEG_F;
  const float horizontal = sqrtf(body[0] * body[0] + body[1] * body[1]);
  *pitchDeg = atan2f(body[2], horizontal) * RAD_TO_DEG_F;
}

inline void quaternionToEuler(const float q[4], float* yawDeg,
                              float* pitchDeg, float* rollDeg) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  *yawDeg =
      atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * RAD_TO_DEG_F;
  float s = 2 * (w * y - z * x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  *pitchDeg = asinf(s) * RAD_TO_DEG_F;
  *rollDeg =
      atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD_TO_DEG_F;
}

}  // namespace attitude
