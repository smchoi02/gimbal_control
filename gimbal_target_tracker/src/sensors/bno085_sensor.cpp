#include "bno085_sensor.h"

#include "../math/attitude_math.h"

bool Bno085Sensor::begin(uint8_t address, uint16_t reportIntervalMs,
                        bool useGameRotationVector) {
  reportIntervalMs_ = reportIntervalMs;
  useGameRotationVector_ = useGameRotationVector;
  present_ = device_.begin(address, wire_);
  if (!present_) return false;

  reportEnabled_ =
      useGameRotationVector_
          ? device_.enableGameRotationVector(reportIntervalMs_)
          : device_.enableRotationVector(reportIntervalMs_);
  lastRecoveryAttemptMs_ = millis();
  return reportEnabled_;
}

bool Bno085Sensor::poll(uint32_t nowMs) {
  if (!present_) return false;

  // A BNO08x reset clears all enabled reports. Re-enable the rotation-vector
  // report immediately, otherwise the sensor remains present but imu=0 forever.
  if (device_.wasReset()) {
    ++resetCount_;
    sample_.valid = false;
    restoreRotationVectorReport(nowMs);
  }

  // Also retry when no event has arrived for a while. This covers a missed
  // reset notification without repeatedly configuring the sensor every loop.
  const bool reportStale =
      !sample_.valid || (nowMs - sample_.timestampMs > 500U);
  if (reportStale && nowMs - lastRecoveryAttemptMs_ >= 1000U) {
    restoreRotationVectorReport(nowMs);
  }

  if (!device_.getSensorEvent()) return false;
  const uint8_t eventId = device_.getSensorEventID();
  const uint8_t expectedId =
      useGameRotationVector_ ? SENSOR_REPORTID_GAME_ROTATION_VECTOR
                             : SENSOR_REPORTID_ROTATION_VECTOR;
  if (eventId != expectedId) return false;

  float newQ[4];
  if (useGameRotationVector_) {
    newQ[0] = device_.getGameQuatReal();
    newQ[1] = device_.getGameQuatI();
    newQ[2] = device_.getGameQuatJ();
    newQ[3] = device_.getGameQuatK();
  } else {
    newQ[0] = device_.getQuatReal();
    newQ[1] = device_.getQuatI();
    newQ[2] = device_.getQuatJ();
    newQ[3] = device_.getQuatK();
  }
  attitude::normalizeQuaternion(newQ);
  if (sample_.valid && nowMs != sample_.timestampMs) {
    float deltaQ[4];
    attitude::relativeToReference(sample_.q, newQ, deltaQ);
    // q and -q are identical rotations. Use the shortest delta rotation.
    if (deltaQ[0] < 0.0f) {
      for (uint8_t i = 0; i < 4; ++i) deltaQ[i] = -deltaQ[i];
    }
    const float sinHalfAngle = sqrtf(deltaQ[1] * deltaQ[1] +
                                     deltaQ[2] * deltaQ[2] +
                                     deltaQ[3] * deltaQ[3]);
    const float dtSeconds = static_cast<uint32_t>(nowMs - sample_.timestampMs) * 1.0e-3f;
    if (sinHalfAngle > 1.0e-6f && dtSeconds > 1.0e-4f) {
      const float radiansPerSecond = 2.0f * atan2f(sinHalfAngle, deltaQ[0]) / dtSeconds;
      const float scale = radiansPerSecond * attitude::RAD_TO_DEG_F / sinHalfAngle;
      for (uint8_t i = 0; i < 3; ++i) sample_.angularRateDps[i] = deltaQ[i + 1] * scale;
    } else {
      sample_.angularRateDps[0] = sample_.angularRateDps[1] =
          sample_.angularRateDps[2] = 0.0f;
    }
  }
  for (uint8_t i = 0; i < 4; ++i) sample_.q[i] = newQ[i];
  attitude::quaternionToEuler(sample_.q, &sample_.yawDeg,
                              &sample_.pitchDeg, &sample_.rollDeg);
  sample_.accuracy =
      useGameRotationVector_ ? 3 : device_.getQuatAccuracy();
  sample_.timestampMs = nowMs;
  sample_.valid = true;
  return true;
}

void Bno085Sensor::restoreRotationVectorReport(uint32_t nowMs) {
  reportEnabled_ =
      useGameRotationVector_
          ? device_.enableGameRotationVector(reportIntervalMs_)
          : device_.enableRotationVector(reportIntervalMs_);
  lastRecoveryAttemptMs_ = nowMs;
  ++reportRecoveryCount_;
}
