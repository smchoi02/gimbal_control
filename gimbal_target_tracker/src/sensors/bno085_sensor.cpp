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

  if (useGameRotationVector_) {
    sample_.q[0] = device_.getGameQuatReal();
    sample_.q[1] = device_.getGameQuatI();
    sample_.q[2] = device_.getGameQuatJ();
    sample_.q[3] = device_.getGameQuatK();
  } else {
    sample_.q[0] = device_.getQuatReal();
    sample_.q[1] = device_.getQuatI();
    sample_.q[2] = device_.getQuatJ();
    sample_.q[3] = device_.getQuatK();
  }
  attitude::normalizeQuaternion(sample_.q);
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
