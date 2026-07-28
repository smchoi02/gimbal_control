#include "../config/imu_selection.h"

#if TRACKER_USE_ICM20948

#include "icm20948_sensor.h"

#include <math.h>

#include "../math/attitude_math.h"

bool Icm20948Sensor::begin(bool preferredAd0High) {
  const uint8_t preferredAddress = preferredAd0High ? 0x69 : 0x68;
  const uint8_t alternateAddress = preferredAd0High ? 0x68 : 0x69;
  uint8_t selectedAddress = 0;
  if (addressResponds(preferredAddress)) {
    selectedAddress = preferredAddress;
  } else if (addressResponds(alternateAddress)) {
    selectedAddress = alternateAddress;
  } else {
    return false;
  }

  const bool ad0High = selectedAddress == 0x69;
  present_ = device_.begin(wire_, ad0High) == ICM_20948_Stat_Ok;
  if (!present_) return false;

  address_ = selectedAddress;
  fusion_.reset();
  sample_ = AttitudeSample();
  lastUpdateUs_ = micros();
  return true;
}

bool Icm20948Sensor::poll(uint32_t nowMs) {
  if (!present_ || !device_.dataReady()) return false;
  device_.getAGMT();
  if (device_.status != ICM_20948_Stat_Ok) {
    ++readErrors_;
    return false;
  }

  const float ax = device_.accX();
  const float ay = device_.accY();
  const float az = device_.accZ();
  const float gx = device_.gyrX() * attitude::DEG_TO_RAD_F;
  const float gy = device_.gyrY() * attitude::DEG_TO_RAD_F;
  const float gz = device_.gyrZ() * attitude::DEG_TO_RAD_F;
  const float mx = device_.magX();
  const float my = device_.magY();
  const float mz = device_.magZ();
  if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
      !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {
    ++readErrors_;
    return false;
  }

  const uint32_t nowUs = micros();
  float dt = static_cast<uint32_t>(nowUs - lastUpdateUs_) * 1.0e-6f;
  lastUpdateUs_ = nowUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.1f) dt = 0.1f;

  const float magNormSq = mx * mx + my * my + mz * mz;
  const bool magValid =
      isfinite(magNormSq) && magNormSq > 25.0f && magNormSq < 1000000.0f;
  fusion_.update(gx, gy, gz, ax, ay, az,
                 magValid ? mx : 0.0f,
                 magValid ? my : 0.0f,
                 magValid ? mz : 0.0f, dt);

  const float* q = fusion_.quaternion();
  for (uint8_t i = 0; i < 4; ++i) sample_.q[i] = q[i];
  attitude::normalizeQuaternion(sample_.q);
  attitude::quaternionToEuler(sample_.q, &sample_.yawDeg,
                              &sample_.pitchDeg, &sample_.rollDeg);
  sample_.accuracy = magValid ? 3 : 2;
  sample_.timestampMs = nowMs;
  sample_.valid = true;
  return true;
}

bool Icm20948Sensor::addressResponds(uint8_t address) {
  wire_.beginTransmission(address);
  return wire_.endTransmission() == 0;
}

#endif

