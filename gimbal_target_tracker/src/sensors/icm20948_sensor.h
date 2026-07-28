#pragma once

#include "../config/imu_selection.h"

#if TRACKER_USE_ICM20948

#include <Arduino.h>
#include <ICM_20948.h>
#include <Wire.h>

#include "../common/types.h"
#include "../math/mahony_ahrs.h"

class Icm20948Sensor {
 public:
  explicit Icm20948Sensor(TwoWire& wire) : wire_(wire) {}

  bool begin(bool preferredAd0High = false);
  bool poll(uint32_t nowMs);

  bool present() const { return present_; }
  bool reportEnabled() const { return present_; }
  char sourceCode() const { return 'I'; }
  uint32_t resetCount() const { return readErrors_; }
  uint32_t reportRecoveryCount() const { return 0; }
  uint8_t address() const { return address_; }
  const AttitudeSample& sample() const { return sample_; }

 private:
  TwoWire& wire_;
  ICM_20948_I2C device_;
  MahonyAhrs fusion_;
  AttitudeSample sample_;
  bool present_ = false;
  uint8_t address_ = 0;
  uint32_t lastUpdateUs_ = 0;
  uint32_t readErrors_ = 0;

  bool addressResponds(uint8_t address);
};

#endif

