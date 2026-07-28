#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

#include "../common/types.h"

class Bno085Sensor {
 public:
  explicit Bno085Sensor(TwoWire& wire) : wire_(wire) {}

  bool begin(uint8_t address, uint16_t reportIntervalMs = 20,
             bool useGameRotationVector = true);
  bool poll(uint32_t nowMs);

  bool present() const { return present_; }
  bool reportEnabled() const { return reportEnabled_; }
  bool usingGameRotationVector() const { return useGameRotationVector_; }
  char sourceCode() const { return useGameRotationVector_ ? 'G' : 'R'; }
  const AttitudeSample& sample() const { return sample_; }
  uint32_t resetCount() const { return resetCount_; }
  uint32_t reportRecoveryCount() const { return reportRecoveryCount_; }

 private:
  void restoreRotationVectorReport(uint32_t nowMs);

  TwoWire& wire_;
  BNO08x device_;
  AttitudeSample sample_;
  bool present_ = false;
  bool reportEnabled_ = false;
  bool useGameRotationVector_ = true;
  uint16_t reportIntervalMs_ = 20;
  uint32_t lastRecoveryAttemptMs_ = 0;
  uint32_t resetCount_ = 0;
  uint32_t reportRecoveryCount_ = 0;
};
