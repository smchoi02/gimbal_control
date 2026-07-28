#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../common/types.h"

// Small BMP581 I2C driver using Bosch BMP5 register definitions.
// The BMP581 provides already compensated 24-bit pressure and temperature.
class Bmp581Sensor {
 public:
  explicit Bmp581Sensor(TwoWire& wire) : wire_(wire) {}

  bool begin(uint8_t address);
  bool poll(uint32_t nowMs);

  bool present() const { return present_; }
  uint8_t chipId() const { return chipId_; }
  const BarometerSample& sample() const { return sample_; }

 private:
  static constexpr uint8_t REG_CHIP_ID = 0x01;
  static constexpr uint8_t REG_TEMP_XLSB = 0x1D;
  static constexpr uint8_t REG_DSP_CONFIG = 0x30;
  static constexpr uint8_t REG_DSP_IIR = 0x31;
  static constexpr uint8_t REG_OSR_CONFIG = 0x36;
  static constexpr uint8_t REG_ODR_CONFIG = 0x37;
  static constexpr uint8_t REG_OSR_EFF = 0x38;
  static constexpr uint8_t REG_CMD = 0x7E;

  TwoWire& wire_;
  BarometerSample sample_;
  uint8_t address_ = 0;
  uint8_t chipId_ = 0;
  bool present_ = false;

  bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
  bool writeRegister(uint8_t reg, uint8_t value);
};
