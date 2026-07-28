#include "bmp581_sensor.h"

bool Bmp581Sensor::begin(uint8_t address) {
  address_ = address;
  if (!readRegisters(REG_CHIP_ID, &chipId_, 1)) return false;
  if (chipId_ != 0x50 && chipId_ != 0x51) return false;

  if (!writeRegister(REG_CMD, 0xB6)) return false;  // Soft reset.
  delay(3);
  if (!readRegisters(REG_CHIP_ID, &chipId_, 1)) return false;

  // Standby + deep-standby disabled + ODR 50 Hz.
  if (!writeRegister(REG_ODR_CONFIG, 0xBC)) return false;
  delay(3);

  // temp OSR=2x, pressure OSR=16x, pressure enabled.
  if (!writeRegister(REG_OSR_CONFIG, 0x61)) return false;
  // Keep pressure/temperature compensation enabled (bits 1:0 = 0b11)
  // while selecting the filtered temperature and pressure shadow data.
  if (!writeRegister(REG_DSP_CONFIG, 0x2B)) return false;
  if (!writeRegister(REG_DSP_IIR, 0x1A)) return false;

  // Continuous mode (mode bits=3), preserving 50 Hz and deep-disable.
  if (!writeRegister(REG_ODR_CONFIG, 0xBF)) return false;
  // Pressure OSR 16x plus temperature OSR 2x needs about 12 ms before
  // the first complete compensated sample is available.
  delay(15);

  uint8_t effective = 0;
  if (!readRegisters(REG_OSR_EFF, &effective, 1)) return false;
  present_ = (effective & 0x80u) != 0;  // ODR setting accepted.
  return present_;
}

bool Bmp581Sensor::poll(uint32_t nowMs) {
  if (!present_) return false;
  uint8_t bytes[6];
  if (!readRegisters(REG_TEMP_XLSB, bytes, sizeof(bytes))) return false;

  uint32_t rawTemperature = static_cast<uint32_t>(bytes[0]) |
                            (static_cast<uint32_t>(bytes[1]) << 8) |
                            (static_cast<uint32_t>(bytes[2]) << 16);
  int32_t signedTemperature =
      (rawTemperature & 0x800000u)
          ? static_cast<int32_t>(rawTemperature | 0xFF000000u)
          : static_cast<int32_t>(rawTemperature);
  const uint32_t rawPressure = static_cast<uint32_t>(bytes[3]) |
                               (static_cast<uint32_t>(bytes[4]) << 8) |
                               (static_cast<uint32_t>(bytes[5]) << 16);

  sample_.temperatureC = static_cast<float>(signedTemperature) / 65536.0f;
  sample_.pressurePa = static_cast<float>(rawPressure) / 64.0f;
  sample_.timestampMs = nowMs;
  sample_.valid = isfinite(sample_.pressurePa) &&
                  sample_.pressurePa >= 30000.0f &&
                  sample_.pressurePa <= 125000.0f;
  return sample_.valid;
}

bool Bmp581Sensor::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;
  const size_t received = wire_.requestFrom(static_cast<int>(address_),
                                            static_cast<int>(length));
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = wire_.read();
  return true;
}

bool Bmp581Sensor::writeRegister(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}
