#pragma once

#include <stddef.h>
#include <stdint.h>

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin/out=false, xorout=0.
inline uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}
