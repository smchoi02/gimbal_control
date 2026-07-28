#pragma once

#include <stdint.h>

inline bool elapsedUs(uint32_t now, uint32_t previous, uint32_t period) {
  return static_cast<uint32_t>(now - previous) >= period;
}

inline bool isFresh(uint32_t nowMs, uint32_t sampleMs, uint32_t timeoutMs) {
  return static_cast<uint32_t>(nowMs - sampleMs) <= timeoutMs;
}
