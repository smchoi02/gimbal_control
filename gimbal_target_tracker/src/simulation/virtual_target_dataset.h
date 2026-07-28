#pragma once

#include <stddef.h>
#include <stdint.h>

struct VirtualTargetKeyframe {
  uint32_t timeMs;
  float northM;
  float eastM;
  float upM;
};

// A 40-second closed route around the receiver. The first and last points
// match, so playback can loop without a position discontinuity.
constexpr VirtualTargetKeyframe VIRTUAL_TARGET_DATASET[] = {
    {0, 150.0f, 0.0f, 0.0f},
    {5000, 106.0f, 106.0f, 10.0f},
    {10000, 0.0f, 150.0f, 20.0f},
    {15000, -106.0f, 106.0f, 10.0f},
    {20000, -150.0f, 0.0f, 0.0f},
    {25000, -106.0f, -106.0f, -10.0f},
    {30000, 0.0f, -150.0f, -20.0f},
    {35000, 106.0f, -106.0f, -10.0f},
    {40000, 150.0f, 0.0f, 0.0f},
};

constexpr size_t VIRTUAL_TARGET_KEYFRAME_COUNT =
    sizeof(VIRTUAL_TARGET_DATASET) / sizeof(VIRTUAL_TARGET_DATASET[0]);
constexpr uint32_t VIRTUAL_TARGET_LOOP_MS = 40000;

// Fixed local reference used by full bench mode (S2).
constexpr int32_t VIRTUAL_LOCAL_LAT_I7 = 375000000;   // 37.5000000 deg
constexpr int32_t VIRTUAL_LOCAL_LON_I7 = 1270000000;  // 127.0000000 deg
constexpr float VIRTUAL_LOCAL_PRESSURE_PA = 101325.0f;
constexpr float VIRTUAL_LOCAL_TEMPERATURE_C = 25.0f;
