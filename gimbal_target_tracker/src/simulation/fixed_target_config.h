#pragma once

#include <stdint.h>

// S3 fixed-target simulation settings.
//
// Replace these three target values with the absolute position that the
// receiver should point toward. Latitude/longitude use degrees * 1e7 and
// altitude is meters above mean sea level.
namespace fixed_target {

constexpr int32_t LAT_I7 = 375000000;    // 37.5000000 deg
constexpr int32_t LON_I7 = 1270000000;   // 127.0000000 deg
constexpr float ALTITUDE_M = 100.0f;

// Local BMP581 pressure is converted to barometric altitude with this QNH.
// Set this to the local sea-level pressure for a more accurate pitch result.
constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
constexpr float TEMPERATURE_C = 25.0f;

}  // namespace fixed_target
