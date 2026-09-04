#pragma once

#include <stdint.h>

namespace cfg {

// Scheduling
constexpr uint32_t CONTROL_PERIOD_US = 20000;   // 50 Hz
constexpr uint32_t BARO_PERIOD_US = 20000;      // 50 Hz
constexpr uint32_t GPS_POLL_PERIOD_US = 10000;  // NAV-PVT itself is 10 Hz
constexpr uint32_t LOG_PERIOD_US = 100000;      // 10 Hz
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr uint32_t INITIAL_ALIGNMENT_MS = 10000;
constexpr uint16_t INITIAL_ALIGNMENT_MIN_SAMPLES = 30;  // E22 frames
constexpr float MAX_ALIGNMENT_ANGULAR_RATE_DPS = 10.0f;
constexpr float MAX_ALIGNMENT_DIRECTION_SPREAD_DEG = 10.0f;
constexpr uint16_t BARO_REFERENCE_SAMPLES = 50;
constexpr uint16_t BARO_BIAS_MIN_SAMPLES = 30;  // E22 frames

// Freshness limits, measured from local millis().
constexpr uint32_t IMU_TIMEOUT_MS = 200;
constexpr uint32_t LOCAL_BARO_TIMEOUT_MS = 300;
constexpr uint32_t LOCAL_GPS_TIMEOUT_MS = 1500;
constexpr uint32_t REMOTE_TIMEOUT_MS = 1000;

// Shared I2C bus: OpenRB-150 D11=SDA, D12=SCL.
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t BNO085_ADDRESS = 0x4A;  // Use 0x4B only if ADR is high.
// Game Rotation Vector has a power-on-relative yaw. The explicit manual LOS
// alignment in TrackerApp is required when this is true.
constexpr bool BNO_USE_GAME_ROTATION_VECTOR = true;
constexpr uint8_t BMP581_ADDRESS = 0x47;  // BMP581 secondary address.
constexpr uint8_t MAX_M10S_ADDRESS = 0x42;

// EBYTE E22-900T22S on OpenRB Serial3. E22 transparent normal mode requires
// M0=M1=LOW. Keep -1 only when that pin is physically tied to GND.
constexpr uint32_t E22_UART_BAUD = 9600;
constexpr int8_t E22_M0_PIN = -1;
constexpr int8_t E22_M1_PIN = -1;
constexpr int8_t E22_AUX_PIN = -1;  // Optional; HIGH means ready.

// SD card on the hardware SPI bus.
constexpr uint8_t SD_CS_PIN = 4;
constexpr uint32_t SD_FLUSH_PERIOD_MS = 1000;

// DYNAMIXEL on Serial1 / OpenRB built-in TTL bus.
constexpr uint8_t YAW_DXL_ID = 1;
constexpr uint8_t PITCH_DXL_ID = 2;
// Keep false for one-axis bench work; true requires both motor pings.
constexpr bool REQUIRE_BOTH_DXL = false;
constexpr uint32_t DXL_BAUD = 57600;
constexpr int DXL_DIR_PIN = -1;
constexpr float DXL_CENTER_DEG = 180.0f;
// These signs are motor-side settings. Establish them by restrained bench test.
constexpr float YAW_SIGN = -1.0f;
constexpr float PITCH_SIGN = -1.0f;
// Software-only offsets from captured physical zero to the optical/LOS axis.
constexpr float GIMBAL_YAW_REFERENCE_OFFSET_DEG = 0.0f;
constexpr float GIMBAL_PITCH_REFERENCE_OFFSET_DEG = 0.0f;
constexpr int16_t DXL_GOAL_CURRENT = 700;
constexpr uint32_t DXL_PROFILE_VELOCITY = 60;
constexpr int8_t DXL_TEMP_LIMIT_C = 65;

// A soft zero is current encoder position stored only in RAM. It does not
// change DYNAMIXEL EEPROM, homing offset, or any other hardware setting.
constexpr bool AUTO_CAPTURE_GIMBAL_ZERO_ON_BOOT = true;
constexpr bool ENFORCE_SOFT_GIMBAL_LIMITS = false;
constexpr float YAW_MIN_DEG = -170.0f;
constexpr float YAW_MAX_DEG = 170.0f;
constexpr float PITCH_MIN_DEG = -10.0f;
constexpr float PITCH_MAX_DEG = 90.0f;
constexpr float MAX_RATE_DEG_S = 120.0f;
// A first-order target filter plus acceleration limit turns 10 Hz GPS/LoRa
// target steps into a continuous 50 Hz servo trajectory. Increase the time
// constant for a calmer bench motion; lower it only after verifying tracking.
constexpr float GIMBAL_TARGET_FILTER_TIME_CONSTANT_S = 0.35f;
constexpr float MAX_ACCEL_DEG_S2 = 360.0f;
constexpr float GIMBAL_COMMAND_DEADBAND_DEG = 0.25f;

// BNO085 body-vector -> gimbal body-vector mapping. target_geometry first
// obtains a vector in BNO085 body axes, then applies this signed permutation
// before yaw/pitch atan2. Defaults assume +X forward, +Y right, +Z down.
// Change only after a static axis test; indices must be a permutation of 0,1,2.
constexpr uint8_t IMU_TO_GIMBAL_X_AXIS = 0;
constexpr uint8_t IMU_TO_GIMBAL_Y_AXIS = 1;
constexpr uint8_t IMU_TO_GIMBAL_Z_AXIS = 2;
constexpr float IMU_TO_GIMBAL_X_SIGN = 1.0f;
constexpr float IMU_TO_GIMBAL_Y_SIGN = 1.0f;
constexpr float IMU_TO_GIMBAL_Z_SIGN = 1.0f;

// NED tracking geometry, all lengths in metres.
constexpr float MIN_TARGET_RANGE_M = 1.0f;
constexpr float MIN_TRACK_HORIZONTAL_RANGE_M = 15.0f;
constexpr float MIN_ALIGNMENT_HORIZONTAL_RANGE_M = 30.0f;
constexpr float MAX_BARO_BIAS_HORIZONTAL_RANGE_M = 30.0f;
constexpr float MAX_ABS_RELATIVE_ALT_M = 30000.0f;
constexpr float MAX_ABS_VELOCITY_MPS = 5000.0f;

}  // namespace cfg
