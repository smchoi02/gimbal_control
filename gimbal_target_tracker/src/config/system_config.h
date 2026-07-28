#pragma once

#include <stdint.h>

namespace cfg {

// Scheduling
constexpr uint32_t CONTROL_PERIOD_US = 20000;   // 50 Hz
constexpr uint32_t BARO_PERIOD_US = 20000;      // 50 Hz
constexpr uint32_t GPS_POLL_PERIOD_US = 10000;  // 100 Hz bus polling, NAV-PVT is 10 Hz
constexpr uint32_t LOG_PERIOD_US = 100000;      // 10 Hz
constexpr uint32_t STATUS_PERIOD_MS = 1000;

// Freshness limits
constexpr uint32_t IMU_TIMEOUT_MS = 200;
constexpr uint32_t LOCAL_BARO_TIMEOUT_MS = 300;
constexpr uint32_t LOCAL_GPS_TIMEOUT_MS = 1500;
constexpr uint32_t REMOTE_TIMEOUT_MS = 1000;

// Shared I2C bus: OpenRB-150 D11=SDA, D12=SCL.
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t BNO085_ADDRESS = 0x4A;  // Change to 0x4B if ADR is high.
// true uses the Game Rotation Vector proven to work on this hardware. Its yaw
// is relative to the power-on heading. Set false only after the magnetic
// Rotation Vector has been verified for north-referenced field tracking.
constexpr bool BNO_USE_GAME_ROTATION_VECTOR = true;
// ICM-20948 uses 0x68 when AD0 is low and 0x69 when AD0 is high. The driver
// tries this preferred value first and then automatically tries the other.
constexpr bool ICM20948_AD0_HIGH = false;
constexpr uint8_t BMP581_ADDRESS = 0x47;  // BMP581: 0x46 primary, 0x47 secondary.
constexpr uint8_t MAX_M10S_ADDRESS = 0x42;

// EBYTE E22-900T22S. Serial3 is OpenRB D13(RX), D14(TX).
constexpr uint32_t LORA_UART_BAUD = 9600;  // E22 factory UART default.
// Set these to real GPIO numbers if M0/M1/AUX are connected to the MCU.
// If -1, wire M0 and M1 to GND for Normal/transparent mode; AUX is optional.
constexpr int8_t E22_M0_PIN = -1;
constexpr int8_t E22_M1_PIN = -1;
constexpr int8_t E22_AUX_PIN = -1;

// SD card on the board hardware SPI bus. Confirm this CS pin against your wiring.
constexpr uint8_t SD_CS_PIN = 4;
constexpr uint32_t SD_FLUSH_PERIOD_MS = 1000;

// DYNAMIXEL on Serial1 / OpenRB built-in TTL bus.
constexpr uint8_t YAW_DXL_ID = 1;
constexpr uint8_t PITCH_DXL_ID = 2;
// false: one connected axis is enough for bench testing.
// true: both axes must answer ping or the system enters FAULT.
constexpr bool REQUIRE_BOTH_DXL = false;
constexpr uint32_t DXL_BAUD = 57600;
constexpr int DXL_DIR_PIN = -1;
constexpr float DXL_CENTER_DEG = 180.0f;
constexpr float YAW_SIGN = -1.0f;
constexpr float PITCH_SIGN = -1.0f;
constexpr int16_t DXL_GOAL_CURRENT = 700;
constexpr uint32_t DXL_PROFILE_VELOCITY = 60;
constexpr int8_t DXL_TEMP_LIMIT_C = 65;

// Mechanical and command limits
constexpr float YAW_MIN_DEG = -170.0f;
constexpr float YAW_MAX_DEG = 170.0f;
constexpr float PITCH_MIN_DEG = -10.0f;
constexpr float PITCH_MAX_DEG = 90.0f;
constexpr float MAX_RATE_DEG_S = 120.0f;

// Tracking geometry
constexpr float MIN_TARGET_RANGE_M = 1.0f;
constexpr float MAX_ABS_RELATIVE_ALT_M = 30000.0f;

}  // namespace cfg
