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
// 송신 주기와 같은 값이면 LoRa air time과 지터만으로도 매 주기 경계를 넘어
// TRACK과 HOLD_LAST_DIRECTION이 번갈아 발생한다. trs_test.ino의 TX_DT_MS가
// 200 ms(5 Hz)이므로 그 3배로 둔다. 송신 주기를 바꾸면 여기도 함께 바꾼다.
constexpr uint32_t REMOTE_TIMEOUT_MS = 600;
// true: CRC와 RK packet structure are valid이면 sender GPS fix/AGL semantic
// validation failure도 원격 추적 입력으로 사용한다. 송신 GPS가 아직 fix=0을
// 보내는 통신 시험용 옵션이다.
// 링크 확인이 끝났으면 false로 둔다. true이면 fix=0 패킷의 위치까지 추적에
// 사용하므로, 송신 GPS가 끊긴 순간 짐벌이 엉뚱한 방향으로 크게 돌아간다.
constexpr bool REMOTE_FORCE_USE_RECEIVED_DATA = false;

// false keeps the USB port as it has always been: only the 1 Hz "# ..."
// status lines, readable by hand in a serial monitor. Ground-station tools
// turn the CSV stream on with L1 when they attach, so the default costs them
// nothing. Set true only if a run must be captured from the first row.
// The stream is about 2.5 kB/s at 10 Hz, well inside 115200 baud.
constexpr bool TELEMETRY_MIRROR_TO_USB = false;

// Shared I2C bus: OpenRB-150 D11=SDA, D12=SCL.
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t BNO085_ADDRESS = 0x4A;  // Change to 0x4B if ADR is high.
// true uses the Game Rotation Vector proven to work on this hardware. Its yaw
// is relative to the power-on heading. Set false only after the magnetic
// Rotation Vector has been verified for north-referenced field tracking.
constexpr bool BNO_USE_GAME_ROTATION_VECTOR = true;
constexpr uint8_t BMP581_ADDRESS = 0x47;  // BMP581: 0x46 primary, 0x47 secondary.
constexpr uint8_t MAX_M10S_ADDRESS = 0x42;

// EBYTE E22-900T22S. Serial3 is the OpenRB header marked RX/TX.
// The OpenRB Arduino core internally numbers those pins 13 (RX) and 14 (TX),
// but those numbers are not printed on the board.
// The module UART configuration must match on the transmitter and receiver.
constexpr uint32_t E22_UART_BAUD = 9600;
// E22-900T22S normal/transparent mode is M0=LOW, M1=LOW. These pins must
// never float. Set each value to an MCU GPIO when connected; otherwise wire
// both module pins to GND and leave the corresponding value at -1.
constexpr int8_t E22_M0_PIN = -1;
constexpr int8_t E22_M1_PIN = -1;
// AUX is optional. When connected, LOW means the module is busy or starting.
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
// 실물에서 보상 방향이 반대로 나오면 코드가 아니라 이 부호만 뒤집는다.
// 2026-08-06 벤치 확인: yaw는 -1이 맞고 pitch만 반대여서 -1 -> +1로 변경.
constexpr float YAW_SIGN = -1.0f;
constexpr float PITCH_SIGN = 1.0f;
constexpr int16_t DXL_GOAL_CURRENT = 700;
constexpr uint32_t DXL_PROFILE_VELOCITY = 60;
constexpr int8_t DXL_TEMP_LIMIT_C = 65;

// false keeps the fixed zero at DXL_CENTER_DEG, so the gimbal 0 degree is a
// property of the mechanism rather than of wherever the axes happened to sit
// at power-on. With +-150 travel centred on servo 180 deg the whole range
// stays inside one servo revolution and never crosses the 0/360 wrap, which
// is what makes a fixed zero safe. Mount each horn so the reference pose
// reads about 180 deg (the status line prints dxl_raw for this), then leave
// this false. Set true only as a fallback, and send K after aligning by hand.
constexpr bool AUTO_CALIBRATE_GIMBAL_ZERO_ON_BOOT = false;
// true clamps every command to the mechanical travel below. Keep it true now
// that the real limits are known: without it a command past a hard stop is
// held against the stop by the servo until the current limit trips.
constexpr bool ENFORCE_SOFT_GIMBAL_LIMITS = true;
// Measured mechanical travel. Yaw +-150 leaves a 60 deg dead zone directly
// behind the payload; a target crossing it forces a 300 deg sweep the long
// way round, so keep the expected target bearing forward of the payload.
constexpr float YAW_MIN_DEG = -150.0f;
constexpr float YAW_MAX_DEG = 150.0f;
constexpr float PITCH_MIN_DEG = -10.0f;
constexpr float PITCH_MAX_DEG = 90.0f;
constexpr float MAX_RATE_DEG_S = 120.0f;

// A manual M command is released this long after the last one, returning the
// gimbal to STOW. Bench work needs a longer leash than flight would.
constexpr uint32_t MANUAL_TIMEOUT_MS = 10000;

// Tracking geometry
// 두 GPS 오차(각 2~3 m, 무상관)의 차분이 기선 길이로 나뉘어 각도가 되므로,
// 기선이 짧으면 yaw가 사실상 무작위가 된다. 30 m에서도 ±40° 수준이라 이 값은
// 폭주를 막는 안전 하한일 뿐이고, 실제 지향 검증은 기선 100 m 이상에서 해야
// 한다. 송수신기를 같은 책상에 두고 하는 시험은 이 값에 걸려 추적되지 않는다.
constexpr float MIN_TARGET_RANGE_M = 30.0f;
constexpr float MAX_ABS_RELATIVE_ALT_M = 30000.0f;

}  // namespace cfg
