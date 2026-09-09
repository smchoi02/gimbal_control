#pragma once

#include <stdint.h>

namespace cfg {

// Scheduling
constexpr uint32_t CONTROL_PERIOD_US = 20000;   // 50 Hz
constexpr uint32_t GPS_POLL_PERIOD_US = 10000;  // NAV-PVT itself is 10 Hz
// Non-control work is dispatched in one bounded round-robin slot per control
// tick. Never add a blocking transaction to the 50 Hz command path.
// RTOS validation uses the real mission log load: one CSV row at 10 Hz and a
// filesystem sync each second.  Do not compare results with SD disabled.
constexpr uint32_t SD_IDLE_LOG_PERIOD_MS = 100;
constexpr uint32_t RTOS_BAROMETER_PERIOD_MS = 67;  // approximately 15 Hz
// A normal request-response read is about 5--6 ms at 57,600 baud. Count
// only genuine stalls, not nominal wire time.
constexpr uint32_t DXL_FEEDBACK_SLOW_US = 10000;
// SD writes and verbose USB serial output have unbounded blocking time on an
// Arduino-class MCU. These are enabled for the current logging experiment.
constexpr bool ALLOW_SD_WRITES_WHILE_TORQUE_ON = true;
constexpr bool ALLOW_STATUS_OUTPUT_WHILE_TORQUE_ON = true;
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr uint32_t INITIAL_ALIGNMENT_MS = 10000;
constexpr uint16_t INITIAL_ALIGNMENT_MIN_SAMPLES = 30;  // E22 frames
constexpr float MAX_ALIGNMENT_ANGULAR_RATE_DPS = 10.0f;
constexpr uint16_t BARO_REFERENCE_SAMPLES = 50;
constexpr uint16_t BARO_BIAS_MIN_SAMPLES = 30;  // E22 frames

// Freshness limits, measured from local millis().
constexpr uint32_t IMU_TIMEOUT_MS = 200;
constexpr uint32_t LOCAL_BARO_TIMEOUT_MS = 300;
constexpr uint32_t LOCAL_GPS_TIMEOUT_MS = 1500;
constexpr uint32_t REMOTE_TIMEOUT_MS = 1000;
// A GPS that briefly resumes must remain healthy for this long before its
// changing position is allowed back into TRACK. During this interval the
// gimbal continuously holds the prior line of sight using the IMU.
constexpr uint32_t TRACK_REACQUIRE_STABLE_MS = 500;
// Recover only a silent MAX-M10S output stream. This is not used for a real
// no-fix condition: incoming NAV-PVT frames, including fixType=0, reset the
// silence timer.
constexpr uint32_t GPS_NAV_PVT_SILENCE_RECOVERY_MS = 3000;
constexpr uint32_t GPS_RECOVERY_RETRY_MS = 5000;

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
// Diagnostic only: the legacy Arduino SD driver completes a multi-step SPI
// transaction inside File::write()/flush(). Temporarily defer RTOS task
// switches only during those already-open-file calls to test whether a switch
// mid-transaction causes the card-busy timeout. Interrupts remain enabled,
// but ControlTask can be delayed by the SD call duration. This is not the
// final flight architecture.
constexpr bool RTOS_SERIALIZE_OPEN_SD_CALLS_TEST = false;
// Wait briefly for UTC date from MAX-M10S before choosing a log filename.
// If it is still unavailable, 0000 is used as the date portion.
constexpr uint32_t SD_DATE_WAIT_MS = 15000;

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
// SD fault isolation only. With true the controller still pings/configures and
// reads the Dynamixels, but leaves torque off so motor current/EMI cannot
// disturb the SD card. The gimbal is intentionally unable to move in this
// test. Restore false before any motion/tracking test.
constexpr bool RTOS_DXL_TORQUE_OFF_SD_ISOLATION_TEST = false;
// Second SD isolation stage: skip the 50 Hz Dynamixel status-request UART
// traffic as well as motor torque. Control math still runs at 50 Hz, but this
// is intentionally not a motion test. Restore false before flight testing.
constexpr bool RTOS_DISABLE_DXL_FEEDBACK_SD_ISOLATION_TEST = false;
// Third SD isolation stage: do not transact on the shared I2C sensor bus.
// Logger/control tasks and FreeRTOS remain active. This makes the test a
// direct comparison with the standalone SD test while retaining RTOS.
constexpr bool RTOS_DISABLE_SENSOR_IO_SD_ISOLATION_TEST = false;
// Fourth SD isolation stage: retain BMP581/MAX-M10S polling but skip the
// high-frequency BNO085 event transaction. Gimbal motion is intentionally
// unavailable because there is no fresh attitude sample.
constexpr bool RTOS_DISABLE_BNO_POLL_SD_ISOLATION_TEST = false;
// Fifth SD isolation stage: BMP581 remains active, but skip the MAX-M10S
// 32-byte I2C chunk/parser path. The log opens immediately with a P0000 date.
constexpr bool RTOS_DISABLE_GPS_POLL_SD_ISOLATION_TEST = false;
// Final I2C isolation stage: skip BMP581 runtime reads as well. With BNO/GPS
// tests already disabled, SensorTask still handles E22/status but performs no
// repeated Wire transaction.
constexpr bool RTOS_DISABLE_BAROMETER_POLL_SD_ISOLATION_TEST = false;
// Give the fully isolated RTOS logger test an unmistakable filename. This
// avoids confusing a copied historical P000000x.CSV with the current run.
constexpr bool RTOS_USE_ISOLATION_LOG_PREFIX = false;
constexpr char RTOS_ISOLATION_LOG_PREFIX = 'T';
// Bench-tuned for faster IMU stabilization. Verify temperature/current before
// raising these further on the flight mechanism.
constexpr uint32_t DXL_PROFILE_VELOCITY = 100;
constexpr int8_t DXL_TEMP_LIMIT_C = 65;

// A soft zero is current encoder position stored only in RAM. It does not
// change DYNAMIXEL EEPROM, homing offset, or any other hardware setting.
constexpr bool AUTO_CAPTURE_GIMBAL_ZERO_ON_BOOT = true;
constexpr bool ENFORCE_SOFT_GIMBAL_LIMITS = false;
constexpr float YAW_MIN_DEG = -170.0f;
constexpr float YAW_MAX_DEG = 170.0f;
constexpr float PITCH_MIN_DEG = -10.0f;
constexpr float PITCH_MAX_DEG = 90.0f;
constexpr float MAX_RATE_DEG_S = 150.0f;
// A first-order target filter plus acceleration limit turns 10 Hz GPS/LoRa
// target steps into a continuous 50 Hz servo trajectory. Increase the time
// constant for a calmer bench motion; lower it only after verifying tracking.
constexpr float GIMBAL_TARGET_FILTER_TIME_CONSTANT_S = 0.08f;
constexpr float MAX_ACCEL_DEG_S2 = 1200.0f;
constexpr float GIMBAL_COMMAND_DEADBAND_DEG = 0.08f;

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
constexpr float MIN_TRACK_HORIZONTAL_RANGE_M = 5.0f;
constexpr float MIN_ALIGNMENT_HORIZONTAL_RANGE_M = 5.0f;
constexpr float MAX_BARO_BIAS_HORIZONTAL_RANGE_M = 30.0f;
constexpr float MAX_ABS_RELATIVE_ALT_M = 30000.0f;
constexpr float MAX_ABS_VELOCITY_MPS = 5000.0f;

}  // namespace cfg
