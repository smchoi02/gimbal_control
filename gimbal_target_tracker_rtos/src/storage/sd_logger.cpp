#include "sd_logger.h"

#include <math.h>

#include "../config/system_config.h"

namespace {

class CsvRowBuilder {
 public:
  CsvRowBuilder(char* buffer, size_t capacity)
      : buffer_(buffer), capacity_(capacity) {}

  bool ok() const { return ok_; }
  const char* data() const { return buffer_; }
  size_t size() const { return length_; }

  void comma() { appendChar(','); }
  void newline() { appendChar('\n'); }
  void unsignedValue(uint32_t value) {
    char text[11] = {};
    snprintf(text, sizeof(text), "%lu", static_cast<unsigned long>(value));
    appendText(text);
  }
  void signedValue(int32_t value) {
    char text[12] = {};
    snprintf(text, sizeof(text), "%ld", static_cast<long>(value));
    appendText(text);
  }
  void floatValue(float value, uint8_t decimals) {
    if (!isfinite(value)) {
      appendText("nan");
      return;
    }
    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) scale *= 10U;
    const bool negative = value < 0.0f;
    const float magnitude = negative ? -value : value;
    const uint32_t scaled = static_cast<uint32_t>(magnitude * scale + 0.5f);
    if (negative) appendChar('-');
    unsignedValue(scaled / scale);
    if (decimals == 0) return;
    appendChar('.');
    uint32_t divisor = scale / 10U;
    uint32_t fraction = scaled % scale;
    while (divisor > 0) {
      appendChar(static_cast<char>('0' + (fraction / divisor)));
      fraction %= divisor;
      divisor /= 10U;
    }
  }

 private:
  void appendChar(char value) {
    if (!ok_ || length_ + 1 >= capacity_) {
      ok_ = false;
      return;
    }
    buffer_[length_++] = value;
  }
  void appendText(const char* text) {
    while (*text != '\0') appendChar(*text++);
  }

  char* buffer_;
  size_t capacity_;
  size_t length_ = 0;
  bool ok_ = true;
};

}  // namespace

bool SdLogger::begin(uint8_t chipSelectPin) {
  cardReady_ = SD.begin(chipSelectPin);
  return cardReady_;
}

bool SdLogger::openForDate(uint8_t month, uint8_t day) {
  if (!cardReady_ || ready_) return ready_;
  const uint32_t startedUs = micros();
  ++openAttempts_;
  ready_ = createUniqueFile(month, day);
  if (ready_) {
    faulted_ = false;
    usingFallbackDate_ = month == 0 && day == 0;
    const uint32_t headerStartedUs = micros();
    writeHeader();
    lastHeaderWriteUs_ = static_cast<uint32_t>(micros() - headerStartedUs);
    if (lastHeaderWriteUs_ > maxHeaderWriteUs_)
      maxHeaderWriteUs_ = lastHeaderWriteUs_;
    const bool headerWriteError = file_.getWriteError();
    if (headerWriteError) {
      ++headerWriteErrors_;
      file_.clearWriteError();
    }

    const uint32_t headerFlushStartedUs = micros();
    file_.flush();
    lastHeaderFlushUs_ =
        static_cast<uint32_t>(micros() - headerFlushStartedUs);
    if (lastHeaderFlushUs_ > maxHeaderFlushUs_)
      maxHeaderFlushUs_ = lastHeaderFlushUs_;
    const bool headerFlushError = file_.getWriteError();
    if (headerFlushError) {
      ++headerFlushErrors_;
      file_.clearWriteError();
    }
    if (headerWriteError || headerFlushError) {
      ++openErrors_;
      file_.close();
      ready_ = false;
    }
  } else {
    ++openErrors_;
  }
  lastOpenUs_ = static_cast<uint32_t>(micros() - startedUs);
  if (lastOpenUs_ > maxOpenUs_) maxOpenUs_ = lastOpenUs_;
  return ready_;
}

bool SdLogger::reopenWithGpsDate(uint8_t month, uint8_t day) {
  if (!cardReady_ || !ready_ || !usingFallbackDate_ || month == 0 || day == 0)
    return ready_;
  // Keep early rows in P0000_NN.CSV, then continue in the dated file once the
  // GPS has supplied a valid UTC date.
  file_.flush();
  file_.close();
  ready_ = false;
  return openForDate(month, day);
}

bool SdLogger::createUniqueFile(uint8_t month, uint8_t day) {
  const uint32_t searchStartedUs = micros();
  const char filePrefix = cfg::RTOS_USE_ISOLATION_LOG_PREFIX
      ? cfg::RTOS_ISOLATION_LOG_PREFIX : 'P';
  for (uint16_t index = 0; index <= 999; ++index) {
    // The bundled SD library uses 8.3 names. PMMDDNNN.CSV is the readable
    // on-card equivalent of payload_srad_MMDD_NNN.csv.
    snprintf(filename_, sizeof(filename_), "%c%02u%02u%03u.CSV", filePrefix,
             month, day, index);
    if (!SD.exists(filename_)) {
      lastFilenameSearchUs_ =
          static_cast<uint32_t>(micros() - searchStartedUs);
      if (lastFilenameSearchUs_ > maxFilenameSearchUs_)
        maxFilenameSearchUs_ = lastFilenameSearchUs_;
      const uint32_t openStartedUs = micros();
      file_ = SD.open(filename_, FILE_WRITE);
      lastFileOpenUs_ = static_cast<uint32_t>(micros() - openStartedUs);
      if (lastFileOpenUs_ > maxFileOpenUs_)
        maxFileOpenUs_ = lastFileOpenUs_;
      if (!file_) {
        ++fileOpenErrors_;
        return false;
      }
      return true;
    }
  }
  lastFilenameSearchUs_ = static_cast<uint32_t>(micros() - searchStartedUs);
  if (lastFilenameSearchUs_ > maxFilenameSearchUs_)
    maxFilenameSearchUs_ = lastFilenameSearchUs_;
  ++filenameSearchErrors_;
  return false;
}

void SdLogger::writeHeader() {
  file_.println(
      F("t_ms,mode,imu_valid,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg,"
        "blocker,local_baro_valid,local_baro_reference_valid,local_pressure_pa,local_temp_c,local_agl_m,local_reference_pa,"
        "local_gps_valid,local_itow_ms,local_lat_i7,local_lon_i7,"
        "local_hmsl_mm,local_fix_type,local_num_sv,"
        "remote_valid,remote_seq,remote_itow_ms,remote_lat_i7,remote_lon_i7,"
        "remote_agl_m,remote_vel_n_mps,remote_vel_e_mps,remote_vel_d_mps,remote_fix_type,"
        "initial_baro_bias_m,baro_bias_ready,rel_n_m,rel_e_m,rel_d_m,rel_up_m,range_m,target_yaw_deg,target_pitch_deg,"
        "gimbal_yaw_cmd,gimbal_pitch_cmd,gimbal_yaw_cmd_unwrapped,gimbal_yaw_pos,gimbal_pitch_pos,gimbal_yaw_pos_unwrapped,"
        "gimbal_cur_y,gimbal_cur_p,gimbal_temp_y,gimbal_temp_p,gimbal_feedback_us,gimbal_feedback_max_us,gimbal_feedback_slow_count,control_interval_us,control_interval_max_us,control_overrun_count,"
        "gimbal_limit,gimbal_torque"));
}

void SdLogger::log(uint32_t nowMs, TrackMode mode, TrackingBlocker blocker,
                   const AttitudeSample& attitude,
                   const BarometerSample& localBaro, const GpsFix& localGps,
                   const RemoteTargetSample& remote,
                   const RelativeTarget& relative,
                   const GimbalState& gimbal, float initialBaroBiasM,
                   bool baroBiasReady) {
  if (!ready_ || faulted_) return;
  const uint32_t startedUs = micros();
  ++logAttempts_;

  const uint32_t rowBuildStartedUs = micros();
  CsvRowBuilder row(csvRowBuffer_, sizeof(csvRowBuffer_));
#define CSV_U(value) do { row.unsignedValue(static_cast<uint32_t>(value)); row.comma(); } while (0)
#define CSV_I(value) do { row.signedValue(static_cast<int32_t>(value)); row.comma(); } while (0)
#define CSV_F(value, digits) do { row.floatValue((value), (digits)); row.comma(); } while (0)

  CSV_U(nowMs); CSV_U(static_cast<uint8_t>(mode)); CSV_U(attitude.valid ? 1 : 0);
  CSV_F(attitude.q[0], 6); CSV_F(attitude.q[1], 6); CSV_F(attitude.q[2], 6); CSV_F(attitude.q[3], 6);
  CSV_F(attitude.yawDeg, 3); CSV_F(attitude.pitchDeg, 3); CSV_F(attitude.rollDeg, 3);
  CSV_U(static_cast<uint8_t>(blocker)); CSV_U(localBaro.valid ? 1 : 0); CSV_U(localBaro.referenceValid ? 1 : 0);
  CSV_F(localBaro.pressurePa, 2); CSV_F(localBaro.temperatureC, 2); CSV_F(localBaro.aglM, 3); CSV_F(localBaro.referencePressurePa, 2);
  CSV_U(localGps.valid ? 1 : 0); CSV_U(localGps.iTowMs); CSV_I(localGps.latI7); CSV_I(localGps.lonI7);
  CSV_I(localGps.hMslMm); CSV_U(localGps.fixType); CSV_U(localGps.numSv);
  CSV_U(remote.valid ? 1 : 0); CSV_U(remote.sequence); CSV_U(remote.senderITowMs); CSV_I(remote.latI7); CSV_I(remote.lonI7);
  CSV_F(remote.aglM, 3); CSV_F(remote.velNMps, 3); CSV_F(remote.velEMps, 3); CSV_F(remote.velDMps, 3); CSV_U(remote.fixType);
  CSV_F(initialBaroBiasM, 3); CSV_U(baroBiasReady ? 1 : 0);
  CSV_F(relative.northM, 3); CSV_F(relative.eastM, 3); CSV_F(relative.downM, 3); CSV_F(relative.valid ? -relative.downM : 0.0f, 3);
  CSV_F(relative.rangeM, 3); CSV_F(relative.yawDeg, 3); CSV_F(relative.pitchDeg, 3);
  CSV_F(gimbal.yawCommandDeg, 3); CSV_F(gimbal.pitchCommandDeg, 3); CSV_F(gimbal.yawCommandUnwrappedDeg, 3);
  CSV_F(gimbal.yawPresentDeg, 3); CSV_F(gimbal.pitchPresentDeg, 3); CSV_F(gimbal.yawPresentUnwrappedDeg, 3);
  CSV_I(gimbal.yawCurrentRaw); CSV_I(gimbal.pitchCurrentRaw); CSV_I(gimbal.yawTemperatureC); CSV_I(gimbal.pitchTemperatureC);
  CSV_U(gimbal.lastFeedbackUs); CSV_U(gimbal.maxFeedbackUs); CSV_U(gimbal.slowFeedbackCount);
  CSV_U(gimbal.lastControlIntervalUs); CSV_U(gimbal.maxControlIntervalUs); CSV_U(gimbal.controlOverrunCount);
  CSV_U(gimbal.limitActive ? 1 : 0); row.unsignedValue(gimbal.torqueOn ? 1 : 0); row.newline();

#undef CSV_U
#undef CSV_I
#undef CSV_F

  lastRowBuildUs_ = static_cast<uint32_t>(micros() - rowBuildStartedUs);
  if (lastRowBuildUs_ > maxRowBuildUs_) maxRowBuildUs_ = lastRowBuildUs_;
  lastRowBytes_ = static_cast<uint32_t>(row.size());

  // Capture the three independent failure modes.  In particular, do not
  // conflate an overflowing RAM row, a short SdFile write, and SdFile's
  // sticky write-error flag.
  const bool rowBuildError = !row.ok();
  size_t written = 0;
  bool writeFlagError = false;
  lastFileWriteUs_ = 0;
  if (!rowBuildError) {
    file_.clearWriteError();
    const uint32_t fileWriteStartedUs = micros();
    written = file_.write(reinterpret_cast<const uint8_t*>(row.data()), row.size());
    lastFileWriteUs_ = static_cast<uint32_t>(micros() - fileWriteStartedUs);
    if (lastFileWriteUs_ > maxFileWriteUs_)
      maxFileWriteUs_ = lastFileWriteUs_;
    writeFlagError = file_.getWriteError();
  }
  lastWrittenBytes_ = static_cast<uint32_t>(written);
  lastFileSize_ = static_cast<uint32_t>(file_.size());
  const bool shortWrite = !rowBuildError && written != row.size();
  if (rowBuildError) ++rowBuildErrors_;
  if (shortWrite) ++shortWrites_;
  if (writeFlagError) ++writeFlagErrors_;
  if (rowBuildError || shortWrite || writeFlagError) {
    ++writeErrors_;
    file_.clearWriteError();
    // The Arduino SD backend retries an unhealthy card with a long busy
    // timeout. One failed row is enough evidence: stop issuing SD commands
    // for this boot and keep control/sensor work responsive.
    faulted_ = true;
  } else {
    ++rowsWritten_;
  }
  lastLogUs_ = static_cast<uint32_t>(micros() - startedUs);
  if (lastLogUs_ > maxLogUs_) maxLogUs_ = lastLogUs_;
}

void SdLogger::service(uint32_t nowMs) {
  if (!ready_ || faulted_) return;
  if (static_cast<uint32_t>(nowMs - lastFlushMs_) >=
      cfg::SD_FLUSH_PERIOD_MS) {
    const uint32_t startedUs = micros();
    ++flushAttempts_;
    file_.flush();
    if (file_.getWriteError()) {
      ++flushErrors_;
      file_.clearWriteError();
      faulted_ = true;
    }
    lastFlushUs_ = static_cast<uint32_t>(micros() - startedUs);
    if (lastFlushUs_ > maxFlushUs_) maxFlushUs_ = lastFlushUs_;
    lastFlushMs_ = nowMs;
  }
}
