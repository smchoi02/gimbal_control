#include "sd_logger.h"

#include "../config/system_config.h"

bool SdLogger::begin(uint8_t chipSelectPin) {
  if (!SD.begin(chipSelectPin)) return false;
  ready_ = createUniqueFile();
  if (ready_) {
    writeHeader();
    file_.flush();
  }
  return ready_;
}

bool SdLogger::createUniqueFile() {
  for (uint16_t index = 0; index < 1000; ++index) {
    snprintf(filename_, sizeof(filename_), "TRK%03u.CSV", index);
    if (!SD.exists(filename_)) {
      file_ = SD.open(filename_, FILE_WRITE);
      return static_cast<bool>(file_);
    }
  }
  return false;
}

void SdLogger::writeHeader() { emitHeader(file_); }

void SdLogger::emitHeader(Print& out) {
  out.println(
      F("t_ms,mode,imu_valid,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg,"
        "local_baro_valid,local_pressure_pa,local_temp_c,"
        "local_gps_valid,local_itow_ms,local_lat_i7,local_lon_i7,"
        "local_hmsl_mm,local_fix_type,local_num_sv,"
        "remote_valid,remote_seq,remote_sender_ms,remote_lat_i7,remote_lon_i7,"
        "remote_agl_m,remote_vel_n_mps,remote_vel_e_mps,remote_vel_d_mps,remote_fix_type,"
        "rel_n_m,rel_e_m,rel_d_m,range_m,target_yaw_deg,target_pitch_deg,"
        "gimbal_yaw_cmd,gimbal_pitch_cmd,gimbal_yaw_pos,gimbal_pitch_pos,"
        "gimbal_cur_y,gimbal_cur_p,gimbal_temp_y,gimbal_temp_p,"
        "gimbal_limit,gimbal_torque"));
}

void SdLogger::log(uint32_t nowMs, TrackMode mode,
                   const AttitudeSample& attitude,
                   const BarometerSample& localBaro, const GpsFix& localGps,
                   const RemoteTargetSample& remote,
                   const RelativeTarget& relative,
                   const GimbalState& gimbal) {
  if (!ready_ && !mirror_) return;

  // The header goes out once so a reader attaching mid-run can map columns.
  if (mirror_ && !mirrorHeaderSent_) {
    emitHeader(*mirror_);
    mirrorHeaderSent_ = true;
  }

#define CSV_VALUE(...)  \
  do {                  \
    emit(__VA_ARGS__);  \
    emitChar(',');      \
  } while (0)

  CSV_VALUE(nowMs);
  CSV_VALUE(static_cast<uint8_t>(mode));
  CSV_VALUE(attitude.valid ? 1 : 0);
  CSV_VALUE(attitude.q[0], 6);
  CSV_VALUE(attitude.q[1], 6);
  CSV_VALUE(attitude.q[2], 6);
  CSV_VALUE(attitude.q[3], 6);
  CSV_VALUE(attitude.yawDeg, 3);
  CSV_VALUE(attitude.pitchDeg, 3);
  CSV_VALUE(attitude.rollDeg, 3);
  CSV_VALUE(localBaro.valid ? 1 : 0);
  CSV_VALUE(localBaro.pressurePa, 2);
  CSV_VALUE(localBaro.temperatureC, 2);
  CSV_VALUE(localGps.valid ? 1 : 0);
  CSV_VALUE(localGps.iTowMs);
  CSV_VALUE(localGps.latI7);
  CSV_VALUE(localGps.lonI7);
  CSV_VALUE(localGps.hMslMm);
  CSV_VALUE(localGps.fixType);
  CSV_VALUE(localGps.numSv);
  CSV_VALUE(remote.valid ? 1 : 0);
  CSV_VALUE(remote.sequence);
  CSV_VALUE(remote.senderTimeMs);
  CSV_VALUE(remote.latI7);
  CSV_VALUE(remote.lonI7);
  CSV_VALUE(remote.aglM, 3);
  CSV_VALUE(remote.velNMps, 3);
  CSV_VALUE(remote.velEMps, 3);
  CSV_VALUE(remote.velDMps, 3);
  CSV_VALUE(remote.fixType);
  CSV_VALUE(relative.northM, 3);
  CSV_VALUE(relative.eastM, 3);
  CSV_VALUE(relative.downM, 3);
  CSV_VALUE(relative.rangeM, 3);
  CSV_VALUE(relative.yawDeg, 3);
  CSV_VALUE(relative.pitchDeg, 3);
  CSV_VALUE(gimbal.yawCommandDeg, 3);
  CSV_VALUE(gimbal.pitchCommandDeg, 3);
  CSV_VALUE(gimbal.yawPresentDeg, 3);
  CSV_VALUE(gimbal.pitchPresentDeg, 3);
  CSV_VALUE(gimbal.yawCurrentRaw);
  CSV_VALUE(gimbal.pitchCurrentRaw);
  CSV_VALUE(gimbal.yawTemperatureC);
  CSV_VALUE(gimbal.pitchTemperatureC);
  CSV_VALUE(gimbal.limitActive ? 1 : 0);
  emit(gimbal.torqueOn ? 1 : 0);
  emitNewline();

#undef CSV_VALUE

  if (!ready_) return;
  if (file_.getWriteError()) {
    ++writeErrors_;
    file_.clearWriteError();
  } else {
    ++rowsWritten_;
  }
}

void SdLogger::service(uint32_t nowMs) {
  if (!ready_) return;
  if (static_cast<uint32_t>(nowMs - lastFlushMs_) >=
      cfg::SD_FLUSH_PERIOD_MS) {
    file_.flush();
    lastFlushMs_ = nowMs;
  }
}
