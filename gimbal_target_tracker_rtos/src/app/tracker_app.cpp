#include "tracker_app.h"

#include <math.h>
#include <stdio.h>

#include <FreeRTOS_SAMD21.h>

#include "../common/time_utils.h"
#include "../config/system_config.h"
#include "../math/target_geometry.h"

TrackerApp::TrackerApp(Stream& debug, Stream& lora)
    : debug_(debug), imu_(Wire), barometer_(Wire), gps_(Wire), e22_(lora),
      gimbal_(Serial1, cfg::DXL_DIR_PIN) {}

void TrackerApp::begin() {
  debug_.println(F("boot: imu"));
  const bool imuOk =
      imu_.begin(cfg::BNO085_ADDRESS, 20, cfg::BNO_USE_GAME_ROTATION_VECTOR);
  debug_.print(F("boot: imu=")); debug_.println(imuOk ? F("ok") : F("fail"));

  debug_.println(F("boot: baro"));
  const bool baroOk = barometer_.begin(cfg::BMP581_ADDRESS);
  if (baroOk) barometer_.startReferenceCalibration(cfg::BARO_REFERENCE_SAMPLES);
  debug_.print(F("boot: baro=")); debug_.println(baroOk ? F("ok") : F("fail"));

  debug_.println(F("boot: gps"));
  const bool gpsOk = gps_.begin(cfg::MAX_M10S_ADDRESS);
  debug_.print(F("boot: gps=")); debug_.println(gpsOk ? F("ok") : F("fail"));

  debug_.println(F("boot: e22"));
  e22_.begin(cfg::E22_M0_PIN, cfg::E22_M1_PIN, cfg::E22_AUX_PIN);
  debug_.println(F("boot: sd"));
  const bool sdOk = logger_.begin(cfg::SD_CS_PIN);
  debug_.print(F("boot: sd=")); debug_.println(sdOk ? F("ok") : F("unavailable"));

  debug_.println(F("boot: gimbal"));
  const bool gimbalOk = gimbal_.begin();
  debug_.print(F("boot: gimbal=")); debug_.println(gimbalOk ? F("ok") : F("fail"));
  if (!gimbalOk) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
  }
  // Capture pressure mismatch automatically while both units are still near
  // each other. No serial command or operator mode is required.
  baroBiasCaptureActive_ = true;
  resetInitialAlignment(millis());

  const uint32_t nowUs = micros();
  lastControlUs_ = lastGpsPollUs_ = nowUs;
  lastStatusMs_ = millis();
  debug_.println(F("boot: ready"));
}

void TrackerApp::update() {
  imu_.poll(millis());
  e22_.poll(millis());
  serviceStatusTx();

  // The IMU and command path always win. All potentially blocking I/O is
  // deliberately dispatched only after this deadline has been serviced.
  const uint32_t controlNowUs = micros();
  const uint32_t controlNowMs = millis();
  bool controlRan = false;
  if (elapsedUs(controlNowUs, lastControlUs_, cfg::CONTROL_PERIOD_US)) {
    const uint32_t controlIntervalUs =
        static_cast<uint32_t>(controlNowUs - lastControlUs_);
    const float dtSeconds = controlIntervalUs * 1.0e-6f;
    lastControlUs_ = controlNowUs;
    gimbal_.recordControlTiming(controlIntervalUs);
    controlTick(controlNowMs, dtSeconds);
    backgroundTick(millis());
    controlRan = true;
  }

  // GPS parsing is also bounded to one 32-byte I2C chunk per call. Defer it
  // from a command loop so a control tick never starts two background jobs.
  const uint32_t gpsNowUs = micros();
  if (!controlRan &&
      elapsedUs(gpsNowUs, lastGpsPollUs_, cfg::GPS_POLL_PERIOD_US)) {
    lastGpsPollUs_ = gpsNowUs;
    gps_.poll(millis());
  }
  const uint32_t statusNowMs = millis();
  if ((cfg::ALLOW_STATUS_OUTPUT_WHILE_TORQUE_ON || !gimbal_.state().torqueOn) &&
      static_cast<uint32_t>(statusNowMs - lastStatusMs_) >= cfg::STATUS_PERIOD_MS) {
    lastStatusMs_ = statusNowMs;
    queueStatus(statusNowMs);
  }
}

void TrackerApp::rtosSensorTick(uint32_t nowMs) {
  // This is the sole owner of the shared BNO085/BMP581/MAX-M10S I2C bus.
  // Keep each transaction in this lower-priority task; control only reads
  // the latest completed samples.
  if (!cfg::RTOS_DISABLE_BNO_POLL_SD_ISOLATION_TEST) {
    imu_.poll(nowMs);
  }
  e22_.poll(nowMs);
  serviceStatusTx();

  const uint32_t nowUs = micros();
  if (!cfg::RTOS_DISABLE_GPS_POLL_SD_ISOLATION_TEST &&
      elapsedUs(nowUs, lastGpsPollUs_, cfg::GPS_POLL_PERIOD_US)) {
    lastGpsPollUs_ = nowUs;
    gps_.poll(nowMs);  // one bounded 32-byte chunk
  }
  if (!cfg::RTOS_DISABLE_BAROMETER_POLL_SD_ISOLATION_TEST &&
      static_cast<uint32_t>(nowMs - lastBarometerPollMs_) >=
      cfg::RTOS_BAROMETER_PERIOD_MS) {
    lastBarometerPollMs_ = nowMs;
    barometer_.poll(nowMs);
  }

  if ((cfg::ALLOW_STATUS_OUTPUT_WHILE_TORQUE_ON || !gimbal_.state().torqueOn) &&
      static_cast<uint32_t>(nowMs - lastStatusMs_) >= cfg::STATUS_PERIOD_MS) {
    lastStatusMs_ = nowMs;
    queueStatus(nowMs);
  }
}

void TrackerApp::rtosControlTick(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  const uint32_t intervalUs = static_cast<uint32_t>(nowUs - lastControlUs_);
  lastControlUs_ = nowUs;
  gimbal_.recordControlTiming(intervalUs);
  controlTick(nowMs, intervalUs * 1.0e-6f);

  // Command is sent before the optional status read. Therefore a delayed
  // Dynamixel status response can affect telemetry, but not this tick's goal.
  // Do not couple this to SD activity: a stuck SD write must not make motor
  // feedback appear stale or change the tracker behaviour.
  if (!cfg::RTOS_DISABLE_DXL_FEEDBACK_SD_ISOLATION_TEST) {
    rtosFeedbackTick();
  }
}

void TrackerApp::rtosLoggerTick(uint32_t nowMs) {
  // SD is intentionally owned by the lowest-priority task. It may block in
  // the card driver; a healthy RTOS tick must still release ControlTask.
  sdLogTick(nowMs);
  // With the sensor task intentionally disabled for SD isolation, retain the
  // normal non-blocking status transport from this task so bench results are
  // still observable over USB.
  if (cfg::RTOS_DISABLE_SENSOR_IO_SD_ISOLATION_TEST) {
    if (static_cast<uint32_t>(nowMs - lastStatusMs_) >= cfg::STATUS_PERIOD_MS) {
      lastStatusMs_ = nowMs;
      queueStatus(nowMs);
    }
    serviceStatusTx();
  }
}

void TrackerApp::setRtosStackFreeWords(uint16_t controlWords,
                                       uint16_t sensorWords,
                                       uint16_t loggerWords) {
  controlStackFreeWords_ = controlWords;
  sensorStackFreeWords_ = sensorWords;
  loggerStackFreeWords_ = loggerWords;
}

void TrackerApp::controlTick(uint32_t nowMs, float dtSeconds) {
  if (!gimbal_.state().healthy) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
    return;
  }
  if (baroBiasCaptureActive_) captureBaroBiasSample(nowMs);

  if (mode_ == TrackMode::BOOT) {
    mode_ = TrackMode::WAIT_FOR_DATA;
    return;
  }
  if (mode_ == TrackMode::FAULT) return;

  if (mode_ == TrackMode::WAIT_FOR_DATA) {
    // IMU stabilization is independent of GPS, barometer, and LoRa. As soon
    // as the BNO085 produces a valid attitude, capture the current payload
    // pose and gimbal encoder position, then counter subsequent body tilt.
    if (initializeImuHold(nowMs) && attitudeFresh(nowMs)) {
      commandHeldDirection(dtSeconds);
    }
    if (!alignmentInputsFresh(nowMs)) {
      blocker_ = inputBlocker(nowMs);
      return;
    }
    // All inputs are ready: start automatic initial alignment. The gimbal
    // must already be mechanically aimed at the intended reference direction.
    resetInitialAlignment(nowMs);
    mode_ = TrackMode::ALIGNING;
    blocker_ = TrackingBlocker::ALIGNMENT_IN_PROGRESS;
    return;
  }

  if (mode_ == TrackMode::ALIGNING) {
    if (imuHoldReferenceReady_ && attitudeFresh(nowMs)) {
      commandHeldDirection(dtSeconds);
    }
    collectInitialAlignmentSample(nowMs);
    finishInitialAlignment(nowMs);
    return;
  }

  if (mode_ == TrackMode::SAFE_HOLD) {
    // Continue inertial compensation of the last valid line-of-sight while
    // waiting for data to return; no stow/zero movement is issued.
    if (trackingReferenceReady_ && attitudeFresh(nowMs)) {
      commandHeldDirection(dtSeconds);
    }
    if (!trackingInputsFresh(nowMs)) {
      // Do not keep displaying the reason that originally entered HOLD.
      // One sensor may have recovered while another is now stale.
      blocker_ = inputBlocker(nowMs);
      trackingRecoveryStartMs_ = 0;
      return;
    }

    float ned[3];
    RelativeTarget target;
    if (!targetGeometry(ned, &target)) {
      blocker_ = target_geometry::horizontalRangeM(ned) <
                         cfg::MIN_TRACK_HORIZONTAL_RANGE_M
                     ? TrackingBlocker::RANGE_TOO_SMALL
                     : TrackingBlocker::GEOMETRY_INVALID;
      trackingRecoveryStartMs_ = 0;
      return;
    }
    if (trackingRecoveryStartMs_ == 0) {
      trackingRecoveryStartMs_ = nowMs;
      return;
    }
    if (static_cast<uint32_t>(nowMs - trackingRecoveryStartMs_) <
        cfg::TRACK_REACQUIRE_STABLE_MS) {
      return;
    }
    mode_ = TrackMode::TRACKING;
    blocker_ = TrackingBlocker::NONE;
    trackingRecoveryStartMs_ = 0;
    return;
  }

  if (mode_ != TrackMode::TRACKING) return;
  if (!trackingInputsFresh(nowMs)) {
    trackingRecoveryStartMs_ = 0;
    enterSafeHold(inputBlocker(nowMs), false);
    return;
  }

  float targetNed[3];
  RelativeTarget calculated;
  if (!targetGeometry(targetNed, &calculated)) {
    const float horizontal = target_geometry::horizontalRangeM(targetNed);
    enterSafeHold(horizontal < cfg::MIN_TRACK_HORIZONTAL_RANGE_M
                      ? TrackingBlocker::RANGE_TOO_SMALL
                      : TrackingBlocker::GEOMETRY_INVALID,
                  false);
    return;
  }
  relative_ = calculated;
  target_geometry::normalizeDirection(targetNed, lastTargetNed_);
  haveLastTargetNed_ = true;
  gimbal_.command(relative_.yawDeg, relative_.pitchDeg, dtSeconds);
  blocker_ = TrackingBlocker::NONE;
}

bool TrackerApp::targetGeometry(float ned[3], RelativeTarget* calculated) const {
  ned[0] = ned[1] = ned[2] = 0.0f;
  if (!target_geometry::relativeNed(localGpsInput(), localBarometerInput(),
                                    remoteInput(), initialBaroBiasM_, ned) ||
      target_geometry::horizontalRangeM(ned) <
          cfg::MIN_TRACK_HORIZONTAL_RANGE_M) {
    return false;
  }
  float relativeAttitudeQ[4];
  attitude::relativeToReference(localImuReferenceQ_, imu_.sample().q,
                                relativeAttitudeQ);
  if (!target_geometry::pointingAngles(ned, relativeAttitudeQ, calculated)) {
    return false;
  }
  calculated->yawDeg -= initialTargetYawDeg_;
  calculated->pitchDeg -= initialTargetPitchDeg_;
  calculated->valid = isfinite(calculated->yawDeg) &&
                      isfinite(calculated->pitchDeg);
  return calculated->valid;
}

bool TrackerApp::initializeImuHold(uint32_t nowMs) {
  if (imuHoldReferenceReady_) return true;
  if (!attitudeFresh(nowMs)) return false;

  for (uint8_t i = 0; i < 4; ++i) localImuReferenceQ_[i] = imu_.sample().q[i];
  attitude::normalizeQuaternion(localImuReferenceQ_);
  initialTargetYawDeg_ = 0.0f;
  initialTargetPitchDeg_ = 0.0f;
  lastTargetNed_[0] = 1.0f;
  lastTargetNed_[1] = 0.0f;
  lastTargetNed_[2] = 0.0f;
  haveLastTargetNed_ = true;
  if (!gimbal_.captureSoftZero()) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
    return false;
  }
  imuHoldReferenceReady_ = true;
  return true;
}

bool TrackerApp::commandHeldDirection(float dtSeconds) {
  if (!haveLastTargetNed_) return false;
  float relativeAttitudeQ[4];
  attitude::relativeToReference(localImuReferenceQ_, imu_.sample().q,
                                relativeAttitudeQ);
  RelativeTarget held;
  if (!target_geometry::pointingAngles(lastTargetNed_, relativeAttitudeQ,
                                       &held)) return false;
  held.yawDeg -= initialTargetYawDeg_;
  held.pitchDeg -= initialTargetPitchDeg_;
  if (!isfinite(held.yawDeg) || !isfinite(held.pitchDeg)) return false;
  relative_.yawDeg = held.yawDeg;
  relative_.pitchDeg = held.pitchDeg;
  relative_.valid = true;
  gimbal_.command(held.yawDeg, held.pitchDeg, dtSeconds);
  return true;
}

void TrackerApp::enterSafeHold(TrackingBlocker blocker, bool) {
  mode_ = TrackMode::SAFE_HOLD;
  blocker_ = blocker;
}

void TrackerApp::backgroundTick(uint32_t nowMs) {
  // One slot per 20 ms command tick. This yields 15 Hz barometer updates,
  // 10 Hz position feedback, 5 Hz current feedback, and 2.5 Hz temperature
  // feedback without ever issuing the former six back-to-back UART reads.
  switch (backgroundSlot_) {
    case 0:
    case 3:
    case 6:
      barometer_.poll(nowMs);
      break;
    case 1:
    case 4:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::YAW_POSITION);
      break;
    case 2:
    case 7:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::PITCH_POSITION);
      break;
    case 5:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::YAW_CURRENT);
      break;
    case 8:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::PITCH_CURRENT);
      break;
    default:
      if (logger_.cardReady() &&
          (cfg::ALLOW_SD_WRITES_WHILE_TORQUE_ON ||
           !gimbal_.state().torqueOn)) {
        sdLogTick(nowMs);
      } else {
        gimbal_.pollFeedbackItem(nextTemperatureIsYaw_
            ? GimbalController::FeedbackItem::YAW_TEMPERATURE
            : GimbalController::FeedbackItem::PITCH_TEMPERATURE);
        nextTemperatureIsYaw_ = !nextTemperatureIsYaw_;
      }
      break;
  }
  backgroundSlot_ = (backgroundSlot_ + 1) % 10;
  if (!gimbal_.state().healthy) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
  }
}

void TrackerApp::sdLogTick(uint32_t nowMs) {
  if (gimbal_.state().torqueOn && !cfg::ALLOW_SD_WRITES_WHILE_TORQUE_ON) return;
  if (static_cast<uint32_t>(nowMs - lastSdLogMs_) < cfg::SD_IDLE_LOG_PERIOD_MS)
    return;
  lastSdLogMs_ = nowMs;
  const GpsFix& localGps = localGpsInput();
  const uint32_t dateWaitMs =
      (cfg::RTOS_DISABLE_SENSOR_IO_SD_ISOLATION_TEST ||
       cfg::RTOS_DISABLE_GPS_POLL_SD_ISOLATION_TEST)
          ? 0U : cfg::SD_DATE_WAIT_MS;
  if (!logger_.ready() && logger_.cardReady() &&
      (localGps.dateValid || nowMs >= dateWaitMs)) {
    logger_.openForDate(localGps.dateValid ? localGps.month : 0,
                        localGps.dateValid ? localGps.day : 0);
  } else if (logger_.ready() && logger_.usingFallbackDate() &&
             localGps.dateValid) {
    logger_.reopenWithGpsDate(localGps.month, localGps.day);
  }
  // This is a causality test, not a final real-time design. The standalone SD
  // tests pass, whereas the same File::write() path fails only after RTOS task
  // switches. Keep one opened-file SD transaction contiguous, then compare
  // both SD errors and ctl_us against the previous run.
  if (cfg::RTOS_SERIALIZE_OPEN_SD_CALLS_TEST && logger_.ready()) {
    vTaskSuspendAll();
    logger_.log(nowMs, mode_, blocker_, imu_.sample(), localBarometerInput(),
                localGps, remoteInput(), relative_, gimbal_.state(),
                initialBaroBiasM_, baroBiasReady_);
    logger_.service(nowMs);
    xTaskResumeAll();
  } else {
    logger_.log(nowMs, mode_, blocker_, imu_.sample(), localBarometerInput(),
                localGps, remoteInput(), relative_, gimbal_.state(),
                initialBaroBiasM_, baroBiasReady_);
    logger_.service(nowMs);
  }
}

void TrackerApp::rtosFeedbackTick() {
  // Exactly one request/response transaction per control period.  DXL UART
  // ownership stays with ControlTask, preventing command/read interleaving.
  switch (backgroundSlot_) {
    case 0:
    case 3:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::YAW_POSITION);
      break;
    case 1:
    case 4:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::PITCH_POSITION);
      break;
    case 5:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::YAW_CURRENT);
      break;
    case 6:
      gimbal_.pollFeedbackItem(GimbalController::FeedbackItem::PITCH_CURRENT);
      break;
    default:
      gimbal_.pollFeedbackItem(nextTemperatureIsYaw_
          ? GimbalController::FeedbackItem::YAW_TEMPERATURE
          : GimbalController::FeedbackItem::PITCH_TEMPERATURE);
      nextTemperatureIsYaw_ = !nextTemperatureIsYaw_;
      break;
  }
  backgroundSlot_ = (backgroundSlot_ + 1) % 10;
  if (!gimbal_.state().healthy) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
  }
}

void TrackerApp::queueStatus(uint32_t nowMs) {
  // Never replace a partially transmitted line. Dropping an old telemetry
  // sample is preferable to making the 50 Hz command loop wait for USB.
  if (statusTxOffset_ < statusTxLength_) return;

  const RemoteTargetSample& remote = remoteInput();
  const GpsFix& localGps = localGpsInput();
  const BarometerSample& localBaro = localBarometerInput();
  const AttitudeSample& imu = imu_.sample();
  const GimbalState& gimbal = gimbal_.state();
  const remote_protocol::Parser& parser = e22_.parser();
  const uint32_t packetAge = remote.timestampMs == 0
      ? 0xFFFFFFFFUL : static_cast<uint32_t>(nowMs - remote.timestampMs);
  const uint32_t imuAge = imu.timestampMs == 0
      ? 0xFFFFFFFFUL : static_cast<uint32_t>(nowMs - imu.timestampMs);
  const uint32_t localGpsAge = localGps.timestampMs == 0
      ? 0xFFFFFFFFUL : static_cast<uint32_t>(nowMs - localGps.timestampMs);
  const uint32_t localBaroAge = localBaro.timestampMs == 0
      ? 0xFFFFFFFFUL : static_cast<uint32_t>(nowMs - localBaro.timestampMs);

  // newlib-nano on this target may omit printf floating-point support. Keep
  // this diagnostic line integer-only; *_d10 values are degrees/metres x10.
  const auto d10 = [](float value) -> long {
    return isfinite(value) ? static_cast<long>(lroundf(value * 10.0f)) : 0L;
  };
  const int written = snprintf(
      statusTxBuffer_, sizeof(statusTxBuffer_),
      "state/block=%s/%s lora=ok%lu bad%lu lost%lu age%lu sens_age=%lu/%lu/%lu "
      "gps_recov=%lu remote_fix=%u local_fix=%u imu_d10=%ld/%ld/%ld "
      "target_d10=%ld/%ld cmd_d10=%ld/%ld pos_d10=%ld/%ld ctl_us=%lu/%lu/%lu "
      "fb_us=%lu/%lu/%lu sd_file=%s sd_fault=%u sd_n=o%lu/%lu s%lu x%lu h%lu q%lu l%lu/%lu/%lu f%lu/%lu "
      "sd_w=b%lu p%lu e%lu row=%lu/%lu size=%lu wu=%lu/%lu bu=%lu/%lu stk_w=%u/%u/%u "
      "sd_us=s%lu o%lu h%lu q%lu l%lu/%lu f%lu/%lu\r\n",
      modeName(mode_), blockerName(blocker_),
      static_cast<unsigned long>(parser.packetsOk),
      static_cast<unsigned long>(parser.packetsBad),
      static_cast<unsigned long>(parser.sequenceLost),
      static_cast<unsigned long>(packetAge), static_cast<unsigned long>(imuAge),
      static_cast<unsigned long>(localGpsAge),
      static_cast<unsigned long>(localBaroAge),
      static_cast<unsigned long>(gps_.recoveryCount()), remote.fixType,
      localGps.fixType, d10(imu.yawDeg), d10(imu.pitchDeg), d10(imu.rollDeg),
      d10(relative_.yawDeg), d10(relative_.pitchDeg),
      d10(gimbal.yawCommandDeg), d10(gimbal.pitchCommandDeg),
      d10(gimbal.yawPresentDeg), d10(gimbal.pitchPresentDeg),
      static_cast<unsigned long>(gimbal.lastControlIntervalUs),
      static_cast<unsigned long>(gimbal.maxControlIntervalUs),
      static_cast<unsigned long>(gimbal.controlOverrunCount),
      static_cast<unsigned long>(gimbal.lastFeedbackUs),
      static_cast<unsigned long>(gimbal.maxFeedbackUs),
      static_cast<unsigned long>(gimbal.slowFeedbackCount),
      logger_.filename(), logger_.faulted() ? 1U : 0U,
      static_cast<unsigned long>(logger_.openAttempts()),
      static_cast<unsigned long>(logger_.openErrors()),
      static_cast<unsigned long>(logger_.filenameSearchErrors()),
      static_cast<unsigned long>(logger_.fileOpenErrors()),
      static_cast<unsigned long>(logger_.headerWriteErrors()),
      static_cast<unsigned long>(logger_.headerFlushErrors()),
      static_cast<unsigned long>(logger_.logAttempts()),
      static_cast<unsigned long>(logger_.rowsWritten()),
      static_cast<unsigned long>(logger_.writeErrors()),
      static_cast<unsigned long>(logger_.flushAttempts()),
      static_cast<unsigned long>(logger_.flushErrors()),
      static_cast<unsigned long>(logger_.rowBuildErrors()),
      static_cast<unsigned long>(logger_.shortWrites()),
      static_cast<unsigned long>(logger_.writeFlagErrors()),
      static_cast<unsigned long>(logger_.lastRowBytes()),
      static_cast<unsigned long>(logger_.lastWrittenBytes()),
      static_cast<unsigned long>(logger_.lastFileSize()),
      static_cast<unsigned long>(logger_.lastFileWriteUs()),
      static_cast<unsigned long>(logger_.maxFileWriteUs()),
      static_cast<unsigned long>(logger_.lastRowBuildUs()),
      static_cast<unsigned long>(logger_.maxRowBuildUs()),
      controlStackFreeWords_, sensorStackFreeWords_, loggerStackFreeWords_,
      static_cast<unsigned long>(logger_.lastFilenameSearchUs()),
      static_cast<unsigned long>(logger_.lastFileOpenUs()),
      static_cast<unsigned long>(logger_.lastHeaderWriteUs()),
      static_cast<unsigned long>(logger_.lastHeaderFlushUs()),
      static_cast<unsigned long>(logger_.lastLogUs()),
      static_cast<unsigned long>(logger_.maxLogUs()),
      static_cast<unsigned long>(logger_.lastFlushUs()),
      static_cast<unsigned long>(logger_.maxFlushUs()));
  if (written <= 0 || static_cast<size_t>(written) >= sizeof(statusTxBuffer_)) {
    statusTxLength_ = statusTxOffset_ = 0;
    return;
  }
  statusTxLength_ = static_cast<size_t>(written);
  statusTxOffset_ = 0;
}

void TrackerApp::serviceStatusTx() {
  if (statusTxOffset_ >= statusTxLength_) return;
  const int available = debug_.availableForWrite();
  if (available <= 0) return;
  const size_t remaining = statusTxLength_ - statusTxOffset_;
  const size_t count = remaining < static_cast<size_t>(available)
      ? remaining : static_cast<size_t>(available);
  const size_t sent = debug_.write(
      reinterpret_cast<const uint8_t*>(statusTxBuffer_ + statusTxOffset_), count);
  statusTxOffset_ += sent;
}

bool TrackerApp::trackingInputsFresh(uint32_t nowMs) const {
  // Barometer bias is an optional preflight refinement. It must never keep a
  // valid GPS/IMU/LoRa system in WAIT forever just because the two units were
  // not within the close-range capture window.
  return trackingReferenceReady_ && alignmentInputsFresh(nowMs);
}

bool TrackerApp::alignmentInputsFresh(uint32_t nowMs) const {
  return attitudeFresh(nowMs) && localGpsInput().valid &&
         isFresh(nowMs, localGpsInput().timestampMs,
                 cfg::LOCAL_GPS_TIMEOUT_MS) &&
         remoteInput().valid &&
         isFresh(nowMs, remoteInput().timestampMs, cfg::REMOTE_TIMEOUT_MS) &&
         localBarometerFresh(nowMs);
}

bool TrackerApp::attitudeFresh(uint32_t nowMs) const {
  return imu_.sample().valid &&
         isFresh(nowMs, imu_.sample().timestampMs, cfg::IMU_TIMEOUT_MS);
}

bool TrackerApp::localBarometerFresh(uint32_t nowMs) const {
  const BarometerSample& baro = localBarometerInput();
  return baro.valid && baro.referenceValid &&
         isFresh(nowMs, baro.timestampMs, cfg::LOCAL_BARO_TIMEOUT_MS);
}

TrackingBlocker TrackerApp::inputBlocker(uint32_t nowMs) const {
  if (!attitudeFresh(nowMs)) return TrackingBlocker::IMU_INVALID;
  if (!localGpsInput().valid ||
      !isFresh(nowMs, localGpsInput().timestampMs, cfg::LOCAL_GPS_TIMEOUT_MS))
    return TrackingBlocker::LOCAL_GPS_INVALID;
  if (!localBarometerFresh(nowMs)) return TrackingBlocker::LOCAL_BARO_INVALID;
  if (!isFresh(nowMs, remoteInput().timestampMs, cfg::REMOTE_TIMEOUT_MS))
    return TrackingBlocker::REMOTE_PACKET_STALE;
  if (!remoteInput().valid) return TrackingBlocker::REMOTE_GPS_INVALID;
  return TrackingBlocker::GEOMETRY_INVALID;
}

bool TrackerApp::captureBaroBiasSample(uint32_t nowMs) {
  if (!baroBiasCaptureActive_ || !localBarometerFresh(nowMs) ||
      !localGpsInput().valid ||
      !isFresh(nowMs, localGpsInput().timestampMs, cfg::LOCAL_GPS_TIMEOUT_MS) ||
      !remoteInput().valid ||
      !isFresh(nowMs, remoteInput().timestampMs, cfg::REMOTE_TIMEOUT_MS)) {
    return false;
  }
  const uint8_t sequence = remoteInput().sequence;
  if (haveBaroBiasSequence_ && sequence == lastBaroBiasSequence_) return false;
  float temporaryNed[3];
  if (!target_geometry::relativeNed(localGpsInput(), localBarometerInput(),
                                    remoteInput(), 0.0f, temporaryNed) ||
      target_geometry::horizontalRangeM(temporaryNed) >
          cfg::MAX_BARO_BIAS_HORIZONTAL_RANGE_M) return false;
  haveBaroBiasSequence_ = true;
  lastBaroBiasSequence_ = sequence;
  ++baroBiasSamples_;
  const float sample = remoteInput().aglM - localBarometerInput().aglM;
  baroBiasSumM_ += (sample - baroBiasSumM_) /
                   static_cast<float>(baroBiasSamples_);
  initialBaroBiasM_ = baroBiasSumM_;
  if (baroBiasSamples_ >= cfg::BARO_BIAS_MIN_SAMPLES) {
    baroBiasCaptureActive_ = false;
    baroBiasReady_ = true;
  }
  return true;
}

void TrackerApp::resetInitialAlignment(uint32_t nowMs) {
  initialAlignmentStartMs_ = nowMs;
  initialAlignmentSamples_ = 0;
  trackingReferenceReady_ = false;
  haveAlignmentSequence_ = false;
  initialImuSum_[0] = initialImuSum_[1] = initialImuSum_[2] = initialImuSum_[3] = 0.0f;
  initialTargetDirectionSum_[0] = initialTargetDirectionSum_[1] =
      initialTargetDirectionSum_[2] = 0.0f;
}

void TrackerApp::collectInitialAlignmentSample(uint32_t nowMs) {
  if (!alignmentInputsFresh(nowMs)) { blocker_ = inputBlocker(nowMs); return; }
  const uint8_t sequence = remoteInput().sequence;
  if (haveAlignmentSequence_ && sequence == lastAlignmentSequence_) return;
  float targetNed[3];
  if (!target_geometry::relativeNed(localGpsInput(), localBarometerInput(),
                                    remoteInput(), initialBaroBiasM_, targetNed) ||
      target_geometry::horizontalRangeM(targetNed) <
          cfg::MIN_ALIGNMENT_HORIZONTAL_RANGE_M) {
    blocker_ = TrackingBlocker::RANGE_TOO_SMALL;
    return;
  }
  target_geometry::normalizeDirection(targetNed, targetNed);
  const AttitudeSample& imu = imu_.sample();
  const float maxRate = fmaxf(fabsf(imu.angularRateDps[0]),
                               fmaxf(fabsf(imu.angularRateDps[1]),
                                     fabsf(imu.angularRateDps[2])));
  if (maxRate > cfg::MAX_ALIGNMENT_ANGULAR_RATE_DPS) {
    // A moving payload invalidates the current alignment window. Discard its
    // old average so that, once it is still again, a clean 10-second window
    // starts automatically instead of remaining permanently UNSTABLE.
    resetInitialAlignment(nowMs);
    blocker_ = TrackingBlocker::ALIGNMENT_UNSTABLE;
    return;
  }
  float sign = 1.0f;
  if (initialAlignmentSamples_ > 0 &&
      initialImuSum_[0] * imu.q[0] + initialImuSum_[1] * imu.q[1] +
      initialImuSum_[2] * imu.q[2] + initialImuSum_[3] * imu.q[3] < 0.0f)
    sign = -1.0f;
  for (uint8_t i = 0; i < 4; ++i) initialImuSum_[i] += sign * imu.q[i];
  for (uint8_t i = 0; i < 3; ++i) initialTargetDirectionSum_[i] += targetNed[i];
  haveAlignmentSequence_ = true;
  lastAlignmentSequence_ = sequence;
  ++initialAlignmentSamples_;
  blocker_ = TrackingBlocker::ALIGNMENT_IN_PROGRESS;
}

bool TrackerApp::finishInitialAlignment(uint32_t nowMs) {
  if (initialAlignmentSamples_ < cfg::INITIAL_ALIGNMENT_MIN_SAMPLES ||
      static_cast<uint32_t>(nowMs - initialAlignmentStartMs_) <
          cfg::INITIAL_ALIGNMENT_MS) return false;
  for (uint8_t i = 0; i < 4; ++i) localImuReferenceQ_[i] = initialImuSum_[i];
  attitude::normalizeQuaternion(localImuReferenceQ_);
  target_geometry::normalizeDirection(initialTargetDirectionSum_,
                                      initialTargetDirectionSum_);
  const float identityQ[4] = {1, 0, 0, 0};
  RelativeTarget initialTarget;
  if (!target_geometry::pointingAngles(initialTargetDirectionSum_, identityQ,
                                       &initialTarget) ||
      !gimbal_.captureSoftZero()) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
    return false;
  }
  initialTargetYawDeg_ = initialTarget.yawDeg;
  initialTargetPitchDeg_ = initialTarget.pitchDeg;
  trackingReferenceReady_ = true;
  mode_ = TrackMode::TRACKING;
  blocker_ = TrackingBlocker::NONE;
  return true;
}

const char* TrackerApp::modeName(TrackMode mode) {
  switch (mode) {
    case TrackMode::BOOT: return "BOOT";
    case TrackMode::WAIT_FOR_DATA: return "WAIT";
    case TrackMode::ALIGNING: return "ALIGN";
    case TrackMode::TRACKING: return "TRACK";
    case TrackMode::SAFE_HOLD: return "HOLD";
    case TrackMode::FAULT: return "FAULT";
  }
  return "?";
}

const char* TrackerApp::blockerName(TrackingBlocker blocker) {
  switch (blocker) {
    case TrackingBlocker::NONE: return "NONE";
    case TrackingBlocker::GIMBAL_FAULT: return "GIMBAL";
    case TrackingBlocker::ALIGNMENT_IN_PROGRESS: return "ALIGN";
    case TrackingBlocker::ALIGNMENT_UNSTABLE: return "UNSTABLE";
    case TrackingBlocker::BARO_REFERENCE_NOT_READY: return "BARO_REF";
    case TrackingBlocker::BARO_BIAS_NOT_READY: return "BARO_BIAS";
    case TrackingBlocker::REMOTE_PACKET_STALE: return "REMOTE_STALE";
    case TrackingBlocker::REMOTE_GPS_INVALID: return "REMOTE_GPS";
    case TrackingBlocker::LOCAL_GPS_INVALID: return "LOCAL_GPS";
    case TrackingBlocker::LOCAL_BARO_INVALID: return "LOCAL_BARO";
    case TrackingBlocker::IMU_INVALID: return "IMU";
    case TrackingBlocker::RANGE_TOO_SMALL: return "RANGE";
    case TrackingBlocker::GEOMETRY_INVALID: return "GEOMETRY";
  }
  return "?";
}

const GpsFix& TrackerApp::localGpsInput() const { return gps_.fix(); }
const BarometerSample& TrackerApp::localBarometerInput() const {
  return barometer_.sample();
}
const RemoteTargetSample& TrackerApp::remoteInput() const { return e22_.sample(); }
