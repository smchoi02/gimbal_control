#include "tracker_app.h"

#include <math.h>

#include "../common/time_utils.h"
#include "../config/system_config.h"
#include "../math/target_geometry.h"

TrackerApp::TrackerApp(Stream& debug, Stream& lora)
    : debug_(debug), imu_(Wire), barometer_(Wire), gps_(Wire), e22_(lora),
      gimbal_(Serial1, cfg::DXL_DIR_PIN) {}

void TrackerApp::begin() {
  imu_.begin(cfg::BNO085_ADDRESS, 20, cfg::BNO_USE_GAME_ROTATION_VECTOR);
  const bool baroOk = barometer_.begin(cfg::BMP581_ADDRESS);
  if (baroOk) barometer_.startReferenceCalibration(cfg::BARO_REFERENCE_SAMPLES);
  gps_.begin(cfg::MAX_M10S_ADDRESS);
  e22_.begin(cfg::E22_M0_PIN, cfg::E22_M1_PIN, cfg::E22_AUX_PIN);

  if (!gimbal_.begin()) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
  }
  // Capture pressure mismatch automatically while both units are still near
  // each other. No serial command or operator mode is required.
  baroBiasCaptureActive_ = true;
  resetInitialAlignment(millis());

  const uint32_t nowUs = micros();
  lastControlUs_ = lastBaroUs_ = lastGpsPollUs_ = lastLogUs_ = nowUs;
  lastStatusMs_ = millis();
}

void TrackerApp::update() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  imu_.poll(nowMs);
  e22_.poll(nowMs);
  if (elapsedUs(nowUs, lastGpsPollUs_, cfg::GPS_POLL_PERIOD_US)) {
    lastGpsPollUs_ = nowUs;
    gps_.poll(nowMs);
  }
  if (elapsedUs(nowUs, lastBaroUs_, cfg::BARO_PERIOD_US)) {
    lastBaroUs_ = nowUs;
    barometer_.poll(nowMs);
  }
  if (elapsedUs(nowUs, lastControlUs_, cfg::CONTROL_PERIOD_US)) {
    const float dtSeconds =
        static_cast<uint32_t>(nowUs - lastControlUs_) * 1.0e-6f;
    lastControlUs_ = nowUs;
    controlTick(nowMs, dtSeconds);
  }
  if (elapsedUs(nowUs, lastLogUs_, cfg::LOG_PERIOD_US)) {
    lastLogUs_ = nowUs;
    logTick(nowMs);
  }
  logger_.service(nowMs);
  if (static_cast<uint32_t>(nowMs - lastStatusMs_) >= cfg::STATUS_PERIOD_MS) {
    lastStatusMs_ = nowMs;
    printStatus(nowMs);
  }
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
    if (!baroBiasReady_) {
      blocker_ = localBarometerFresh(nowMs)
          ? TrackingBlocker::BARO_BIAS_NOT_READY
          : TrackingBlocker::BARO_REFERENCE_NOT_READY;
      return;
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
    if (trackingInputsFresh(nowMs)) {
      float ned[3];
      RelativeTarget target;
      if (targetGeometry(ned, &target)) {
        mode_ = TrackMode::TRACKING;
        blocker_ = TrackingBlocker::NONE;
      }
    }
    return;
  }

  if (mode_ != TrackMode::TRACKING) return;
  if (!trackingInputsFresh(nowMs)) {
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

void TrackerApp::logTick(uint32_t nowMs) {
  gimbal_.pollFeedback();
  if (!gimbal_.state().healthy) {
    mode_ = TrackMode::FAULT;
    blocker_ = TrackingBlocker::GIMBAL_FAULT;
  }
  logger_.log(nowMs, mode_, blocker_, imu_.sample(), localBarometerInput(),
              localGpsInput(), remoteInput(), relative_, gimbal_.state(),
              initialBaroBiasM_, baroBiasReady_);
}

void TrackerApp::printStatus(uint32_t nowMs) {
  const RemoteTargetSample& remote = remoteInput();
  const GpsFix& localGps = localGpsInput();
  const BarometerSample& localBaro = localBarometerInput();
  const AttitudeSample& imu = imu_.sample();
  const GimbalState& gimbal = gimbal_.state();
  const remote_protocol::Parser& parser = e22_.parser();
  const uint32_t packetAge = remote.timestampMs == 0
      ? 0xFFFFFFFFUL : static_cast<uint32_t>(nowMs - remote.timestampMs);

  debug_.print(F("state/block=")); debug_.print(modeName(mode_));
  debug_.print('/'); debug_.print(blockerName(blocker_));
  debug_.print(F(" lora=ok")); debug_.print(parser.packetsOk);
  debug_.print(F(" bad")); debug_.print(parser.packetsBad);
  debug_.print(F(" lost")); debug_.print(parser.sequenceLost);
  debug_.print(F(" age")); debug_.print(packetAge);
  debug_.print(F(" remote_fix=")); debug_.print(remote.fixType);
  debug_.print(F(" local_fix=")); debug_.print(localGps.fixType);
  debug_.print(F(" ned=")); debug_.print(relative_.northM, 1); debug_.print('/');
  debug_.print(relative_.eastM, 1); debug_.print('/');
  debug_.print(relative_.valid ? -relative_.downM : NAN, 1);
  debug_.print(F(" baro=")); debug_.print(remote.aglM, 1); debug_.print('/');
  debug_.print(localBaro.aglM, 1); debug_.print('/');
  debug_.print(initialBaroBiasM_, 1);
  debug_.print(F(" imu_ypr=")); debug_.print(imu.yawDeg, 1); debug_.print('/');
  debug_.print(imu.pitchDeg, 1); debug_.print('/'); debug_.print(imu.rollDeg, 1);
  debug_.print(F(" target=")); debug_.print(relative_.yawDeg, 1); debug_.print('/');
  debug_.print(relative_.pitchDeg, 1);
  debug_.print(F(" cmd=")); debug_.print(gimbal.yawCommandDeg, 1); debug_.print('/');
  debug_.print(gimbal.pitchCommandDeg, 1);
  debug_.print(F(" pos=")); debug_.print(gimbal.yawPresentDeg, 1); debug_.print('/');
  debug_.println(gimbal.pitchPresentDeg, 1);
}

bool TrackerApp::trackingInputsFresh(uint32_t nowMs) const {
  return baroBiasReady_ && trackingReferenceReady_ &&
         alignmentInputsFresh(nowMs);
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
  if (!baroBiasReady_) return TrackingBlocker::BARO_BIAS_NOT_READY;
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
    blocker_ = TrackingBlocker::ALIGNMENT_UNSTABLE;
    return;
  }
  if (initialAlignmentSamples_ > 0) {
    float mean[3] = {initialTargetDirectionSum_[0], initialTargetDirectionSum_[1],
                     initialTargetDirectionSum_[2]};
    target_geometry::normalizeDirection(mean, mean);
    const float dot = mean[0] * targetNed[0] + mean[1] * targetNed[1] +
                      mean[2] * targetNed[2];
    if (dot < cosf(cfg::MAX_ALIGNMENT_DIRECTION_SPREAD_DEG *
                   attitude::DEG_TO_RAD_F)) {
      blocker_ = TrackingBlocker::ALIGNMENT_UNSTABLE;
      return;
    }
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
