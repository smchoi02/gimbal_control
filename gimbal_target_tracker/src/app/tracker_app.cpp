#include "tracker_app.h"

#include <math.h>

#include "../common/time_utils.h"
#include "../config/system_config.h"
#include "../math/target_geometry.h"

TrackerApp::TrackerApp(Stream& debug, Stream& lora)
    : debug_(debug),
      imu_(Wire),
      barometer_(Wire),
      gps_(Wire),
      e22_(lora),
      gimbal_(Serial1, cfg::DXL_DIR_PIN) {}

void TrackerApp::begin() {
  imu_.begin(cfg::BNO085_ADDRESS, 20, cfg::BNO_USE_GAME_ROTATION_VECTOR);
  barometer_.begin(cfg::BMP581_ADDRESS);
  gps_.begin(cfg::MAX_M10S_ADDRESS);

  e22_.begin(cfg::E22_M0_PIN, cfg::E22_M1_PIN, cfg::E22_AUX_PIN);
  const bool gimbalOk = gimbal_.begin();
  logger_.begin(cfg::SD_CS_PIN);

  if (!gimbalOk) mode_ = TrackMode::FAULT;
  resetInitialAlignment(millis());

  const uint32_t nowUs = micros();
  lastControlUs_ = lastBaroUs_ = lastGpsPollUs_ = lastLogUs_ = nowUs;
  lastStatusMs_ = millis();
}

void TrackerApp::update() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  pollCommands();
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
  if (static_cast<uint32_t>(nowMs - lastStatusMs_) >=
      cfg::STATUS_PERIOD_MS) {
    lastStatusMs_ = nowMs;
    printStatus(nowMs);
  }
}

void TrackerApp::controlTick(uint32_t nowMs, float dtSeconds) {
  if (!gimbal_.state().healthy) {
    mode_ = TrackMode::FAULT;
    return;
  }
  if (!trackingEnabled_) {
    mode_ = TrackMode::STOW;
    gimbal_.stow(dtSeconds);
    return;
  }

  // Do not send stow/track commands while the user holds the gimbal pointed
  // at the transmitter. This establishes the initial line-of-sight reference.
  if (!trackingReferenceReady_) {
    collectInitialAlignmentSample(nowMs);
    if (finishInitialAlignment(nowMs)) return;
    mode_ = TrackMode::STOW;
    relative_.valid = false;
    return;
  }

  if (trackingInputsFresh(nowMs)) {
    float targetNed[3];
    RelativeTarget calculated;
    float relativeAttitudeQ[4];
    attitude::relativeToReference(localImuReferenceQ_, imu_.sample().q,
                                  relativeAttitudeQ);
    if (target_geometry::relativeNed(localGpsInput(), remoteInput(),
                                     targetNed) &&
        target_geometry::pointingAngles(targetNed, relativeAttitudeQ,
                                        &calculated)) {
      // The user-aligned transmitter direction is the gimbal origin.
      calculated.yawDeg -= initialTargetYawDeg_;
      calculated.pitchDeg -= initialTargetPitchDeg_;
      relative_ = calculated;
      target_geometry::normalizeDirection(targetNed, lastDirectionNed_);
      haveLastDirection_ = true;
      mode_ = TrackMode::TRACK;
      gimbal_.command(relative_.yawDeg, relative_.pitchDeg, dtSeconds);
      return;
    }
  }

  // On GPS/barometer/LoRa loss, keep the last world direction using live IMU.
  if (haveLastDirection_ && attitudeFresh(nowMs)) {
    RelativeTarget held = relative_;
    float body[3];
    attitude::nedToBody(lastDirectionNed_, imu_.sample().q, body);
    attitude::bodyToGimbal(body, &held.yawDeg, &held.pitchDeg);
    held.valid = true;
    relative_ = held;
    mode_ = TrackMode::HOLD_LAST_DIRECTION;
    gimbal_.command(held.yawDeg, held.pitchDeg, dtSeconds);
    return;
  }

  mode_ = TrackMode::STOW;
  relative_.valid = false;
  gimbal_.stow(dtSeconds);
}

void TrackerApp::logTick(uint32_t nowMs) {
  gimbal_.pollFeedback();
  if (!gimbal_.state().healthy) mode_ = TrackMode::FAULT;
  logger_.log(nowMs, mode_, imu_.sample(), localBarometerInput(),
              localGpsInput(), remoteInput(), relative_, gimbal_.state());
}

void TrackerApp::printStatus(uint32_t nowMs) {
  const RemoteTargetSample& remote = remoteInput();
  const GpsFix& localGps = localGpsInput();
  const AttitudeSample& localImu = imu_.sample();
  const GimbalState& gimbal = gimbal_.state();
  const bool remoteFresh =
      isFresh(nowMs, remote.timestampMs, cfg::REMOTE_TIMEOUT_MS);
  const bool txGpsOk = remoteFresh && remote.satelliteCount >= 4 &&
                       remote.latI7 != 0 && remote.lonI7 != 0;
  // trs_test's 34-byte packet has no barometer-valid bit. A recent packet
  // with a finite AGL field is the available transmitter-barometer indication.
  const bool txBaroOk = remoteFresh && isfinite(remote.aglM);
  const bool rxGpsOk = localGps.valid &&
                       isFresh(nowMs, localGps.timestampMs,
                               cfg::LOCAL_GPS_TIMEOUT_MS);
  const bool rxImuOk = attitudeFresh(nowMs);
  const bool rxBaroOk = localBarometerInput().valid &&
                        isFresh(nowMs, localBarometerInput().timestampMs,
                                cfg::LOCAL_BARO_TIMEOUT_MS);
  const bool servoOk = gimbal.healthy && gimbal.torqueOn &&
                       gimbal.yawOnline && gimbal.pitchOnline;

  // trs_test's 34-byte RK packet carries satellite count at byte 3.
  debug_.print(F("TX[gps=")); debug_.print(txGpsOk ? F("OK") : F("NO"));
  debug_.print(F(" lat=")); debug_.print(static_cast<double>(remote.latI7) * 1.0e-7, 7);
  debug_.print(F(" lon=")); debug_.print(static_cast<double>(remote.lonI7) * 1.0e-7, 7);
  debug_.print(F(" sv=")); debug_.print(remote.satelliteCount);
  debug_.print(F(" agl_m=")); debug_.print(remote.aglM, 2);
  debug_.print(F(" baro=")); debug_.print(txBaroOk ? F("OK") : F("NO"));

  debug_.print(F("] RX[gps=")); debug_.print(rxGpsOk ? F("OK") : F("NO"));
  debug_.print(F(" lat=")); debug_.print(static_cast<double>(localGps.latI7) * 1.0e-7, 7);
  debug_.print(F(" lon=")); debug_.print(static_cast<double>(localGps.lonI7) * 1.0e-7, 7);
  debug_.print(F(" sv=")); debug_.print(localGps.numSv);
  debug_.print(F(" hmsl_m=")); debug_.print(static_cast<float>(localGps.hMslMm) * 0.001f, 2);
  debug_.print(F(" imu=")); debug_.print(rxImuOk ? F("OK") : F("NO"));
  debug_.print(F(" omega_xyz_dps="));
  debug_.print(localImu.angularRateDps[0], 1); debug_.print('/');
  debug_.print(localImu.angularRateDps[1], 1); debug_.print('/');
  debug_.print(localImu.angularRateDps[2], 1);
  debug_.print(F(" baro=")); debug_.print(rxBaroOk ? F("OK") : F("NO"));

  debug_.print(F("] SERVO=")); debug_.print(servoOk ? F("OK") : F("NO"));
  debug_.print(F("(yaw=")); debug_.print(gimbal.yawOnline ? 1 : 0);
  debug_.print(F(" pitch=")); debug_.print(gimbal.pitchOnline ? 1 : 0);
  debug_.print(F(" torque=")); debug_.print(gimbal.torqueOn ? 1 : 0);
  debug_.println(F(")"));
}

void TrackerApp::pollCommands() {
  while (debug_.available()) {
    const char c = debug_.read();
    if (c == '\r' || c == '\n') {
      if (commandLength_ > 0) {
        commandBuffer_[commandLength_] = '\0';
        handleCommand(commandBuffer_);
        commandLength_ = 0;
      }
    } else if (commandLength_ < sizeof(commandBuffer_) - 1) {
      commandBuffer_[commandLength_++] = c;
    }
  }
}

void TrackerApp::handleCommand(char* line) {
  switch (line[0]) {
    case 'C':
      trackingEnabled_ = true;
      break;
    case 'Z':
      trackingEnabled_ = false;
      break;
    case 'T':
      gimbal_.setTorque(line[1] == '1');
      break;
    case 'K':
      gimbal_.calibrateZero();
      break;
    case 'R':
      resetInitialAlignment(millis());
      haveLastDirection_ = false;
      relative_.valid = false;
      break;
    case 'P':
      gps_.configure10Hz();
      break;
    case '?':
      break;
  }
}

bool TrackerApp::trackingInputsFresh(uint32_t nowMs) const {
  return attitudeFresh(nowMs) && localGpsInput().valid &&
         isFresh(nowMs, localGpsInput().timestampMs,
                 cfg::LOCAL_GPS_TIMEOUT_MS) &&
         remoteInput().valid &&
         isFresh(nowMs, remoteInput().timestampMs, cfg::REMOTE_TIMEOUT_MS);
}

bool TrackerApp::attitudeFresh(uint32_t nowMs) const {
  return imu_.sample().valid &&
         isFresh(nowMs, imu_.sample().timestampMs, cfg::IMU_TIMEOUT_MS);
}

void TrackerApp::resetInitialAlignment(uint32_t nowMs) {
  initialAlignmentStartMs_ = nowMs;
  initialAlignmentSamples_ = 0;
  trackingReferenceReady_ = false;
  initialImuSum_[0] = initialImuSum_[1] = initialImuSum_[2] = initialImuSum_[3] = 0.0f;
  initialTargetDirectionSum_[0] = initialTargetDirectionSum_[1] =
      initialTargetDirectionSum_[2] = 0.0f;
}

void TrackerApp::collectInitialAlignmentSample(uint32_t nowMs) {
  if (!trackingInputsFresh(nowMs)) return;

  float targetNed[3];
  if (!target_geometry::relativeNed(localGpsInput(), remoteInput(), targetNed)) {
    return;
  }
  target_geometry::normalizeDirection(targetNed, targetNed);
  const AttitudeSample& imu = imu_.sample();
  float sign = 1.0f;
  if (initialAlignmentSamples_ > 0 &&
      initialImuSum_[0] * imu.q[0] + initialImuSum_[1] * imu.q[1] +
          initialImuSum_[2] * imu.q[2] + initialImuSum_[3] * imu.q[3] < 0.0f) {
    sign = -1.0f;
  }
  for (uint8_t i = 0; i < 4; ++i) initialImuSum_[i] += sign * imu.q[i];
  for (uint8_t i = 0; i < 3; ++i) initialTargetDirectionSum_[i] += targetNed[i];
  ++initialAlignmentSamples_;
}

bool TrackerApp::finishInitialAlignment(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - initialAlignmentStartMs_) <
      cfg::INITIAL_ALIGNMENT_MS || initialAlignmentSamples_ == 0) {
    return false;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    localImuReferenceQ_[i] = initialImuSum_[i];
  }
  attitude::normalizeQuaternion(localImuReferenceQ_);
  target_geometry::normalizeDirection(initialTargetDirectionSum_,
                                      initialTargetDirectionSum_);
  const float identityQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  RelativeTarget initialTarget;
  if (!target_geometry::pointingAngles(initialTargetDirectionSum_, identityQ,
                                       &initialTarget)) {
    return false;
  }
  initialTargetYawDeg_ = initialTarget.yawDeg;
  initialTargetPitchDeg_ = initialTarget.pitchDeg;
  gimbal_.calibrateZero();
  trackingReferenceReady_ = true;
  haveLastDirection_ = false;
  return true;
}

const GpsFix& TrackerApp::localGpsInput() const {
  return gps_.fix();
}

const BarometerSample& TrackerApp::localBarometerInput() const {
  return barometer_.sample();
}

const RemoteTargetSample& TrackerApp::remoteInput() const {
  return e22_.sample();
}
