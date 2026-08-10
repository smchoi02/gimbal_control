#include "tracker_app.h"

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
  debug_.println(F("# gimbal_target_tracker boot"));

  const bool imuOk =
      imu_.begin(cfg::BNO085_ADDRESS, 20, cfg::BNO_USE_GAME_ROTATION_VECTOR);
  debug_.print(F("# BNO085: "));
  if (imuOk) {
    debug_.println(imu_.usingGameRotationVector()
                       ? F("OK; Game Rotation Vector")
                       : F("OK; magnetic Rotation Vector"));
  } else {
    debug_.println(F("FAIL"));
  }

  const bool baroOk = barometer_.begin(cfg::BMP581_ADDRESS);
  debug_.print(F("# BMP581: "));
  debug_.print(baroOk ? F("OK id=0x") : F("FAIL id=0x"));
  debug_.println(barometer_.chipId(), HEX);

  const bool gpsOk = gps_.begin(cfg::MAX_M10S_ADDRESS);
  debug_.print(F("# MAX-M10S: "));
  debug_.println(gpsOk ? F("OK; NAV-PVT 10Hz requested") : F("FAIL"));

  e22_.begin(cfg::E22_M0_PIN, cfg::E22_M1_PIN, cfg::E22_AUX_PIN);
  debug_.print(F("# E22-900T22S: normal/transparent receiver "));
  debug_.println(e22_.moduleReady() ? F("ready") : F("starting/busy"));

  const bool gimbalOk = gimbal_.begin();
  debug_.print(F("# DYNAMIXEL: "));
  if (gimbalOk) {
    debug_.print(F("OK yaw="));
    debug_.print(gimbal_.state().yawOnline ? 1 : 0);
    debug_.print(F(" pitch="));
    debug_.println(gimbal_.state().pitchOnline ? 1 : 0);
  } else {
    debug_.println(F("FAIL (no configured motor answered ping)"));
  }

  const bool sdOk = logger_.begin(cfg::SD_CS_PIN);
  debug_.print(F("# SD: "));
  if (sdOk) {
    debug_.print(F("OK file="));
    debug_.println(logger_.filename());
  } else {
    debug_.println(F("FAIL (tracking continues without logging)"));
  }

  if (!gimbalOk) mode_ = TrackMode::FAULT;
  debug_.println(F("# commands: ? | C | Z | T0/T1 | K | P"));

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

  if (trackingInputsFresh(nowMs)) {
    if (!trackingReferenceReady_) {
      captureImuReference();
      mode_ = TrackMode::STOW;
      relative_.valid = false;
      gimbal_.stow(dtSeconds);
      return;
    }
    float targetNed[3];
    RelativeTarget calculated;
    float relativeAttitudeQ[4];
    attitude::relativeToReference(localImuReferenceQ_, imu_.sample().q,
                                  relativeAttitudeQ);
    if (target_geometry::relativeNed(localGpsInput(), remoteInput(),
                                     targetNed) &&
        target_geometry::pointingAngles(targetNed, relativeAttitudeQ,
                                        &calculated)) {
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
  const remote_protocol::Parser& selectedParser = e22_.parser();
  debug_.print(F("# mode="));
  debug_.print(static_cast<uint8_t>(mode_));
  debug_.print(F(" imu="));
  debug_.print(attitudeFresh(nowMs));
  debug_.print(F(" imu_hw/report/src="));
  debug_.print(imu_.present());
  debug_.print('/');
  debug_.print(imu_.reportEnabled());
  debug_.print('/');
  debug_.print(imu_.sourceCode());
  debug_.print(F(" imu_rst/rec="));
  debug_.print(imu_.resetCount());
  debug_.print('/');
  debug_.print(imu_.reportRecoveryCount());
  debug_.print(F(" baro="));
  debug_.print(localBarometerInput().valid &&
               isFresh(nowMs, localBarometerInput().timestampMs,
                       cfg::LOCAL_BARO_TIMEOUT_MS));
  debug_.print(F(" gps="));
  debug_.print(localGpsInput().valid &&
               isFresh(nowMs, localGpsInput().timestampMs,
                       cfg::LOCAL_GPS_TIMEOUT_MS));
  debug_.print(F(" remote="));
  debug_.print(remoteInput().valid &&
               isFresh(nowMs, remoteInput().timestampMs,
                       cfg::REMOTE_TIMEOUT_MS));
  debug_.print(F(" e22_ready="));
  debug_.print(e22_.moduleReady());
  debug_.print(F(" imu_ref/remote_imu_ref="));
  debug_.print(trackingReferenceReady_);
  debug_.print('/');
  debug_.print(remoteImuReferenceReady_);
  debug_.print(F(" range_m="));
  debug_.print(relative_.rangeM, 1);
  debug_.print(F(" cmd="));
  debug_.print(gimbal_.state().yawCommandDeg, 1);
  debug_.print(',');
  debug_.print(gimbal_.state().pitchCommandDeg, 1);
  debug_.print(F(" rx_ok/bad/lost="));
  debug_.print(selectedParser.packetsOk);
  debug_.print('/');
  debug_.print(selectedParser.packetsBad);
  debug_.print('/');
  debug_.print(selectedParser.sequenceLost);
  debug_.print(F(" sd_rows/errors="));
  debug_.print(logger_.rowsWritten());
  debug_.print('/');
  debug_.println(logger_.writeErrors());
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
      debug_.println(F("# tracking enabled"));
      break;
    case 'Z':
      trackingEnabled_ = false;
      debug_.println(F("# tracking disabled; STOW"));
      break;
    case 'T':
      gimbal_.setTorque(line[1] == '1');
      debug_.println(line[1] == '1' ? F("# torque ON") : F("# torque OFF"));
      break;
    case 'K':
      if (gimbal_.calibrateZero()) {
        debug_.println(F("# gimbal zero calibrated at current pose"));
      } else {
        debug_.println(F("# gimbal zero calibration failed: no motor online"));
      }
      break;
    case 'R':
      trackingReferenceReady_ = false;
      remoteImuReferenceReady_ = false;
      haveLastDirection_ = false;
      relative_.valid = false;
      debug_.println(F("# IMU reference reset; waiting for real RK packet"));
      break;
    case 'P':
      gps_.configure10Hz();
      debug_.println(F("# MAX-M10S configuration sent"));
      break;
    case '?':
      debug_.println(
          F("# C=track, Z=stow, T0/T1=torque, K=zero now, R=reference reset, P=GPS cfg"));
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

bool TrackerApp::captureImuReference() {
  if (!localGpsInput().valid || !remoteInput().valid || !imu_.sample().valid) {
    return false;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    localImuReferenceQ_[i] = imu_.sample().q[i];
  }
  attitude::normalizeQuaternion(localImuReferenceQ_);
  remoteImuReferenceReady_ = (remoteInput().imuFlags & 0x04u) != 0;
  if (remoteImuReferenceReady_) {
    for (uint8_t i = 0; i < 4; ++i) {
      remoteImuReferenceQ_[i] = remoteInput().quaternion[i];
    }
    attitude::normalizeQuaternion(remoteImuReferenceQ_);
  }
  trackingReferenceReady_ = true;
  haveLastDirection_ = false;
  debug_.println(F("# local IMU reference captured; target starts at yaw/pitch 0/0"));
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
