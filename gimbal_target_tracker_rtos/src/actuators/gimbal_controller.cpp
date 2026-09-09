#include "gimbal_controller.h"

#include <math.h>

#include "../config/system_config.h"

using namespace ControlTableItem;

GimbalController::GimbalController(HardwareSerial& serial, int directionPin)
    : dxl_(serial, directionPin) {}

bool GimbalController::begin() {
  dxl_.begin(cfg::DXL_BAUD);
  dxl_.setPortProtocolVersion(2.0);

  const bool yawOk = dxl_.ping(cfg::YAW_DXL_ID);
  const bool pitchOk = dxl_.ping(cfg::PITCH_DXL_ID);
  state_.yawOnline = yawOk;
  state_.pitchOnline = pitchOk;
  state_.healthy = cfg::REQUIRE_BOTH_DXL ? (yawOk && pitchOk)
                                        : (yawOk || pitchOk);
  if (!state_.healthy) {
    setTorque(false);
    return false;
  }

  const uint8_t motorIds[2] = {cfg::YAW_DXL_ID, cfg::PITCH_DXL_ID};
  for (uint8_t index = 0; index < 2; ++index) {
    const uint8_t id = motorIds[index];
    const bool online = (id == cfg::YAW_DXL_ID) ? state_.yawOnline
                                                 : state_.pitchOnline;
    if (!online) continue;
    dxl_.torqueOff(id);
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    dxl_.writeControlTableItem(GOAL_CURRENT, id, cfg::DXL_GOAL_CURRENT);
    dxl_.writeControlTableItem(PROFILE_VELOCITY, id,
                               cfg::DXL_PROFILE_VELOCITY);
  }
  if (cfg::AUTO_CAPTURE_GIMBAL_ZERO_ON_BOOT) captureSoftZero();
  setTorque(!cfg::RTOS_DXL_TORQUE_OFF_SD_ISOLATION_TEST);
  return true;
}

void GimbalController::command(float yawDeg, float pitchDeg, float dtSeconds) {
  if (!state_.healthy || !state_.torqueOn) return;
  state_.limitActive = false;
  // Keep the filter stable if a slow SD/I2C transaction delayed one control
  // tick. A longer pause may move at the configured maximum rate, but cannot
  // create an unbounded command jump.
  if (dtSeconds < 0.001f) dtSeconds = 0.001f;
  if (dtSeconds > 0.100f) dtSeconds = 0.100f;

  const float normalizedYaw = wrapDeg(yawDeg);
  float safeYaw = normalizedYaw;
  float safePitch = pitchDeg;
  if (cfg::ENFORCE_SOFT_GIMBAL_LIMITS) {
    safeYaw = clampValue(normalizedYaw, cfg::YAW_MIN_DEG, cfg::YAW_MAX_DEG);
    safePitch = clampValue(pitchDeg, cfg::PITCH_MIN_DEG, cfg::PITCH_MAX_DEG);
    if (safeYaw != normalizedYaw || safePitch != pitchDeg) {
      state_.limitActive = true;
    }
  }

  float safeYawUnwrapped =
      nearestEquivalentDeg(safeYaw, filteredYawUnwrappedDeg_);
  if (fabsf(safeYawUnwrapped - filteredYawUnwrappedDeg_) <
      cfg::GIMBAL_COMMAND_DEADBAND_DEG) {
    safeYawUnwrapped = filteredYawUnwrappedDeg_;
  }
  if (fabsf(safePitch - filteredPitchDeg_) < cfg::GIMBAL_COMMAND_DEADBAND_DEG) {
    safePitch = filteredPitchDeg_;
  }
  filteredYawUnwrappedDeg_ = smoothAxis(
      safeYawUnwrapped, filteredYawUnwrappedDeg_, &yawRateDegS_, dtSeconds,
      false, &state_.limitActive);
  filteredYawDeg_ = wrapDeg(filteredYawUnwrappedDeg_);
  filteredPitchDeg_ = smoothAxis(safePitch, filteredPitchDeg_, &pitchRateDegS_,
                                 dtSeconds, false, &state_.limitActive);
  state_.yawCommandDeg = filteredYawDeg_;
  state_.yawCommandUnwrappedDeg = filteredYawUnwrappedDeg_;
  state_.pitchCommandDeg = filteredPitchDeg_;

  if (state_.yawOnline) {
    // Keep the proven single-turn servo coordinate until the multi-turn
    // command path has been bench-validated for this actuator/library pair.
    dxl_.setGoalPosition(cfg::YAW_DXL_ID,
                         servoDeg(cfg::YAW_SIGN *
                                      (filteredYawDeg_ + cfg::GIMBAL_YAW_REFERENCE_OFFSET_DEG) +
                                  yawZeroDeg_),
                         UNIT_DEGREE);
  }
  if (state_.pitchOnline) {
    dxl_.setGoalPosition(cfg::PITCH_DXL_ID,
                         servoDeg(cfg::PITCH_SIGN *
                                      (filteredPitchDeg_ + cfg::GIMBAL_PITCH_REFERENCE_OFFSET_DEG) +
                                  pitchZeroDeg_),
                         UNIT_DEGREE);
  }
}

bool GimbalController::captureSoftZero() {
  if (!state_.healthy) return false;
  if (state_.yawOnline) {
    yawZeroDeg_ = servoDeg(
        dxl_.getPresentPosition(cfg::YAW_DXL_ID, UNIT_DEGREE));
  }
  if (state_.pitchOnline) {
    pitchZeroDeg_ = servoDeg(
        dxl_.getPresentPosition(cfg::PITCH_DXL_ID, UNIT_DEGREE));
  }
  // Prevent a calibration from being followed by an old rate-limited target.
  filteredYawDeg_ = 0.0f;
  filteredYawUnwrappedDeg_ = 0.0f;
  filteredPitchDeg_ = 0.0f;
  yawRateDegS_ = 0.0f;
  pitchRateDegS_ = 0.0f;
  state_.yawCommandDeg = 0.0f;
  state_.yawCommandUnwrappedDeg = 0.0f;
  state_.yawPresentUnwrappedDeg = 0.0f;
  state_.pitchCommandDeg = 0.0f;
  return true;
}

void GimbalController::setTorque(bool enabled) {
  if (enabled && state_.healthy) {
    if (state_.yawOnline) dxl_.torqueOn(cfg::YAW_DXL_ID);
    if (state_.pitchOnline) dxl_.torqueOn(cfg::PITCH_DXL_ID);
    state_.torqueOn = true;
  } else {
    if (state_.yawOnline) dxl_.torqueOff(cfg::YAW_DXL_ID);
    if (state_.pitchOnline) dxl_.torqueOff(cfg::PITCH_DXL_ID);
    state_.torqueOn = false;
  }
}

void GimbalController::recordControlTiming(uint32_t intervalUs) {
  state_.lastControlIntervalUs = intervalUs;
  if (intervalUs > state_.maxControlIntervalUs)
    state_.maxControlIntervalUs = intervalUs;
  if (intervalUs > cfg::CONTROL_PERIOD_US + 2000U)
    ++state_.controlOverrunCount;
}

void GimbalController::pollFeedbackItem(FeedbackItem item) {
  if (!state_.healthy) return;
  const uint32_t startedUs = micros();
  switch (item) {
    case FeedbackItem::YAW_POSITION:
      if (state_.yawOnline) {
        const float wrappedPosition = cfg::YAW_SIGN * wrapDeg(
            dxl_.getPresentPosition(cfg::YAW_DXL_ID, UNIT_DEGREE) -
            yawZeroDeg_) - cfg::GIMBAL_YAW_REFERENCE_OFFSET_DEG;
        state_.yawPresentUnwrappedDeg = wrappedPosition;
        state_.yawPresentDeg = wrappedPosition;
      }
      break;
    case FeedbackItem::PITCH_POSITION:
      if (state_.pitchOnline) {
        state_.pitchPresentDeg = cfg::PITCH_SIGN * wrapDeg(
            dxl_.getPresentPosition(cfg::PITCH_DXL_ID, UNIT_DEGREE) -
            pitchZeroDeg_) - cfg::GIMBAL_PITCH_REFERENCE_OFFSET_DEG;
      }
      break;
    case FeedbackItem::YAW_CURRENT:
      if (state_.yawOnline) {
        state_.yawCurrentRaw =
            dxl_.readControlTableItem(PRESENT_CURRENT, cfg::YAW_DXL_ID);
      }
      break;
    case FeedbackItem::PITCH_CURRENT:
      if (state_.pitchOnline) {
        state_.pitchCurrentRaw =
            dxl_.readControlTableItem(PRESENT_CURRENT, cfg::PITCH_DXL_ID);
      }
      break;
    case FeedbackItem::YAW_TEMPERATURE:
      if (state_.yawOnline) {
        state_.yawTemperatureC =
            dxl_.readControlTableItem(PRESENT_TEMPERATURE, cfg::YAW_DXL_ID);
      }
      break;
    case FeedbackItem::PITCH_TEMPERATURE:
      if (state_.pitchOnline) {
        state_.pitchTemperatureC =
            dxl_.readControlTableItem(PRESENT_TEMPERATURE, cfg::PITCH_DXL_ID);
      }
      break;
  }

  const bool yawOverTemperature =
      state_.yawOnline &&
      state_.yawTemperatureC >= cfg::DXL_TEMP_LIMIT_C;
  const bool pitchOverTemperature =
      state_.pitchOnline &&
      state_.pitchTemperatureC >= cfg::DXL_TEMP_LIMIT_C;
  if (yawOverTemperature || pitchOverTemperature) {
    setTorque(false);
    state_.healthy = false;
  }
  state_.lastFeedbackUs = static_cast<uint32_t>(micros() - startedUs);
  if (state_.lastFeedbackUs > state_.maxFeedbackUs)
    state_.maxFeedbackUs = state_.lastFeedbackUs;
  if (state_.lastFeedbackUs > cfg::DXL_FEEDBACK_SLOW_US)
    ++state_.slowFeedbackCount;
}

float GimbalController::wrapDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float GimbalController::nearestEquivalentDeg(float wrappedTarget,
                                             float reference) {
  return reference + wrapDeg(wrappedTarget - wrapDeg(reference));
}

float GimbalController::servoDeg(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float GimbalController::clampValue(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

float GimbalController::moveToward(float value, float target, float maxDelta) {
  if (target > value + maxDelta) return value + maxDelta;
  if (target < value - maxDelta) return value - maxDelta;
  return target;
}

float GimbalController::smoothAxis(float target, float current,
                                   float* rateDegS, float dtSeconds,
                                   bool circular, bool* limited) {
  const float rawDelta = circular ? wrapDeg(target - current) : target - current;
  const float alpha = dtSeconds /
      (cfg::GIMBAL_TARGET_FILTER_TIME_CONSTANT_S + dtSeconds);
  const float filteredDelta = rawDelta * alpha;
  const float desiredRate = clampValue(filteredDelta / dtSeconds,
                                       -cfg::MAX_RATE_DEG_S,
                                       cfg::MAX_RATE_DEG_S);
  const float maxRateDelta = cfg::MAX_ACCEL_DEG_S2 * dtSeconds;
  float nextRate = moveToward(*rateDegS, desiredRate, maxRateDelta);
  float step = nextRate * dtSeconds;

  // Never overshoot a nearby target. Reset the velocity only after arriving;
  // otherwise short GPS target changes are still eased by acceleration.
  if (fabsf(rawDelta) > cfg::GIMBAL_COMMAND_DEADBAND_DEG &&
      ((rawDelta >= 0.0f && step >= rawDelta) ||
       (rawDelta < 0.0f && step <= rawDelta))) {
    step = rawDelta;
    nextRate = 0.0f;
  }
  if (fabsf(rawDelta) > cfg::GIMBAL_COMMAND_DEADBAND_DEG ||
      fabsf(nextRate - *rateDegS) > 0.01f) {
    *limited = true;
  }
  *rateDegS = nextRate;
  return circular ? wrapDeg(current + step) : current + step;
}
