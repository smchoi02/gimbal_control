#include "gimbal_controller.h"

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
  setTorque(true);
  return true;
}

void GimbalController::command(float yawDeg, float pitchDeg, float dtSeconds) {
  if (!state_.healthy || !state_.torqueOn) return;
  state_.limitActive = false;
  const float maxStep = cfg::MAX_RATE_DEG_S * dtSeconds;

  const float normalizedYaw = wrapDeg(yawDeg);
  float safeYaw =
      clampValue(normalizedYaw, cfg::YAW_MIN_DEG, cfg::YAW_MAX_DEG);
  float safePitch =
      clampValue(pitchDeg, cfg::PITCH_MIN_DEG, cfg::PITCH_MAX_DEG);
  if (safeYaw != normalizedYaw || safePitch != pitchDeg) {
    state_.limitActive = true;
  }

  safeYaw = rateLimit(safeYaw, filteredYawDeg_, maxStep, &state_.limitActive);
  safePitch =
      rateLimit(safePitch, filteredPitchDeg_, maxStep, &state_.limitActive);
  filteredYawDeg_ = safeYaw;
  filteredPitchDeg_ = safePitch;
  state_.yawCommandDeg = safeYaw;
  state_.pitchCommandDeg = safePitch;

  if (state_.yawOnline) {
    dxl_.setGoalPosition(cfg::YAW_DXL_ID,
                         cfg::YAW_SIGN * safeYaw + cfg::DXL_CENTER_DEG,
                         UNIT_DEGREE);
  }
  if (state_.pitchOnline) {
    dxl_.setGoalPosition(cfg::PITCH_DXL_ID,
                         cfg::PITCH_SIGN * safePitch + cfg::DXL_CENTER_DEG,
                         UNIT_DEGREE);
  }
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

void GimbalController::pollFeedback() {
  if (!state_.healthy) return;
  if (state_.yawOnline) {
    state_.yawPresentDeg =
        cfg::YAW_SIGN *
        (dxl_.getPresentPosition(cfg::YAW_DXL_ID, UNIT_DEGREE) -
         cfg::DXL_CENTER_DEG);
    state_.yawCurrentRaw =
        dxl_.readControlTableItem(PRESENT_CURRENT, cfg::YAW_DXL_ID);
    state_.yawTemperatureC =
        dxl_.readControlTableItem(PRESENT_TEMPERATURE, cfg::YAW_DXL_ID);
  }
  if (state_.pitchOnline) {
    state_.pitchPresentDeg =
        cfg::PITCH_SIGN *
        (dxl_.getPresentPosition(cfg::PITCH_DXL_ID, UNIT_DEGREE) -
         cfg::DXL_CENTER_DEG);
    state_.pitchCurrentRaw =
        dxl_.readControlTableItem(PRESENT_CURRENT, cfg::PITCH_DXL_ID);
    state_.pitchTemperatureC =
        dxl_.readControlTableItem(PRESENT_TEMPERATURE, cfg::PITCH_DXL_ID);
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
}

float GimbalController::wrapDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float GimbalController::clampValue(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

float GimbalController::rateLimit(float target, float current, float maxStep,
                                  bool* limited) {
  const float delta = target - current;
  if (delta > maxStep) {
    *limited = true;
    return current + maxStep;
  }
  if (delta < -maxStep) {
    *limited = true;
    return current - maxStep;
  }
  return target;
}
