#include "virtual_remote_source.h"

#include <math.h>

#include "../math/attitude_math.h"
#include "../math/target_geometry.h"
#include "fixed_target_config.h"
#include "virtual_target_dataset.h"

namespace {

float interpolate(float a, float b, float fraction) {
  return a + (b - a) * fraction;
}

void datasetPosition(uint32_t elapsedMs, float* northM, float* eastM,
                     float* upM) {
  const uint32_t loopTime = elapsedMs % VIRTUAL_TARGET_LOOP_MS;
  size_t upper = 1;
  while (upper < VIRTUAL_TARGET_KEYFRAME_COUNT &&
         loopTime > VIRTUAL_TARGET_DATASET[upper].timeMs) {
    ++upper;
  }
  if (upper >= VIRTUAL_TARGET_KEYFRAME_COUNT) {
    upper = VIRTUAL_TARGET_KEYFRAME_COUNT - 1;
  }
  const VirtualTargetKeyframe& a = VIRTUAL_TARGET_DATASET[upper - 1];
  const VirtualTargetKeyframe& b = VIRTUAL_TARGET_DATASET[upper];
  const float fraction =
      static_cast<float>(loopTime - a.timeMs) /
      static_cast<float>(b.timeMs - a.timeMs);
  *northM = interpolate(a.northM, b.northM, fraction);
  *eastM = interpolate(a.eastM, b.eastM, fraction);
  *upM = interpolate(a.upM, b.upM, fraction);
}

float pressureAtRelativeHeight(float localPressurePa, float remoteUpM) {
  const float base =
      1.0f - remoteUpM / target_geometry::BARO_SCALE_M;
  if (base <= 0.0f) return 0.0f;
  return localPressurePa *
         powf(base, 1.0f / target_geometry::BARO_EXPONENT);
}

float altitudeFromPressure(float pressurePa, float seaLevelPressurePa) {
  if (pressurePa <= 0.0f || seaLevelPressurePa <= 0.0f) return 0.0f;
  return target_geometry::BARO_SCALE_M *
         (1.0f -
          powf(pressurePa / seaLevelPressurePa,
               target_geometry::BARO_EXPONENT));
}

}  // namespace

void VirtualRemoteSource::setMode(Mode mode, uint32_t nowMs) {
  mode_ = mode;
  restart(nowMs);
  if (mode_ == Mode::OFF) remote_.valid = false;
}

void VirtualRemoteSource::restart(uint32_t nowMs) {
  startMs_ = nowMs;
  lastPacketMs_ = nowMs - 100;
  sequence_ = 0;
}

bool VirtualRemoteSource::update(
    uint32_t nowMs, const GpsFix& realLocalGps,
    const BarometerSample& realLocalBarometer) {
  if (mode_ == Mode::OFF) return false;
  if (mode_ == Mode::FULL_BENCH) updateVirtualLocal(nowMs);
  if (static_cast<uint32_t>(nowMs - lastPacketMs_) < 100) return false;
  lastPacketMs_ = nowMs;

  const GpsFix& localGps =
      usesVirtualLocal() ? virtualLocalGps_ : realLocalGps;
  const BarometerSample& localBarometer =
      usesVirtualLocal() ? virtualLocalBarometer_ : realLocalBarometer;
  if (mode_ == Mode::FIXED_ABSOLUTE_TARGET) {
    return createFixedTargetPacket(nowMs, localGps, localBarometer);
  }
  return createPacket(nowMs, localGps, localBarometer);
}

void VirtualRemoteSource::updateVirtualLocal(uint32_t nowMs) {
  virtualLocalGps_.iTowMs = nowMs;
  virtualLocalGps_.latI7 = VIRTUAL_LOCAL_LAT_I7;
  virtualLocalGps_.lonI7 = VIRTUAL_LOCAL_LON_I7;
  virtualLocalGps_.hMslMm = 0;
  virtualLocalGps_.velNMmS = 0;
  virtualLocalGps_.velEMmS = 0;
  virtualLocalGps_.velDMmS = 0;
  virtualLocalGps_.fixType = 3;
  virtualLocalGps_.numSv = 12;
  virtualLocalGps_.timestampMs = nowMs;
  virtualLocalGps_.valid = true;

  virtualLocalBarometer_.pressurePa = VIRTUAL_LOCAL_PRESSURE_PA;
  virtualLocalBarometer_.temperatureC = VIRTUAL_LOCAL_TEMPERATURE_C;
  virtualLocalBarometer_.timestampMs = nowMs;
  virtualLocalBarometer_.valid = true;
}

bool VirtualRemoteSource::createPacket(
    uint32_t nowMs, const GpsFix& localGps,
    const BarometerSample& localBarometer) {
  if (!localGps.valid || !localBarometer.valid ||
      localBarometer.pressurePa <= 0.0f) {
    remote_.valid = false;
    return false;
  }

  float northM = 0.0f, eastM = 0.0f, upM = 0.0f;
  datasetPosition(static_cast<uint32_t>(nowMs - startMs_), &northM, &eastM,
                  &upM);
  const float latitudeRad =
      static_cast<float>(static_cast<double>(localGps.latI7) * 1.0e-7) *
      attitude::DEG_TO_RAD_F;
  const float cosLatitude = cosf(latitudeRad);
  if (fabsf(cosLatitude) < 1.0e-6f) return false;

  remote_protocol::Payload payload;
  payload.sequence = sequence_++;
  payload.senderTimeMs = nowMs;
  payload.latI7 =
      localGps.latI7 +
      static_cast<int32_t>(lroundf(
          northM / target_geometry::METERS_PER_I7_LAT));
  payload.lonI7 =
      localGps.lonI7 +
      static_cast<int32_t>(lroundf(
          eastM / (target_geometry::METERS_PER_I7_LAT * cosLatitude)));
  const float remotePressure =
      pressureAtRelativeHeight(localBarometer.pressurePa, upM);
  payload.pressurePaX10 =
      static_cast<uint32_t>(lroundf(remotePressure * 10.0f));
  payload.temperatureCX100 =
      static_cast<int16_t>(lroundf(localBarometer.temperatureC * 100.0f));
  payload.fixType = 3;
  payload.flags =
      remote_protocol::FLAG_GPS_VALID | remote_protocol::FLAG_BARO_VALID;

  uint8_t bytes[remote_protocol::PACKET_SIZE];
  const size_t length =
      remote_protocol::encode(payload, bytes, sizeof(bytes));
  remote_protocol::Payload decoded;
  bool complete = false;
  for (size_t i = 0; i < length; ++i) {
    if (parser_.feed(bytes[i], &decoded)) complete = true;
  }
  if (!complete) {
    remote_.valid = false;
    return false;
  }
  remote_ = remote_protocol::toSample(decoded, nowMs);
  return remote_.valid;
}

bool VirtualRemoteSource::createFixedTargetPacket(
    uint32_t nowMs, const GpsFix& localGps,
    const BarometerSample& localBarometer) {
  if (!localGps.valid || !localBarometer.valid ||
      localBarometer.pressurePa <= 0.0f) {
    remote_.valid = false;
    return false;
  }

  // Convert the receiver's real BMP581 pressure to an absolute barometric
  // altitude, then encode the fixed target altitude as a remote pressure.
  // The existing relative-altitude and packet-processing paths remain intact.
  const float localAltitudeM =
      altitudeFromPressure(localBarometer.pressurePa,
                           fixed_target::SEA_LEVEL_PRESSURE_PA);
  const float targetAboveLocalM =
      fixed_target::ALTITUDE_M - localAltitudeM;
  const float remotePressurePa =
      pressureAtRelativeHeight(localBarometer.pressurePa,
                               targetAboveLocalM);
  if (!isfinite(remotePressurePa) || remotePressurePa <= 0.0f) {
    remote_.valid = false;
    return false;
  }

  remote_protocol::Payload payload;
  payload.sequence = sequence_++;
  payload.senderTimeMs = nowMs;
  payload.latI7 = fixed_target::LAT_I7;
  payload.lonI7 = fixed_target::LON_I7;
  payload.pressurePaX10 =
      static_cast<uint32_t>(lroundf(remotePressurePa * 10.0f));
  payload.temperatureCX100 = static_cast<int16_t>(
      lroundf(fixed_target::TEMPERATURE_C * 100.0f));
  payload.fixType = 3;
  payload.flags =
      remote_protocol::FLAG_GPS_VALID | remote_protocol::FLAG_BARO_VALID;

  uint8_t bytes[remote_protocol::PACKET_SIZE];
  const size_t length =
      remote_protocol::encode(payload, bytes, sizeof(bytes));
  remote_protocol::Payload decoded;
  bool complete = false;
  for (size_t i = 0; i < length; ++i) {
    if (parser_.feed(bytes[i], &decoded)) complete = true;
  }
  if (!complete) {
    remote_.valid = false;
    return false;
  }
  remote_ = remote_protocol::toSample(decoded, nowMs);
  return remote_.valid;
}
