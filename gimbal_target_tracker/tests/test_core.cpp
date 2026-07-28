#include <cassert>
#include <cmath>
#include <cstdio>

#include "../src/comm/remote_packet.h"
#include "../src/math/target_geometry.h"
#include "../src/simulation/virtual_remote_source.h"

static bool nearValue(float actual, float expected, float tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

int main() {
  using namespace remote_protocol;

  Payload sent;
  sent.sequence = 42;
  sent.senderTimeMs = 123456;
  sent.latI7 = 375000123;
  sent.lonI7 = 1270000456;
  sent.pressurePaX10 = 1001290;
  sent.temperatureCX100 = 2350;
  sent.fixType = 3;
  sent.flags = FLAG_GPS_VALID | FLAG_BARO_VALID;

  uint8_t wire[PACKET_SIZE];
  assert(encode(sent, wire, sizeof(wire)) == PACKET_SIZE);

  Payload decoded;
  assert(decode(wire, sizeof(wire), &decoded));
  assert(decoded.sequence == sent.sequence);
  assert(decoded.latI7 == sent.latI7);
  assert(decoded.lonI7 == sent.lonI7);
  assert(decoded.pressurePaX10 == sent.pressurePaX10);

  Parser parser;
  Payload parsed;
  assert(!parser.feed(0x00, &parsed));
  assert(!parser.feed('G', &parsed));
  for (size_t i = 0; i < PACKET_SIZE; ++i) {
    const bool complete = parser.feed(wire[i], &parsed);
    if (i + 1 < PACKET_SIZE) assert(!complete);
    if (i + 1 == PACKET_SIZE) assert(complete);
  }
  assert(parser.packetsOk == 1);

  const float height =
      target_geometry::relativeAltitudeM(101325.0f, 100129.0f);
  assert(nearValue(height, 100.0f, 2.0f));

  GpsFix local;
  local.latI7 = 375000000;
  local.lonI7 = 1270000000;
  local.valid = true;
  BarometerSample localBaro;
  localBaro.pressurePa = 101325.0f;
  localBaro.valid = true;
  RemoteTargetSample remote;
  remote.latI7 = local.latI7 + 898;  // approximately 10 m north
  remote.lonI7 = local.lonI7;
  remote.pressurePa = 101325.0f;
  remote.valid = true;

  float ned[3];
  assert(target_geometry::relativeNed(local, localBaro, remote, ned));
  assert(nearValue(ned[0], 10.0f, 0.1f));
  assert(nearValue(ned[1], 0.0f, 0.01f));
  assert(nearValue(ned[2], 0.0f, 0.01f));

  const float identity[4] = {1, 0, 0, 0};
  RelativeTarget target;
  assert(target_geometry::pointingAngles(ned, identity, &target));
  assert(nearValue(target.yawDeg, 0.0f, 0.01f));
  assert(nearValue(target.pitchDeg, 0.0f, 0.01f));

  VirtualRemoteSource simulation;
  simulation.setMode(VirtualRemoteSource::Mode::FULL_BENCH, 1000);
  GpsFix noRealGps;
  BarometerSample noRealBarometer;
  assert(simulation.update(1000, noRealGps, noRealBarometer));
  assert(simulation.virtualLocalGps().valid);
  assert(simulation.virtualLocalBarometer().valid);
  assert(simulation.remote().valid);
  float simulatedNed[3];
  assert(target_geometry::relativeNed(simulation.virtualLocalGps(),
                                      simulation.virtualLocalBarometer(),
                                      simulation.remote(), simulatedNed));
  assert(nearValue(simulatedNed[0], 150.0f, 0.2f));
  assert(nearValue(simulatedNed[1], 0.0f, 0.2f));
  assert(nearValue(simulatedNed[2], 0.0f, 0.2f));

  std::puts("test_core: all checks passed");
  return 0;
}
