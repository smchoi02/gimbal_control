#include <cassert>
#include <cmath>
#include <cstdio>

#include "../src/comm/remote_packet.h"
#include "../src/math/target_geometry.h"

static bool nearValue(float actual, float expected, float tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

int main() {
  using namespace remote_protocol;

  Payload sent;
  sent.sequence = 42;
  sent.iTowMs = 123456;
  sent.latI7 = 375000123;
  sent.lonI7 = 1270000456;
  sent.aglMm = 100000;
  sent.velNMmS = 1200;
  sent.velEMmS = -500;
  sent.velDMmS = 300;
  sent.imuFlags = 0x07;
  sent.quatAccuracy = 3;
  sent.imuAgeMs = 20;
  sent.accelMps2X100[0] = 981;
  sent.gyroDpsX10[2] = -123;
  sent.quaternionQ14[0] = 16384;
  sent.fixType = 3;

  uint8_t wire[PACKET_SIZE];
  assert(encode(sent, wire, sizeof(wire)) == PACKET_SIZE);

  Payload decoded;
  assert(decode(wire, sizeof(wire), &decoded));
  assert(decoded.sequence == sent.sequence);
  assert(decoded.latI7 == sent.latI7);
  assert(decoded.lonI7 == sent.lonI7);
  assert(decoded.aglMm == sent.aglMm);
  assert(decoded.imuFlags == sent.imuFlags);
  assert(decoded.gyroDpsX10[2] == sent.gyroDpsX10[2]);

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

  GpsFix local;
  local.latI7 = 375000000;
  local.lonI7 = 1270000000;
  local.valid = true;
  RemoteTargetSample remote;
  remote.latI7 = local.latI7 + 898;  // approximately 10 m north
  remote.lonI7 = local.lonI7;
  remote.aglM = 100.0f;
  remote.valid = true;

  float ned[3];
  assert(target_geometry::relativeNed(local, remote, ned));
  assert(nearValue(ned[0], 10.0f, 0.1f));
  assert(nearValue(ned[1], 0.0f, 0.01f));
  assert(nearValue(ned[2], -100.0f, 0.01f));


  const float identity[4] = {1, 0, 0, 0};
  RelativeTarget target;
  assert(target_geometry::pointingAngles(ned, identity, &target));
  assert(nearValue(target.yawDeg, 0.0f, 0.01f));
  assert(nearValue(target.pitchDeg, 0.0f, 0.01f));

  std::puts("test_core: all checks passed");
  return 0;
}
