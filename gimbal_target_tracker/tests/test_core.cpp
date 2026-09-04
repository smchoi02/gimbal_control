#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../src/comm/remote_packet.h"
#include "../src/math/target_geometry.h"

static bool nearValue(float actual, float expected, float tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

static void feedFrame(remote_protocol::Parser* parser, const uint8_t* frame,
                      remote_protocol::Payload* payload, bool* complete) {
  *complete = false;
  for (size_t i = 0; i < remote_protocol::PACKET_SIZE; ++i) {
    if (parser->feed(frame[i], payload)) *complete = true;
  }
}

int main() {
  using namespace remote_protocol;

  // This is the final_test contract: byte 3 is UBX NAV-PVT fixType (3=3D),
  // iTOW is little-endian, and all distances/velocities are signed mm units.
  Payload sent;
  sent.sequence = 42;
  sent.fixType = 3;
  sent.iTowMs = 123456;
  sent.latI7 = 375000123;
  sent.lonI7 = 1270000456;
  sent.aglMm = 100000;
  sent.velNMmS = 1200;
  sent.velEMmS = -500;
  sent.velDMmS = 300;

  uint8_t wire[PACKET_SIZE];
  assert(encode(sent, wire, sizeof(wire)) == PACKET_SIZE);
  assert(PACKET_SIZE == 34 && CRC_OFFSET == 32);
  assert(wire[0] == 'R' && wire[1] == 'K');
  assert(wire[2] == sent.sequence && wire[3] == 3);
  assert(readU32(wire + 4) == sent.iTowMs);
  assert(static_cast<int32_t>(readU32(wire + 8)) == sent.latI7);
  assert(static_cast<int32_t>(readU32(wire + 12)) == sent.lonI7);
  assert(static_cast<int32_t>(readU32(wire + 16)) == sent.aglMm);
  assert(static_cast<int32_t>(readU32(wire + 20)) == sent.velNMmS);
  assert(static_cast<int32_t>(readU32(wire + 24)) == sent.velEMmS);
  assert(static_cast<int32_t>(readU32(wire + 28)) == sent.velDMmS);
  assert(readU16(wire + CRC_OFFSET) == crc16CcittFalse(wire, CRC_OFFSET));

  Payload decoded;
  assert(decode(wire, sizeof(wire), &decoded));
  const RemoteTargetSample remoteFix3 = toSample(decoded, 9000);
  assert(remoteFix3.valid);
  assert(remoteFix3.fixType == 3);
  assert(remoteFix3.senderITowMs == sent.iTowMs);
  assert(nearValue(remoteFix3.velEMps, -0.5f, 0.0001f));

  decoded.fixType = 4;
  assert(toSample(decoded, 9000).valid);
  decoded.fixType = 2;
  assert(!toSample(decoded, 9000).valid);
  decoded = sent;
  decoded.latI7 = 900000001;
  assert(!toSample(decoded, 9000).valid);

  // Garbage before the frame is ignored, as is a CRC-failed frame. The next
  // valid header is rediscovered without a parser reset.
  Parser parser;
  Payload parsed;
  assert(!parser.feed(0x00, &parsed));
  assert(!parser.feed('G', &parsed));
  uint8_t corrupt[PACKET_SIZE];
  memcpy(corrupt, wire, sizeof(corrupt));
  corrupt[15] ^= 0x7F;
  bool complete = false;
  feedFrame(&parser, corrupt, &parsed, &complete);
  assert(!complete && parser.packetsBad == 1);
  feedFrame(&parser, wire, &parsed, &complete);
  assert(complete && parser.packetsOk == 1 && parser.lastFrameIsNew());

  // A truncated frame followed by a complete frame must resynchronize. It may
  // count the truncated accumulation as bad, but it recovers the next frame.
  Parser partialParser;
  for (size_t i = 0; i < 11; ++i) assert(!partialParser.feed(wire[i], &parsed));
  feedFrame(&partialParser, wire, &parsed, &complete);
  assert(complete && partialParser.packetsOk == 1);

  // Sequence bookkeeping: lost frames are counted, while duplicate and late
  // frames do not become a new packet that could refresh the local age.
  Payload seq = sent;
  Parser sequenceParser;
  seq.sequence = 250; encode(seq, wire, sizeof(wire));
  feedFrame(&sequenceParser, wire, &parsed, &complete);
  assert(complete && sequenceParser.lastFrameIsNew());
  seq.sequence = 252; encode(seq, wire, sizeof(wire));
  feedFrame(&sequenceParser, wire, &parsed, &complete);
  assert(sequenceParser.lastFrameIsNew() && sequenceParser.sequenceLost == 1);
  feedFrame(&sequenceParser, wire, &parsed, &complete);
  assert(!sequenceParser.lastFrameIsNew() && sequenceParser.sequenceDuplicate == 1);
  seq.sequence = 251; encode(seq, wire, sizeof(wire));
  feedFrame(&sequenceParser, wire, &parsed, &complete);
  assert(!sequenceParser.lastFrameIsNew() && sequenceParser.sequenceOutOfOrder == 1);
  seq.sequence = 0; encode(seq, wire, sizeof(wire));
  feedFrame(&sequenceParser, wire, &parsed, &complete);
  assert(sequenceParser.lastFrameIsNew());  // uint8 sequence wrap is forward.

  GpsFix local;
  local.latI7 = 375000000;
  local.lonI7 = 1270000000;
  local.valid = true;
  BarometerSample localBaro;
  localBaro.valid = true;
  localBaro.referenceValid = true;
  localBaro.aglM = 20.0f;
  RemoteTargetSample remote;
  remote.latI7 = local.latI7 + 898;  // approximately 10 m north
  remote.lonI7 = local.lonI7;
  remote.aglM = 125.0f;
  remote.valid = true;

  float ned[3];
  assert(target_geometry::relativeNed(local, localBaro, remote, 5.0f, ned));
  assert(nearValue(ned[0], 10.0f, 0.1f));
  assert(nearValue(ned[1], 0.0f, 0.01f));
  assert(nearValue(ned[2], -100.0f, 0.01f));
  assert(nearValue(target_geometry::horizontalRangeM(ned), 10.0f, 0.1f));

  const float identity[4] = {1, 0, 0, 0};
  RelativeTarget target;
  assert(target_geometry::pointingAngles(ned, identity, &target));
  assert(nearValue(target.yawDeg, 0.0f, 0.01f));
  assert(nearValue(target.pitchDeg, -84.29f, 0.1f));

  std::puts("test_core: all checks passed");
  return 0;
}
