#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../common/crc16.h"
#include "../common/types.h"

// TRS RK wire format, little-endian, 34 bytes total.
//  0 "RK", 2 seq, 3 satellite count, 4 UTC time-of-day ms, 8 lat_i7, 12 lon_i7,
// 16 AGL mm, 20/24/28 N/E/D mm/s,
// 32 CRC-16/CCITT-FALSE over bytes 0..31.
namespace remote_protocol {

constexpr uint8_t MAGIC_0 = 'R';
constexpr uint8_t MAGIC_1 = 'K';
constexpr size_t PACKET_SIZE = 34;
constexpr size_t CRC_OFFSET = 32;

struct Payload {
  uint8_t sequence = 0;
  uint8_t satelliteCount = 0;
  uint32_t iTowMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  int32_t aglMm = 0;
  int32_t velNMmS = 0;
  int32_t velEMmS = 0;
  int32_t velDMmS = 0;
};

inline void writeU16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}
inline void writeU32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}
inline uint16_t readU16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) |
         (static_cast<uint16_t>(in[1]) << 8);
}
inline int16_t readI16(const uint8_t* in) {
  return static_cast<int16_t>(readU16(in));
}
inline uint32_t readU32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

inline size_t encode(const Payload& payload, uint8_t* out, size_t capacity) {
  if (capacity < PACKET_SIZE) return 0;
  out[0] = MAGIC_0; out[1] = MAGIC_1;
  out[2] = payload.sequence; out[3] = payload.satelliteCount;
  writeU32(out + 4, payload.iTowMs);
  writeU32(out + 8, static_cast<uint32_t>(payload.latI7));
  writeU32(out + 12, static_cast<uint32_t>(payload.lonI7));
  writeU32(out + 16, static_cast<uint32_t>(payload.aglMm));
  writeU32(out + 20, static_cast<uint32_t>(payload.velNMmS));
  writeU32(out + 24, static_cast<uint32_t>(payload.velEMmS));
  writeU32(out + 28, static_cast<uint32_t>(payload.velDMmS));
  writeU16(out + CRC_OFFSET, crc16CcittFalse(out, CRC_OFFSET));
  return PACKET_SIZE;
}

inline bool decode(const uint8_t* in, size_t length, Payload* payload) {
  if (length != PACKET_SIZE || in[0] != MAGIC_0 || in[1] != MAGIC_1 ||
      crc16CcittFalse(in, CRC_OFFSET) != readU16(in + CRC_OFFSET)) return false;
  payload->sequence = in[2]; payload->satelliteCount = in[3];
  payload->iTowMs = readU32(in + 4);
  payload->latI7 = static_cast<int32_t>(readU32(in + 8));
  payload->lonI7 = static_cast<int32_t>(readU32(in + 12));
  payload->aglMm = static_cast<int32_t>(readU32(in + 16));
  payload->velNMmS = static_cast<int32_t>(readU32(in + 20));
  payload->velEMmS = static_cast<int32_t>(readU32(in + 24));
  payload->velDMmS = static_cast<int32_t>(readU32(in + 28));
  return true;
}

class Parser {
 public:
  uint32_t packetsOk = 0, packetsBad = 0, sequenceLost = 0;
  bool feed(uint8_t byte, Payload* payload) {
    buffer_[count_++] = byte;
    if (count_ == 1 && buffer_[0] != MAGIC_0) { count_ = 0; return false; }
    if (count_ == 2 && buffer_[1] != MAGIC_1) {
      count_ = buffer_[1] == MAGIC_0 ? 1 : 0;
      if (count_ == 1) buffer_[0] = MAGIC_0;
      return false;
    }
    if (count_ < PACKET_SIZE) return false;
    if (decode(buffer_, PACKET_SIZE, payload)) {
      ++packetsOk;
      if (haveSequence_) sequenceLost += static_cast<uint8_t>(payload->sequence - lastSequence_ - 1);
      lastSequence_ = payload->sequence; haveSequence_ = true; count_ = 0;
      return true;
    }
    ++packetsBad;
    resynchronize();
    return false;
  }
 private:
  uint8_t buffer_[PACKET_SIZE] = {};
  size_t count_ = 0;
  uint8_t lastSequence_ = 0;
  bool haveSequence_ = false;
  void resynchronize() {
    for (size_t i = 1; i < PACKET_SIZE; ++i) {
      if (buffer_[i] == MAGIC_0 &&
          (i == PACKET_SIZE - 1 || buffer_[i + 1] == MAGIC_1)) {
        memmove(buffer_, buffer_ + i, PACKET_SIZE - i);
        count_ = PACKET_SIZE - i;
        return;
      }
    }
    count_ = 0;
  }
};

inline RemoteTargetSample toSample(const Payload& payload, uint32_t nowMs) {
  RemoteTargetSample out;
  out.sequence = payload.sequence; out.senderTimeMs = payload.iTowMs;
  out.latI7 = payload.latI7; out.lonI7 = payload.lonI7;
  out.aglM = static_cast<float>(payload.aglMm) * 0.001f;
  out.velNMps = static_cast<float>(payload.velNMmS) * 0.001f;
  out.velEMps = static_cast<float>(payload.velEMmS) * 0.001f;
  out.velDMps = static_cast<float>(payload.velDMmS) * 0.001f;
  out.satelliteCount = payload.satelliteCount; out.timestampMs = nowMs;
  out.valid = payload.satelliteCount >= 4 &&
              isfinite(out.aglM) && fabsf(out.aglM) <= 30000.0f;
  return out;
}
}  // namespace remote_protocol
