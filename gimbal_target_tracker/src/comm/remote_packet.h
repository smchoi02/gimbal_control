#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../common/crc16.h"
#include "../common/types.h"
#include "../config/system_config.h"

// final_test RK wire format, little-endian, exactly 34 bytes total.
//  0 "RK", 2 sequence, 3 UBX NAV-PVT fixType, 4 iTOW ms,
//  8 latitude i7 deg, 12 longitude i7 deg, 16 rocket barometric AGL mm,
//  20/24/28 NAV-PVT velocity North/East/Down mm/s,
//  32 CRC-16/CCITT-FALSE over bytes 0..31.
// iTOW is recorded for diagnostics only; receive freshness uses local millis.
namespace remote_protocol {

constexpr uint8_t MAGIC_0 = 'R';
constexpr uint8_t MAGIC_1 = 'K';
constexpr size_t PACKET_SIZE = 34;
constexpr size_t CRC_OFFSET = 32;

struct Payload {
  uint8_t sequence = 0;
  uint8_t fixType = 0;
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
  out[2] = payload.sequence; out[3] = payload.fixType;
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
  payload->sequence = in[2]; payload->fixType = in[3];
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
  uint32_t sequenceDuplicate = 0, sequenceOutOfOrder = 0;

  // True only for an in-order, new sequence number. Duplicates and late
  // packets remain CRC-valid but must not refresh the remote packet age.
  bool lastFrameIsNew() const { return lastFrameIsNew_; }

  bool feed(uint8_t byte, Payload* payload) {
    if (count_ >= PACKET_SIZE) resynchronize();
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
      lastFrameIsNew_ = classifySequence(payload->sequence);
      count_ = 0;
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
  bool lastFrameIsNew_ = false;

  bool classifySequence(uint8_t sequence) {
    if (!haveSequence_) {
      lastSequence_ = sequence;
      haveSequence_ = true;
      return true;
    }
    const uint8_t delta = static_cast<uint8_t>(sequence - lastSequence_);
    if (delta == 0) {
      ++sequenceDuplicate;
      return false;
    }
    // A forward jump of 1..127 is accepted, including natural uint8 wrap.
    // A larger delta is a delayed/out-of-order older frame and is ignored.
    if (delta <= 127) {
      sequenceLost += static_cast<uint8_t>(delta - 1);
      lastSequence_ = sequence;
      return true;
    }
    ++sequenceOutOfOrder;
    return false;
  }
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
  out.sequence = payload.sequence; out.senderITowMs = payload.iTowMs;
  out.latI7 = payload.latI7; out.lonI7 = payload.lonI7;
  out.aglM = static_cast<float>(payload.aglMm) * 0.001f;
  out.velNMps = static_cast<float>(payload.velNMmS) * 0.001f;
  out.velEMps = static_cast<float>(payload.velEMmS) * 0.001f;
  out.velDMps = static_cast<float>(payload.velDMmS) * 0.001f;
  out.fixType = payload.fixType;
  out.timestampMs = nowMs;
  const bool coordinatesInRange =
      out.latI7 >= -900000000 && out.latI7 <= 900000000 &&
      out.lonI7 >= -1800000000 && out.lonI7 <= 1800000000;
  const bool velocityInRange =
      isfinite(out.velNMps) && isfinite(out.velEMps) && isfinite(out.velDMps) &&
      fabsf(out.velNMps) <= cfg::MAX_ABS_VELOCITY_MPS &&
      fabsf(out.velEMps) <= cfg::MAX_ABS_VELOCITY_MPS &&
      fabsf(out.velDMps) <= cfg::MAX_ABS_VELOCITY_MPS;
  // final_test publishes a last-good NAV-PVT only for a valid 3D or combined
  // fix. Its byte 3 is that fixType, so it must never be treated as numSV.
  out.valid = (out.fixType == 3 || out.fixType == 4) && coordinatesInRange &&
              isfinite(out.aglM) && fabsf(out.aglM) <= cfg::MAX_ABS_RELATIVE_ALT_M &&
              velocityInRange;
  return out;
}
}  // namespace remote_protocol
