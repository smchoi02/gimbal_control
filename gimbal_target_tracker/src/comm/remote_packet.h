#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../common/crc16.h"
#include "../common/types.h"

namespace remote_protocol {

constexpr uint8_t MAGIC_0 = 'G';
constexpr uint8_t MAGIC_1 = 'T';
constexpr uint8_t VERSION = 1;
constexpr size_t PACKET_SIZE = 27;

// Wire layout, little-endian:
//  0 magic "GT", 2 version, 3 sequence, 4 sender_ms,
//  8 lat_i7, 12 lon_i7, 16 pressure_pa_x10,
// 20 temperature_c_x100, 22 fix_type, 23 flags,
// 24 reserved, 25 CRC16 over bytes 0..24.
struct Payload {
  uint8_t sequence = 0;
  uint32_t senderTimeMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  uint32_t pressurePaX10 = 0;
  int16_t temperatureCX100 = 0;
  uint8_t fixType = 0;
  uint8_t flags = 0;
};

constexpr uint8_t FLAG_GPS_VALID = 0x01;
constexpr uint8_t FLAG_BARO_VALID = 0x02;

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

inline uint32_t readU32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

inline size_t encode(const Payload& payload, uint8_t* out, size_t capacity) {
  if (capacity < PACKET_SIZE) return 0;
  out[0] = MAGIC_0;
  out[1] = MAGIC_1;
  out[2] = VERSION;
  out[3] = payload.sequence;
  writeU32(out + 4, payload.senderTimeMs);
  writeU32(out + 8, static_cast<uint32_t>(payload.latI7));
  writeU32(out + 12, static_cast<uint32_t>(payload.lonI7));
  writeU32(out + 16, payload.pressurePaX10);
  writeU16(out + 20, static_cast<uint16_t>(payload.temperatureCX100));
  out[22] = payload.fixType;
  out[23] = payload.flags;
  out[24] = 0;
  writeU16(out + 25, crc16CcittFalse(out, 25));
  return PACKET_SIZE;
}

inline bool decode(const uint8_t* in, size_t length, Payload* payload) {
  if (length != PACKET_SIZE || in[0] != MAGIC_0 || in[1] != MAGIC_1 ||
      in[2] != VERSION || crc16CcittFalse(in, 25) != readU16(in + 25)) {
    return false;
  }
  payload->sequence = in[3];
  payload->senderTimeMs = readU32(in + 4);
  payload->latI7 = static_cast<int32_t>(readU32(in + 8));
  payload->lonI7 = static_cast<int32_t>(readU32(in + 12));
  payload->pressurePaX10 = readU32(in + 16);
  payload->temperatureCX100 = static_cast<int16_t>(readU16(in + 20));
  payload->fixType = in[22];
  payload->flags = in[23];
  return true;
}

class Parser {
 public:
  uint32_t packetsOk = 0;
  uint32_t packetsBad = 0;
  uint32_t sequenceLost = 0;

  bool feed(uint8_t byte, Payload* payload) {
    buffer_[count_++] = byte;
    if (count_ == 1 && buffer_[0] != MAGIC_0) {
      count_ = 0;
      return false;
    }
    if (count_ == 2 && buffer_[1] != MAGIC_1) {
      count_ = (buffer_[1] == MAGIC_0) ? 1 : 0;
      if (count_ == 1) buffer_[0] = MAGIC_0;
      return false;
    }
    if (count_ < PACKET_SIZE) return false;

    if (decode(buffer_, PACKET_SIZE, payload)) {
      ++packetsOk;
      if (haveSequence_) {
        sequenceLost +=
            static_cast<uint8_t>(payload->sequence - lastSequence_ - 1);
      }
      lastSequence_ = payload->sequence;
      haveSequence_ = true;
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
  out.sequence = payload.sequence;
  out.senderTimeMs = payload.senderTimeMs;
  out.latI7 = payload.latI7;
  out.lonI7 = payload.lonI7;
  out.pressurePa = static_cast<float>(payload.pressurePaX10) * 0.1f;
  out.temperatureC = static_cast<float>(payload.temperatureCX100) * 0.01f;
  out.fixType = payload.fixType;
  out.timestampMs = nowMs;
  out.valid = (payload.flags & (FLAG_GPS_VALID | FLAG_BARO_VALID)) ==
                  (FLAG_GPS_VALID | FLAG_BARO_VALID) &&
              (payload.fixType == 3 || payload.fixType == 4) &&
              payload.pressurePaX10 >= 300000u &&
              payload.pressurePaX10 <= 1250000u;
  return out;
}

}  // namespace remote_protocol
