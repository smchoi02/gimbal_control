#include "max_m10s_sensor.h"

namespace {

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t readI32(const uint8_t* data) {
  return static_cast<int32_t>(readU32(data));
}

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

}  // namespace

void UbxNavPvtParser::checksum(uint8_t byte) {
  ckA_ = static_cast<uint8_t>(ckA_ + byte);
  ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
}

bool UbxNavPvtParser::feed(uint8_t byte, GpsFix* fix) {
  switch (state_) {
    case SYNC1:
      if (byte == 0xB5) state_ = SYNC2;
      break;
    case SYNC2:
      state_ = (byte == 0x62) ? CLASS : SYNC1;
      ckA_ = ckB_ = 0;
      break;
    case CLASS:
      messageClass_ = byte;
      checksum(byte);
      state_ = ID;
      break;
    case ID:
      messageId_ = byte;
      checksum(byte);
      state_ = LEN1;
      break;
    case LEN1:
      length_ = byte;
      checksum(byte);
      state_ = LEN2;
      break;
    case LEN2:
      length_ |= static_cast<uint16_t>(byte) << 8;
      checksum(byte);
      index_ = 0;
      if (length_ > sizeof(payload_)) {
        state_ = SYNC1;
      } else {
        state_ = length_ ? PAYLOAD : CK_A;
      }
      break;
    case PAYLOAD:
      payload_[index_++] = byte;
      checksum(byte);
      if (index_ >= length_) state_ = CK_A;
      break;
    case CK_A:
      receivedCkA_ = byte;
      state_ = CK_B;
      break;
    case CK_B: {
      const bool checksumOk = receivedCkA_ == ckA_ && byte == ckB_;
      const bool complete = checksumOk && finish(fix);
      state_ = SYNC1;
      return complete;
    }
  }
  return false;
}

bool UbxNavPvtParser::finish(GpsFix* fix) {
  if (messageClass_ != 0x01 || messageId_ != 0x07 || length_ != 92) {
    return false;
  }
  fix->iTowMs = readU32(payload_ + 0);
  fix->fixType = payload_[20];
  fix->numSv = payload_[23];
  fix->lonI7 = readI32(payload_ + 24);
  fix->latI7 = readI32(payload_ + 28);
  fix->hMslMm = readI32(payload_ + 36);
  fix->velNMmS = readI32(payload_ + 48);
  fix->velEMmS = readI32(payload_ + 52);
  fix->velDMmS = readI32(payload_ + 56);
  const bool gnssFixOk = (payload_[21] & 0x01u) != 0;
  fix->valid = gnssFixOk && (fix->fixType == 3 || fix->fixType == 4);
  return true;
}

bool MaxM10sSensor::begin(uint8_t address) {
  address_ = address;
  wire_.beginTransmission(address_);
  present_ = wire_.endTransmission() == 0;
  if (present_) configure10Hz();
  return present_;
}

bool MaxM10sSensor::configure10Hz() {
  // Each CFG-VALSET frame is kept below the smallest common Wire TX buffer.
  // CFG-MSGOUT-UBX_NAV_PVT_I2C=1, CFG-I2COUTPROT-NMEA=false
  uint8_t outputConfig[10];
  writeU32(outputConfig + 0, 0x20910006u);
  outputConfig[4] = 1;
  writeU32(outputConfig + 5, 0x10720002u);
  outputConfig[9] = 0;

  // CFG-RATE-MEAS=100 ms, CFG-RATE-NAV=1
  uint8_t rateConfig[12];
  writeU32(rateConfig + 0, 0x30210001u);
  rateConfig[4] = 100;
  rateConfig[5] = 0;
  writeU32(rateConfig + 6, 0x30210002u);
  rateConfig[10] = 1;
  rateConfig[11] = 0;

  return sendValset(outputConfig, sizeof(outputConfig)) &&
         sendValset(rateConfig, sizeof(rateConfig));
}

bool MaxM10sSensor::poll(uint32_t nowMs) {
  if (!present_) return false;
  uint16_t available = bytesAvailable();
  bool gotFix = false;

  while (available > 0) {
    const uint8_t chunk = available > 32 ? 32 : static_cast<uint8_t>(available);
    wire_.beginTransmission(address_);
    wire_.write(0xFF);
    if (wire_.endTransmission(false) != 0) break;
    const int received =
        wire_.requestFrom(static_cast<int>(address_), static_cast<int>(chunk));
    if (received <= 0) break;
    while (wire_.available()) {
      if (parser_.feed(static_cast<uint8_t>(wire_.read()), &fix_)) {
        fix_.timestampMs = nowMs;
        gotFix = true;
      }
    }
    available = available > chunk ? available - chunk : 0;
  }
  return gotFix;
}

uint16_t MaxM10sSensor::bytesAvailable() {
  wire_.beginTransmission(address_);
  wire_.write(0xFD);
  if (wire_.endTransmission(false) != 0) return 0;
  if (wire_.requestFrom(static_cast<int>(address_), 2) != 2) return 0;
  const uint16_t count =
      (static_cast<uint16_t>(wire_.read()) << 8) | wire_.read();
  return count == 0xFFFFu ? 0 : count;
}

bool MaxM10sSensor::sendValset(const uint8_t* body, size_t bodyLength) {
  uint8_t frame[32];
  const size_t payloadLength = 4 + bodyLength;
  const size_t frameLength = payloadLength + 8;
  if (frameLength > sizeof(frame)) return false;

  frame[0] = 0xB5;
  frame[1] = 0x62;
  frame[2] = 0x06;
  frame[3] = 0x8A;
  frame[4] = static_cast<uint8_t>(payloadLength);
  frame[5] = static_cast<uint8_t>(payloadLength >> 8);
  frame[6] = 0;     // VALSET version
  frame[7] = 0x01;  // RAM layer
  frame[8] = 0;
  frame[9] = 0;
  memcpy(frame + 10, body, bodyLength);

  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < frameLength - 2; ++i) {
    ckA = static_cast<uint8_t>(ckA + frame[i]);
    ckB = static_cast<uint8_t>(ckB + ckA);
  }
  frame[frameLength - 2] = ckA;
  frame[frameLength - 1] = ckB;

  wire_.beginTransmission(address_);
  wire_.write(frame, frameLength);
  return wire_.endTransmission() == 0;
}
