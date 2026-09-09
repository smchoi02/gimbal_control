// Standalone MAX-M10S I2C read test for OpenRB-150.
// No project-relative includes are required.
// Serial Monitor: 115200 baud. MAX-M10S I2C address: 0x42.

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t MAX_M10S_ADDRESS = 0x42;

struct GpsFix {
  uint32_t iTowMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  int32_t hMslMm = 0;
  uint8_t fixType = 0;
  uint8_t numSv = 0;
  uint32_t timestampMs = 0;
  bool valid = false;
};

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t readI32(const uint8_t* data) {
  return static_cast<int32_t>(readU32(data));
}

class NavPvtParser {
 public:
  bool feed(uint8_t byte, GpsFix* fix) {
    switch (state_) {
      case SYNC_1:
        if (byte == 0xB5) state_ = SYNC_2;
        break;
      case SYNC_2:
        state_ = (byte == 0x62) ? MESSAGE_CLASS : SYNC_1;
        ckA_ = ckB_ = 0;
        break;
      case MESSAGE_CLASS:
        messageClass_ = byte;
        checksum(byte);
        state_ = MESSAGE_ID;
        break;
      case MESSAGE_ID:
        messageId_ = byte;
        checksum(byte);
        state_ = LENGTH_1;
        break;
      case LENGTH_1:
        length_ = byte;
        checksum(byte);
        state_ = LENGTH_2;
        break;
      case LENGTH_2:
        length_ |= static_cast<uint16_t>(byte) << 8;
        checksum(byte);
        index_ = 0;
        state_ = length_ > sizeof(payload_) ? SYNC_1
                                             : (length_ ? PAYLOAD : CHECKSUM_A);
        break;
      case PAYLOAD:
        payload_[index_++] = byte;
        checksum(byte);
        if (index_ >= length_) state_ = CHECKSUM_A;
        break;
      case CHECKSUM_A:
        receivedCkA_ = byte;
        state_ = CHECKSUM_B;
        break;
      case CHECKSUM_B: {
        const bool checksumOk = receivedCkA_ == ckA_ && byte == ckB_;
        state_ = SYNC_1;
        if (!checksumOk || messageClass_ != 0x01 || messageId_ != 0x07 ||
            length_ != 92) {
          return false;
        }
        fix->iTowMs = readU32(payload_ + 0);
        fix->fixType = payload_[20];
        fix->numSv = payload_[23];
        fix->lonI7 = readI32(payload_ + 24);
        fix->latI7 = readI32(payload_ + 28);
        fix->hMslMm = readI32(payload_ + 36);
        fix->valid = (payload_[21] & 0x01u) != 0 &&
                     (fix->fixType == 3 || fix->fixType == 4);
        return true;
      }
    }
    return false;
  }

 private:
  enum State : uint8_t {
    SYNC_1, SYNC_2, MESSAGE_CLASS, MESSAGE_ID, LENGTH_1, LENGTH_2,
    PAYLOAD, CHECKSUM_A, CHECKSUM_B
  } state_ = SYNC_1;
  uint8_t messageClass_ = 0;
  uint8_t messageId_ = 0;
  uint16_t length_ = 0;
  uint16_t index_ = 0;
  uint8_t ckA_ = 0;
  uint8_t ckB_ = 0;
  uint8_t receivedCkA_ = 0;
  uint8_t payload_[92] = {};

  void checksum(uint8_t byte) {
    ckA_ = static_cast<uint8_t>(ckA_ + byte);
    ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
  }
};

NavPvtParser parser;
GpsFix fix;
bool present = false;
uint32_t lastPrintMs = 0;

uint16_t bytesAvailable() {
  Wire.beginTransmission(MAX_M10S_ADDRESS);
  Wire.write(0xFD);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(static_cast<int>(MAX_M10S_ADDRESS), 2) != 2) return 0;
  const uint16_t count =
      (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  return count == 0xFFFFu ? 0 : count;
}

bool sendValset(const uint8_t* body, size_t bodyLength) {
  uint8_t frame[32];
  const size_t payloadLength = 4 + bodyLength;
  const size_t frameLength = payloadLength + 8;
  if (frameLength > sizeof(frame)) return false;

  frame[0] = 0xB5; frame[1] = 0x62; frame[2] = 0x06; frame[3] = 0x8A;
  frame[4] = static_cast<uint8_t>(payloadLength);
  frame[5] = static_cast<uint8_t>(payloadLength >> 8);
  frame[6] = 0; frame[7] = 0x01; frame[8] = 0; frame[9] = 0;
  for (size_t i = 0; i < bodyLength; ++i) frame[10 + i] = body[i];

  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < frameLength - 2; ++i) {
    ckA = static_cast<uint8_t>(ckA + frame[i]);
    ckB = static_cast<uint8_t>(ckB + ckA);
  }
  frame[frameLength - 2] = ckA;
  frame[frameLength - 1] = ckB;
  Wire.beginTransmission(MAX_M10S_ADDRESS);
  Wire.write(frame, frameLength);
  return Wire.endTransmission() == 0;
}

bool configure10Hz() {
  // CFG-MSGOUT-UBX_NAV_PVT_I2C=1, CFG-I2COUTPROT-NMEA=false.
  const uint8_t outputConfig[] = {
      0x06, 0x00, 0x91, 0x20, 0x01, 0x02, 0x00, 0x72, 0x10, 0x00};
  // CFG-RATE-MEAS=100 ms, CFG-RATE-NAV=1.
  const uint8_t rateConfig[] = {
      0x01, 0x00, 0x21, 0x30, 100, 0, 0x02, 0x00, 0x21, 0x30, 1, 0};
  return sendValset(outputConfig, sizeof(outputConfig)) &&
         sendValset(rateConfig, sizeof(rateConfig));
}

void pollGps(uint32_t nowMs) {
  uint16_t available = bytesAvailable();
  while (available > 0) {
    const uint8_t chunk = available > 32 ? 32 : static_cast<uint8_t>(available);
    Wire.beginTransmission(MAX_M10S_ADDRESS);
    Wire.write(0xFF);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(static_cast<int>(MAX_M10S_ADDRESS),
                         static_cast<int>(chunk)) <= 0) return;
    while (Wire.available()) {
      if (parser.feed(static_cast<uint8_t>(Wire.read()), &fix)) {
        fix.timestampMs = nowMs;
      }
    }
    available = available > chunk ? available - chunk : 0;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MAX_M10S_ADDRESS);
  present = Wire.endTransmission() == 0;

  Serial.println(F("# MAX-M10S standalone read test"));
  if (!present) {
    Serial.println(F("# FAIL: no device at I2C 0x42"));
  } else {
    Serial.println(configure10Hz() ? F("# OK: NAV-PVT 10Hz requested")
                                   : F("# WARN: device found, configuration failed"));
  }
}

void loop() {
  const uint32_t now = millis();
  if (present) pollGps(now);
  if (now - lastPrintMs < 500) return;
  lastPrintMs = now;

  Serial.print(F("present/valid="));
  Serial.print(present ? 1 : 0);
  Serial.print('/');
  Serial.print(fix.valid ? 1 : 0);
  Serial.print(F(" fix/sv="));
  Serial.print(fix.fixType);
  Serial.print('/');
  Serial.print(fix.numSv);
  Serial.print(F(" lat/lon="));
  Serial.print(static_cast<double>(fix.latI7) * 1.0e-7, 7);
  Serial.print('/');
  Serial.print(static_cast<double>(fix.lonI7) * 1.0e-7, 7);
  Serial.print(F(" hMSL_m="));
  Serial.println(static_cast<float>(fix.hMslMm) * 0.001f, 2);
}
