// E22-900T22S receive validation for the real trs_test.ino transmitter.
// Standalone sketch: no project-relative includes are required.
// Expected packet: RK, 58 bytes, little-endian, CRC-16/CCITT-FALSE.
// Serial Monitor: 115200 baud. E22 UART: Serial3, 9600 baud.

#include <Arduino.h>
#include <string.h>

#define E22_SERIAL Serial3
constexpr uint32_t E22_UART_BAUD = 9600;
constexpr size_t RK_PACKET_SIZE = 58;
constexpr size_t RK_CRC_OFFSET = 56;

struct RkPacket {
  uint8_t sequence = 0;
  uint8_t fixType = 0;
  uint32_t iTowMs = 0;
  int32_t latI7 = 0;
  int32_t lonI7 = 0;
  int32_t aglMm = 0;
  int32_t velNMmS = 0;
  int32_t velEMmS = 0;
  int32_t velDMmS = 0;
  uint8_t imuFlags = 0;
  uint8_t quatAcc = 0;
  uint16_t imuAgeMs = 0;
  int16_t accelMps2X100[3] = {};
  int16_t gyroDpsX10[3] = {};
  int16_t quaternionQ14[4] = {};
};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t readU16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) |
         (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t readU32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

int16_t readI16(const uint8_t* in) {
  return static_cast<int16_t>(readU16(in));
}

void decodeRk(const uint8_t* in, RkPacket* packet) {
  packet->sequence = in[2];
  packet->fixType = in[3];
  packet->iTowMs = readU32(in + 4);
  packet->latI7 = static_cast<int32_t>(readU32(in + 8));
  packet->lonI7 = static_cast<int32_t>(readU32(in + 12));
  packet->aglMm = static_cast<int32_t>(readU32(in + 16));
  packet->velNMmS = static_cast<int32_t>(readU32(in + 20));
  packet->velEMmS = static_cast<int32_t>(readU32(in + 24));
  packet->velDMmS = static_cast<int32_t>(readU32(in + 28));
  packet->imuFlags = in[32];
  packet->quatAcc = in[33];
  packet->imuAgeMs = readU16(in + 34);
  for (uint8_t i = 0; i < 3; ++i) {
    packet->accelMps2X100[i] = readI16(in + 36 + 2 * i);
    packet->gyroDpsX10[i] = readI16(in + 42 + 2 * i);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    packet->quaternionQ14[i] = readI16(in + 48 + 2 * i);
  }
}

class RkParser {
 public:
  uint32_t packetsOk = 0;
  uint32_t packetsBad = 0;
  uint32_t sequenceLost = 0;

  bool feed(uint8_t byte, RkPacket* packet) {
    buffer_[count_++] = byte;
    if (count_ == 1 && buffer_[0] != 'R') {
      count_ = 0;
      return false;
    }
    if (count_ == 2 && buffer_[1] != 'K') {
      count_ = (buffer_[1] == 'R') ? 1 : 0;
      if (count_ == 1) buffer_[0] = 'R';
      return false;
    }
    if (count_ < RK_PACKET_SIZE) return false;

    if (crc16CcittFalse(buffer_, RK_CRC_OFFSET) ==
        readU16(buffer_ + RK_CRC_OFFSET)) {
      decodeRk(buffer_, packet);
      ++packetsOk;
      if (haveSequence_) {
        sequenceLost += static_cast<uint8_t>(packet->sequence - lastSequence_ - 1);
      }
      lastSequence_ = packet->sequence;
      haveSequence_ = true;
      count_ = 0;
      return true;
    }

    ++packetsBad;
    resynchronize();
    return false;
  }

 private:
  uint8_t buffer_[RK_PACKET_SIZE] = {};
  size_t count_ = 0;
  uint8_t lastSequence_ = 0;
  bool haveSequence_ = false;

  void resynchronize() {
    for (size_t i = 1; i < RK_PACKET_SIZE; ++i) {
      if (buffer_[i] == 'R' &&
          (i == RK_PACKET_SIZE - 1 || buffer_[i + 1] == 'K')) {
        memmove(buffer_, buffer_ + i, RK_PACKET_SIZE - i);
        count_ = RK_PACKET_SIZE - i;
        return;
      }
    }
    count_ = 0;
  }
};

RkParser parser;
uint32_t lastPacketMs = 0;
uint32_t lastStatusMs = 0;

void printPacket(const RkPacket& packet) {
  const bool valid = (packet.fixType == 3 || packet.fixType == 4) &&
                     packet.aglMm >= -30000000L &&
                     packet.aglMm <= 30000000L;
  Serial.print(F("RK seq="));
  Serial.print(packet.sequence);
  Serial.print(F(" fix="));
  Serial.print(packet.fixType);
  Serial.print(F(" itow_ms="));
  Serial.print(packet.iTowMs);
  Serial.print(F(" lat/lon="));
  Serial.print(static_cast<double>(packet.latI7) * 1.0e-7, 7);
  Serial.print('/');
  Serial.print(static_cast<double>(packet.lonI7) * 1.0e-7, 7);
  Serial.print(F(" agl_m="));
  Serial.print(static_cast<float>(packet.aglMm) * 0.001f, 3);
  Serial.print(F(" vel_n/e/d_mps="));
  Serial.print(static_cast<float>(packet.velNMmS) * 0.001f, 3);
  Serial.print('/');
  Serial.print(static_cast<float>(packet.velEMmS) * 0.001f, 3);
  Serial.print('/');
  Serial.print(static_cast<float>(packet.velDMmS) * 0.001f, 3);
  Serial.print(F(" imu_flags/quat_acc/age_ms="));
  Serial.print(packet.imuFlags, HEX);
  Serial.print('/');
  Serial.print(packet.quatAcc);
  Serial.print('/');
  Serial.print(packet.imuAgeMs);
  Serial.print(F(" valid="));
  Serial.println(valid ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  E22_SERIAL.begin(E22_UART_BAUD);
  Serial.println(F("# E22-900T22S RX test: trs_test RK packet validation"));
  Serial.println(F("# M0/M1=LOW; 900 MHz channel, NETID, air-rate, and encryption must match TX."));
}

void loop() {
  const uint32_t now = millis();
  RkPacket packet;
  while (E22_SERIAL.available()) {
    if (parser.feed(static_cast<uint8_t>(E22_SERIAL.read()), &packet)) {
      lastPacketMs = now;
      printPacket(packet);
    }
  }

  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    Serial.print(F("rx_ok/bad/lost/age_ms="));
    Serial.print(parser.packetsOk);
    Serial.print('/');
    Serial.print(parser.packetsBad);
    Serial.print('/');
    Serial.print(parser.sequenceLost);
    Serial.print('/');
    Serial.println(lastPacketMs ? now - lastPacketMs : 0xFFFFFFFFUL);
  }
}
