/*
  E22-900T22S RocketLink receiver / packet monitor -- Arduino Uno

  Wiring (transparent / normal mode):
    E22 TXD -> Uno D10 (RX)
    Uno GND -> E22 GND
    E22 M0, M1 -> GND (normal transmission mode)

  This sketch decodes the 34-byte RocketLink packet sent by
  lora_tx_only_test and used by gimbal_control/rocketLinkPoll.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

constexpr uint8_t LORA_RX_PIN = 10;  // E22 TXD -> Uno D10.
constexpr uint8_t LORA_TX_PIN = 11;  // Unused here; may remain disconnected.
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t LORA_UART_BAUD = 9600;
constexpr size_t ROCKET_PACKET_LEN = 34;
constexpr size_t ROCKET_CRC_OFFSET = 32;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN);

struct RocketPacket {
  uint8_t sequence;
  uint8_t fixType;
  uint32_t iTowMs;
  int32_t latI7;
  int32_t lonI7;
  int32_t aglMm;
  int32_t velNMmS;
  int32_t velEMmS;
  int32_t velDMmS;
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

uint16_t readU16Le(const uint8_t* source) {
  return static_cast<uint16_t>(source[0]) |
         (static_cast<uint16_t>(source[1]) << 8);
}

uint32_t readU32Le(const uint8_t* source) {
  return static_cast<uint32_t>(source[0]) |
         (static_cast<uint32_t>(source[1]) << 8) |
         (static_cast<uint32_t>(source[2]) << 16) |
         (static_cast<uint32_t>(source[3]) << 24);
}

int32_t readI32Le(const uint8_t* source) {
  return static_cast<int32_t>(readU32Le(source));
}

void decodePacket(const uint8_t raw[ROCKET_PACKET_LEN], RocketPacket* packet) {
  packet->sequence = raw[2];
  packet->fixType = raw[3];
  packet->iTowMs = readU32Le(raw + 4);
  packet->latI7 = readI32Le(raw + 8);
  packet->lonI7 = readI32Le(raw + 12);
  packet->aglMm = readI32Le(raw + 16);
  packet->velNMmS = readI32Le(raw + 20);
  packet->velEMmS = readI32Le(raw + 24);
  packet->velDMmS = readI32Le(raw + 28);
}

class RocketLinkParser {
 public:
  uint32_t packetsOk = 0;
  uint32_t packetsBad = 0;
  uint32_t sequenceLost = 0;

  bool feed(uint8_t byte, RocketPacket* packet) {
    buffer_[count_++] = byte;

    if (count_ == 1) {
      if (buffer_[0] != 'R') count_ = 0;
      return false;
    }
    if (count_ == 2) {
      if (buffer_[1] != 'K') {
        count_ = (buffer_[1] == 'R') ? 1 : 0;
        if (count_ == 1) buffer_[0] = 'R';
      }
      return false;
    }
    if (count_ < ROCKET_PACKET_LEN) return false;

    if (crc16CcittFalse(buffer_, ROCKET_CRC_OFFSET) ==
        readU16Le(buffer_ + ROCKET_CRC_OFFSET)) {
      decodePacket(buffer_, packet);
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
  uint8_t buffer_[ROCKET_PACKET_LEN] = {};
  size_t count_ = 0;
  uint8_t lastSequence_ = 0;
  bool haveSequence_ = false;

  void resynchronize() {
    for (size_t i = 1; i < ROCKET_PACKET_LEN; ++i) {
      if (buffer_[i] == 'R' &&
          (i == ROCKET_PACKET_LEN - 1 || buffer_[i + 1] == 'K')) {
        memmove(buffer_, buffer_ + i, ROCKET_PACKET_LEN - i);
        count_ = ROCKET_PACKET_LEN - i;
        return;
      }
    }
    count_ = 0;
  }
};

RocketLinkParser parser;
uint32_t lastPacketMs = 0;
uint32_t lastStatusMs = 0;

void printPacket(const RocketPacket& packet) {
  Serial.print(F("RX seq="));
  Serial.print(packet.sequence);
  Serial.print(F(" fix="));
  Serial.print(packet.fixType);
  Serial.print(F(" iTOW_ms="));
  Serial.print(packet.iTowMs);
  Serial.print(F(" lat="));
  Serial.print(static_cast<double>(packet.latI7) * 1.0e-7, 7);
  Serial.print(F(" lon="));
  Serial.print(static_cast<double>(packet.lonI7) * 1.0e-7, 7);
  Serial.print(F(" agl_m="));
  Serial.print(static_cast<float>(packet.aglMm) * 0.001f, 3);
  Serial.print(F(" vel_N/E/D_mps="));
  Serial.print(static_cast<float>(packet.velNMmS) * 0.001f, 3);
  Serial.print('/');
  Serial.print(static_cast<float>(packet.velEMmS) * 0.001f, 3);
  Serial.print('/');
  Serial.println(static_cast<float>(packet.velDMmS) * 0.001f, 3);
}

void printStatus(uint32_t nowMs) {
  Serial.print(F("status ok/bad/lost/age_ms="));
  Serial.print(parser.packetsOk);
  Serial.print('/');
  Serial.print(parser.packetsBad);
  Serial.print('/');
  Serial.print(parser.sequenceLost);
  Serial.print('/');
  Serial.println(lastPacketMs ? nowMs - lastPacketMs : 0xFFFFFFFFUL);
}

void setup() {
  Serial.begin(USB_BAUD);
  loraSerial.begin(LORA_UART_BAUD);

  Serial.println(F("E22 RocketLink RX monitor started"));
  Serial.println(F("E22 TXD -> Uno D10, UART 9600"));
  Serial.println(F("Ensure M0=M1=LOW and both E22 radio settings match."));
}

void loop() {
  const uint32_t nowMs = millis();
  RocketPacket packet;

  while (loraSerial.available()) {
    if (parser.feed(static_cast<uint8_t>(loraSerial.read()), &packet)) {
      lastPacketMs = nowMs;
      printPacket(packet);
    }
  }

  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    printStatus(nowMs);
  }
}
