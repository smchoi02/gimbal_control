// E22-900T22S raw receive test for OpenRB-150.
//
// This sketch deliberately does not expect the RK packet format. It prints
// every arbitrary byte sequence received from the E22 as both HEX and ASCII.
// A silence of PACKET_GAP_MS ends one displayed packet.
//
// Serial Monitor: 115200 baud
// E22 UART: Serial3, 9600 baud (must match the transmitter module setting)
// E22 normal transparent mode: M0=LOW, M1=LOW

#include <Arduino.h>

#define E22_SERIAL Serial3

constexpr uint32_t E22_UART_BAUD = 9600;
constexpr uint32_t PACKET_GAP_MS = 50;
constexpr size_t RX_BUFFER_SIZE = 256;

uint8_t rxBuffer[RX_BUFFER_SIZE] = {};
size_t rxCount = 0;
uint32_t lastByteMs = 0;

void printHex(uint8_t value) {
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
}

void printPacket() {
  if (rxCount == 0) return;

  Serial.print(F("RX t_ms="));
  Serial.print(lastByteMs);
  Serial.print(F(" len="));
  Serial.print(rxCount);
  Serial.print(F(" hex="));
  for (size_t i = 0; i < rxCount; ++i) {
    printHex(rxBuffer[i]);
    if (i + 1 < rxCount) Serial.print(' ');
  }
  Serial.print(F(" ascii=\""));
  for (size_t i = 0; i < rxCount; ++i) {
    const uint8_t value = rxBuffer[i];
    Serial.print(value >= 32 && value <= 126 ? static_cast<char>(value) : '.');
  }
  Serial.println('"');
  rxCount = 0;
}

void setup() {
  Serial.begin(115200);
  E22_SERIAL.begin(E22_UART_BAUD);
  Serial.println(F("E22 raw RX ready: Serial3 @ 9600"));
  Serial.println(F("Send any bytes from the transmitter; RX shows HEX + ASCII."));
}

void loop() {
  const uint32_t nowMs = millis();
  while (E22_SERIAL.available()) {
    if (rxCount == RX_BUFFER_SIZE) {
      // Do not discard data silently when a sender transmits a long packet.
      printPacket();
    }
    rxBuffer[rxCount++] = static_cast<uint8_t>(E22_SERIAL.read());
    lastByteMs = millis();
  }

  if (rxCount > 0 &&
      static_cast<uint32_t>(nowMs - lastByteMs) >= PACKET_GAP_MS) {
    printPacket();
  }
}
