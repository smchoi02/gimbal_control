// Reference transmitter for the receiver packet format.
// Replace the example values with the sender's GPS and BMP581 readings.

#include <Arduino.h>
#include "../../src/comm/remote_packet.h"

#define E22_SERIAL Serial3
constexpr uint32_t E22_UART_BAUD = 9600;

uint8_t sequenceNumber = 0;

void setup() {
  E22_SERIAL.begin(E22_UART_BAUD);
  // E22-400T22S: M0 and M1 must be LOW for Normal/transparent mode. Drive
  // them from MCU GPIOs or wire both pins to GND; neither pin may float.
}

void loop() {
  remote_protocol::Payload payload;
  payload.sequence = sequenceNumber++;
  payload.senderTimeMs = millis();
  payload.latI7 = 375000000;            // 37.5000000 deg
  payload.lonI7 = 1270000000;           // 127.0000000 deg
  payload.pressurePaX10 = 1013250;      // 101325.0 Pa
  payload.temperatureCX100 = 2500;      // 25.00 C
  payload.fixType = 3;
  payload.flags =
      remote_protocol::FLAG_GPS_VALID | remote_protocol::FLAG_BARO_VALID;

  uint8_t packet[remote_protocol::PACKET_SIZE];
  const size_t length =
      remote_protocol::encode(payload, packet, sizeof(packet));
  E22_SERIAL.write(packet, length);
  delay(100);  // 10 Hz
}
