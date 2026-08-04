#include "e22_receiver.h"

void E22Receiver::begin(int8_t m0Pin, int8_t m1Pin, int8_t auxPin) {
  if (m0Pin >= 0) {
    pinMode(m0Pin, OUTPUT);
    digitalWrite(m0Pin, LOW);
  }
  if (m1Pin >= 0) {
    pinMode(m1Pin, OUTPUT);
    digitalWrite(m1Pin, LOW);
  }
  auxPin_ = auxPin;
  if (auxPin_ >= 0) pinMode(auxPin_, INPUT);
  // The E22-400T22S needs at least 1 ms after an M0/M1 mode transition.
  delay(5);
}

bool E22Receiver::poll(uint32_t nowMs) {
  // With AUX wired, defer parsing until the module has completed startup or
  // the current UART/radio operation. Bytes remain in the UART RX buffer.
  if (!moduleReady()) return false;

  bool received = false;
  remote_protocol::Payload payload;
  while (serial_.available()) {
    if (parser_.feed(static_cast<uint8_t>(serial_.read()), &payload)) {
      sample_ = remote_protocol::toSample(payload, nowMs);
      received = true;
    }
  }
  return received;
}

bool E22Receiver::moduleReady() const {
  return auxPin_ < 0 || digitalRead(auxPin_) == HIGH;
}
