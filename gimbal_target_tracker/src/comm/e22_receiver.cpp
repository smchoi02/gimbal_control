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
  delay(5);  // E22 mode-switch settling time.
}

bool E22Receiver::poll(uint32_t nowMs) {
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
