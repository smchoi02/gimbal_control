#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../common/types.h"

class UbxNavPvtParser {
 public:
  bool feed(uint8_t byte, GpsFix* fix);

 private:
  enum State : uint8_t {
    SYNC1,
    SYNC2,
    CLASS,
    ID,
    LEN1,
    LEN2,
    PAYLOAD,
    CK_A,
    CK_B
  };

  State state_ = SYNC1;
  uint8_t messageClass_ = 0;
  uint8_t messageId_ = 0;
  uint16_t length_ = 0;
  uint16_t index_ = 0;
  uint8_t ckA_ = 0;
  uint8_t ckB_ = 0;
  uint8_t receivedCkA_ = 0;
  uint8_t payload_[92] = {};

  void checksum(uint8_t byte);
  bool finish(GpsFix* fix);
};

class MaxM10sSensor {
 public:
  explicit MaxM10sSensor(TwoWire& wire) : wire_(wire) {}

  bool begin(uint8_t address);
  bool configure10Hz();
  bool poll(uint32_t nowMs);

  bool present() const { return present_; }
  const GpsFix& fix() const { return fix_; }

 private:
  TwoWire& wire_;
  UbxNavPvtParser parser_;
  GpsFix fix_;
  uint8_t address_ = 0;
  bool present_ = false;

  uint16_t bytesAvailable();
  bool sendValset(const uint8_t* body, size_t bodyLength);
};
