#pragma once

#include <Arduino.h>

#include "../common/types.h"
#include "remote_packet.h"

class E22Receiver {
 public:
  explicit E22Receiver(Stream& serial) : serial_(serial) {}

  void begin(int8_t m0Pin, int8_t m1Pin, int8_t auxPin);
  bool poll(uint32_t nowMs);

  const RemoteTargetSample& sample() const { return sample_; }
  const remote_protocol::Parser& parser() const { return parser_; }
  bool moduleReady() const;

 private:
  Stream& serial_;
  remote_protocol::Parser parser_;
  RemoteTargetSample sample_;
  int8_t auxPin_ = -1;
};
