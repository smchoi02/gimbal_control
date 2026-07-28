#pragma once

#include <stdint.h>

#include "../comm/remote_packet.h"
#include "../common/types.h"

class VirtualRemoteSource {
 public:
  enum class Mode : uint8_t {
    OFF = 0,
    REMOTE_AROUND_REAL_LOCAL = 1,
    FULL_BENCH = 2,
    FIXED_ABSOLUTE_TARGET = 3
  };

  void setMode(Mode mode, uint32_t nowMs);
  void restart(uint32_t nowMs);
  bool update(uint32_t nowMs, const GpsFix& realLocalGps,
              const BarometerSample& realLocalBarometer);

  Mode mode() const { return mode_; }
  bool enabled() const { return mode_ != Mode::OFF; }
  bool usesVirtualLocal() const { return mode_ == Mode::FULL_BENCH; }
  const RemoteTargetSample& remote() const { return remote_; }
  const GpsFix& virtualLocalGps() const { return virtualLocalGps_; }
  const BarometerSample& virtualLocalBarometer() const {
    return virtualLocalBarometer_;
  }
  const remote_protocol::Parser& parser() const { return parser_; }

 private:
  Mode mode_ = Mode::OFF;
  remote_protocol::Parser parser_;
  RemoteTargetSample remote_;
  GpsFix virtualLocalGps_;
  BarometerSample virtualLocalBarometer_;
  uint32_t startMs_ = 0;
  uint32_t lastPacketMs_ = 0;
  uint8_t sequence_ = 0;

  void updateVirtualLocal(uint32_t nowMs);
  bool createPacket(uint32_t nowMs, const GpsFix& localGps,
                    const BarometerSample& localBarometer);
  bool createFixedTargetPacket(uint32_t nowMs, const GpsFix& localGps,
                               const BarometerSample& localBarometer);
};
