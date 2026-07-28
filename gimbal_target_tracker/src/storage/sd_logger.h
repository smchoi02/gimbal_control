#pragma once

#include <Arduino.h>
#include <SD.h>

#include "../common/types.h"

class SdLogger {
 public:
  bool begin(uint8_t chipSelectPin);
  void log(uint32_t nowMs, TrackMode mode, const AttitudeSample& attitude,
           const BarometerSample& localBaro, const GpsFix& localGps,
           const RemoteTargetSample& remote, const RelativeTarget& relative,
           const GimbalState& gimbal);
  void service(uint32_t nowMs);

  bool ready() const { return ready_; }
  const char* filename() const { return filename_; }
  uint32_t rowsWritten() const { return rowsWritten_; }
  uint32_t writeErrors() const { return writeErrors_; }

 private:
  File file_;
  char filename_[13] = {};
  uint32_t lastFlushMs_ = 0;
  uint32_t rowsWritten_ = 0;
  uint32_t writeErrors_ = 0;
  bool ready_ = false;

  bool createUniqueFile();
  void writeHeader();
};
