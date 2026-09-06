#pragma once

#include <Arduino.h>
#include <SD.h>

#include "../common/types.h"

class SdLogger {
 public:
  bool begin(uint8_t chipSelectPin);
  bool openForDate(uint8_t month, uint8_t day);
  bool reopenWithGpsDate(uint8_t month, uint8_t day);
  void log(uint32_t nowMs, TrackMode mode, TrackingBlocker blocker,
           const AttitudeSample& attitude,
           const BarometerSample& localBaro, const GpsFix& localGps,
           const RemoteTargetSample& remote, const RelativeTarget& relative,
           const GimbalState& gimbal, float initialBaroBiasM,
           bool baroBiasReady);
  void service(uint32_t nowMs);

  bool ready() const { return ready_; }
  bool cardReady() const { return cardReady_; }
  bool usingFallbackDate() const { return usingFallbackDate_; }
  const char* filename() const { return filename_; }
  uint32_t rowsWritten() const { return rowsWritten_; }
  uint32_t writeErrors() const { return writeErrors_; }

 private:
  File file_;
  char filename_[13] = {};
  uint32_t lastFlushMs_ = 0;
  uint32_t rowsWritten_ = 0;
  uint32_t writeErrors_ = 0;
  bool cardReady_ = false;
  bool ready_ = false;
  bool usingFallbackDate_ = false;

  bool createUniqueFile(uint8_t month, uint8_t day);
  void writeHeader();
};
