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
  // A failed SD transaction is terminal for this run.  Do not retry from the
  // real-time task; retrying the legacy driver's timeout would starve useful
  // work repeatedly.  Reset/power-cycle before attempting a new log.
  bool faulted() const { return faulted_; }
  bool usingFallbackDate() const { return usingFallbackDate_; }
  const char* filename() const { return filename_; }
  uint32_t rowsWritten() const { return rowsWritten_; }
  uint32_t writeErrors() const { return writeErrors_; }
  uint32_t rowBuildErrors() const { return rowBuildErrors_; }
  uint32_t shortWrites() const { return shortWrites_; }
  uint32_t writeFlagErrors() const { return writeFlagErrors_; }
  uint32_t logAttempts() const { return logAttempts_; }
  uint32_t flushAttempts() const { return flushAttempts_; }
  uint32_t flushErrors() const { return flushErrors_; }
  uint32_t openAttempts() const { return openAttempts_; }
  uint32_t openErrors() const { return openErrors_; }
  uint32_t filenameSearchErrors() const { return filenameSearchErrors_; }
  uint32_t fileOpenErrors() const { return fileOpenErrors_; }
  uint32_t headerWriteErrors() const { return headerWriteErrors_; }
  uint32_t headerFlushErrors() const { return headerFlushErrors_; }
  uint32_t lastOpenUs() const { return lastOpenUs_; }
  uint32_t maxOpenUs() const { return maxOpenUs_; }
  uint32_t lastFilenameSearchUs() const { return lastFilenameSearchUs_; }
  uint32_t maxFilenameSearchUs() const { return maxFilenameSearchUs_; }
  uint32_t lastFileOpenUs() const { return lastFileOpenUs_; }
  uint32_t maxFileOpenUs() const { return maxFileOpenUs_; }
  uint32_t lastHeaderWriteUs() const { return lastHeaderWriteUs_; }
  uint32_t maxHeaderWriteUs() const { return maxHeaderWriteUs_; }
  uint32_t lastHeaderFlushUs() const { return lastHeaderFlushUs_; }
  uint32_t maxHeaderFlushUs() const { return maxHeaderFlushUs_; }
  uint32_t lastLogUs() const { return lastLogUs_; }
  uint32_t maxLogUs() const { return maxLogUs_; }
  uint32_t lastRowBuildUs() const { return lastRowBuildUs_; }
  uint32_t maxRowBuildUs() const { return maxRowBuildUs_; }
  uint32_t lastFileWriteUs() const { return lastFileWriteUs_; }
  uint32_t maxFileWriteUs() const { return maxFileWriteUs_; }
  uint32_t lastRowBytes() const { return lastRowBytes_; }
  uint32_t lastWrittenBytes() const { return lastWrittenBytes_; }
  uint32_t lastFileSize() const { return lastFileSize_; }
  uint32_t lastFlushUs() const { return lastFlushUs_; }
  uint32_t maxFlushUs() const { return maxFlushUs_; }

 private:
  static constexpr size_t CSV_ROW_BUFFER_SIZE = 768;
  File file_;
  char filename_[13] = {};
  // One reusable row only; this is not a flight-duration RAM log buffer.
  char csvRowBuffer_[CSV_ROW_BUFFER_SIZE] = {};
  uint32_t lastFlushMs_ = 0;
  uint32_t rowsWritten_ = 0;
  uint32_t writeErrors_ = 0;
  uint32_t rowBuildErrors_ = 0;
  uint32_t shortWrites_ = 0;
  uint32_t writeFlagErrors_ = 0;
  uint32_t logAttempts_ = 0;
  uint32_t flushAttempts_ = 0;
  uint32_t flushErrors_ = 0;
  uint32_t openAttempts_ = 0;
  uint32_t openErrors_ = 0;
  uint32_t filenameSearchErrors_ = 0;
  uint32_t fileOpenErrors_ = 0;
  uint32_t headerWriteErrors_ = 0;
  uint32_t headerFlushErrors_ = 0;
  uint32_t lastOpenUs_ = 0;
  uint32_t maxOpenUs_ = 0;
  uint32_t lastFilenameSearchUs_ = 0;
  uint32_t maxFilenameSearchUs_ = 0;
  uint32_t lastFileOpenUs_ = 0;
  uint32_t maxFileOpenUs_ = 0;
  uint32_t lastHeaderWriteUs_ = 0;
  uint32_t maxHeaderWriteUs_ = 0;
  uint32_t lastHeaderFlushUs_ = 0;
  uint32_t maxHeaderFlushUs_ = 0;
  uint32_t lastLogUs_ = 0;
  uint32_t maxLogUs_ = 0;
  uint32_t lastRowBuildUs_ = 0;
  uint32_t maxRowBuildUs_ = 0;
  uint32_t lastFileWriteUs_ = 0;
  uint32_t maxFileWriteUs_ = 0;
  uint32_t lastRowBytes_ = 0;
  uint32_t lastWrittenBytes_ = 0;
  uint32_t lastFileSize_ = 0;
  uint32_t lastFlushUs_ = 0;
  uint32_t maxFlushUs_ = 0;
  bool cardReady_ = false;
  bool ready_ = false;
  bool faulted_ = false;
  bool usingFallbackDate_ = false;

  bool createUniqueFile(uint8_t month, uint8_t day);
  void writeHeader();
};
