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

  // Send every CSV row to this stream as well as to the card. Ground-station
  // tools read the same 46 columns over USB, so no separate wire format is
  // needed. Mirroring works even when no SD card is fitted.
  void setMirror(Print* mirror) { mirror_ = mirror; }

  bool ready() const { return ready_; }
  const char* filename() const { return filename_; }
  uint32_t rowsWritten() const { return rowsWritten_; }
  uint32_t writeErrors() const { return writeErrors_; }

 private:
  File file_;
  Print* mirror_ = nullptr;
  bool mirrorHeaderSent_ = false;
  char filename_[13] = {};
  uint32_t lastFlushMs_ = 0;
  uint32_t rowsWritten_ = 0;
  uint32_t writeErrors_ = 0;
  bool ready_ = false;

  bool createUniqueFile();
  void writeHeader();
  void emitHeader(Print& out);

  // One value to both sinks. The card is skipped when it is not ready so the
  // mirror still works on a board with no SD card installed.
  template <typename T>
  void emit(const T& value) {
    if (ready_) file_.print(value);
    if (mirror_) mirror_->print(value);
  }
  template <typename T>
  void emit(const T& value, int digits) {
    if (ready_) file_.print(value, digits);
    if (mirror_) mirror_->print(value, digits);
  }
  void emitChar(char c) {
    if (ready_) file_.print(c);
    if (mirror_) mirror_->print(c);
  }
  void emitNewline() {
    if (ready_) file_.println();
    if (mirror_) mirror_->println();
  }
};
