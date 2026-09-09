#include <Arduino.h>
#include <SD.h>

namespace {
constexpr uint8_t SD_CS_PIN = 4;
// P0909000.CSV measurement: its longest real CSV record was 312 bytes.
constexpr size_t RECORD_SIZE = 312;
constexpr uint16_t RECORD_COUNT = 128;       // 12.8 seconds at 10 Hz
constexpr uint8_t FLUSH_EVERY_RECORDS = 10;  // one second
constexpr uint32_t RECORD_PERIOD_US = 100000;

uint8_t writeRecord[RECORD_SIZE];
uint8_t readRecord[RECORD_SIZE];

uint32_t elapsedUs(uint32_t startedUs) {
  return static_cast<uint32_t>(micros() - startedUs);
}

bool chooseFilename(char filename[13]) {
  // 8.3: U + seven digits + .BIN.  Never overwrites a prior result.
  for (uint32_t candidate = 0; candidate < 10000000UL; ++candidate) {
    snprintf(filename, 13, "U%07lu.BIN", candidate);
    if (!SD.exists(filename)) return true;
  }
  return false;
}

void fillPattern(uint16_t recordIndex, uint8_t* destination) {
  for (size_t i = 0; i < RECORD_SIZE; ++i) {
    destination[i] = static_cast<uint8_t>((i * 31U) ^ (recordIndex * 73U));
  }
  destination[0] = 0xA5;
  destination[1] = static_cast<uint8_t>(recordIndex);
  destination[2] = static_cast<uint8_t>(recordIndex >> 8);
  destination[RECORD_SIZE - 2] = 0x5A;
  destination[RECORD_SIZE - 1] = static_cast<uint8_t>(~recordIndex);
}

bool verifyRecord(uint16_t recordIndex) {
  fillPattern(recordIndex, writeRecord);
  for (size_t i = 0; i < RECORD_SIZE; ++i) {
    if (readRecord[i] != writeRecord[i]) return false;
  }
  return true;
}

void runUnalignedTest() {
  char filename[13] = {};
  const uint32_t scanStartedUs = micros();
  if (!chooseFilename(filename)) {
    Serial.println(F("unaligned result=no-free-8.3-name"));
    return;
  }
  const uint32_t scanUs = elapsedUs(scanStartedUs);

  const uint32_t openStartedUs = micros();
  File file = SD.open(filename, FILE_WRITE);
  const uint32_t openUs = elapsedUs(openStartedUs);
  if (!file) {
    Serial.print(F("unaligned file=")); Serial.print(filename);
    Serial.print(F(" scan_us=")); Serial.print(scanUs);
    Serial.print(F(" open_us=")); Serial.print(openUs);
    Serial.println(F(" result=open-fail"));
    return;
  }

  Serial.print(F("unaligned file=")); Serial.print(filename);
  Serial.print(F(" record_bytes=")); Serial.print(RECORD_SIZE);
  Serial.print(F(" records=")); Serial.print(RECORD_COUNT);
  Serial.print(F(" period_us=")); Serial.print(RECORD_PERIOD_US);
  Serial.print(F(" scan_us=")); Serial.print(scanUs);
  Serial.print(F(" open_us=")); Serial.println(openUs);

  uint32_t maxWriteUs = 0;
  uint32_t maxFlushUs = 0;
  uint16_t completed = 0;
  bool failed = false;
  uint32_t nextRecordUs = micros();

  for (uint16_t record = 0; record < RECORD_COUNT; ++record) {
    while (static_cast<int32_t>(micros() - nextRecordUs) < 0) { delay(1); }
    nextRecordUs += RECORD_PERIOD_US;

    fillPattern(record, writeRecord);
    const uint32_t writeStartedUs = micros();
    const size_t written = file.write(writeRecord, RECORD_SIZE);
    const uint32_t writeUs = elapsedUs(writeStartedUs);
    if (writeUs > maxWriteUs) maxWriteUs = writeUs;
    const bool writeError = file.getWriteError();
    if (writeError) file.clearWriteError();

    const bool flushNow = ((record + 1U) % FLUSH_EVERY_RECORDS) == 0U;
    uint32_t flushUs = 0;
    bool flushError = false;
    if (flushNow) {
      const uint32_t flushStartedUs = micros();
      file.flush();
      flushUs = elapsedUs(flushStartedUs);
      if (flushUs > maxFlushUs) maxFlushUs = flushUs;
      flushError = file.getWriteError();
      if (flushError) file.clearWriteError();
    }

    if (record == 0 || flushNow || written != RECORD_SIZE || writeError || flushError) {
      Serial.print(F("unaligned r=")); Serial.print(record);
      Serial.print(F(" size=")); Serial.print((record + 1UL) * RECORD_SIZE);
      Serial.print(F(" w_us=")); Serial.print(writeUs);
      if (flushNow) { Serial.print(F(" f_us=")); Serial.print(flushUs); }
      Serial.print(F(" written=")); Serial.print(written);
      Serial.print(F(" we=")); Serial.print(writeError ? 1 : 0);
      Serial.print(F(" fe=")); Serial.print(flushError ? 1 : 0);
      Serial.println((written == RECORD_SIZE && !writeError && !flushError) ? F(" result=PASS") : F(" result=FAIL"));
    }

    if (written != RECORD_SIZE || writeError || flushError) {
      failed = true;
      break;
    }
    ++completed;
  }

  const uint32_t closeStartedUs = micros();
  file.close();
  const uint32_t closeUs = elapsedUs(closeStartedUs);

  bool verified = !failed;
  uint16_t verifiedRecords = 0;
  if (!failed) {
    File readFile = SD.open(filename, FILE_READ);
    if (!readFile) {
      verified = false;
    } else {
      for (uint16_t record = 0; record < completed; ++record) {
        if (readFile.read(readRecord, RECORD_SIZE) != RECORD_SIZE || !verifyRecord(record)) {
          verified = false;
          break;
        }
        ++verifiedRecords;
      }
      readFile.close();
    }
  }

  Serial.print(F("unaligned summary records=")); Serial.print(completed);
  Serial.print(F(" bytes=")); Serial.print(completed * RECORD_SIZE);
  Serial.print(F(" max_write_us=")); Serial.print(maxWriteUs);
  Serial.print(F(" max_flush_us=")); Serial.print(maxFlushUs);
  Serial.print(F(" close_us=")); Serial.print(closeUs);
  Serial.print(F(" verified_records=")); Serial.print(verifiedRecords);
  Serial.print(F(" result="));
  Serial.println(!failed && verified && completed == RECORD_COUNT ? F("PASS") : F("FAIL"));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  // Do not wait indefinitely for the USB CDC monitor.  That can make a
  // successfully uploaded one-shot diagnostic look completely silent.
  delay(1500);
  Serial.println(F("unaligned serial=ready"));
  Serial.println(F("unaligned calling SD.begin"));
  const uint32_t beginStartedUs = micros();
  const bool sdOk = SD.begin(SD_CS_PIN);
  Serial.print(F("unaligned begin=")); Serial.print(sdOk ? F("ok") : F("fail"));
  Serial.print(F(" begin_us=")); Serial.println(elapsedUs(beginStartedUs));
  if (!sdOk) for (;;) {}
  runUnalignedTest();
}

void loop() {
  // Keep a visible indication after the one-shot test has completed, even if
  // the serial monitor was opened after reset.
  static uint32_t lastHeartbeatMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastHeartbeatMs >= 1000U) {
    lastHeartbeatMs = nowMs;
    Serial.println(F("unaligned diagnostic=complete reset-to-run-again"));
  }
}
