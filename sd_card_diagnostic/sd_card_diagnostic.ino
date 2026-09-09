#include <Arduino.h>
#include <SD.h>

namespace {
constexpr uint8_t SD_CS_PIN = 4;
// This must not be uint8_t: 512 would wrap to zero and silently turn the
// test into a zero-byte write/read.
constexpr size_t BLOCK_SIZE = 512;
constexpr uint8_t MAX_TESTS = 10;
constexpr uint32_t TEST_INTERVAL_MS = 2000;

uint8_t writeBlock[BLOCK_SIZE];
uint8_t readBlock[BLOCK_SIZE];
uint8_t testIndex = 0;
uint32_t lastTestMs = 0;
uint32_t passed = 0;
uint32_t failed = 0;

uint32_t elapsedUs(uint32_t startedUs) {
  return static_cast<uint32_t>(micros() - startedUs);
}

bool chooseFilename(char filename[13]) {
  // 8.3 format: S + seven digits + .BIN.  Never remove or overwrite any
  // existing file; each run creates at most MAX_TESTS diagnostic files.
  for (uint32_t candidate = 0; candidate < 10000000UL; ++candidate) {
    snprintf(filename, 13, "S%07lu.BIN", candidate);
    if (!SD.exists(filename)) return true;
  }
  return false;
}

void fillPattern(uint8_t sequence) {
  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    writeBlock[i] = static_cast<uint8_t>((i * 37U) ^ (sequence * 73U));
  }
  writeBlock[0] = 0xA5;
  writeBlock[1] = sequence;
  writeBlock[BLOCK_SIZE - 2] = 0x5A;
  writeBlock[BLOCK_SIZE - 1] = static_cast<uint8_t>(~sequence);
}

bool verifyPattern() {
  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    if (readBlock[i] != writeBlock[i]) return false;
  }
  return true;
}

void runTest() {
  char filename[13] = {};
  const uint32_t scanStartedUs = micros();
  const bool nameFound = chooseFilename(filename);
  const uint32_t scanUs = elapsedUs(scanStartedUs);
  if (!nameFound) {
    Serial.println(F("sd_diag result=no-free-8.3-name"));
    ++failed;
    return;
  }
  fillPattern(testIndex);

  const uint32_t openStartedUs = micros();
  File file = SD.open(filename, FILE_WRITE);
  const uint32_t openUs = elapsedUs(openStartedUs);
  const bool openOk = static_cast<bool>(file);
  if (!openOk) {
    ++failed;
    Serial.print(F("sd_diag n=")); Serial.print(testIndex);
    Serial.print(F(" file=")); Serial.print(filename);
    Serial.print(F(" scan_us=")); Serial.print(scanUs);
    Serial.print(F(" open_us=")); Serial.print(openUs);
    Serial.println(F(" result=open-fail"));
    return;
  }

  const uint32_t writeStartedUs = micros();
  const size_t written = file.write(writeBlock, BLOCK_SIZE);
  const uint32_t writeUs = elapsedUs(writeStartedUs);
  const bool writeError = file.getWriteError();
  if (writeError) file.clearWriteError();

  const uint32_t flushStartedUs = micros();
  file.flush();
  const uint32_t flushUs = elapsedUs(flushStartedUs);
  const bool flushError = file.getWriteError();
  if (flushError) file.clearWriteError();

  const uint32_t closeStartedUs = micros();
  file.close();
  const uint32_t closeUs = elapsedUs(closeStartedUs);

  const uint32_t readOpenStartedUs = micros();
  File readFile = SD.open(filename, FILE_READ);
  const uint32_t readOpenUs = elapsedUs(readOpenStartedUs);
  const bool readOpenOk = static_cast<bool>(readFile);
  const uint32_t readStartedUs = micros();
  const int readCount = readOpenOk ? readFile.read(readBlock, BLOCK_SIZE) : -1;
  const uint32_t readUs = elapsedUs(readStartedUs);
  if (readOpenOk) readFile.close();

  const bool verified = readCount == BLOCK_SIZE && verifyPattern();
  const bool ok = written == BLOCK_SIZE && !writeError && !flushError &&
                  readOpenOk && verified;
  if (ok) ++passed; else ++failed;

  Serial.print(F("sd_diag n=")); Serial.print(testIndex);
  Serial.print(F(" file=")); Serial.print(filename);
  Serial.print(F(" us=s")); Serial.print(scanUs);
  Serial.print(F(" o")); Serial.print(openUs);
  Serial.print(F(" w")); Serial.print(writeUs);
  Serial.print(F(" f")); Serial.print(flushUs);
  Serial.print(F(" c")); Serial.print(closeUs);
  Serial.print(F(" ro")); Serial.print(readOpenUs);
  Serial.print(F(" r")); Serial.print(readUs);
  Serial.print(F(" bytes=")); Serial.print(written);
  Serial.print(F(" read=")); Serial.print(readCount);
  Serial.print(F(" we=")); Serial.print(writeError ? 1 : 0);
  Serial.print(F(" fe=")); Serial.print(flushError ? 1 : 0);
  Serial.print(F(" verify=")); Serial.print(verified ? 1 : 0);
  Serial.print(F(" result=")); Serial.println(ok ? F("PASS") : F("FAIL"));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  // This diagnostic is interactive.  Waiting here makes a missing banner
  // meaningful: the host has opened the monitor before SD.begin() is called.
  while (!Serial) { delay(10); }
  Serial.println(F("sd_diag serial=ready"));
  Serial.println(F("sd_diag calling SD.begin"));
  delay(20);

  const uint32_t beginStartedUs = micros();
  const bool sdOk = SD.begin(SD_CS_PIN);
  const uint32_t beginUs = elapsedUs(beginStartedUs);
  Serial.print(F("sd_diag begin=")); Serial.print(sdOk ? F("ok") : F("fail"));
  Serial.print(F(" begin_us=")); Serial.println(beginUs);
  if (!sdOk) {
    Serial.println(F("sd_diag halt: check card, socket, SPI wiring, and power"));
    for (;;) {}
  }
}

void loop() {
  const uint32_t nowMs = millis();
  if (testIndex < MAX_TESTS &&
      static_cast<uint32_t>(nowMs - lastTestMs) >= TEST_INTERVAL_MS) {
    lastTestMs = nowMs;
    runTest();
    ++testIndex;
    if (testIndex == MAX_TESTS) {
      Serial.print(F("sd_diag summary pass=")); Serial.print(passed);
      Serial.print(F(" fail=")); Serial.println(failed);
    }
  }
}
