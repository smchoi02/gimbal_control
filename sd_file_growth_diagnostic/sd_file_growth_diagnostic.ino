#include <Arduino.h>
#include <SD.h>

namespace {
constexpr uint8_t SD_CS_PIN = 4;
constexpr size_t BLOCK_SIZE = 512;
constexpr uint16_t BLOCK_COUNT = 128;      // 64 KiB total
constexpr uint8_t FLUSH_EVERY_BLOCKS = 8;  // 4 KiB per flush

uint8_t writeBlock[BLOCK_SIZE];
uint8_t readBlock[BLOCK_SIZE];

uint32_t elapsedUs(uint32_t startedUs) {
  return static_cast<uint32_t>(micros() - startedUs);
}

bool chooseFilename(char filename[13]) {
  // 8.3 name: G + seven digits + .BIN.  This test never removes a file.
  for (uint32_t candidate = 0; candidate < 10000000UL; ++candidate) {
    snprintf(filename, 13, "G%07lu.BIN", candidate);
    if (!SD.exists(filename)) return true;
  }
  return false;
}

void fillPattern(uint16_t blockIndex, uint8_t* destination) {
  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    destination[i] = static_cast<uint8_t>((i * 29U) ^ (blockIndex * 67U));
  }
  destination[0] = 0xA5;
  destination[1] = static_cast<uint8_t>(blockIndex);
  destination[2] = static_cast<uint8_t>(blockIndex >> 8);
  destination[BLOCK_SIZE - 2] = 0x5A;
  destination[BLOCK_SIZE - 1] = static_cast<uint8_t>(~blockIndex);
}

bool verifyBlock(uint16_t blockIndex) {
  fillPattern(blockIndex, writeBlock);
  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    if (readBlock[i] != writeBlock[i]) return false;
  }
  return true;
}

void runGrowthTest() {
  char filename[13] = {};
  const uint32_t scanStartedUs = micros();
  const bool nameFound = chooseFilename(filename);
  const uint32_t scanUs = elapsedUs(scanStartedUs);
  if (!nameFound) {
    Serial.println(F("grow result=no-free-8.3-name"));
    return;
  }

  const uint32_t openStartedUs = micros();
  File file = SD.open(filename, FILE_WRITE);
  const uint32_t openUs = elapsedUs(openStartedUs);
  if (!file) {
    Serial.print(F("grow file=")); Serial.print(filename);
    Serial.print(F(" scan_us=")); Serial.print(scanUs);
    Serial.print(F(" open_us=")); Serial.print(openUs);
    Serial.println(F(" result=open-fail"));
    return;
  }
  Serial.print(F("grow file=")); Serial.print(filename);
  Serial.print(F(" scan_us=")); Serial.print(scanUs);
  Serial.print(F(" open_us=")); Serial.println(openUs);

  uint32_t maxWriteUs = 0;
  uint32_t maxFlushUs = 0;
  uint16_t completedBlocks = 0;
  bool failed = false;

  for (uint16_t block = 0; block < BLOCK_COUNT; ++block) {
    fillPattern(block, writeBlock);
    const uint32_t writeStartedUs = micros();
    const size_t written = file.write(writeBlock, BLOCK_SIZE);
    const uint32_t writeUs = elapsedUs(writeStartedUs);
    if (writeUs > maxWriteUs) maxWriteUs = writeUs;
    const bool writeError = file.getWriteError();
    if (writeError) file.clearWriteError();

    uint32_t flushUs = 0;
    bool flushError = false;
    const bool flushNow = ((block + 1U) % FLUSH_EVERY_BLOCKS) == 0U;
    if (flushNow) {
      const uint32_t flushStartedUs = micros();
      file.flush();
      flushUs = elapsedUs(flushStartedUs);
      if (flushUs > maxFlushUs) maxFlushUs = flushUs;
      flushError = file.getWriteError();
      if (flushError) file.clearWriteError();
    }

    Serial.print(F("grow b=")); Serial.print(block);
    Serial.print(F(" size=")); Serial.print((block + 1UL) * BLOCK_SIZE);
    Serial.print(F(" w_us=")); Serial.print(writeUs);
    if (flushNow) {
      Serial.print(F(" f_us=")); Serial.print(flushUs);
    }
    Serial.print(F(" written=")); Serial.print(written);
    Serial.print(F(" we=")); Serial.print(writeError ? 1 : 0);
    Serial.print(F(" fe=")); Serial.print(flushError ? 1 : 0);
    Serial.println((written == BLOCK_SIZE && !writeError && !flushError)
                       ? F(" result=PASS") : F(" result=FAIL"));

    if (written != BLOCK_SIZE || writeError || flushError) {
      failed = true;
      break;
    }
    ++completedBlocks;
  }

  const uint32_t closeStartedUs = micros();
  file.close();
  const uint32_t closeUs = elapsedUs(closeStartedUs);

  bool verified = !failed;
  uint16_t verifiedBlocks = 0;
  if (!failed) {
    File readFile = SD.open(filename, FILE_READ);
    if (!readFile) {
      verified = false;
    } else {
      for (uint16_t block = 0; block < completedBlocks; ++block) {
        if (readFile.read(readBlock, BLOCK_SIZE) != BLOCK_SIZE ||
            !verifyBlock(block)) {
          verified = false;
          break;
        }
        ++verifiedBlocks;
      }
      readFile.close();
    }
  }

  Serial.print(F("grow summary blocks=")); Serial.print(completedBlocks);
  Serial.print(F(" bytes=")); Serial.print(completedBlocks * BLOCK_SIZE);
  Serial.print(F(" max_write_us=")); Serial.print(maxWriteUs);
  Serial.print(F(" max_flush_us=")); Serial.print(maxFlushUs);
  Serial.print(F(" close_us=")); Serial.print(closeUs);
  Serial.print(F(" verified_blocks=")); Serial.print(verifiedBlocks);
  Serial.print(F(" result="));
  Serial.println(!failed && verified && completedBlocks == BLOCK_COUNT
                     ? F("PASS") : F("FAIL"));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("grow serial=ready"));
  Serial.println(F("grow calling SD.begin"));

  const uint32_t beginStartedUs = micros();
  const bool sdOk = SD.begin(SD_CS_PIN);
  Serial.print(F("grow begin=")); Serial.print(sdOk ? F("ok") : F("fail"));
  Serial.print(F(" begin_us=")); Serial.println(elapsedUs(beginStartedUs));
  if (!sdOk) for (;;) {}
  runGrowthTest();
}

void loop() {}
