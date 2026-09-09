#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

constexpr uint8_t SD_CS_PIN = 4;

Sd2Card diagnosticCard;
SdVolume diagnosticVolume;

void setup() {
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {}

  Serial.println(F("# OpenRB-150 SD diagnostic"));
  Serial.println(F("# expected: CS=D4 MOSI=D8 SCK=D9 MISO=D10"));

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (!diagnosticCard.init(SPI_HALF_SPEED, SD_CS_PIN)) {
    Serial.print(F("# CARD INIT FAIL errorCode=0x"));
    Serial.print(diagnosticCard.errorCode(), HEX);
    Serial.print(F(" errorData=0x"));
    Serial.println(diagnosticCard.errorData(), HEX);
    Serial.println(F("# Check power, SPI wiring, card insertion, and FAT format."));
    return;
  }

  Serial.print(F("# CARD INIT OK type="));
  switch (diagnosticCard.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println(F("SD1"));
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println(F("SD2"));
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println(F("SDHC/SDXC"));
      break;
    default:
      Serial.println(F("UNKNOWN"));
      break;
  }

  if (!diagnosticVolume.init(diagnosticCard)) {
    Serial.println(F("# FAT VOLUME FAIL"));
    Serial.println(F("# Reformat the card as FAT16 or FAT32; exFAT is unsupported."));
    return;
  }

  Serial.print(F("# FAT VOLUME OK FAT"));
  Serial.println(diagnosticVolume.fatType());

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("# SD LIBRARY BEGIN FAIL"));
    return;
  }

  SD.remove("SDTEST.TXT");
  File testFile = SD.open("SDTEST.TXT", FILE_WRITE);
  if (!testFile) {
    Serial.println(F("# FILE OPEN FAIL"));
    Serial.println(F("# Check write protection and filesystem integrity."));
    return;
  }

  testFile.println(F("OpenRB-150 SD write test OK"));
  testFile.flush();
  const bool writeFailed = testFile.getWriteError();
  testFile.close();

  if (writeFailed) {
    Serial.println(F("# FILE WRITE FAIL"));
    return;
  }

  testFile = SD.open("SDTEST.TXT", FILE_READ);
  if (!testFile) {
    Serial.println(F("# FILE READBACK OPEN FAIL"));
    return;
  }

  Serial.print(F("# READBACK: "));
  while (testFile.available()) {
    Serial.write(testFile.read());
  }
  testFile.close();
  Serial.println(F("# SD DIAGNOSTIC PASS"));
}

void loop() {}
