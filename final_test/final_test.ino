/*
  final_test.ino -- Arduino Uno RocketLink final integration test

  Inputs / wiring
    MAX-M10S TX -> D8  (AltSoftSerial RX)
    MAX-M10S RX -> D9  (AltSoftSerial TX, optional; use level shifting if needed)
    BMP581 SDA -> A4, SCL -> A5
    Uno D1/TX  -> E22 RXD through a 5 V-to-3.3 V level shifter or divider
    E22 TXD    not connected (this is a one-way transmitter)
    All grounds common; E22 M0=M1=LOW for normal mode.

  Output: every 100 ms (10 Hz), emit the 34-byte RocketLink packet:
    RK | seq | fix | iTOW | lat | lon | barometric AGL | vN | vE | vD | CRC16

  Required libraries:
    - AltSoftSerial
    - SparkFun BMP581 Arduino Library
*/

#include <AltSoftSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "SparkFun_BMP581_Arduino_Library.h"

constexpr uint32_t GPS_BOOT_BAUD = 9600;
constexpr uint32_t GPS_RUN_BAUD = 38400;
constexpr uint32_t E22_UART_BAUD = 9600;
constexpr uint32_t TX_PERIOD_MS = 100;       // Required 10 Hz transmission period.
constexpr uint32_t BARO_SAMPLE_PERIOD_US = 10000;
constexpr uint8_t BARO_SAMPLES_PER_AVERAGE = 10;
constexpr uint8_t BARO_REFERENCE_SAMPLES = 50;
constexpr uint32_t GPS_STALE_MS = 300;
constexpr size_t RK_PACKET_LEN = 34;

struct NavPvtData {
  uint32_t iTowMs;
  int32_t latI7;
  int32_t lonI7;
  int32_t velNMmS;
  int32_t velEMmS;
  int32_t velDMmS;
  uint32_t hAccMm;
  uint32_t sAccMmS;
  uint8_t fixType;
  uint8_t numSv;
  bool valid;
};

AltSoftSerial gpsSerial;                         // Uno: RX=D8, TX=D9.
BMP581 bmp;

NavPvtData lastGoodFix = {};
bool haveGoodFix = false;
uint8_t validFixStreak = 0;
uint32_t lastGoodFixMs = 0;

float referencePressurePa = NAN;
float pressureSumPa = 0.0f;
uint8_t pressureSampleCount = 0;
int32_t latestAglMm = 0;
bool baroValid = false;

uint8_t sequenceNumber = 0;
uint32_t lastTxMs = 0;
uint32_t lastBaroSampleUs = 0;
uint32_t packetsSent = 0;
uint32_t gpsPvtCount = 0;
uint32_t gpsValidCount = 0;
uint32_t baroAverageCount = 0;

// UBX NAV-PVT parser state; NAV-PVT payload is exactly 92 bytes.
uint8_t ubxState = 0;
uint8_t ubxClass = 0;
uint8_t ubxId = 0;
uint8_t ubxCkA = 0;
uint8_t ubxCkB = 0;
uint16_t ubxLength = 0;
uint16_t ubxIndex = 0;
uint8_t navPayload[92] = {};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint32_t readU32Le(const uint8_t* source) {
  return static_cast<uint32_t>(source[0]) |
         (static_cast<uint32_t>(source[1]) << 8) |
         (static_cast<uint32_t>(source[2]) << 16) |
         (static_cast<uint32_t>(source[3]) << 24);
}

void writeU16Le(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

void writeU32Le(uint8_t* destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
  destination[2] = static_cast<uint8_t>(value >> 16);
  destination[3] = static_cast<uint8_t>(value >> 24);
}

void writeI32Le(uint8_t* destination, int32_t value) {
  writeU32Le(destination, static_cast<uint32_t>(value));
}

bool beginBmp581() {
  if (bmp.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) == BMP5_OK) return true;
  return bmp.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) == BMP5_OK;
}

bool configureBmp581() {
  if (bmp.setMode(BMP5_POWERMODE_STANDBY) != BMP5_OK ||
      bmp.setODRFrequency(BMP5_ODR_100_2_HZ) != BMP5_OK ||
      bmp.setMode(BMP5_POWERMODE_NORMAL) != BMP5_OK) {
    return false;
  }
  bmp5_osr_odr_eff effective = {};
  return bmp.getOSREffective(&effective) == BMP5_OK && effective.odr_is_valid != 0;
}

bool readPressure(float* pressurePa) {
  bmp5_sensor_data sample = {0, 0};
  if (bmp.getSensorData(&sample) != BMP5_OK || !isfinite(sample.pressure) ||
      sample.pressure < 30000.0f || sample.pressure > 125000.0f) {
    return false;
  }
  *pressurePa = sample.pressure;
  return true;
}

float altitudeFromPressure(float pressurePa, float referencePa) {
  if (!isfinite(pressurePa) || !isfinite(referencePa) || pressurePa <= 0.0f ||
      referencePa <= 0.0f) {
    return NAN;
  }
  return 44330.77f * (1.0f - pow(pressurePa / referencePa, 0.190263f));
}

bool calibrateBarometer() {
  float meanPressurePa = 0.0f;
  uint8_t count = 0;
  uint32_t previousMs = 0;
  const uint32_t startedMs = millis();

  while (count < BARO_REFERENCE_SAMPLES && millis() - startedMs < 10000UL) {
    const uint32_t nowMs = millis();
    if (nowMs - previousMs < 100UL) continue;
    previousMs = nowMs;
    float pressurePa;
    if (!readPressure(&pressurePa)) continue;
    ++count;
    meanPressurePa += (pressurePa - meanPressurePa) / count;
  }
  if (count < 10) return false;
  referencePressurePa = meanPressurePa;
  return true;
}

void configureGps10Hz() {
  gpsSerial.begin(GPS_BOOT_BAUD);
  delay(200);

  // UBX-CFG-VALSET: RAM layer; output NAV-PVT at 10 Hz, then UART1=38400.
  uint8_t command[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x22, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x07, 0x00, 0x91, 0x20, 0x01,
    0x02, 0x00, 0x74, 0x10, 0x00,
    0x01, 0x00, 0x21, 0x30, 0x64, 0x00,
    0x02, 0x00, 0x21, 0x30, 0x01, 0x00,
    0x01, 0x00, 0x52, 0x40, 0x00, 0x96, 0x00, 0x00,
    0x00, 0x00
  };
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  for (uint8_t i = 2; i < sizeof(command) - 2; ++i) {
    ckA += command[i];
    ckB += ckA;
  }
  command[sizeof(command) - 2] = ckA;
  command[sizeof(command) - 1] = ckB;
  gpsSerial.write(command, sizeof(command));
  gpsSerial.flush();
  delay(100);
  gpsSerial.end();
  gpsSerial.begin(GPS_RUN_BAUD);
}

void acceptNavPvt() {
  NavPvtData fix = {};
  fix.iTowMs = readU32Le(navPayload + 0);
  fix.fixType = navPayload[20];
  fix.numSv = navPayload[23];
  fix.lonI7 = static_cast<int32_t>(readU32Le(navPayload + 24));
  fix.latI7 = static_cast<int32_t>(readU32Le(navPayload + 28));
  fix.hAccMm = readU32Le(navPayload + 40);
  fix.velNMmS = static_cast<int32_t>(readU32Le(navPayload + 48));
  fix.velEMmS = static_cast<int32_t>(readU32Le(navPayload + 52));
  fix.velDMmS = static_cast<int32_t>(readU32Le(navPayload + 56));
  fix.sAccMmS = readU32Le(navPayload + 68);
  fix.valid = (fix.fixType == 3 || fix.fixType == 4) && ((navPayload[21] & 0x01) != 0);
  ++gpsPvtCount;

  // Accuracy values are error estimates. These gates do not limit rocket speed.
  const bool qualityGood = fix.valid && fix.hAccMm <= 30000UL && fix.sAccMmS <= 5000UL;
  if (!qualityGood) {
    validFixStreak = 0;
    return;
  }
  if (validFixStreak < 3) ++validFixStreak;
  if (validFixStreak >= 3) {
    lastGoodFix = fix;
    haveGoodFix = true;
    lastGoodFixMs = millis();
    ++gpsValidCount;
  }
}

void pollGps() {
  while (gpsSerial.available()) {
    const uint8_t byte = static_cast<uint8_t>(gpsSerial.read());
    bool pvtReady = false;
    if (ubxState == 0) {
      if (byte == 0xB5) ubxState = 1;
    } else if (ubxState == 1) {
      ubxState = (byte == 0x62) ? 2 : 0;
    } else if (ubxState == 2) {
      ubxClass = byte; ubxCkA = byte; ubxCkB = byte; ubxState = 3;
    } else if (ubxState == 3) {
      ubxId = byte; ubxCkA += byte; ubxCkB += ubxCkA; ubxState = 4;
    } else if (ubxState == 4) {
      ubxLength = byte; ubxCkA += byte; ubxCkB += ubxCkA; ubxState = 5;
    } else if (ubxState == 5) {
      ubxLength |= static_cast<uint16_t>(byte) << 8;
      ubxCkA += byte; ubxCkB += ubxCkA; ubxIndex = 0;
      ubxState = (ubxClass == 0x01 && ubxId == 0x07 && ubxLength == 92) ? 6 : 0;
    } else if (ubxState == 6) {
      navPayload[ubxIndex++] = byte; ubxCkA += byte; ubxCkB += ubxCkA;
      if (ubxIndex == 92) ubxState = 7;
    } else if (ubxState == 7) {
      ubxState = (byte == ubxCkA) ? 8 : 0;
    } else {
      pvtReady = (byte == ubxCkB);
      ubxState = 0;
    }
    if (pvtReady) acceptNavPvt();
  }
}

void pollBarometer(uint32_t nowUs) {
  if (static_cast<uint32_t>(nowUs - lastBaroSampleUs) < BARO_SAMPLE_PERIOD_US) return;
  lastBaroSampleUs += BARO_SAMPLE_PERIOD_US;
  if (static_cast<uint32_t>(nowUs - lastBaroSampleUs) >= BARO_SAMPLE_PERIOD_US) {
    lastBaroSampleUs = nowUs;
  }

  float pressurePa;
  if (!readPressure(&pressurePa)) {
    pressureSumPa = 0.0f;
    pressureSampleCount = 0;
    baroValid = false;
    return;
  }
  pressureSumPa += pressurePa;
  ++pressureSampleCount;
  if (pressureSampleCount < BARO_SAMPLES_PER_AVERAGE) return;

  const float meanPressurePa = pressureSumPa / pressureSampleCount;
  pressureSumPa = 0.0f;
  pressureSampleCount = 0;
  const float aglM = altitudeFromPressure(meanPressurePa, referencePressurePa);
  if (!isfinite(aglM)) {
    baroValid = false;
    return;
  }
  latestAglMm = static_cast<int32_t>(lroundf(aglM * 1000.0f));
  baroValid = true;
  ++baroAverageCount;
}

void buildRocketPacket(uint8_t packet[RK_PACKET_LEN], uint32_t nowMs) {
  memset(packet, 0, RK_PACKET_LEN);
  const bool gpsFresh = haveGoodFix && (nowMs - lastGoodFixMs <= GPS_STALE_MS);

  packet[0] = 'R';
  packet[1] = 'K';
  packet[2] = sequenceNumber++;
  packet[3] = gpsFresh ? lastGoodFix.fixType : 0;
  writeU32Le(packet + 4, gpsFresh ? lastGoodFix.iTowMs : nowMs);
  if (gpsFresh) {
    writeI32Le(packet + 8, lastGoodFix.latI7);
    writeI32Le(packet + 12, lastGoodFix.lonI7);
    writeI32Le(packet + 20, lastGoodFix.velNMmS);
    writeI32Le(packet + 24, lastGoodFix.velEMmS);
    writeI32Le(packet + 28, lastGoodFix.velDMmS);
  }
  writeI32Le(packet + 16, baroValid ? latestAglMm : 0);
  writeU16Le(packet + 32, crc16CcittFalse(packet, 32));
}

void sendRocketPacket(uint32_t nowMs) {
  uint8_t packet[RK_PACKET_LEN];
  buildRocketPacket(packet, nowMs);
  Serial.write(packet, sizeof(packet));
  Serial.flush();
  ++packetsSent;
}

void setup() {
  // D0/D1 hardware UART is dedicated to E22. Do not print debug text here.
  Serial.begin(E22_UART_BAUD);
  Wire.begin();
  Wire.setClock(400000);
  delay(500);

  if (!beginBmp581()) {
    while (true) delay(1000);
  }
  if (!configureBmp581() || !calibrateBarometer()) {
    while (true) delay(1000);
  }

  configureGps10Hz();
  lastBaroSampleUs = micros();
}

void loop() {
  pollGps();
  pollBarometer(micros());

  const uint32_t nowMs = millis();
  if (nowMs - lastTxMs >= TX_PERIOD_MS) {
    lastTxMs = nowMs;
    sendRocketPacket(nowMs);
  }
}
