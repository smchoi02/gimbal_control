#include <Wire.h>
#include <SoftwareSerial.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

SoftwareSerial gpsSerial(8, 9);     // MAX-M10S TX -> Uno D8
SoftwareSerial loraSerial(10, 11); // Uno D11(TX) -> LoRa RX

BMP581 bmp;

const uint32_t GPS_BAUD = 9600;
const uint32_t LORA_BAUD = 9600;
const uint32_t TX_DT_MS = 1000;
const uint32_t BMP_DT_MS = 100;
const uint32_t GPS_MAX_AGE_MS = 3000;
const uint8_t PACKET_LEN = 34;

struct {
  bool valid;
  uint8_t satellites;
  float lat;
  float lon;
  float mslAltM;
  uint32_t utcMs;
  uint32_t lastGgaMs;
} gps;

char line[100];
byte lineIndex = 0;

bool bmpReady = false;
float groundPressurePa = NAN;
float bmpAglM = 0.0f;

uint8_t sequence = 0;
uint32_t lastTxMs = 0;
uint32_t lastBmpMs = 0;

uint16_t crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;

  while (length--) {
    crc ^= (uint16_t)(*data++) << 8;

    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                            : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

void writeU16(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

void writeU32(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

void writeI32(uint8_t *p, int32_t value) {
  writeU32(p, (uint32_t)value);
}

int32_t meterToMm(float meter) {
  return isnan(meter) ? 0 : (int32_t)(meter * 1000.0f);
}

float toDegree(const char *value, const char *dir) {
  float raw = atof(value);
  int degree = (int)(raw / 100.0f);
  float minute = raw - degree * 100.0f;
  float result = degree + minute / 60.0f;

  if (dir[0] == 'S' || dir[0] == 'W') {
    result = -result;
  }

  return result;
}

uint32_t toUtcMs(const char *text) {
  if (strlen(text) < 6) return millis();

  uint32_t hour = (uint32_t)(text[0] - '0') * 10 + (text[1] - '0');
  uint32_t minute = (uint32_t)(text[2] - '0') * 10 + (text[3] - '0');
  uint32_t second = (uint32_t)(text[4] - '0') * 10 + (text[5] - '0');

  return hour * 3600000UL + minute * 60000UL + second * 1000UL;
}

byte splitNmea(char *text, char *field[], byte maxField) {
  byte count = 1;
  field[0] = text;

  for (char *p = text; *p && count < maxField; p++) {
    if (*p == ',') {
      *p = '\0';
      field[count++] = p + 1;
    }
  }

  return count;
}

void parseGGA(char *text) {
  char *f[15];
  byte n = splitNmea(text, f, 15);

  if (n < 10 || strstr(f[0], "GGA") == NULL) {
    return;
  }

  int fix = atoi(f[6]);
  gps.satellites = (uint8_t)atoi(f[7]);
  gps.utcMs = toUtcMs(f[1]);

  gps.valid =
    fix > 0 &&
    f[2][0] != '\0' &&
    f[4][0] != '\0';

  if (gps.valid) {
    gps.lat = toDegree(f[2], f[3]);
    gps.lon = toDegree(f[4], f[5]);
    gps.mslAltM = atof(f[9]);
  }

  gps.lastGgaMs = millis();
}

void readGps() {
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      line[lineIndex] = '\0';

      if (lineIndex > 6 &&
          line[0] == '$' &&
          strstr(line, "GGA") != NULL) {
        parseGGA(line);
      }

      lineIndex = 0;
    } else if (lineIndex < sizeof(line) - 1) {
      line[lineIndex++] = c;
    } else {
      lineIndex = 0;
    }
  }
}

bool gpsOk() {
  return gps.valid &&
         gps.satellites >= 4 &&
         millis() - gps.lastGgaMs < GPS_MAX_AGE_MS;
}

bool zeroBmp() {
  float mean = 0.0f;
  uint8_t count = 0;

  for (uint8_t i = 0; i < 30; i++) {
    bmp5_sensor_data sensor = {0, 0};

    if (bmp.getSensorData(&sensor) == BMP5_OK &&
        sensor.pressure > 0.0f) {
      count++;
      mean += (sensor.pressure - mean) / count;
    }

    delay(100);
  }

  if (count < 10) return false;

  groundPressurePa = mean;
  return true;
}

void updateBmpAltitude() {
  if (!bmpReady) {
    bmpAglM = 0.0f;
    return;
  }

  bmp5_sensor_data sensor = {0, 0};

  if (bmp.getSensorData(&sensor) == BMP5_OK &&
      sensor.pressure > 0.0f) {
    bmpAglM = 44330.77f *
              (1.0f - pow(sensor.pressure / groundPressurePa, 0.190263f));
  }
}

void sendPacket() {
  uint8_t packet[PACKET_LEN];
  bool valid = gpsOk();

  int32_t latE7 = valid ? (int32_t)(gps.lat * 10000000.0f) : 0;
  int32_t lonE7 = valid ? (int32_t)(gps.lon * 10000000.0f) : 0;

  // trs_test.ino와 동일한 34-byte RK 패킷 구조
  writeU16(packet + 0, 0x4B52);         // bytes: 'R' 'K'
  packet[2] = sequence++;
  packet[3] = valid ? gps.satellites : 0; // 기존 fix 자리 -> 위성 수
  writeU32(packet + 4, gps.utcMs);
  writeI32(packet + 8, latE7);
  writeI32(packet + 12, lonE7);
  writeI32(packet + 16, meterToMm(bmpAglM)); // BMP 상대 고도(AGL)
  writeI32(packet + 20, 0);              // vN 미사용
  writeI32(packet + 24, 0);              // vE 미사용
  writeI32(packet + 28, 0);              // vD 미사용

  writeU16(packet + 32, crc16(packet, 32));

  loraSerial.write(packet, PACKET_LEN);
  loraSerial.flush();

  Serial.print(F("TX | seq="));
  Serial.print(packet[2]);

  Serial.print(F(" | sats="));
  Serial.print(packet[3]);

  Serial.print(F(" | lat="));
  Serial.print(valid ? gps.lat : 0.0f, 6);

  Serial.print(F(" | lon="));
  Serial.print(valid ? gps.lon : 0.0f, 6);

  Serial.print(F(" | gps_msl_m="));
  Serial.print(valid ? gps.mslAltM : 0.0f, 1);

  Serial.print(F(" | bmp_agl_m="));
  Serial.println(bmpAglM, 2);
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);
  loraSerial.begin(LORA_BAUD);
  gpsSerial.listen();

  Wire.begin();
  delay(300);

  bmpReady =
    bmp.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) == BMP5_OK ||
    bmp.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) == BMP5_OK;

  if (bmpReady) {
    bmpReady = zeroBmp();
  }

  Serial.println(
    bmpReady ?
    F("GPS + BMP581 + LoRa READY") :
    F("BMP581 NOT READY: BMP altitude is 0")
  );
}

void loop() {
  readGps();

  if (millis() - lastBmpMs >= BMP_DT_MS) {
    lastBmpMs = millis();
    updateBmpAltitude();
  }

  if (millis() - lastTxMs >= TX_DT_MS) {
    lastTxMs = millis();
    sendPacket();
  }
}