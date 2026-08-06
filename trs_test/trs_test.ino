#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <math.h>
#include <stdint.h>

// MAX-M10S TX -> D8, optional RX <- D9
const uint8_t GPS_RX = 8;
const uint8_t GPS_TX = 9;
const uint8_t LORA_RX = 10;
const uint8_t LORA_TX = 11;
const uint32_t GPS_BAUD = 9600;
const uint32_t LORA_BAUD = 9600;
const uint32_t USB_BAUD = 115200;

const uint32_t BMP_DT_MS = 100;
const uint32_t PRINT_DT_MS = 1000;
const uint32_t GPS_MAX_AGE_MS = 3000;
const size_t PACKET_LEN = 34;
const bool LORA_ASCII_TEST = false; // PuTTY link check. Set false for packet-only TX.

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial loraSerial(LORA_RX, LORA_TX);
TinyGPSPlus gps;
BMP581 bmp;

float p0Pa = NAN;
float aglM = NAN;
uint8_t seq = 0;
uint32_t tBmp = 0;
uint32_t tPrint = 0;

uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
  }
  return c;
}

void wrU2(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void wrU4(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

void wrI4(uint8_t *p, int32_t v) {
  wrU4(p, (uint32_t)v);
}

int32_t mm(float m) {
  if (isnan(m)) return 0;
  float v = m * 1000.0f;
  return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

float altitudeFromPressure(float pPa, float refPa) {
  if (pPa <= 0.0f || refPa <= 0.0f) return NAN;
  return 44330.77f * (1.0f - pow(pPa / refPa, 0.190263f));
}

int32_t rawI7(const RawDegrees &r) {
  int32_t v = (int32_t)r.deg * 10000000L + (int32_t)((r.billionths + 50UL) / 100UL);
  return r.negative ? -v : v;
}

void readGps() {
  while (gpsSerial.available()) gps.encode((char)gpsSerial.read());
}

bool readBmp(float *pressurePa) {
  bmp5_sensor_data s = {0, 0};
  if (bmp.getSensorData(&s) != BMP5_OK || s.pressure <= 0.0f) return false;
  *pressurePa = s.pressure;
  return true;
}

bool initBmp() {
  if (bmp.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) == BMP5_OK) return true; // 0x46
  return bmp.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) == BMP5_OK;             // 0x47
}

bool zeroBmp() {
  const uint8_t target = 50;
  uint8_t n = 0;
  float mean = 0.0f;
  uint32_t last = 0;
  uint32_t start = millis();

  Serial.print(F("BMP zero"));
  while (n < target && millis() - start < 10000UL) {
    readGps();
    if (millis() - last < 100UL) continue;
    last = millis();

    float p;
    if (readBmp(&p)) {
      n++;
      mean += (p - mean) / n;
      Serial.print('.');
    }
  }
  Serial.println();

  if (n < 10) return false;
  p0Pa = mean;
  return true;
}

uint32_t tempItowMs() {
  if (!gps.time.isValid()) return millis();
  return (uint32_t)gps.time.hour() * 3600000UL +
         (uint32_t)gps.time.minute() * 60000UL +
         (uint32_t)gps.time.second() * 1000UL +
         (uint32_t)gps.time.centisecond() * 10UL;
}

bool gpsOk() {
  return gps.location.isValid() &&
         gps.location.age() < GPS_MAX_AGE_MS &&
         (!gps.satellites.isValid() || gps.satellites.value() >= 4);
}

void buildPacket(uint8_t out[PACKET_LEN]) {
  bool fix = gpsOk();
  int32_t lat = fix ? rawI7(gps.location.rawLat()) : 0;
  int32_t lon = fix ? rawI7(gps.location.rawLng()) : 0;
  int32_t vN = 0, vE = 0, vD = 0;

  if (fix && gps.speed.isValid() && gps.course.isValid()) {
    float spd = gps.speed.mps();
    float crs = gps.course.deg() * (PI / 180.0f);
    vN = mm(spd * cos(crs));
    vE = mm(spd * sin(crs));
  }

  wrU2(out + 0, 0x4B52);       // bytes: 'R' 'K'
  out[2] = seq++;
  out[3] = fix ? 3 : 0;        // 3 = provisional 3D fix from NMEA
  wrU4(out + 4, tempItowMs()); // temporary until UBX-NAV-PVT is used
  wrI4(out + 8, lat);
  wrI4(out + 12, lon);
  wrI4(out + 16, mm(aglM));
  wrI4(out + 20, vN);
  wrI4(out + 24, vE);
  wrI4(out + 28, vD);
  wrU2(out + 32, crc16(out, 32));
}

void printHex(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void printPacket(const uint8_t p[PACKET_LEN]) {
  Serial.print(F("RK packet: "));
  for (size_t i = 0; i < PACKET_LEN; i++) {
    printHex(p[i]);
    Serial.print(i + 1 == PACKET_LEN ? '\n' : ' ');
  }

  Serial.print(F("fix="));
  Serial.print(p[3]);
  Serial.print(F(", lat="));
  Serial.print(gps.location.isValid() ? gps.location.lat() : 0.0, 7);
  Serial.print(F(", lon="));
  Serial.print(gps.location.isValid() ? gps.location.lng() : 0.0, 7);
  Serial.print(F(", age_ms="));
  Serial.print(gps.location.isValid() ? gps.location.age() : 999999UL);
  Serial.print(F(", sats="));
  if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
  else Serial.print(F("NA"));
  Serial.print(F(", agl_m="));
  Serial.print(isnan(aglM) ? 0.0 : aglM, 2);
  Serial.print(F(", crc=0x"));
  Serial.print((uint16_t)p[32] | ((uint16_t)p[33] << 8), HEX);
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  gpsSerial.begin(GPS_BAUD);
  loraSerial.begin(LORA_BAUD);
  gpsSerial.listen();
  Wire.begin();
  delay(500);

  Serial.println(F("TRS packet monitor + LoRa TX"));
  Serial.println(F("LoRa: SoftwareSerial D10(RX), D11(TX), 9600 baud"));
  Serial.println(F("NOTE: iTOW/velD are temporary in this NMEA version."));

  if (!initBmp()) {
    Serial.println(F("BMP581 not found"));
    while (true) readGps();
  }
  if (!zeroBmp()) {
    Serial.println(F("BMP zero failed"));
    while (true) readGps();
  }
}

void loop() {
  readGps();
  uint32_t now = millis();

  if (now - tBmp >= BMP_DT_MS) {
    tBmp = now;
    float p;
    if (readBmp(&p)) aglM = altitudeFromPressure(p, p0Pa);
  }

  if (now - tPrint >= PRINT_DT_MS) {
    tPrint = now;
    uint8_t packet[PACKET_LEN];
    buildPacket(packet);
    if (LORA_ASCII_TEST) {
      loraSerial.print(F("PING seq="));
      loraSerial.print(packet[2]);
      loraSerial.print(F(" fix="));
      loraSerial.println(packet[3]);
    }
    loraSerial.write(packet, PACKET_LEN);
    loraSerial.flush();
    printPacket(packet);
  }
}
