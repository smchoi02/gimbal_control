#include <Wire.h>
#include <TinyGPSPlus.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <math.h>
#include <stdint.h>

// OpenRB-150 hardware UARTs:
//   Serial2: board Serial2 header for the MAX-M10S NMEA UART.
//   Serial3: board header marked RX/TX for the E22-900T22S UART.
#define GPS_SERIAL Serial2
#define E22_SERIAL Serial3
const uint32_t GPS_BAUD = 9600;
const uint32_t E22_BAUD = 9600;
const uint32_t USB_BAUD = 115200;

const uint32_t BMP_DT_MS = 100;
const uint32_t PRINT_DT_MS = 1000;
const uint32_t GPS_MAX_AGE_MS = 3000;
// Must match gimbal_target_tracker/src/comm/remote_packet.h exactly.
// "GT", version, sequence, sender_ms, latitude, longitude, pressure,
// temperature, fix type, flags, reserved, CRC-16/CCITT-FALSE.
const size_t PACKET_LEN = 27;
const bool LORA_ASCII_TEST = false; // PuTTY link check. Set false for packet-only TX.

TinyGPSPlus gps;
BMP581 bmp;

float p0Pa = NAN;
float aglM = NAN;
float pressurePa = NAN;
float temperatureC = NAN;
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

float altitudeFromPressure(float pPa, float refPa) {
  if (pPa <= 0.0f || refPa <= 0.0f) return NAN;
  return 44330.77f * (1.0f - pow(pPa / refPa, 0.190263f));
}

int32_t rawI7(const RawDegrees &r) {
  int32_t v = (int32_t)r.deg * 10000000L + (int32_t)((r.billionths + 50UL) / 100UL);
  return r.negative ? -v : v;
}

void readGps() {
  while (GPS_SERIAL.available()) gps.encode((char)GPS_SERIAL.read());
}

bool readBmp(float *outPressurePa, float *outTemperatureC) {
  bmp5_sensor_data s = {0, 0};
  if (bmp.getSensorData(&s) != BMP5_OK || s.pressure <= 0.0f) return false;
  *outPressurePa = s.pressure;
  *outTemperatureC = s.temperature;
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

    float p, temperature;
    if (readBmp(&p, &temperature)) {
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

bool gpsOk() {
  return gps.location.isValid() &&
         gps.location.age() < GPS_MAX_AGE_MS &&
         (!gps.satellites.isValid() || gps.satellites.value() >= 4);
}

void buildPacket(uint8_t out[PACKET_LEN]) {
  const bool fix = gpsOk();
  const bool barometerOk = isfinite(pressurePa) && isfinite(temperatureC) &&
                           pressurePa >= 30000.0f && pressurePa <= 125000.0f;
  int32_t lat = fix ? rawI7(gps.location.rawLat()) : 0;
  int32_t lon = fix ? rawI7(gps.location.rawLng()) : 0;
  const uint32_t pressurePaX10 =
      barometerOk ? static_cast<uint32_t>(pressurePa * 10.0f + 0.5f) : 0;
  const float temperatureX100 = temperatureC * 100.0f;
  const int16_t temperatureCX100 =
      barometerOk
          ? static_cast<int16_t>(temperatureX100 >= 0.0f
                                     ? temperatureX100 + 0.5f
                                     : temperatureX100 - 0.5f)
          : 0;

  out[0] = 'G';
  out[1] = 'T';
  out[2] = 1;                  // Protocol version.
  out[3] = seq++;
  wrU4(out + 4, millis());
  wrI4(out + 8, lat);
  wrI4(out + 12, lon);
  wrU4(out + 16, pressurePaX10);
  wrU2(out + 20, static_cast<uint16_t>(temperatureCX100));
  out[22] = fix ? 3 : 0;
  out[23] = (fix ? 0x01 : 0x00) | (barometerOk ? 0x02 : 0x00);
  out[24] = 0;
  wrU2(out + 25, crc16(out, 25));
}

void printHex(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void printPacket(const uint8_t p[PACKET_LEN]) {
  Serial.print(F("GT packet: "));
  for (size_t i = 0; i < PACKET_LEN; i++) {
    printHex(p[i]);
    Serial.print(i + 1 == PACKET_LEN ? '\n' : ' ');
  }

  Serial.print(F("seq="));
  Serial.print(p[3]);
  Serial.print(F(", fix="));
  Serial.print(p[22]);
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
  Serial.print(F(", pressure_pa="));
  Serial.print(pressurePa, 1);
  Serial.print(F(", temp_c="));
  Serial.print(temperatureC, 2);
  Serial.print(F(", flags=0x"));
  Serial.print(p[23], HEX);
  Serial.print(F(", crc=0x"));
  Serial.print((uint16_t)p[25] | ((uint16_t)p[26] << 8), HEX);
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  GPS_SERIAL.begin(GPS_BAUD);
  E22_SERIAL.begin(E22_BAUD);
  Wire.begin();
  delay(500);

  Serial.println(F("TRS packet monitor + E22-900T22S TX"));
  Serial.println(F("GPS: Serial2, E22: Serial3, 9600 baud"));
  Serial.println(F("GT v1: GPS + absolute BMP581 pressure/temperature."));

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
    float p, temperature;
    if (readBmp(&p, &temperature)) {
      pressurePa = p;
      temperatureC = temperature;
      aglM = altitudeFromPressure(p, p0Pa);
    }
  }

  if (now - tPrint >= PRINT_DT_MS) {
    tPrint = now;
    uint8_t packet[PACKET_LEN];
    buildPacket(packet);
    if (LORA_ASCII_TEST) {
      E22_SERIAL.print(F("PING seq="));
      E22_SERIAL.print(packet[3]);
      E22_SERIAL.print(F(" fix="));
      E22_SERIAL.println(packet[22]);
    }
    E22_SERIAL.write(packet, PACKET_LEN);
    E22_SERIAL.flush();
    printPacket(packet);
  }
}
