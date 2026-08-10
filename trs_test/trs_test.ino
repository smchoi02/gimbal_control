#include <Wire.h>
#include <SoftwareSerial.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>
#include <stdint.h>

// MAX-M10S TX -> D8, MAX-M10S RX <- D9
// E22 RX <- D11. D10 is reserved as LoRa RX if needed.
// BMP581 + BNO085 share I2C: SDA=A4, SCL=A5.
const uint8_t GPS_RX = 8;
const uint8_t GPS_TX = 9;
const uint8_t LORA_RX = 10;
const uint8_t LORA_TX = 11;

const uint32_t GPS_BAUD = 9600;
const uint32_t LORA_BAUD = 9600;
const uint32_t USB_BAUD = 115200;

const uint32_t BMP_DT_MS = 100;
const uint32_t TX_DT_MS = 200;       // 5 Hz LoRa packet TX
const uint32_t DBG_DT_MS = 1000;
const uint32_t GPS_MAX_AGE_MS = 1000;
const uint32_t IMU_MAX_AGE_MS = 500;
const size_t RK_LEN = 58;            // RK v2: GPS/baro + IMU + final CRC

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial loraSerial(LORA_RX, LORA_TX);
BMP581 pressureSensor;
BNO080 bno;

struct GpsFix {
  uint32_t iTOW;
  int32_t lat_i7;
  int32_t lon_i7;
  int32_t hMSL_mm;
  int32_t velN_mms;
  int32_t velE_mms;
  int32_t velD_mms;
  uint8_t fixType;
  uint8_t numSV;
  bool valid;
  uint32_t tMs;
};

struct ImuData {
  float ax, ay, az;     // linear acceleration [m/s^2], gravity removed
  float gx, gy, gz;     // angular velocity [rad/s]
  float qw, qx, qy, qz; // rotation vector quaternion
  uint8_t quatAcc;
  uint32_t tAccMs;
  uint32_t tGyroMs;
  uint32_t tQuatMs;
};

GpsFix fix = {0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
ImuData imu = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

float p0Pa = NAN;
float aglM = NAN;
uint8_t seq = 0;
bool bnoPresent = false;
uint32_t tBmp = 0, tTx = 0, tDbg = 0;

uint32_t rdU4(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int32_t rdI4(const uint8_t *p) {
  return (int32_t)rdU4(p);
}

void wrU2(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void wrI2(uint8_t *p, int16_t v) {
  wrU2(p, (uint16_t)v);
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

int16_t clampI16(long v) {
  if (v > 32767L) return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}

int16_t accCms2(float a) {
  // m/s^2 -> centi-m/s^2. Range: +/-327.68 m/s^2.
  return clampI16((long)(a * 100.0f + (a >= 0 ? 0.5f : -0.5f)));
}

int16_t gyroDps10(float gRadS) {
  // rad/s -> 0.1 deg/s. Range: +/-3276.8 deg/s.
  float dps10 = gRadS * (180.0f / PI) * 10.0f;
  return clampI16((long)(dps10 + (dps10 >= 0 ? 0.5f : -0.5f)));
}

int16_t q14(float q) {
  return clampI16((long)(q * 16384.0f + (q >= 0 ? 0.5f : -0.5f)));
}

uint16_t u16Age(uint32_t tMs) {
  if (tMs == 0) return 65535;
  uint32_t age = millis() - tMs;
  return age > 65535UL ? 65535 : (uint16_t)age;
}

uint32_t latestImuTime() {
  uint32_t t = imu.tAccMs;
  if (imu.tGyroMs > t) t = imu.tGyroMs;
  if (imu.tQuatMs > t) t = imu.tQuatMs;
  return t;
}

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

float altitudeFromPressure(float pPa, float refPa) {
  if (pPa <= 0.0f || refPa <= 0.0f) return NAN;
  return 44330.77f * (1.0f - pow(pPa / refPa, 0.190263f));
}

// ======================================================
// MAX-M10S UBX configuration and NAV-PVT parser
// ======================================================

void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t a = 0, b = 0;
  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);

  uint8_t hdr[4] = {cls, id, (uint8_t)len, (uint8_t)(len >> 8)};
  for (uint8_t i = 0; i < 4; i++) {
    gpsSerial.write(hdr[i]);
    a += hdr[i];
    b += a;
  }

  for (uint16_t i = 0; i < len; i++) {
    gpsSerial.write(payload[i]);
    a += payload[i];
    b += a;
  }

  gpsSerial.write(a);
  gpsSerial.write(b);
}

void addU1(uint8_t *pl, uint8_t &n, uint32_t key, uint8_t value) {
  wrU4(pl + n, key); n += 4; pl[n++] = value;
}

void addU2(uint8_t *pl, uint8_t &n, uint32_t key, uint16_t value) {
  wrU4(pl + n, key); n += 4; wrU2(pl + n, value); n += 2;
}

void configureMaxM10S() {
  uint8_t pl[64];
  uint8_t n = 0;

  pl[n++] = 0x00; // VALSET version
  pl[n++] = 0x01; // RAM layer only
  pl[n++] = 0x00;
  pl[n++] = 0x00;

  addU1(pl, n, 0x20910007UL, 1);   // CFG-MSGOUT-UBX_NAV_PVT_UART1
  addU1(pl, n, 0x10740001UL, 1);   // CFG-UART1OUTPROT-UBX
  addU1(pl, n, 0x10740002UL, 0);   // CFG-UART1OUTPROT-NMEA
  addU2(pl, n, 0x30210001UL, 200); // CFG-RATE-MEAS, 200 ms = 5 Hz
  addU2(pl, n, 0x30210002UL, 1);   // CFG-RATE-NAV
  sendUbx(0x06, 0x8A, pl, n);      // UBX-CFG-VALSET
}

bool feedUbx(uint8_t x, GpsFix *out) {
  static uint8_t st = 0, cls = 0, id = 0, ckA = 0, ckB = 0;
  static uint16_t len = 0, idx = 0;
  static uint8_t pl[92];

  switch (st) {
    case 0: if (x == 0xB5) st = 1; break;
    case 1: st = (x == 0x62) ? 2 : 0; ckA = ckB = 0; break;
    case 2: cls = x; ckA += x; ckB += ckA; st = 3; break;
    case 3: id = x; ckA += x; ckB += ckA; st = 4; break;
    case 4: len = x; ckA += x; ckB += ckA; st = 5; break;
    case 5:
      len |= (uint16_t)x << 8; ckA += x; ckB += ckA;
      idx = 0; st = (len <= sizeof(pl)) ? 6 : 0;
      break;
    case 6:
      ckA += x; ckB += ckA; pl[idx++] = x;
      if (idx >= len) st = 7;
      break;
    case 7: st = (x == ckA) ? 8 : 0; break;
    case 8:
      st = 0;
      if (x != ckB) return false;
      if (cls != 0x01 || id != 0x07 || len != 92) return false;

      out->iTOW = rdU4(pl + 0);
      out->fixType = pl[20];
      uint8_t flags = pl[21];
      out->numSV = pl[23];
      out->lon_i7 = rdI4(pl + 24);
      out->lat_i7 = rdI4(pl + 28);
      out->hMSL_mm = rdI4(pl + 36);
      out->velN_mms = rdI4(pl + 48);
      out->velE_mms = rdI4(pl + 52);
      out->velD_mms = rdI4(pl + 56);
      out->valid = ((out->fixType == 3 || out->fixType == 4) && (flags & 0x01));
      out->tMs = millis();
      return true;
  }
  return false;
}

void readGps() {
  while (gpsSerial.available()) feedUbx((uint8_t)gpsSerial.read(), &fix);
}

// ======================================================
// BMP581 and BNO085
// ======================================================

bool readBmp(float *pressurePa) {
  bmp5_sensor_data s = {0, 0};
  if (pressureSensor.getSensorData(&s) != BMP5_OK || s.pressure <= 0.0f) return false;
  *pressurePa = s.pressure;
  return true;
}

bool initBmp() {
  if (pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) == BMP5_OK) return true;
  return pressureSensor.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) == BMP5_OK;
}

bool zeroBmp() {
  uint8_t n = 0;
  float mean = 0.0f;
  uint32_t last = 0, start = millis();

  Serial.print(F("BMP zero"));
  while (n < 50 && millis() - start < 10000UL) {
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

bool initBno() {
  if (!bno.begin(0x4A, Wire) && !bno.begin(0x4B, Wire)) return false;
  bno.enableLinearAccelerometer(20); // 50 Hz, gravity removed
  bno.enableGyro(20);                // 50 Hz, rad/s
  bno.enableRotationVector(20);      // 50 Hz quaternion
  return true;
}

void readImu() {
  if (!bnoPresent) return;

  if (bno.hasReset()) {
    bno.enableLinearAccelerometer(20);
    bno.enableGyro(20);
    bno.enableRotationVector(20);
  }

  for (uint8_t i = 0; i < 4; i++) {
    uint16_t id = bno.getReadings();
    if (id == 0) break;

    if (id == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      imu.ax = bno.getLinAccelX();
      imu.ay = bno.getLinAccelY();
      imu.az = bno.getLinAccelZ();
      imu.tAccMs = millis();
    } else if (id == SENSOR_REPORTID_GYROSCOPE) {
      imu.gx = bno.getGyroX();
      imu.gy = bno.getGyroY();
      imu.gz = bno.getGyroZ();
      imu.tGyroMs = millis();
    } else if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      imu.qw = bno.getQuatReal();
      imu.qx = bno.getQuatI();
      imu.qy = bno.getQuatJ();
      imu.qz = bno.getQuatK();
      imu.quatAcc = bno.getQuatAccuracy();
      imu.tQuatMs = millis();
    }
  }
}

// ======================================================
// RK v2 packet TX
// ======================================================

bool gpsFresh() {
  return fix.valid && millis() - fix.tMs <= GPS_MAX_AGE_MS;
}

uint8_t imuFlags() {
  uint8_t f = 0;
  uint32_t now = millis();
  if (bnoPresent && imu.tAccMs  != 0 && now - imu.tAccMs  <= IMU_MAX_AGE_MS) f |= 0x01;
  if (bnoPresent && imu.tGyroMs != 0 && now - imu.tGyroMs <= IMU_MAX_AGE_MS) f |= 0x02;
  if (bnoPresent && imu.tQuatMs != 0 && now - imu.tQuatMs <= IMU_MAX_AGE_MS) f |= 0x04;
  return f;
}

void buildRkPacket(uint8_t out[RK_LEN]) {
  bool ok = gpsFresh();
  uint8_t flags = imuFlags();

  // Offsets 0..31 are identical to old RK packet, but CRC is now at 56..57.
  wrU2(out + 0, 0x4B52);          // bytes: 'R' 'K'
  out[2] = seq++;
  out[3] = ok ? fix.fixType : 0;
  wrU4(out + 4, ok ? fix.iTOW : 0);
  wrI4(out + 8, ok ? fix.lat_i7 : 0);
  wrI4(out + 12, ok ? fix.lon_i7 : 0);
  wrI4(out + 16, mm(aglM));
  wrI4(out + 20, ok ? fix.velN_mms : 0);
  wrI4(out + 24, ok ? fix.velE_mms : 0);
  wrI4(out + 28, ok ? fix.velD_mms : 0);

  out[32] = flags;                // bit0 accel, bit1 gyro, bit2 quat
  out[33] = imu.quatAcc;
  wrU2(out + 34, u16Age(latestImuTime()));
  wrI2(out + 36, (flags & 0x01) ? accCms2(imu.ax) : 0);
  wrI2(out + 38, (flags & 0x01) ? accCms2(imu.ay) : 0);
  wrI2(out + 40, (flags & 0x01) ? accCms2(imu.az) : 0);
  wrI2(out + 42, (flags & 0x02) ? gyroDps10(imu.gx) : 0);
  wrI2(out + 44, (flags & 0x02) ? gyroDps10(imu.gy) : 0);
  wrI2(out + 46, (flags & 0x02) ? gyroDps10(imu.gz) : 0);
  wrI2(out + 48, (flags & 0x04) ? q14(imu.qw) : 0);
  wrI2(out + 50, (flags & 0x04) ? q14(imu.qx) : 0);
  wrI2(out + 52, (flags & 0x04) ? q14(imu.qy) : 0);
  wrI2(out + 54, (flags & 0x04) ? q14(imu.qz) : 0);
  wrU2(out + 56, crc16(out, 56));
}

void printHex(uint8_t x) {
  if (x < 16) Serial.print('0');
  Serial.print(x, HEX);
}

void printStatus(const uint8_t p[RK_LEN]) {
  Serial.print(F("RKv2 packet: "));
  for (uint8_t i = 0; i < RK_LEN; i++) {
    printHex(p[i]);
    Serial.print(i + 1 == RK_LEN ? '\n' : ' ');
  }

  Serial.print(F("fix=")); Serial.print(p[3]);
  Serial.print(F(", sats=")); Serial.print(fix.numSV);
  Serial.print(F(", agl_m=")); Serial.print(isnan(aglM) ? 0.0f : aglM, 2);
  Serial.print(F(", imuFlags=0x")); Serial.print(p[32], HEX);
  Serial.print(F(", acc="));
  Serial.print(imu.ax, 2); Serial.print(',');
  Serial.print(imu.ay, 2); Serial.print(',');
  Serial.print(imu.az, 2);
  Serial.print(F(", gyro_dps="));
  Serial.print(imu.gx * 180.0f / PI, 1); Serial.print(',');
  Serial.print(imu.gy * 180.0f / PI, 1); Serial.print(',');
  Serial.print(imu.gz * 180.0f / PI, 1);
  Serial.print(F(", q="));
  Serial.print(imu.qw, 3); Serial.print(',');
  Serial.print(imu.qx, 3); Serial.print(',');
  Serial.print(imu.qy, 3); Serial.print(',');
  Serial.print(imu.qz, 3);
  Serial.println();
}

void haltWithMessage(const __FlashStringHelper *msg) {
  while (true) {
    Serial.println(msg);
    Serial.flush();
    readGps();
    delay(1000);
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(1500);
  Serial.println(F("BOOT trs_test3"));
  Serial.flush();

  gpsSerial.begin(GPS_BAUD);
  loraSerial.begin(LORA_BAUD);
  gpsSerial.listen();
  Serial.println(F("UART OK"));

  Wire.begin();
  Wire.setClock(100000);
  Serial.println(F("I2C OK"));
  delay(500);

  Serial.println(F("TRS test3: RK v2 GPS/BMP/IMU LoRa TX"));
  Serial.println(F("Configuring MAX-M10S..."));
  configureMaxM10S();
  Serial.println(F("MAX-M10S config sent: UBX-NAV-PVT UART1, NMEA off, 5 Hz"));

  Serial.println(F("Initializing BMP581..."));
  if (!initBmp()) {
    haltWithMessage(F("BMP581 not found"));
  }
  Serial.println(F("Zeroing BMP581..."));
  if (!zeroBmp()) {
    haltWithMessage(F("BMP zero failed"));
  }

  Serial.println(F("Initializing BNO085/BNO080 library..."));
  Wire.setClock(400000);
  bnoPresent = initBno();
  Serial.println(bnoPresent ? F("BNO085 OK") : F("BNO085 not found"));
}

void loop() {
  readGps();
  readImu();
  uint32_t now = millis();

  if (now - tBmp >= BMP_DT_MS) {
    tBmp = now;
    float p;
    if (readBmp(&p)) aglM = altitudeFromPressure(p, p0Pa);
  }

  if (now - tTx >= TX_DT_MS) {
    tTx = now;
    uint8_t packet[RK_LEN];
    buildRkPacket(packet);
    loraSerial.write(packet, RK_LEN);
    loraSerial.flush();
  }

  if (now - tDbg >= DBG_DT_MS) {
    tDbg = now;
    uint8_t packet[RK_LEN];
    buildRkPacket(packet);
    printStatus(packet);
  }
}
