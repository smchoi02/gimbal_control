#pragma once
// ═══════════════════════════════════════════════════════════════
// hw_backends.h — 실하드웨어 read 백엔드 (Phase 4 통합의 마지막 얇은 계층)
//
// 담당: GPS geo 파트 (CYN)
//
// 검증된 파서 코어(gps_ublox.h / rocket_link.h / baro.h)는 전부
// "feed(1바이트)"·"altitude(P)" 단위로만 동작한다. 이 파일은 그 코어에
// 실제 센서 바이트/값을 먹여주는 얇은 래퍼일 뿐이다. 로직은 코어에 있고
// 여기엔 전송(transport)만 있다 → 로직 회귀 위험 없음.
//
// ── 포트/버스 배정 (펌웨어가 확정 → 하드웨어가 이에 맞춰 배선) ──
//   DXL ×2      : Serial1 (OpenRB 내장 TTL)           [기존]
//   PC/디버그   : Serial (USB-C), 115200               [기존]
//   BNO085 IMU  : I2C(Wire) 0x4B                        [기존, .ino bnoInit]
//   BMP390 기압 : I2C(Wire) 0x77                        [신규] ← 이 파일
//   u-blox GPS  : I2C(Wire) DDC 0x42                    [신규] ← 이 파일
//                 (UART 대신 I2C로 붙여 추가 UART 1개 절약. 3개 다 주소 상이)
//   로켓 LoRa   : UART Serial3 (OpenRB D13 RX / D14 TX), LORA_BAUD  [신규]
//
//   ★VERIFY: I2C 주소(BMP390 SDO 스트랩 0x76/0x77), 센서 존재,
//            설치된 OpenRB 코어에 Serial3 인스턴스 有(없으면 Serial2로 교체).
//            실보드 도착 후 아두이노 IDE Verify로 확정.
//
// 의존 라이브러리(아두이노): Wire(내장), Adafruit_BMP3XX(기압계).
//   Adafruit_BMP3XX 미설치 시 라이브러리 매니저에서 설치.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#include "gps_ublox.h"     // UbxNavPvtParser + GpsFix
#include "rocket_link.h"   // RocketLinkParser + RocketPacket
#include "baro.h"          // BaroAgl

// ── 하드웨어 주소/설정 (★VERIFY: 실보드 스트랩 확인) ──
#ifndef GPS_I2C_ADDR
#define GPS_I2C_ADDR   (0x42)   // u-blox DDC 기본 주소
#endif
#ifndef BARO_I2C_ADDR
#define BARO_I2C_ADDR  (0x77)   // BMP390 (SDO=high). SDO=low면 0x76
#endif
#ifndef LORA_BAUD
#define LORA_BAUD      (57600)  // 로켓 LoRa 모듈 UART 속도 (모듈 설정과 일치)
#endif
#ifndef GPS_I2C_CHUNK
#define GPS_I2C_CHUNK  (32)     // DDC 1회 read 청크(바이트). Wire 버퍼 안전값
#endif

// u-blox DDC 레지스터
static const uint8_t UBX_DDC_REG_LEN_HI = 0xFD;  // 대기 바이트수 상위
static const uint8_t UBX_DDC_REG_STREAM = 0xFF;  // 데이터 스트림 레지스터

// ═══════════════════════════════════════════════════════════════
// GPS — u-blox DDC(I2C) 백엔드
//   0xFD/0xFE로 대기 바이트수를 읽고, 그만큼만 0xFF 스트림에서 읽어
//   UbxNavPvtParser에 먹인다. "대기수만큼만" 읽으므로 0xFF 필러를 안 먹어
//   (프레임 내부의 정상 0xFF 데이터도 잃지 않음).
// ═══════════════════════════════════════════════════════════════
inline UbxNavPvtParser& gpsParser() { static UbxNavPvtParser p; return p; }

// 대기 바이트수 질의 (I2C 오류 시 0)
inline uint16_t gpsBytesAvailable() {
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(UBX_DDC_REG_LEN_HI);
  if (Wire.endTransmission(false) != 0) return 0;      // repeated-start 유지
  if (Wire.requestFrom((int)GPS_I2C_ADDR, 2) != 2) return 0;
  uint8_t hi = (uint8_t)Wire.read();
  uint8_t lo = (uint8_t)Wire.read();
  uint16_t n = ((uint16_t)hi << 8) | lo;
  return (n == 0xFFFF) ? 0 : n;                        // 0xFFFF = 센서 미준비
}

// 논블로킹 폴. 완결 NAV-PVT를 얻으면 *out 채우고(수신시각 기록) true.
inline bool gpsPoll(GpsFix* out) {
  uint16_t avail = gpsBytesAvailable();
  bool got = false;
  while (avail > 0) {
    uint8_t chunk = (avail > GPS_I2C_CHUNK) ? (uint8_t)GPS_I2C_CHUNK : (uint8_t)avail;
    // 스트림 레지스터(0xFF)로 포인터 지정 후 chunk바이트 읽기
    Wire.beginTransmission(GPS_I2C_ADDR);
    Wire.write(UBX_DDC_REG_STREAM);
    if (Wire.endTransmission(false) != 0) break;
    int n = Wire.requestFrom((int)GPS_I2C_ADDR, (int)chunk);
    if (n <= 0) break;
    while (Wire.available()) {
      uint8_t b = (uint8_t)Wire.read();
      if (gpsParser().feed(b, out)) {
        out->t_local_ms = millis();   // 로컬 수신 시각(파서는 안 채움)
        got = true;
      }
    }
    avail = (avail > chunk) ? (uint16_t)(avail - chunk) : 0;
  }
  return got;
}

// u-blox 초기화 — UBX-NAV-PVT만 I2C로 10Hz 출력, NMEA off (RAM 레이어=매 부팅 재설정).
//   ★ MAX-M10S 기본은 NMEA 출력 → 이 설정 없이는 gpsPoll이 NAV-PVT를 못 받는다.
//   CFG-VALSET 프레임을 I2C로 write. ACK 실증은 브링업(G-A) 때 u-center로 확인.
inline bool gpsConfigure() {
  UbxValsetBuilder v;                          // RAM 레이어
  v.addU1(CFG_MSGOUT_UBX_NAV_PVT_I2C, 1);      // NAV-PVT를 I2C로 매 epoch 출력
  v.addBool(CFG_I2COUTPROT_NMEA, false);       // I2C NMEA 전체 off (버스 정리)
  v.addU2(CFG_RATE_MEAS, 100);                 // 100ms = 10Hz
  v.addU2(CFG_RATE_NAV, 1);                    // 매 측정마다 NAV 해
  uint8_t f[64];
  size_t n = v.frame(f, sizeof(f));
  if (n == 0) return false;
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(f, n);
  return Wire.endTransmission() == 0;          // I2C write ACK
}

// ═══════════════════════════════════════════════════════════════
// 로켓 LoRa — UART 백엔드
//   LORA_SERIAL은 .ino에서 SERCOM Uart로 생성해 넘긴다(전역 + IRQ 핸들러).
//   바이트 스트림을 RocketLinkParser에 먹여 CRC 검증된 패킷을 뽑는다.
// ═══════════════════════════════════════════════════════════════
inline RocketLinkParser& rocketParser() { static RocketLinkParser p; return p; }

#ifdef LORA_SERIAL
inline bool rocketLinkPoll(RocketPacket* out) {
  bool got = false;
  // 루프가 패킷율(10Hz)보다 빠르므로 보통 1루프당 ≤1패킷.
  // 여러 패킷이 쌓였으면 가장 최근 완결분을 out에 남김(과거분은 파서가 소비).
  while (LORA_SERIAL.available()) {
    if (rocketParser().feed((uint8_t)LORA_SERIAL.read(), out)) got = true;
  }
  return got;
}
#endif

// ═══════════════════════════════════════════════════════════════
// 기압계 — BMP390(I2C) 백엔드 + 발사대 0점
//   baroInit()  : 센서 초기화(오버샘플·IIR·ODR)
//   baroZero(n) : 지상(발사대)에서 n샘플 평균으로 P0 캡처 → 'B' 명령/부팅후
//   baroPoll()  : 현재 압력 → BaroAgl.altitude() = AGL[m]
// ★ 로켓측도 같은 발사대에서 동시에 0점을 잡아야 dD가 물리 상대고도와 일치.
// ═══════════════════════════════════════════════════════════════
inline Adafruit_BMP3XX& baroDev()  { static Adafruit_BMP3XX b; return b; }
inline BaroAgl&         baroAgl()   { static BaroAgl a; return a; }

inline bool baroInit() {
  if (!baroDev().begin_I2C(BARO_I2C_ADDR)) return false;
  baroDev().setPressureOversampling(BMP3_OVERSAMPLING_8X);
  baroDev().setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  baroDev().setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  baroDev().setOutputDataRate(BMP3_ODR_50_HZ);
  return true;
}

// 현재 AGL[m]을 aglM에 쓰고 true. 읽기 실패 시 false(호출측이 timeout→GPS 폴백).
inline bool baroPoll(float* aglM) {
  if (!baroDev().performReading()) return false;
  *aglM = baroAgl().altitude((float)baroDev().pressure);   // Pa
  return true;
}

// 발사대 0점: n샘플 평균 압력으로 P0 캡처. 성공 시 true.
inline bool baroZero(int nSamples) {
  if (nSamples < 1) nSamples = 1;
  float sum = 0.0f; int ok = 0;
  for (int i = 0; i < nSamples; ++i) {
    if (baroDev().performReading()) { sum += (float)baroDev().pressure; ok++; }
    delay(20);   // ~50Hz ODR에 맞춰 새 표본 확보
  }
  if (ok < 1) return false;
  baroAgl().zero(sum / (float)ok);
  return true;
}
