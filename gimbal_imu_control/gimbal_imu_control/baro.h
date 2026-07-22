#pragma once
// ═══════════════════════════════════════════════════════════════
// baro.h — 기압계 고도 코어 (Phase 4 고도 기준: 기압계 AGL)
//          [고도기준 2026-07-21] GPS 수직오차(수 m~십수 m) 회피용.
//
// 담당: GPS geo 파트 (CYN)
//
// 알고리즘의 고도(dD)를 GPS hMSL 대신 기압계 AGL로 공급한다.
//   · 수평(위경도) = GPS 유지, 수직(고도) = 기압계.
//   · 로켓·페이로드 둘 다 발사대에서 0점 → 같은 datum 공유 → 상대 고도차 정합.
//     (페이로드가 로켓에 실려 같은 발사대에서 출발 = 결정된 운용 전제)
//
// Arduino 비의존 (표준 C++/math.h만) → PC에서 g++로 단위시험 가능
//   (test_cpp/test_baro.cpp — geo.h와 동일한 part-free 검증 전략).
//   실제 센서(BMP390/MS5611 등)의 I2C 읽기는 이 코어 뒤에 얇은 백엔드로
//   붙는다 — gps_ublox.h가 UART/I2C를 추상화한 것과 같은 방식 (부품 도착 후).
//
// ★ 상대 고도차(dD)만 쓰므로 P0 절대값의 오차는 무관하다. 두 유닛이
//   같은 기준면(발사대)에서 0점을 잡는 것이 유일한 정확도 조건이다.
// ═══════════════════════════════════════════════════════════════
#include <math.h>
#include <stdint.h>

// 국제표준대기(ISA) 대류권 기압-고도식:
//   h = (T0/L) · (1 − (P/P0)^(R·L/(g·M)))
//   지수 R·L/(g·M) = 0.190263,  계수 T0/L = 288.15/0.0065 = 44330.77 m
// P0 = 0점(발사대)에서 측정한 기준 기압. 결과 h = 기준면 대비 고도(AGL).
#ifndef BARO_EXP
#define BARO_EXP   (0.190263f)
#endif
#ifndef BARO_COEF
#define BARO_COEF  (44330.77f)
#endif
#ifndef BARO_P0_STD_PA
#define BARO_P0_STD_PA (101325.0f)   // 표준 해면기압 (미영점 시 폴백)
#endif

// 기압 P[Pa]·기준기압 P0[Pa] → 기준면 대비 고도[m] (AGL).
//   P == P0 → 0.  P < P0(상승) → 양수.
inline float baroAltitudeAgl(float pressurePa, float p0Pa) {
  if (p0Pa <= 0.0f || pressurePa <= 0.0f) return 0.0f;
  return BARO_COEF * (1.0f - powf(pressurePa / p0Pa, BARO_EXP));
}

// ═══════════════════════════════════════════════════════════════
// BaroAgl — 발사대 0점 관리.
//   zero(P0)         : 지상(발사대)에서 기준기압 캡처 → 이후 altitude()=AGL.
//   zeroAverage(...) : 지상 노이즈 완화용 다중 샘플 평균 0점.
//   altitude(P)      : 현재 기압 → 기준면 대비 고도[m].
// 로켓측·페이로드측이 각자 자기 센서로 이 클래스를 하나씩 갖고, 발사 직전
// 같은 발사대에서 zero() → 두 AGL의 차이(dD)가 물리 상대고도와 일치.
// ═══════════════════════════════════════════════════════════════
class BaroAgl {
public:
  bool  zeroed = false;
  float p0Pa   = BARO_P0_STD_PA;   // 미영점 시 표준 해면기압 폴백

  void zero(float groundPa) {
    if (groundPa > 0.0f) { p0Pa = groundPa; zeroed = true; }
  }
  // n개 지상 샘플 평균으로 0점 (n>=1). 부팅 안정화 후 호출 권장.
  void zeroAverage(const float* samplesPa, int n) {
    if (n < 1 || samplesPa == nullptr) return;
    float s = 0.0f;
    for (int i = 0; i < n; ++i) s += samplesPa[i];
    zero(s / (float)n);
  }
  float altitude(float pressurePa) const {
    return baroAltitudeAgl(pressurePa, p0Pa);
  }
};
