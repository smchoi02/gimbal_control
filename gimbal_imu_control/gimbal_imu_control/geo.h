#pragma once
// ═══════════════════════════════════════════════════════════════
// geo.h — GPS geo 코어 (Phase 4 COARSE_TRACK) — 파이썬 geometry.py와 1:1 대응
//
// 담당: GPS 파트. 파이썬 geometry.py의 GPS 부분을 이식:
//   predict_position  → predictPosition   (통신 지연 보정 P + V·dt)
//   geo_to_ned        → geoToNed          (위경도차 → NED 미터, int 산술)
//   coarse_pipeline   → coarsePipeline    (전체 파이프라인)
//
// 뒷단(nedToBody, bodyToGimbal)은 geometry.h를 재사용한다 — 이미 파이썬과
// parity 검증된 코드이므로 중복 구현하지 않는다.
//
// Arduino 비의존 (표준 C++/math.h만) → PC에서 g++로 단위시험 가능.
// test_cpp/test_parity.cpp가 파이썬 test_vectors.py와 동일 값 검증.
//
// ★ 정밀도 규약 (Phase 0, config.py 계승):
//   위경도는 int32 (1e-7 deg). 차이를 반드시 '정수로 먼저' 계산한 뒤 미터로
//   변환한다 (float 직접 뺄셈 = catastrophic cancellation, 금지).
//   int32 뺄셈은 int64로 승격해 오버플로를 원천 차단한다.
// ═══════════════════════════════════════════════════════════════
#include <math.h>
#include <stdint.h>
#include "geometry.h"   // nedToBody, bodyToGimbal, DEG2RADF

// ── 지구 상수 (config.py와 동일값) ──
// M_PER_DEG_LAT = 111320.0 m,  M_PER_1E7DEG = 111320.0 * 1e-7 = 0.011132 m
#ifndef M_PER_1E7DEG
#define M_PER_1E7DEG (0.0111320f)   // int32 1e-7deg 1단위당 미터(위도)
#endif

// ── 자료구조 ──
struct GeoPoint {
  int32_t lat_i7;   // 위도 (1e-7 deg) — GPS 원본 형식 그대로
  int32_t lon_i7;   // 경도 (1e-7 deg)
  float   alt_m;    // 고도 (m, hMSL 기준 — 로켓·페이로드 통일)
};

struct NedVel {
  float vN, vE, vD; // NED 속도 (m/s). velD+ = 아래 (UBX velD와 부호 일치)
};

// ═══════════════════════════════════════════════════════════════
// (1) 통신 지연 보정: P_pred = P + V·dt   (파이썬 predict_position)
//     로켓 위치를 dt_s 초만큼 속도로 외삽.
//     C++: 위경도 증분은 실수로 계산 후 반올림해 int32에 더한다.
// ═══════════════════════════════════════════════════════════════
inline GeoPoint predictPosition(const GeoPoint& p, const NedVel& v, float dt_s) {
  // 북쪽 이동(vN+) → 위도 증가. m → 1e-7deg 단위
  float dlat_i7 = (v.vN * dt_s) / M_PER_1E7DEG;
  // 동쪽 이동(vE+) → 경도 증가 (위도에 따른 축소 보정)
  double lat_deg = (double)p.lat_i7 * 1e-7;
  float  coslat  = cosf((float)(lat_deg * (double)DEG2RADF));
  float  dlon_i7 = (v.vE * dt_s) / (M_PER_1E7DEG * coslat);
  // 아래 이동(vD+) → 고도 감소
  float  dalt_m  = -v.vD * dt_s;

  GeoPoint out;
  out.lat_i7 = p.lat_i7 + (int32_t)lroundf(dlat_i7);
  out.lon_i7 = p.lon_i7 + (int32_t)lroundf(dlon_i7);
  out.alt_m  = p.alt_m + dalt_m;
  return out;
}

// ═══════════════════════════════════════════════════════════════
// (2) 위경도차 → NED 미터   (파이썬 geo_to_ned)
//     로켓(R)이 페이로드(P)로부터 어느 방향에 있는지 NED 벡터로.
//     dN+ 북, dE+ 동, dD+ 아래(로켓이 페이로드보다 낮으면 +).
//     차이는 int64로 계산 → 정보 손실·오버플로 없음.
// ═══════════════════════════════════════════════════════════════
inline void geoToNed(const GeoPoint& rocket, const GeoPoint& payload, float dNED[3]) {
  int64_t dlat_i7 = (int64_t)rocket.lat_i7 - (int64_t)payload.lat_i7;  // 정수 뺄셈
  int64_t dlon_i7 = (int64_t)rocket.lon_i7 - (int64_t)payload.lon_i7;

  float dN = (float)dlat_i7 * M_PER_1E7DEG;
  double lat_deg = (double)payload.lat_i7 * 1e-7;
  float  coslat  = cosf((float)(lat_deg * (double)DEG2RADF));
  float dE = (float)dlon_i7 * M_PER_1E7DEG * coslat;
  float dD = payload.alt_m - rocket.alt_m;   // 로켓이 아래면 양수

  dNED[0] = dN;
  dNED[1] = dE;
  dNED[2] = dD;
}

// ═══════════════════════════════════════════════════════════════
// (3) 전체 coarse 파이프라인   (파이썬 coarse_pipeline)
//     predict → geoToNed → nedToBody(geometry.h) → bodyToGimbal(geometry.h)
//
//     rocket/rocketVel : 수신한 로켓 위치·NED 속도
//     payload          : 페이로드 자기 GPS 위치
//     q                : BNO085 자세 쿼터니언 (w,x,y,z)
//     commDelay_s      : 통신 지연 (t_now − t_rocket). 0이면 보정 없음.
//     반환: *yawDeg, *pitchDeg (클램프 전 원시 명령각)
// ═══════════════════════════════════════════════════════════════
inline void coarsePipeline(const GeoPoint& rocket, const NedVel& rocketVel,
                           const GeoPoint& payload, const float q[4],
                           float commDelay_s, float* yawDeg, float* pitchDeg) {
  // (1) 지연 보정
  GeoPoint rPred = predictPosition(rocket, rocketVel, commDelay_s);
  // (2) NED 상대 위치
  float dNED[3];
  geoToNed(rPred, payload, dNED);
  // (3)+(4) Body 변환 (geometry.h)
  float dB[3];
  nedToBody(dNED, q, dB);
  // (5) 짐벌각 (geometry.h)
  bodyToGimbal(dB, yawDeg, pitchDeg);
}
