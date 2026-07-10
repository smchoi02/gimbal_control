#pragma once
// ═══════════════════════════════════════════════════════════════
// geometry.h — 자세·좌표 변환 핵심 (파이썬 geometry.py와 1:1 대응)
//
// Arduino 비의존 (표준 C++만 사용) → PC에서 g++로 단위시험 가능.
// test_cpp/test_parity.cpp가 파이썬 test_vectors.py와 동일 값 검증.
//
// SAMD21은 FPU가 없으므로 double 대신 float 사용 (자세 계산엔 충분).
// GPS 위경도 int32 처리는 팀원 담당 geo 모듈에서 (여긴 IMU 파트만).
// ═══════════════════════════════════════════════════════════════
#include <math.h>

#ifndef DEG2RADF
#define DEG2RADF (0.017453292519943295f)
#define RAD2DEGF (57.29577951308232f)
#endif

// ── 쿼터니언 정규화 ──
inline void quatNormalize(float q[4]) {
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-9f) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}

// ── 쿼터니언 (w,x,y,z) → Body→NED 회전행렬 R[3][3] ──
// q는 body 벡터를 NED로 돌리는 자세 (BNO085 Rotation Vector와 동일 의미)
inline void quatToR_b2n(const float q[4], float R[3][3]) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
  R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
  R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

// ── NED 벡터 → Body 벡터 (R_b2n의 전치를 곱함) ──
inline void nedToBody(const float vN[3], const float q[4], float vB[3]) {
  float R[3][3];
  quatToR_b2n(q, R);
  // v_B = Rᵀ · v_N
  vB[0] = R[0][0]*vN[0] + R[1][0]*vN[1] + R[2][0]*vN[2];
  vB[1] = R[0][1]*vN[0] + R[1][1]*vN[1] + R[2][1]*vN[2];
  vB[2] = R[0][2]*vN[0] + R[1][2]*vN[1] + R[2][2]*vN[2];
}

// ── Body 벡터 → NED 벡터 ──
inline void bodyToNed(const float vB[3], const float q[4], float vN[3]) {
  float R[3][3];
  quatToR_b2n(q, R);
  vN[0] = R[0][0]*vB[0] + R[0][1]*vB[1] + R[0][2]*vB[2];
  vN[1] = R[1][0]*vB[0] + R[1][1]*vB[1] + R[1][2]*vB[2];
  vN[2] = R[2][0]*vB[0] + R[2][1]*vB[1] + R[2][2]*vB[2];
}

// ── Body 벡터 → 짐벌각 (yaw+=시계, pitch+=아래) ──
inline void bodyToGimbal(const float vB[3], float* yawDeg, float* pitchDeg) {
  *yawDeg   = atan2f(vB[1], vB[0]) * RAD2DEGF;
  float horiz = sqrtf(vB[0]*vB[0] + vB[1]*vB[1]);
  *pitchDeg = atan2f(vB[2], horiz) * RAD2DEGF;   // z+(아래) → pitch+
}

// ── 짐벌각 → Body 단위벡터 (카메라가 보는 방향) ──
inline void gimbalToBody(float yawDeg, float pitchDeg, float vB[3]) {
  float y = yawDeg * DEG2RADF, p = pitchDeg * DEG2RADF;
  vB[0] = cosf(p) * cosf(y);
  vB[1] = cosf(p) * sinf(y);
  vB[2] = sinf(p);
}

// ── ZYX 오일러 (deg) → 쿼터니언. SIM 모드 가상 자세 생성용 ──
inline void eulerToQuat(float yawDeg, float pitchDeg, float rollDeg, float q[4]) {
  float y = yawDeg * DEG2RADF * 0.5f;
  float p = pitchDeg * DEG2RADF * 0.5f;
  float r = rollDeg * DEG2RADF * 0.5f;
  float cy=cosf(y), sy=sinf(y), cp=cosf(p), sp=sinf(p), cr=cosf(r), sr=sinf(r);
  q[0] = cr*cp*cy + sr*sp*sy;
  q[1] = sr*cp*cy - cr*sp*sy;
  q[2] = cr*sp*cy + sr*cp*sy;
  q[3] = cr*cp*sy - sr*sp*cy;
}

// ── 쿼터니언 → 오일러 (deg). 로그·디버그용 ──
inline void quatToEuler(const float q[4], float* yawDeg, float* pitchDeg, float* rollDeg) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  *yawDeg = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * RAD2DEGF;
  float s = 2*(w*y - z*x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  *pitchDeg = asinf(s) * RAD2DEGF;
  *rollDeg = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * RAD2DEGF;
}

// ═══════════════════════════════════════════════════════════════
// STABILIZED_HOLD (Phase 3 핵심) — stabilization.py와 1:1 대응
// ═══════════════════════════════════════════════════════════════

// HOLD 진입 순간: 지금 카메라가 보는 세계 방향을 NED 벡터로 저장
inline void captureHoldDirection(float gimbalYawDeg, float gimbalPitchDeg,
                                 const float q[4], float dirNED[3]) {
  float vB[3];
  gimbalToBody(gimbalYawDeg, gimbalPitchDeg, vB);
  bodyToNed(vB, q, dirNED);
}

// 매 사이클: 저장된 세계 방향을 최신 자세로 재계산 → 짐벌각
// 페이로드가 돌면 짐벌각이 자동으로 반대 회전 → 상쇄
inline void holdDirection(const float dirNED[3], const float q[4],
                          float* yawDeg, float* pitchDeg) {
  float vB[3];
  nedToBody(dirNED, q, vB);
  bodyToGimbal(vB, yawDeg, pitchDeg);
}
