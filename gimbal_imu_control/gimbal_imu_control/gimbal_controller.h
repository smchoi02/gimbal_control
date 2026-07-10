#pragma once
// ═══════════════════════════════════════════════════════════════
// gimbal_controller.h — 명령 안전 필터 (파이썬 controller.py와 1:1 대응)
//
// Arduino 비의존. DYNAMIXEL 명령 직전에 반드시 이 필터를 통과시킨다.
//
// ★ 설계 노트 (파이썬 시뮬레이터 테스트에서 발견):
//   Yaw ±170° 짐벌은 +170↔-170 wrap 이동이 물리적으로 불가능(dead zone).
//   따라서 rate limit 차이 계산은 wrap 없이 '선형'으로 한다.
//   wrapDeg()는 명령 입력 정규화에만 사용 (예: 200° → -160°).
// ═══════════════════════════════════════════════════════════════
#include "gimbal_config.h"

inline float wrapDeg(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

class GimbalCommandFilter {
public:
  float yawOut   = 0.0f;   // 마지막 출력각 (rate limit 기준점)
  float pitchOut = 0.0f;
  bool  limitFlag = false; // 이번 사이클 clamp/rate limit 발생 (STAT에 실림)

  void reset(float yaw = 0.0f, float pitch = 0.0f) {
    yawOut = yaw; pitchOut = pitch; limitFlag = false;
  }

  // 한 사이클 처리. dt초 동안 최대 MAX_RATE_DEG_S*dt 만큼만 이동.
  void step(float yawCmd, float pitchCmd, float dt) {
    limitFlag = false;
    const float maxStep = MAX_RATE_DEG_S * dt;

    // 1) 명령 정규화 + 물리 한계 clamp
    float yawNorm = wrapDeg(yawCmd);
    float yawC   = clampf(yawNorm, YAW_MIN_DEG, YAW_MAX_DEG);
    float pitchC = clampf(pitchCmd, PITCH_MIN_DEG, PITCH_MAX_DEG);
    if (yawC != yawNorm || pitchC != pitchCmd) limitFlag = true;

    // 2) rate limit — 선형 차이 (wrap 금지: dead zone 통과 불가)
    float dyaw = yawC - yawOut;
    if (dyaw >  maxStep) { yawC = yawOut + maxStep; limitFlag = true; }
    if (dyaw < -maxStep) { yawC = yawOut - maxStep; limitFlag = true; }

    float dpitch = pitchC - pitchOut;
    if (dpitch >  maxStep) { pitchC = pitchOut + maxStep; limitFlag = true; }
    if (dpitch < -maxStep) { pitchC = pitchOut - maxStep; limitFlag = true; }

    yawOut = yawC;
    pitchOut = pitchC;
  }
};
