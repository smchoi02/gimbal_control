#pragma once
// ═══════════════════════════════════════════════════════════════
// coarse_track.h — COARSE_TRACK 상태 보관·판정·계산 (Phase 4 통합 코어)
//
// 담당: GPS 파트 (CYN) · .ino의 COARSE 모드가 쓰는 얇은 코어.
// 로직을 Arduino 비의존 헤더로 분리해 PC에서 단위시험한다
// (test_cpp/test_coarse.cpp — 파이프라인·타임아웃·폴백 판정 검증).
//
// 데이터 공급은 두 경로 모두 같은 함수(onFix/onPacket)로 들어온다:
//   · 실물: gpsPoll()/rocketLinkPoll() (부품 도착·§8 포트 확정 후 배선)
//   · 주입: 시리얼 'G'/'R' 명령 (부품 없이 지금 검증 — Stage B 방식)
//
// FSM 대응 (fsm.py 축약 구현 — .ino 쪽 배선과 세트):
//   ready = 패킷 나이 ≤ COMM_TIMEOUT_S && fix 나이 ≤ GPS_FIX_TIMEOUT_S
//           && fix.valid && 로켓 fixType 3·4
//   COARSE --(not ready)--> HOLD(현재 방향 캡처)   [fsm.py: comm lost]
//   HOLD  --(armed && ready)--> COARSE             [fsm.py: comm recovered]
//   SCAN(HOLD 5초 후 탐색)은 이번 통합에서 생략 — 필요 시 fsm.py 보고 추가
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include "gimbal_config.h"
#include "rocket_link.h"     // RocketPacket + GpsFix(gps_ublox.h) + geo.h까지 포함

// iTOW 차이 → 지연[s]. 주(週) 경계 랩 + 부호 모두 방어.
//   로켓 패킷이 fix보다 최신이면 음수(역외삽)도 허용하되 [-1, +2]s로 클램프
//   — 비정상 큰 값(시계 불일치)이 predictPosition을 폭주시키지 않게.
inline float itowDelayS(uint32_t payloadItow, uint32_t rocketItow) {
  int64_t d = (int64_t)payloadItow - (int64_t)rocketItow;
  if (d >  302400000LL) d -= 604800000LL;    // 주 경계(토→일) 랩
  if (d < -302400000LL) d += 604800000LL;
  float s = (float)d * 0.001f;
  if (s < -1.0f) s = -1.0f;
  if (s >  2.0f) s =  2.0f;
  return s;
}

class CoarseTracker {
public:
  GpsFix       payload;    // 자기(페이로드) 마지막 fix
  RocketPacket rocket;     // 마지막 유효 로켓 패킷

  void onFix(const GpsFix& f, uint32_t nowMs) {
    payload = f;
    payload.t_local_ms = nowMs;
    haveFix_ = true;
  }
  void onPacket(const RocketPacket& p, uint32_t nowMs) {
    rocket = p;
    tPktMs_ = nowMs;
    havePkt_ = true;
  }

  uint32_t pktAgeMs(uint32_t nowMs) const { return havePkt_ ? (nowMs - tPktMs_) : 0xFFFFFFFFu; }
  uint32_t fixAgeMs(uint32_t nowMs) const { return haveFix_ ? (nowMs - payload.t_local_ms) : 0xFFFFFFFFu; }

  bool commOk(uint32_t nowMs) const {
    return havePkt_ && pktAgeMs(nowMs) <= (uint32_t)(COMM_TIMEOUT_S * 1000.0f);
  }
  bool fixOk(uint32_t nowMs) const {
    return haveFix_ && payload.valid &&
           fixAgeMs(nowMs) <= (uint32_t)(GPS_FIX_TIMEOUT_S * 1000.0f);
  }
  // COARSE 조준 가능 상태인가 (fsm.py의 comm_ok 판정 + 데이터 품질)
  bool ready(uint32_t nowMs) const {
    return commOk(nowMs) && fixOk(nowMs) && rocketPacketUsable(rocket);
  }

  // 원시 짐벌 명령각 계산 (클램프/rate limit은 기존 GimbalCommandFilter 담당).
  // 반환 false = 데이터 부족/오래됨 → 호출측(.ino)은 HOLD로 폴백.
  bool compute(const float q[4], uint32_t nowMs, float* yawDeg, float* pitchDeg) const {
    if (!ready(nowMs)) return false;
    const float delay = itowDelayS(payload.iTOW, rocket.iTOW);   // §5.2 방법1
    const GeoPoint pl = { payload.lat_i7, payload.lon_i7, payload.alt_m };
    coarsePipeline(rocketPacketToGeo(rocket), rocketPacketToVel(rocket),
                   pl, q, delay, yawDeg, pitchDeg);
    return true;
  }

  void reset() { havePkt_ = false; haveFix_ = false; }

private:
  uint32_t tPktMs_ = 0;
  bool havePkt_ = false;
  bool haveFix_ = false;
};
