#pragma once
// ═══════════════════════════════════════════════════════════════
// coarse_track.h — COARSE_TRACK 상태 보관·판정·계산 (Phase 4 통합 코어)
//
// 담당: GPS 파트 (CYN) · .ino의 COARSE 모드가 쓰는 얇은 코어.
// 로직을 Arduino 비의존 헤더로 분리해 PC에서 단위시험한다
// (test_cpp/test_coarse.cpp — 파이프라인·타임아웃·폴백 판정 검증).
//
// 데이터 공급 경로 (모두 같은 on*() 함수로 들어옴):
//   · 실물: gpsPoll()/rocketLinkPoll()/baroPoll() (부품 도착·§8 포트 확정 후 배선)
//   · 주입: 시리얼 'G'(fix)/'R'(로켓 패킷)/'A'(페이로드 기압계 AGL) 명령
//           (부품 없이 지금 검증 — Stage B 방식)
//
// [고도기준 2026-07-21] 페이로드 고도는 기압계 AGL(onBaro) 우선.
//   기압계 값이 없거나 오래되면(BARO_TIMEOUT_S) GPS hMSL(payload.alt_m)로 폴백
//   → 기존 동작·테스트 하위호환. 로켓 고도는 패킷 alt_mm(=로켓 기압계 AGL).
//   ★ 로켓·페이로드 고도는 같은 기준(발사대 0점 AGL)이어야 dD가 맞음.
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
#include "baro.h"            // [고도기준 2026-07-21] 기압계 AGL 코어

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

// ── [예측 업그레이드 2026-07-21] 외삽 지평·전개 가드 (순수함수 — 파이썬 run_sim.py와 동일) ──
// 외삽 지평 Δt = 통신지연 + 서보 리드타임. 전개 가드 중엔 리드를 죽여(0) 과-외삽 방지.
// 모션 모델은 등속(P+V·Δt) 그대로 — 바뀌는 건 "얼마나 앞을 보느냐"뿐.
inline float predictHorizonS(float commDelayS, bool deployGuard) {
  float lead = deployGuard ? 0.0f : SERVO_LEAD_S;
  float h = commDelayS + lead;
  if (h < PREDICT_HORIZON_MIN_S) h = PREDICT_HORIZON_MIN_S;
  if (h > PREDICT_HORIZON_MAX_S) h = PREDICT_HORIZON_MAX_S;
  return h;
}
// 로켓 고도(AGL, m)가 메인 전개 밴드 안인가 — feed-forward 게이트(알고 있는 사출 고도 기반).
inline bool inDeployAltBand(float rocketAltAglM) {
  return rocketAltAglM <= MAIN_DEPLOY_ALT_AGL + DEPLOY_GUARD_MARGIN_UP_M &&
         rocketAltAglM >= MAIN_DEPLOY_ALT_AGL - DEPLOY_GUARD_MARGIN_DN_M;
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
    // [예측 업그레이드 2026-07-21] 반응형 ΔV 가드: 직전 패킷 대비 속도 급변 →
    //   전개(또는 오프노미널)로 간주해 가드 arm. 고도 게이트의 백업.
    if (havePkt_) {
      float dvN = (float)(p.velN_mms - rocket.velN_mms) * 0.001f;
      float dvE = (float)(p.velE_mms - rocket.velE_mms) * 0.001f;
      float dvD = (float)(p.velD_mms - rocket.velD_mms) * 0.001f;
      float dv  = sqrtf(dvN*dvN + dvE*dvE + dvD*dvD);
      if (dv >= DEPLOY_DV_THRESH_MPS) tReactiveGuardUntil_ = nowMs + DEPLOY_GUARD_HOLD_MS;
    }
    rocket = p;
    tPktMs_ = nowMs;
    havePkt_ = true;
  }
  // [고도기준 2026-07-21] 페이로드 기압계 AGL 주입 (발사대 0점 기준, m).
  //   실물: baroPoll()이 baro.altitude(P) 결과를 공급. 주입: 시리얼 'A' 명령.
  void onBaro(float aglM, uint32_t nowMs) {
    payloadBaroAlt_m = aglM;
    tBaroMs_ = nowMs;
    haveBaro_ = true;
  }

  uint32_t pktAgeMs(uint32_t nowMs) const { return havePkt_ ? (nowMs - tPktMs_) : 0xFFFFFFFFu; }
  uint32_t fixAgeMs(uint32_t nowMs) const { return haveFix_ ? (nowMs - payload.t_local_ms) : 0xFFFFFFFFu; }
  uint32_t baroAgeMs(uint32_t nowMs) const { return haveBaro_ ? (nowMs - tBaroMs_) : 0xFFFFFFFFu; }

  bool commOk(uint32_t nowMs) const {
    return havePkt_ && pktAgeMs(nowMs) <= (uint32_t)(COMM_TIMEOUT_S * 1000.0f);
  }
  bool fixOk(uint32_t nowMs) const {
    return haveFix_ && payload.valid &&
           fixAgeMs(nowMs) <= (uint32_t)(GPS_FIX_TIMEOUT_S * 1000.0f);
  }
  // 기압계 고도가 신선한가 (고도 소스로 쓸 수 있나). 아니면 GPS 폴백.
  bool baroOk(uint32_t nowMs) const {
    return haveBaro_ && baroAgeMs(nowMs) <= (uint32_t)(BARO_TIMEOUT_S * 1000.0f);
  }
  // [예측 업그레이드 2026-07-21] 반응형 ΔV 가드가 아직 유효한가 (uint 랩 안전한 부호 비교).
  bool reactiveGuardActive(uint32_t nowMs) const {
    return (int32_t)(tReactiveGuardUntil_ - nowMs) > 0;
  }
  // 전개 가드 통합 판정: 고도 게이트(feed-forward) OR 반응형 ΔV(백업). 텔레메트리에도 노출.
  //   DEPLOY_GUARD_ENABLE=0이면 항상 false(리드 유지) — 완만한 전개용.
  bool inDeployGuard(uint32_t nowMs) const {
    if (!DEPLOY_GUARD_ENABLE) return false;
    return (havePkt_ && inDeployAltBand((float)rocket.alt_mm * 0.001f))
           || reactiveGuardActive(nowMs);
  }
  // COARSE 조준 가능 상태인가 (fsm.py의 comm_ok 판정 + 데이터 품질)
  bool ready(uint32_t nowMs) const {
    return commOk(nowMs) && fixOk(nowMs) && rocketPacketUsable(rocket);
  }

  // 원시 짐벌 명령각 계산 (클램프/rate limit은 기존 GimbalCommandFilter 담당).
  // 반환 false = 데이터 부족/오래됨 → 호출측(.ino)은 HOLD로 폴백.
  bool compute(const float q[4], uint32_t nowMs, float* yawDeg, float* pitchDeg) const {
    if (!ready(nowMs)) return false;
    const float commDelay = itowDelayS(payload.iTOW, rocket.iTOW);   // §5.2 방법1
    // [예측 업그레이드 2026-07-21] 외삽 지평 = 통신지연 + 서보 리드(전개 가드 시 리드 드롭).
    const float horizon = predictHorizonS(commDelay, inDeployGuard(nowMs));
    // [고도기준 2026-07-21] 페이로드 고도: 기압계 AGL 우선, 없으면 GPS hMSL 폴백.
    //   수평(위경도)은 GPS 유지. 로켓 고도는 rocketPacketToGeo가 패킷 alt_mm 사용.
    const float plAlt = baroOk(nowMs) ? payloadBaroAlt_m : payload.alt_m;
    const GeoPoint pl = { payload.lat_i7, payload.lon_i7, plAlt };
    coarsePipeline(rocketPacketToGeo(rocket), rocketPacketToVel(rocket),
                   pl, q, horizon, yawDeg, pitchDeg);
    return true;
  }

  void reset() { havePkt_ = false; haveFix_ = false; haveBaro_ = false; }

private:
  uint32_t tPktMs_ = 0;
  uint32_t tBaroMs_ = 0;
  uint32_t tReactiveGuardUntil_ = 0;   // 반응형 ΔV 가드 만료 시각 [ms]
  bool havePkt_ = false;
  bool haveFix_ = false;
  bool haveBaro_ = false;
  float payloadBaroAlt_m = 0.0f;   // 페이로드 기압계 AGL [m] (발사대 0점 기준)
};
