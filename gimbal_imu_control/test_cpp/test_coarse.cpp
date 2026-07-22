// ===============================================================
// test_coarse.cpp — COARSE_TRACK 통합 코어를 PC에서 검증 (부품 불필요)
//
// coarse_track.h의 판정(comm/fix 타임아웃, 품질)·폴백 조건·계산 파이프라인이
// fsm.py/geometry.py와 같은 결과를 내는지 확인한다.
//
// 컴파일·실행:
//   g++ -std=c++17 -I../gimbal_imu_control test_coarse.cpp -o test_coarse && ./test_coarse
// ===============================================================
#include <cstdio>
#include <cmath>
#include "coarse_track.h"

static int PASS = 0, FAIL = 0;
void check(const char* name, bool cond, double got = 0, double want = 0) {
  if (cond) { PASS++; printf("  ✓ %s\n", name); }
  else      { FAIL++; printf("  ✗ %s  (got=%.4f want=%.4f)\n", name, got, want); }
}
bool approx(double a, double b, double tol = 0.5) { return fabs(a - b) < tol; }

static int32_t degToI7(double deg) { return (int32_t)llround(deg * 1e7); }

int main() {
  const double LAT0 = 37.5, LON0 = 127.0;
  const int32_t P_LAT = degToI7(LAT0), P_LON = degToI7(LON0);
  float q0[4], q90[4], yaw, pitch;
  // eulerToQuat는 geometry.h 제공
  eulerToQuat(0, 0, 0, q0);
  eulerToQuat(90, 0, 0, q90);

  printf("=== 1. 초기 상태 — 데이터 없으면 ready/compute 불가 ===\n");
  {
    CoarseTracker c;
    check("초기 ready=false", !c.ready(1000));
    check("초기 compute=false", !c.compute(q0, 1000, &yaw, &pitch));
    check("pktAge = 무한대 표기", c.pktAgeMs(1000) == 0xFFFFFFFFu);
  }

  printf("=== 2. 기하 검증 — 북50m+아래30m → yaw 0 / pitch +31 (parity 동일) ===\n");
  CoarseTracker c;
  {
    GpsFix f;
    f.iTOW = 5000000u; f.lat_i7 = P_LAT; f.lon_i7 = P_LON; f.alt_m = 100.0f;
    f.fixType = 3; f.valid = true;
    c.onFix(f, 1000);

    RocketPacket p;
    p.iTOW = 5000000u;                       // 지연 0
    p.lat_i7 = degToI7(LAT0 + 50.0 / 111320.0); p.lon_i7 = P_LON;
    p.alt_mm = 70000; p.velN_mms = 0; p.velE_mms = 0; p.velD_mms = 0;
    p.fixType = 3;
    c.onPacket(p, 1000);

    check("데이터 주입 후 ready", c.ready(1100));
    check("compute 성공", c.compute(q0, 1100, &yaw, &pitch));
    check("yaw 0", approx(yaw, 0), yaw, 0);
    check("pitch +31", approx(pitch, 31, 1), pitch, 31);
    c.compute(q90, 1100, &yaw, &pitch);
    check("페이로드 90° 회전 → yaw -90 (상쇄)", approx(yaw, -90), yaw, -90);
  }

  printf("=== 3. iTOW 지연 보정 — 로켓 북진 30m/s × 150ms → 각도 이동 ===\n");
  {
    // payload.iTOW 5000150 vs rocket 5000000 → 150ms, vN=30 → dN 50+4.5=54.5, dD 30
    GpsFix f = c.payload; f.iTOW = 5000150u;
    c.onFix(f, 1200);
    RocketPacket p = c.rocket; p.velN_mms = 30000;
    c.onPacket(p, 1200);
    c.compute(q0, 1300, &yaw, &pitch);
    // horizon = 지연(0.15s) + 서보리드(가드 OFF일 때만). 가드 상태에서 기대값 유도 → ENABLE flag 무관.
    //   ENABLE=0: 리드 살아 h=0.30 → dN=59 → 26.95° / ENABLE=1: 반응형가드 발동 리드드롭 h=0.15 → dN=54.5 → 28.84°
    const float  h3 = 0.150f + (c.inDeployGuard(1300) ? 0.0f : SERVO_LEAD_S);
    const double expect = atan2(30.0, 50.0 + 30.0 * (double)h3) * 180.0 / M_PI;
    check("지연 보정 + 서보리드 반영", approx(pitch, expect, 0.3), pitch, expect);
  }

  printf("=== 4. itowDelayS 방어 ===\n");
  {
    check("정상 150ms", approx(itowDelayS(5000150u, 5000000u), 0.150, 1e-4));
    check("로켓이 더 최신(-100ms) 허용", approx(itowDelayS(5000000u, 5000100u), -0.100, 1e-4));
    check("주 경계 랩: 150ms", approx(itowDelayS(100u, 604799950u), 0.150, 1e-4));
    check("비정상 큰 지연 → +2s 클램프", approx(itowDelayS(10000000u, 5000000u), 2.0, 1e-4));
    check("비정상 음수 → -1s 클램프", approx(itowDelayS(5000000u, 10000000u), -1.0, 1e-4));
  }

  printf("=== 5. 타임아웃 폴백 조건 (fsm.py: comm lost → HOLD) ===\n");
  {
    // 마지막 수신 t=1200. COMM_TIMEOUT 0.5s → 1701ms부터 comm 상실
    check("400ms 경과: 아직 ready", c.ready(1600));
    check("501ms 경과: comm 상실 → ready=false", !c.ready(1702));
    check("compute도 거부", !c.compute(q0, 1702, &yaw, &pitch));
    // 새 패킷 도착 → 회복 (fsm.py: comm recovered → COARSE)
    RocketPacket p = c.rocket;
    c.onPacket(p, 2000);
    check("새 패킷 → 회복", c.ready(2100));
    // fix가 오래되면 (GPS_FIX_TIMEOUT 1.5s) → 역시 불가
    check("fix 1.6s 경과 → ready=false", !c.ready(1200 + 1600 + 1));
  }

  printf("=== 6. 데이터 품질 거부 ===\n");
  {
    CoarseTracker d;
    GpsFix f = c.payload; f.valid = false;          // no-fix 페이로드
    d.onFix(f, 100);
    RocketPacket p = c.rocket;
    d.onPacket(p, 100);
    check("fix invalid → ready=false", !d.ready(200));
    f.valid = true; d.onFix(f, 300);
    p.fixType = 0; d.onPacket(p, 300);              // 로켓 no-fix
    check("로켓 fixType 0 → ready=false", !d.ready(400));
    p.fixType = 4; d.onPacket(p, 500);              // GNSS+DR은 허용
    check("로켓 fixType 4 → ready", d.ready(600));
  }

  printf("=== 7. 기압계 고도 주입 (고도기준 2026-07-21) ===\n");
  {
    CoarseTracker e;
    // 페이로드 GPS 고도는 일부러 엉뚱한 값(999m) → baro가 이걸 덮어써야 함
    GpsFix f; f.iTOW = 5000000u; f.lat_i7 = P_LAT; f.lon_i7 = P_LON; f.alt_m = 999.0f;
    f.fixType = 3; f.valid = true;
    e.onFix(f, 1000);
    RocketPacket p;
    p.iTOW = 5000000u;                       // 지연 0
    p.lat_i7 = degToI7(LAT0 + 50.0 / 111320.0); p.lon_i7 = P_LON;
    p.alt_mm = 70000;                        // 로켓 기압계 AGL 70m
    p.velN_mms = 0; p.velE_mms = 0; p.velD_mms = 0; p.fixType = 3;
    e.onPacket(p, 1000);

    // baro 미주입 → GPS 폴백(999m) → dD 엉망 → pitch ≠ 31
    e.compute(q0, 1100, &yaw, &pitch);
    check("baro 미주입 시 GPS 고도(999) 폴백 → pitch != 31", !approx(pitch, 31, 1), pitch, 31);

    // baro 주입: 페이로드 AGL 100m → dD = 100-70 = 30, 북 50 → pitch +31 (§2와 동일)
    e.onBaro(100.0f, 1100);
    check("baro 신선 → baroOk", e.baroOk(1100));
    check("baro 주입 후 compute 성공", e.compute(q0, 1150, &yaw, &pitch));
    check("baro 고도 사용 → pitch +31", approx(pitch, 31, 1), pitch, 31);
    check("baro 사용 시 yaw 0", approx(yaw, 0), yaw, 0);

    // 데이터 갱신하되 baro는 안 줌 → baro 만료(0.5s) → GPS(999) 폴백 복귀
    GpsFix f2 = f; f2.iTOW = 5000000u; e.onFix(f2, 2000);
    RocketPacket p2 = p; e.onPacket(p2, 2000);
    check("baro 나이 950ms > 0.5s → baroOk=false", !e.baroOk(2050));
    e.compute(q0, 2050, &yaw, &pitch);
    check("baro 만료 → GPS 폴백 → pitch != 31", !approx(pitch, 31, 1), pitch, 31);
  }

  printf("=== 8. 서보 리드타임 + 메인 전개 가드 (예측 업그레이드 2026-07-21) ===\n");
  {
    // 공통: 페이로드 LAT0/LON0, 로켓 북 50m. dD는 각 케이스에서 payload.alt로 30m 고정.
    const int32_t R_LAT = degToI7(LAT0 + 50.0 / 111320.0);

    // 8a) 순항(고도 밴드 밖, 첫 패킷) → 서보 리드 적용. 등속이면 예측이 더 앞으로.
    {
      CoarseTracker g;
      GpsFix f; f.iTOW=6000000u; f.lat_i7=P_LAT; f.lon_i7=P_LON; f.alt_m=630.0f;
      f.fixType=3; f.valid=true; g.onFix(f, 10000);
      RocketPacket p; p.iTOW=6000000u; p.lat_i7=R_LAT; p.lon_i7=P_LON;
      p.alt_mm=600000; p.velN_mms=20000; p.fixType=3;   // 600m AGL(밴드 밖), 북 20m/s
      g.onPacket(p, 10000);
      check("8a 순항: 전개 가드 OFF", !g.inDeployGuard(10050));
      g.compute(q0, 10050, &yaw, &pitch);
      // horizon=0.15 → dN=50+20*0.15=53, dD=630-600=30 → pitch=atan2(30,53)=29.51 (리드 없으면 31)
      check("8a 서보 리드 반영 → pitch ~29.5", approx(pitch, 29.51, 0.4), pitch, 29.51);
    }
    // 8b) 고도 게이트(로켓 450m 근처) → 리드 드롭 → 등속 예측 안 함.
    {
      CoarseTracker g;
      GpsFix f; f.iTOW=6000000u; f.lat_i7=P_LAT; f.lon_i7=P_LON; f.alt_m=480.0f;
      f.fixType=3; f.valid=true; g.onFix(f, 10000);
      RocketPacket p; p.iTOW=6000000u; p.lat_i7=R_LAT; p.lon_i7=P_LON;
      p.alt_mm=450000; p.velN_mms=20000; p.fixType=3;   // 450m AGL = 전개 밴드 안
      g.onPacket(p, 10000);
#if DEPLOY_GUARD_ENABLE
      check("8b 고도 게이트: 전개 밴드 안 → 가드 ON", g.inDeployGuard(10050));
      g.compute(q0, 10050, &yaw, &pitch);
      // 가드 → lead 0 → dN=50, dD=480-450=30 → pitch=atan2(30,50)=30.96 (8a와 구별됨)
      check("8b 가드 시 리드 드롭 → pitch ~31", approx(pitch, 30.96, 0.4), pitch, 30.96);
#else
      // [2025 IREC 실측] ENABLE=0: 밴드 안이어도 가드 미발동 → 리드(0.15s) 유지
      check("8b ENABLE=0: 밴드 안이어도 가드 OFF", !g.inDeployGuard(10050));
      g.compute(q0, 10050, &yaw, &pitch);
      // 리드 유지 → dN=50+20*0.15=53, dD=30 → pitch=atan2(30,53)=29.51 (8a와 동일)
      check("8b ENABLE=0: 리드 유지 → pitch ~29.5", approx(pitch, 29.51, 0.4), pitch, 29.51);
#endif
    }
    // 8c) 반응형 ΔV(속도 급변) → 리드 드롭, 유지시간 후 해제.
    {
      CoarseTracker g;
      GpsFix f; f.iTOW=6000000u; f.lat_i7=P_LAT; f.lon_i7=P_LON; f.alt_m=630.0f;
      f.fixType=3; f.valid=true; g.onFix(f, 10000);
      RocketPacket p; p.iTOW=6000000u; p.lat_i7=R_LAT; p.lon_i7=P_LON;
      p.alt_mm=600000; p.velN_mms=20000; p.fixType=3;   // 순항 600m
      g.onPacket(p, 10000);
      check("8c 첫 패킷: 가드 OFF", !g.inDeployGuard(10000));
      RocketPacket p2=p; p2.velN_mms=45000;             // ΔV=25 m/s > 15 → 반응형 트리거
      g.onPacket(p2, 10100);
      // 반응형 arming(onPacket의 tReactiveGuardUntil_)은 flag와 무관 → 항상 검증
      check("8c 속도 급변 → reactiveGuardActive ON", g.reactiveGuardActive(10150));
      check("8c 반응형 유지시간(1.5s) 후 해제", !g.reactiveGuardActive(10100 + 1500 + 1));
#if DEPLOY_GUARD_ENABLE
      check("8c 반응형 → inDeployGuard ON", g.inDeployGuard(10150));
      g.compute(q0, 10150, &yaw, &pitch);
      // 가드 → lead 0 → dN=50, dD=30 → pitch=atan2(30,50)=30.96
      check("8c 반응형 가드 시 리드 드롭 → pitch ~31", approx(pitch, 30.96, 0.4), pitch, 30.96);
      check("8c 유지시간(1.5s) 후 inDeployGuard 해제", !g.inDeployGuard(10100 + 1500 + 1));
#else
      // [2025 IREC 실측] ENABLE=0: 반응형 arm돼도 inDeployGuard OFF → 리드 유지
      check("8c ENABLE=0: 반응형 arm돼도 inDeployGuard OFF", !g.inDeployGuard(10150));
      g.compute(q0, 10150, &yaw, &pitch);
      // 리드 유지 → dN=50+45*0.15=56.75, dD=30 → pitch=atan2(30,56.75)=27.86
      check("8c ENABLE=0: 리드 유지 → pitch ~27.9", approx(pitch, 27.86, 0.4), pitch, 27.86);
#endif
    }
    // 8d) predictHorizonS 순수함수 경계 (파이썬과 동일해야 함)
    {
      check("8d 순항 horizon = 지연+리드", approx(predictHorizonS(0.12f, false), 0.27, 1e-4));
      check("8d 가드 horizon = 지연만", approx(predictHorizonS(0.12f, true), 0.12, 1e-4));
      check("8d 상한 클램프 +2s", approx(predictHorizonS(2.0f, false), 2.0, 1e-4));
      check("8d inDeployAltBand 450 안", inDeployAltBand(450.0f));
      check("8d inDeployAltBand 600 밖", !inDeployAltBand(600.0f));
    }
  }

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — COARSE 통합 코어가 규약대로 동작합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
