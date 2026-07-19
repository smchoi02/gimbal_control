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
    const double expect = atan2(30.0, 54.5) * 180.0 / M_PI;   // ≈ 28.84°
    check("지연 보정 반영: pitch 31→28.8", approx(pitch, expect, 0.3), pitch, expect);
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

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — COARSE 통합 코어가 규약대로 동작합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
