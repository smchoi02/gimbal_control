// ===============================================================
// test_baro.cpp — 기압계 고도 코어(baro.h) PC 검증 (부품 불필요)
//   [고도기준 2026-07-21]
//   · 기압-고도식(baroAltitudeAgl) 물리값 확인
//   · BaroAgl 0점(zero/zeroAverage) 동작
//   · 두 유닛(로켓·페이로드)이 같은 발사대 0점 → 상대 고도차 정합
//
// 컴파일·실행:
//   g++ -std=c++17 -I../gimbal_imu_control test_baro.cpp -o test_baro && ./test_baro
// ===============================================================
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "baro.h"

static int PASS = 0, FAIL = 0;
void check(const char* name, bool cond, double got = 0, double want = 0) {
  if (cond) { PASS++; printf("  ✓ %s\n", name); }
  else      { FAIL++; printf("  ✗ %s  (got=%.4f want=%.4f)\n", name, got, want); }
}
bool approx(double a, double b, double tol) { return fabs(a - b) < tol; }

// 표준대기 역함수: 고도 h[m] → 기압[Pa] (테스트 입력 생성용)
static float presFromAlt(float h, float p0) {
  return p0 * powf(1.0f - h / BARO_COEF, 1.0f / BARO_EXP);
}

int main() {
  const float P0 = 101325.0f;

  printf("=== 1. 기압-고도식 기본값 ===\n");
  {
    check("P==P0 → 0m", approx(baroAltitudeAgl(P0, P0), 0.0, 1e-3), baroAltitudeAgl(P0, P0), 0);
    check("P0=0 방어 → 0", approx(baroAltitudeAgl(P0, 0.0f), 0.0, 1e-9));
    check("P=0 방어 → 0", approx(baroAltitudeAgl(0.0f, P0), 0.0, 1e-9));
    // 10% 기압 저하 ≈ 약 880~900m
    float h10 = baroAltitudeAgl(0.9f * P0, P0);
    check("P=0.9·P0 → ~884m", approx(h10, 884.0, 5.0), h10, 884.0);
  }

  printf("=== 2. 왕복(고도→기압→고도) 일치 ===\n");
  {
    for (float h : {5.0f, 100.0f, 305.0f, 1000.0f}) {
      float p = presFromAlt(h, P0);
      float back = baroAltitudeAgl(p, P0);
      char nm[64]; snprintf(nm, sizeof(nm), "h=%.0fm 왕복 복원", h);
      check(nm, approx(back, h, 0.05), back, h);
    }
  }

  printf("=== 3. BaroAgl 0점(zero) ===\n");
  {
    BaroAgl b;
    check("미영점 시 zeroed=false", !b.zeroed);
    check("미영점 폴백 P0 = 표준기압", approx(b.p0Pa, 101325.0, 1e-3));
    float pad = 100200.0f;                 // 발사대 지상 기압(해면과 다름)
    b.zero(pad);
    check("zero 후 zeroed=true", b.zeroed);
    check("0점에서 altitude=0", approx(b.altitude(pad), 0.0, 1e-3), b.altitude(pad), 0);
    float p100 = presFromAlt(100.0f, pad); // 지상 대비 100m 상승 시 기압
    check("지상+100m → altitude ~100", approx(b.altitude(p100), 100.0, 0.05), b.altitude(p100), 100.0);
    b.zero(-5.0f);                          // 잘못된 값 방어
    check("음수 기압 zero 무시", approx(b.p0Pa, pad, 1e-3));
  }

  printf("=== 4. zeroAverage(지상 노이즈 평균) ===\n");
  {
    BaroAgl b;
    float samples[5] = {100190.0f, 100210.0f, 100200.0f, 100205.0f, 100195.0f};
    b.zeroAverage(samples, 5);
    check("평균 0점 = 100200", approx(b.p0Pa, 100200.0, 1e-1), b.p0Pa, 100200.0);
    b.zeroAverage(nullptr, 0);              // 방어: 무시
    check("n=0/null 무시", approx(b.p0Pa, 100200.0, 1e-1));
  }

  printf("=== 5. 두 유닛 같은 발사대 0점 → 상대 고도차 정합 (핵심) ===\n");
  {
    // 로켓·페이로드가 같은 발사대(같은 지상 기압)에서 각자 0점.
    // 실제로는 센서 개체차로 P0가 미세히 다를 수 있으나, 같은 지상면이면 근사 일치.
    const float padPa = 100200.0f;
    BaroAgl payload, rocket;
    payload.zero(padPa);
    rocket.zero(padPa + 8.0f);     // 개체차/노이즈로 8Pa(≈0.7m) 어긋난 상황

    // 물리 진실: 페이로드 지상 5m, 로켓 상공 305m → dD(payload-rocket) = -300m
    float pPayload = presFromAlt(5.0f, padPa);
    float pRocket  = presFromAlt(305.0f, padPa);
    float aglP = payload.altitude(pPayload);
    float aglR = rocket.altitude(pRocket);
    float dD = aglP - aglR;                 // geo.h geoToNed의 dD 정의와 동일

    check("페이로드 AGL ~5m", approx(aglP, 5.0, 0.1), aglP, 5.0);
    check("로켓 AGL ~305m", approx(aglR, 305.0, 1.0), aglR, 305.0);
    // 8Pa 개체차 → 약 0.7m 편의. dD는 -300m 근방(±1m)이면 합격.
    check("상대 고도차 dD ≈ -300m (개체차 무시 가능)", approx(dD, -300.0, 1.5), dD, -300.0);
  }

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — 기압계 AGL 코어가 규약대로 동작합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
