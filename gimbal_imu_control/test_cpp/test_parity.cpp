// ═══════════════════════════════════════════════════════════════
// test_parity.cpp — C++ 코어 로직을 PC에서 검증 (부품 불필요)
//
// 파이썬 시뮬레이터 test_vectors.py와 동일한 입력·기대값 사용.
// 컴파일·실행:
//   g++ -std=c++17 -I../gimbal_imu_control test_parity.cpp -o test_parity && ./test_parity
//
// 모두 통과하면 → C++ 코어가 파이썬과 수치적으로 동일 → 부품 도착 후
// 하드웨어 이슈와 알고리즘 이슈를 분리해서 디버깅 가능.
// ═══════════════════════════════════════════════════════════════
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "geometry.h"
#include "gimbal_controller.h"

static int PASS = 0, FAIL = 0;

void check(const char* name, bool cond, float got = 0, float want = 0) {
  if (cond) { PASS++; printf("  \u2713 %s\n", name); }
  else      { FAIL++; printf("  \u2717 %s  (got=%.3f want=%.3f)\n", name, got, want); }
}
bool approx(float a, float b, float tol = 0.5f) { return fabsf(a - b) < tol; }

int main() {
  float q[4], vB[3], yaw, pitch;

  printf("=== 1. Body 변환 + 짐벌각 부호 규약 (파이썬 test 4와 동일) ===\n");

  // 북향 페이로드, 정북 목표 → yaw 0, pitch 0
  eulerToQuat(0, 0, 0, q);
  float north[3] = {100, 0, 0};
  nedToBody(north, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("북향+정북 목표 -> yaw 0", approx(yaw, 0), yaw, 0);
  check("수평 목표 -> pitch 0", approx(pitch, 0), pitch, 0);

  // 정동 목표 → yaw +90
  float east[3] = {0, 100, 0};
  nedToBody(east, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("정동 목표 -> yaw +90 (시계+)", approx(yaw, 90), yaw, 90);

  // 정서 → yaw -90
  float west[3] = {0, -100, 0};
  nedToBody(west, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("정서 목표 -> yaw -90", approx(yaw, -90), yaw, -90);

  // 전방50+아래30 → pitch +31
  float fwd_down[3] = {50, 0, 30};
  nedToBody(fwd_down, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("전방50+아래30 -> pitch +31 (아래+)", approx(pitch, 31, 1), pitch, 31);

  // 동향 페이로드(yaw 90), 정북 목표 → yaw -90
  eulerToQuat(90, 0, 0, q);
  nedToBody(north, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("동향 페이로드+정북 목표 -> yaw -90", approx(yaw, -90), yaw, -90);

  // 바로 아래 → pitch +90, NaN 없음
  eulerToQuat(0, 0, 0, q);
  float below[3] = {0, 0, 50};
  nedToBody(below, q, vB);
  bodyToGimbal(vB, &yaw, &pitch);
  check("바로 아래 -> pitch +90, NaN 없음",
        approx(pitch, 90) && !std::isnan(yaw), pitch, 90);

  printf("=== 2. 쿼터니언<->오일러 왕복 (파이썬 test 3) ===\n");
  float cases[4][3] = {{0,0,0},{45,10,-5},{-120,30,15},{170,-5,0}};
  for (auto& c : cases) {
    eulerToQuat(c[0], c[1], c[2], q);
    float y2, p2, r2;
    quatToEuler(q, &y2, &p2, &r2);
    char name[64];
    snprintf(name, sizeof(name), "왕복 (%.0f,%.0f,%.0f)", c[0], c[1], c[2]);
    check(name, approx(y2, c[0], 0.01f) && approx(p2, c[1], 0.01f) && approx(r2, c[2], 0.01f));
  }

  printf("=== 3. STABILIZED_HOLD (파이썬 test 7) ===\n");
  float holdDir[3] = {1, 0, 0};   // 북쪽 수평 고정
  for (float py : {0.f, 30.f, 60.f, 90.f}) {
    eulerToQuat(py, 0, 0, q);
    holdDirection(holdDir, q, &yaw, &pitch);
    char name[64];
    snprintf(name, sizeof(name), "페이로드 %.0f도 회전 -> 짐벌 yaw %.0f도", py, -py);
    check(name, approx(yaw, -py), yaw, -py);
  }

  // capture -> hold 왕복
  eulerToQuat(10, 5, 0, q);
  float captured[3];
  captureHoldDirection(45, 20, q, captured);
  holdDirection(captured, q, &yaw, &pitch);
  check("capture->hold 왕복 (45,20)", approx(yaw, 45, 0.01f) && approx(pitch, 20, 0.01f));

  printf("=== 4. 명령 필터 (파이썬 test 8) ===\n");
  GimbalCommandFilter f;
  const float dt = 0.02f;   // 50Hz -> max step 3.6

  f.reset(0, 0);
  f.step(200, 50, dt);   // 200도 = wrap -160도 방향
  check("명령 정규화: 200=-160 방향, 첫 스텝 -3.6", approx(f.yawOut, -3.6f, 0.01f), f.yawOut, -3.6f);
  check("limit_flag 세워짐", f.limitFlag);

  for (int i = 0; i < 200; i++) f.step(200, 50, dt);
  check("수렴: yaw -160", approx(f.yawOut, -160, 0.01f), f.yawOut, -160);
  check("수렴: pitch +50", approx(f.pitchOut, 50, 0.01f), f.pitchOut, 50);

  f.reset(0, 0);
  for (int i = 0; i < 200; i++) f.step(175, 0, dt);
  check("한계 초과 175 -> +170 클램프", approx(f.yawOut, 170, 0.01f), f.yawOut, 170);

  f.reset(0, 0);
  for (int i = 0; i < 200; i++) f.step(0, -50, dt);
  check("pitch 하한 -10 클램프", approx(f.pitchOut, -10, 0.01f), f.pitchOut, -10);

  f.reset(170, 0);
  f.step(-170, 0, dt);
  check("dead zone: +170->-170 되돌아감 (첫 스텝 166.4)",
        approx(f.yawOut, 166.4f, 0.01f), f.yawOut, 166.4f);

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — C++ 코어가 파이썬 시뮬레이터와 일치합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
