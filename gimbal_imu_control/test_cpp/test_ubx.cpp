// ===============================================================
// test_ubx.cpp — UBX-NAV-PVT 파서를 PC에서 검증 (부품 불필요)
//
// gps_module_design.md §6 테스트 계획 2번:
//   "합성/캡처한 NAV-PVT 바이트버퍼 주입 → 필드 디코드·체크섬 검증
//    (고의 손상 프레임 거부 확인)"
//
// 컴파일·실행:
//   g++ -std=c++17 -I../gimbal_imu_control test_ubx.cpp -o test_ubx && ./test_ubx
//
// 마지막 섹션은 파서 출력(GpsFix) → geo.h 파이프라인 연결(이음새 §7)까지
// 확인한다 — 파싱된 값이 단위·형식 그대로 coarsePipeline에 들어감을 보장.
// ===============================================================
#include <cstdio>
#include <cmath>
#include <cstring>
#include "gps_ublox.h"
#include "geo.h"

static int PASS = 0, FAIL = 0;

void check(const char* name, bool cond, double got = 0, double want = 0) {
  if (cond) { PASS++; printf("  ✓ %s\n", name); }
  else      { FAIL++; printf("  ✗ %s  (got=%.7f want=%.7f)\n", name, got, want); }
}
bool approx(double a, double b, double tol = 0.5) { return fabs(a - b) < tol; }

// ── 합성 NAV-PVT 프레임 생성 (little-endian 기록) ──
static void wrU4(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wrI4(uint8_t* p, int32_t v) { wrU4(p, (uint32_t)v); }

struct PvtParams {
  uint32_t iTOW;
  double   lat_deg, lon_deg, hmsl_m;
  double   vN, vE, vD;          // m/s
  uint8_t  fixType, flags, numSV;
};

static size_t buildPvtFrame(const PvtParams& s, uint8_t* out, size_t cap) {
  uint8_t pl[92];
  memset(pl, 0, sizeof(pl));
  wrU4(pl + 0,  s.iTOW);
  pl[20] = s.fixType;
  pl[21] = s.flags;
  pl[23] = s.numSV;
  wrI4(pl + 24, (int32_t)llround(s.lon_deg * 1e7));       // lon (1e-7 deg)
  wrI4(pl + 28, (int32_t)llround(s.lat_deg * 1e7));       // lat
  wrI4(pl + 32, (int32_t)llround((s.hmsl_m + 30.0) * 1000.0)); // height(타원체) — 파서가 안 읽는 필드
  wrI4(pl + 36, (int32_t)llround(s.hmsl_m * 1000.0));     // hMSL (mm)
  wrI4(pl + 48, (int32_t)llround(s.vN * 1000.0));         // velN (mm/s)
  wrI4(pl + 52, (int32_t)llround(s.vE * 1000.0));
  wrI4(pl + 56, (int32_t)llround(s.vD * 1000.0));
  return ubxMakeFrame(UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92, out, cap);
}

static int feedAll(UbxNavPvtParser& p, const uint8_t* d, size_t n, GpsFix* fix) {
  int got = 0;
  for (size_t i = 0; i < n; i++)
    if (p.feed(d[i], fix)) got++;
  return got;
}

int main() {
  const double LAT0 = 37.5, LON0 = 127.0;   // test_vectors.py / test_parity.cpp와 동일 기준점
  uint8_t frame[128], frame2[128];
  GpsFix fix;

  printf("=== 1. 체크섬 독립 골든 벡터 (손으로 계산한 ACK-ACK) ===\n");
  {
    // ACK-ACK(0x05 0x01), payload = {0x06, 0x8A} (CFG-VALSET에 대한 ACK)
    // CK_A=0x98, CK_B=0xC1 — 빌더와 무관하게 수기 계산한 값 (Fletcher-8 자체 검증)
    const uint8_t golden[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xC1};
    UbxNavPvtParser p;
    int got = feedAll(p, golden, sizeof(golden), &fix);
    check("ACK 프레임: PVT 아님 → 픽스 없음", got == 0, got, 0);
    check("체크섬 통과 → framesOther=1, Bad=0", p.framesOther == 1 && p.framesBad == 0);
    check("ACK 식별: cls/id = 0x05/0x01", p.lastOtherCls == 0x05 && p.lastOtherId == 0x01);
    check("ACK 대상 = 0x06 0x8A (VALSET)", p.lastOtherPl0 == 0x06 && p.lastOtherPl1 == 0x8A);

    // 같은 프레임의 빌더 출력과 수기 골든 벡터가 일치하는지 (빌더 체크섬 교차 검증)
    const uint8_t ackPl[] = {0x06, 0x8A};
    size_t n = ubxMakeFrame(0x05, 0x01, ackPl, 2, frame, sizeof(frame));
    check("빌더 출력 == 수기 골든 벡터", n == 10 && memcmp(frame, golden, 10) == 0);
  }

  printf("=== 2. NAV-PVT 정상 파싱 — 전 필드 디코드 ===\n");
  {
    PvtParams s = {123456789u, LAT0, LON0, 100.0, 30.0, -5.0, 12.5, 3, 0x01, 12};
    size_t n = buildPvtFrame(s, frame, sizeof(frame));
    check("프레임 크기 = 92+8", n == 100, (double)n, 100);

    UbxNavPvtParser p;
    int got = feedAll(p, frame, n, &fix);
    check("완결 프레임 → true 1회", got == 1 && p.framesOk == 1, got, 1);
    check("iTOW", fix.iTOW == 123456789u, fix.iTOW, 123456789.0);
    check("lat_i7 = 375000000 (int32 그대로)", fix.lat_i7 == 375000000, fix.lat_i7, 375000000);
    check("lon_i7 = 1270000000", fix.lon_i7 == 1270000000, fix.lon_i7, 1270000000);
    check("hMSL 100000mm → 100m", approx(fix.alt_m, 100.0, 0.001), fix.alt_m, 100);
    check("velN 30000mm/s → 30m/s", approx(fix.vN, 30.0, 0.001), fix.vN, 30);
    check("velE → -5m/s", approx(fix.vE, -5.0, 0.001), fix.vE, -5);
    check("velD → +12.5m/s (아래+)", approx(fix.vD, 12.5, 0.001), fix.vD, 12.5);
    check("fixType/numSV", fix.fixType == 3 && fix.numSV == 12);
    check("3D fix + gnssFixOK → valid", fix.valid);
  }

  printf("=== 3. 재동기: 쓰레기 바이트·손상 프레임 뒤에도 복구 ===\n");
  {
    PvtParams s = {1000u, LAT0, LON0, 50.0, 0, 0, 0, 3, 0x01, 8};
    size_t n = buildPvtFrame(s, frame, sizeof(frame));

    // (a) 쓰레기 프리픽스 (가짜 0xB5 포함) 후 정상 프레임
    UbxNavPvtParser p;
    const uint8_t junk[] = {0x47, 0xB5, 0x13, 0x00, 0xFF, 0xB5, 0xB5};
    int got = feedAll(p, junk, sizeof(junk), &fix);
    got += feedAll(p, frame, n, &fix);
    check("쓰레기 프리픽스 후 정상 파싱", got == 1, got, 1);

    // (b) payload 1바이트 오염 (체크섬 불일치) → 폐기, 다음 프레임은 정상
    memcpy(frame2, frame, n);
    frame2[10] ^= 0xFF;
    UbxNavPvtParser p2;
    got = feedAll(p2, frame2, n, &fix);
    check("오염 프레임 거부 (framesBad=1)", got == 0 && p2.framesBad == 1);
    got = feedAll(p2, frame, n, &fix);
    check("거부 직후 정상 프레임 파싱 (재동기)", got == 1 && p2.framesOk == 1, got, 1);

    // (c) 프레임이 중간에서 잘린 뒤 새 프레임들 → 첫 프레임 잔여를 먹더라도 결국 복구
    UbxNavPvtParser p3;
    got = feedAll(p3, frame, 40, &fix);          // 40바이트에서 절단
    got += feedAll(p3, frame, n, &fix);          // 이 프레임은 잔여 payload로 소모될 수 있음
    got += feedAll(p3, frame, n, &fix);          // 다음 프레임에서는 반드시 복구
    check("절단 프레임 후 복구 (got=1, Bad>=1)", got == 1 && p3.framesBad >= 1, got, 1);
  }

  printf("=== 4. valid 판정 — fixType·gnssFixOK ===\n");
  {
    UbxNavPvtParser p;
    PvtParams s = {2000u, LAT0, LON0, 50.0, 0, 0, 0, 0, 0x00, 0};   // no fix
    size_t n = buildPvtFrame(s, frame, sizeof(frame));
    int got = feedAll(p, frame, n, &fix);
    check("no fix: 프레임은 수신됨(true)", got == 1, got, 1);
    check("no fix → valid=false", !fix.valid);

    s.fixType = 3; s.flags = 0x00;                                   // 3D인데 gnssFixOK=0
    n = buildPvtFrame(s, frame, sizeof(frame));
    feedAll(p, frame, n, &fix);
    check("gnssFixOK=0 → valid=false", !fix.valid);

    s.fixType = 5; s.flags = 0x01;                                   // time-only fix
    n = buildPvtFrame(s, frame, sizeof(frame));
    feedAll(p, frame, n, &fix);
    check("fixType=5(time only) → valid=false (설계문서 >=3 강화)", !fix.valid);

    s.fixType = 4; s.flags = 0x01;                                   // GNSS+DR
    n = buildPvtFrame(s, frame, sizeof(frame));
    feedAll(p, frame, n, &fix);
    check("fixType=4(GNSS+DR)+fixOK → valid=true", fix.valid);
  }

  printf("=== 5. CFG-VALSET 빌더 (§3.3 초기화 프레임) ===\n");
  {
    UbxValsetBuilder v;                          // RAM layer
    v.addU1(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
    v.addBool(CFG_UART1OUTPROT_NMEA, false);
    v.addU2(CFG_RATE_MEAS, 100);                 // 10Hz
    v.addU2(CFG_RATE_NAV, 1);
    size_t n = v.frame(frame, sizeof(frame));
    // payload = 4(헤더) + 5 + 5 + 6 + 6 = 26 → 프레임 26+8 = 34
    check("VALSET 프레임 크기 = 34", n == 34, (double)n, 34);
    check("cls/id = 0x06/0x8A", frame[2] == 0x06 && frame[3] == 0x8A);
    check("version=0, layers=RAM", frame[6] == 0x00 && frame[7] == 0x01);
    check("첫 키 little-endian (07 00 91 20)",
          frame[10] == 0x07 && frame[11] == 0x00 && frame[12] == 0x91 && frame[13] == 0x20);
    check("RATE_MEAS 값 100 (0x64 0x00)", frame[24] == 0x64 && frame[25] == 0x00);

    // 파서에 통과시켜 체크섬 자기 검증 (비-PVT → Other, Bad=0)
    UbxNavPvtParser p;
    feedAll(p, frame, n, &fix);
    check("VALSET 체크섬 유효 (파서 통과)", p.framesOther == 1 && p.framesBad == 0);
  }

  printf("=== 6. 이음새: 파서 출력 → geo.h 파이프라인 (§7) ===\n");
  {
    // parity 테스트 8과 동일 기하: 로켓 = 북 50m + 아래 30m
    PvtParams rocketP  = {5000150u, LAT0 + 50.0 / 111320.0, LON0, 70.0, 0, 0, 0, 3, 0x01, 10};
    PvtParams payloadP = {5000300u, LAT0, LON0, 100.0, 0, 0, 0, 3, 0x01, 11};
    GpsFix rocketFix, payloadFix;
    UbxNavPvtParser p;
    size_t n = buildPvtFrame(rocketP, frame, sizeof(frame));
    feedAll(p, frame, n, &rocketFix);
    n = buildPvtFrame(payloadP, frame, sizeof(frame));
    feedAll(p, frame, n, &payloadFix);
    check("두 픽스 연속 수신", p.framesOk == 2);

    GeoPoint rocket  = {rocketFix.lat_i7,  rocketFix.lon_i7,  rocketFix.alt_m};
    GeoPoint payload = {payloadFix.lat_i7, payloadFix.lon_i7, payloadFix.alt_m};
    float q[4], yaw, pitch;
    eulerToQuat(0, 0, 0, q);
    coarsePipeline(rocket, NedVel{rocketFix.vN, rocketFix.vE, rocketFix.vD},
                   payload, q, 0.0f, &yaw, &pitch);
    check("파싱값 → coarsePipeline: yaw 0", approx(yaw, 0), yaw, 0);
    check("파싱값 → coarsePipeline: pitch +31", approx(pitch, 31, 1), pitch, 31);

    // iTOW 시각 동기(§5.2 방법1): delay = (payload.iTOW - rocket.iTOW) ms
    float delay_s = (float)(payloadFix.iTOW - rocketFix.iTOW) * 0.001f;
    check("iTOW 지연 = 150ms", approx(delay_s, 0.150, 1e-4), delay_s, 0.150);
    PvtParams movingP = {6000000u, LAT0, LON0, 100.0, 30.0, 0, 0, 3, 0x01, 10};
    GpsFix movingFix;
    n = buildPvtFrame(movingP, frame, sizeof(frame));
    feedAll(p, frame, n, &movingFix);
    GeoPoint mv = {movingFix.lat_i7, movingFix.lon_i7, movingFix.alt_m};
    GeoPoint pr = predictPosition(mv, NedVel{movingFix.vN, movingFix.vE, movingFix.vD}, delay_s);
    float dNED[3];
    geoToNed(pr, mv, dNED);
    check("파싱된 vN=30 × 150ms → +4.5m 예측", approx(dNED[0], 4.5, 0.05), dNED[0], 4.5);

    // int32 정밀도가 파서를 거쳐도 보존되는가: 1e-7 deg 차이 = 1.1132cm
    PvtParams aP = {7000000u, 37.5000000, LON0, 0.0, 0, 0, 0, 3, 0x01, 10};
    PvtParams bP = {7000100u, 37.5000001, LON0, 0.0, 0, 0, 0, 3, 0x01, 10};
    GpsFix aF, bF;
    n = buildPvtFrame(aP, frame, sizeof(frame));  feedAll(p, frame, n, &aF);
    n = buildPvtFrame(bP, frame, sizeof(frame));  feedAll(p, frame, n, &bF);
    check("1e-7deg가 int32 1단위로 보존", bF.lat_i7 - aF.lat_i7 == 1,
          (double)(bF.lat_i7 - aF.lat_i7), 1);
    GeoPoint aG = {aF.lat_i7, aF.lon_i7, aF.alt_m}, bG = {bF.lat_i7, bF.lon_i7, bF.alt_m};
    geoToNed(bG, aG, dNED);
    check("파서 경유 후에도 1.1132cm 분해", approx(dNED[0], 0.011132, 1e-4), dNED[0], 0.011132);
  }

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — UBX 파서가 설계문서 §3 규약대로 동작합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
