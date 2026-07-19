// ===============================================================
// test_rocket_link.cpp — RocketPacket 링크를 PC에서 검증 (부품 불필요)
//
// gps_module_design.md §6 테스트 계획 3번:
//   "RocketPacket 인코드→디코드 왕복 일치, CRC가 손상 패킷 거부"
//
// 컴파일·실행:
//   g++ -std=c++17 -I../gimbal_imu_control test_rocket_link.cpp -o test_rocket_link && ./test_rocket_link
//
// 마지막 섹션은 풀체인 이음새 검증:
//   [로켓 GPS bytes] → UBX 파서 → GpsFix → RocketPacket → 와이어 34B
//   → RocketLinkParser → toGeo/toVel + iTOW 지연 → coarsePipeline → 짐벌각
// 즉 Phase 4 수신 경로 전체가 부품 없이 숫자로 맞는지 확인한다.
// ===============================================================
#include <cstdio>
#include <cmath>
#include <cstring>
#include "rocket_link.h"

static int PASS = 0, FAIL = 0;

void check(const char* name, bool cond, double got = 0, double want = 0) {
  if (cond) { PASS++; printf("  ✓ %s\n", name); }
  else      { FAIL++; printf("  ✗ %s  (got=%.7f want=%.7f)\n", name, got, want); }
}
bool approx(double a, double b, double tol = 0.5) { return fabs(a - b) < tol; }

// ── 합성 NAV-PVT 프레임 (test_ubx.cpp와 동일 헬퍼 — 풀체인용) ──
static void wrU4(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wrI4(uint8_t* p, int32_t v) { wrU4(p, (uint32_t)v); }

static size_t buildPvtFrame(uint32_t iTOW, double lat, double lon, double hmsl_m,
                            double vN, double vE, double vD,
                            uint8_t fixType, uint8_t flags, uint8_t numSV,
                            uint8_t* out, size_t cap) {
  uint8_t pl[92];
  memset(pl, 0, sizeof(pl));
  wrU4(pl + 0, iTOW);
  pl[20] = fixType; pl[21] = flags; pl[23] = numSV;
  wrI4(pl + 24, (int32_t)llround(lon * 1e7));
  wrI4(pl + 28, (int32_t)llround(lat * 1e7));
  wrI4(pl + 36, (int32_t)llround(hmsl_m * 1000.0));
  wrI4(pl + 48, (int32_t)llround(vN * 1000.0));
  wrI4(pl + 52, (int32_t)llround(vE * 1000.0));
  wrI4(pl + 56, (int32_t)llround(vD * 1000.0));
  return ubxMakeFrame(UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92, out, cap);
}

static int feedAll(RocketLinkParser& p, const uint8_t* d, size_t n, RocketPacket* out) {
  int got = 0;
  for (size_t i = 0; i < n; i++)
    if (p.feed(d[i], out)) got++;
  return got;
}

int main() {
  const double LAT0 = 37.5, LON0 = 127.0;   // 다른 테스트와 동일 기준점
  uint8_t wire[64], wire2[64];
  RocketPacket rx;

  printf("=== 1. CRC-16/CCITT-FALSE 독립 골든 벡터 ===\n");
  {
    // 표준 검증값: "123456789" → 0x29B1 (구현과 무관하게 정해진 값)
    const uint8_t s[] = {'1','2','3','4','5','6','7','8','9'};
    uint16_t c = crc16ccitt(s, 9);
    check("crc16(\"123456789\") = 0x29B1", c == 0x29B1, c, 0x29B1);
    check("crc16(빈 입력) = init 0xFFFF", crc16ccitt(s, 0) == 0xFFFF);
  }

  printf("=== 2. 와이어 레이아웃 고정 (§5.1 — 로켓 팀 공유 스펙) ===\n");
  {
    RocketPacket tx;
    tx.seq = 7; tx.fixType = 3;
    tx.iTOW = 0x11223344u;
    tx.lat_i7 = 375000000; tx.lon_i7 = 1270000000;
    tx.alt_mm = 2100000;                       // 2100 m
    tx.velN_mms = -1234; tx.velE_mms = 8000; tx.velD_mms = 30000;
    size_t n = rocketPacketEncode(tx, wire, sizeof(wire));
    check("패킷 크기 = 34", n == 34, (double)n, 34);
    check("magic 와이어 = 'R','K'", wire[0] == 0x52 && wire[1] == 0x4B);
    check("seq/fixType 오프셋 2/3", wire[2] == 7 && wire[3] == 3);
    check("iTOW little-endian (44 33 22 11)",
          wire[4] == 0x44 && wire[5] == 0x33 && wire[6] == 0x22 && wire[7] == 0x11);
    check("lat 오프셋 8 (LE 하위바이트 0xC0)", wire[8] == 0xC0);   // 375000000 = 0x165A0BC0
    check("crc = crc16(bytes 0..31)", rlRdU2(wire + 32) == crc16ccitt(wire, 32));
    check("버퍼 부족 시 0 반환", rocketPacketEncode(tx, wire, 33) == 0);
  }

  printf("=== 3. 인코드 → 디코드 루프백 (전 필드 왕복 일치) ===\n");
  {
    RocketPacket tx;
    tx.seq = 42; tx.fixType = 3;
    tx.iTOW = 432000150u;                      // 미션 중 임의 시각
    tx.lat_i7 = 375004492;                     // LAT0 + 약 50m
    tx.lon_i7 = 1270000000;
    tx.alt_mm = 2100000;
    tx.velN_mms = 0; tx.velE_mms = 8000; tx.velD_mms = 30000;
    size_t n = rocketPacketEncode(tx, wire, sizeof(wire));

    RocketLinkParser p;
    int got = feedAll(p, wire, n, &rx);
    check("루프백: 패킷 1개 수신", got == 1 && p.pktOk == 1, got, 1);
    check("seq/fixType 일치", rx.seq == 42 && rx.fixType == 3);
    check("iTOW 일치", rx.iTOW == 432000150u);
    check("lat_i7 정수 그대로 (규약 §2)", rx.lat_i7 == 375004492);
    check("lon_i7 일치", rx.lon_i7 == 1270000000);
    check("alt/vel 고정소수 일치",
          rx.alt_mm == 2100000 && rx.velN_mms == 0 &&
          rx.velE_mms == 8000 && rx.velD_mms == 30000);
  }

  printf("=== 4. CRC 손상 거부 + 재동기 ===\n");
  {
    RocketPacket tx;
    tx.seq = 1; tx.fixType = 3; tx.iTOW = 1000;
    tx.lat_i7 = 375000000; tx.lon_i7 = 1270000000; tx.alt_mm = 500000;
    size_t n = rocketPacketEncode(tx, wire, sizeof(wire));

    memcpy(wire2, wire, n);
    wire2[10] ^= 0xFF;                          // payload 오염 (CRC 불일치)
    RocketLinkParser p;
    int got = feedAll(p, wire2, n, &rx);
    check("오염 패킷 거부 (pktBad>=1)", got == 0 && p.pktBad >= 1, got, 0);
    got = feedAll(p, wire, n, &rx);
    check("거부 직후 정상 패킷 수신 (재동기)", got == 1, got, 1);

    // 쓰레기 프리픽스 (가짜 'R' 포함) 후 정상 패킷
    RocketLinkParser p2;
    const uint8_t junk[] = {0x00, 0x52, 0x00, 0xFF, 0x52, 0x52, 0x13};
    got = feedAll(p2, junk, sizeof(junk), &rx);
    got += feedAll(p2, wire, n, &rx);
    check("쓰레기 프리픽스 후 정상 수신", got == 1, got, 1);
  }

  printf("=== 5. misframe 복구 — payload 속 가짜 'R''K'에도 다음 패킷 생존 ===\n");
  {
    // pktX: velE_mms = 0x004B5200 → 와이어 오프셋 24..27 = 00 52 4B 00 (가짜 magic!)
    RocketPacket x;
    x.seq = 9; x.fixType = 3; x.iTOW = 5000;
    x.lat_i7 = 375000000; x.lon_i7 = 1270000000; x.alt_mm = 70000;
    x.velN_mms = 0; x.velE_mms = 0x004B5200; x.velD_mms = 30000;
    size_t nx = rocketPacketEncode(x, wire, sizeof(wire));

    RocketPacket good;
    good.seq = 10; good.fixType = 3; good.iTOW = 6000;
    good.lat_i7 = 375004492; good.lon_i7 = 1270000000; good.alt_mm = 2100000;
    good.velN_mms = 0; good.velE_mms = 8000; good.velD_mms = 30000;
    size_t ng = rocketPacketEncode(good, wire2, sizeof(wire2));

    // 스트림 중간 진입: pktX의 뒤쪽 24바이트(가짜 magic 포함)부터 수신 시작
    RocketLinkParser p;
    int got = feedAll(p, wire + 10, nx - 10, &rx);   // 헤드 유실 → misframe 유도
    got += feedAll(p, wire2, ng, &rx);               // 이어서 정상 패킷
    check("misframe 발생 (pktBad>=1)", p.pktBad >= 1);
    check("그래도 다음 패킷은 생존 (got=1)", got == 1, got, 1);
    check("생존 패킷 필드 정상 (seq=10, lat 일치)",
          rx.seq == 10 && rx.lat_i7 == 375004492);
  }

  printf("=== 6. seq 유실 추적 ===\n");
  {
    RocketLinkParser p;
    RocketPacket tx;
    tx.fixType = 3; tx.lat_i7 = 375000000; tx.lon_i7 = 1270000000;
    uint8_t seqs[] = {1, 2, 5, 255};
    for (uint8_t s : seqs) {
      tx.seq = s;
      size_t n = rocketPacketEncode(tx, wire, sizeof(wire));
      feedAll(p, wire, n, &rx);
    }
    // 1→2 (0 유실), 2→5 (2 유실), 5→255 (249 유실) = 251
    check("seq 간격 누적 (2→5: +2 등)", p.seqLost == 251, p.seqLost, 251);
    tx.seq = 0;                                 // 255 → 0: 랩, 유실 0
    size_t n = rocketPacketEncode(tx, wire, sizeof(wire));
    feedAll(p, wire, n, &rx);
    check("seq 255→0 랩은 유실 아님", p.seqLost == 251, p.seqLost, 251);
  }

  printf("=== 7. 품질·지연 헬퍼 ===\n");
  {
    RocketPacket p0; p0.fixType = 0;
    RocketPacket p3; p3.fixType = 3;
    RocketPacket p5; p5.fixType = 5;
    check("fixType 0 → usable=false", !rocketPacketUsable(p0));
    check("fixType 3 → usable=true",  rocketPacketUsable(p3));
    check("fixType 5(time only) → usable=false", !rocketPacketUsable(p5));

    check("iTOW 지연: 150ms", approx(commDelayFromItow(5000300u, 5000150u), 0.150, 1e-4));
    // 토요일→일요일 경계: 로켓 604,799,950ms → 페이로드 100ms = 실제 150ms
    check("iTOW 주 경계 랩 방어",
          approx(commDelayFromItow(100u, 604799950u), 0.150, 1e-4),
          commDelayFromItow(100u, 604799950u), 0.150);
  }

  printf("=== 8. 풀체인 이음새: GPS bytes → 패킷 → coarsePipeline (§7) ===\n");
  {
    // (1) '로켓측': 자기 M10 픽스(UBX bytes) → GpsFix → RocketPacket → 와이어
    uint8_t ubxFrame[128];
    UbxNavPvtParser ubx;
    GpsFix rocketFix;
    size_t n = buildPvtFrame(5000150u, LAT0 + 50.0 / 111320.0, LON0, 70.0,
                             30.0, 0.0, 0.0, 3, 0x01, 10, ubxFrame, sizeof(ubxFrame));
    for (size_t i = 0; i < n; i++) ubx.feed(ubxFrame[i], &rocketFix);
    RocketPacket txPkt = rocketPacketFromFix(rocketFix, 1);
    check("fromFix: lat_i7 정수 무손실", txPkt.lat_i7 == rocketFix.lat_i7);
    check("fromFix: vel m/s → mm/s (30000)", txPkt.velN_mms == 30000,
          txPkt.velN_mms, 30000);
    size_t nw = rocketPacketEncode(txPkt, wire, sizeof(wire));

    // (2) '페이로드측': 와이어 수신 + 자기 GPS 픽스
    RocketLinkParser link;
    int got = feedAll(link, wire, nw, &rx);
    check("링크 수신 + usable", got == 1 && rocketPacketUsable(rx));

    GpsFix payloadFix;
    n = buildPvtFrame(5000300u, LAT0, LON0, 100.0, 0, 0, 0, 3, 0x01, 11,
                      ubxFrame, sizeof(ubxFrame));
    for (size_t i = 0; i < n; i++) ubx.feed(ubxFrame[i], &payloadFix);

    // (3) 지연 보정 없이: parity 테스트 8과 동일 기하 → yaw 0, pitch +31
    GeoPoint payloadGeo = {payloadFix.lat_i7, payloadFix.lon_i7, payloadFix.alt_m};
    float q[4], yaw, pitch;
    eulerToQuat(0, 0, 0, q);
    RocketPacket still = rx; still.velN_mms = 0;       // 정지 가정으로 기하만 확인
    coarsePipeline(rocketPacketToGeo(still), rocketPacketToVel(still),
                   payloadGeo, q, 0.0f, &yaw, &pitch);
    check("풀체인: 북50+아래30 → yaw 0", approx(yaw, 0), yaw, 0);
    check("풀체인: pitch +31", approx(pitch, 31, 1), pitch, 31);

    // (4) iTOW 동기 지연 보정 (§5.2 방법1): 150ms × 북 30m/s → +4.5m
    float delay = commDelayFromItow(payloadFix.iTOW, rx.iTOW);
    check("풀체인 iTOW 지연 = 150ms", approx(delay, 0.150, 1e-4), delay, 0.150);
    GeoPoint pred = predictPosition(rocketPacketToGeo(rx), rocketPacketToVel(rx), delay);
    float dNED[3];
    geoToNed(pred, rocketPacketToGeo(rx), dNED);
    check("지연 보정 +4.5m 북쪽", approx(dNED[0], 4.5, 0.05), dNED[0], 4.5);

    // (5) 페이로드 90° 회전 시 상쇄 (parity 테스트 8 후반과 동일)
    eulerToQuat(90, 0, 0, q);
    coarsePipeline(rocketPacketToGeo(still), rocketPacketToVel(still),
                   payloadGeo, q, 0.0f, &yaw, &pitch);
    check("풀체인: 페이로드 90° 회전 → yaw -90", approx(yaw, -90), yaw, -90);
  }

  printf("\n=== 결과: %d passed, %d failed ===\n", PASS, FAIL);
  if (FAIL == 0) printf("ALL PASSED — RocketPacket 링크가 설계문서 §5 규약대로 동작합니다.\n");
  return FAIL == 0 ? 0 : 1;
}
