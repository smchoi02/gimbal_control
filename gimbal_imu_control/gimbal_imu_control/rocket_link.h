#pragma once
// ═══════════════════════════════════════════════════════════════
// rocket_link.h — 로켓 위치 패킷(RocketPacket) 와이어 포맷 + CRC16 + 파서
//                 (Phase 4 로켓 위치 수신 — gps_module_design.md §5)
//
// 담당: GPS 파트 (CYN)
//
// Arduino 비의존 (표준 C++만) → PC에서 g++로 단위시험 가능.
//   test_cpp/test_rocket_link.cpp 가 인코드→디코드 루프백으로 검증한다.
//
// 전송 매체(LoRa 등)는 이 계층 뒤에 숨는다 (§5.3) — 이 파일은 바이트열만
// 다루므로 어떤 모듈이 오든 그대로 재사용. 인코더는 로켓측 송신기에도
// 그대로 가져가 쓴다 (이 헤더가 곧 패킷 스펙 — 로켓 팀과 공유).
//
// ── 와이어 포맷 (little-endian, 고정 34바이트) — §5.1 확정안 ──
//   off size field
//   0   2   magic     0x4B52 (와이어 순서 'R' 'K')
//   2   1   seq       시퀀스 (유실·순서 감지)
//   3   1   fixType   로켓 GPS 품질 (3=3D, 4=GNSS+DR만 사용 가능)
//   4   4   iTOW      로켓 GPS 주중 시각 [ms] — 지연 보정의 핵심 (§5.2)
//   8   4   lat_i7    위도 1e-7 deg (int32 그대로 — 규약 §2, float 금지)
//   12  4   lon_i7    경도 1e-7 deg
//   16  4   alt_mm    기압계 AGL [mm] — 발사대 0점 기준 (§3.2 결정 2026-07-21)
//                     ※ 와이어 포맷·필드명 불변, 의미만 hMSL→기압계 AGL.
//                       로켓측이 이 칸을 자기 기압계 AGL로 채운다.
//   20  4   velN_mms  NED 속도 [mm/s]
//   24  4   velE_mms
//   28  4   velD_mms  (아래+)
//   32  2   crc       CRC-16/CCITT-FALSE, bytes 0..31 대상
//
// ★ §8.2 로켓 팀 합의 항목: 이 표 + CRC 사양(poly 0x1021, init 0xFFFF,
//   반사 없음, xorout 0, 검증값 "123456789"→0x29B1)을 그대로 공유할 것.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include <stddef.h>
#include <string.h>     // memmove (재동기 시프트)
#include <math.h>       // lroundf
#include "geo.h"        // GeoPoint, NedVel — coarsePipeline 이음새 (§7)
#include "gps_ublox.h"  // GpsFix — 로켓측 송신 헬퍼 + LE 리더 재사용

static const uint16_t ROCKET_MAGIC      = 0x4B52;  // LE로 쓰면 'R'(0x52) 'K'(0x4B)
static const uint8_t  ROCKET_MAGIC0     = 0x52;    // 'R'
static const uint8_t  ROCKET_MAGIC1     = 0x4B;    // 'K'
static const size_t   ROCKET_PACKET_LEN = 34;
static const uint32_t GPS_MS_PER_WEEK   = 604800000u;  // iTOW 랩 주기

struct RocketPacket {
  uint16_t magic   = ROCKET_MAGIC;
  uint8_t  seq     = 0;
  uint8_t  fixType = 0;
  uint32_t iTOW    = 0;      // [ms]
  int32_t  lat_i7  = 0;      // 1e-7 deg — 전 구간 정수 유지 (규약 §2)
  int32_t  lon_i7  = 0;
  int32_t  alt_mm  = 0;      // 기압계 AGL [mm] (발사대 0점 — §3.2 2026-07-21)
  int32_t  velN_mms = 0;     // [mm/s]
  int32_t  velE_mms = 0;
  int32_t  velD_mms = 0;
  uint16_t crc     = 0;      // 수신 시 채워짐 (인코드 시 자동 계산)
};

// ── CRC-16/CCITT-FALSE (비트 단위 — 34B@10Hz엔 테이블 불필요) ──
inline uint16_t crc16ccitt(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
  }
  return crc;
}

// ── little-endian 기록 (읽기는 gps_ublox.h의 ubxRdU4/ubxRdI4 재사용) ──
inline void rlWrU2(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
inline void rlWrU4(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;         p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
inline void rlWrI4(uint8_t* p, int32_t v) { rlWrU4(p, (uint32_t)v); }
inline uint16_t rlRdU2(const uint8_t* p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// ═══════════════════════════════════════════════════════════════
// 인코드 (로켓측 송신) — 반환: 34 (버퍼 부족 시 0)
//   구조체를 memcpy하지 않고 명시적 바이트 직렬화 → 컴파일러 패딩·
//   엔디안 차이에 무관 (로켓측이 다른 MCU여도 동일 와이어 보장).
// ═══════════════════════════════════════════════════════════════
inline size_t rocketPacketEncode(const RocketPacket& p, uint8_t* out, size_t cap) {
  if (cap < ROCKET_PACKET_LEN) return 0;
  rlWrU2(out + 0, ROCKET_MAGIC);
  out[2] = p.seq;
  out[3] = p.fixType;
  rlWrU4(out + 4,  p.iTOW);
  rlWrI4(out + 8,  p.lat_i7);
  rlWrI4(out + 12, p.lon_i7);
  rlWrI4(out + 16, p.alt_mm);
  rlWrI4(out + 20, p.velN_mms);
  rlWrI4(out + 24, p.velE_mms);
  rlWrI4(out + 28, p.velD_mms);
  rlWrU2(out + 32, crc16ccitt(out, 32));
  return ROCKET_PACKET_LEN;
}

// 디코드 — buf는 magic·CRC 검증이 끝난 34바이트 (파서 내부에서 호출)
inline void rocketPacketDecode(const uint8_t* b, RocketPacket* p) {
  p->magic    = rlRdU2(b + 0);
  p->seq      = b[2];
  p->fixType  = b[3];
  p->iTOW     = ubxRdU4(b + 4);
  p->lat_i7   = ubxRdI4(b + 8);
  p->lon_i7   = ubxRdI4(b + 12);
  p->alt_mm   = ubxRdI4(b + 16);
  p->velN_mms = ubxRdI4(b + 20);
  p->velE_mms = ubxRdI4(b + 24);
  p->velD_mms = ubxRdI4(b + 28);
  p->crc      = rlRdU2(b + 32);
}

// ═══════════════════════════════════════════════════════════════
// RocketLinkParser — 바이트 스트림 → 검증된 RocketPacket
//   · magic 헌팅 → 34B 수집 → CRC 검증 → 디코드
//   · CRC 불일치 시: 버퍼 내부를 재스캔(1바이트 시프트)해 재동기.
//     payload에 우연히 'R''K'가 나와 misframe 되어도, 진짜 패킷 시작이
//     버퍼 안에 남아 있으므로 다음 패킷을 잃지 않는다 (테스트 5 검증).
// ═══════════════════════════════════════════════════════════════
class RocketLinkParser {
public:
  uint32_t pktOk   = 0;   // CRC 통과 패킷 수
  uint32_t pktBad  = 0;   // CRC 불일치 (misframe 포함)
  uint32_t seqLost = 0;   // seq 간격 누적 = 유실 추정치 (G-B 진단용)

  void reset() { n_ = 0; haveSeq_ = false; }

  // 1바이트 공급. 완결·CRC 통과 패킷이면 *out 채우고 true.
  bool feed(uint8_t b, RocketPacket* out) {
    buf_[n_++] = b;

    // magic 헌팅 (앞 2바이트는 즉시 판정 → 쓰레기에 버퍼 낭비 없음)
    if (n_ == 1) {
      if (buf_[0] != ROCKET_MAGIC0) n_ = 0;
      return false;
    }
    if (n_ == 2) {
      if (buf_[1] != ROCKET_MAGIC1) {
        n_ = (buf_[1] == ROCKET_MAGIC0) ? 1 : 0;   // 'R' 연속이면 유지
        if (n_ == 1) buf_[0] = ROCKET_MAGIC0;
      }
      return false;
    }
    if (n_ < ROCKET_PACKET_LEN) return false;

    // 34바이트 확보 → CRC 검증
    if (crc16ccitt(buf_, 32) == rlRdU2(buf_ + 32)) {
      rocketPacketDecode(buf_, out);
      n_ = 0;
      pktOk++;
      if (haveSeq_) seqLost += (uint8_t)(out->seq - lastSeq_ - 1);  // uint8 랩 안전
      lastSeq_ = out->seq;
      haveSeq_ = true;
      return true;
    }
    pktBad++;
    resyncShift();
    return false;
  }

private:
  uint8_t  buf_[ROCKET_PACKET_LEN];
  uint16_t n_ = 0;
  bool     haveSeq_ = false;
  uint8_t  lastSeq_ = 0;

  // CRC 실패한 34B 버퍼 안에서 다음 magic 후보를 찾아 앞으로 당김
  void resyncShift() {
    for (uint16_t i = 1; i < ROCKET_PACKET_LEN; i++) {
      if (buf_[i] == ROCKET_MAGIC0 &&
          (i == ROCKET_PACKET_LEN - 1 || buf_[i + 1] == ROCKET_MAGIC1)) {
        memmove(buf_, buf_ + i, ROCKET_PACKET_LEN - i);
        n_ = (uint16_t)(ROCKET_PACKET_LEN - i);
        return;
      }
    }
    n_ = 0;
  }
};

// ═══════════════════════════════════════════════════════════════
// 이음새 헬퍼 (§7 — COARSE_TRACK 배선 시 그대로 사용)
// ═══════════════════════════════════════════════════════════════

// 패킷 품질: 위치를 써도 되는 픽스인가 (gps_ublox.h valid 판정과 동일 기준)
inline bool rocketPacketUsable(const RocketPacket& p) {
  return p.fixType == 3 || p.fixType == 4;
}

// RocketPacket → geo.h 입력형 (mm/mms 고정소수 → m/mps float, 위경도는 int 그대로)
inline GeoPoint rocketPacketToGeo(const RocketPacket& p) {
  GeoPoint g;
  g.lat_i7 = p.lat_i7;
  g.lon_i7 = p.lon_i7;
  g.alt_m  = (float)p.alt_mm * 0.001f;
  return g;
}
inline NedVel rocketPacketToVel(const RocketPacket& p) {
  NedVel v;
  v.vN = (float)p.velN_mms * 0.001f;
  v.vE = (float)p.velE_mms * 0.001f;
  v.vD = (float)p.velD_mms * 0.001f;
  return v;
}

// 로켓측 송신 헬퍼: 자기 GPS 픽스 → 패킷 (G-B 루프백·로켓 펌웨어 공용)
//   ※ 고도 칸(alt_mm)은 GPS hMSL이 들어감. 기압계 고도를 쓰려면
//     rocketPacketFromFixBaro()를 사용하거나, 이 반환값의 alt_mm을 덮어쓸 것.
inline RocketPacket rocketPacketFromFix(const GpsFix& f, uint8_t seq) {
  RocketPacket p;
  p.seq      = seq;
  p.fixType  = f.fixType;
  p.iTOW     = f.iTOW;
  p.lat_i7   = f.lat_i7;                          // int 그대로 (규약 §2)
  p.lon_i7   = f.lon_i7;
  p.alt_mm   = (int32_t)lroundf(f.alt_m * 1000.0f);
  p.velN_mms = (int32_t)lroundf(f.vN * 1000.0f);
  p.velE_mms = (int32_t)lroundf(f.vE * 1000.0f);
  p.velD_mms = (int32_t)lroundf(f.vD * 1000.0f);
  return p;
}

// [고도기준 2026-07-21] 로켓측 송신 헬퍼: GPS 픽스(위경도·속도·시각) +
//   기압계 AGL[m] → 패킷. 고도만 GPS 대신 기압계 AGL로 채운다.
//   baroAglM = 로켓 BaroAgl.altitude(현재기압) (발사대에서 0점).
inline RocketPacket rocketPacketFromFixBaro(const GpsFix& f, float baroAglM, uint8_t seq) {
  RocketPacket p = rocketPacketFromFix(f, seq);
  p.alt_mm = (int32_t)lroundf(baroAglM * 1000.0f);
  return p;
}

// §5.2 방법1 — GPS 시각 동기 지연: delay = payload.iTOW − rocket.iTOW [s]
// iTOW는 주 단위(604,800,000ms)로 랩 → 토요일→일요일 경계 방어 포함.
inline float commDelayFromItow(uint32_t payload_iTOW, uint32_t rocket_iTOW) {
  uint32_t d = (payload_iTOW >= rocket_iTOW)
             ? (payload_iTOW - rocket_iTOW)
             : (payload_iTOW + GPS_MS_PER_WEEK - rocket_iTOW);
  return (float)d * 0.001f;
}

// ── .ino 통합 예시 (§5.3 링크 API — 통합 단계에서 배선, 지금 범위 밖) ──
//   RocketLinkParser link;  uint32_t tLastPkt = 0;
//   bool rocketLinkPoll(RocketPacket* out) {          // loop()마다 (논블로킹)
//     while (LORA_SERIAL.available())
//       if (link.feed(LORA_SERIAL.read(), out)) { tLastPkt = millis(); return true; }
//     return false;
//   }
//   uint32_t rocketLinkAgeMs() { return millis() - tLastPkt; }
//   → FSM comm_ok = rocketLinkAgeMs() <= (uint32_t)(COMM_TIMEOUT_S * 1000)  (§7)
//   → COARSE_TRACK에서:
//        float delay = commDelayFromItow(gpsFix.iTOW, pkt.iTOW);   // 방법1
//        coarsePipeline(rocketPacketToGeo(pkt), rocketPacketToVel(pkt),
//                       {gpsFix.lat_i7, gpsFix.lon_i7, gpsFix.alt_m},
//                       q, delay, &yawRaw, &pitchRaw);
