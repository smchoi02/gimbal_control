#pragma once
// ═══════════════════════════════════════════════════════════════
// gps_ublox.h — MAX-M10S UBX-NAV-PVT 파서 + UBX 프레임/VALSET 빌더
//               (Phase 4 GPS 드라이버 코어 — gps_module_design.md §3)
//
// 담당: GPS 파트 (CYN)
//
// Arduino 비의존 (표준 C++ / stdint.h만) → PC에서 g++로 단위시험 가능.
//   test_cpp/test_ubx.cpp 가 합성 NAV-PVT 프레임 주입으로 검증한다.
//
// ── 읽기 백엔드 추상화 (§3.4) ──
// 파서는 feed(1바이트) 단위로만 동작한다. 바이트가 UART에서 오든
// I2C(DDC)에서 오든 무관 — 설계문서의 gpsPoll()은 .ino 통합 단계에서
// 이 파서를 감싸는 얇은 함수가 된다:
//
//   UbxNavPvtParser ubx;  GpsFix gpsFix;
//   bool gpsPoll(GpsFix* out) {                 // loop()마다 호출 (논블로킹)
//     while (GPS_SERIAL.available())
//       if (ubx.feed(GPS_SERIAL.read(), out)) {
//         out->t_local_ms = millis();           // 로컬 수신 시각은 호출측이 기록
//         return true;
//       }
//     return false;
//   }
//
//   ※ Serial1은 DXL 전용(.ino) → 실제 포트(UART or I2C)는 §8 합의 후 결정.
//
// ── 정밀도 규약 (Phase 0 계승) ──
// 위경도는 UBX 원본 그대로 int32 (1e-7 deg)로 보존한다. float 변환 금지.
// 이후 처리는 geo.h(geoToNed 등)가 int64 승격 정수 산술로 수행.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include <stddef.h>

// ── UBX 프레임 상수 ──
static const uint8_t  UBX_SYNC1        = 0xB5;
static const uint8_t  UBX_SYNC2        = 0x62;
static const uint8_t  UBX_CLASS_NAV    = 0x01;
static const uint8_t  UBX_ID_NAV_PVT   = 0x07;
static const uint16_t UBX_NAV_PVT_LEN  = 92;    // M10 (protocol 34.x) 고정 길이
static const uint8_t  UBX_CLASS_ACK    = 0x05;
static const uint8_t  UBX_ID_ACK_ACK   = 0x01;  // payload[0..1] = ACK된 cls/id
static const uint8_t  UBX_ID_ACK_NAK   = 0x00;
static const uint8_t  UBX_CLASS_CFG    = 0x06;
static const uint8_t  UBX_ID_VALSET    = 0x8A;

// 비-PVT 프레임을 통째로 삼킬 때의 length 상한.
// 초과 = 깨진 length 필드로 간주하고 즉시 재동기 (65535바이트 동안 먹통 방지).
static const uint16_t UBX_MAX_SKIP_LEN = 2048;

// ═══════════════════════════════════════════════════════════════
// GpsFix — 페이로드 자기 위치·속도 (설계문서 §3.4 자료구조)
// ═══════════════════════════════════════════════════════════════
struct GpsFix {
  uint32_t iTOW = 0;         // ms, GPS 주중 시각 — 지연 보정·로켓과 시각 동기(§5.2)용
  int32_t  lat_i7 = 0;       // 위도 1e-7 deg (UBX 원본 그대로 — 규약 §2)
  int32_t  lon_i7 = 0;       // 경도 1e-7 deg
  float    alt_m  = 0.0f;    // hMSL [m] — 로켓과 고도 기준 통일 필수 (§3.2)
  float    vN = 0.0f;        // NED 속도 [m/s] (UBX velN/E/D와 부호 일치)
  float    vE = 0.0f;
  float    vD = 0.0f;        // 아래+ (geo.h NedVel과 동일 규약)
  uint8_t  fixType = 0;      // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
  uint8_t  numSV   = 0;      // 위성 수 (품질 판단)
  bool     valid   = false;  // 위치 신뢰 가능 여부 (decode() 주석 참고)
  uint32_t t_local_ms = 0;   // 수신 시각 millis() — 파서는 안 채움, 호출측이 기록
};

// little-endian 필드 조립 (호스트 엔디안 무관하게 동작)
inline uint32_t ubxRdU4(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline int32_t ubxRdI4(const uint8_t* p) { return (int32_t)ubxRdU4(p); }

// ═══════════════════════════════════════════════════════════════
// UbxNavPvtParser — 바이트 단위 상태머신 (§3.4)
//   SYNC1→SYNC2→CLASS→ID→LEN_L→LEN_H→PAYLOAD→CK_A→CK_B
//   체크섬(Fletcher-8) 불일치 프레임은 폐기 후 재동기.
//   NAV-PVT(0x01 0x07, len 92)만 디코드, 다른 UBX 프레임은 검증만 하고 통과.
// ═══════════════════════════════════════════════════════════════
class UbxNavPvtParser {
public:
  // 품질 카운터 — 브링업(G-A) 디버그·'?' 상태 출력용
  uint32_t framesOk    = 0;  // NAV-PVT 정상 수신 수
  uint32_t framesOther = 0;  // 다른 UBX 프레임 (체크섬 통과)
  uint32_t framesBad   = 0;  // 체크섬 불일치/깨진 length → 폐기

  // 마지막 비-PVT 프레임 식별자 — CFG-VALSET의 ACK-ACK/NAK 확인용 씨앗.
  // ACK 프레임이면 Pl0/Pl1 = ACK 대상의 cls/id.
  uint8_t lastOtherCls = 0, lastOtherId = 0;
  uint8_t lastOtherPl0 = 0, lastOtherPl1 = 0;

  void reset() { st_ = SYNC1; }

  // 1바이트 공급. 완결 NAV-PVT 프레임을 파싱했으면 *out을 채우고 true 반환.
  // true여도 위치를 믿어도 되는지는 out->valid로 판단할 것 (no-fix도 프레임은 옴).
  bool feed(uint8_t b, GpsFix* out) {
    switch (st_) {
      case SYNC1:
        if (b == UBX_SYNC1) st_ = SYNC2;
        break;

      case SYNC2:
        if (b == UBX_SYNC2)      { st_ = CLS; ckA_ = 0; ckB_ = 0; }
        else if (b != UBX_SYNC1) { st_ = SYNC1; }   // 0xB5 연속이면 SYNC2 유지
        break;

      case CLS:   ck(b); cls_ = b; st_ = ID;    break;
      case ID:    ck(b); id_  = b; st_ = LEN_L; break;
      case LEN_L: ck(b); len_ = b; st_ = LEN_H; break;

      case LEN_H:
        ck(b);
        len_ |= (uint16_t)b << 8;
        isPvt_ = (cls_ == UBX_CLASS_NAV && id_ == UBX_ID_NAV_PVT
                  && len_ == UBX_NAV_PVT_LEN);
        if (!isPvt_ && len_ > UBX_MAX_SKIP_LEN) {   // 깨진 length → 즉시 재동기
          framesBad++; st_ = SYNC1; break;
        }
        idx_ = 0; pl0_ = 0; pl1_ = 0;
        st_ = (len_ == 0) ? CK_A : PAYLOAD;
        break;

      case PAYLOAD:
        ck(b);
        if (isPvt_)         buf_[idx_] = b;   // PVT만 저장 (92B 버퍼)
        else if (idx_ == 0) pl0_ = b;         // 그 외엔 선두 2바이트만 (ACK 판별용)
        else if (idx_ == 1) pl1_ = b;
        if (++idx_ >= len_) st_ = CK_A;
        break;

      case CK_A:
        if (b == ckA_) st_ = CK_B;
        else { framesBad++; st_ = SYNC1; }    // 불일치 → 폐기·재동기
        break;

      case CK_B:
        st_ = SYNC1;
        if (b != ckB_) { framesBad++; break; }
        if (isPvt_) { decode(out); framesOk++; return true; }
        framesOther++;
        lastOtherCls = cls_; lastOtherId = id_;
        lastOtherPl0 = pl0_; lastOtherPl1 = pl1_;
        break;
    }
    return false;
  }

private:
  enum St : uint8_t { SYNC1, SYNC2, CLS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B };
  St       st_  = SYNC1;
  uint8_t  cls_ = 0, id_ = 0;
  uint16_t len_ = 0, idx_ = 0;
  bool     isPvt_ = false;
  uint8_t  ckA_ = 0, ckB_ = 0;
  uint8_t  pl0_ = 0, pl1_ = 0;
  uint8_t  buf_[UBX_NAV_PVT_LEN];

  void ck(uint8_t b) { ckA_ = (uint8_t)(ckA_ + b); ckB_ = (uint8_t)(ckB_ + ckA_); }

  // NAV-PVT payload 디코드 (§3.1 검증된 오프셋)
  void decode(GpsFix* out) const {
    out->iTOW    = ubxRdU4(buf_ + 0);
    out->fixType = buf_[20];
    const uint8_t flags = buf_[21];
    out->numSV   = buf_[23];
    out->lon_i7  = ubxRdI4(buf_ + 24);            // int32 그대로 (규약 §2)
    out->lat_i7  = ubxRdI4(buf_ + 28);
    out->alt_m   = (float)ubxRdI4(buf_ + 36) * 0.001f;  // hMSL mm → m
    out->vN      = (float)ubxRdI4(buf_ + 48) * 0.001f;  // mm/s → m/s
    out->vE      = (float)ubxRdI4(buf_ + 52) * 0.001f;
    out->vD      = (float)ubxRdI4(buf_ + 56) * 0.001f;
    // ★ 설계문서 §3.4는 "fixType>=3"이지만 fixType 5 = time-only(위치 없음)이라
    //   그대로 쓰면 위치 없는 픽스를 유효 처리할 수 있다 → 3(3D)/4(GNSS+DR)만 인정.
    //   + flags bit0 gnssFixOK (DOP·정확도 마스크 통과).
    const bool gnssFixOK = (flags & 0x01u) != 0;
    out->valid = (out->fixType == 3 || out->fixType == 4) && gnssFixOK;
  }
};

// ═══════════════════════════════════════════════════════════════
// UBX 프레임 빌더 (TX 공용) — sync + 헤더 + payload + Fletcher 체크섬
//   반환: 총 바이트 수 (len+8). 버퍼 부족 시 0.
// ═══════════════════════════════════════════════════════════════
inline size_t ubxMakeFrame(uint8_t cls, uint8_t id,
                           const uint8_t* payload, uint16_t len,
                           uint8_t* out, size_t outCap) {
  const size_t total = (size_t)len + 8;
  if (outCap < total) return 0;
  out[0] = UBX_SYNC1; out[1] = UBX_SYNC2;
  out[2] = cls;       out[3] = id;
  out[4] = (uint8_t)(len & 0xFF);
  out[5] = (uint8_t)(len >> 8);
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + (size_t)len; i++) {
    if (i >= 6) out[i] = payload[i - 6];
    a = (uint8_t)(a + out[i]); b = (uint8_t)(b + a);
  }
  out[6 + len] = a;
  out[7 + len] = b;
  return total;
}

// ═══════════════════════════════════════════════════════════════
// CFG-VALSET 빌더 — M10 초기화 (§3.3)
//   M10은 레거시 CFG-MSG가 아니라 configuration interface(VALSET)를 쓴다.
//   layer는 RAM만 권장 (매 부팅 재설정 = 안전·재현성 — §3.3 결정).
//
// ★ 아래 키 ID는 u-blox M10 Interface Description 기준으로 기입한 값.
//   부품 도착(G-A) 시 ACK-ACK 응답(lastOther*)과 u-center로 반드시 실증 검증할 것.
// ═══════════════════════════════════════════════════════════════
static const uint32_t CFG_MSGOUT_UBX_NAV_PVT_I2C   = 0x20910006; // U1: epoch당 출력 횟수
static const uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007; // U1
static const uint32_t CFG_I2COUTPROT_NMEA          = 0x10720002; // L : I2C NMEA 전체 off
static const uint32_t CFG_UART1OUTPROT_NMEA        = 0x10740002; // L : UART1 NMEA 전체 off
static const uint32_t CFG_RATE_MEAS                = 0x30210001; // U2 [ms]  (100 = 10Hz)
static const uint32_t CFG_RATE_NAV                 = 0x30210002; // U2 [cycles] (보통 1)
static const uint32_t CFG_UART1_BAUDRATE           = 0x40520001; // U4 (변경 직후엔 로컬 UART도 전환 필요)

class UbxValsetBuilder {
public:
  static const uint8_t LAYER_RAM   = 0x01;   // BBR=0x02, FLASH=0x04
  static const uint8_t VALSET_VERSION = 0x00;

  explicit UbxValsetBuilder(uint8_t layers = LAYER_RAM) { start(layers); }

  void start(uint8_t layers = LAYER_RAM) {
    n_ = 4; overflow_ = false;
    pl_[0] = VALSET_VERSION;
    pl_[1] = layers;
    pl_[2] = 0; pl_[3] = 0;                  // reserved
  }

  void addU1(uint32_t key, uint8_t v)   { addKey(key); put(v); }
  void addBool(uint32_t key, bool v)    { addU1(key, v ? 1 : 0); }  // L형 = 1바이트
  void addU2(uint32_t key, uint16_t v)  { addKey(key); put((uint8_t)v); put((uint8_t)(v >> 8)); }
  void addU4(uint32_t key, uint32_t v)  {
    addKey(key);
    put((uint8_t)v); put((uint8_t)(v >> 8));
    put((uint8_t)(v >> 16)); put((uint8_t)(v >> 24));
  }

  // 완성 프레임을 out에 기록. 반환 = 총 바이트 수 (키 없음/overflow면 0).
  size_t frame(uint8_t* out, size_t cap) const {
    if (overflow_ || n_ <= 4) return 0;
    return ubxMakeFrame(UBX_CLASS_CFG, UBX_ID_VALSET, pl_, (uint16_t)n_, out, cap);
  }

private:
  uint8_t  pl_[4 + 60];   // 우리 초기화는 ~30B (VALSET 자체 한도는 64쌍)
  uint16_t n_ = 4;
  bool     overflow_ = false;

  void put(uint8_t b) { if (n_ < sizeof(pl_)) pl_[n_++] = b; else overflow_ = true; }
  void addKey(uint32_t k) {
    put((uint8_t)k); put((uint8_t)(k >> 8));
    put((uint8_t)(k >> 16)); put((uint8_t)(k >> 24));
  }
};

// ── .ino 초기화 예시 (§3.3 — NAV-PVT만 10Hz, NMEA off, RAM layer) ──
//   UbxValsetBuilder v;
//   v.addU1(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);   // I2C 연결이면 _I2C 키 사용
//   v.addBool(CFG_UART1OUTPROT_NMEA, false);
//   v.addU2(CFG_RATE_MEAS, 100);                // 100ms = 10Hz
//   v.addU2(CFG_RATE_NAV, 1);
//   uint8_t f[64]; size_t n = v.frame(f, sizeof(f));
//   GPS_SERIAL.write(f, n);                     // 이후 ACK-ACK(0x05 0x01) 확인 권장
//   ※ 부팅 직후엔 9600bps + NMEA 혼재 상태임 — baud 상향은 마지막에, 별도 프레임으로.
