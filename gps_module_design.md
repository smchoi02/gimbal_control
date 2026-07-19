# GPS 모듈 개발 설계/계획 (Phase 4 — COARSE_TRACK)

담당: CYN (GPS geo 파트) · 대상 HW: OpenRB-150 + MAX-M10S + (로켓 통신 링크)
IMU 파트(최석민)의 `gimbal_imu_control`에 COARSE 모드로 결합됨.
버전: v0.1 (초안, 검토용)

---

## 0. 이 문서의 범위

이번 단계에서 만들 것은 세 덩어리다 (IMU 파트와 동일하게 *부품 없이 PC에서 먼저 검증* 가능하게 설계).

1. **C++ geo 코어** — 파이썬 `geometry.py`의 GPS 부분(`predict_position`, `geo_to_ned`, `coarse_pipeline`)을 C++로 이식 + `test_parity.cpp`와 같은 방식의 수치 일치 테스트
2. **MAX-M10S 센서 드라이버** — UBX-NAV-PVT 파싱으로 자기(페이로드) 위치·속도 취득
3. **로켓 위치 수신(통신)** — 로켓의 위치 패킷을 받는 링크 + 패킷 포맷·타임스탬프 처리

`.ino` / FSM 통합(COARSE_TRACK 배선)은 이번 범위 밖이지만, 세 모듈이 붙을 **이음새(seam)** 만 §7에 명시한다.

**설계 원칙 (기존 코드의 규약 계승):**
- 파이썬 `geometry.py`가 알고리즘의 단일 진실 소스. C++는 그것과 수치가 일치해야 함.
- geo 코어는 **Arduino 비의존(표준 C++)** → PC에서 `g++`로 단위시험 (기존 `geometry.h`와 동일 전략).
- 부호가 실물과 반대여도 **코드 수정 금지 → `gimbal_config.h`의 sign 상수만 조정** (기존 규약).

---

## 1. 배경: COARSE_TRACK이 필요로 하는 것

Phase 4 지향 파이프라인은 이미 파이썬에 완성돼 있고, C++에는 **뒷단(자세→짐벌각)만** 있고 **앞단(GPS→NED)이 비어 있다.**

```
[페이로드 GPS] ──┐
                 ├─► geo_to_ned ─► NED 상대벡터 ─► nedToBody ─► bodyToGimbal ─► 짐벌각
[로켓 GPS(수신)]─┘        ▲                            ▲(있음)      ▲(있음)
                    predict_position                geometry.h    geometry.h
                    (지연 보정, 있어야 함)          ← 여기부터는 이미 C++에 구현됨
   ────────────── 이번에 새로 만들 부분 ──────────────
```

`geometry.h`에 이미 있는 것: `nedToBody`, `bodyToGimbal`, `quatToR_b2n` 등 자세·짐벌 변환.
`geometry.h`에 주석으로 명시된 공백: *"GPS 위경도 int32 처리는 팀원 담당 geo 모듈에서 (여긴 IMU 파트만)"*.

FSM(`fsm.py`)에는 `COARSE_TRACK` 상태 전이 조건(`armed + comm_ok`)이 이미 정의돼 있어, geo 모듈이 유효 패킷을 공급하면 상태머신은 그대로 동작한다.

---

## 2. 좌표·정밀도 규약 (Phase 0 계승, 절대 어기지 말 것)

| 항목 | 규약 | 근거 |
|---|---|---|
| 위경도 저장 | **int32, 1e-7 deg 단위** | MAX-M10S UBX-NAV-PVT 원본 형식과 동일 |
| 위경도 뺄셈 | **정수로 먼저 차이 계산** 후 미터 변환 | float32 직접 뺄셈 = catastrophic cancellation (파이썬 test 2가 시연) |
| C++ 중간 연산 | 차이 계산 시 **int64로 승격** 후 float 변환 | `config.py` 주석: "int64 승격 후 정수 나눗셈 주의" |
| NED 부호 | dN+ 북, dE+ 동, dD+ 아래(로켓이 페이로드보다 낮으면 +) | `geo_to_ned` docstring |
| 경도 축소 보정 | `cos(payload_lat)` 사용 | `geo_to_ned` |
| 위도 1도 | 111,320 m (`M_PER_DEG_LAT`) | `config.py` |
| 고도 기준 | **로켓·페이로드가 같은 기준을 써야 함** (아래 §3.2 결정 필요) | — |

int32 차이의 크기 감각: 두 점이 같은 로켓·페이로드 거리(수백 m~수 km)면 `lat_r - lat_p`는 수만~수십만 단위. int32 안에 충분하지만, **곱셈 전에 int64로 올려야** 안전하다.

---

## 3. MAX-M10S 드라이버 설계

### 3.1 UBX-NAV-PVT 메시지 (검증된 필드 오프셋)

- 프레임: `0xB5 0x62` sync → class `0x01` id `0x07` → len(U2, LE) → payload(**92B**) → CK_A CK_B (Fletcher 체크섬)
- 파싱해야 할 페이로드 필드 (payload 선두 기준 오프셋):

| 오프셋 | 필드 | 타입 | 단위 | 용도 |
|---|---|---|---|---|
| 0 | iTOW | U4 | ms(주중 시각) | **타임스탬프**(지연 보정·동기화) |
| 20 | fixType | U1 | — | 0=no fix, 3=3D fix. 3 미만이면 무효 처리 |
| 21 | flags | X1 | — | bit0 gnssFixOK 확인 |
| 23 | numSV | U1 | 개 | 위성 수(품질 판단) |
| 24 | lon | I4 | 1e-7 deg | 경도 → 그대로 int32 저장 |
| 28 | lat | I4 | 1e-7 deg | 위도 → 그대로 int32 저장 |
| 36 | hMSL | I4 | mm | 평균해수면 고도 → /1000 → m |
| 48 | velN | I4 | mm/s | 북 속도 → /1000 → m/s |
| 52 | velE | I4 | mm/s | 동 속도 |
| 56 | velD | I4 | mm/s | 아래 속도(NED와 부호 일치) |

velN/E/D가 NED 프레임으로 바로 나오는 점이 유리하다 — 별도 변환 없이 `predict_position`에 투입.

### 3.2 결정 필요 — 고도 기준
UBX는 `height`(타원체 기준, offset 32)와 `hMSL`(해수면 기준, offset 36) 둘 다 제공.
**로켓과 페이로드가 반드시 같은 필드를 써야** 상대 고도차(dD)가 맞는다. 상대값만 쓰므로 무엇을 쓰든 상관없지만 *양쪽 통일* 필수. → 권장: **hMSL 통일**. (로켓 팀과 합의 항목)

### 3.3 초기화 (M10 계열 주의점)
- 기본 UART **9600bps**, 부팅 시 UBX·NMEA 동시 출력.
- M10은 레거시 `CFG-MSG`가 아니라 **configuration interface(CFG-VALSET)** 를 쓴다:
  - `CFG-MSGOUT-UBX_NAV_PVT_UART1` = 1 (NAV-PVT만 켜기)
  - NMEA 메시지들 = 0 (대역 절약)
  - `CFG-RATE-MEAS` / `CFG-RATE-NAV` 로 항법 주기 설정 (예: 5~10Hz)
  - (선택) `CFG-UART1-BAUDRATE`로 38400/115200 상향 — 9600에서 92B@10Hz는 여유는 있으나 상향 권장
- 설정을 RAM에만 넣을지, BBR/Flash에 저장할지 결정(매 부팅 재설정이 안전·재현성 높음).

### 3.4 파서 구조 (논블로킹)
```
struct GpsFix {
  uint32_t iTOW;          // ms, 타임스탬프
  int32_t  lat_i7, lon_i7;// 1e-7 deg (원본 그대로)
  float    alt_m;         // hMSL/1000
  float    vN, vE, vD;    // m/s
  uint8_t  fixType, numSV;
  bool     valid;         // fixType>=3 && gnssFixOK
  uint32_t t_local_ms;    // millis() 수신 시각(로컬 지연 계산용)
};

bool gpsPoll(GpsFix* out);   // loop마다 호출, 완결 프레임 파싱 시 true
```
- 바이트 단위 상태머신(SYNC1→SYNC2→CLASS→ID→LEN_L→LEN_H→PAYLOAD→CK_A→CK_B).
- 체크섬 불일치 프레임은 폐기(파서 재동기).
- **HW 제약(중요):** OpenRB-150의 `Serial1`은 이미 **DYNAMIXEL 전용**(`.ino`에서 `DXL_SERIAL=Serial1`). GPS는
  (a) 남는 UART, 또는 (b) **I2C**(MAX-M10S는 I2C도 지원, BNO085와 같은 Wire 버스 공유 가능, 주소 다름)로 붙여야 함.
  → 배선·핀 확정은 하드웨어 담당과 합의. 드라이버는 UART/I2C 어느 쪽이든 되게 **읽기 백엔드를 추상화**한다.

---

## 4. C++ geo 코어 설계 (`geo.h` / 필요시 `geo.cpp`)

파이썬 `geometry.py`의 3개 함수를 이식. 기존 `geometry.h` 스타일(inline, 표준 C++)과 통일.

```
struct GeoPoint { int32_t lat_i7; int32_t lon_i7; float alt_m; };
struct NedVel   { float vN, vE, vD; };

// (1) 통신 지연 보정: P_pred = P + V·dt  (파이썬 predict_position)
GeoPoint predictPosition(const GeoPoint& p, const NedVel& v, float dt_s);

// (2) 위경도차 → NED 미터. int64 승격 후 차이 계산 (파이썬 geo_to_ned)
void geoToNed(const GeoPoint& rocket, const GeoPoint& payload, float dNED[3]);

// (3) 전체 coarse 파이프라인 (파이썬 coarse_pipeline)
//     predict → geoToNed → nedToBody(geometry.h) → bodyToGimbal(geometry.h)
void coarsePipeline(const GeoPoint& rocket, const NedVel& rocketVel,
                    const GeoPoint& payload, const float q[4],
                    float commDelay_s, float* yawDeg, float* pitchDeg);
```
- `nedToBody`, `bodyToGimbal`는 **기존 `geometry.h` 재사용** (중복 구현 금지 → 파이썬과 이미 parity 검증된 코드).
- `predictPosition`의 경도 보정에서 `cos(lat)` 0 분모(극지) 방어. (미션 고도/위도에선 문제없지만 방어코드 권장)
- 부동소수: SAMD21은 FPU 없음 → `float` 사용(기존 `geometry.h`와 동일). 단, **위경도 차이만은 정수 연산** 후 float.

---

## 5. 로켓 위치 수신(통신) 설계

이 부분이 **미결 변수가 가장 많다** (§8 결정 필요). 전송 매체와 무관하게 동작하도록 계층 분리.

### 5.1 패킷 포맷 (제안 — 고정 길이 바이너리)
```
struct RocketPacket {          // little-endian, 고정 크기
  uint16_t magic;              // 0xR0CK 류 프레임 마커
  uint8_t  seq;                // 시퀀스(패킷 유실·순서 감지)
  uint8_t  fixType;            // 로켓 GPS 품질
  uint32_t iTOW;               // 로켓 GPS 주중 시각(ms) — 지연 보정의 핵심
  int32_t  lat_i7, lon_i7;     // 1e-7 deg
  int32_t  alt_mm;             // hMSL, mm (페이로드와 기준 통일)
  int32_t  velN_mms, velE_mms, velD_mms;  // mm/s
  uint16_t crc;                // CRC16 — 무결성
};
```
- **정수·고정소수로만 전송** (float 금지) → 규약 §2 유지, 파싱 단순, 크기 작음.
- CRC로 손상 패킷 폐기.

### 5.2 타임스탬프·지연 보정 전략 (택1, 권장순)
1. **GPS 시각 동기(권장):** 로켓·페이로드 둘 다 GPS iTOW 보유 → `delay = payload.iTOW - rocket.iTOW`.
   두 수신기가 같은 GPS 시계를 공유하므로 절대 지연을 정확히 측정 → `predict_position`에 투입.
2. **로컬 도착시각 근사:** 패킷 수신 `millis()` − 직전 패킷 간격 추정. GPS 동기 불가 시 대안.
3. **고정 지연:** `TYPICAL_LAG_S`(현재 0.07s) 상수. 최후 수단.

파이썬 `run_sim.py`가 지연 보정 ON/OFF로 RMS 오차 0.002° vs 0.485°를 보여줬으므로, **방법 1이 목표**.

### 5.3 링크 API (전송 추상화)
```
bool rocketLinkPoll(RocketPacket* out);  // 완결·CRC통과 패킷 수신 시 true
uint32_t rocketLinkAgeMs();              // 마지막 유효 패킷 후 경과 → FSM comm_ok 판단
```
- 전송 백엔드(LoRa 모듈 등)는 이 API 뒤에 숨김 → geo 로직은 매체 불문 동일.
- `rocketLinkAgeMs()`가 `COMM_TIMEOUT_S`(0.5s) 초과 → FSM이 HOLD로 (이미 `fsm.py`에 로직 존재).

---

## 6. 테스트 계획 (부품 없이 PC에서 먼저)

기존 `test_parity.cpp`(파이썬 `test_vectors.py`와 수치 일치 확인)를 **그대로 확장**.

1. **geo 코어 parity** — 파이썬 test 1·2·5·6에 대응:
   - `geoToNed` 부호(정북/정동/아래 500m)
   - int32 정밀도(1e-7deg 차이 = 1.1132cm 분해)
   - `predictPosition`(북 30m/s×150ms=+4.5m, 하강 20m/s×150ms=alt−3m)
   - `coarsePipeline`(북50+아래30 → yaw0/pitch+31, 페이로드 90°회전 → yaw−90)
   - → **파이썬과 같은 입력에 같은 출력**이 목표.
2. **UBX 파서 단위테스트** — 합성/캡처한 NAV-PVT 바이트버퍼 주입 → 필드 디코드·체크섬 검증(고의 손상 프레임 거부 확인).
3. **로켓 링크 루프백** — `RocketPacket` 인코드→디코드 왕복 일치, CRC가 손상 패킷 거부.

**하드웨어 브링업 단계 (IMU 파트의 Stage A/B/C 방식 계승):**
- **G-A (GPS 단독):** MAX-M10S만 결선 → fixType/numSV/lat/lon 시리얼 출력, 휴대폰 GPS와 대조. 콜드스타트 시간 측정.
- **G-B (링크 루프백):** 보드 2개 책상 위에서 로켓 패킷 송수신, 알려진 좌표 주입 → 계산된 yaw/pitch 확인.
- **G-C (통합):** IMU + GPS + 링크 → COARSE_TRACK end-to-end, `run_sim.py` 시나리오 로그와 대조.

로그는 기존 텔레메트리 CSV에 GPS 열(fix, numSV, dN/dE/dD, commDelay) 추가 → 파이썬 시뮬레이터 replay로 분석.

---

## 7. 통합 이음새 (이번 범위 밖, 배선 지점만 명시)

- `.ino`의 `controlTick()` `case`에 `COARSE_TRACK` 추가 예정 위치:
  ```
  case COARSE_TRACK:
    coarsePipeline(rocketState, rocketVel, payloadFix, q, commDelay, &yawRaw, &pitchRaw);
    break;   // 이후 filt.step → dxlSend 는 기존 그대로
  ```
- `loop()`에 `gpsPoll()`·`rocketLinkPoll()` 호출 추가(제어 루프 예산 밖, 논블로킹).
- FSM `comm_ok` 입력 = `rocketLinkAgeMs() <= COMM_TIMEOUT_S`.
- 자세 소스: COARSE에선 절대 yaw가 필요 → BNO085 `V R`(Rotation Vector) + yaw bias 보정(팀 합의 사항, README에 이미 언급).

---

## 8. 착수 전 확정 필요한 결정 (팀/하드웨어)

1. **로켓 통신 하드웨어**: LoRa 모듈 기종? UART형/SPI형? 로켓측 송신기는 누가 담당? → 5.3 백엔드 확정에 필요.
2. **패킷 포맷 합의**: §5.1 구조를 로켓 팀과 확정(필드·엔디안·CRC 다항식).
3. **고도 기준 통일**: hMSL vs 타원체(§3.2).
4. **GPS 물리 연결**: UART(남는 포트) vs I2C(BNO085와 버스 공유). `Serial1`은 DXL 점유 중.
5. **타임스탬프 방식**: GPS iTOW 동기(권장) 가능 여부.
6. **항법 주기·baud**: NAV-PVT 5Hz? 10Hz? UART 상향 여부.

---

## 9. 제안 작업 순서 (착수 시)

1. `geo.h` 작성 + `test_parity.cpp`에 GPS 벡터 추가 → PC에서 파이썬과 parity 통과 *(부품 0개로 가능, 먼저 시작 권장)*
2. UBX 파서(`gps_ublox.h/.cpp`) + 합성 버퍼 단위테스트
3. `RocketPacket` + CRC + 루프백 테스트 (전송 백엔드는 스텁)
4. (부품 도착) G-A → G-B → G-C 브링업
5. `.ino`/FSM 통합 (다음 범위)

1번은 지금 당장, 하드웨어·팀 합의 없이도 착수 가능하다.

---

### 참고 출처
- UBX-NAV-PVT 필드 구조: u-blox M10 SPG 5.20 Interface Description; ROS ublox_msgs NavPVT
- MAX-M10S 기본 UART 9600bps·UBX/NMEA 동시·CFG-VALSET: MAX-M10S Integration Manual
