#include <Dynamixel2Arduino.h>
#include <actuator.h>

// ═══════════════════════════════════════════════════════════════
// gimbal_imu_control.ino — IMU 기반 짐벌 제어 (Phase 2+3)
// OpenRB-150 + DYNAMIXEL XL430 ×2 + BNO085
//
// 담당: 최석민 (IMU 파트). GPS coarse track은 팀원 geo 모듈이 여기에 붙음.
//
// ── IMU 소스 3종 (시리얼 명령 'I'로 전환) ──
//   REAL   : 실제 BNO085 (I2C)
//   SIM    : 보드 내 가상 생성 (±40° 사인 요동) — BNO085 없이 DXL 파이프라인 시험
//   INJECT : PC에서 시리얼로 쿼터니언 주입 — 파이썬 시뮬레이터와 동일 데이터 재현
//
// ── 시리얼 명령 (115200bps, 줄 단위) ──
//   M <yaw> <pitch>  MANUAL 모드, 각도 지정 (예: M 30 10)
//   H                STABILIZED_HOLD — 현재 방향 캡처 후 고정
//   Z                STOW (0,0 복귀)
//   T0 / T1          Torque Off / On
//   I R|S|J          IMU 소스: Real / Sim / inJect
//   V R|G            자세 소스: Rotation Vector / Game RV (지자기 미사용)
//   Q <w> <x> <y> <z>  쿼터니언 주입 (INJECT 모드에서)
//   ?                도움말 + 현재 상태
//
// ── 텔레메트리 (10Hz CSV) ──
//   t_ms,mode,imuSrc,acc,pYaw,pPitch,pRoll,gYawCmd,gPitchCmd,
//   dxlYaw,dxlPitch,dxlCurY,dxlCurP,limitFlag
//   → PC에서 시리얼 로그를 CSV 저장하면 파이썬 시뮬레이터로 replay 분석 가능
// ═══════════════════════════════════════════════════════════════
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <initializer_list>
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "gimbal_config.h"
#include "geometry.h"
#include "gimbal_controller.h"
#include "coarse_track.h"   // [GPS 통합 2026-07-19] COARSE_TRACK 코어 (geo/gps_ublox/rocket_link 포함)

// ── 하드웨어 객체 ──
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;              // OpenRB-150 내부 처리
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

BNO08x bno;
bool bnoPresent = false;

// ── [GPS 통합] 로켓 LoRa UART = Serial3 (OpenRB-150 내장, SERCOM 수작업 불필요) ──
//   OpenRB-150 variant: D13 = Serial3 RX(PB23), D14 = Serial3 TX(PB22) — GPIO 헤더 노출.
//   Serial1은 DXL 전용 → 로켓 링크는 Serial3 사용. (대안: BT용 Serial2 = VIN/DXL 옆 4핀 홀)
//   ※ 설치된 OpenRB 코어에 Serial3 인스턴스가 있는지 1회 확인(없으면 Serial2로 교체).
#define LORA_SERIAL Serial3

#include "hw_backends.h"   // [GPS 통합] gpsPoll/rocketLinkPoll/baroPoll (실하드웨어 백엔드)
bool baroPresent = false;  // BMP390 존재 여부 (없으면 GPS 고도 폴백)

// ── 모드 ──
enum Mode { STOW, MANUAL, HOLD, FAULT, COARSE };   // [GPS 통합] COARSE=4 (기존 번호 유지 위해 뒤에 추가)
enum ImuSrc { IMU_REAL, IMU_SIM, IMU_INJECT };
enum AttSrc { ATT_ROTATION, ATT_GAME };   // Rotation Vector vs Game RV

Mode   mode   = STOW;
ImuSrc imuSrc = IMU_SIM;                  // 기본 SIM → BNO085 없이도 동작
AttSrc attSrc = ATT_GAME;                 // 안정화는 지자기 미사용이 기본 (본문 참고)

// ── 상태 변수 ──
float q[4] = {1, 0, 0, 0};               // 현재 자세 (w,x,y,z)
uint8_t quatAccuracy = 0;                 // BNO085 accuracy (0~3)
float manualYaw = 0, manualPitch = 0;
float holdDirNED[3] = {1, 0, 0};          // HOLD 목표 방향 (세계 기준)
bool torqueOn = false;
GimbalCommandFilter filt;

// [GPS 통합 2026-07-19] COARSE_TRACK 상태
CoarseTracker coarse;
bool coarseArmed = false;    // 'C'로 세팅, M/H/Z로 해제 — HOLD 폴백 후 자동 복귀 조건

uint32_t tLastControl = 0, tLastTelem = 0, tLastCmd = 0;
uint32_t tLastSensor = 0;   // [GPS 통합] GPS/기압계 폴 주기 타이머
uint32_t tStart = 0;

// ─────────────────────────────────────────────────────
// DYNAMIXEL
// ─────────────────────────────────────────────────────
bool dxlInit() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  bool ok = true;
  for (uint8_t id : {(uint8_t)YAW_ID, (uint8_t)PITCH_ID}) {
    if (!dxl.ping(id)) {
      DEBUG_SERIAL.print(F("# DXL ping FAIL id="));
      DEBUG_SERIAL.println(id);
      ok = false;
      continue;
    }
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(GOAL_CURRENT, id, DXL_GOAL_CURRENT);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, DXL_PROFILE_VEL);
    dxl.torqueOn(id);
  }
  return ok;
}

void dxlSend(float gimbalYaw, float gimbalPitch) {
  if (!torqueOn) return;
  dxl.setGoalPosition(YAW_ID,   YAW_SIGN   * gimbalYaw   + DXL_CENTER_DEG, UNIT_DEGREE);
  dxl.setGoalPosition(PITCH_ID, PITCH_SIGN * gimbalPitch + DXL_CENTER_DEG, UNIT_DEGREE);
}

void dxlTorque(bool on) {
  torqueOn = on;
  if (on) { dxl.torqueOn(YAW_ID); dxl.torqueOn(PITCH_ID); }
  else    { dxl.torqueOff(YAW_ID); dxl.torqueOff(PITCH_ID); }
}

// ─────────────────────────────────────────────────────
// IMU — 소스별 자세 갱신
// ─────────────────────────────────────────────────────
void imuUpdate() {
  if (imuSrc == IMU_SIM) {
    // 가상 요동: yaw = ±40° sin (주기 8s) — run_sim.py와 동일 시나리오
    float t = (millis() - tStart) * 0.001f;
    float simYaw = 40.0f * sinf(2.0f * PI * t / 8.0f);
    eulerToQuat(simYaw, 0, 0, q);
    quatAccuracy = 3;
    return;
  }
  if (imuSrc == IMU_INJECT) {
    return;   // 'Q' 명령이 q[]를 직접 채움
  }
  // IMU_REAL
  if (!bnoPresent) return;
  if (bno.getSensorEvent()) {
    uint8_t id = bno.getSensorEventID();
    if (attSrc == ATT_ROTATION && id == SENSOR_REPORTID_ROTATION_VECTOR) {
      q[0] = bno.getQuatReal();
      q[1] = bno.getQuatI();
      q[2] = bno.getQuatJ();
      q[3] = bno.getQuatK();
      quatAccuracy = bno.getQuatAccuracy();
      quatNormalize(q);
    } else if (attSrc == ATT_GAME && id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      q[0] = bno.getGameQuatReal();
      q[1] = bno.getGameQuatI();
      q[2] = bno.getGameQuatJ();
      q[3] = bno.getGameQuatK();
      quatAccuracy = 3;             // Game RV는 accuracy 미제공
      quatNormalize(q);
    }
  }
}

bool bnoInit() {
  Wire.begin();
  if (!bno.begin()) return false;
  bno.enableRotationVector(10);       // 100Hz
  bno.enableGameRotationVector(10);   // 100Hz — 둘 다 켜고 attSrc로 선택 소비
  return true;
}

// ─────────────────────────────────────────────────────
// 제어 루프 (50Hz)
// ─────────────────────────────────────────────────────
void controlTick(float dt) {
  float yawRaw = 0, pitchRaw = 0;

  switch (mode) {
    case STOW:
      yawRaw = 0; pitchRaw = 0;
      break;
    case MANUAL:
      yawRaw = manualYaw; pitchRaw = manualPitch;
      // heartbeat: 일정 시간 명령 없으면 STOW
      if ((millis() - tLastCmd) * 0.001f > MANUAL_HEARTBEAT_S) {
        mode = STOW;
        DEBUG_SERIAL.println(F("# heartbeat timeout -> STOW"));
      }
      break;
    case HOLD:
      // [GPS 통합] COARSE 대기 중(armed)이고 데이터 회복되면 복귀 (fsm.py: comm recovered)
      if (coarseArmed && coarse.compute(q, millis(), &yawRaw, &pitchRaw)) {
        mode = COARSE;
        DEBUG_SERIAL.println(F("# comm recovered -> COARSE"));
      } else {
        holdDirection(holdDirNED, q, &yawRaw, &pitchRaw);
      }
      break;
    case COARSE:
      // [GPS 통합] 로켓 지향각 계산. 데이터 상실 시 현재 방향 잡고 HOLD 폴백 (fsm.py: comm lost)
      if (!coarse.compute(q, millis(), &yawRaw, &pitchRaw)) {
        captureHoldDirection(filt.yawOut, filt.pitchOut, q, holdDirNED);
        mode = HOLD;
        DEBUG_SERIAL.println(F("# comm lost -> HOLD"));
        yawRaw = filt.yawOut; pitchRaw = filt.pitchOut;
      }
      break;
    case FAULT:
      return;   // torque off 상태 유지, 명령 안 보냄
  }

  filt.step(yawRaw, pitchRaw, dt);
  dxlSend(filt.yawOut, filt.pitchOut);
}

// ─────────────────────────────────────────────────────
// 안전 감시 (텔레메트리 주기에서 함께)
// ─────────────────────────────────────────────────────
void safetyCheck(int8_t tempY, int8_t tempP) {
  if (tempY >= DXL_TEMP_LIMIT_C || tempP >= DXL_TEMP_LIMIT_C) {
    mode = FAULT;
    dxlTorque(false);
    DEBUG_SERIAL.println(F("# OVERTEMP -> FAULT, torque off"));
  }
}

// ─────────────────────────────────────────────────────
// 시리얼 명령 파서
// ─────────────────────────────────────────────────────
void handleLine(char* line) {
  tLastCmd = millis();
  switch (line[0]) {
    case 'M': {
    char* endp;
    float y = strtof(line + 1, &endp);
    float p = strtof(endp, &endp);
    manualYaw = y; manualPitch = p;
    coarseArmed = false;                       // [GPS 통합] 수동 개입 시 COARSE 해제
    if (mode != FAULT) mode = MANUAL;
    DEBUG_SERIAL.print(F("# MANUAL ")); DEBUG_SERIAL.print(y);
    DEBUG_SERIAL.print(' '); DEBUG_SERIAL.println(p);
    break;
    }
    case 'H':
      captureHoldDirection(filt.yawOut, filt.pitchOut, q, holdDirNED);
      coarseArmed = false;                     // [GPS 통합] 수동 HOLD는 COARSE 해제
      if (mode != FAULT) mode = HOLD;
      DEBUG_SERIAL.println(F("# HOLD: direction captured"));
      break;
    case 'Z':
      coarseArmed = false;                     // [GPS 통합]
      if (mode != FAULT) mode = STOW;
      DEBUG_SERIAL.println(F("# STOW"));
      break;
    case 'T':
      dxlTorque(line[1] == '1');
      if (line[1] == '1' && mode == FAULT) mode = STOW;   // fault clear
      DEBUG_SERIAL.println(line[1] == '1' ? F("# torque ON") : F("# torque OFF"));
      break;
    case 'I':
      if (line[2] == 'R') { imuSrc = IMU_REAL;   DEBUG_SERIAL.println(F("# IMU=REAL")); }
      if (line[2] == 'S') { imuSrc = IMU_SIM;    DEBUG_SERIAL.println(F("# IMU=SIM")); }
      if (line[2] == 'J') { imuSrc = IMU_INJECT; DEBUG_SERIAL.println(F("# IMU=INJECT")); }
      break;
    case 'V':
      if (line[2] == 'R') { attSrc = ATT_ROTATION; DEBUG_SERIAL.println(F("# ATT=RotationVector")); }
      if (line[2] == 'G') { attSrc = ATT_GAME;     DEBUG_SERIAL.println(F("# ATT=GameRV")); }
      break;
    case 'Q': {
    // [FIX 2026-07-19] sscanf %f 파싱은 SAMD21(newlib-nano)에서 float 변환이
    // 비활성이라 항상 실패 → Q 명령이 조용히 무시됨 (Stage B에서 실증: pYaw 동결).
    // M 명령과 동일하게 strtof로 파싱한다.
    char* endp;
    float w = strtof(line + 1, &endp);
    float x = strtof(endp, &endp);
    float y = strtof(endp, &endp);
    float z = strtof(endp, &endp);
    if (!(w == 0.0f && x == 0.0f && y == 0.0f && z == 0.0f)) {  // 파싱 실패(전부 0) 방어
        q[0]=w; q[1]=x; q[2]=y; q[3]=z;
        quatNormalize(q);
        quatAccuracy = 3;
    }
    break;
    }
    // ── [GPS 통합 2026-07-19] COARSE_TRACK 명령 ──
    case 'C':      // COARSE 시작 (데이터 없으면 HOLD로 대기하다 자동 진입)
      if (mode != FAULT) {
        coarseArmed = true;
        if (coarse.ready(millis())) {
          mode = COARSE;
          DEBUG_SERIAL.println(F("# COARSE start"));
        } else {
          captureHoldDirection(filt.yawOut, filt.pitchOut, q, holdDirNED);
          mode = HOLD;
          DEBUG_SERIAL.println(F("# COARSE armed - no data yet (HOLD)"));
        }
      }
      break;
    case 'G': {    // 페이로드 GPS 주입: G <iTOW_ms> <lat_i7> <lon_i7> <alt_m>
      char* e;     //   (실물에선 gpsPoll()이 대신 공급 — loop()의 배선 주석 참고)
      GpsFix f;
      f.iTOW    = (uint32_t)strtoul(line + 1, &e, 10);
      f.lat_i7  = (int32_t)strtol(e, &e, 10);
      f.lon_i7  = (int32_t)strtol(e, &e, 10);
      f.alt_m   = strtof(e, &e);
      f.fixType = 3; f.numSV = 12; f.valid = true;
      coarse.onFix(f, millis());
      break;
    }
    case 'R': {    // 로켓 패킷 주입: R <iTOW_ms> <lat_i7> <lon_i7> <alt_mm> <vN_mms> <vE_mms> <vD_mms>
      char* e;     //   (실물에선 rocketLinkPoll()이 대신 공급)
      RocketPacket p;
      p.iTOW     = (uint32_t)strtoul(line + 1, &e, 10);
      p.lat_i7   = (int32_t)strtol(e, &e, 10);
      p.lon_i7   = (int32_t)strtol(e, &e, 10);
      p.alt_mm   = (int32_t)strtol(e, &e, 10);
      p.velN_mms = (int32_t)strtol(e, &e, 10);
      p.velE_mms = (int32_t)strtol(e, &e, 10);
      p.velD_mms = (int32_t)strtol(e, &e, 10);
      p.fixType  = 3;
      coarse.onPacket(p, millis());
      break;
    }
    case 'A': {    // 페이로드 기압계 AGL 주입: A <aglM>  (실물에선 baroPoll이 공급)
      char* e; float agl = strtof(line + 1, &e);
      coarse.onBaro(agl, millis());
      break;
    }
    case 'B':      // [GPS 통합] 기압계 발사대 0점 재캡처 (지상에서만)
      if (baroPresent && baroZero(50)) DEBUG_SERIAL.println(F("# baro re-zeroed@pad"));
      else DEBUG_SERIAL.println(F("# baro zero FAIL (BMP390 없음?)"));
      break;
    case 'P':      // [GPS 통합] u-blox 재설정 (부팅 시 GPS 미부팅으로 놓쳤을 때 수동 재전송)
      DEBUG_SERIAL.println(gpsConfigure() ? F("# GPS reconfigured (UBX-NAV-PVT I2C 10Hz)") : F("# GPS cfg FAIL"));
      break;
    case '?': {
      DEBUG_SERIAL.println(F("# cmds: M yaw pitch | H | Z | C | T0/T1 | I R/S/J | V R/G | Q w x y z | G iTOW lat7 lon7 alt | R iTOW lat7 lon7 altmm vN vE vD | A aglM | B(baro 0점) | P(GPS 재설정)"));
      DEBUG_SERIAL.print(F("# mode=")); DEBUG_SERIAL.print(mode);
      DEBUG_SERIAL.print(F(" imuSrc=")); DEBUG_SERIAL.print(imuSrc);
      DEBUG_SERIAL.print(F(" bno=")); DEBUG_SERIAL.print(bnoPresent);
      DEBUG_SERIAL.print(F(" torque=")); DEBUG_SERIAL.println(torqueOn);
      break;
    }
  }
}

void pollSerial() {
  static char buf[64];
  static uint8_t n = 0;
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ─────────────────────────────────────────────────────
// 텔레메트리 (10Hz CSV) — DXL 피드백도 이 주기로만 읽음
// (57600bps에서 매 사이클 읽으면 50Hz 루프 예산 초과)
// ─────────────────────────────────────────────────────
void telemetryTick() {
  float dy = dxl.getPresentPosition(YAW_ID, UNIT_DEGREE) - DXL_CENTER_DEG;
  float dp = dxl.getPresentPosition(PITCH_ID, UNIT_DEGREE) - DXL_CENTER_DEG;
  int16_t cy = dxl.readControlTableItem(PRESENT_CURRENT, YAW_ID);
  int16_t cp = dxl.readControlTableItem(PRESENT_CURRENT, PITCH_ID);
  int8_t  ty = dxl.readControlTableItem(PRESENT_TEMPERATURE, YAW_ID);
  int8_t  tp = dxl.readControlTableItem(PRESENT_TEMPERATURE, PITCH_ID);

  safetyCheck(ty, tp);

  float py, pp, pr;
  quatToEuler(q, &py, &pp, &pr);

  DEBUG_SERIAL.print(millis() - tStart); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(mode);              DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(imuSrc);            DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(quatAccuracy);      DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(py, 2);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(pp, 2);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(pr, 2);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(filt.yawOut, 2);   DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(filt.pitchOut, 2); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dy, 2);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dp, 2);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(cy);     DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(cp);     DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(filt.limitFlag ? 1 : 0);
  DEBUG_SERIAL.print(',');                    // [GPS 통합] 15번째 열: 로켓 패킷 나이 (ms, -1=수신 전)
  uint32_t pAge = coarse.pktAgeMs(millis());
  DEBUG_SERIAL.println(pAge == 0xFFFFFFFFu ? -1L : (long)pAge);
}

// ─────────────────────────────────────────────────────
void setup() {
  DEBUG_SERIAL.begin(115200);
  uint32_t t0 = millis();
  while (!DEBUG_SERIAL && millis() - t0 < 3000) {}   // 모니터 대기 (최대 3s)

  DEBUG_SERIAL.println(F("# gimbal_imu_control v0.1"));

  bool dxlOk = dxlInit();
  torqueOn = dxlOk;
  DEBUG_SERIAL.print(F("# DXL: ")); DEBUG_SERIAL.println(dxlOk ? F("OK") : F("FAIL"));

  bnoPresent = bnoInit();
  DEBUG_SERIAL.print(F("# BNO085: "));
  DEBUG_SERIAL.println(bnoPresent ? F("OK") : F("not found (SIM/INJECT 사용 가능)"));

  if (bnoPresent) imuSrc = IMU_REAL;   // 실물 있으면 자동 REAL

  // [GPS 통합] 실하드웨어 백엔드 초기화 (Wire는 bnoInit에서 begin됨 — 없어도 아래서 재보장)
  Wire.begin();
  LORA_SERIAL.begin(LORA_BAUD);             // Serial3 = D13(RX)/D14(TX)
  delay(200);                               // u-blox 콜드 스타트 여유
  bool gpsCfg = gpsConfigure();             // u-blox → UBX-NAV-PVT I2C 10Hz (없으면 GPS 무응답)
  DEBUG_SERIAL.print(F("# GPS cfg: "));
  DEBUG_SERIAL.println(gpsCfg ? F("sent (UBX-NAV-PVT I2C 10Hz)") : F("I2C FAIL (GPS 연결?)"));
  baroPresent = baroInit();
  DEBUG_SERIAL.print(F("# BMP390: "));
  DEBUG_SERIAL.println(baroPresent ? F("OK") : F("not found (GPS 고도 폴백)"));
  if (baroPresent && baroZero(50)) DEBUG_SERIAL.println(F("# baro zeroed@pad (재영점: 'B')"));

  DEBUG_SERIAL.println(F("# CSV: t_ms,mode,imuSrc,acc,pYaw,pPitch,pRoll,gYawCmd,gPitchCmd,dxlYaw,dxlPitch,curY,curP,limit,pktAgeMs"));
  DEBUG_SERIAL.println(F("# type ? for help"));

  filt.reset(0, 0);
  tStart = millis();
  tLastControl = tLastTelem = tLastCmd = micros();
}

void loop() {
  pollSerial();
  imuUpdate();

  uint32_t now = micros();

  // [GPS 통합] 실하드웨어 백엔드 폴 (hw_backends.h). 시리얼 'G'/'R'/'A' 주입과 동일 경로로 공급.
  { RocketPacket pk; if (rocketLinkPoll(&pk)) coarse.onPacket(pk, millis()); }  // LoRa: 매 루프 UART 드레인
  if (now - tLastSensor >= 40000u) {          // GPS(DDC)·기압계 25Hz (baro performReading 블로킹 방지)
    tLastSensor = now;
    GpsFix fx;   if (gpsPoll(&fx))    coarse.onFix(fx, millis());
    float aglM;  if (baroPoll(&aglM)) coarse.onBaro(aglM, millis());
  }

  if (now - tLastControl >= (uint32_t)(CONTROL_DT * 1e6f)) {
    float dt = (now - tLastControl) * 1e-6f;
    tLastControl = now;
    controlTick(dt);
  }

  if (now - tLastTelem >= (uint32_t)(1e6f / TELEMETRY_HZ)) {
    tLastTelem = now;
    telemetryTick();
  }
}
