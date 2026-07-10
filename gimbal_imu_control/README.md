# gimbal_imu_control — IMU 기반 짐벌 제어 (Phase 2+3)

OpenRB-150 + DYNAMIXEL XL430 ×2 + BNO085 대상. 담당: 최석민 (IMU 파트).
팀원의 GPS geo 모듈은 나중에 `controlTick()`에 COARSE 모드로 붙는다.

## 폴더 구조

```
gimbal_imu_control/
├── gimbal_imu_control/          ← Arduino IDE로 이 폴더 열기
│   ├── gimbal_imu_control.ino   메인 스케치 (모드, 시리얼 CLI, 루프)
│   ├── gimbal_config.h          Phase 0 규약 (파이썬 config.py와 1:1)
│   ├── geometry.h               자세·좌표 변환 (파이썬 geometry.py와 1:1)
│   └── gimbal_controller.h      clamp/rate limit (파이썬 controller.py와 1:1)
├── test_cpp/
│   └── test_parity.cpp          PC 검증 — 파이썬 test vector와 동일값 확인
└── inject_test.py               PC→보드 가상 IMU 주입 + 로그 저장 (부품 도착 후)
```

## 지금 (부품 도착 전) 할 수 있는 것

```bash
# 1) C++ 코어를 PC에서 검증 — 파이썬 시뮬레이터와 수치 일치 확인 (23개 테스트)
cd test_cpp
g++ -std=c++17 -I../gimbal_imu_control test_parity.cpp -o test_parity && ./test_parity

# 2) Arduino IDE에서 컴파일 확인
#    gimbal_imu_control/ 폴더 열기 → 보드 OpenRB-150 선택 → ✓(Verify)
```

## 부품 도착 후 3단계 브링업

**Stage A — SIM 모드 (BNO085 결선 전, OpenRB+DXL만)**
전원·USB 연결 → 업로드 → 기본이 SIM 모드라 짐벌이 ±40° 요동을 스스로
상쇄하는 것처럼 움직임. DXL 배선·ID·방향·rate limit 확인.
yaw가 반대로 돌면 `gimbal_config.h`의 `YAW_SIGN`만 뒤집기 (코드 수정 금지).

**Stage B — INJECT 모드 (PC에서 재현 가능한 데이터 주입)**
```bash
python3 inject_test.py COM5     # 포트는 장치관리자에서 확인
```
파이썬 시뮬레이터와 동일한 요동 데이터로 하드웨어 전체 파이프라인 검증.
`log_inject.csv`의 pYaw(주입)와 gYawCmd(명령)가 부호 반대·크기 일치하면 성공.

**Stage C — REAL 모드 (BNO085 결선 후)**
BNO085를 3.3V/GND/SDA/SCL로 결선 (5V 금지). 부팅 시 자동 감지되어 REAL 전환.
`H` 명령으로 HOLD 진입 후 보드를 손으로 돌려 카메라 방향이 유지되는지 확인.

## 시리얼 명령 (115200bps)

| 명령 | 동작 |
|---|---|
| `M 30 10` | MANUAL — yaw 30°, pitch 10° (2초 무명령 시 자동 STOW) |
| `H` | 현재 방향 캡처 → STABILIZED_HOLD |
| `Z` | STOW (0,0) |
| `T0` / `T1` | Torque Off / On (T1은 FAULT 해제 겸용) |
| `I R/S/J` | IMU 소스: Real / Sim / inJect |
| `V R/G` | 자세: Rotation Vector / Game RV |
| `Q w x y z` | 쿼터니언 주입 (INJECT) |
| `?` | 상태 출력 |

## IMU 오차에 대해 (설계에 이미 반영된 사항)

**핵심: STABILIZED_HOLD는 차동 방식이라 일정한 yaw bias가 상쇄된다.**
캡처 시점과 현재 시점의 자세 '차이'만 쓰므로, BNO085의 절대 yaw가 5° 틀려도
두 시점에 똑같이 틀리면 결과는 정확. Phase 3에는 절대 오차가 거의 안 물린다.

문제가 되는 건 (1) 시간에 따른 drift, (2) 지자기 간섭에 의한 yaw 점프.
그래서 기본 자세 소스를 **Game Rotation Vector(지자기 미사용)**로 설정했다
— 실내·모터 옆에서 자기 간섭 점프가 없어 안정화에 더 적합. 절대 yaw가
필요한 건 Phase 4(GPS coarse)부터이고, 그때 `V R`로 전환 + 팀원의
yaw bias 보정이 다룬다.

**BNO085 오차 정량화 프로토콜 (Stage C에서 30분 투자):**
1. 정적 drift: 보드를 고정하고 10분 로그 → pYaw 변화량 = drift율 (°/min)
2. 왕복 시험: 보드를 손으로 360° 돌렸다가 정확히 원위치 → pYaw 복귀 오차
3. Rotation vs Game 비교: `V R`과 `V G`로 같은 동작 반복 → 지자기 점프 유무
4. (선택) 레이저 포인터를 짐벌에 붙이고 벽의 표식 조준 → HOLD 상태로
   베이스를 돌린 뒤 점 이동거리 측정 → 거리로 나누면 각도 오차 [deg]
   (3m 거리에서 5cm 이동 = 약 1°) — 가장 값싼 정량 측정법

로그는 CSV로 나오므로 저장해서 파이썬 시뮬레이터 replay로 분석 가능.

## 텔레메트리 CSV 열 정의

```
t_ms, mode, imuSrc, acc, pYaw, pPitch, pRoll,
gYawCmd, gPitchCmd, dxlYaw, dxlPitch, curY, curP, limit
```
mode: 0=STOW 1=MANUAL 2=HOLD 3=FAULT / imuSrc: 0=REAL 1=SIM 2=INJECT
acc: BNO085 accuracy 0~3 / cur: DXL 전류 (×2.69mA) / limit: clamp 발생

## 안전 장치 (내장)

- 각도 clamp (Yaw ±170°, Pitch −10~+90°) + rate limit 180°/s
- MANUAL heartbeat: 2초 무명령 시 STOW
- DXL 온도 70°C 초과 시 FAULT + torque off (T1으로 해제)
- Current-based Position 모드: 전류 807mA 제한 → 걸림 시 강제로 밀지 않음
