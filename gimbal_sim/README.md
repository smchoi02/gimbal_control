# 짐벌 제어 파이썬 시뮬레이터 (gimbal_sim)

IREC 페이로드 짐벌 제어 알고리즘을 부품 없이 검증하는 도구.
부품 도착 전 Phase 2~4의 **알고리즘 로직**을 확정하고, 도착 후에는
같은 test vector로 C++ 코드를 재검증하는 용도.

## 실행 방법

```bash
# 1) 부호·좌표계 규약 검증 (46개 테스트)
python3 test_vectors.py

# 2) 60초 낙하 시나리오 통합 시뮬레이션 (그래프 생성)
python3 run_sim.py    # → sim_result.png
```

의존성: Python 3.8+, matplotlib (run_sim만). test_vectors는 표준 라이브러리만 사용.

## 파일 구조 및 C++ 이식 맵

| 파이썬 파일 | 역할 | Phase | C++ 이식 대상 (OpenRB) |
|---|---|---|---|
| `config.py` | 좌표계 규약, 짐벌 한계, 주기 | 0 | `gimbal_config.h` |
| `geometry.py` | geo_to_ned, 쿼터니언, 짐벌각 계산 | 4 | `geometry.h/.cpp` |
| `controller.py` | clamp, rate limit, DXL 단위 변환 | 2 | `gimbal_controller.h` |
| `stabilization.py` | STABILIZED_HOLD (방향 고정) | 3 | `stabilization.h` |
| `fsm.py` | 상태머신 | 2~4 | `state_machine.h` |
| `test_vectors.py` | 검증 테스트 (test_vector.md 실행판) | 0 | 이식 후 재검증용 |
| `run_sim.py` | 낙하 시나리오 통합 시뮬레이션 | — | (PC 전용) |

`geometry.py`는 numpy 없이 스칼라 연산만 사용 → C++로 한 줄씩 대응 이식 가능.

## 확정된 규약 (Phase 0 — 이 코드가 기준)

- **NED** 세계 좌표계 (D+ = 아래), **Body**: X=전방, Y=우, Z=아래
- **Yaw+** = 위에서 봤을 때 시계방향 / **Pitch+** = 아래 (수평 0°, 바로 아래 +90°)
- 짐벌 한계: Yaw ±170°, Pitch −10°~+90°
- GPS 위경도는 **int32 (1e-7 deg)** 로 다룸 — float32 직접 뺄셈 금지 (테스트 2번이 시연)
- DYNAMIXEL: 4096 units/rev, center 2048, Yaw=ID1, Pitch=ID2

## 시뮬레이터가 잡아낸 설계 이슈 (하드웨어였으면 사고)

1. **Yaw dead zone과 rate limit**: ±170° 제한 짐벌은 +170↔−170 wrap 이동이
   물리적으로 불가능 → rate limit 차이 계산은 wrap 없이 선형으로 해야 함.
   (`controller.py` step() 주석 참고)
2. **로켓이 페이로드 후방에 오는 기하**: 목표가 yaw ±180° 근처(dead zone)면
   지향 자체가 불가능. 연속 스핀하는 페이로드도 주기적으로 이 구간 통과.
   → 운용 설계에서 로켓 예상 방위를 전방에 두거나 요동 억제 필요.

## 시뮬레이션 결과 요약 (run_sim.py, 완벽 센서 가정)

| 조건 | 지향 오차 RMS | 최대 |
|---|---|---|
| 통신 지연 보정 ON (P+V·Δt) | 0.002° | 0.011° |
| 통신 지연 보정 OFF | 0.485° | 4.43° |

→ 지연 보정의 효과가 수치로 확인됨 (문서의 손계산 예측 4.5m/150ms와 일치).
실제로는 여기에 센서 오차(GPS ±1.3°, BNO085 yaw ±2°)가 더해짐 — 그건
알고리즘이 아니라 센서 한계이므로 이 시뮬레이터의 검증 범위 밖.

## 부품 도착 후 사용법

1. C++로 `geometry.py` 이식 → `test_vectors.py`의 입출력 값으로 단위 시험
2. 부호가 실물과 반대면 **코드 수정 말고 `config.py`(→gimbal_config.h)의
   sign 상수만 뒤집기** (test vector 재실행으로 확인)
3. 실비행/실험 로그를 `run_sim.py` 형식으로 replay → Phase 6~7 필터 검증
