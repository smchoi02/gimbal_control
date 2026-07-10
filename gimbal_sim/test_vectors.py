# -*- coding: utf-8 -*-
"""
test_vectors.py — 부호·좌표계 규약 검증 (test_vector.md의 실행 가능 버전)

실행: python3 test_vectors.py
모든 테스트 통과 시 "ALL PASSED". 하나라도 실패하면 어디가 틀렸는지 출력.

★ 이 테스트가 두 SW 담당자의 '같은 결과 도출' 기준이다 (로드맵 Phase 0 완료 조건).
★ C++ 이식 후 같은 입력으로 같은 출력이 나오는지 이 파일로 재검증한다.
"""
import math
from config import deg_to_i7, TYPICAL_LAG_S
from geometry import (geo_to_ned, predict_position, euler_to_quat,
                      quat_to_euler, ned_to_body, body_to_gimbal,
                      coarse_pipeline)
from controller import GimbalCommandFilter, wrap_deg, deg_to_dxl_units, dxl_units_to_deg
from stabilization import direction_from_gimbal_hold, capture_hold_direction
from fsm import GimbalFSM, State
import config as cfg

PASS, FAIL = 0, 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ✓ {name}")
    else:
        FAIL += 1
        print(f"  ✗ {name}  {detail}")


def approx(a, b, tol=0.5):
    return abs(a - b) < tol


# 기준 위치: 서울 근교 (37.5N, 127.0E)
LAT0, LON0 = 37.5000000, 127.0000000
P_LAT, P_LON = deg_to_i7(LAT0), deg_to_i7(LON0)


print("━━━ 1. geo_to_ned: NED 부호 규약 ━━━")

# 로켓이 정북 500m (위도 +0.0044917°)
r_lat = deg_to_i7(LAT0 + 500.0 / 111_320.0)
dN, dE, dD = geo_to_ned(r_lat, P_LON, 100, P_LAT, P_LON, 100)
check("정북 500m → dN≈+500", approx(dN, 500, 1), f"dN={dN:.2f}")
check("정북 500m → dE≈0", approx(dE, 0, 1), f"dE={dE:.2f}")

# 로켓이 정동 500m
r_lon = deg_to_i7(LON0 + 500.0 / (111_320.0 * math.cos(math.radians(LAT0))))
dN, dE, dD = geo_to_ned(P_LAT, r_lon, 100, P_LAT, P_LON, 100)
check("정동 500m → dE≈+500", approx(dE, 500, 1), f"dE={dE:.2f}")

# 로켓이 아래 30m (alt 낮음)
dN, dE, dD = geo_to_ned(P_LAT, P_LON, 70, P_LAT, P_LON, 100)
check("로켓 30m 아래 → dD=+30", approx(dD, 30, 0.01), f"dD={dD:.2f}")


print("━━━ 2. int32 정밀도: catastrophic cancellation 방지 ━━━")

# 위도 차이가 딱 1 unit (1e-7 deg = 1.1132 cm)일 때 정확히 분해되는가
dN, dE, dD = geo_to_ned(P_LAT + 1, P_LON, 0, P_LAT, P_LON, 0)
check("1e-7deg 차이 → 1.1132cm 분해", approx(dN, 0.011132, 1e-6), f"dN={dN:.7f}")

# float32로 같은 계산을 하면 어떻게 되는지 (실패 사례 시연)
import struct
f32 = lambda x: struct.unpack('f', struct.pack('f', x))[0]
lat_a_f32 = f32(LAT0 + 1e-7)
lat_b_f32 = f32(LAT0)
naive = (lat_a_f32 - lat_b_f32) * 111_320.0
check("float32 직접 뺄셈은 오차 발생 (시연)", not approx(naive, 0.011132, 1e-3),
      f"naive={naive:.7f} (int32 방식과 비교)")


print("━━━ 3. 쿼터니언 ↔ 오일러 왕복 ━━━")

for (y, p, r) in [(0, 0, 0), (45, 10, -5), (-120, 30, 15), (170, -5, 0)]:
    q = euler_to_quat(y, p, r)
    y2, p2, r2 = quat_to_euler(*q)
    check(f"왕복 ({y},{p},{r})", approx(y2, y, 0.01) and approx(p2, p, 0.01) and approx(r2, r, 0.01),
          f"→ ({y2:.2f},{p2:.2f},{r2:.2f})")


print("━━━ 4. Body 변환 + 짐벌각 부호 규약 ━━━")

# 페이로드가 북쪽을 봄 (yaw=0). 목표가 정북 수평 → yaw=0, pitch=0
q_north = euler_to_quat(0, 0, 0)
yaw, pitch = body_to_gimbal(ned_to_body((100, 0, 0), *q_north))
check("북향 페이로드, 정북 목표 → yaw 0", approx(yaw, 0), f"yaw={yaw:.2f}")
check("수평 목표 → pitch 0", approx(pitch, 0), f"pitch={pitch:.2f}")

# 목표가 정동 → yaw +90 (시계방향 = +)
yaw, pitch = body_to_gimbal(ned_to_body((0, 100, 0), *q_north))
check("정동 목표 → yaw +90 (시계+)", approx(yaw, 90), f"yaw={yaw:.2f}")

# 목표가 정서 → yaw -90
yaw, pitch = body_to_gimbal(ned_to_body((0, -100, 0), *q_north))
check("정서 목표 → yaw -90", approx(yaw, -90), f"yaw={yaw:.2f}")

# 목표가 전방 50m + 아래 30m → pitch +31 (아래 = +)
yaw, pitch = body_to_gimbal(ned_to_body((50, 0, 30), *q_north))
check("전방50+아래30 → pitch +31 (아래+)", approx(pitch, 31, 1), f"pitch={pitch:.2f}")

# 페이로드가 동쪽을 봄 (yaw=90). 목표는 정북 → 목표는 페이로드의 좌측 → yaw -90
q_east = euler_to_quat(90, 0, 0)
yaw, pitch = body_to_gimbal(ned_to_body((100, 0, 0), *q_east))
check("동향 페이로드, 정북 목표 → yaw -90", approx(yaw, -90), f"yaw={yaw:.2f}")

# 목표가 바로 아래 → pitch +90, NaN 없이
yaw, pitch = body_to_gimbal(ned_to_body((0, 0, 50), *q_north))
check("바로 아래 목표 → pitch +90, NaN 없음",
      approx(pitch, 90) and not math.isnan(yaw), f"pitch={pitch:.2f}, yaw={yaw:.2f}")


print("━━━ 5. 통신 지연 보정 ━━━")

# 로켓이 북으로 30m/s, 150ms 지연 → 4.5m 북쪽 보정
lat2, lon2, alt2 = predict_position(P_LAT, P_LON, 100, 30, 0, 0, 0.15)
dN, dE, dD = geo_to_ned(lat2, lon2, alt2, P_LAT, P_LON, 100)
check("북 30m/s × 150ms → +4.5m 보정", approx(dN, 4.5, 0.05), f"dN={dN:.3f}")

# 하강 20m/s (vD=+20), 150ms → 고도 3m 감소
lat2, lon2, alt2 = predict_position(P_LAT, P_LON, 100, 0, 0, 20, 0.15)
check("하강 20m/s × 150ms → alt -3m", approx(alt2, 97, 0.05), f"alt={alt2:.2f}")


print("━━━ 6. 전체 파이프라인 (coarse_pipeline) ━━━")

rocket = dict(lat_i7=deg_to_i7(LAT0 + 50/111_320.0), lon_i7=P_LON,
              alt_m=70, vN=0, vE=0, vD=0)
payload = dict(lat_i7=P_LAT, lon_i7=P_LON, alt_m=100)
# 페이로드 북향: 목표는 북쪽 50m + 아래 30m → yaw 0, pitch +31
yaw, pitch, dNED, dB = coarse_pipeline(rocket, payload, q_north)
check("파이프라인: 북50+아래30 → yaw 0", approx(yaw, 0), f"yaw={yaw:.2f}")
check("파이프라인: pitch +31", approx(pitch, 31, 1), f"pitch={pitch:.2f}")

# 페이로드가 90° 돌면 (동향) 같은 목표가 yaw -90으로
yaw, pitch, _, _ = coarse_pipeline(rocket, payload, q_east)
check("페이로드 90° 회전 → yaw -90 (상쇄 원리)", approx(yaw, -90), f"yaw={yaw:.2f}")


print("━━━ 7. STABILIZED_HOLD (Phase 3 원리) ━━━")

# 북쪽 수평을 고정한 채 페이로드가 0→90° 회전하면 짐벌 yaw가 0→-90으로 상쇄
hold_dir = (1.0, 0.0, 0.0)
for payload_yaw in [0, 30, 60, 90]:
    q = euler_to_quat(payload_yaw, 0, 0)
    gy, gp = direction_from_gimbal_hold(hold_dir, q)
    check(f"페이로드 {payload_yaw}° 회전 → 짐벌 yaw {-payload_yaw}°",
          approx(gy, -payload_yaw), f"gimbal_yaw={gy:.2f}")

# capture: 짐벌 (yaw 45, pitch 20)로 보고 있을 때 그 방향을 잡아 고정 → 재계산 시 동일 각 반환
q0 = euler_to_quat(10, 5, 0)
captured = capture_hold_direction(45, 20, q0)
gy, gp = direction_from_gimbal_hold(captured, q0)
check("capture→hold 왕복 일치", approx(gy, 45, 0.01) and approx(gp, 20, 0.01),
      f"({gy:.2f},{gp:.2f})")


print("━━━ 8. 명령 필터 (clamp / rate limit) ━━━")

f = GimbalCommandFilter(dt=0.02)   # 50Hz → max_step = 3.6°
f.reset(0, 0)
# yaw 200° 명령 = wrap하면 -160° (동일한 물리 방향). rate limit로 -3.6°부터 이동
y, p = f.step(200, 50)
check("명령 정규화: 200°=-160° 방향, 첫 스텝 -3.6°", approx(y, -3.6, 0.01), f"y={y:.2f}")
check("limit_flag 세워짐 (rate limit)", f.limit_flag)

# 여러 사이클 돌리면 -160에 수렴
for _ in range(200):
    y, p = f.step(200, 50)
check("수렴: yaw -160 도달 (wrap된 명령 방향)", approx(y, -160, 0.01), f"y={y:.2f}")
check("수렴: pitch는 +50 도달", approx(p, 50, 0.01), f"p={p:.2f}")

# 물리 한계 초과 명령: 175°는 wrap해도 175 → 한계 170으로 clamp
f.reset(0, 0)
for _ in range(200):
    y, p = f.step(175, 0)
check("한계 초과 명령 175° → +170 클램프", approx(y, 170, 0.01), f"y={y:.2f}")

# pitch 한계
f.reset(0, 0)
for _ in range(200):
    y, p = f.step(0, -50)
check("pitch 하한 -10 클램프", approx(p, -10, 0.01), f"p={p:.2f}")

# dead zone 물리 제약: +170에서 -170 명령 → 최단경로(20°)가 아니라
# 반대로 340° 되돌아가야 함. 첫 스텝이 '-' 방향인지 확인
f.reset(170, 0)
y, p = f.step(-170, 0)
check("dead zone: +170→-170은 되돌아감 (첫 스텝 166.4)",
      approx(y, 170 - 3.6, 0.01), f"y={y:.2f}")


print("━━━ 9. DYNAMIXEL 단위 변환 ━━━")

check("0° → 2048 units", deg_to_dxl_units(0) == 2048)
check("+90° → 3072 units", deg_to_dxl_units(90) == 3072)
check("-90° → 1024 units", deg_to_dxl_units(-90) == 1024)
check("왕복 일치", approx(dxl_units_to_deg(deg_to_dxl_units(45.5)), 45.5, 0.1))


print("━━━ 10. FSM 전이 ━━━")

fsm = GimbalFSM()
s = fsm.update(0.0, sensors_ok=True)
check("INIT → STOW", s == State.STOW)
s = fsm.update(0.1, armed=True, rocket_packet=True)
check("STOW → COARSE_TRACK (armed+comm)", s == State.COARSE_TRACK)
s = fsm.update(0.2, armed=True, vision_valid=True, rocket_packet=True)
check("COARSE → VISION_TRACK", s == State.VISION_TRACK)
# 통신+영상 동시 상실 → STABILIZED_HOLD
s = fsm.update(1.0, armed=True)
check("통신·영상 상실 → STABILIZED_HOLD", s == State.STABILIZED_HOLD)
# 5초 경과 → SCAN
s = fsm.update(6.1, armed=True)
check("HOLD 5s 초과 → SCAN", s == State.SCAN)
# 통신 재개 → COARSE
s = fsm.update(6.2, armed=True, rocket_packet=True)
check("SCAN → COARSE (통신 재개)", s == State.COARSE_TRACK)
# fault → FAULT
s = fsm.update(6.3, armed=True, rocket_packet=True, fault=True)
check("fault → FAULT", s == State.FAULT)
s = fsm.update(6.4, armed=True, rocket_packet=True)
check("FAULT 유지 (clear 전)", s == State.FAULT)


print()
print(f"━━━ 결과: {PASS} passed, {FAIL} failed ━━━")
if FAIL == 0:
    print("ALL PASSED ✓  — 좌표계·부호 규약이 코드로 확정되었습니다.")
else:
    print("실패 항목을 확인하세요.")
    raise SystemExit(1)
