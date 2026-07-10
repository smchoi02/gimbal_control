# -*- coding: utf-8 -*-
"""
geometry.py — 핵심 좌표 변환 알고리즘 (Phase 4 coarse track의 심장)

C++ 이식 대상: 이 파일의 모든 함수를 OpenRB의 geometry.h / geometry.cpp 로 옮긴다.
numpy 없이 스칼라 연산만 사용 → C++로 한 줄씩 대응 이식 가능하게 작성.

파이프라인:
  (1) predict_position   : 통신 지연 보정 (P_pred = P + V·Δt)
  (2) geo_to_ned         : 위경도 차이 → NED 미터 벡터 (int32 산술)
  (3) quat_to_R_n2b      : 쿼터니언 → NED→Body 회전 행렬
  (4) ned_to_body        : NED 벡터 → Body 벡터
  (5) body_to_gimbal     : Body 벡터 → yaw/pitch 명령각
"""
import math
from config import M_PER_1E7DEG, M_PER_DEG_LAT


# ═══════════════════════════════════════════════════════
# (1) 통신 지연 보정
# ═══════════════════════════════════════════════════════
def predict_position(lat_i7, lon_i7, alt_m, vN, vE, vD, dt_s):
    """
    로켓 위치를 dt_s 초만큼 속도로 외삽.
    입력: 위경도 int32(1e-7deg), 고도 m, NED 속도 m/s, 지연시간 s
    출력: 외삽된 (lat_i7, lon_i7, alt_m)

    C++: int64 승격 후 정수 나눗셈 주의.
    """
    # 북쪽 이동 → 위도 증가. m → 1e-7deg 단위 변환
    dlat_i7 = (vN * dt_s) / M_PER_1E7DEG
    # 동쪽 이동 → 경도 증가 (위도에 따른 축소 보정)
    coslat = math.cos(math.radians(lat_i7 * 1e-7))
    dlon_i7 = (vE * dt_s) / (M_PER_1E7DEG * coslat)
    # 아래 이동(vD+) → 고도 감소
    dalt_m = -vD * dt_s

    return (int(round(lat_i7 + dlat_i7)),
            int(round(lon_i7 + dlon_i7)),
            alt_m + dalt_m)


# ═══════════════════════════════════════════════════════
# (2) NED 상대 위치 (int32 산술 → 정밀도 보존)
# ═══════════════════════════════════════════════════════
def geo_to_ned(lat_r_i7, lon_r_i7, alt_r_m,
               lat_p_i7, lon_p_i7, alt_p_m):
    """
    로켓(R)이 페이로드(P)로부터 어느 방향에 있는지 NED 벡터로.

    입력 위경도는 int32 (1e-7 deg). 차이를 정수로 먼저 계산 →
    catastrophic cancellation 방지 (float32로 위경도 직접 빼기 금지).

    반환: (dN, dE, dD) [m]
      dN+ : 로켓이 북쪽
      dE+ : 로켓이 동쪽
      dD+ : 로켓이 아래 (alt_p > alt_r)
    """
    dlat_i7 = lat_r_i7 - lat_p_i7          # 정수 뺄셈: 정보 손실 없음
    dlon_i7 = lon_r_i7 - lon_p_i7

    dN = dlat_i7 * M_PER_1E7DEG
    coslat = math.cos(math.radians(lat_p_i7 * 1e-7))
    dE = dlon_i7 * M_PER_1E7DEG * coslat
    dD = alt_p_m - alt_r_m                  # 로켓이 아래면 양수

    return (dN, dE, dD)


# ═══════════════════════════════════════════════════════
# (3) 자세: 쿼터니언 / 오일러 → 회전 행렬
# ═══════════════════════════════════════════════════════
def quat_normalize(qw, qx, qy, qz):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    return (qw/n, qx/n, qy/n, qz/n)


def quat_to_R_b2n(qw, qx, qy, qz):
    """
    쿼터니언 → Body→NED 회전 행렬 (3x3, 리스트의 리스트).
    q는 body 벡터를 NED로 돌리는 자세 쿼터니언 (v_N = R · v_B).
    BNO085 Rotation Vector와 동일한 의미.
    """
    qw, qx, qy, qz = quat_normalize(qw, qx, qy, qz)
    return [
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz),   2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),     2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)],
    ]


def quat_to_R_n2b(qw, qx, qy, qz):
    """NED→Body = (Body→NED)의 전치."""
    R = quat_to_R_b2n(qw, qx, qy, qz)
    return [[R[0][0], R[1][0], R[2][0]],
            [R[0][1], R[1][1], R[2][1]],
            [R[0][2], R[1][2], R[2][2]]]


def euler_to_quat(yaw_deg, pitch_deg, roll_deg):
    """
    ZYX (yaw→pitch→roll) 오일러 → 쿼터니언 (body→NED).
    시뮬레이터에서 가짜 자세 만들 때 사용. BNO085 출력 모사.
    """
    y = math.radians(yaw_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    r = math.radians(roll_deg) * 0.5
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)


def quat_to_euler(qw, qx, qy, qz):
    """쿼터니언 → (yaw, pitch, roll) deg. 디버그·로그용."""
    yaw = math.degrees(math.atan2(2*(qw*qz + qx*qy),
                                  1 - 2*(qy*qy + qz*qz)))
    s = 2*(qw*qy - qz*qx)
    s = max(-1.0, min(1.0, s))
    pitch = math.degrees(math.asin(s))
    roll = math.degrees(math.atan2(2*(qw*qx + qy*qz),
                                   1 - 2*(qx*qx + qy*qy)))
    return (yaw, pitch, roll)


# ═══════════════════════════════════════════════════════
# (4) NED → Body
# ═══════════════════════════════════════════════════════
def mat_vec(R, v):
    """3x3 행렬 × 3벡터. C++ 이식 시 그대로 3중 곱."""
    return (R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2])


def ned_to_body(dNED, qw, qx, qy, qz):
    """NED 상대 벡터를 Body 좌표계로 회전."""
    R_n2b = quat_to_R_n2b(qw, qx, qy, qz)
    return mat_vec(R_n2b, dNED)


# ═══════════════════════════════════════════════════════
# (5) Body → 짐벌 명령각
# ═══════════════════════════════════════════════════════
def body_to_gimbal(dB):
    """
    Body 벡터 (x=전방, y=우, z=아래) → (yaw_deg, pitch_deg)

    yaw+   = 시계방향 (목표가 우측이면 +)
    pitch+ = 아래 (목표가 아래면 +)   ← 로드맵 §6 규약

    반환각은 클램프 전 원시값. 클램프는 controller.py 담당.
    """
    x, y, z = dB
    yaw_deg = math.degrees(math.atan2(y, x))
    horiz = math.sqrt(x*x + y*y)
    pitch_deg = math.degrees(math.atan2(z, horiz))   # z+(아래) → pitch+
    return (yaw_deg, pitch_deg)


# ═══════════════════════════════════════════════════════
# 통합 파이프라인 (Phase 4 전체)
# ═══════════════════════════════════════════════════════
def coarse_pipeline(rocket, payload, quat, comm_delay_s=0.0):
    """
    전체 coarse track 파이프라인. OpenRB loop()에서 매 사이클 호출될 함수.

    rocket : dict(lat_i7, lon_i7, alt_m, vN, vE, vD)
    payload: dict(lat_i7, lon_i7, alt_m)
    quat   : (qw, qx, qy, qz)  — BNO085 자세
    comm_delay_s: 통신 지연 (t_now − t_R)

    반환: (yaw_deg, pitch_deg, dNED, dB)  — 원시 명령각 + 중간값(디버그)
    """
    # (1) 지연 보정
    lat_r, lon_r, alt_r = predict_position(
        rocket['lat_i7'], rocket['lon_i7'], rocket['alt_m'],
        rocket['vN'], rocket['vE'], rocket['vD'], comm_delay_s)

    # (2) NED 상대 위치
    dNED = geo_to_ned(lat_r, lon_r, alt_r,
                      payload['lat_i7'], payload['lon_i7'], payload['alt_m'])

    # (3)+(4) Body 변환
    dB = ned_to_body(dNED, *quat)

    # (5) 짐벌각
    yaw_deg, pitch_deg = body_to_gimbal(dB)
    return (yaw_deg, pitch_deg, dNED, dB)
