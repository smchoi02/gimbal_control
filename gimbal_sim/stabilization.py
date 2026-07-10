# -*- coding: utf-8 -*-
"""
stabilization.py — STABILIZED_HOLD 로직 (Phase 3)

목표: 페이로드가 회전해도 카메라가 세계 기준 같은 방향을 계속 보게 한다.

원리: 목표 방향을 NED 단위벡터로 고정해두고, 매 사이클 최신 자세(쿼터니언)로
Body 변환 → 짐벌각 재계산. 페이로드가 돌면 R_N→B가 바뀌므로 짐벌각이
자동으로 반대 회전 → 상쇄.

C++ 이식 대상: stabilization.h — Phase 3 데모(손으로 돌려도 카메라 고정)의 본체.
"""
import math
from geometry import ned_to_body, body_to_gimbal


def direction_from_gimbal_hold(target_dir_ned, quat):
    """
    STABILIZED_HOLD 한 사이클.

    target_dir_ned : 고정할 세계 방향 (NED 단위벡터). 예: 북쪽 수평 = (1,0,0)
    quat           : 현재 페이로드 자세 (qw,qx,qy,qz)

    반환: (yaw_deg, pitch_deg) — 카메라가 그 방향을 유지하기 위한 짐벌각
    """
    dB = ned_to_body(target_dir_ned, *quat)
    return body_to_gimbal(dB)


def capture_hold_direction(current_gimbal_yaw_deg, current_gimbal_pitch_deg, quat):
    """
    HOLD 진입 순간 호출: 지금 카메라가 보고 있는 세계 방향을 계산해 고정 목표로 저장.

    짐벌각 → Body 방향 벡터 → NED 방향 벡터 (자세의 역회전).

    반환: NED 단위벡터
    """
    from geometry import quat_to_R_b2n, mat_vec
    y = math.radians(current_gimbal_yaw_deg)
    p = math.radians(current_gimbal_pitch_deg)
    # 짐벌각 → body 벡터 (x=전방, y=우, z=아래; pitch+=아래)
    v_body = (math.cos(p)*math.cos(y),
              math.cos(p)*math.sin(y),
              math.sin(p))
    R_b2n = quat_to_R_b2n(*quat)
    return mat_vec(R_b2n, v_body)
