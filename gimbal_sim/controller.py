# -*- coding: utf-8 -*-
"""
controller.py — 짐벌 명령 안전 처리 (Phase 2: MANUAL 모드의 핵심)

C++ 이식 대상: gimbal_controller.h — OpenRB에서 DYNAMIXEL 명령 직전에 통과하는 층.

책임:
  - angle clamp   : 물리 한계 내로 제한
  - rate limit    : 사이클당 최대 이동량 제한
  - wrap 처리     : yaw ±180° 경계 연속성
  - DXL 단위 변환 : deg ↔ DYNAMIXEL raw units
"""
import config as cfg


def wrap_deg(a):
    """각도를 (-180, +180] 범위로 정규화."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GimbalCommandFilter:
    """
    명령각을 받아 clamp + rate limit을 적용한 뒤 반환하는 상태 보유 필터.
    OpenRB에서는 이 클래스가 loop 내 static 구조체로 존재.
    """

    def __init__(self, dt=cfg.CONTROL_DT):
        self.dt = dt
        self.max_step = cfg.MAX_RATE_DEG_S * dt   # 사이클당 최대 이동 deg
        self.yaw_out = 0.0     # 마지막 출력각 (rate limit 기준점)
        self.pitch_out = 0.0
        self.limit_flag = False   # 이번 사이클에 clamp 발생 여부 (STAT_GIMBAL에 실림)

    def reset(self, yaw=0.0, pitch=0.0):
        self.yaw_out = yaw
        self.pitch_out = pitch
        self.limit_flag = False

    def step(self, yaw_cmd, pitch_cmd):
        """
        한 사이클 처리. 반환: (yaw_safe, pitch_safe)

        ★ 설계 노트 (시뮬레이터 테스트에서 발견된 함정):
        Yaw ±170° 제한 짐벌은 +170°~180°~-170° 구간이 dead zone이라
        wrap-around 최단경로 이동이 물리적으로 불가능하다.
        따라서 rate limit의 차이 계산은 wrap 없이 '선형'으로 해야 한다.
        (+170에서 -170으로 가려면 340°를 되돌아가야 함 — 이게 물리적 현실)
        wrap_deg는 '명령 입력 정규화'에만 사용한다 (예: 200° 명령 → -160°).
        """
        self.limit_flag = False

        # ── 1) 명령 정규화 + angle clamp (물리 한계) ──
        yaw_norm = wrap_deg(yaw_cmd)          # 200° → -160° (동일 방향의 표준 표현)
        yaw_c = clamp(yaw_norm, cfg.YAW_MIN_DEG, cfg.YAW_MAX_DEG)
        pitch_c = clamp(pitch_cmd, cfg.PITCH_MIN_DEG, cfg.PITCH_MAX_DEG)
        if yaw_c != yaw_norm or pitch_c != pitch_cmd:
            self.limit_flag = True

        # ── 2) rate limit — 선형 차이 (wrap 금지: dead zone 통과 불가) ──
        dyaw = yaw_c - self.yaw_out
        if abs(dyaw) > self.max_step:
            yaw_c = self.yaw_out + self.max_step * (1 if dyaw > 0 else -1)
            self.limit_flag = True

        dpitch = pitch_c - self.pitch_out
        if abs(dpitch) > self.max_step:
            pitch_c = self.pitch_out + self.max_step * (1 if dpitch > 0 else -1)
            self.limit_flag = True

        self.yaw_out = yaw_c        # -170..+170 범위이므로 wrap 불필요
        self.pitch_out = pitch_c
        return (self.yaw_out, self.pitch_out)


# ═══════════════════════════════════════════════════════
# DYNAMIXEL 단위 변환
# ═══════════════════════════════════════════════════════
def deg_to_dxl_units(angle_deg):
    """
    짐벌각(deg, 0=기계 중앙) → DYNAMIXEL raw units.
    XL430: 4096 units/rev, center=2048.
    """
    units = cfg.DXL_CENTER_UNITS + angle_deg * cfg.DXL_UNITS_PER_REV / 360.0
    return int(round(units))


def dxl_units_to_deg(units):
    return (units - cfg.DXL_CENTER_UNITS) * 360.0 / cfg.DXL_UNITS_PER_REV
