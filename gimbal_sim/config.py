# -*- coding: utf-8 -*-
"""
config.py — 좌표계 규약 + 짐벌 파라미터 (gimbal_config.yaml 대응)

이 파일이 Phase 0 결정사항의 '단일 진실 소스(single source of truth)'다.
C++ 이식 시 gimbal_config.h 로 동일하게 옮긴다.

━━━ 좌표계 규약 (Phase 0 확정안) ━━━
세계 좌표계  : NED (North-East-Down). D+ = 아래
Body 좌표계  : X+ = 페이로드 전방, Y+ = 우측, Z+ = 아래
Yaw+        : 위에서 봤을 때 시계방향 (북→동 방향 회전)
Pitch+      : 아래로 회전 (수평=0°, 바로 아래=+90°)
              ※ 요약문서 v3의 pitch = atan2(-z, ...) 와 부호 다름에 주의.
              로드맵 §6 "pitch+ = 아래로 회전" 규약을 따른다.
              → pitch = atan2(+z_down, horiz)

━━━ GPS 정밀도 규약 ━━━
위경도는 int32 (1e-7 deg 단위)로 다룬다 (MAX-M10S UBX-NAV-PVT와 동일).
float32로 위경도를 직접 빼면 catastrophic cancellation 발생 → 금지.
"""

# ── 짐벌 물리 한계 (deg) ──────────────────────────────
YAW_MIN_DEG   = -170.0
YAW_MAX_DEG   = +170.0
PITCH_MIN_DEG = -10.0     # 수평보다 10° 위까지
PITCH_MAX_DEG = +90.0     # 바로 아래

# ── 명령 제한 (Phase 2: MANUAL 모드 safety) ───────────
MAX_RATE_DEG_S  = 180.0   # 짐벌 최대 각속도 (deg/s)
MAX_ACCEL_DEG_S2 = 720.0  # 최대 각가속도 (deg/s^2) — 초기엔 미사용 가능

# ── 루프 주기 ─────────────────────────────────────────
CONTROL_HZ = 50           # 짐벌 명령 루프 (로드맵 §6: 50~100Hz)
CONTROL_DT = 1.0 / CONTROL_HZ

# ── 통신 규약 ─────────────────────────────────────────
COMM_TIMEOUT_S = 0.5      # 이 시간 동안 로켓 패킷 없으면 HOLD
SCAN_AFTER_S   = 5.0      # HOLD 지속 시 SCAN 진입
TYPICAL_LAG_S  = 0.07     # LoRa 통신 평균 지연 (offset 추정 초기값)

# ── DYNAMIXEL 규약 ────────────────────────────────────
YAW_ID   = 1
PITCH_ID = 2
DXL_UNITS_PER_REV = 4096       # XL430: 4096 units = 360°
DXL_CENTER_UNITS  = 2048       # 짐벌 기계적 0° = 2048 units 가정 (실측 후 조정)

# ── 지구 상수 ─────────────────────────────────────────
M_PER_DEG_LAT = 111_320.0          # 위도 1도당 미터
M_PER_1E7DEG  = M_PER_DEG_LAT * 1e-7   # = 0.011132 m (int32 1단위당)

# ── GPS int32 변환 ────────────────────────────────────
def deg_to_i7(deg: float) -> int:
    """위경도 deg → int32 (1e-7 deg 단위). GPS 원본 형식."""
    return int(round(deg * 1e7))

def i7_to_deg(i7: int) -> float:
    return i7 * 1e-7
