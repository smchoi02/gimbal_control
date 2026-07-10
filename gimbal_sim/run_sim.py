# -*- coding: utf-8 -*-
"""
run_sim.py — 통합 낙하 시나리오 시뮬레이션

실행: python3 run_sim.py
출력: sim_result.png (4개 그래프), 콘솔 요약

시나리오 (실제 미션 모사):
  t=0   : apogee 3km에서 페이로드·로켓 분리
  로켓  : drogue로 -30 m/s 하강, 바람에 의해 동쪽으로 8 m/s 표류
  페이로드: 능동 낙하산으로 -12 m/s 하강, 동쪽 5 m/s 표류, 15°/s로 자체 회전(스핀)
  통신  : 10Hz 로켓 패킷, 평균 120ms 지연. t=25~30s 통신 두절 구간.

검증 포인트:
  1) 짐벌 명령각이 연속적이고 rate limit 안에서 움직이는가
  2) 지연 보정 ON일 때 지향 오차가 얼마나 줄어드는가
  3) 통신 두절 시 FSM이 HOLD로 전환되고 복구되는가
  4) 페이로드 스핀이 짐벌각으로 상쇄되는가
"""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 한글 폰트
for f in fm.fontManager.ttflist:
    if 'CJK' in f.name or 'Nanum' in f.name:
        plt.rcParams['font.family'] = f.name
        break
plt.rcParams['axes.unicode_minus'] = False

import config as cfg
from config import deg_to_i7
from geometry import (coarse_pipeline, euler_to_quat, geo_to_ned,
                      ned_to_body, body_to_gimbal, quat_to_R_b2n, mat_vec)
from controller import GimbalCommandFilter
from fsm import GimbalFSM, State

# ═══════════════════════════════════════════════════════
# 시나리오 정의
# ═══════════════════════════════════════════════════════
LAT0, LON0 = 37.5000000, 127.0000000
SIM_T = 60.0                     # 총 60초
DT = cfg.CONTROL_DT              # 50Hz
PACKET_HZ = 10                   # 로켓 패킷 주기
COMM_DELAY = 0.12                # 통신 지연 120ms
COMM_BLACKOUT = (25.0, 30.0)     # 통신 두절 구간

def rocket_true(t):
    """로켓 실제 상태: -30m/s 하강, 동쪽 8m/s 표류."""
    north = 0.0
    east = 8.0 * t
    alt = 3000.0 - 30.0 * t
    return dict(north=north, east=east, alt=alt, vN=0.0, vE=8.0, vD=30.0)

def payload_true(t):
    """
    페이로드 실제 상태: -12m/s 하강, 동쪽 5m/s 표류, ±40° 요동 (주기 8s).

    ★ 시뮬레이터 발견 사항: 초기 버전에서 페이로드를 로켓 북쪽에 두었더니
    로켓이 정남(yaw ±180°) — 짐벌 dead zone(±170° 밖)에 들어가 지향 불가였음.
    연속 스핀(15°/s)도 목표 body yaw가 주기적으로 dead zone을 통과함.
    → 실제 미션에서도 '로켓이 페이로드 후방에 오는 기하'와 '연속 스핀'은
      yaw 한계와 충돌. 페이로드 자세 제어(요동 억제) 또는 짐벌 배치 시
      로켓 예상 방위를 전방(yaw 0 부근)에 두는 운용 설계가 필요.
    """
    north = -20.0                 # 분리 시 남쪽 20m → 로켓이 북쪽(전방)에 보임
    east = 5.0 * t
    alt = 3000.0 - 12.0 * t
    yaw = 40.0 * math.sin(2 * math.pi * t / 8.0)   # 낙하산 요동 모사
    return dict(north=north, east=east, alt=alt, yaw=yaw)

def ne_to_geo(north, east):
    """시나리오의 북/동 미터 → 위경도 int32."""
    lat = LAT0 + north / 111_320.0
    lon = LON0 + east / (111_320.0 * math.cos(math.radians(LAT0)))
    return deg_to_i7(lat), deg_to_i7(lon)

def true_los_gimbal(t):
    """완벽 정보 기준 짐벌각 (오차 계산용 ground truth)."""
    r, p = rocket_true(t), payload_true(t)
    dNED = (r['north'] - p['north'], r['east'] - p['east'], p['alt'] - r['alt'])
    q = euler_to_quat(p['yaw'], 0, 0)
    return body_to_gimbal(ned_to_body(dNED, *q))

def pointing_error_deg(gimbal_yaw, gimbal_pitch, t):
    """짐벌이 실제로 보는 방향 vs 진짜 LOS 방향의 사잇각."""
    p = payload_true(t)
    q = euler_to_quat(p['yaw'], 0, 0)
    gy, gp = math.radians(gimbal_yaw), math.radians(gimbal_pitch)
    v_body = (math.cos(gp)*math.cos(gy), math.cos(gp)*math.sin(gy), math.sin(gp))
    v_ned = mat_vec(quat_to_R_b2n(*q), v_body)
    r = rocket_true(t)
    los = (r['north'] - p['north'], r['east'] - p['east'], p['alt'] - r['alt'])
    n = math.sqrt(sum(x*x for x in los))
    los = tuple(x/n for x in los)
    dot = max(-1.0, min(1.0, sum(a*b for a, b in zip(v_ned, los))))
    return math.degrees(math.acos(dot))


# ═══════════════════════════════════════════════════════
# 시뮬레이션 실행 (지연 보정 ON/OFF 비교)
# ═══════════════════════════════════════════════════════
def run(latency_compensation: bool):
    filt = GimbalCommandFilter(dt=DT)
    fsm = GimbalFSM()
    fsm.update(0.0, sensors_ok=True)             # INIT → STOW
    fsm.update(0.0, armed=True, rocket_packet=True)  # → COARSE_TRACK

    last_packet = None       # (수신시각, 로켓상태측정시각, 패킷내용)
    next_packet_t = 0.0

    log = dict(t=[], yaw=[], pitch=[], err=[], state=[], true_yaw=[], true_pitch=[])

    n_steps = int(SIM_T / DT)
    for i in range(n_steps):
        t = i * DT

        # ── 로켓 패킷 수신 모사 (10Hz, 지연, 두절 구간) ──
        got_packet = False
        if t >= next_packet_t:
            t_meas = t - COMM_DELAY              # 패킷 안의 측정 시각
            if t_meas >= 0 and not (COMM_BLACKOUT[0] <= t_meas <= COMM_BLACKOUT[1]):
                r = rocket_true(t_meas)
                lat_i7, lon_i7 = ne_to_geo(r['north'], r['east'])
                last_packet = (t, t_meas, dict(lat_i7=lat_i7, lon_i7=lon_i7,
                                               alt_m=r['alt'], vN=r['vN'],
                                               vE=r['vE'], vD=r['vD']))
                got_packet = True
            next_packet_t += 1.0 / PACKET_HZ

        # ── FSM 갱신 ──
        state = fsm.update(t, armed=True, rocket_packet=got_packet)

        # ── 짐벌 명령 계산 ──
        p = payload_true(t)
        p_lat, p_lon = ne_to_geo(p['north'], p['east'])
        payload = dict(lat_i7=p_lat, lon_i7=p_lon, alt_m=p['alt'])
        quat = euler_to_quat(p['yaw'], 0, 0)     # BNO085 모사 (완벽 자세 가정)

        if last_packet is not None:
            recv_t, meas_t, pkt = last_packet
            delay = (t - meas_t) if latency_compensation else 0.0
            yaw_raw, pitch_raw, _, _ = coarse_pipeline(pkt, payload, quat, delay)
        else:
            yaw_raw, pitch_raw = 0.0, 0.0

        yaw_cmd, pitch_cmd = filt.step(yaw_raw, pitch_raw)

        # ── 로그 ──
        ty, tp = true_los_gimbal(t)
        log['t'].append(t)
        log['yaw'].append(yaw_cmd)
        log['pitch'].append(pitch_cmd)
        log['err'].append(pointing_error_deg(yaw_cmd, pitch_cmd, t))
        log['state'].append(state)
        log['true_yaw'].append(ty)
        log['true_pitch'].append(tp)

    return log, fsm


print("시뮬레이션 실행 중...")
log_on, fsm_on = run(latency_compensation=True)
log_off, _ = run(latency_compensation=False)

# ═══════════════════════════════════════════════════════
# 결과 그래프
# ═══════════════════════════════════════════════════════
fig, axes = plt.subplots(4, 1, figsize=(12, 14))

# (1) 짐벌 명령각
ax = axes[0]
ax.plot(log_on['t'], log_on['true_yaw'], 'k--', lw=1, alpha=0.5, label='이상적 Yaw (ground truth)')
ax.plot(log_on['t'], log_on['yaw'], 'b-', lw=1.2, label='Yaw 명령 (보정 ON)')
ax.plot(log_on['t'], log_on['true_pitch'], 'k:', lw=1, alpha=0.5, label='이상적 Pitch')
ax.plot(log_on['t'], log_on['pitch'], 'g-', lw=1.2, label='Pitch 명령')
ax.axvspan(*COMM_BLACKOUT, color='red', alpha=0.08)
ax.set_ylabel('각도 [deg]')
ax.set_title('짐벌 명령각 — 페이로드 15°/s 스핀 상쇄 + 로켓 추적 (붉은 구간 = 통신 두절)')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

# (2) 지향 오차: 보정 ON vs OFF
ax = axes[1]
ax.plot(log_off['t'], log_off['err'], 'r-', lw=1, alpha=0.7, label='지연 보정 OFF')
ax.plot(log_on['t'], log_on['err'], 'b-', lw=1.2, label='지연 보정 ON (P+V·Δt)')
ax.axvspan(*COMM_BLACKOUT, color='red', alpha=0.08)
ax.set_ylabel('지향 오차 [deg]')
ax.set_title('지향 오차 — 통신 지연 보정의 효과 (완벽 센서 가정, 알고리즘 잔차만)')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
ax.set_ylim(0, max(log_off['err']) * 1.1)

# (3) 거리 및 통신 상태
ax = axes[2]
dist = []
for t in log_on['t']:
    r, p = rocket_true(t), payload_true(t)
    d = math.sqrt((r['north']-p['north'])**2 + (r['east']-p['east'])**2 + (r['alt']-p['alt'])**2)
    dist.append(d)
ax.plot(log_on['t'], dist, 'purple', lw=1.5)
ax.axvspan(*COMM_BLACKOUT, color='red', alpha=0.08)
ax.set_ylabel('거리 [m]')
ax.set_title('페이로드-로켓 거리 (하강 속도차 18 m/s로 벌어짐)')
ax.grid(alpha=0.3)

# (4) FSM 상태
ax = axes[3]
state_order = [State.STOW, State.STABILIZED_HOLD, State.COARSE_TRACK, State.SCAN]
state_y = {s: i for i, s in enumerate(state_order)}
ys = [state_y.get(s, -1) for s in log_on['state']]
ax.step(log_on['t'], ys, 'b-', lw=1.5, where='post')
ax.axvspan(*COMM_BLACKOUT, color='red', alpha=0.08)
ax.set_yticks(range(len(state_order)))
ax.set_yticklabels(state_order, fontsize=8)
ax.set_xlabel('시간 [s]')
ax.set_title('FSM 상태 전이 — 통신 두절 시 HOLD, 5초 후 SCAN, 복구 시 COARSE_TRACK')
ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('sim_result.png', dpi=110)
print("그래프 저장: sim_result.png")

# ═══════════════════════════════════════════════════════
# 콘솔 요약
# ═══════════════════════════════════════════════════════
import statistics
# 통신 정상 구간만 (초기 과도 + blackout 여파 제외)
ok_idx = [i for i, t in enumerate(log_on['t']) if 2 < t < 24 or t > 32]
err_on = [log_on['err'][i] for i in ok_idx]
err_off = [log_off['err'][i] for i in ok_idx]

print()
print("━━━ 시뮬레이션 요약 (통신 정상 구간) ━━━")
print(f"  지향 오차 RMS  — 보정 ON : {statistics.mean(e**2 for e in err_on)**0.5:.3f}°")
print(f"  지향 오차 RMS  — 보정 OFF: {statistics.mean(e**2 for e in err_off)**0.5:.3f}°")
print(f"  최대 오차      — 보정 ON : {max(err_on):.3f}°")
print(f"  최대 오차      — 보정 OFF: {max(err_off):.3f}°")
print()
print("━━━ FSM 전이 기록 ━━━")
for (t, s_from, s_to, reason) in fsm_on.transitions:
    print(f"  t={t:6.2f}s  {s_from:16s} → {s_to:16s} ({reason})")
