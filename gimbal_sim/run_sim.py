# -*- coding: utf-8 -*-
"""
run_sim.py — 2단 낙하산 전개 시나리오 + 예측 옵션 비교 (예측 업그레이드 2026-07-21)

실행: python3 run_sim.py
출력: sim_result.png (3개 그래프), 콘솔 요약

■ 이 시뮬이 재는 것: "예측 조준점 정확도"
  짐벌이 실제로 겨누는 순간은 명령 시점보다 서보지연(SERVO_LAG_S)만큼 뒤다.
  따라서 명령은 '지금'이 아니라 '겨눌 시점 = t + 서보지연'의 로켓 위치를 겨눠야 한다.
  → 지표 = (예측 조준 LOS) vs (t+서보지연에서의 실제 로켓 LOS) 사잇각.
  자세/스핀·rate limit은 배제해 '로켓 예측' 효과만 격리 (스핀 보상은 HOLD 테스터에서 별도 검증).

■ 시나리오 (드로그→메인 2단 전개):
  드로그로 -30 m/s 하강(+동풍 15 m/s 표류) → 고도 450m에서 메인 전개
  → 1.5s 인플레이션 동안 -30→-8 m/s 급감속(가속 임펄스) → 메인 -8 m/s.
  통신 10Hz·120ms 지연. 서보지연 150ms.

■ 비교 3옵션 (모션 모델은 셋 다 등속 P+V·Δt):
  (1) 리드 없음  : 통신지연만 보정 (현재 알고리즘) → 서보지연만큼 항상 뒤처짐
  (2) 리드만     : + 서보 리드타임 → 순항 정확↑, 그러나 전개 순간 옛 속도로 과-외삽(오버슈트)
  (3) 리드+가드  : + 메인 전개 가드(고도 게이트 + 반응형 ΔV) → 순항 정확 + 전개 오버슈트 회피 ← 제안
"""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

for f in fm.fontManager.ttflist:                 # 한글 폰트(있으면)
    if 'CJK' in f.name or 'Nanum' in f.name or 'Gothic' in f.name:
        plt.rcParams['font.family'] = f.name
        break
plt.rcParams['axes.unicode_minus'] = False

import config as cfg
from config import deg_to_i7
from geometry import predict_position, geo_to_ned

# ═══════════════════════════════════════════════════════
# 시나리오 정의
# ═══════════════════════════════════════════════════════
LAT0, LON0 = 37.5000000, 127.0000000
SIM_T   = 30.0
DT      = cfg.CONTROL_DT          # 50Hz
PACKET_HZ  = 10
COMM_DELAY = 0.12                 # 통신 지연 120ms
SERVO_LAG_S = 0.15               # ★ 물리 서보 수송지연 — 겨눌 시점 = t + 이 값 (SERVO_LEAD_S가 이걸 겨냥)

ROCKET_START_ALT = 900.0
DROGUE_VD = 30.0
MAIN_VD   = 8.0
T_DEP  = (ROCKET_START_ALT - cfg.MAIN_DEPLOY_ALT_AGL) / DROGUE_VD   # 전개 시각 = 15.0s
T_INF  = 0.6                      # 메인 인플레이션(감속) 시간 [s] — 현실적 중간값
ROCKET_VE = 15.0                  # 동풍 표류 — 접선 운동(→각속도) 생성

_ALT_DEP = ROCKET_START_ALT - DROGUE_VD * T_DEP
_ALT_INF_END = _ALT_DEP - (DROGUE_VD * T_INF + (MAIN_VD - DROGUE_VD) * T_INF / 2.0)


def set_inflation(tinf):
    """인플레이션 시간 스윕용 — T_INF와 종속 상수 갱신."""
    global T_INF, _ALT_INF_END
    T_INF = tinf
    _ALT_INF_END = _ALT_DEP - (DROGUE_VD * T_INF + (MAIN_VD - DROGUE_VD) * T_INF / 2.0)


def rocket_true(t):
    """로켓 실제 상태 (2단 전개). 위치는 속도의 적분과 일치."""
    if t < 0:
        t = 0.0
    east = ROCKET_VE * t
    if t <= T_DEP:                                   # 드로그 등속
        alt = ROCKET_START_ALT - DROGUE_VD * t
        vD = DROGUE_VD
    elif t <= T_DEP + T_INF:                         # 인플레이션 감속 램프
        tau = t - T_DEP
        vD = DROGUE_VD + (MAIN_VD - DROGUE_VD) * (tau / T_INF)
        alt = _ALT_DEP - (DROGUE_VD * tau + (MAIN_VD - DROGUE_VD) * tau * tau / (2.0 * T_INF))
    else:                                            # 메인 등속
        tau = t - T_DEP - T_INF
        alt = _ALT_INF_END - MAIN_VD * tau
        vD = MAIN_VD
    return dict(north=0.0, east=east, alt=alt, vN=0.0, vE=ROCKET_VE, vD=vD)


def payload_true(t):
    """페이로드 실제 상태: -12 m/s 하강, 수평 고정 (스핀=0, 예측효과 격리)."""
    return dict(north=-20.0, east=0.0, alt=ROCKET_START_ALT - 12.0 * t)


def ne_to_geo(north, east):
    lat = LAT0 + north / 111_320.0
    lon = LON0 + east / (111_320.0 * math.cos(math.radians(LAT0)))
    return deg_to_i7(lat), deg_to_i7(lon)


def angle_between(a, b):
    """두 NED 벡터 사잇각 [deg]."""
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    if na == 0 or nb == 0:
        return 0.0
    dot = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


# ═══════════════════════════════════════════════════════
# 예측 지평·전개 가드 (C++ coarse_track.h와 동일한 순수함수)
# ═══════════════════════════════════════════════════════
def predict_horizon(comm_delay, servo_lead, guard):
    lead = 0.0 if guard else servo_lead
    h = comm_delay + lead
    return max(cfg.PREDICT_HORIZON_MIN_S, min(cfg.PREDICT_HORIZON_MAX_S, h))


def in_deploy_band(alt_agl):
    return (cfg.MAIN_DEPLOY_ALT_AGL - cfg.DEPLOY_GUARD_MARGIN_DN_M
            <= alt_agl <=
            cfg.MAIN_DEPLOY_ALT_AGL + cfg.DEPLOY_GUARD_MARGIN_UP_M)


def dist3(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


# ═══════════════════════════════════════════════════════
# 시뮬레이션 실행 — 예측 조준점 오차 측정
# ═══════════════════════════════════════════════════════
def run(servo_lead, use_guard):
    last_packet = None
    next_packet_t = 0.0
    prev_pkt_vel = None
    reactive_until = -1.0

    log = dict(t=[], err=[], horizon=[], guard=[])
    n_steps = int(SIM_T / DT)
    for i in range(n_steps):
        t = i * DT

        # ── 로켓 패킷 수신 (10Hz, 지연) ──
        if t >= next_packet_t:
            t_meas = t - COMM_DELAY
            if t_meas >= 0:
                r = rocket_true(t_meas)
                lat_i7, lon_i7 = ne_to_geo(r['north'], r['east'])
                V = (r['vN'], r['vE'], r['vD'])
                if prev_pkt_vel is not None and dist3(V, prev_pkt_vel) >= cfg.DEPLOY_DV_THRESH_MPS:
                    reactive_until = t + cfg.DEPLOY_GUARD_HOLD_MS / 1000.0
                prev_pkt_vel = V
                last_packet = dict(lat_i7=lat_i7, lon_i7=lon_i7, alt_m=r['alt'],
                                   vN=r['vN'], vE=r['vE'], vD=r['vD'], meas_t=t_meas)
            next_packet_t += 1.0 / PACKET_HZ

        guard = False
        horizon = 0.0
        err = 0.0
        if last_packet is not None:
            comm_delay = t - last_packet['meas_t']
            alt_gate = in_deploy_band(last_packet['alt_m'])
            reactive = t < reactive_until
            guard = use_guard and (alt_gate or reactive)
            horizon = predict_horizon(comm_delay, servo_lead, guard)

            # 예측 조준점: 패킷 위치를 horizon만큼 등속 외삽
            plat, plon, palt = predict_position(
                last_packet['lat_i7'], last_packet['lon_i7'], last_packet['alt_m'],
                last_packet['vN'], last_packet['vE'], last_packet['vD'], horizon)
            p = payload_true(t)
            p_lat, p_lon = ne_to_geo(p['north'], p['east'])
            dNED_pred = geo_to_ned(plat, plon, palt, p_lat, p_lon, p['alt'])

            # 원하는 조준점: 짐벌이 실제로 겨눌 시점(t+서보지연)의 실제 로켓 LOS
            r_des = rocket_true(t + SERVO_LAG_S)
            dNED_des = (r_des['north'] - p['north'],
                        r_des['east'] - p['east'],
                        p['alt'] - r_des['alt'])
            err = angle_between(dNED_pred, dNED_des)

        log['t'].append(t)
        log['err'].append(err)
        log['horizon'].append(horizon)
        log['guard'].append(1 if guard else 0)
    return log


print("시뮬레이션 실행 중 (3옵션)...")
log_none = run(0.0, False)                     # 리드 없음 (현재 알고리즘)
log_lead = run(cfg.SERVO_LEAD_S, False)        # 리드만
log_guard = run(cfg.SERVO_LEAD_S, True)        # 리드 + 가드 (제안)

# ═══════════════════════════════════════════════════════
# 결과 그래프
# ═══════════════════════════════════════════════════════
band_lo = cfg.MAIN_DEPLOY_ALT_AGL - cfg.DEPLOY_GUARD_MARGIN_DN_M
band_hi = cfg.MAIN_DEPLOY_ALT_AGL + cfg.DEPLOY_GUARD_MARGIN_UP_M
t_band = [t for t in log_none['t'] if band_lo <= rocket_true(t)['alt'] <= band_hi]
tb0, tb1 = (min(t_band), max(t_band)) if t_band else (T_DEP, T_DEP)

fig, axes = plt.subplots(3, 1, figsize=(12, 11))

ax = axes[0]
ax.plot(log_none['t'], log_none['err'], color='#d62728', lw=1.2, label='(1) 리드 없음 (현재)')
ax.plot(log_lead['t'], log_lead['err'], color='#ff7f0e', lw=1.2, label='(2) 리드만 (가드 없음)')
ax.plot(log_guard['t'], log_guard['err'], color='#1f77b4', lw=1.6, label='(3) 리드+가드 (제안)')
ax.axvspan(tb0, tb1, color='orange', alpha=0.10)
ax.axvline(T_DEP, color='k', ls='--', lw=0.8, alpha=0.6)
ax.set_ylabel('예측 조준 오차 [deg]')
ax.set_title('예측 조준점 오차 (주황 음영 = 전개 가드 밴드 / 세로점선 = 메인 전개)')
ax.legend(loc='upper left', fontsize=9)
ax.grid(alpha=0.3)

ax = axes[1]
alts = [rocket_true(t)['alt'] for t in log_none['t']]
ax.plot(log_none['t'], alts, color='purple', lw=1.5, label='로켓 고도 (AGL)')
ax.axhspan(band_lo, band_hi, color='orange', alpha=0.12, label=f'전개 가드 밴드 [{band_lo:.0f},{band_hi:.0f}]m')
ax.axhline(cfg.MAIN_DEPLOY_ALT_AGL, color='k', ls='--', lw=0.8, label=f'메인 사출 {cfg.MAIN_DEPLOY_ALT_AGL:.0f}m')
ax.axvspan(tb0, tb1, color='orange', alpha=0.10)
ax.set_ylabel('고도 [m AGL]')
ax.set_title('로켓 고도 — 고도 게이트가 전개 밴드에서 가드 arm')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(log_none['t'], log_none['horizon'], color='#d62728', lw=1.0, label='(1) 리드 없음')
ax.plot(log_lead['t'], log_lead['horizon'], color='#ff7f0e', lw=1.0, label='(2) 리드만')
ax.plot(log_guard['t'], log_guard['horizon'], color='#1f77b4', lw=1.6, label='(3) 리드+가드')
ax.axvspan(tb0, tb1, color='orange', alpha=0.10)
ax.set_ylabel('외삽 지평 Δt [s]')
ax.set_xlabel('시간 [s]')
ax.set_title('예측 지평 — 가드(3)는 전개 밴드에서 리드를 죽여 과-외삽 방지')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('sim_result.png', dpi=110)
print("그래프 저장: sim_result.png")

# ═══════════════════════════════════════════════════════
# 콘솔 요약
# ═══════════════════════════════════════════════════════
import statistics


def window_rms(log, t0, t1):
    e = [log['err'][i] for i, t in enumerate(log['t']) if t0 <= t <= t1]
    return statistics.mean(x * x for x in e) ** 0.5 if e else float('nan')


def window_peak(log, t0, t1):
    e = [log['err'][i] for i, t in enumerate(log['t']) if t0 <= t <= t1]
    return max(e) if e else float('nan')


CRUISE = (3.0, tb0 - 1.0)              # 전개 전 순항
DEPLOY = (T_DEP - 0.3, T_DEP + 2.5)    # 전개 순간 창

print()
print("━━━ 순항 구간 RMS 예측오차 (서보지연 보상 효과) ━━━")
for name, lg in [("리드 없음", log_none), ("리드만", log_lead), ("리드+가드", log_guard)]:
    print(f"  {name:10s}: {window_rms(lg, *CRUISE):.3f} deg")
print()
print("━━━ 전개 순간 피크 예측오차 (과-외삽 회피 효과) ━━━")
for name, lg in [("리드 없음", log_none), ("리드만", log_lead), ("리드+가드", log_guard)]:
    print(f"  {name:10s}: {window_peak(lg, *DEPLOY):.3f} deg")
print()
print(f"  (전개 시각 T_DEP={T_DEP:.1f}s, 가드 밴드 시간창 [{tb0:.1f},{tb1:.1f}]s)")

# ── 인플레이션 시간 감도 스윕: 가드가 언제 이득인가 ──
print()
print("━━━ 인플레이션(전개 급격도)별 전개 피크: 리드만 vs 리드+가드 ━━━")
print("   (가드는 급전개일수록 이득, 완만하면 리드 유지가 나음)")
for tinf in [0.3, 0.6, 1.0, 1.5]:
    set_inflation(tinf)
    lo = run(cfg.SERVO_LEAD_S, False)
    gd = run(cfg.SERVO_LEAD_S, True)
    pk_lo = window_peak(lo, *DEPLOY)
    pk_gd = window_peak(gd, *DEPLOY)
    verdict = "가드 이득" if pk_gd < pk_lo - 1e-3 else ("동일" if abs(pk_gd - pk_lo) <= 1e-3 else "리드만 나음")
    print(f"  인플레이션 {tinf:.1f}s: 리드만 {pk_lo:.3f}° / 리드+가드 {pk_gd:.3f}°  → {verdict}")
set_inflation(0.6)  # 원복
