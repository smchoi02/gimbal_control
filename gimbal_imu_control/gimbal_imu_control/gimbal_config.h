#pragma once
// ═══════════════════════════════════════════════════════════════
// gimbal_config.h v0.2 — XL330-M288-T + OpenRB-150 대응
//
// ★ XL430에서 변경된 부분은 [XL330] 태그로 표시
// ═══════════════════════════════════════════════════════════════

// ── 짐벌 물리 한계 (deg) ──
#define YAW_MIN_DEG    (-170.0f)
#define YAW_MAX_DEG    (+170.0f)
#define PITCH_MIN_DEG  (-10.0f)
#define PITCH_MAX_DEG  (+90.0f)

// ── 명령 제한 (Phase 2 safety) ──
// [XL330] XL430(1.5Nm)에서 XL330(0.52Nm)로 토크 35% 수준 → rate 완화
// 카메라+브래킷 관성 시험 후 조정. 초기값은 매우 보수적으로.
#define MAX_RATE_DEG_S (120.0f)     // XL430 시절 180 → 120으로 하향

// ── 루프 주기 ──
#define CONTROL_HZ     (50)
#define CONTROL_DT     (1.0f / CONTROL_HZ)
#define TELEMETRY_HZ   (10)

// ── 부호 규약 ──
#define YAW_SIGN       (-1.0f)
#define PITCH_SIGN     (-1.0f)

// ── DYNAMIXEL ──
// [XL330] Model number 1200 (XL430은 1060)
#define DXL_MODEL_XL330  (1200)
#define YAW_ID         (1)
#define PITCH_ID       (2)
#define DXL_BAUD       (57600)       // 초기값. 통합 후 상향
#define DXL_CENTER_DEG (180.0f)

// [XL330] Current unit이 XL430과 다름
// e-Manual 표에서 확정 후 반영 (Robotis Docs의 Goal Current 항목)
// stall current 1.47A에서 60% = 880mA 정도로 초기 제한
// ★ 부품 도착 후 Wizard로 실제 unit 값 확인하고 여기 상수 조정
#define DXL_GOAL_CURRENT   (700)     // 초기값 (안전한 낮은 값)
#define DXL_PROFILE_VEL    (60)      // 프로파일 속도 낮춤 (XL430은 100)

// ── 전원 (참고용 문서 — 하드웨어 담당자와 공유) ──
// ★★★★★ XL330은 5V 서보 (범위 3.7~6.0V)
// ★★★★★ 4S LiFePO4 (14.6V 만충)을 직접 연결하면 즉시 파손
// 배터리 → 5V Buck 컨버터 (5A급) → 서보 rail
// OpenRB-150 로직은 USB 또는 별도 5V

// ── 통신 규약 (Phase 4 — config.py와 동일) [GPS 통합 2026-07-19 추가] ──
#define COMM_TIMEOUT_S     (0.5f)    // 로켓 패킷이 이 시간 없으면 COARSE→HOLD 폴백
#define GPS_FIX_TIMEOUT_S  (1.5f)    // 자기 GPS fix 신선도 한계

// ── 고도 기준: 기압계 AGL [고도기준 2026-07-21] ──
// 수직 고도를 GPS hMSL 대신 기압계 AGL로 사용(GPS 수직오차 회피).
// 로켓·페이로드 둘 다 발사대에서 0점 → 공유 datum. baro.h 참고.
#define BARO_TIMEOUT_S     (0.5f)       // 페이로드 기압계 고도 신선도 한계 (초과 시 GPS hMSL 폴백)
#define BARO_P0_PA         (101325.0f)  // 미영점 시 기준기압 폴백 (표준 해면기압)

// ── 예측/추적 튜닝 [예측 업그레이드 2026-07-21] ──
// 등속 예측 P+V·Δt는 유지. 외삽 지평 Δt = 통신지연 + 서보 리드타임.
// 메인 낙하산 전개 순간엔 리드를 죽여 과-외삽 방지(고도 게이트 = feed-forward + 반응형 ΔV = 백업).
#define SERVO_LEAD_S          (0.15f)   // ★TUNE: 짐벌이 실제로 겨눌 때까지의 리드(서보 스텝응답 측정 후 조정)
#define PREDICT_HORIZON_MIN_S (-1.0f)   // 외삽 지평 하한 (역외삽 허용폭)
#define PREDICT_HORIZON_MAX_S (2.0f)    // 외삽 지평 상한 (발산 방지)

// 메인 낙하산 전개 가드
// ※ sim(run_sim.py, 2026-07-21): 가드는 메인 인플레이션이 급격(<~0.7s)할 때만 이득.
//    완만한 전개(≳1s)면 리드 유지가 더 나음. 실제 메인 인플레이션 시간으로 ENABLE 결정.
#define DEPLOY_GUARD_ENABLE      (0)       // [2025 IREC 실측] 메인전개 전환 2~4s(완만) → 가드 손해·반응형 ΔV 미발화 → OFF(리드 유지)
#define MAIN_DEPLOY_ALT_AGL      (450.0f)  // ★ 메인 사출 고도 [m AGL] (미션값 — 로켓 고도계 설정과 일치시킬 것)
#define DEPLOY_GUARD_MARGIN_UP_M (40.0f)   // 사출 고도 위 이만큼부터 가드 arm (고도계 오차 마진)
#define DEPLOY_GUARD_MARGIN_DN_M (120.0f)  // 사출 고도 아래 이만큼까지 유지 (낙하산 인플레이션·정착)
#define DEPLOY_DV_THRESH_MPS     (15.0f)   // 반응형 백업: |ΔV| 이 이상이면 전개로 간주
#define DEPLOY_GUARD_HOLD_MS     (1500u)   // 반응형 가드 유지 시간 [ms]

// ── 안전 ──
#define MANUAL_HEARTBEAT_S (30.0f)
// [XL330] 온도 한계도 조정 (Shutdown 기본값 확인 필요)
#define DXL_TEMP_LIMIT_C   (65)