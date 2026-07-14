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

// ── 안전 ──
#define MANUAL_HEARTBEAT_S (30.0f)
// [XL330] 온도 한계도 조정 (Shutdown 기본값 확인 필요)
#define DXL_TEMP_LIMIT_C   (65)