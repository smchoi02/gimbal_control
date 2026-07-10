# -*- coding: utf-8 -*-
"""
fsm.py — 상태머신 (로드맵 §7 그대로)

C++ 이식 대상: state_machine.h — OpenRB loop()에서 매 사이클 update() 호출.

상태: INIT → STOW → (MANUAL | STABILIZED_HOLD | COARSE_TRACK → VISION_TRACK)
      통신 상실 → HOLD 성격은 STABILIZED_HOLD로 흡수, 지속 시 SCAN
      fault 발생 → FAULT
"""
import config as cfg


class State:
    INIT            = "INIT"
    STOW            = "STOW"
    MANUAL          = "MANUAL"
    STABILIZED_HOLD = "STABILIZED_HOLD"
    COARSE_TRACK    = "COARSE_TRACK"
    VISION_TRACK    = "VISION_TRACK"
    SCAN            = "SCAN"
    FAULT           = "FAULT"


class GimbalFSM:
    """
    입력(센서 상태·명령·시각)을 받아 상태를 갱신.
    모든 전이는 transitions 리스트에 기록 → 사후 분석용 (LOG_RECORD 대응).
    """

    def __init__(self):
        self.state = State.INIT
        self.t_last_packet = None    # 마지막 유효 로켓 패킷 시각
        self.t_hold_start = None     # 통신 상실 시작 시각
        self.transitions = []        # (t, from, to, reason)

    def _go(self, t, new_state, reason):
        if new_state != self.state:
            self.transitions.append((t, self.state, new_state, reason))
            self.state = new_state

    def update(self, t, *,
               sensors_ok=True,
               armed=False,
               manual_cmd=False,
               rocket_packet=False,
               vision_valid=False,
               fault=False):
        """
        매 사이클 호출. t = 현재 시각 [s]

        sensors_ok    : DXL ping + BNO085 정상
        armed         : enable 명령 수신됨
        manual_cmd    : MANUAL 명령 활성
        rocket_packet : 이번 사이클에 유효 로켓 패킷 수신
        vision_valid  : 영상 target 유효 (confidence 충분)
        fault         : DXL error / 과열 / 저전압 / heartbeat 상실
        """
        # ── FAULT는 최우선 ──
        if fault:
            self._go(t, State.FAULT, "fault flag")
            return self.state

        if self.state == State.FAULT:
            return self.state    # operator clear 전까지 유지 (reset()으로만 탈출)

        # ── 패킷 시각 기록 ──
        if rocket_packet:
            self.t_last_packet = t
            self.t_hold_start = None

        comm_age = (t - self.t_last_packet) if self.t_last_packet is not None else None
        comm_ok = comm_age is not None and comm_age <= cfg.COMM_TIMEOUT_S

        # ── 상태별 전이 ──
        if self.state == State.INIT:
            if sensors_ok:
                self._go(t, State.STOW, "sensors ok")

        elif self.state == State.STOW:
            if manual_cmd:
                self._go(t, State.MANUAL, "manual cmd")
            elif armed and comm_ok:
                self._go(t, State.COARSE_TRACK, "armed + comm")
            elif armed:
                self._go(t, State.STABILIZED_HOLD, "armed, no comm")

        elif self.state == State.MANUAL:
            if not manual_cmd:
                self._go(t, State.STOW, "manual end")

        elif self.state == State.STABILIZED_HOLD:
            if comm_ok:
                self._go(t, State.COARSE_TRACK, "comm recovered")
            else:
                if self.t_hold_start is None:
                    self.t_hold_start = t
                elif t - self.t_hold_start >= cfg.SCAN_AFTER_S:
                    self._go(t, State.SCAN, "hold timeout")

        elif self.state == State.COARSE_TRACK:
            if vision_valid:
                self._go(t, State.VISION_TRACK, "vision valid")
            elif not comm_ok:
                self.t_hold_start = t
                self._go(t, State.STABILIZED_HOLD, "comm lost")

        elif self.state == State.VISION_TRACK:
            if not vision_valid:
                if comm_ok:
                    self._go(t, State.COARSE_TRACK, "vision lost, comm ok")
                else:
                    self.t_hold_start = t
                    self._go(t, State.STABILIZED_HOLD, "vision+comm lost")

        elif self.state == State.SCAN:
            if comm_ok:
                self._go(t, State.COARSE_TRACK, "comm recovered")
            elif vision_valid:
                self._go(t, State.VISION_TRACK, "vision reacquired")

        return self.state

    def reset(self, t):
        """operator clear / reboot."""
        self._go(t, State.INIT, "reset")
        self.t_last_packet = None
        self.t_hold_start = None
