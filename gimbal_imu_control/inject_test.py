# -*- coding: utf-8 -*-
"""
inject_test.py — PC에서 OpenRB로 가상 IMU 데이터 주입 + 텔레메트리 로그 저장

부품 도착 후 사용 (BNO085 없이 OpenRB + DYNAMIXEL만으로 전체 파이프라인 검증):

  pip install pyserial
  python3 inject_test.py COM5            # Windows (장치관리자에서 포트 확인)
  python3 inject_test.py /dev/ttyACM0    # Linux/Mac

동작:
  1) INJECT 모드 전환 → HOLD 모드 진입 (북쪽 방향 캡처)
  2) ±40° 요동 쿼터니언을 20Hz로 60초간 주입 (run_sim.py와 동일 시나리오)
  3) 보드가 보내는 CSV 텔레메트리를 log_inject.csv로 저장
  4) 기대 결과: 짐벌 yaw가 주입 요동의 '반대'로 움직임 (상쇄)
     → log를 파이썬 시뮬레이터 결과와 비교하면 하드웨어 파이프라인 검증 완료
"""
import sys, time, math

try:
    import serial
except ImportError:
    sys.exit("pyserial 필요: pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
DURATION = 60.0

def euler_to_quat(yaw_deg):
    h = math.radians(yaw_deg) * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(2.5)                      # 보드 리셋 대기
ser.reset_input_buffer()

log = open("log_inject.csv", "w")

def send(line):
    ser.write((line + "\n").encode())

print(f"[{PORT}] INJECT 모드 시작, {DURATION:.0f}초간 요동 주입...")
send("I J")                          # INJECT 모드
time.sleep(0.2)
send("Q 1 0 0 0")                    # 초기 자세 = 북향
time.sleep(0.2)
send("H")                            # 현재 방향 캡처 → HOLD

t0 = time.time()
next_q = 0.0
while (t := time.time() - t0) < DURATION:
    if t >= next_q:                  # 20Hz 주입
        yaw = 40.0 * math.sin(2 * math.pi * t / 8.0)
        w, x, y, z = euler_to_quat(yaw)
        send(f"Q {w:.5f} {x:.5f} {y:.5f} {z:.5f}")
        next_q += 0.05
        # HOLD heartbeat 유지용이기도 함
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line and not line.startswith("#"):
            log.write(line + "\n")
        elif line:
            print(" ", line)

send("Z")                            # STOW 복귀
log.close()
ser.close()
print("완료. log_inject.csv 저장됨.")
print("검증: pYaw 열(주입 요동)과 gYawCmd 열(짐벌 명령)이 부호 반대 & 크기 일치하면 성공.")
