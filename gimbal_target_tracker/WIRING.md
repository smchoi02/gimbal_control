# gimbal_target_tracker 배선 가이드

아래 배선은 `gimbal_target_tracker` 펌웨어와 OpenRB-150을 기준으로 한다. 모든 모듈은 GND를 공통으로 연결하고, I2C 신호는 3.3 V 로직으로 사용한다.

## 필수 연결

| 장치 | 신호 | OpenRB-150 | 비고 |
|---|---|---|---|
| BNO085 | SDA / SCL | D11 / D12 | `Wire`, 기본 주소 `0x4A` |
| BNO085 | VCC / GND | 3.3V / GND | BNO085 보드 사양 준수 |
| BMP581 | SDA / SCL | D11 / D12 | 기본 주소 `0x47` |
| MAX-M10S | SDA / SCL | D11 / D12 | 기본 주소 `0x42` |
| E22-900T22S | TXD | OpenRB `RX` / `Serial3 RX` | 수신 데이터 |
| E22-900T22S | RXD | OpenRB `TX` / `Serial3 TX` | 설정 또는 양방향 통신 시 연결 |
| E22-900T22S | M0 | GND 또는 설정 GPIO | 정상 투명 모드에서는 LOW |
| E22-900T22S | M1 | GND 또는 설정 GPIO | 정상 투명 모드에서는 LOW |
| E22-900T22S | AUX | 선택 GPIO 또는 미연결 | LOW=기동/처리 중, HIGH=준비 |
| E22-900T22S | NRST | 풀업 또는 MCU GPIO | 사용하지 않으면 보드 권장 회로대로 유지; LOW는 리셋 |
| E22-900T22S | VCC / GND | 안정된 전원 / GND | VCC 허용 범위와 I/O 로직 전압은 사용 보드 사양 확인 |
| E22-900T22S | ANT | 지정 안테나 | 안테나 연결 후 전원 인가 |
| SD | MOSI / SCK / MISO / CS | D8 / D9 / D10 / D4 | `SD_CS_PIN = 4` |
| DYNAMIXEL yaw/pitch | TTL 3선 | OpenRB DYNAMIXEL 포트 | ID 1 / 2 |

E22-900T22S의 M0/M1은 약한 pull-up이므로 미연결 상태로 두지 않는다. `system_config.h`에서 두 GPIO가 `-1`인 기본 구성은 두 핀을 GND에 물리적으로 연결하는 구성이다. MCU GPIO를 쓸 때는 `E22_M0_PIN`, `E22_M1_PIN`에 실제 핀 번호를 넣는다.

```text
E22-900T22S TXD  ─────────→ OpenRB RX (Serial3 RX)
E22-900T22S RXD  ←───────── OpenRB TX (Serial3 TX, 선택)
E22-900T22S M0   ────────── GND 또는 OpenRB GPIO (LOW)
E22-900T22S M1   ────────── GND 또는 OpenRB GPIO (LOW)
E22-900T22S AUX  ─────────→ OpenRB GPIO (선택)
E22-900T22S GND  ────────── OpenRB GND
E22-900T22S VCC  ────────── 안정된 전원
```

## E22-900T22S 운용 조건

- 일반 투명 전송/수신: `M0=0`, `M1=0`
- 설정 모드: `M0=0`, `M1=1`
- M0/M1 변경 뒤 모듈이 준비될 때까지 기다린다. AUX를 연결했다면 HIGH를 확인한다.
- 수신만 할 때도 M0/M1은 반드시 LOW로 고정한다.
- 송신기와 수신기의 UART baud, 900 MHz 대역 채널, NETID, air data rate, 암호화 설정을 동일하게 맞춘다.
- 출력 전류 여유가 있는 안정 전원을 사용한다. 무선 송신과 SD 쓰기 순간의 전압 강하를 확인한다.

펌웨어는 부팅 시 M0/M1을 LOW로 구동하고 5 ms 대기한다. AUX가 설정된 경우 LOW 동안 수신 패킷 해석을 보류하며, 상태 로그에 `e22_ready`를 출력한다.

## I2C 버스

```text
D11 (SDA) ──┬── BNO085 SDA
            ├── BMP581 SDA
            └── MAX-M10S SDA

D12 (SCL) ──┬── BNO085 SCL
            ├── BMP581 SCL
            └── MAX-M10S SCL
```

여러 breakout의 pull-up 저항이 병렬로 너무 강해지지 않는지 확인한다. BNO085가 부팅 로그에서 `FAIL`이면 먼저 `0x4A`/`0x4B` 주소와 전원을 확인한다.

## 전원 인가 전 확인

- E22 안테나가 연결되었다.
- E22 M0/M1은 GND 또는 명시적인 MCU 출력에 연결되었고 부동 상태가 아니다.
- E22 TXD는 OpenRB `RX`, 공통 GND는 OpenRB GND에 연결되었다.
- BNO085·BMP581·MAX-M10S I2C 주소가 설정과 맞는다.
- DYNAMIXEL 전원은 모터 정격에 맞고 OpenRB와 GND가 공통이다.
- SD CS 배선이 D4 및 `SD_CS_PIN` 설정과 맞는다.

정상 부팅 시 다음과 비슷한 로그를 확인한다.

```text
# BNO085: OK; Game Rotation Vector
# E22-900T22S: normal/transparent receiver ready
```
