# gimbal_target_tracker 배선 연결 가이드

이 문서는 `gimbal_target_tracker` 펌웨어를 기준으로 각 모듈을
OpenRB-150의 어느 핀에 연결하는지 정리한다.

## 1. 전체 연결표

| 장치 | 장치 신호 | OpenRB-150 연결 | 펌웨어 객체/설정 |
|---|---|---|---|
| BNO085 | SDA | D11 / SDA | `Wire`, 주소 `0x4A` |
| BNO085 | SCL | D12 / SCL | `Wire` |
| BNO085 | 전원 | 3.3V | `BNO085_ADDRESS` |
| BNO085 | GND | GND | 공통 접지 |
| BMP581 | SDA | D11 / SDA | `Wire`, 주소 `0x47` |
| BMP581 | SCL | D12 / SCL | `Wire` |
| BMP581 | 전원 | 3.3V | `BMP581_ADDRESS` |
| BMP581 | GND | GND | 공통 접지 |
| MAX-M10S | SDA | D11 / SDA | `Wire`, 주소 `0x42` |
| MAX-M10S | SCL | D12 / SCL | `Wire` |
| MAX-M10S | 전원 | 사용 중인 GPS 보드 사양 확인 | `MAX_M10S_ADDRESS` |
| MAX-M10S | GND | GND | 공통 접지 |
| E22-900T22S | TXD | D13 / `Serial3 RX` | 수신 데이터 |
| E22-900T22S | RXD | D14 / `Serial3 TX` | 설정·송신을 쓸 때 필요 |
| E22-900T22S | NC1 | 미연결 | 연결하지 않음 |
| E22-900T22S | NC2 | 미연결 | 연결하지 않음 |
| E22-900T22S | NC3 | 미연결 | 연결하지 않음 |
| E22-900T22S | AUX | 미연결 가능 | 연결 시 임의 GPIO와 설정값 변경 |
| E22-900T22S | VCC | 안정된 3.3V 전원 권장 | 전원 주의사항 참고 |
| E22-900T22S | GND | GND | 공통 접지 |
| SD 모듈 | MOSI/DI | D8 / MOSI | 하드웨어 `SPI` |
| SD 모듈 | SCK/CLK | D9 / SCK | 하드웨어 `SPI` |
| SD 모듈 | MISO/DO | D10 / MISO | 하드웨어 `SPI` |
| SD 모듈 | CS/SS | D4 | `SD_CS_PIN = 4` |
| SD 모듈 | VCC | 모듈 사양에 따라 3.3V 또는 5V | 아래 주의사항 확인 |
| SD 모듈 | GND | GND | 공통 접지 |
| Yaw DYNAMIXEL | TTL 3핀 | OpenRB DYNAMIXEL 포트 | ID 1 |
| Pitch DYNAMIXEL | TTL 3핀 | OpenRB DYNAMIXEL 포트 | ID 2 |
| PC | USB-C | OpenRB USB-C | `Serial`, 115200 bps |

OpenRB-150 보드 정의상 하드웨어 SPI는 `MOSI=D8`, `SCK=D9`,
`MISO=D10`, 기본 SS는 `D4`다. 현재
`src/config/system_config.h`의 `SD_CS_PIN`도 D4로 설정되어 있다.

## 2. 전체 배선 개념도

```text
                           OpenRB-150
                    ┌────────────────────┐
 BNO085 SDA ────────┤ D11 / SDA          │
 BMP581 SDA ────────┤                    │
 MAX-M10S SDA ──────┤                    │
                    │                    │
 BNO085 SCL ────────┤ D12 / SCL          │
 BMP581 SCL ────────┤                    │
 MAX-M10S SCL ──────┤                    │
                    │                    │
 E22 TXD ───────────┤ D13 / Serial3 RX   │
 E22 RXD ───────────┤ D14 / Serial3 TX   │
                    │                    │
 SD MOSI ───────────┤ D8  / MOSI         │
 SD SCK  ───────────┤ D9  / SCK          │
 SD MISO ───────────┤ D10 / MISO         │
 SD CS   ───────────┤ D4  / GPIO CS      │
                    │                    │
 Yaw DXL ID 1 ──────┤ DYNAMIXEL TTL PORT │
 Pitch DXL ID 2 ────┤ DYNAMIXEL TTL PORT │
                    │                    │
 모든 모듈 GND ─────┤ GND                │
                    └────────────────────┘
```

## 3. I2C 센서 연결

BNO085, BMP581, MAX-M10S는 D11/D12 한 쌍을 병렬로 공유한다.

```text
D11(SDA) ──┬── BNO085 SDA
           ├── BMP581 SDA
           └── MAX-M10S SDA

D12(SCL) ──┬── BNO085 SCL
           ├── BMP581 SCL
           └── MAX-M10S SCL

GND ───────┬── BNO085 GND
           ├── BMP581 GND
           └── MAX-M10S GND
```

현재 펌웨어 주소는 다음과 같다.

| 센서 | 기본 설정 | 가능한 다른 주소 |
|---|---:|---:|
| BNO085 | `0x4A` | `0x4B` |
| BMP581 | `0x47` | `0x46` |
| MAX-M10S | `0x42` | 설정으로 변경 가능 |

부팅 로그에 센서가 `FAIL`로 나오면 먼저 실제 I2C 주소를 확인하고
`src/config/system_config.h`의 주소를 변경한다.

I2C 버스는 400 kHz로 동작한다. 각 breakout에 pull-up 저항이 이미 달려 있는
경우가 많으므로, 여러 보드의 강한 pull-up이 병렬로 겹치지 않는지도 확인한다.
모든 I2C 신호 전압은 3.3V여야 한다.

## 4. EBYTE E22-900T22S 연결

수신에 꼭 필요한 연결은 다음과 같다.

```text
E22 TXD  ─────────→ OpenRB D13 (Serial3 RX)
E22 GND  ────────── OpenRB GND
E22 VCC  ────────── 안정된 전원

E22 NC1  ────────── 미연결
E22 NC2  ────────── 미연결
E22 NC3  ────────── 미연결
```

E22 설정 명령이나 양방향 송신도 사용한다면 다음을 추가한다.

```text
E22 RXD  ←───────── OpenRB D14 (Serial3 TX)
```

현재 사용 중인 모듈에는 M0/M1 대신 `NC1~NC3`가 표시되어 있으므로 이 핀은
어디에도 연결하지 않는다. 펌웨어도 M0/M1 GPIO를 사용하지 않으며, E22가
미리 설정된 UART 투명전송 상태로 동작한다고 가정한다.

양쪽 E22에서 다음 값이 같아야 통신된다.

- UART baud: 현재 코드 `9600`
- 무선 channel
- NETID
- air data rate
- 암호화 키를 사용한다면 동일한 키

AUX를 연결하지 않아도 수신은 가능하다. AUX를 사용하려면 남는 GPIO에
연결하고 `E22_AUX_PIN`을 해당 번호로 변경한다. `NC1~NC3`는 AUX나 GPIO
대용으로 사용하지 말고 계속 미연결 상태로 둔다.

## 5. SD 모듈 연결

```text
SD MOSI / DI  ←──── OpenRB D8
SD SCK / CLK  ←──── OpenRB D9
SD MISO / DO  ────→ OpenRB D10
SD CS / SS    ←──── OpenRB D4
SD GND        ───── OpenRB GND
```

현재 코드는 `SD.begin(4)`에 해당하는 설정을 사용한다. CS를 다른 GPIO로
옮겼다면 `src/config/system_config.h`의 다음 값을 함께 변경해야 한다.

```cpp
constexpr uint8_t SD_CS_PIN = 4;
```

SD 모듈의 전원 입력은 보드 종류에 따라 다르다.

- bare microSD 소켓 또는 3.3V 전용 보드: 3.3V만 사용
- 레귤레이터와 level shifter가 포함된 `5V 지원` 모듈: 제조사 표기 확인

전원 핀에 적힌 사양을 확인하지 않고 5V를 연결하면 안 된다. OpenRB의 GPIO는
3.3V 로직이며 5V 신호를 직접 입력하면 손상될 수 있다.

## 6. DYNAMIXEL 연결

Yaw와 Pitch 모터는 일반 GPIO가 아니라 OpenRB의 3핀 DYNAMIXEL TTL 포트에
연결한다.

```text
OpenRB DYNAMIXEL PORT
1: GND
2: VDD
3: DATA
```

모터 ID는 다음과 같이 설정되어 있어야 한다.

| 축 | ID | 중심 위치 |
|---|---:|---:|
| Yaw | 1 | 180° |
| Pitch | 2 | 180° |

두 모터는 서로 다른 DYNAMIXEL 포트에 꽂거나 TTL 데이지체인으로 연결할 수
있다. 펌웨어 baud는 57600 bps다.

DYNAMIXEL 모델의 정격전압을 반드시 확인한다. 특히 XL330 계열은 5V급이므로
고전압 배터리를 직접 연결하면 안 된다. 모터 부하가 있는 상태에서는 USB
전원만 사용하지 말고, 모터 전압에 맞는 충분한 용량의 별도 전원을 사용한다.
OpenRB와 모터 전원의 GND는 공통이어야 한다.

## 7. 전원 구성 권장

센서와 통신 모듈을 모두 OpenRB의 3.3V 핀 하나에서 공급하기 전에 총 전류와
순간 피크를 계산해야 한다. E22 송신 순간과 SD 카드 쓰기 순간에는 전류가
증가하므로 전압 강하가 발생할 수 있다.

권장 구성은 다음과 같다.

```text
배터리
 ├─ OpenRB 입력
 ├─ DYNAMIXEL 정격전압용 전원
 └─ 센서/E22/SD용 안정된 3.3V 레귤레이터

각 전원의 GND는 공통 연결
```

외부 3.3V 레귤레이터를 사용할 경우 OpenRB의 3.3V 출력과 외부 레귤레이터
출력을 서로 직접 묶지 않는다. 외부 레귤레이터의 3.3V 출력은 모듈 VCC에만
공급하고 GND만 OpenRB와 공유한다.

## 8. 최초 전원 인가 전 체크리스트

- 전원을 끈 상태에서 모든 배선을 연결했다.
- 모든 모듈의 GND가 공통이다.
- I2C SDA와 SCL이 뒤바뀌지 않았다.
- E22 TXD가 OpenRB RX인 D13으로 연결됐다.
- SD MOSI/MISO 방향이 뒤바뀌지 않았다.
- SD CS 배선과 `SD_CS_PIN` 값이 같다.
- BNO085와 BMP581의 실제 I2C 주소를 확인했다.
- DYNAMIXEL ID가 Yaw 1, Pitch 2로 서로 다르다.
- DYNAMIXEL 전원전압이 모터 정격과 일치한다.
- E22에 안테나를 연결한 뒤 전원을 인가한다.
- OpenRB GPIO에 5V 신호가 들어오지 않는다.

전원을 넣은 후 USB 시리얼 모니터를 115200 bps로 열면 다음 장치의 초기화
결과를 각각 확인할 수 있다.

```text
# BNO085: OK
# BMP581: OK
# MAX-M10S: OK
# E22: transparent receiver ready
# DYNAMIXEL: OK
# SD: OK file=TRK000.CSV
```

하나씩 연결해 초기화 상태를 확인한 뒤 전체 모듈을 조립하는 방법이 가장
안전하다.
