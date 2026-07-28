# gimbal_target_tracker

OpenRB-150에서 다음 장치를 통합하는 모듈형 Arduino 스케치다.

- I2C: BNO085, BMP581, MAX-M10S
- UART: EBYTE E22-900T22S (`Serial3`) 수신
- OpenRB DYNAMIXEL TTL: yaw/pitch 모터 (`Serial1`)
- SPI: SD 카드

기존 `gimbal_imu_control`은 변경하지 않았으며 이 폴더가 별도 프로젝트다.
Arduino IDE에서는 `gimbal_target_tracker.ino`를 연다.

구체적인 핀별 연결은 [`WIRING.md`](WIRING.md)에서 확인할 수 있다.
송신 장치 없이 시험하는 방법은 [`SIMULATION.md`](SIMULATION.md)에서
확인할 수 있다.

## 코드 구조

```text
gimbal_target_tracker/
├─ gimbal_target_tracker.ino       최소 setup/loop
├─ src/
│  ├─ app/                         전체 스케줄·상태기계
│  ├─ actuators/                   DYNAMIXEL 구동·안전 제한
│  ├─ comm/                        E22 수신·바이너리 패킷
│  ├─ common/                      공용 타입·CRC·시간 함수
│  ├─ config/                      핀·주소·주기·안전값
│  ├─ math/                        NED/자세/상대위치 계산
│  ├─ sensors/                     BNO085/BMP581/MAX-M10S
│  └─ storage/                     SD CSV 로거
├─ examples/e22_sender_example/    송신측 패킷 생성 예제
└─ tests/test_core.cpp             PC에서 수학·패킷 검증
```

## 처리 순서

1. E22가 투명전송 UART로 송신측 위도·경도·기압 데이터를 수신한다.
2. MAX-M10S의 로컬 위도·경도와 수신 위도·경도의 정수 차이를 미터 N/E로
   바꾼다.
3. 로컬 BMP581과 원격 기압의 비로 상대 고도를 구하고 NED의 D로 바꾼다.
4. BNO085 Rotation Vector로 NED 목표벡터를 body 좌표로 회전한다.
5. body 벡터를 yaw/pitch로 바꾸고 기계각 clamp와 120 deg/s rate limit을
   거쳐 DYNAMIXEL에 보낸다.
6. 모든 로컬 센서, 원격 데이터, 상대 위치, 명령 및 모터 피드백을 SD에
   10 Hz CSV로 기록한다.

정상 추적 중 데이터가 끊기면 마지막 NED 시선 방향을 IMU로 계속 유지한다.
아직 유효 목표를 한 번도 얻지 못했거나 IMU도 끊기면 STOW `(0,0)`로 간다.
모터가 65°C 이상이면 토크를 끄고 FAULT가 된다.

## 먼저 확인할 설정

`src/config/system_config.h`를 하드웨어에 맞게 확인한다.

| 항목 | 기본값 | 확인 사항 |
|---|---:|---|
| `BNO085_ADDRESS` | `0x4A` | ADR 상태에 따라 `0x4B` 가능 |
| `BMP581_ADDRESS` | `0x47` | BMP581 주소는 `0x46` 또는 `0x47` |
| `MAX_M10S_ADDRESS` | `0x42` | u-blox 기본 7-bit I2C 주소 |
| `LORA_UART_BAUD` | `9600` | E22 양쪽 UART 설정과 일치 |
| `SD_CS_PIN` | `4` | 실제 SD 모듈 CS 배선으로 반드시 수정/확인 |
| DXL ID | yaw 1, pitch 2 | Wizard에서 확인 |
| `YAW_SIGN/PITCH_SIGN` | 둘 다 `-1` | 실제 기구 방향 확인 |

## 배선 기준

- I2C 공유: OpenRB D11 SDA, D12 SCL
- E22 TXD → OpenRB D13 (`Serial3 RX`)
- E22 RXD ← OpenRB D14 (`Serial3 TX`, 수신 전용이면 생략 가능)
- 현재 모듈에 표시된 E22 `NC1~NC3`는 모두 미연결
- E22 AUX는 선택 사항이다. 연결하면 설정 파일에 GPIO 번호를 넣는다.
- DYNAMIXEL은 OpenRB 전용 TTL 포트(`Serial1`)
- SD는 OpenRB 하드웨어 SPI의 SCK/MOSI/MISO와 설정된 CS 핀
- 모든 장치 GND 공통

현재 사용 중인 E22 모듈은 M0/M1 대신 `NC1~NC3`가 표시되어 있으므로 해당
핀은 연결하지 않는다. 코드는 모듈이 미리 설정된 UART 투명전송 상태라고
가정한다. 양쪽 모듈의 UART baud, 무선 channel, NETID, air data rate가
일치해야 하며, 이 코드는 E22 레지스터를 자동 변경하지 않는다.

## LoRa 패킷 규격

송신측도 `src/comm/remote_packet.h`를 사용하면 동일한 인코더를 재사용할 수
있다. 패킷은 27바이트, little-endian이다.

| Offset | Size | 내용 |
|---:|---:|---|
| 0 | 2 | ASCII `GT` |
| 2 | 1 | protocol version = 1 |
| 3 | 1 | sequence |
| 4 | 4 | 송신측 `millis()` |
| 8 | 4 | latitude, signed `1e-7 deg` |
| 12 | 4 | longitude, signed `1e-7 deg` |
| 16 | 4 | pressure `Pa × 10` |
| 20 | 2 | temperature `°C × 100`, signed |
| 22 | 1 | GPS fixType |
| 23 | 1 | bit0 GPS valid, bit1 barometer valid |
| 24 | 1 | reserved = 0 |
| 25 | 2 | CRC-16/CCITT-FALSE, bytes 0..24 |

`examples/e22_sender_example`에 10 Hz 송신 예제가 있다.

## 상대 고도에 대한 전제

코드는 두 기압의 비로 `송신자 높이 - 수신자 높이`를 계산한다. 두 센서가
비슷한 시간과 기상 조건에 있고 같은 종류의 대기압 변화를 겪는다는 전제가
필요하다. 장거리이거나 두 위치의 기압계 편차가 크면 오차가 생긴다. 실제
비행 전 두 BMP581을 같은 높이에 놓고 계산 상대고도가 0 m 부근인지 반드시
확인해야 한다.

## BNO085 장착 전제

현재 수학은 BNO085 좌표축이 짐벌 body 좌표축과 정렬되어 있다고 가정한다.
COARSE 지향은 지리적 NED 방향이 필요하므로 Game Rotation Vector가 아닌
자력계가 포함된 Rotation Vector를 사용한다. 센서를 회전 장착했으면 고정
mount quaternion 보정을 추가해야 하며, 모터 자기장 환경에서 yaw 캘리브레이션
시험이 필요하다.

## 필요한 Arduino 라이브러리

- ROBOTIS `Dynamixel2Arduino`
- SparkFun `BNO08x Arduino Library`
- Arduino 기본 `Wire`, `SPI`, `SD`

BMP581과 MAX-M10S는 이 프로젝트 내부 드라이버를 사용하므로 별도 센서
라이브러리가 필요하지 않다.

## 시리얼 명령

- `C`: 자동 추적 활성화
- `Z`: 자동 추적 비활성화 및 STOW
- `T0`, `T1`: DYNAMIXEL 토크 OFF/ON
- `P`: MAX-M10S 10 Hz UBX 설정 재전송
- `S0`: 가상 입력 해제, 실제 E22 입력 사용
- `S1`: 실제 로컬 GPS/BMP581 기준 가상 송신자 재생
- `S2`: 로컬 GPS/기압까지 가상화한 실내 벤치 재생
- `SR`: 가상 데이터셋 처음부터 재시작
- `?`: 도움말

USB 시리얼에는 1초마다 센서 신선도, 모드, 거리, 명령각, LoRa CRC/유실,
SD 기록 통계가 출력된다.

벤치 시험에서는 DYNAMIXEL 한 축만 연결해도 동작한다
(`REQUIRE_BOTH_DXL=false`). ID 1은 Yaw, ID 2는 Pitch 축으로 취급한다.

## PC 코어 테스트

Arduino와 무관한 패킷 및 수학 코어는 다음으로 검사할 수 있다.

```bash
g++ -std=c++17 -I./src tests/test_core.cpp \
    src/simulation/virtual_remote_source.cpp -o tests/test_core
./tests/test_core
```
