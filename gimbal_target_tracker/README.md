# gimbal_target_tracker

OpenRB-150 기반 표적 추적 짐벌 펌웨어다.

- 자세: BNO085 (`Wire` I2C)
- 로컬 위치/고도: MAX-M10S, BMP581 (`Wire` I2C)
- 원격 표적 링크: EBYTE E22-900T22S (`Serial3` UART)
- 구동: DYNAMIXEL yaw/pitch (`Serial1`)
- 기록: SD 카드 (SPI)

Arduino IDE에서 `gimbal_target_tracker.ino`를 열어 OpenRB-150 대상으로 빌드한다.
ICM-20948 관련 선택 분기와 드라이버는 제거되었으며, 이 프로젝트의 IMU는 BNO085만 사용한다.

배선은 [WIRING.md](WIRING.md)를 참고한다. 이 펌웨어는 실제 E22 수신 패킷만 사용한다.

## 동작

1. E22-900T22S가 투명 UART 모드로 원격 GPS·AGL·속도 패킷을 수신한다.
2. 로컬 MAX-M10S/BMP581과 원격 데이터를 NED 목표 벡터로 계산한다.
3. BNO085 Rotation Vector로 NED 벡터를 짐벌 body 좌표계로 회전한다.
4. yaw/pitch 명령을 기구 한계와 속도 제한에 맞춰 DYNAMIXEL에 전송한다.

원격·GPS·기압 입력이 끊기면 마지막 NED 방향을 BNO085 자세로 유지한다. 유효 방향이나 BNO085 자세도 없으면 STOW `(0, 0)`로 전환한다.

## 하드웨어 설정

`src/config/system_config.h`에서 실제 배선과 일치하는지 확인한다.

| 항목 | 기본값 | 설명 |
|---|---:|---|
| `BNO085_ADDRESS` | `0x4A` | ADR 상태에 따라 `0x4B` 가능 |
| `BNO_USE_GAME_ROTATION_VECTOR` | `true` | 전원 인가 방향 기준 yaw. 북 기준 추적을 검증한 뒤에만 `false` 고려 |
| `BMP581_ADDRESS` | `0x47` | 실제 보드 주소 확인 |
| `MAX_M10S_ADDRESS` | `0x42` | u-blox 기본 I2C 주소 |
| `E22_UART_BAUD` | `9600` | 송신·수신 E22 UART 설정과 반드시 일치 |
| `E22_M0_PIN`, `E22_M1_PIN` | `-1` | `-1`이면 두 모드 핀을 물리적으로 GND에 연결 |
| `E22_AUX_PIN` | `-1` | 선택. 연결하면 모듈 준비 상태를 상태 출력에 반영 |

E22-900T22S는 M0/M1을 부동 상태로 두면 안 된다. 정상 투명 전송은 `M0=LOW`, `M1=LOW`이며, 두 핀을 GND에 고정하거나 각각 MCU GPIO로 구동한다. 송신기와 수신기의 UART baud, 900 MHz 대역 채널, NETID, air data rate, 암호화 설정은 모두 같아야 한다. 이 펌웨어는 E22 레지스터를 자동 설정하지 않는다.

## 원격 패킷

수신기는 `trs_test.ino`의 34-byte little-endian `RK` 패킷을 해석한다. 실제 수신 검증에는 `examples/e22_link_rx_test/e22_link_rx_test.ino`를 사용한다.

| Offset | Size | 내용 |
|---:|---:|---|
| 0 | 2 | ASCII `RK` |
| 2 | 1 | sequence |
| 3 | 1 | GPS fix type |
| 4 | 4 | temporary iTOW ms |
| 8 | 4 | latitude, signed `1e-7 deg` |
| 12 | 4 | longitude, signed `1e-7 deg` |
| 16 | 4 | AGL, signed mm |
| 20 | 4 | north velocity, signed mm/s |
| 24 | 4 | east velocity, signed mm/s |
| 28 | 4 | down velocity, signed mm/s |
| 32 | 2 | CRC-16/CCITT-FALSE, bytes 0..31 |

## BNO085 주의사항

현재 좌표 변환은 BNO085 축과 짐벌 body 축이 정렬됐다고 가정한다. 센서 장착 방향이 다르면 mount quaternion 보정을 추가해야 한다. `Game Rotation Vector`는 자력을 쓰지 않아 전원 인가 시점의 yaw가 0°가 된다. 지리적 북 기준 추적이 필요한 경우, 모터·배선 자기장 영향을 포함해 `Rotation Vector`의 현장 검증과 yaw 보정을 먼저 수행해야 한다.

필요한 외부 라이브러리는 ROBOTIS `Dynamixel2Arduino` 및 SparkFun `BNO08x Arduino Library`다.

## 시리얼 명령

- `C`: 추적 활성화
- `Z`: 추적 비활성화 및 STOW
- `T0` / `T1`: DYNAMIXEL 토크 OFF / ON
- `P`: MAX-M10S 10 Hz 설정 재전송
- `?`: 명령 도움말

1초 상태 출력의 `imu_hw/report/src=1/1/G`는 BNO085 Game Rotation Vector가 정상이라는 뜻이다. `e22_ready=0`은 AUX를 연결한 경우 모듈이 기동 중이거나 busy 상태임을 뜻한다.

## PC 코어 테스트

```bash
g++ -std=c++17 -I./src tests/test_core.cpp -o tests/test_core
./tests/test_core
```
