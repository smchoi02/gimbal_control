# Payload gimbal tracker

로켓의 `final_test` 송신 패킷을 LoRa(E22)로 수신하고, 페이로드의 GPS, BMP581, BNO085 및 Dynamixel 짐벌을 사용해 로켓 방향을 지향하는 수신부입니다.

`final_test`는 송신부의 단일 기준이며 이 폴더에서는 수정하지 않습니다.

## 패킷 기준

수신기는 `final_test`의 34-byte `RK` 패킷을 기준으로 동작합니다. GPS 상태 바이트는 위성 수가 아니라 u-blox `fixType`입니다. 따라서 `fixType=3`은 정상적인 3D fix이며 유효한 원격 위치로 처리합니다. 패킷 신선도는 GPS iTOW가 아니라 마지막 정상 패킷의 로컬 수신 시각과 sequence로 판단합니다.

## 자동 동작 흐름

`BOOT -> WAIT -> ALIGN -> TRACK` 순서로 자동 진행합니다. 데이터가 끊기거나 방향 계산이 불가능하면 마지막 안전 시선 방향을 유지하는 `HOLD`로 들어가며, Dynamixel 이상은 `FAULT`로 처리합니다. `HOLD`에서 입력이 다시 정상이어도 0.5초 동안 연속 정상인지 확인한 뒤 `TRACK`으로 복귀한다. 이 동안에도 IMU 기반 hold 제어는 계속된다. 별도 Serial 명령은 필요하지 않습니다.

IMU 자세가 유효해지면 GPS fix가 아직 없어도 현재 짐벌 엔코더 위치와 IMU 자세를 소프트웨어 기준점으로 잡아 기울임 보정을 시작합니다. GPS 기반 로켓 추적은 GPS, 기압계, LoRa 조건까지 충족한 뒤에만 추가됩니다.

## 상태(state)

| 상태 | 의미 |
| --- | --- |
| `BOOT` | 기동 직후 초기 상태 |
| `WAIT` | 센서, 통신, 기압 기준값 등 `TRACK` 준비를 기다리는 상태 |
| `ALIGN` | 모든 입력이 준비된 뒤 10초 동안 초기 기준 방향을 잡는 상태 |
| `TRACK` | 최신 로켓 위치로 짐벌을 추적하는 상태 |
| `HOLD` | 마지막 정상 시선 방향을 유지하고 IMU로 흔들림만 보정하는 안전 유지 상태 |
| `FAULT` | Dynamixel 통신, 온도 등 짐벌 이상 상태 |

## 차단 사유(block)

| 차단 사유 | 의미 |
| --- | --- |
| `NONE` | 차단 사유 없음 |
| `GIMBAL` | 짐벌 이상 |
| `ALIGN` | 초기 정렬 진행 중 |
| `UNSTABLE` | 정렬 중 IMU 움직임 또는 표적 방향 변화가 너무 큼. 움직임이 멈추면 10초 정렬을 다시 시작함 |
| `BARO_REF` | 수신부 기압 기준값 준비 안 됨 |
| `BARO_BIAS` | 이전 코드에서 초기 기압 보정값 준비 대기를 나타내던 값. 현재 보정은 백그라운드로 수행되며 tracking을 차단하지 않음 |
| `REMOTE_STALE` | 로켓 패킷이 시간 초과됨 |
| `REMOTE_GPS` | 로켓 GPS가 유효하지 않음 |
| `LOCAL_GPS` | 수신부 GPS가 유효하지 않거나 최신 위치가 시간 초과됨 |
| `LOCAL_BARO` | 수신부 기압계가 유효하지 않음 |
| `IMU` | IMU가 유효하지 않음 |
| `RANGE` | 수평 거리가 최소 조건보다 짧음 |
| `GEOMETRY` | 위치/자세로 방향 계산을 만들 수 없음 |

`local_fix=3` 또는 `remote_fix=3`이라도 마지막 GPS 측정값이 오래되었거나 u-blox의 위치 유효 플래그가 없으면 각각 `LOCAL_GPS`, `REMOTE_GPS`가 될 수 있습니다. 페이로드 MAX-M10S에서 NAV-PVT 출력 자체가 3초 이상 멈추면 수신부는 10 Hz 출력 설정을 다시 적용하며, 이 시도 횟수는 `gps_recov`로 표시합니다.

## Serial 상태 줄

예시:

```text
state/block=TRACK/NONE lora=ok532 bad3 lost5 age23 sens_age=20/100/20 gps_recov=0 remote_fix=3 local_fix=3 ned=12.4/-3.1/8.2 baro=2.6/-0.1/2.7 imu_ypr=-0.0/1.1/-2.6 target=14.0/31.5 cmd=13.6/31.2 pos=13.4/31.0
```

| 항목 | 의미 |
| --- | --- |
| `state/block` | 현재 모드/그 모드가 된 원인. `FAULT/GIMBAL`은 짐벌 문제로 정지한 상태 |
| `lora=ok532` | CRC까지 정상인 LoRa 패킷 누적 532개 |
| `bad3` | 형식 또는 CRC가 틀려 버린 패킷 누적 3개 |
| `lost5` | sequence 번호상 중간에 빠진 패킷 누적 5개 |
| `age23` | 마지막 새 패킷을 받은 뒤 경과 시간(ms). 보통 수십 ms 정도면 정상 |
| `sens_age=20/100/20` | IMU/페이로드 GPS/BMP581의 마지막 새 측정 후 경과 시간(ms). 각각 200/1500/300 ms 이하가 정상 조건 |
| `gps_recov=0` | 페이로드 MAX-M10S NAV-PVT 출력 자동 복구 시도 횟수. 증가가 계속되면 GPS 전원 또는 I²C 연결을 점검 |
| `remote_fix=3` | 로켓 GPS fix 상태. `3`은 정상 3D fix, `0`은 fix 없음 |
| `local_fix=3` | 수신부(페이로드) GPS fix 상태. `3`이면 정상 3D fix |
| `ned=N/E/Up` | 로켓의 수신부 기준 상대 위치[m]: 북쪽/동쪽/위쪽. `FAULT` 또는 `HOLD`에서는 마지막 계산값 또는 초기값일 수 있음 |
| `baro=rocket/local/bias` | 로켓 기압 고도[m]/수신부 기압 고도[m]/양쪽 기압계 초기 오차 보정값[m] |
| `bias=ready/pending/samples` | 기압계 초기 오차 보정 완료 여부와 누적 샘플 수. `pending`이어도 GPS/IMU 추적은 진행됨 |
| `imu_ypr=yaw/pitch/roll` | 수신부 IMU 자세[deg] |
| `target=yaw/pitch` | 계산된 로켓 방향. 짐벌 초기 정렬 기준을 뺀 목표 각도[deg] |
| `cmd=yaw/pitch` | 실제 모터에 보낸 목표 각도[deg]. 급격한 동작을 막는 필터가 적용됨 |
| `pos=yaw/pitch` | Dynamixel 엔코더에서 읽은 현재 각도[deg] |

## 운용 주의

- GPS만으로 방위각을 계산하려면 로켓과 페이로드가 충분히 떨어져야 합니다. `TRACK`과 자동 초기 정렬의 최소 수평 거리는 모두 5 m이고, 초기 정렬에는 10초 안정 구간이 필요합니다. 기압계 초기 오차 보정값 수집은 두 장치가 수평 30 m 이내일 때만 수행합니다.
- 짐벌 목표 명령은 50 Hz로 계속 전송한다. 제어 tick마다 배경 I/O는 최대 한 항목만 실행한다. Dynamixel 위치 feedback은 10 Hz, 전류는 5 Hz, 온도는 축당 2.5 Hz로 분산해 읽으며, GPS는 호출당 최대 32 byte만 처리한다. 토크가 켜진 추적 중에는 SD 쓰기와 긴 상태 Serial 출력이 기본 비활성화되어 제어 주기를 방해하지 않는다.
- 비행 중 SD 로그 또는 상태 Serial 출력을 다시 켜야 한다면 `system_config.h`의 `ALLOW_SD_WRITES_WHILE_TORQUE_ON` 또는 `ALLOW_STATUS_OUTPUT_WHILE_TORQUE_ON`을 `true`로 바꿀 수 있다. 두 작업은 실행 시간이 보장되지 않으므로, 추적 끊김을 조사하거나 최소화하는 동안에는 `false`를 유지한다.
- 두 기기가 같은 높이에 있는 preflight 구간에서 기압계 오프셋을 잡습니다. 보정은 수평거리 30 m 이하에서 30개 LoRa 패킷을 모아 수행하지만, 이 조건을 못 맞춰도 tracking은 시작합니다. 이때 pitch에는 기압계 초기 오차가 남을 수 있습니다.
- BNO085 Game Rotation Vector의 yaw는 절대 북쪽 기준이 아닙니다. 기구 영점, IMU 축 방향, 짐벌 축 방향과 모터 회전 방향은 설정 상수로 보정해야 합니다.
- GPS 안테나 배치, BMP581 통풍, IMU 축 고정, 짐벌 기구 유격, Dynamixel 영점 및 케이블 간섭은 코드만으로 해결할 수 없는 하드웨어 점검 항목입니다.
