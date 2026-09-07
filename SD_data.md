# SD 데이터 기록

## 현재 파일명 규칙

| 장치 | 실제 파일명 | 예시 |
| --- | --- | --- |
| 로켓 `final_test` | `RMMDDNNN.CSV` | `R0906000.CSV` |
| 페이로드 `gimbal_target_tracker` | `PMMDDNNN.CSV` | `P0906000.CSV` |

- `MMDD`는 각 장치의 MAX-M10S가 받은 UBX NAV-PVT UTC 날짜다. 9월 6일은 `0906`이다.
- GPS 날짜가 부팅 뒤 15초 안에 유효해지지 않으면 `0000`을 쓴다. 예: `R0000000.CSV`, `P0000000.CSV`.
- `NNN`은 같은 날짜에 이미 존재하는 파일을 피하기 위한 번호다. `000`부터 빈 번호를 찾아 `999`까지 증가한다. 기존 파일에는 이어 쓰지 않는다.
- `0000` 파일을 연 뒤 GPS 날짜가 유효해지면, 기존 파일을 닫아 보존하고 이후 행은 실제 날짜의 새 파일에 쓴다. 따라서 GPS 미획득 구간과 이후 구간은 파일이 나뉠 수 있다.
- LoRa 송신과 짐벌 제어는 SD와 독립적으로 동작한다. 로켓과 페이로드 SD는 각각 2 Hz로 한 행씩 기록하고, 약 5초마다 `flush()`한다. 갑작스러운 전원 차단 시 마지막 약 5초의 행은 유실될 수 있다. 로켓 LoRa 송신은 10 Hz, 페이로드 짐벌 목표 명령은 50 Hz를 유지한다. 페이로드 Dynamixel 위치/전류/온도 feedback은 UART 제어 끊김을 줄이기 위해 2 Hz로 읽으며, CSV의 feedback 값은 기록 시점의 최신 값이다.

## 파일 생성/수정시각과 날짜 소스

### 로켓: Arduino Uno

- Uno에는 RTC가 없으므로 GPS UTC가 없으면 SD 카드 FAT 파일시각은 기본값 `2000-01-01 00:00`으로 보인다. 이 값은 실제 날짜가 아니다.
- `final_test`는 MAX-M10S NAV-PVT의 UTC 날짜와 시간이 모두 유효해지면, SD FAT 파일 생성/수정시각을 그 GPS UTC로 기록한다.
- GPS가 전혀 없는 상태에서 실제 날짜를 자동으로 알 수는 없다. 날짜를 고정값으로 코드에 넣거나 DS3231 같은 배터리 백업 RTC를 추가해야 한다.

### 페이로드: OpenRB-150

- 현재 `gimbal_target_tracker`는 MAX-M10S GPS 날짜를 파일명에만 사용한다. SD FAT 파일 생성/수정시각을 설정하는 코드는 현재 없다. 따라서 PC에서 보이는 파일시각은 신뢰할 수 있는 비행 시간 정보가 아니다.
- OpenRB-150의 SAMD21에는 전원이 켜진 동안 사용할 수 있는 RTC 주변장치가 있지만, 현재 코드에서 설정하지 않으며 배터리 백업된 달력 시계도 아니다. 전원을 끄면 실제 날짜를 유지하지 못한다.
- GPS 없이 전원을 새로 켠 경우 실제 날짜가 필요하면 코드의 고정 날짜 설정, USB/지상국에서 시간 주입, 또는 DS3231 같은 외부 배터리 백업 RTC가 필요하다.

## 로켓: `final_test` CSV

로켓에서 실제 LoRa로 송신한 34-byte `RK` 패킷을 CSV로 풀어 기록한다. `gps_fresh=0`이면 GPS 관련 필드는 송신 패킷과 같이 0 또는 로컬 시간 대체값이며, `baro_valid=0`이면 `agl_mm=0`이다.

| CSV 열 | 의미 |
| --- | --- |
| `ms` | Arduino `millis()` 기준 기록 시각 |
| `seq` | RocketLink sequence 번호 |
| `fix_type` | UBX NAV-PVT GPS fixType. `3`은 정상 3D fix |
| `gps_fresh` | 최근 GPS 위치가 유효하고 신선한지 (`1`/`0`) |
| `itow_ms` | GPS iTOW [ms]. GPS가 stale이면 로컬 `ms` 대체값 |
| `lat_i7`, `lon_i7` | 위도/경도, `1e-7 deg` 정수 단위 |
| `agl_mm` | BMP581 기준 기압 AGL [mm] |
| `vel_n_mm_s`, `vel_e_mm_s`, `vel_d_mm_s` | GPS 속도 North/East/Down [mm/s] |
| `hacc_mm`, `sacc_mm_s` | GPS 수평 위치 정확도/속도 정확도 추정치 |
| `baro_valid` | BMP581 AGL이 유효한지 (`1`/`0`) |
| `crc16` | 송신한 RocketLink 패킷의 CRC-16/CCITT-FALSE 값 |

## 페이로드: `gimbal_target_tracker` CSV

한 행에 페이로드 센서 입력, LoRa 로켓 수신값, 상대좌표 계산 결과, 짐벌 명령 및 Dynamixel feedback을 함께 기록한다. `mode`와 `blocker`의 숫자 뜻은 `gimbal_target_tracker/README.md`의 상태 표를 따른다. Serial의 `sens_age`와 `gps_recov`는 실시간 진단값이며 현재 CSV 열에는 기록하지 않는다.

| CSV 열 묶음 | 포함 열 | 의미 |
| --- | --- | --- |
| 시간/상태 | `t_ms`, `mode`, `blocker` | 로컬 시간, 추적 상태, 차단 사유 |
| IMU | `imu_valid`, `qw`, `qx`, `qy`, `qz`, `yaw_deg`, `pitch_deg`, `roll_deg` | BNO085 quaternion(w/x/y/z)과 자세[deg] |
| 페이로드 기압계 | `local_baro_valid`, `local_baro_reference_valid`, `local_pressure_pa`, `local_temp_c`, `local_agl_m`, `local_reference_pa` | BMP581 유효성, 기준압력, 압력/온도/자체 AGL |
| 페이로드 GPS | `local_gps_valid`, `local_itow_ms`, `local_lat_i7`, `local_lon_i7`, `local_hmsl_mm`, `local_fix_type`, `local_num_sv` | MAX-M10S 위치, 고도, fixType, 위성 수 |
| 로켓 LoRa 수신값 | `remote_valid`, `remote_seq`, `remote_itow_ms`, `remote_lat_i7`, `remote_lon_i7`, `remote_agl_m`, `remote_vel_n_mps`, `remote_vel_e_mps`, `remote_vel_d_mps`, `remote_fix_type` | `final_test` RocketLink 패킷에서 파싱한 로켓 데이터 |
| 기압 보정 | `initial_baro_bias_m`, `baro_bias_ready` | 로켓/페이로드 기압 AGL 초기 오차 및 보정 준비 상태 |
| 상대 위치/표적 | `rel_n_m`, `rel_e_m`, `rel_d_m`, `rel_up_m`, `range_m`, `target_yaw_deg`, `target_pitch_deg` | 페이로드 기준 로켓 NED/Up 거리, 3D 거리, 계산된 짐벌 목표각 |
| 짐벌 명령/feedback | `gimbal_yaw_cmd`, `gimbal_pitch_cmd`, `gimbal_yaw_pos`, `gimbal_pitch_pos`, `gimbal_cur_y`, `gimbal_cur_p`, `gimbal_temp_y`, `gimbal_temp_p`, `gimbal_limit`, `gimbal_torque` | Dynamixel 명령 각도, 엔코더 현재각, 전류 raw값, 온도, 제한/토크 상태 |

좌표계는 NED(North, East, Down)이며 `rel_up_m = -rel_d_m`이다. 기압 AGL은 각 장치의 자체 기준압력 기준이므로, 장치 사이 비교에는 `initial_baro_bias_m`을 함께 사용해야 한다.
