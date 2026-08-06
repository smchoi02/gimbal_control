# 독립 하드웨어 시험 스케치

각 폴더의 `.ino` 하나만 Arduino IDE에서 열어 해당 OpenRB-150에 업로드한다. 모든 시리얼 모니터 속도는 115200 baud다.

| 스케치 | 대상 | 성공 기준 |
|---|---|---|
| `bno085_read_test` | BNO085 | `present/report/valid=1/1/1`, 자세 변화에 yaw/pitch/roll 변화 |
| `bmp581_read_test` | BMP581 | `present/valid=1/1`, 정상 대기압·온도 출력 |
| `max_m10s_read_test` | MAX-M10S | `present/valid=1/1`, fix type 3 또는 4와 위경도 출력 |
| `e22_link_rx_test` | 수신 E22-900T22S | `rx_ok` 증가, `bad/lost=0`, `age_ms` 약 1000 이하 |

## E22-900T22S 실제 링크 시험 순서

1. 두 E22-900T22S의 M0/M1을 LOW(GND)로 고정하고 안테나와 공통 GND를 연결한다.
2. 두 모듈의 UART baud(9600), 900 MHz 대역 RF 채널, NETID, air data rate, 암호화를 동일하게 설정한다.
3. 송신 보드에는 실제 `trs_test.ino`, 수신 OpenRB에는 `e22_link_rx_test`를 업로드한다.
4. 수신 시리얼 모니터에서 `RK seq=...` 출력과 `rx_ok` 증가를 확인한다.

이 링크 시험은 실제 무선 구간에서 `trs_test.ino`의 production `RK` 34-byte 패킷을 CRC까지 검증한다. 따라서 배선, E22-900T22S 설정, RF 수신, 패킷 파서를 실제 송신 코드 기준으로 확인한다.
