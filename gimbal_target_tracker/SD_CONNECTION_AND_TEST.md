# OpenRB-150 SD 모듈 연결 및 진단

## 1. 연결

tracker는 OpenRB-150의 기본 하드웨어 SPI를 사용한다.

| SD 모듈 표기 | 신호 방향 | OpenRB-150 |
|---|---|---|
| `CS`, `SS` | OpenRB → SD | `D4` (`~4`라고 인쇄된 핀) |
| `MOSI`, `DI`, `CMD` | OpenRB → SD | `D8 / MOSI` |
| `SCK`, `CLK` | OpenRB → SD | `D9 / SCK` |
| `MISO`, `DO`, `DAT0` | SD → OpenRB | `D10 / MISO` |
| `GND` | 공통 접지 | `GND` |
| `VCC` | 모듈 전원 | 아래 전원 구분 참고 |

`MOSI`와 `MISO`를 서로 바꾸면 초기화되지 않는다. 모든 배선은 전원을 끈
상태에서 작업한다.

## 2. 모듈 전원 구분

OpenRB-150의 GPIO 논리 전압은 3.3V다.

- `3.3V 전용`, 레귤레이터가 없는 소형 breakout: `VCC → 3.3V`
- `5V 지원`이라고 표시되고 AMS1117 같은 레귤레이터와 레벨 시프터가 있는
  일반 SD 모듈: 보통 `VCC → 5V`
- 모듈 사양을 모르면 5V를 임의로 연결하지 말고 제품 설명이나 기판의
  레귤레이터 표기를 먼저 확인한다.

5V용 모듈의 VCC를 3.3V에 연결하면 온보드 레귤레이터의 전압 강하 때문에
카드 전압이 부족해져 `SD.begin()`이 실패할 수 있다. 반대로 3.3V 전용
모듈에 5V를 연결하면 카드와 OpenRB가 손상될 수 있다.

## 3. microSD 카드 조건

- 먼저 8GB~32GB microSD를 권장한다.
- 파티션 하나를 `FAT32`로 포맷한다.
- `exFAT`이나 `NTFS`는 현재 Arduino SD 라이브러리에서 사용하지 않는다.
- 어댑터를 사용한다면 쓰기 방지 스위치를 해제한다.
- 카드 삽입과 제거는 전원을 끄고 수행한다.

## 4. 독립 진단

Arduino IDE에서 다음 스케치를 열고 OpenRB-150에 업로드한다.

```text
examples/sd_diagnostic/sd_diagnostic.ino
```

시리얼 모니터는 115200 bps로 연다. 완전 정상일 때:

```text
# CARD INIT OK type=SDHC/SDXC
# FAT VOLUME OK FAT32
# READBACK: OpenRB-150 SD write test OK
# SD DIAGNOSTIC PASS
```

결과 해석:

- `CARD INIT FAIL`: 전원, CS/MOSI/MISO/SCK 배선, 카드 삽입 문제
- `FAT VOLUME FAIL`: exFAT, 손상된 파티션 또는 지원하지 않는 포맷
- `FILE OPEN FAIL`: 파일시스템 손상 또는 쓰기 방지
- `FILE WRITE FAIL`: 전원 불안정, 접촉 불량 또는 카드 불량

진단에 성공하면 카드 루트에 `SDTEST.TXT`가 생성된다.

## 5. tracker에서 정상 확인

진단 스케치가 성공하면 tracker를 다시 업로드한다. 부팅할 때:

```text
# SD: OK file=TRK000.CSV
```

파일 번호는 기존 파일에 따라 증가한다. 실행 중에는:

```text
sd_rows/errors=10/0
sd_rows/errors=20/0
```

처럼 첫 번째 숫자가 초당 약 10개씩 증가하고 오류는 0이어야 한다.

tracker는 `TRK000.CSV`부터 `TRK999.CSV`까지만 새 이름을 찾는다. 카드에
해당 파일이 1000개 모두 있으면 카드 초기화가 정상이어도 부팅 메시지가
`SD: FAIL`로 나온다. 이 경우 필요한 로그를 백업한 뒤 기존 `TRKxxx.CSV`
파일을 정리한다.
