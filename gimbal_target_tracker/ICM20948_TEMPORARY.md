# ICM-20948 임시 사용

BNO085 드라이버 파일은 삭제하지 않았으며, 현재는 ICM-20948이 선택되어 있다.

## 선택 변경

`src/config/imu_selection.h`:

```cpp
#define TRACKER_USE_ICM20948 1
```

- `1`: ICM-20948 사용
- `0`: 기존 BNO085 사용

BNO085로 돌아갈 때는 이 값만 `0`으로 바꾸고 다시 업로드한다.

## ICM-20948 연결

| ICM-20948 | OpenRB-150 |
|---|---|
| SDA | D11 / SDA |
| SCL | D12 / SCL |
| VCC | 사용하는 breakout의 전원 사양 확인, 3.3V 권장 |
| GND | GND |
| AD0 | LOW이면 `0x68`, HIGH이면 `0x69` |

드라이버는 `0x68`과 `0x69`를 모두 검사한다. BNO085는 전원과 I2C에서
분리해 두어도 소스 코드는 그대로 남는다.

## 필요한 라이브러리

Arduino Library Manager에서 다음 라이브러리를 설치한다.

```text
SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library
```

검증한 버전은 `1.3.2`다.

## 자세 계산

ICM-20948은 BNO085처럼 애플리케이션용 자세를 바로 출력하지 않으므로,
가속도계·자이로·자력계 데이터를 프로젝트 내부 Mahony 9축 필터로 융합한다.
부팅 직후에는 센서를 움직이지 않고 수 초간 안정화한다.

정상 부팅:

```text
# ICM20948: OK; I2C address=0x68
```

정상 상태:

```text
imu=1 imu_hw/report/src=1/1/I
```

`I`는 ICM-20948 자세융합을 뜻한다. 자력계는 모터, 철제 프레임, 전원선의
자기장에 민감하므로 절대 방위 시험 전에는 하드아이언/소프트아이언 보정이
필요하다.
