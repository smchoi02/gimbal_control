# S3 고정 절대좌표 표적 시뮬레이션

`S3`는 기존 `S0`, `S1`, `S2`를 변경하지 않고 추가한 시험 모드다.

## 검증 경로

```text
고정 표적 위도·경도·해발고도
              +
실제 MAX-M10S 수신부 위도·경도
              +
실제 BMP581 수신부 기압고도
              +
실제 BNO085 수신부 자세
              ↓
       상대 N/E/D 위치 계산
              ↓
         yaw/pitch 계산
              ↓
         DYNAMIXEL 제어
```

LoRa 무선 구간만 우회한다. 고정 표적은 실제 송신 패킷과 같은 형식으로
인코딩한 뒤 기존 수신 파서로 다시 읽기 때문에, 상대 위치 계산과 짐벌
제어 코드는 실제 운용 모드와 동일하다.

## 표적 좌표 설정

다음 파일의 값을 변경한 뒤 다시 업로드한다.

```text
src/simulation/fixed_target_config.h
```

```cpp
constexpr int32_t LAT_I7 = 375000000;    // 37.5000000 deg
constexpr int32_t LON_I7 = 1270000000;   // 127.0000000 deg
constexpr float ALTITUDE_M = 100.0f;     // mean sea level
constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
```

위도와 경도는 실제 도(degree)에 `10,000,000`을 곱한 정수다. 예를 들어
`37.1234567`도는 `371234567`로 입력한다. `ALTITUDE_M`는 평균해수면 기준
고도이며 `SEA_LEVEL_PRESSURE_PA`는 시험 시각·지역의 해면기압(QNH)이다.

QNH가 실제 날씨와 다르면 수평 방향은 그대로지만 계산된 상대고도와 pitch에
오차가 생긴다.

## 실행

시리얼 모니터를 115200 bps로 열고 다음을 입력한다.

```text
S3
```

설정한 표적이 출력되고 정상 상태는 다음과 같다.

```text
# simulation S3: fixed target lat/lon/alt=37.5000000,127.0000000,100.0
# mode=1 imu=1 baro=1 gps=1 remote=1 sim=3 range_m=... cmd=...,...
```

수신부를 이동하면 실제 GPS와 기압고도가 바뀌므로 같은 고정 표적에 대한
`range_m`과 `cmd`가 다시 계산된다. 수신부 본체만 회전하면 GPS 상대 위치는
같지만 BNO085 자세 보상에 따라 `cmd`가 변해야 한다.

현재 BNO085의 Game Rotation Vector는 전원을 켠 방향을 yaw 0도로 사용한다.
따라서 벤치에서 상대 지향 동작은 확인할 수 있지만 절대 북쪽 정확도 검증에는
자북/진북 기준의 heading 보정이 추가로 필요하다.
