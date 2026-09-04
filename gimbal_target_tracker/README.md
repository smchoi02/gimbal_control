# Payload gimbal tracker

Open `gimbal_target_tracker.ino` directly in Arduino IDE and select
`OpenRB-150`. This is the only tracker document.

## What it does

The rocket's unchanged `final_test` sends this 34-byte little-endian E22 frame:

```text
RK | sequence | fixType | iTOW | latitude_i7 | longitude_i7 |
rocket_AGL_mm | velocity_N_mmps | velocity_E_mmps | velocity_D_mmps | CRC16
```

`fixType` is byte 3. Valid remote GPS is only fix type 3 or 4. Packet freshness
uses the local receive time and sequence number, never rocket iTOW.

The payload reads its own MAX-M10S, BMP581, and BNO085. It calculates NED
(north/east/down) relative position. Vertical position is:

```text
relativeUp = rocketAGL - payloadAGL - initialBaroBias
```

## Automatic behavior

There are no serial control commands and no test modes.

```text
BOOT → WAIT → ALIGN → TRACK
                     └→ HOLD on bad/stale data
```

- At startup it averages the payload pressure reference and automatically
  captures the rocket/payload barometer bias while both are near each other.
- As soon as the IMU is valid, it captures the current payload pose and gimbal
  encoder pose, then stabilizes subsequent payload tilt even if GPS/LoRa are
  not valid yet.
- When remote GPS, local GPS, local barometer, IMU, and LoRa are all valid, it
  automatically aligns for 10 seconds and starts tracking. The gimbal must be
  physically aimed at the intended reference direction during this interval.
- GPS azimuth is rejected below the configured horizontal distance threshold.
- In `TRACK`, the gimbal tracks the newest rocket direction.
- In `HOLD`, it retains the last valid world direction and uses live IMU data
  to counter payload tilt. It never commands an abrupt 0/0 stow move.
- In `FAULT`, Dynamixel health failed or temperature protection stopped it.

At one second intervals USB Serial prints state/blocker, LoRa counters and age,
GPS fixes, N/E/up, barometer values, IMU Euler angles, and gimbal target/
command/encoder positions. Typical blockers are `REMOTE_STALE`, `REMOTE_GPS`,
`LOCAL_GPS`, `LOCAL_BARO`, `IMU`, and `RANGE`.

## Settings to verify on hardware

All wiring, ports, IDs, and tunable values are in `src/config/system_config.h`.
Do not change them blindly. Bench-test the following in this order:

1. Keep the mechanism restrained and check `torque=1` plus changing IMU Y/P/R.
2. Verify `cmd` changes when the payload is tilted in `TRACK` or `HOLD`.
3. If command changes but encoder `pos` does not, check Dynamixel power/bus/
   torque rather than tracking math.
4. If motion sign is wrong, adjust only the documented motor sign or
   IMU-to-gimbal axis permutation/sign after a static-axis test.
5. Confirm GPS antenna view, BMP581 static venting, rigid IMU mounting on the
   non-moving payload base, gimbal backlash, mechanical limits, and cable
   clearance before free movement.

The gimbal trajectory is filtered and acceleration-limited at 50 Hz. The main
tuning values are `GIMBAL_TARGET_FILTER_TIME_CONSTANT_S`, `MAX_RATE_DEG_S`,
and `MAX_ACCEL_DEG_S2`.

Host parser/geometry test:

```powershell
g++ -std=c++17 -I./src tests/test_core.cpp -o tests/test_core.exe
.\tests\test_core.exe
```
