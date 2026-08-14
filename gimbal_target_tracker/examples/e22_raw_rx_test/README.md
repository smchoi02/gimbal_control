# E22 raw receive test

Open `e22_raw_rx_test.ino` in Arduino IDE, select the OpenRB-150, and upload.
Open Serial Monitor at **115200 baud**. The E22 module uses `Serial3` at
**9600 baud**, matching the current `trs_test.ino` UART setting.

This is a raw UART/RF link check: it does not require an `RK` packet, a CRC,
GPS, an IMU, or a gimbal. Any received byte sequence is printed after a 50 ms
idle gap, as both hexadecimal and printable ASCII.

For example, a transmitter sending `HELLO` produces:

```text
RX t_ms=1234 len=5 hex=48 45 4C 4C 4F ascii="HELLO"
```

For binary packets, non-printable ASCII bytes are shown as `.` while the HEX
section preserves the exact received values. Both E22 modules must use the
same UART speed, channel, NETID, air rate, and encryption settings; wire
`M0=LOW` and `M1=LOW` for normal transparent mode.
