# OpenRB SD card diagnostic

This standalone sketch intentionally excludes FreeRTOS, IMU, GPS, Dynamixel,
and the tracker.  It isolates the exact SD path used by the payload:

`SD.begin -> SD.open -> 512-byte write -> flush -> close -> reopen/read/verify`

The serial result must show `bytes=512 read=512 verify=1`.  A zero in either
byte count means the wrong sketch version is still on the board.

It creates up to ten unique 8.3 files named `S0000000.BIN`, etc.  It never
deletes or overwrites a card file.

Upload `sd_card_diagnostic.ino` to the same OpenRB-150 with the same SD card,
socket, power supply, and wiring.  Copy the complete serial output after the
ten-test summary.

Interpretation:

- `begin=fail`: card detection/SPI/power problem.
- `result=open-fail`: FAT/directory or physical-card operation failure.
- `we=1` or `fe=1`: write or flush failure.
- `verify=0`: data was not read back exactly after a successful flush.
- Any `s`, `o`, `w`, `f`, or `c` time in the hundreds of milliseconds or seconds
  is unsuitable for a synchronous control-loop logger, regardless of RTOS.
