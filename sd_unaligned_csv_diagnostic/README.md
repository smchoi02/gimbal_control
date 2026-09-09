# SD unaligned CSV diagnostic

This is a standalone OpenRB-150 SD test. It reproduces the logger's relevant
write shape without FreeRTOS, sensors, or Dynamixel activity:

- one growing file;
- 312 bytes per record (the measured maximum CSV-row length);
- one record every 100 ms (10 Hz);
- `flush()` every ten records (1 s);
- 128 records / 12.8 s, then full read-back verification.

Upload `sd_unaligned_csv_diagnostic.ino`, open the 115200-baud Serial Monitor,
then reset the board. The sketch waits 1.5 seconds and does not depend on the
monitor connection; after it finishes it repeats a completion heartbeat every
second. Preserve and send the `unaligned summary` line (and any
`result=FAIL` line). The test creates a new `Uxxxxxxx.BIN` file and never
overwrites a previous test result.
