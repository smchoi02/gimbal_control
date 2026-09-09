# RTOS SD-preemption validation

This is an isolated copy of `gimbal_target_tracker`.  The original tracker is
not changed.  It intentionally enables the real experiment load: CSV logging
at 10 Hz and `flush()` at 1 Hz.

## Task ownership

| Task | Priority | Owns | Period |
| --- | ---: | --- | --- |
| `control` | 3 | Dynamixel command and one feedback request | 20 ms |
| `sensor` | 1 | the shared `Wire` bus, E22 parsing, USB status buffering | yields every 1 ms |
| `logger` | 1 | SD file open/write/flush | checks every 5 ms; writes at 10 Hz |

No ISR sends I2C or UART data.  The FreeRTOS 1 kHz tick only schedules
`control`; this prevents the unsafe pattern of doing BNO085 or Dynamixel I/O
inside an interrupt.

`sensor` and `logger` intentionally share the background priority.  The
stage-level timing test showed that a higher-priority sensor task repeatedly
starved the logger even though both were non-control work.  Equal priority
lets them time-slice while `control` still preempts either one.

The vendored `libraries/Arduino_FreeRTOS_SAMD21` dependency is the SAMD21
FreeRTOS port.  If the Arduino IDE does not discover a library stored inside a
sketch folder, copy that directory into your Arduino sketchbook's `libraries`
directory before compiling.

## First bench test

1. Upload this sketch with the same OpenRB board and library versions as the
   current tracker.
2. Keep payload restrained.  Enable SD logging (already enabled in this test)
   and rotate the IMU smoothly for at least two minutes.
3. Record the serial status and the CSV.  Compare
   `control_interval_max_us` and `control_overrun_count` with the non-RTOS SD
   test.  A valid preemption result keeps ordinary control intervals near
   20,000 us and prevents multi-second gaps while rows continue to be written.
   `sd_n` reports open attempts/errors, filename-search errors (`s`), actual
   file-open errors (`x`), header write/flush errors (`h`/`q`), log
   attempts/rows/errors, and periodic flush attempts/errors.  `sd_us` reports
   the last filename-search, file-open, header-write, header-flush,
   log-last/max, and periodic-flush-last/max duration.  This separates SD
   driver latency from task scheduling starvation.
4. Check each task's stack high-water mark before flight.  This first version
   has deliberately conservative dynamic stacks; it is a bench experiment,
   not flight approval.

## Limits that remain

The control task still performs two goal writes and one Dynamixel feedback
transaction itself.  RTOS priority cannot shorten a timeout inside that task.
If a motor disappears, that timeout must be measured and separately bounded.
Likewise, a card driver that disables global interrupts for longer than a
control period defeats all RTOS guarantees.  The installed SD library's
hardware-SPI path uses a long card-busy polling loop, which is precisely what
this experiment tests.
