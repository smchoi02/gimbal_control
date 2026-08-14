# Fixed-receiver transmitter replay

`replay_relative_position.py` is a standalone PC tool for a `TRKxxx.CSV`
file copied from the SD card written by `gimbal_target_tracker`.

It deliberately holds the receiver fixed at one GPS coordinate and draws the
transmitter relative to it.  The fixed coordinate defaults to the first valid
`local_lat_i7` / `local_lon_i7` CSV record, so later local-GPS wander does not
move the receiver in the animation.

The current `trs_test.ino` RK packet is 34 bytes: `RK`, sequence, fix type,
iTOW, latitude, longitude, AGL, N/E/D velocity, then CRC-16/CCITT-FALSE over
bytes 0 through 31.  The replay uses the logger's decoded `remote_*` columns;
it does not alter the transmitter packet or require a transmitter-side change.

## Run

Install Python 3 and matplotlib, then copy a `TRKxxx.CSV` from the SD card.

```powershell
py -m pip install matplotlib
py replay_relative_position.py E:\logs\TRK000.CSV
```

For a known fixed receiver origin instead of using the first valid local GPS
sample, supply both values in signed `1e-7 degree` units:

```powershell
py replay_relative_position.py E:\logs\TRK000.CSV `
  --fixed-lat-i7 375000000 --fixed-lon-i7 1270000000
```

The left panel animates North/East position and path.  The receiver stays at
`(0, 0)` and the red line is its instantaneous line of sight to the
transmitter.  The right panel shows transmitter AGL and three-dimensional
range over time.  The default 1 ms frame interval is 100× faster than the
original 50 ms setting. Use `--stride 5` for a large log.

`--save replay.gif` or `--save replay.mp4` exports an animation when the
matching matplotlib writer (Pillow or ffmpeg) is installed.
