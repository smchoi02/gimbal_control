# SD file-growth diagnostic

This standalone OpenRB sketch isolates FAT/file expansion.  It creates one
new file and appends 128 × 512-byte blocks (64 KiB), flushing every 8 blocks.
It then reopens the file and verifies every block's contents.

It never overwrites or removes files; it creates `G0000000.BIN`-style files.

Send the complete output, especially the first `result=FAIL` line and the
final `grow summary`.  A failure at a repeatable `b=`/`size=` points to file
growth/allocation rather than CSV formatting or the gimbal control loop.
