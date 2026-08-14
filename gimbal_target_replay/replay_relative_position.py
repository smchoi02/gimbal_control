#!/usr/bin/env python3
"""Replay a gimbal_target_tracker TRKxxx.CSV file as a relative-position animation.

The receiver is intentionally fixed at the origin.  Its origin is either the
first valid local GPS record in the CSV, or --fixed-lat-i7/--fixed-lon-i7.
This matches bench/field analysis where the receiver may have GPS jitter but
is physically stationary.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.animation as animation
import matplotlib.pyplot as plt


METERS_PER_I7_LAT = 0.0111320


@dataclass(frozen=True)
class ReplayPoint:
    time_s: float
    north_m: float
    east_m: float
    up_m: float
    range_m: float
    seq: int


def integer(value: str, field: str) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid {field}: {value!r}") from error


def number(value: str, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid {field}: {value!r}") from error


def valid_remote(row: dict[str, str]) -> bool:
    """Accept usable position records from current and legacy RK CSV logs."""
    if row.get("remote_valid") != "1":
        return False
    if "remote_satellites" in row:
        quality_ok = integer(row["remote_satellites"], "remote_satellites") >= 4
    else:
        quality_ok = integer(row.get("remote_fix_type", "0"), "remote_fix_type") in (3, 4)
    return (quality_ok and integer(row.get("remote_lat_i7", "0"), "remote_lat_i7") != 0
            and integer(row.get("remote_lon_i7", "0"), "remote_lon_i7") != 0)


def first_fixed_position(rows: Iterable[dict[str, str]]) -> tuple[int, int]:
    for row in rows:
        if row.get("local_gps_valid") != "1":
            continue
        lat = integer(row.get("local_lat_i7", "0"), "local_lat_i7")
        lon = integer(row.get("local_lon_i7", "0"), "local_lon_i7")
        if lat != 0 and lon != 0:
            return lat, lon
    raise ValueError("CSV has no valid local GPS row; provide --fixed-lat-i7 and --fixed-lon-i7")


def load_points(path: Path, fixed_lat_i7: int | None,
                fixed_lon_i7: int | None) -> tuple[list[ReplayPoint], tuple[int, int]]:
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))
    required = {"t_ms", "remote_valid", "remote_lat_i7", "remote_lon_i7", "remote_agl_m"}
    if not rows or not required.issubset(rows[0]):
        missing = ", ".join(sorted(required - set(rows[0] if rows else {})))
        raise ValueError(f"not a gimbal tracker CSV; missing: {missing}")

    if (fixed_lat_i7 is None) != (fixed_lon_i7 is None):
        raise ValueError("provide both fixed GPS coordinates or neither")
    origin = (fixed_lat_i7, fixed_lon_i7) if fixed_lat_i7 is not None else first_fixed_position(rows)
    origin_lat_i7, origin_lon_i7 = origin
    cos_lat = __import__("math").cos(origin_lat_i7 * 1.0e-7 * __import__("math").pi / 180.0)

    points: list[ReplayPoint] = []
    time0_ms: float | None = None
    for row in rows:
        if not valid_remote(row):
            continue
        now_ms = number(row["t_ms"], "t_ms")
        if time0_ms is None:
            time0_ms = now_ms
        lat_i7 = integer(row["remote_lat_i7"], "remote_lat_i7")
        lon_i7 = integer(row["remote_lon_i7"], "remote_lon_i7")
        north_m = (lat_i7 - origin_lat_i7) * METERS_PER_I7_LAT
        east_m = (lon_i7 - origin_lon_i7) * METERS_PER_I7_LAT * cos_lat
        up_m = number(row["remote_agl_m"], "remote_agl_m")
        points.append(ReplayPoint((now_ms - time0_ms) / 1000.0, north_m, east_m,
                                  up_m, (north_m**2 + east_m**2 + up_m**2) ** 0.5,
                                  integer(row.get("remote_seq", "0"), "remote_seq")))
    if not points:
        raise ValueError("no valid remote GPS records in CSV")
    return points, origin


def padded_limits(values: list[float]) -> tuple[float, float]:
    low, high = min(values), max(values)
    padding = max((high - low) * 0.1, 5.0)
    return low - padding, high + padding


def animate(points: list[ReplayPoint], origin: tuple[int, int], interval_ms: int,
            stride: int, output: Path | None) -> None:
    shown = points[::stride]
    if shown[-1] is not points[-1]:
        shown.append(points[-1])

    figure = plt.figure(figsize=(12, 6.5), constrained_layout=True)
    map_axis = figure.add_subplot(1, 2, 1)
    height_axis = figure.add_subplot(1, 2, 2)
    map_axis.set_title("Transmitter relative position (receiver fixed at origin)")
    map_axis.set_xlabel("East [m]")
    map_axis.set_ylabel("North [m]")
    map_axis.grid(True, alpha=0.3)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlim(*padded_limits([0.0] + [point.east_m for point in shown]))
    map_axis.set_ylim(*padded_limits([0.0] + [point.north_m for point in shown]))
    map_axis.plot(0, 0, marker="^", color="tab:blue", markersize=10, label="Receiver (fixed)")
    trail, = map_axis.plot([], [], color="tab:orange", linewidth=1.5, label="Transmitter trail")
    transmitter, = map_axis.plot([], [], marker="o", color="tab:red", markersize=8,
                                 label="Transmitter")
    vector, = map_axis.plot([], [], color="tab:red", alpha=0.45)
    map_axis.legend(loc="best")

    height_axis.set_title("Altitude and range")
    height_axis.set_xlabel("Time [s]")
    height_axis.set_ylabel("Meters")
    height_axis.grid(True, alpha=0.3)
    height_axis.set_xlim(shown[0].time_s, max(shown[-1].time_s, shown[0].time_s + 1.0))
    height_axis.set_ylim(*padded_limits([value for point in shown for value in (point.up_m, point.range_m)]))
    height_axis.plot([point.time_s for point in shown], [point.up_m for point in shown],
                     color="tab:green", alpha=0.35, label="TX AGL (up)")
    height_axis.plot([point.time_s for point in shown], [point.range_m for point in shown],
                     color="tab:purple", alpha=0.35, label="3D range")
    height_cursor = height_axis.axvline(shown[0].time_s, color="black", linewidth=1)
    status = figure.text(0.02, 0.01, "")
    height_axis.legend(loc="best")

    def draw(frame: int):
        point = shown[frame]
        trail.set_data([value.east_m for value in shown[:frame + 1]],
                       [value.north_m for value in shown[:frame + 1]])
        transmitter.set_data([point.east_m], [point.north_m])
        vector.set_data([0.0, point.east_m], [0.0, point.north_m])
        height_cursor.set_xdata([point.time_s, point.time_s])
        status.set_text(
            f"t={point.time_s:.1f}s  seq={point.seq}  N/E/U="
            f"{point.north_m:.1f}/{point.east_m:.1f}/{point.up_m:.1f} m  "
            f"range={point.range_m:.1f} m  "
            f"fixed RX={origin[0] * 1e-7:.7f}, {origin[1] * 1e-7:.7f}")
        return trail, transmitter, vector, height_cursor, status

    replay = animation.FuncAnimation(figure, draw, frames=len(shown), interval=interval_ms,
                                     repeat=True, blit=False)
    if output is not None:
        replay.save(output)
    else:
        plt.show()


def choose_csv_file() -> Path | None:
    """Open a file picker for IDE/F5 launches where no CLI path was supplied."""
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        filename = filedialog.askopenfilename(
            title="Select TRKxxx.CSV copied from the SD card",
            filetypes=(("CSV files", "*.csv *.CSV"), ("All files", "*.*")),
        )
        root.destroy()
        return Path(filename) if filename else None
    except Exception:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path,
                        help="TRKxxx.CSV copied from the SD card")
    parser.add_argument("--fixed-lat-i7", type=int, help="fixed receiver latitude (1e-7 deg)")
    parser.add_argument("--fixed-lon-i7", type=int, help="fixed receiver longitude (1e-7 deg)")
    parser.add_argument("--interval-ms", type=int, default=1,
                        help="animation frame interval in ms (default: 1; 100x faster than the original 50 ms setting)")
    parser.add_argument("--stride", type=int, default=1, help="render every Nth CSV record")
    parser.add_argument("--save", type=Path, help="write animation (.gif or .mp4) instead of opening a window")
    args = parser.parse_args()
    if args.interval_ms <= 0 or args.stride <= 0:
        parser.error("--interval-ms and --stride must be positive")
    csv_path = args.csv or choose_csv_file()
    if csv_path is None:
        parser.print_help()
        print("\nNo CSV selected. Run again with a TRKxxx.CSV path or select it in the file dialog.")
        return
    points, origin = load_points(csv_path, args.fixed_lat_i7, args.fixed_lon_i7)
    animate(points, origin, args.interval_ms, args.stride, args.save)


if __name__ == "__main__":
    main()
