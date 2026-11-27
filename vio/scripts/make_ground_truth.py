#!/usr/bin/env python3
"""
Make a "ground truth" path for episode_2 from the GNSS log.

Honest warning: this is ordinary GPS, not a survey-grade reference. It is smooth over a few
seconds (steps between fixes are about 0.1 m) but its absolute position can be off by 2 to 3 m,
and it has no heading. Use it to judge the SHAPE and the length of a path, not centimetres.

    python3 vio/scripts/make_ground_truth.py

Output: vio/results/ground_truth_gnss.csv
    t_s        seconds since the first IMU sample (the same time axis as every other file)
    east_m     metres east of the first GNSS fix
    north_m    metres north of the first GNSS fix
    std_m      horizontal accuracy the receiver itself reports (metres)
               (fixes the receiver itself calls worse than 10 m are left out)
    jump       1 if this fix jumped away from its neighbours (a bad fix). It is already
               replaced by the median of its neighbours in east_m / north_m.
"""
import csv
from pathlib import Path

import numpy as np

EARTH_RADIUS_M = 6_371_000
MAX_STD_M = 10.0      # drop fixes whose own reported accuracy is worse than this (the first few are like that)
JUMP_M = 3.0          # a fix this far from the median of its neighbours counts as a bad fix
WINDOW = 7            # median over 7 fixes (about 1.75 s)


def moving_median(values, window):
    half = window // 2
    padded = np.pad(values, half, mode="edge")
    return np.array([np.median(padded[i:i + window]) for i in range(len(values))])


def main():
    start_ns = int(next(csv.DictReader(open("episode_2/imu_raw.csv")))["t_ns"])   # t_s = 0
    all_rows = [r for r in csv.DictReader(open("episode_2/gnss_log.csv")) if int(r["status"]) >= 0]   # fixes only
    rows = [r for r in all_rows if float(r["position_covariance_0"]) ** 0.5 <= MAX_STD_M]            # and accurate enough
    dropped = len(all_rows) - len(rows)

    t = np.array([(int(r["timestamp_ns"]) - start_ns) * 1e-9 for r in rows])
    lat = np.radians([float(r["latitude"]) for r in rows])
    lon = np.radians([float(r["longitude"]) for r in rows])
    std = np.sqrt([float(r["position_covariance_0"]) for r in rows])

    east = (lon - lon[0]) * EARTH_RADIUS_M * np.cos(lat[0])
    north = (lat - lat[0]) * EARTH_RADIUS_M

    # Find bad fixes: far from the median of the fixes around them. Replace them by that median.
    east_med, north_med = moving_median(east, WINDOW), moving_median(north, WINDOW)
    jump = np.hypot(east - east_med, north - north_med) > JUMP_M
    east, north = np.where(jump, east_med, east), np.where(jump, north_med, north)

    out = Path("vio/results/ground_truth_gnss.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "east_m", "north_m", "std_m", "jump"])
        for row in zip(t, east, north, std, jump.astype(int)):
            writer.writerow([f"{row[0]:.3f}", f"{row[1]:.3f}", f"{row[2]:.3f}", f"{min(row[3], 99):.2f}", row[4]])

    length = np.hypot(np.diff(east), np.diff(north))[np.diff(t) < 1.0].sum()
    print(f"{len(rows)} fixes written to {out}  ({dropped} dropped because the receiver reported an accuracy worse than {MAX_STD_M:.0f} m)")
    print(f"bad fixes replaced: {jump.sum()}   total path length (without the data hole): {length:.0f} m")
    print(f"receiver's own accuracy: median {np.median(std):.1f} m, 90% below {np.percentile(std, 90):.1f} m")


if __name__ == "__main__":
    main()
