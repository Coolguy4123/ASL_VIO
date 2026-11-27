#!/usr/bin/env python3
"""
List the times when the vehicle stood still: good places to START an OpenVINS run.
(OpenVINS starts best from a standstill, see vio/README.MD.)

    python3 vio/scripts/find_stops.py          stops of at least 5 s
    python3 vio/scripts/find_stops.py 20       stops of at least 20 s

It reads episode_2/unified.csv: a frame counts as "standing still" if the logged speed is below
0.02 m/s and the IMU has data there. Times are seconds since the start of the recording.
Pick a start a little before the stop (or at its start) and use it with make_bag.py --start.
"""
import csv
import sys

MIN_LENGTH_S = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
STILL_SPEED_MPS = 0.02
MAX_GAP_S = 1.0       # a longer gap between frames ends a stop (for example the IMU hole)

frames = []
for row in csv.DictReader(open("episode_2/unified.csv")):
    if row["speed_mps"] == "" or row["imu_valid"] != "True":
        frames.append((float(row["t_s"]), False))
    else:
        frames.append((float(row["t_s"]), float(row["speed_mps"]) < STILL_SPEED_MPS))

stops, start, last = [], None, None
for t, still in frames:
    if still and start is not None and t - last <= MAX_GAP_S:
        last = t
    else:
        if start is not None:
            stops.append((start, last))
        start, last = (t, t) if still else (None, None)
if start is not None:
    stops.append((start, last))

print(f"Stops of at least {MIN_LENGTH_S:.0f} s (start, end, length in seconds):")
for start, end in stops:
    if end - start >= MIN_LENGTH_S:
        print(f"  {start:6.1f}  {end:6.1f}  {end - start:5.1f} s")
