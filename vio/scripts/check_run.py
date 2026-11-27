#!/usr/bin/env python3
"""
Check one OpenVINS run against the speed logged by the vehicle.

    source /opt/ros/humble/setup.bash
    python3 vio/scripts/check_run.py seg_a_short_undist seg_c_highnoise        (several runs can be listed)

For each run it prints, every 20 s: OpenVINS's speed, the logged speed, and how many times too
fast OpenVINS is (ratio, should be about 1). A good run stays near 1 and keeps the height near 0.
"""
import csv
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message

FIRST_IMU_TIME = 1791076700.012597869   # recording start; the t_s columns count from here
WINDOW_S = 20.0


def load_poses(run_name):
    """Read the saved poses: rows of (time since start, x, y, z)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f"vio/results/{run_name}/poses", storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/ov_msckf/poseimu"]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append((stamp - FIRST_IMU_TIME, p.x, p.y, p.z))
    return np.array(rows)


def load_logged_speed():
    rows = [r for r in csv.DictReader(open("episode_2/unified.csv")) if r["speed_mps"] != ""]
    return (np.array([float(r["t_s"]) for r in rows]), np.array([float(r["speed_mps"]) for r in rows]))


def main():
    logged_t, logged_speed = load_logged_speed()
    for run in sys.argv[1:]:
        poses = load_poses(run)
        if len(poses) < 2:
            print(f"{run}: no poses saved (OpenVINS never started)")
            continue
        steps = np.linalg.norm(np.diff(poses[:, 1:], axis=0), axis=1)
        print(f"\n=== {run}: first pose at {poses[0, 0]:.0f} s, last at {poses[-1, 0]:.0f} s")
        print("  window (s)    OpenVINS m/s   logged m/s   ratio   height (m)")
        t = poses[0, 0]
        while t + WINDOW_S <= poses[-1, 0]:
            a = np.searchsorted(poses[:, 0], t)
            b = np.searchsorted(poses[:, 0], t + WINDOW_S) - 1
            speed = np.linalg.norm(poses[b, 1:] - poses[a, 1:]) / (poses[b, 0] - poses[a, 0])
            inside = (logged_t >= t) & (logged_t <= t + WINDOW_S)
            true_speed = logged_speed[inside].mean() if inside.any() else float("nan")
            ratio = speed / true_speed if true_speed > 0.05 else float("nan")
            print(f"  {t:4.0f} - {t + WINDOW_S:4.0f}   {speed:9.2f}      {true_speed:7.2f}    {ratio:5.2f}   {poses[b, 3]:8.1f}")
            t += WINDOW_S
        expected = 0.41 * (poses[-1, 0] - poses[0, 0])
        print(f"  path length {steps.sum():.0f} m  (at most about {expected:.0f} m is possible at 0.41 m/s)   "
              f"height range {poses[:, 3].min():.1f} to {poses[:, 3].max():.1f} m")


if __name__ == "__main__":
    main()
