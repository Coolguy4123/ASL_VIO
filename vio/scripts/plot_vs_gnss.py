#!/usr/bin/env python3
"""
Compare one OpenVINS run with the GNSS ground truth: a picture and a few numbers.
(run_openvins.sh calls this automatically at the end of every run.)

    source /opt/ros/humble/setup.bash
    python3 vio/scripts/plot_vs_gnss.py seg_c_best

Needs vio/results/ground_truth_gnss.csv (made by make_ground_truth.py).

Output, in vio/results/<run>/:
    vs_gnss.png   left: OpenVINS path and GNSS track.  right: distance between them over time.
    metrics.txt   the same numbers as text

The OpenVINS path is only rotated and shifted onto the GNSS track, NOT scaled, so a wrong
scale (a path that is too long) stays visible. "Best scale" tells you the number that would fix it.
GNSS is ordinary GPS (about 2 to 3 m accurate), so gaps below that mean little.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message

FIRST_IMU_TIME = 1791076700.012597869   # recording start; "t_s" in the csv files counts from here


def load_openvins_path(run):
    """OpenVINS positions: arrays of time (s since start), x and y (m)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f"vio/results/{run}/poses", storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/ov_msckf/poseimu"]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append((stamp - FIRST_IMU_TIME, msg.pose.pose.position.x, msg.pose.pose.position.y))
    return np.array(rows)


def align(path, target, allow_scale):
    """Rotate and shift `path` onto `target`. With allow_scale it is also stretched. Returns (result, scale)."""
    mean_p, mean_t = path.mean(axis=0), target.mean(axis=0)
    u, singular, vt = np.linalg.svd((target - mean_t).T @ (path - mean_p))
    flip = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    rotation = u @ flip @ vt
    scale = (singular * np.diag(flip)).sum() / ((path - mean_p) ** 2).sum() if allow_scale else 1.0
    return scale * (rotation @ (path - mean_p).T).T + mean_t, scale


def length(xy):
    return float(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])).sum())


def main():
    run = sys.argv[1]
    out_dir = Path(f"vio/results/{run}")

    poses = load_openvins_path(run)
    if len(poses) < 10:
        raise SystemExit(f"{run}: OpenVINS saved only {len(poses)} poses, nothing to compare.")

    rows = list(csv.DictReader(open("vio/results/ground_truth_gnss.csv")))
    gnss_t = np.array([float(r["t_s"]) for r in rows])
    gnss_xy = np.array([(float(r["east_m"]), float(r["north_m"])) for r in rows])
    inside = (gnss_t >= poses[0, 0]) & (gnss_t <= poses[-1, 0])      # only while OpenVINS was running
    gnss_t, gnss_xy = gnss_t[inside], gnss_xy[inside]

    # OpenVINS position at the moments of the GNSS fixes, then line it up with GNSS (no stretching)
    at_gnss = np.column_stack([np.interp(gnss_t, poses[:, 0], poses[:, 1]), np.interp(gnss_t, poses[:, 0], poses[:, 2])])
    lined_up, _ = align(at_gnss, gnss_xy, allow_scale=False)
    gap = np.hypot(*(lined_up - gnss_xy).T)
    _, best_scale = align(at_gnss, gnss_xy, allow_scale=True)
    fixed, _ = align(at_gnss, gnss_xy, allow_scale=True)
    gap_fixed = np.hypot(*(fixed - gnss_xy).T)

    # The whole OpenVINS path, moved the same way (for drawing the line)
    mean_a, mean_g = at_gnss.mean(axis=0), gnss_xy.mean(axis=0)
    u, _, vt = np.linalg.svd((gnss_xy - mean_g).T @ (at_gnss - mean_a))
    rotation = u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    full_path = (rotation @ (poses[:, 1:3] - mean_a).T).T + mean_g

    vio_len, gnss_len = length(full_path), length(gnss_xy)
    rms, mean_gap, worst = float(np.sqrt((gap ** 2).mean())), float(gap.mean()), float(gap.max())
    text = (f"run: {run}\n"
            f"time range: {poses[0, 0]:.0f} s to {poses[-1, 0]:.0f} s ({poses[-1, 0] - poses[0, 0]:.0f} s)\n"
            f"path length: OpenVINS {vio_len:.0f} m, GNSS {gnss_len:.0f} m (ratio {vio_len / gnss_len:.2f})\n"
            f"distance to GNSS (rotated and shifted, not scaled): average {mean_gap:.1f} m, RMS {rms:.1f} m, worst {worst:.1f} m, at the end {gap[-1]:.1f} m\n"
            f"best scale factor: {best_scale:.2f}  (error would drop to {np.sqrt((gap_fixed ** 2).mean()):.1f} m RMS)")
    (out_dir / "metrics.txt").write_text(text + "\n")
    print(text)

    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 5))
    left.plot(gnss_xy[:, 0], gnss_xy[:, 1], ".", color="gray", markersize=4, label=f"GNSS ({gnss_len:.0f} m)")
    left.plot(full_path[:, 0], full_path[:, 1], color="tab:purple", lw=1.8, label=f"OpenVINS ({vio_len:.0f} m)")
    left.plot(*gnss_xy[0], "go", label="start")
    left.set_aspect("equal", adjustable="datalim")
    left.set_xlabel("east [m]")
    left.set_ylabel("north [m]")
    left.set_title(f"{run}: OpenVINS path vs GNSS")
    left.legend()
    left.grid(True)
    right.plot(gnss_t, gap, color="tab:red")
    right.set_xlabel("time since start of recording [s]")
    right.set_ylabel("distance to GNSS [m]")
    right.set_title(f"Error over time (average {mean_gap:.1f} m, RMS {rms:.1f} m)")
    right.grid(True)
    right.text(0.02, 0.97, f"path ratio {vio_len / gnss_len:.2f}x, best scale {best_scale:.2f}\nGNSS itself is about 2 to 3 m accurate",
               transform=right.transAxes, va="top", fontsize=9, bbox=dict(facecolor="white", alpha=0.8))
    fig.tight_layout()
    fig.savefig(out_dir / "vs_gnss.png", dpi=100)
    print(f"saved {out_dir}/vs_gnss.png and metrics.txt")


if __name__ == "__main__":
    main()
