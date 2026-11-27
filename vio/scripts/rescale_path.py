#!/usr/bin/env python3
"""
Fix the SCALE of an OpenVINS path, using the vehicle's logged speed.

Why: on episode_2 OpenVINS gets the direction the vehicle faces about right, but thinks it
moves 3 to 4 times too fast (see vio/README.MD). So we keep OpenVINS's HEADING and replace
its distance with the logged speed (wheel odometry):

    every 10 ms:  position += logged_speed * 0.01 s * (direction OpenVINS says we face)

NOTE: this is no longer pure visual-inertial odometry. OpenVINS's own positions are not used,
only its heading. A fair comparison shows the gyro alone gives almost the same heading.

Run (needs ROS for reading the bag):
    source /opt/ros/humble/setup.bash
    python3 vio/scripts/rescale_path.py seg_a_undist

Output, in vio/results/<run>_corrected/:
    trajectory.csv   t_s, x_m, y_m (OpenVINS frame, start = 0,0), east_m, north_m (lined up with GNSS)
    plot.png         the corrected path next to the GNSS track, and the error over time
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

SPEED_LAG_S = 0.15     # the logged speed is 0.15 s behind the IMU (found in Step 3 of the notebook)
STEP_S = 0.01          # we move the vehicle forward in 10 ms steps
EARTH_RADIUS_M = 6_371_000


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_openvins_poses(run_name):
    """OpenVINS poses: arrays of time (s) and the quaternion (x, y, z, w) of the IMU in the global frame."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f"vio/results/{run_name}/poses", storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/ov_msckf/poseimu"]))
    times, quats = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        q = msg.pose.pose.orientation
        times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        quats.append((q.x, q.y, q.z, q.w))
    return np.array(times), np.array(quats)


def align_to(path, target):
    """Rotate and shift `path` (no scaling) so it lies as close as possible to `target`."""
    mean_p, mean_t = path.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((target - mean_t).T @ (path - mean_p))
    flip = np.sign(np.linalg.det(u @ vt))
    rotation = u @ np.diag([1.0, flip]) @ vt
    return (rotation @ (path - mean_p).T).T + mean_t


def main():
    run = sys.argv[1]
    out_dir = Path(f"vio/results/{run}_corrected")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- 1) Read the inputs --------------------------------------------------------------
    imu_rows = read_csv("episode_2/imu_raw.csv")
    start_time_s = int(imu_rows[0]["t_ns"]) * 1e-9              # t_s = 0 is the first IMU sample
    pose_times, quats = load_openvins_poses(run)
    pose_times = pose_times - start_time_s

    frames = [r for r in read_csv("episode_2/unified.csv") if r["speed_mps"] != ""]
    speed_t = np.array([float(r["t_s"]) for r in frames])
    speed = np.array([float(r["speed_mps"]) for r in frames])

    # ---- 2) Which time range can we use? --------------------------------------------------
    # From OpenVINS's first pose until the logged speed has a hole (the 232 s gap).
    gaps = np.where(np.diff(speed_t) > 2.0)[0]
    last_speed_time = speed_t[gaps[0]] if len(gaps) else speed_t[-1]
    t_start, t_end = pose_times[0], min(pose_times[-1], last_speed_time - 1.0)
    t = np.arange(t_start, t_end, STEP_S)

    # ---- 3) Heading from OpenVINS's orientation --------------------------------------------
    # The vehicle drives along the IMU's x axis. Take that axis, as seen in OpenVINS's global frame,
    # and turn it into an angle on the ground.
    qx, qy, qz, qw = quats.T
    forward_x = 1 - 2 * (qy * qy + qz * qz)
    forward_y = 2 * (qx * qy + qz * qw)
    heading_at_poses = np.unwrap(np.arctan2(forward_y, forward_x))
    heading = np.interp(t, pose_times, heading_at_poses)

    # ---- 4) Move forward at the logged speed -----------------------------------------------
    speed_now = np.interp(t + SPEED_LAG_S, speed_t, speed)
    x = np.cumsum(speed_now * np.cos(heading) * STEP_S)
    y = np.cumsum(speed_now * np.sin(heading) * STEP_S)
    path = np.column_stack([x, y])

    # ---- 5) Line it up with the GNSS track to see how good it is -----------------------------
    fixes = [r for r in read_csv("episode_2/gnss_log.csv") if int(r["status"]) >= 0]
    gnss_t = np.array([int(r["timestamp_ns"]) * 1e-9 - start_time_s for r in fixes])
    lat = np.radians([float(r["latitude"]) for r in fixes])
    lon = np.radians([float(r["longitude"]) for r in fixes])
    gnss_east = (lon - lon[0]) * EARTH_RADIUS_M * np.cos(lat[0])
    gnss_north = (lat - lat[0]) * EARTH_RADIUS_M
    inside = (gnss_t >= t_start) & (gnss_t <= t_end)
    gnss = np.column_stack([gnss_east[inside], gnss_north[inside]])
    gnss_times = gnss_t[inside]

    path_at_gnss = np.column_stack([np.interp(gnss_times, t, x), np.interp(gnss_times, t, y)])
    lined_up = align_to(path_at_gnss, gnss)
    error = np.hypot(*(lined_up - gnss).T)

    # The same shift and rotation, applied to the whole path (for the east/north columns).
    mean_p, mean_g = path_at_gnss.mean(axis=0), gnss.mean(axis=0)
    u, _, vt = np.linalg.svd((gnss - mean_g).T @ (path_at_gnss - mean_p))
    rotation = u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    east_north = (rotation @ (path - mean_p).T).T + mean_g

    print(f"Time range used: {t_start:.0f} s to {t_end:.0f} s")
    print(f"Corrected path length: {np.hypot(np.diff(x), np.diff(y)).sum():.0f} m   "
          f"(GNSS track: {np.hypot(np.diff(gnss[:, 0]), np.diff(gnss[:, 1])).sum():.0f} m, noisy)")
    print(f"Error against GNSS: average {np.sqrt((error ** 2).mean()):.1f} m, worst {error.max():.1f} m, at the end {error[-1]:.1f} m")

    # ---- 6) Save ----------------------------------------------------------------------------
    keep = slice(0, None, 10)                                   # every 10th row = 10 rows per second
    with open(out_dir / "trajectory.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "x_m", "y_m", "east_m", "north_m"])
        for row in zip(t[keep], x[keep], y[keep], east_north[keep, 0], east_north[keep, 1]):
            writer.writerow([f"{v:.3f}" for v in row])

    # East/north of the corrected path at every 10 ms step, for drawing the full line
    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 5))
    left.plot(gnss[:, 0], gnss[:, 1], ".", color="gray", markersize=3, label="GNSS (noisy)")
    left.plot(east_north[:, 0], east_north[:, 1], color="tab:purple", lw=1.5, label="corrected OpenVINS path")
    left.set_aspect("equal", adjustable="datalim")
    left.set_xlabel("east [m]")
    left.set_ylabel("north [m]")
    left.set_title("Corrected path vs GNSS")
    left.legend()
    right.plot(gnss_times, error)
    right.set_xlabel("time since start [s]")
    right.set_ylabel("distance to GNSS [m]")
    right.set_title("Error over time (GNSS itself is about 1 m noisy)")
    right.grid(True)
    left.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "plot.png", dpi=100)
    print(f"Saved {out_dir}/trajectory.csv and plot.png")


if __name__ == "__main__":
    main()
