#!/usr/bin/env python3
"""
Make one bag for Foxglove that shows the OpenVINS path AND the GNSS "ground truth" path together.

Foxglove cannot draw a CSV in its 3D panel, so we put the GNSS path into a copy of the results bag
as a normal path message (topic /ground_truth/path).

The two paths use different coordinate systems: OpenVINS starts at (0, 0) facing an arbitrary
direction, GNSS is east/north. So we rotate and shift the GNSS path to lie as close as possible
to the OpenVINS path. We do NOT scale it, so a wrong OpenVINS scale stays visible.

Steps:
    python3 vio/scripts/make_ground_truth.py                      (once: makes the GNSS csv)
    source /opt/ros/humble/setup.bash
    python3 vio/scripts/make_foxglove_bag.py seg_c_highnoise

Output: vio/results/<run>/foxglove/   (open foxglove/foxglove_0.db3 in Foxglove)
In the 3D panel (display frame "global") turn on /ov_msckf/pathimu and /ground_truth/path.
"""
import csv
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.serialization import deserialize_message, serialize_message

FIRST_IMU_TIME = 1791076700.012597869   # recording start; "t_s" in the csv files counts from here
REPEAT_EVERY_S = 5.0                    # the path is written again every 5 s, so it shows after any seek


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    return reader


def rigid_fit(a, b):
    """Rotation r and shift s so that  r @ a + s  is as close as possible to b (no scaling)."""
    mean_a, mean_b = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((b - mean_b).T @ (a - mean_a))
    r = u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    return r, mean_b - r @ mean_a


def make_path_message(xy, stamp_ns):
    msg = PathMsg()
    msg.header.frame_id = "global"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    for x, y in xy:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def main():
    run = sys.argv[1]
    results = Path(f"vio/results/{run}")
    out_dir = results / "foxglove"
    if out_dir.exists():
        raise SystemExit(f"{out_dir} already exists - delete it first if you want to make it again.")

    # ---- 1) First pass: OpenVINS positions, and the time range of the bag ---------------------
    reader = open_reader(results / "poses")
    topics = reader.get_all_topics_and_types()
    ov_time, ov_xy, first_ns, last_ns = [], [], None, None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        first_ns = t_ns if first_ns is None else first_ns
        last_ns = t_ns
        if topic == "/ov_msckf/poseimu":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            ov_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - FIRST_IMU_TIME)
            ov_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    ov_time, ov_xy = np.array(ov_time), np.array(ov_xy)

    # ---- 2) The GNSS ground truth, cut to the time OpenVINS was running --------------------------
    rows = list(csv.DictReader(open("vio/results/ground_truth_gnss.csv")))
    gt_t = np.array([float(r["t_s"]) for r in rows])
    gt_en = np.array([(float(r["east_m"]), float(r["north_m"])) for r in rows])
    inside = (gt_t >= ov_time[0]) & (gt_t <= ov_time[-1])
    gt_t, gt_en = gt_t[inside], gt_en[inside]

    # ---- 3) Rotate and shift the GNSS path into OpenVINS's frame ---------------------------------
    ov_at_gt = np.column_stack([np.interp(gt_t, ov_time, ov_xy[:, 0]), np.interp(gt_t, ov_time, ov_xy[:, 1])])
    r, s = rigid_fit(ov_at_gt, gt_en)                  # OpenVINS -> GNSS
    gt_in_ov_frame = (r.T @ (gt_en - s).T).T           # GNSS -> OpenVINS frame
    error = np.hypot(*(ov_at_gt - gt_in_ov_frame).T)
    ov_len = np.hypot(np.diff(ov_xy[:, 0]), np.diff(ov_xy[:, 1])).sum()
    gt_len = np.hypot(np.diff(gt_in_ov_frame[:, 0]), np.diff(gt_in_ov_frame[:, 1])).sum()
    print(f"OpenVINS path {ov_len:.0f} m, GNSS path {gt_len:.0f} m, average gap after lining up {error.mean():.1f} m")

    # ---- 4) Second pass: copy everything, and add the ground-truth path -------------------------
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out_dir), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    for topic in topics:
        writer.create_topic(topic)
    writer.create_topic(rosbag2_py.TopicMetadata("/ground_truth/path", "nav_msgs/msg/Path", "cdr", ""))

    gt_times = list(range(first_ns, last_ns + 1, int(REPEAT_EVERY_S * 1e9)))   # when to write the path
    next_gt = 0
    reader = open_reader(results / "poses")
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        while next_gt < len(gt_times) and gt_times[next_gt] <= t_ns:
            writer.write("/ground_truth/path", serialize_message(make_path_message(gt_in_ov_frame, gt_times[next_gt])), gt_times[next_gt])
            next_gt += 1
        writer.write(topic, data, t_ns)
    print(f"Saved {out_dir}  (open foxglove_0.db3 in Foxglove)")


if __name__ == "__main__":
    main()
