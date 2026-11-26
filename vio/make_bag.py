#!/usr/bin/env python3
"""
Turn the episode CSV logs + JPEG images into a ROS 2 bag that OpenVINS can play.

Run it with the system Python after sourcing ROS 2 Humble (the project .venv has no ROS):

    source /opt/ros/humble/setup.bash
    python3 vio/make_bag.py --start 30 --end 590 --out vio/bags/seg_a

For OpenVINS we also correct the image time and remove the lens distortion:

    python3 vio/make_bag.py --start 30 --end 590 --out vio/bags/seg_a_undist \\
        --image-time-shift -0.425 --undistort-with rgb_intrinsics_640x480.yaml

Inputs  (made by DataPreprocess.ipynb):
    episode_2/imu_raw.csv   every IMU sample (100 Hz)
    episode_2/unified.csv   one row per camera frame (gives each frame's time and image path)
Output  (a new folder; the script refuses to overwrite an existing one):
    /imu0                      sensor_msgs/Imu
    /cam0/image_raw/compressed sensor_msgs/CompressedImage   (the original JPEG bytes, untouched)
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage, Imu

DATA_DIR = Path("episode_2")
JPEG_QUALITY = 95   # quality used when we re-save undistorted images


def make_undistorter(intrinsics_path):
    """
    Build a function that removes the lens distortion from an image.

    The result is a plain pinhole image with the SAME size and the SAME fx, fy, cx, cy as in
    the yaml file, so OpenVINS can be given those four numbers and zero distortion.
    """
    with open(intrinsics_path) as f:
        c = yaml.safe_load(f)
    camera_matrix = np.array([[c["fx"], 0.0, c["cx"]], [0.0, c["fy"], c["cy"]], [0.0, 0.0, 1.0]])
    distortion = np.array(c["distortion_coeffs"], dtype=float)
    size = (int(c["image_width"]), int(c["image_height"]))
    # For every pixel of the NEW image, this map says which pixel of the OLD image to copy.
    map_x, map_y = cv2.initUndistortRectifyMap(camera_matrix, distortion, None, camera_matrix, size, cv2.CV_32FC1)

    def undistort(image):
        return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)

    return undistort


def to_stamp(t_ns):
    """Nanoseconds (int) -> ROS time message."""
    return Time(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000)


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def make_imu_message(row):
    msg = Imu()
    msg.header.stamp = to_stamp(int(row["t_ns"]))
    msg.header.frame_id = "imu0"
    # We do not give OpenVINS the IMU's own orientation estimate. ROS convention for
    # "no orientation": first covariance entry = -1.
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x = float(row["angular_velocity_x"])
    msg.angular_velocity.y = float(row["angular_velocity_y"])
    msg.angular_velocity.z = float(row["angular_velocity_z"])
    msg.linear_acceleration.x = float(row["linear_acceleration_x"])
    msg.linear_acceleration.y = float(row["linear_acceleration_y"])
    msg.linear_acceleration.z = float(row["linear_acceleration_z"])
    return msg


def make_image_message(row, stamp_ns, undistort=None):
    msg = CompressedImage()
    msg.header.stamp = to_stamp(stamp_ns)
    msg.header.frame_id = "cam0"
    msg.format = "jpeg"
    path = DATA_DIR / row["rgb_image"]
    if undistort is None:
        msg.data = path.read_bytes()   # the JPEG file as-is
    else:
        image = undistort(cv2.imread(str(path)))
        _, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        msg.data = jpeg.tobytes()
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--start", type=float, required=True, help="first second to include (t_s in the CSVs)")
    parser.add_argument("--end", type=float, required=True, help="last second to include")
    parser.add_argument("--out", required=True, help="new bag folder to create")
    parser.add_argument("--image-time-shift", type=float, default=0.0,
                        help="seconds added to every image timestamp (we measured -0.425: "
                             "the images were stamped about 0.425 s after they were taken)")
    parser.add_argument("--undistort-with", default=None, metavar="YAML",
                        help="intrinsics yaml; if given, lens distortion is removed from every image")
    args = parser.parse_args()

    shift_ns = round(args.image_time_shift * 1e9)
    undistort = make_undistorter(args.undistort_with) if args.undistort_with else None

    if Path(args.out).exists():
        raise SystemExit(f"{args.out} already exists - choose another name or remove it yourself.")

    imu_rows = [r for r in read_rows(DATA_DIR / "imu_raw.csv") if args.start <= float(r["t_s"]) <= args.end]
    frame_rows = [r for r in read_rows(DATA_DIR / "unified.csv") if args.start <= float(r["t_s"]) <= args.end]
    print(f"IMU samples: {len(imu_rows)}   camera frames: {len(frame_rows)}")

    # Put everything in one list sorted by time, so the bag is written in time order.
    events = [(int(r["t_ns"]), "/imu0", r) for r in imu_rows]
    events += [(int(r["t_ns"]) + shift_ns, "/cam0/image_raw/compressed", r) for r in frame_rows]
    events.sort(key=lambda e: e[0])

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.out, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata("/imu0", "sensor_msgs/msg/Imu", "cdr", ""))
    writer.create_topic(rosbag2_py.TopicMetadata("/cam0/image_raw/compressed", "sensor_msgs/msg/CompressedImage", "cdr", ""))

    for t_ns, topic, row in events:
        message = make_imu_message(row) if topic == "/imu0" else make_image_message(row, t_ns, undistort)
        writer.write(topic, serialize_message(message), t_ns)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
