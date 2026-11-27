#!/usr/bin/env bash
# Run OpenVINS on one of our bags and save the estimated trajectory.
#
#   vio/scripts/run_openvins.sh seg_b_undist          (play at normal speed)
#   vio/scripts/run_openvins.sh seg_b_undist 0.5      (play at half speed, easier on the computer)
#
# Optional settings, put them in front of the command:
#   CONFIG_DIR=vio/config/openvins/baseline   use another settings folder (default: high_imu_noise)
#   OUT_NAME=my_test             save to vio/results/my_test instead of the bag's name
#   LIGHT=1                      record only the pose (small files, for quick tests)
# Example:  CONFIG_DIR=vio/config/openvins/baseline OUT_NAME=my_test LIGHT=1 vio/scripts/run_openvins.sh seg_a_short_undist 2.0
#
# Run it from the project folder (ASL_VIO). Output: vio/results/<bag name>/
set -e
BAG_NAME="$1"
RATE="${2:-1.0}"
CONFIG_DIR="${CONFIG_DIR:-vio/config/openvins/high_imu_noise}"
OUT_NAME="${OUT_NAME:-$BAG_NAME}"
[ -d "vio/bags/$BAG_NAME" ] || { echo "No bag vio/bags/$BAG_NAME"; exit 1; }

source /opt/ros/humble/setup.bash
source vio/ws/install/setup.bash

OUT="vio/results/$OUT_NAME"
mkdir -p "$OUT"
rm -rf "$OUT/poses"      # ros2 bag record refuses to write into an existing folder

# Stop everything we started when this script ends (or when you press Ctrl+C).
# The first line stops the launcher programs; the next two stop the programs they started
# (otherwise OpenVINS can keep running in the background after the script ends).
cleanup() {
    kill $(jobs -p) 2>/dev/null || true
    pkill -f '[o]v_msckf/lib/ov_msckf/run_subscribe_msckf' || true
    pkill -f '[i]mage_transport/republish' || true
}
trap cleanup EXIT

# 1) OpenVINS only reads normal images, but our bag holds JPEG-compressed ones.
#    This turns them back into normal images while the bag plays.
ros2 run image_transport republish compressed raw --ros-args \
    -r in/compressed:=/cam0/image_raw/compressed -r out:=/cam0/image_raw > "$OUT/republish.log" 2>&1 &

# 2) OpenVINS itself (one camera, settings from $CONFIG_DIR)
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$PWD/$CONFIG_DIR/estimator_config.yaml" \
    max_cameras:=1 use_stereo:=false verbosity:=INFO > "$OUT/openvins.log" 2>&1 &

# 3) Save what OpenVINS publishes, so it can be replayed in RViz or Foxglove later:
#    the pose and path, the 3D feature points, the image with tracked features, the plain camera
#    image, and the frames (/tf)
if [ "$LIGHT" = "1" ]; then
    TOPICS="/ov_msckf/poseimu"      # quick tests: only the pose, small files
else
    TOPICS="/ov_msckf/poseimu /ov_msckf/pathimu /ov_msckf/odomimu /ov_msckf/points_msckf \
/ov_msckf/points_slam /ov_msckf/trackhist/compressed /cam0/image_raw/compressed /tf /tf_static"
fi
ros2 bag record -o "$OUT/poses" $TOPICS > "$OUT/record.log" 2>&1 &

sleep 5   # give the three programs a moment to start

# 4) Play our recording
ros2 bag play "vio/bags/$BAG_NAME" --rate "$RATE"

sleep 3
echo "Done. Logs and poses are in $OUT"
