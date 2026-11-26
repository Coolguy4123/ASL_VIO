# How to run

This folder runs **OpenVINS** (a visual-inertial odometry program) on our recording, `episode_2`. It reads the camera images and the IMU, and estimates where the vehicle went.

Status: it runs, but the path is **only reliable for the short `seg_b` test**. On the long `seg_a` drive it goes wrong (see "Results").

## What is in this folder

| Folder / file | What it is |
|---|---|
| `make_bag.py` | Turns the CSV logs + images into a ROS 2 bag |
| `run_openvins.sh` | Plays a bag through OpenVINS and saves the results |
| `config/` | OpenVINS settings (IMU noise, camera numbers, start-up) |
| `bags/` | The recordings, cut into pieces (see next table) |
| `results/<bag name>/` | Output of each run: `openvins.log` and `poses/` (a bag for Foxglove) |
| `ws/` | OpenVINS itself (downloaded and built here) |
| `../rgb_intrinsics_640x480.yaml` | Estimated camera numbers for the 640x480 images |

## Which bag to use

The IMU, GNSS and speed have a **232 s hole (591 to 823 s)**, so the recording is cut in two. Always use the `_undist` bags: they have the lens distortion removed and the image time moved by -0.425 s.

| Bag | Covers | Use |
|---|---|---|
| `seg_a_undist` | 30 to 590 s | Long drive. Starts while already moving, which is hard for OpenVINS. |
| `seg_b_undist` | 823 to 979 s | Short, mostly parked. Good first test. |
| `seg_c_undist` | 427 to 590 s | Starts parked, then drives for about 2 minutes. Better test, not run yet. |

## Steps

Run everything from the project folder (`ASL_VIO`).

**1. Make the bags (once).** Needs `episode_2/imu_raw.csv` and `episode_2/unified.csv`, which `DataPreprocess.ipynb` creates.
```bash
source /opt/ros/humble/setup.bash
python3 vio/make_bag.py --start 823 --end 980 --out vio/bags/seg_b_undist \
    --image-time-shift -0.425 --undistort-with rgb_intrinsics_640x480.yaml
```

**2. Run OpenVINS.** It plays the bag at normal speed, so it takes as long as the bag (about 3 minutes for `seg_b`).
```bash
vio/run_openvins.sh seg_b_undist        # add 0.5 at the end for half speed
```

**3. Check it finished.** Nothing printed means nothing is running.
```bash
pgrep -af "[r]un_subscribe_msckf|[r]os2 (bag|launch|run)|[i]mage_transport/republish"
```

**4. Check the result.** Prints the estimated position and distance travelled. The vehicle drives about 0.41 m/s, so 560 s should give at most about 230 m, and the height (3rd number) should stay near 0.
```bash
grep -o "p_IinG = [^|]*| dist = [0-9.]*" vio/results/seg_a_undist/openvins.log | awk 'NR%300==1'
```

**5. Look at it.**
- Live in RViz (start it first, then run step 2):
  ```bash
  source /opt/ros/humble/setup.bash
  rviz2 -d vio/ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
  ```
- Afterwards in Foxglove: open `vio/results/<bag name>/poses/poses_0.db3`, then add panels:

| Panel | Topic |
|---|---|
| Image (plain camera) | `/cam0/image_raw/compressed` |
| Image (tracked corners, "vo image") | `/ov_msckf/trackhist/compressed` |
| 3D, display frame `global` | `/ov_msckf/pathimu`, `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` (skip `poseimu` and `odomimu`: their timestamps don't match `/tf`) |

In the 3D panel the yellow line is just the `global` to `imu` transform link. The purple line is the estimated path.