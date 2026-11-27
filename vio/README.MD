# How to run

This folder runs **OpenVINS** (a visual-inertial odometry program) on our recording, `episode_2`. It reads the camera images and the IMU, and estimates where the vehicle went.

Status: it works only partly. On the long `seg_a` drive OpenVINS gets the **scale wrong** and later drifts away. The best pure-VIO run is `seg_c` with the settings in `config/openvins/high_imu_noise` (path 1.5 times too long, 4.9 m error against GNSS over 2 minutes).

## Folders

| Folder | What is in it |
|---|---|
| `config/` | Settings: `camera/` (calibration), `openvins/` (OpenVINS settings, one folder per version) |
| `scripts/` | All the scripts (make bags, run OpenVINS, check, ground truth, fix scale) |
| `bags/` | The recordings OpenVINS plays |
| `results/` | One folder per run, plus the ground truth |
| `ws/` | OpenVINS itself (downloaded and built here) |

Every folder has its own small README.

## Which bag to use

The IMU, GNSS and speed have a **232 s hole (591 to 823 s)**, so the recording is cut there. Always use the `_undist` bags (lens distortion removed, image time moved by -0.425 s).

| Bag | Covers | Use |
|---|---|---|
| `seg_a_undist` | 30 to 590 s | The long drive. About 10 minutes to run. Starts while moving, which is hard. |
| `seg_a_short_undist` | 30 to 260 s | First part of it, for quick tests |
| `seg_c_undist` | 427 to 590 s | Parked for 40 s, then drives. The best test. About 3 minutes. |

## Steps

Run everything from the project folder (`ASL_VIO`).

**1. Make a bag (once).** Needs `episode_2/imu_raw.csv` and `episode_2/unified.csv`, which `DataPreprocess.ipynb` creates.
```bash
source /opt/ros/humble/setup.bash
python3 vio/scripts/make_bag.py --start 427 --end 590 --out vio/bags/seg_c_undist \
    --image-time-shift -0.425 --undistort-with vio/config/camera/rgb_intrinsics_640x480.yaml
```

**2. Run OpenVINS.** It plays the bag at normal speed, so it takes as long as the bag.
```bash
vio/scripts/run_openvins.sh seg_c_undist          # uses config/openvins/high_imu_noise
# other settings, different name for the results, small files:
CONFIG_DIR=vio/config/openvins/baseline OUT_NAME=my_test LIGHT=1 vio/scripts/run_openvins.sh seg_c_undist
```

**3. Check it finished.** Nothing printed means nothing is running.
```bash
pgrep -af "[r]un_subscribe_msckf|[r]os2 (bag|launch|run)|[i]mage_transport/republish"
```

**4. Check the result.** Compares OpenVINS's speed with the logged speed (about 0.41 m/s). A good run has a ratio near 1 and a height near 0.
```bash
source /opt/ros/humble/setup.bash
python3 vio/scripts/check_run.py seg_c_undist
```

**5. Look at it.**
- Live in RViz (start it first, then run step 2):
  ```bash
  source /opt/ros/humble/setup.bash
  rviz2 -d vio/ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
  ```
- Afterwards in Foxglove, open `vio/results/<run>/poses/poses_0.db3` and add panels:

| Panel | Topic |
|---|---|
| Image (plain camera) | `/cam0/image_raw/compressed` |
| Image (tracked corners) | `/ov_msckf/trackhist/compressed` |
| 3D, display frame `global` | `/ov_msckf/pathimu`, `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` (skip `poseimu` and `odomimu`: their timestamps don't match `/tf`) |

- To see the VIO path **and the ground truth** together:
  ```bash
  python3 vio/scripts/make_ground_truth.py                  # once
  python3 vio/scripts/make_foxglove_bag.py seg_c_highnoise  # needs a run saved without LIGHT=1
  ```
  Then open `vio/results/seg_c_highnoise/foxglove/foxglove_0.db3` and turn on `/ov_msckf/pathimu` and `/ground_truth/path` in the 3D panel.

**6. Fix the scale (not pure VIO).** Keeps only OpenVINS's heading and moves at the logged speed:
```bash
python3 vio/scripts/rescale_path.py seg_a_undist
```
It writes `vio/results/seg_a_undist_corrected/trajectory.csv` and `plot.png`.

## Results so far

`seg_c`, 122 s of driving, true path 44 m (GNSS). Each is one run.

| Run (settings folder) | Path length | Error vs GNSS | Height drift |
|---|---|---|---|
| `high_imu_noise` | **65 m (1.5x)** | **4.9 m** | under 1 m |
| `baseline` (IMU noise too low) | 199 m (4.5x) | 31.8 m | up to 22 m |
| `ground_corners` | 520 m (11.7x) | 87.3 m | down to -48 m |

`seg_a`, full drive, true path about 195 m:

| | Path length | Error vs GNSS |
|---|---|---|
| OpenVINS as it is | 5061 m | 802 m on average |
| Scale fixed with the logged speed (step 6) | 177 m | 1.5 m average, 3.9 m worst |

The scale-fixed path is **not pure VIO**: only OpenVINS's heading is used, and the gyro alone gives almost the same (1.6 m).

## Known problems

| Problem | Detail |
|---|---|
| IMU noise | The IMU vibrates about 10 times more than its real acceleration. Setting the noise too low makes the path run away. |
| Start-up | Starting while the vehicle already moves gave a wrong start (speed 0, wrong gyro bias). A parked start is much better. |
| Scale | The vehicle drives at steady speed, so there is almost no acceleration to learn the scale from. |
| Camera numbers | Estimated from the 1280x720 calibration, not a real calibration. |
| Data hole | 591 to 823 s has only images. |
