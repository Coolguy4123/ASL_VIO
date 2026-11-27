# How to run

This folder runs **OpenVINS** (a visual-inertial odometry program) on our recording, `episode_2`. It reads the camera images and the IMU, and estimates where the vehicle went.

Status: it works only partly. On the long `seg_a` drive OpenVINS gets the **scale wrong** and later drifts away. The best pure-VIO runs are `seg_c` with the settings in `config/openvins/noise_0.03_0.002` (three runs: path 1.2 to 1.3 times too long, 2.5 to 4.8 m error against GNSS over 2 minutes).

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
vio/scripts/run_openvins.sh seg_c_undist          # uses config/openvins/noise_0.03_0.002
# other settings, different name for the results, small files:
CONFIG_DIR=vio/config/openvins/baseline OUT_NAME=my_test LIGHT=1 vio/scripts/run_openvins.sh seg_c_undist
```
When it ends, the run is **compared with the GNSS ground truth automatically**. In `vio/results/<run>/`:

| File | What it is |
|---|---|
| `vs_gnss.png` | OpenVINS path vs GNSS (left), and the distance between them over time (right) |
| `metrics.txt` | The same numbers as text: path lengths, average/RMS/worst error, best scale factor |
| `foxglove/` | A bag with the OpenVINS path and the GNSS path together (not made with `LIGHT=1`) |

The OpenVINS path is only rotated and shifted onto the GNSS track, never stretched, so a wrong scale stays visible.

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
- Afterwards in Foxglove, open `vio/results/<run>/foxglove/foxglove_0.db3` (it already contains the GNSS ground truth) and add panels:

| Panel | Topic |
|---|---|
| Image (plain camera) | `/cam0/image_raw/compressed` |
| Image (tracked corners) | `/ov_msckf/trackhist/compressed` |
| 3D, display frame `global` | `/ov_msckf/pathimu` (VIO path), `/ground_truth/path` (GNSS path), `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` (skip `poseimu` and `odomimu`: their timestamps don't match `/tf`) |

- For an older run that has no `foxglove/` folder (and was not made with `LIGHT=1`):
  `python3 vio/scripts/make_foxglove_bag.py <run>`. For the picture and numbers only: `python3 vio/scripts/plot_vs_gnss.py <run>`.

**6. Fix the scale (not pure VIO).** Keeps only OpenVINS's heading and moves at the logged speed:
```bash
python3 vio/scripts/rescale_path.py seg_a_undist
```
It writes `vio/results/seg_a_undist_corrected/trajectory.csv` and `plot.png`.

## Results so far

**Runs kept** (`vio/results/`), `seg_c`, 122 s of driving, true path 44 m (GNSS). Error = RMS distance to GNSS.

| Rank | Run folder | Settings | Path length | Error | Foxglove bag |
|---|---|---|---|---|---|
| 1 | `seg_c_auto` | `noise_0.03_0.002` | 57 m (1.28x) | 2.5 m | yes |
| 2 | `seg_c_noise_0.03` | `noise_0.03_0.002` | 54 m (1.22x) | 2.9 m | no (pose only) |
| 3 | `seg_c_highnoise_run2` | `high_imu_noise` | 63 m (1.42x) | 4.3 m | no (pose only) |

**All settings tried** (the other runs were deleted, the numbers stay here). Several runs agree within about ±1 m.

| Settings folder | Path length | Error vs GNSS | Height drift |
|---|---|---|---|
| `noise_0.03_0.002` (0.03 / 0.002, 3 runs) | **54 to 59 m (1.2 to 1.3x)** | **2.5 to 4.8 m** | under 1 m |
| `high_imu_noise` (0.05 / 0.003, 3 runs) | 63 to 65 m (1.4 to 1.5x) | 4.3 to 4.9 m | under 1 m |
| `noise_0.07_0.004` | 78 m (1.8x) | 9.2 m | under 1 m |
| `higher_imu_noise` (0.1 / 0.005) | 544 m (12x) | 141 m | up to 24 m |
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
