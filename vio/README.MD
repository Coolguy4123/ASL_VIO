# How to run

This folder runs **OpenVINS** on our recording `episode_2`: it reads the camera images and the IMU and estimates where the vehicle went. Every run is compared with the GNSS track automatically.

**Status.** It works over a couple of minutes, not over the whole drive. On `seg_c` (parked start, 122 s) the best setting gives a path 1.2 to 1.3 times too long and 2.5 to 4.8 m error against GNSS. On the full `seg_a` drive it fails.

## What is in here

| Folder | What it holds |
|---|---|
| `config/` | `camera/` calibration, `openvins/` one settings folder per version |
| `scripts/` | make bags, run OpenVINS, compare with GNSS, check, fix scale |
| `bags/` | the recordings OpenVINS plays |
| `results/` | one folder per run, plus the GNSS ground truth |
| `ws/` | OpenVINS itself (downloaded and built here) |

Each folder has a small README of its own.

## Which bag

The IMU, GNSS and speed have a 232 s hole (591 to 823 s), so the recording is cut there. Use the `_undist` bags: lens distortion removed, image time moved by -0.425 s.

| Bag | Covers | Use |
|---|---|---|
| `seg_c_undist` | 427 to 590 s | Parked for 40 s, then drives. **The best test**, about 3 minutes. |
| `seg_a_undist` | 30 to 590 s | Full drive, about 10 minutes. Starts while moving, which OpenVINS handles badly. |
| `seg_a_short_undist` | 30 to 260 s | First part of it, for quick tests |

## Run it

From the project folder (`ASL_VIO`):

```bash
# 1. make a bag (once; needs episode_2/unified.csv and imu_raw.csv from DataPreprocess.ipynb)
source /opt/ros/humble/setup.bash
python3 vio/scripts/make_bag.py --start 427 --end 590 --out vio/bags/seg_c_undist \
    --image-time-shift -0.425 --undistort-with vio/config/camera/rgb_intrinsics_640x480.yaml

# 2. run OpenVINS (plays in real time; uses config/openvins/noise_0.03_0.002)
vio/scripts/run_openvins.sh seg_c_undist
#    other settings / own name / small files only (no Foxglove bag):
CONFIG_DIR=vio/config/openvins/baseline OUT_NAME=my_test LIGHT=1 vio/scripts/run_openvins.sh seg_c_undist

# 3. nothing printed here = nothing still running
pgrep -af "[r]un_subscribe_msckf|[r]os2 (bag|launch|run)|[i]mage_transport/republish"

# 4. optional: OpenVINS speed vs the logged speed (about 0.41 m/s), should be near 1
python3 vio/scripts/check_run.py seg_c_undist
```

## What you get

When a run ends, `vio/results/<run>/` contains, automatically:

| File | What it is |
|---|---|
| `vs_gnss.png` | OpenVINS path vs GNSS (left) and the distance between them over time (right) |
| `metrics.txt` | the same as numbers: path lengths, average / RMS / worst error, best scale factor |
| `foxglove/` | a bag with the OpenVINS path and the GNSS path together (not made with `LIGHT=1`) |
| `openvins.log` | OpenVINS's own messages; read this first if something looks wrong |

The OpenVINS path is only rotated and shifted onto GNSS, never stretched, so a wrong scale stays visible. For an older run: `python3 vio/scripts/plot_vs_gnss.py <run>` or `make_foxglove_bag.py <run>`.

**Foxglove:** open `vio/results/<run>/foxglove/foxglove_0.db3`.

| Panel | Topic |
|---|---|
| Image, plain camera | `/cam0/image_raw/compressed` |
| Image, tracked corners | `/ov_msckf/trackhist/compressed` |
| 3D, display frame `global` | `/ov_msckf/pathimu` (VIO), `/ground_truth/path` (GNSS), `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` |

Skip `poseimu` and `odomimu` in the 3D panel: their timestamps don't match `/tf`. **RViz** (live, start it before the run): `rviz2 -d vio/ws/src/open_vins/ov_msckf/launch/display_ros2.rviz`.

## Results

**Runs kept** (`seg_c`, 122 s, true path 44 m; error = RMS distance to GNSS):

| Run | Settings | Path | Error | Foxglove |
|---|---|---|---|---|
| `seg_c_auto` | `noise_0.03_0.002` | 57 m | 2.5 m | yes |
| `seg_c_noise_0.03` | `noise_0.03_0.002` | 54 m | 2.9 m | no |
| `seg_c_highnoise_run2` | `high_imu_noise` | 63 m | 4.3 m | no |

**Everything tried** (the other runs were deleted). The IMU noise setting matters most: it tells OpenVINS how much to trust the IMU, and the IMU here vibrates a lot.

| IMU noise (accel / gyro) | Settings folder | Path | Error |
|---|---|---|---|
| 0.005 / 0.0004 (too low) | `baseline` | 199 m | 31.8 m |
| **0.03 / 0.002** (3 runs) | `noise_0.03_0.002` | 54 to 59 m | 2.5 to 4.8 m |
| 0.05 / 0.003 (3 runs) | `high_imu_noise` | 63 to 65 m | 4.3 to 4.9 m |
| 0.07 / 0.004 | `noise_0.07_0.004` | 78 m | 9.2 m |
| 0.1 / 0.005 (too high) | `higher_imu_noise` | 544 m | 141 m |
| 0.05 / 0.003 + mask + more corners | `ground_corners` | 520 m | 87 m |

Runs of the same setting differ by about ±1 m, so 0.03 and 0.05 are not clearly different.

**`seg_a`, full drive (about 195 m):** OpenVINS as it is gave 5061 m. Fixing the scale with the logged speed gave 177 m and 1.5 m error, but that keeps only OpenVINS's heading, so it is not pure VIO (the gyro alone gives 1.6 m). The `seg_a` run folders were deleted to save space; to redo it, run `vio/scripts/run_openvins.sh seg_a_undist` (about 10 minutes), then `python3 vio/scripts/rescale_path.py seg_a_undist`.

## Known problems

- **IMU noise setting:** too low and the path runs away, too high and the scale is lost.
- **Start-up:** starting while the vehicle already moves gave a wrong start (speed 0, wrong gyro bias). Start from a standstill.
- **Scale:** the vehicle drives at a steady speed, so there is almost no acceleration to learn real-world scale from. This is probably why the path is still 1.2 to 1.3 times too long.
- **Camera numbers** are estimated, not calibrated.
- **Data hole:** 591 to 823 s has only images.
