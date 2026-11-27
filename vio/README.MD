# How to run

This folder runs **OpenVINS** on our recording `episode_2`: it reads the camera images and the IMU and estimates where the vehicle went. Every run is compared with the GNSS track automatically.

There are two runs and two bags, nothing else:

| Run folder | Bag | Covers | Settings | Why |
|---|---|---|---|---|
| `seg_c_auto` | `seg_c_undist` | 427 to 590 s (parked for 40 s, then drives) | `default` | the most accurate test |
| `seg_a_full_moving_start` | `seg_a_undist` | 30 to 590 s (the whole drive before the data hole, one piece) | `moving_start` | the whole drive |

The two settings differ only in how OpenVINS starts: `default` assumes a standstill start, `moving_start` is for a vehicle that is already moving. The recording begins while moving, so the whole drive needs `moving_start`. Details in `config/openvins/README.md`.

## What is in here

| Folder | What it holds |
|---|---|
| `config/` | `camera/` calibration, `openvins/` the settings folders |
| `scripts/` | make bags, run OpenVINS, compare with GNSS, check, fix scale |
| `bags/` | `seg_c_undist`, `seg_a_undist` (lens distortion removed, image time moved by -0.425 s) |
| `results/` | the two runs above, plus the GNSS ground truth |
| `ws/` | OpenVINS itself (downloaded and built here) |

Each folder has a small README of its own. The IMU, GNSS and speed have a 232 s hole (591 to 823 s), so nothing after 590 s is used.

## Run it

From the project folder (`ASL_VIO`):

```bash
# 1. make a bag for a time range (needs episode_2/unified.csv and imu_raw.csv from DataPreprocess.ipynb)
#    change --start, --end (seconds) and --out; stay out of the data hole (591 to 823 s)
source /opt/ros/humble/setup.bash
python3 vio/scripts/make_bag.py --start 427 --end 590 --out vio/bags/seg_c_undist \
    --image-time-shift -0.425 --undistort-with vio/config/camera/rgb_intrinsics_640x480.yaml

# 2. run OpenVINS (plays in real time)
vio/scripts/run_openvins.sh seg_c_undist                                   # parked start: default settings
CONFIG_DIR=vio/config/openvins/moving_start OUT_NAME=seg_a_full_moving_start \
    vio/scripts/run_openvins.sh seg_a_undist                               # whole drive: moving start
#    OUT_NAME = name of the results folder, LIGHT=1 = small files (no Foxglove bag)

# 3. nothing printed here = nothing still running
pgrep -af "[r]un_subscribe_msckf|[r]os2 (bag|launch|run)|[i]mage_transport/republish"

# 4. optional: standstills (good places to start), and OpenVINS speed vs the logged speed
python3 vio/scripts/find_stops.py
python3 vio/scripts/check_run.py seg_c_auto
```

## What you get

When a run ends, `vio/results/<run>/` contains, automatically:

| File | What it is |
|---|---|
| `vs_gnss.png` | OpenVINS path vs GNSS (left) and the distance between them over time (right) |
| `metrics.txt` | the same as numbers: path lengths, average / RMS / worst error, best scale factor |
| `foxglove/` | a bag with the OpenVINS path and the GNSS path together (not made with `LIGHT=1`) |
| `openvins.log` | OpenVINS's own messages; read this first if something looks wrong |

The OpenVINS path is only rotated and shifted onto GNSS, never stretched, so a wrong scale stays visible.

**Foxglove:** open `vio/results/<run>/foxglove/foxglove_0.db3`.

| Panel | Topic |
|---|---|
| Image, plain camera | `/cam0/image_raw/compressed` |
| Image, tracked corners | `/ov_msckf/trackhist/compressed` |
| 3D, display frame `global` | `/ov_msckf/pathimu` (VIO), `/ground_truth/path` (GNSS), `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` |

Skip `poseimu` and `odomimu` in the 3D panel: their timestamps don't match `/tf`. **RViz** (live, start it before the run): `rviz2 -d vio/ws/src/open_vins/ov_msckf/launch/display_ros2.rviz`.

## Results

Error = RMS distance between the OpenVINS path and GNSS (rotated and shifted, not scaled).

| Run | True path (GNSS) | OpenVINS path | Error | With one scale number |
|---|---|---|---|---|
| `seg_c_auto` | 44 m | 57 m (1.28x) | 2.5 m (worst 5.5 m) | 1.2 m |
| `seg_a_full_moving_start` | 208 m | 418 m (2.0x) | 55.6 m (worst 126.8 m) | 5.2 m |

- **`seg_c_auto`** works well. Three runs of the same settings gave 54 to 59 m and 2.5 to 4.8 m.
- **`seg_a_full_moving_start`** is stable for all 556 s (height stays within -5.9 and 5.3 m, 3422 of 3432 frames) and has the right shape and turn, but the path is about 2 times too long. It was 3.4 times too fast in the first minute and about 1 time in the last 100 s, probably because the stops and restarts give OpenVINS a little acceleration to learn the scale from (a guess, not tested).
- After fixing the scale with one number, both are off by about 2.5 to 2.7% of the distance, so the shape is about equally good. The big difference is the scale.

**Other tests (those runs were deleted, the numbers stay here):**

| Test | Result |
|---|---|
| Whole-drive start with `default` settings (30 to 260 s, vehicle already moving) | complete runaway: 26,039 m for a true 99 m |
| Same part with `moving_start` | stable, 246 m for 99 m (2.5x) |
| `default` on a piece that starts at a 10 s stop (305 to 427 s) | failed: 326 m for a true 48 m (6.8x); a possible reason is the short stop, not tested |
| `default` on the parked end (823 to 980 s) | fine, but the vehicle barely moves |

**IMU noise (accel / gyro), tried on `seg_c`.** It tells OpenVINS how much to trust the IMU, which vibrates a lot:

| Noise | Settings folder | Path (true 44 m) | Error |
|---|---|---|---|
| 0.005 / 0.0004 (too low) | deleted | 199 m | 31.8 m |
| **0.03 / 0.002** (3 runs) | `default` | 54 to 59 m | 2.5 to 4.8 m |
| 0.05 / 0.003 (3 runs) | `noise_0.05_0.003` | 63 to 65 m | 4.3 to 4.9 m |
| 0.07 / 0.004 | deleted | 78 m | 9.2 m |
| 0.1 / 0.005 (too high) | deleted | 544 m | 141 m |

Runs of the same setting differ by about ±1 m, so 0.03 and 0.05 are not clearly different. To recreate a deleted setting, copy a folder and change the two noise lines in its `kalibr_imu_chain.yaml`. The deleted folders are still in git history.

## Known problems

- **IMU noise setting:** too low and the path runs away, too high and the scale is lost.
- **Start-up:** `default` fails when the vehicle already moves at the start; `moving_start` is stable there but less accurate.
- **Scale:** the vehicle drives at a steady speed, so there is almost no acceleration to learn real-world scale from.
- **Camera numbers** are estimated, not calibrated.
- **Data hole:** 591 to 823 s has only images.
