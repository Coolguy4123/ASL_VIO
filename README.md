# Cal Poly Pomona Autonomous System Lab - MARVIN VIO
Freeman Y

We try to estimate where the MARVIN vehicle drove using only its camera and IMU (visual-inertial odometry, VIO), with OpenVINS. Everything is based on one recording, `episode_2`.

# Notes
- Everything is based on episode 2.
- The camera numbers are an **estimate**: the calibration was made at 1280x720, but the frames are 640x480. A real calibration at 640x480 is still needed.
- OpenVINS is installed in `vio/ws` (ROS 2 Humble).
- Raw IMU data is converted from csv to ROS bags (`vio/scripts/make_bag.py`).

# Folders
| Folder | What it is |
|---|---|
| `DataPreprocess.ipynb` | Analysis of the recording (steps below) |
| `episode_2/` | The raw recording (not in git) |
| `vio/` | Everything for running OpenVINS: scripts, settings, results. Start with [vio/README.MD](vio/README.MD) |
| `docs/` | Reference documents |

DataPreprocess does the following
1. Get the csv paths
2. Analyze the dataset
3. IMU noise analysis
4. Time synchronization
5. Combine into one unified csv

# Results
All numbers are for episode 2.

**The recording.** 979 s long. IMU 100 Hz, GNSS 4 Hz, camera 5 to 10 Hz (5,742 frames). From 591 s to 823 s the IMU, GNSS and speed are missing and only images exist, so the recording is cut there.

**What we measured**

| Item | Value |
|---|---|
| Gyro bias (parked), x y z | -0.0003, 0.0013, -0.0001 rad/s |
| Gyro and accel noise (parked) | gyro about 0.002 rad/s, accel 0.02 to 0.03 m/s² |
| Accel x and y offsets | 0.2 and 0.3 m/s²: the vehicle's tilt (1 to 2 degrees), not sensor drift |
| Vibration while driving | accel 1.1 m/s², gyro 0.054 rad/s (about 10 times the real acceleration) |
| Image timestamps | **0.425 s late**; speed and yaw rate are 0.15 s behind the IMU |
| Camera (estimate) | fx = fy = 656 px, cx = 337, cy = 254 |

**VIO results**

| | Path vs the real 44 m (GNSS) | Error vs GNSS |
|---|---|---|
| `seg_c`, best setting, 3 runs (parked start, 122 s) | 54 to 59 m | 2.5 to 4.8 m |
| `seg_a`, full drive, OpenVINS as it is | 5061 m instead of about 195 m | runs away |
| `seg_a`, scale fixed with the logged speed | 177 m | 1.5 m, but this is **not pure VIO** |

In short: pure VIO works over a couple of minutes if it starts from a standstill and the IMU noise setting is right, but the path stays about 1.2 to 1.3 times too long. On the long drive it fails. The causes we found: the IMU noise setting (too low runs away, too high loses the scale), a bad start-up while already moving, and a vehicle that drives at steady speed with almost no acceleration to measure scale from. GNSS (our ground truth) is ordinary GPS, about 2 to 3 m accurate.
