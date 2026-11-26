# Cal Poly Pomona Autonomous System Lab - MARVIN VIO
Freeman Y

# Notes
- Everything is based on episode 2
- Need to recalibrate cameras to 680x480 pixel
- Download OpenVINS
- Raw IMU data is converted from csv to ROS bags

DataPreprocess does the following
1. Get the csv paths
2. Analyze the dataset
3. IMU Noise Analysis
4. Time Synchronization
5. Combine into one unified csv

# Results
All values are for episode 2. More detail and how to run: [vio/README.MD](vio/README.MD).

**Recording**

| Item | Value |
|---|---|
| Duration | 979 s |
| IMU rate | 100 Hz |
| GNSS rate | 4 Hz |
| Camera rate | 5 to 10 Hz (frames 100 or 200 ms apart) |
| Camera frames | 5,742 (640x480) |
| Missing IMU, GNSS, speed | 591 to 823 s (232 s) |

**IMU (measured while parked, 60 s)**

| Item | Value |
|---|---|
| Gyro bias x, y, z | -0.0003, 0.0013, -0.0001 rad/s |
| Gyro noise (std) x, y, z | 0.0022, 0.0025, 0.0018 rad/s |
| Gyro noise density | 0.0002 rad/s/√Hz |
| Accel offset x, y, z | 0.196, 0.301, 9.788 m/s² |
| Accel noise (std) x, y, z | 0.020, 0.025, 0.029 m/s² |
| Accel noise density | 0.002 to 0.003 m/s²/√Hz |
| Gravity measured | 9.794 m/s² |
| Vibration while driving (std) | gyro 0.054 rad/s, accel 1.08 m/s² |

**Time sync**

| Item | Value |
|---|---|
| Image timestamp offset | -0.425 s (images stamped 0.425 s late) |
| Speed / yaw rate lag behind IMU | +0.15 s |
| Offset left after correction (OpenVINS) | 0.0008 s |

**Camera (640x480, estimated from the 1280x720 calibration)**

| Item | Estimate | OpenVINS result (seg_b) |
|---|---|---|
| fx, fy | 655.9, 655.9 px | 657.3, 656.1 px |
| cx, cy | 337.1, 253.5 px | 336.6, 253.2 px |
| Distortion k1, k2, p1, p2, k3 | -0.7477, 1.0905, 0.0078, 0.0020, -1.1605 | removed from images |

**OpenVINS**

| Item | seg_b_undist | seg_a_undist |
|---|---|---|
| Frames processed | 938 / 940 | 3411 / 3432 |
| Gyro bias x, y, z (rad/s) | -0.0002, 0.0013, -0.0001 | 0.0000, 0.0014, 0.0001 |
| Accel bias x, y, z (m/s²) | 0.0007, -0.0092, -0.0159 | 0.078, 0.063, -0.016 |
| Distance travelled | 2.6 m | 3604 m (at most 230 m possible) |
| Drift while parked | 0.02 m | 29 m |
| Result | works | wrong (diverges after about 205 s) |