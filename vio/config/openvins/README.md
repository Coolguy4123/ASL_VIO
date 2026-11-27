# openvins

Each folder is a full OpenVINS setting (`estimator_config.yaml` plus the IMU and camera files). They differ only in a few lines. The IMU noise number tells OpenVINS how much to trust the IMU: larger means trust it less.

| Folder | What is different | Result |
|---|---|---|
| `default` | IMU noise accel 0.03, gyro 0.002. Start from a standstill. | `seg_c` (parked for 40 s, then drives): 3 runs, path 54 to 59 m for a true 44 m, error 2.5 to 4.8 m. Fails when the vehicle already moves at the start. |
| `moving_start` | `default` plus the start-up mode for a vehicle that already moves (`init_dyn_use: true`, `init_dyn_min_deg` 3, `init_imu_thresh` 0.3) | The whole drive before the data hole (`seg_a`, 556 s): stable, path 418 m for a true 208 m (2.0 times too long). |
| `noise_0.05_0.003` | `default` with IMU noise accel 0.05, gyro 0.003 | `seg_c`: 3 runs, path 63 to 65 m, error 4.3 to 4.9 m |

Other noise values were tried and deleted: 0.005 / 0.0004 (too low, path 199 m), 0.07 / 0.004 (path 78 m) and 0.1 / 0.005 (too high, path 544 m). The good range is about 0.03 to 0.05. See `vio/README.MD` for the full table.

Use another one with: `CONFIG_DIR=vio/config/openvins/moving_start vio/scripts/run_openvins.sh ...`

## The default in detail

| Group | Setting |
|---|---|
| IMU noise | accel noise 0.03, accel random walk 0.003, gyro noise 0.002, gyro random walk 0.00002, rate 100 Hz |
| Camera | one camera, pinhole, no distortion (the images are already corrected), fx 655.9, fy 655.9, cx 337.1, cy 253.5, 640x480, time shift 0 |
| Camera mounting (guess) | camera looks forward, IMU x forward, y left, z up, camera position 0 |
| Online calibration | on for camera mounting, camera intrinsics and camera-IMU time offset |
| Start-up | standstill start (`init_dyn_use: false`), 2 s window, `init_imu_thresh` 1.5, `init_max_disparity` 10 |
| Standstill updates | on: `zupt_max_disparity` 0.5, `zupt_noise_multiplier` 10, `zupt_max_velocity` 0.1 |
| Filter | `max_clones` 15, `max_slam` 50, `max_msckf_in_update` 40, rk4, first-estimate Jacobians, gravity 9.81 |
| Corner tracking | 200 points, `fast_threshold` 20, 5x5 grid, `min_px_dist` 10, 10 frames per second, no histogram change, no mask |
| Camera trust | pixel noise 1.5 (`up_msckf_sigma_px` and `up_slam_sigma_px`) |
