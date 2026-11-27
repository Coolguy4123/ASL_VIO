# openvins

Each folder is a full OpenVINS setting (`estimator_config.yaml` plus the IMU and camera files). They differ only in the IMU noise numbers (and `ground_corners`, see below). The noise number tells OpenVINS how much to trust the IMU: larger means trust it less.

Results on `seg_c` (true path 44 m, error = RMS distance to GNSS):

| Folder | IMU noise (accel / gyro) | Result |
|---|---|---|
| `noise_0.03_0.002` | 0.03 / 0.002 | **Default.** 3 runs: path 54 to 59 m, error 2.5 to 4.8 m |
| `high_imu_noise` | 0.05 / 0.003 | 3 runs: path 63 to 65 m, error 4.3 to 4.9 m |
| `noise_0.07_0.004` | 0.07 / 0.004 | path 78 m, error 9.2 m |
| `higher_imu_noise` | 0.1 / 0.005 | too high: path 544 m, error 141 m |
| `baseline` | 0.005 / 0.0004 | too low: path 199 m, error 31.8 m |
| `ground_corners` | 0.05 / 0.003 | plus a mask on the top of the image and a lower corner threshold: path 520 m, error 87 m |

The good range is about 0.03 to 0.05; inside it the runs vary by about ±1 m, so there is no clear winner.

Use another one with: `CONFIG_DIR=vio/config/openvins/baseline vio/scripts/run_openvins.sh ...`
