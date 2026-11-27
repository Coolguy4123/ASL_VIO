# openvins

Each folder holds a full OpenVINS setting (`estimator_config.yaml`, IMU and camera files).

| Folder | Note |
|---|---|
| `noise_0.03_0.002` | **Best so far. Default.** IMU noise accel 0.03, gyro 0.002 (path 1.2x too long on `seg_c`). One run. |
| `noise_0.07_0.004` | Accel 0.07, gyro 0.004. Worse (1.8x). |
| `high_imu_noise` | Accel 0.05, gyro 0.003. 1.4 to 1.5x, repeatable over 3 runs. |
| `baseline` | First version. IMU noise too low: path much too long on `seg_c`. |
| `higher_imu_noise` | `high_imu_noise` with the IMU noise doubled again (accel 0.1, gyro 0.005). Too much: path 12 times too long. |
| `ground_corners` | `high_imu_noise` plus a mask and a lower corner threshold. Tried, much worse. |

Use another one with: `CONFIG_DIR=vio/config/openvins/baseline vio/scripts/run_openvins.sh ...`
