# openvins

Each folder holds a full OpenVINS setting (`estimator_config.yaml`, IMU and camera files).

| Folder | Note |
|---|---|
| `high_imu_noise` | **Best so far. Default.** IMU noise raised to match the vibration. |
| `baseline` | First version. IMU noise too low: path much too long on `seg_c`. |
| `ground_corners` | `high_imu_noise` plus a mask and a lower corner threshold. Tried, much worse. |

Use another one with: `CONFIG_DIR=vio/config/openvins/baseline vio/scripts/run_openvins.sh ...`
