# Go2 `base_link → utlidar_lidar` URDF 外参复验（2026-08-27）

## 方法

- 使用唯一临时静态 TF：`base_link → utlidar_lidar`。
- TF 平移：`[0.28945, 0, -0.046825] m`。
- TF 旋转：`rpy=[0, 2.8782, 0] rad`。
- FAST-LIO 输入：`/utlidar/cloud` + `/utlidar/imu`。
- FAST-LIO 内部 LiDAR→IMU 外参保持 Co-Nav 配置 `[0.171, 0, 0.0908]`，未与安装 TF 混用。
- 机器狗保持静止，记录 `/odometry` 约 60 秒；bridge、Nav2 均未启动。

## 结果

| 指标 | 结果 | Gate2 目标 |
|---|---:|---:|
| 有效时长 | 59.982 s | 60 s |
| `/odometry` 样本数 | 11085 | 持续输出 |
| 平移漂移 | 1530993.37 m | < 0.05 m |
| 航向漂移 | 56.01° | < 2° |

结论：FAIL。该 URDF 候选与当前 `/utlidar` 数据链不一致，
`base_lidar_extrinsic.calibrated` 保持 `false`，不能用于正式 Gate3/导航启动。
