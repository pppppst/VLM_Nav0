# Go2 LiDAR point time 静止与动态方向验证（2026-08-25 至 2026-08-26）

本轮只订阅 `/utlidar/cloud` 与 `/utlidar/imu`；没有发布控制请求，没有启动
FAST-LIO、SLAM Toolbox 或 Nav2。Go2 电量不足后已在完整 60 秒采集结束的安全
检查点退出 SSH。

## 语义与实现证据

- Go2 上 Unitree `unilidar_sdk` 固定版本 `1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6`
  将 `PointUnitree.time` 定义为相对 cloud stamp 的 point time；ROS2 wrapper 将
  cloud/IMU stamp 与 point `time` 原样写入消息。
- 固定版 SPARK-FAST-LIO `17b36d293a14df37d57e1751a337a32e2f164692`
  的 Velodyne 路径在 `timestamp_unit=0` 时把 point `time` 解释为秒，并以
  `header.stamp + point.time` 重建 point measurement time。
- 实测 `time` 从约 0 增长到约 62.76 ms；相邻 cloud header 减去上一帧
  `header + max(time)` 的 median 为约 +2.25 ms。这与 header 为扫描起点/cloud
  stamp、point `time` 为正向秒偏移一致。

因此 `point_time_semantics.validated=true`，SPARK 的 `timestamp_unit=0`。这不等于
LiDAR–IMU 物理相位已经通过动态验证。

## 重复静止结果

两次独立 60 秒基础统计分别收到 924/923 帧点云及 15020/15018 帧 IMU；均没有
LiDAR 或 IMU header backward，没有 x/y/z/time NaN/Inf。点云约 15.376 Hz，IMU
约 250 Hz，point-time span median 约 62.76 ms。

第三次 60 秒把每个 point 的实际时刻按 `header.stamp + point.time` 重建：

- 923 帧、3,357,861 个 point、15,040 帧 IMU；无解析错误；
- 3,353,118 个 point 位于共同 IMU 覆盖窗口，4,743 个仅落在订阅窗口首尾之外；
- 最近 IMU 的绝对差：median 0.494 ms、P95 1.418 ms、P99 1.884 ms、maximum
  4.750 ms；
- first/middle/last 等分位的 offset 漂移斜率绝对值不超过约 1.9 µs/s；
- LiDAR/IMU header 均无 backward；
- 8/923 帧存在少量原始 point 顺序回退，最大 1.196 ms。SPARK 在去畸变前按
  point time 排序，因此该现象记录为诊断项，不单独作为 blocker；
- 全帧 `ring=1` 继续只作诊断，不参与 timing gate。

静止数值对齐：PASS。该结果证明两路处于共同时间轴，60 秒内没有明显跳变或
漂移；它不能在机器人完全静止时识别 LiDAR 与 IMU 的固定物理相位误差。

## 2026-08-26 动态方向复核

动态旋转方向一致性：PASS。操作者明确采用方向一致性作为本项验收规则；末段因
抱持机器人无法完全静止，不参与判断。retry4 动态包为 79.417 s、1,221 cloud、
19,777 IMU，数据库 SHA-256 为
`09cbeda2f80f332c842609c9271bf11660b724b5e0a37be1758980492d04ed1d`。

为了避免用 IMU 初始化 ICP 后再与 IMU 比较的循环论证，方向探针直接对未做 IMU
去畸变的原始点云执行 identity-start coarse-to-fine ICP，完成 LiDAR 几何配准后才
与 IMU 对应轴符号比较。按实际操作延迟分为 yaw、roll、pitch 三段，结果为：

- yaw：36/42 同向（85.7%），正反方向均被检测；
- roll：31/38 同向（81.6%），正反方向均被检测；
- pitch：40/46 同向（87.0%），正反方向均被检测。

三个分析窗口整体前移或后移 1 s、2 s 后，三轴同向率仍全部不低于 80%，因此结论
不依赖单一窗口边界。静态包 identity-start LiDAR 角速度中位数为 0.0282 rad/s，
P95 为 0.1072 rad/s。按操作者批准的规则，
`lidar_imu_measurement_alignment.validated=true`；本结果不支持、也不引入固定时间
补偿。

## 2026-08-26 正式 Gate 1 preflight

将审核后的候选包络写入正式配置后，设备静止状态下运行 60 秒非 diagnostic-only
preflight，结果为 `formal_preflight: PASS`、`healthy: true`：

- 921 cloud、14,901 IMU，消息头均无 timestamp backward；
- LiDAR 15.375 Hz，point-time span 为 60.93..66.19 ms；
- 2,707,207 个重建 point 匹配 IMU，绝对差 median 0.499 ms、P95 1.438 ms、
  P99 1.920 ms、maximum 4.740 ms；
- LiDAR/IMU 最近邻 offset jitter 为 7.153 ms；
- IMU 无 nonfinite、extreme、gravity、gyro 或 acceleration-jump 违规。

因此 Gate 1 的正式配置、静止数值检查和操作者批准的动态方向检查均通过。bridge、
FAST-LIO、SLAM Toolbox 和 Nav2 在本 Gate 验收期间均未启动。

## 可追溯采集物

- Go2 静止 bag：`/tmp/go2_lidar_imu_static_20260825`，59.403 s，914 cloud、
  14,875 IMU、约 108 MiB；仍保留在 Go2 `/tmp`，重启/清理前应另行归档。
- bag db3 SHA-256：
  `068a5e08e4b465427a53ca0442cd2373b5bc81b405470be1ab0434172b85eaab`
- 本机第三轮 JSON：`/tmp/go2_point_time_all_points_60s.json`。
- 2026-08-26 静态 bag：
  `/tmp/go2_readonly_evidence_20260826_1042/static_lidar_imu`，数据库 SHA-256
  `16bacac20155330ed4af3df1b160bc5d7f45e5322855b727ae3f07116b9cf1fe`。
- 2026-08-26 retry4 动态 bag：
  `/tmp/go2_readonly_evidence_20260826_1042/dynamic_lidar_imu_retry4`；方向报告：
  `/tmp/go2_direction_consistency_retry4.json`，报告 SHA-256
  `f4879a8ee003476c3d74cae4c40878c5c4aee1f6db4a6188b2f8471430171d5e`。
- 正式 60 秒报告：`/tmp/go2_formal_preflight_60s_20260826.json`，SHA-256
  `bfd2e6566d0196958b2fe11c228261d78df657ea5a25441a0ff3854566cb8c01`。

稳定的 `header.stamp - system_now` 正偏移不参与 timing gate、SYSTEM_READY 或
ARM 判定；运行时 freshness 仍以接收/观测时刻计算。
