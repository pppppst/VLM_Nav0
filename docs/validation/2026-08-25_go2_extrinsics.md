# Go2 外参来源审计（2026-08-25）

> 当前采用状态（2026-08-28）：正式配置已采用 Unitree 官方 L1
> LiDAR→IMU `[0.007698, 0.014655, -0.00667]`、`R=I`，以及 Unitree Go2
> URDF 的 `base_link→utlidar_lidar` 候选 `[0.28945, 0, -0.046825]`、
> `rpy=[0, 2.8782, 0]`。本文后续关于 Co-Nav 组合和漂移的内容是历史验证记录；
> 新的组合 profile 尚未完成静止 Gate 2 复验。

本次只审计本机 `Co-NavGPT2`、Go2 上已确认的 Unitree L1 SDK 版本，以及 Unitree
官方源码；没有连接或移动 Go2。

## LiDAR → 内置 IMU：已确认

当前 Go2 发布 `/utlidar/cloud`（`utlidar_lidar`）和 `/utlidar/imu`
（`utlidar_imu`），Go2 上安装的 SDK 是 Unitree `unilidar_sdk` v1.0.16，commit
`1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6`。

Unitree L1 SDK 坐标定义说明：LiDAR 与 IMU 三轴平行，IMU 原点在 LiDAR 坐标系中
为：

```text
d_L_I = [-0.007698, -0.014655, 0.00667] m
```

VLM_Nav/SPARK 明确定义：

```text
p_imu = R_lidar_to_imu * p_lidar + T_lidar_in_imu
```

由于两坐标轴平行，点坐标从 LiDAR 转到 IMU 时 `p_imu = p_lidar - d_L_I`，所以：

```text
mapping.extrinsic_T = [0.007698, 0.014655, -0.00667]
mapping.extrinsic_R = identity(3)
```

该符号也与 Unitree 官方 `point_lio_unilidar/config/unilidar_l1.yaml` 一致。此值只
描述同一 L1 LiDAR 内部的 LiDAR→IMU 外参，不能用作机器人安装外参。

来源：

- Unitree `unilidar_sdk`：<https://github.com/unitreerobotics/unilidar_sdk>
- Unitree `point_lio_unilidar`：
  <https://github.com/unitreerobotics/point_lio_unilidar/blob/main/config/unilidar_l1.yaml>

## `base_link → utlidar_lidar`：官方位姿候选，SPARK 静止验收失败

旧 Co-Nav commit `f96ca5b74fe9ecea8021941bd4fbcf6103115ef9` 同时把
`[0.1710, 0, 0.0908]` 写入：

- `configs/spark_fast_lio_go2.yaml` 的 LiDAR→IMU `extrinsic_T`；
- `launch/spark_fast_lio_go2.launch.yaml` 的 `base_link→utlidar_lidar` 静态 TF。

这两个变换物理含义不同，不能共享同一数值。该提交没有测量记录、URDF 或驱动
来源，且相关资料常把该数值用于 XT-16 安装位置，不能证明它适用于当前 L1
UTLiDAR。

用户确认当前雷达为 Unitree 原厂安装、未移动或改装。Unitree 官方 Go2 URDF 的
`radar_joint` 给出父子变换 `xyz=(0.28945, 0, -0.046825) m`、
`rpy=(0, 2.8782, 0) rad`；本机驱动发布的物理雷达帧 `utlidar_lidar` 对应
URDF 的 `radar` 帧，因此已写入 `config/go2_calibration.yaml`。来源：
[Unitree 官方 Go2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)。

此前真机未启动导航栈时不存在 `/tf` 或 `/tf_static`；已用一次临时静态
发布器验证帧语义，`/tf_static` 观察到唯一发布者（1 个），变换值与上述
来源一致。这一步未启动 FAST-LIO、Nav2 或控制桥。

随后使用正确 remap、`use_sim_time` 和 `--clock` 回放动态/静止 bag。SPARK
可以启动并输出 `/odometry`（动态段约 530 Hz、注册点云约 30.7 Hz），但静止
bag 59.1 s 的里程计位移约为 `(3762.9, 2204.0, 618.4) m`，远超验收阈值。
因此官方 URDF 位姿只能保留为候选，`base_lidar_extrinsic.calibrated` 已恢复
为 `false`，正式 launch 继续 fail-closed。

作为对照，复现并采用 Co-Nav 历史配置（`extrinsic_T=[0.171,0,0.0908]`、单位旋转、
关闭 gravity alignment，并用相同数值发布 base TF）后，静止 bag 的 59.1 s
位移约为 `(0.0148, -0.0327, -0.0475) m`，合位移约 `0.060 m`。它明显优于
官方 URDF 候选，但仍略高于 0.05 m 门槛；且该配置把同一数值同时用于两个不同
物理变换。用户已明确接受该漂移水平，因此该 profile 已写入正式配置；Gate 2
记录为通过，后续仍建议在静止真机上重复确认。

进一步将当前已审核的 LiDAR–IMU 平移 `[0.007698, 0.014655, -0.00667]` 与
Co-Nav 的 base 位姿单独组合，静止回放同样发散；这表明 Co-Nav 的两个数值在
历史实现中存在耦合，当前证据不足以把 `[0.171,0,0.0908]` 单独归类为
`base_link→utlidar_lidar`。

## `base_link → camera_link`：仍未确认

旧 `ros_single_nav.py` 中约 0.206 m 的矩阵来自初始 commit `03b0343`，其 README
明确描述 D455 + Livox MID360 双 Go2；本机实测相机是 D435，定位链使用 UTLiDAR。
旧代码还把矩阵命名为 `T_lidar_camera`，不是 VLM_Nav 要求的
`base_link→camera_link`。因此不复用这些值，`camera_extrinsic.calibrated` 继续为
false。

## Gate 结论

- LiDAR→`/utlidar/imu`：PASS，可供 SPARK 配置渲染器使用。
- `base_link→utlidar_lidar`：官方来源/临时唯一 TF PASS；SPARK 静止漂移 FAIL。
- `base_link→camera_link`：BLOCKED / requires measurement。
- 由于后两项仍未确认，`go2_system.launch.py` 和 VLM Gate 继续 fail-closed；本次
  修改不会启动 FAST-LIO、相机三维投影或运动控制。
