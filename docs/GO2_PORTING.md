# 将 VLM_Nav 从 Ranger 移植到 Unitree Go2

本文档是首版冻结架构和实施清单。Ranger 仍是 `system.launch.py` 的默认
`robot_profile:=ranger`；Go2 必须显式使用 `robot_profile:=go2`。Go2 profile
默认 dry-run、DISARMED，并且所有未实测硬件量均为 `TBD / requires
measurement`。任何一级失败，只排查当前一级。

## 状态与使用门槛

- `SYSTEM_READY`：已完成系统 NTP 验证，DDS、LiDAR、IMU、FAST-LIO、基础 TF、
  Sport bridge 正常。
- `NAV_READY`：`SYSTEM_READY` 加 obstacle chain、SLAM Toolbox、`map → odom`
  和 Nav2 正常。
- `CONTROL_ARMED`：通过相应 safety gate 后，由操作者显式 ARM。
- `VLM_ENABLED`：`NAV_READY` 加 RGB、aligned depth、CameraInfo、相机标定和
  VLM API 后显式 enable。

运行时 topic 的 ownership 固定为：`go2_safety_supervisor` 独占发布
`/vlm_nav/system_ready`，`go2_nav_supervisor` 独占发布 `/vlm_nav/nav_ready`，
bridge 发布 `/vlm_nav/control_armed`，VLM 节点发布 `/vlm_nav/vlm_enabled`。
SYSTEM supervisor 持续检查原始传感器与 LIO 的本机接收 freshness、timestamp
backward、基础 TF、bridge 状态和 Sport DDS 接收端；NAV supervisor 还要求
obstacle/scan/map/costmap 新鲜、`map → odom` 和全部 Nav2 lifecycle 节点 ACTIVE。
所有运行时 gate 都按 heartbeat age 判断，不接受已经失去发布者的 transient-local
旧 `true`；Go2 VLM API monitor 后续必须持续发布 `/vlm_nav/vlm_api_ready`，单次
手工发布不能保持 VLM enable。

直接低速 bridge 验证只要求 `SYSTEM_READY`，之后仍需显式 ARM；Nav2 自动运动
要求 `NAV_READY + CONTROL_ARMED`；VLM 自动导航要求
`NAV_READY + CONTROL_ARMED + VLM_ENABLED`。Motion ARM 与 VLM enable
互不替代。VLM API 不可用不会把定位底座判成故障。

## TF ownership

Plan A 是首版唯一实现：

```text
map                 SLAM Toolbox 独占 map → odom
└── odom            SPARK-FAST-LIO 独占 odom → base_link
    └── base_link
        └── sensors 唯一静态 TF、URDF 或 robot_state_publisher
```

SPARK 使用 `map_frame=odom`、`base_frame=base_link`，发布 `/odometry` 和
`/cloud_registered_base`，并显式设置 `publish.scan_baseframe_pub_en=true`。
Go2 不启动 Ranger `fastlio_odom_adapter`。`/utlidar/robot_odom` 只允许诊断，
不得进入主 TF。启动顺序由 readiness waiter 强制为：

```text
static/robot TF → SPARK-FAST-LIO → obstacle filter
→ pointcloud_to_laserscan → SLAM Toolbox → Nav2
```

`base_link → utlidar_lidar` 在 SPARK 启动前必须连续可查询。不得同时有两个
`odom → base_link` 或两个 `map → odom` 发布者。

Plan B 只在动态 bag 证明 6DoF `base_link` 使 costmap 明显抖动后评审启用：

- SPARK 只发布 `/odometry`，不广播 `odom → base_link`。
- planar adapter 独占 `odom → base_footprint`。
- planar adapter 独占 `base_footprint → base_link`。
- 当前固定 SPARK 版本没有关闭 TF 的参数。最小未来修改是在
  `sendTransform` 外增加 `publish_tf`（默认 `true`）布尔保护，保持 odometry
  发布不变；Plan A 为 `true`，Plan B 为 `false`。
- 禁止创建平行 frame 名称或第二条 TF 路径来规避 ownership。

首版不实现 Plan B adapter。

## 三类硬件标定

`config/go2_calibration.yaml` 明确区分：

1. `lidar_imu_extrinsic` 是 FAST-LIO 内部 LiDAR 到 `/utlidar/imu` IMU 的外参，
   定义为 `p_imu = R_lidar_to_imu * p_lidar + T_lidar_in_imu`。只有 Unitree
   文档、驱动/URDF 源码或可复现实测能作为 source。
2. `base_lidar_extrinsic` 是 `base_link → utlidar_lidar` 机器人安装外参，只由
   一个静态 TF、URDF 或 robot_state_publisher 发布。
3. `camera_extrinsic` 是 `base_link → camera_link`。`calibrated=false` 时可以看
   RGB/depth，但不发布假 identity TF，不允许三维投影或 VLM 导航。

Unitree L1 SDK 已确认内部 LiDAR→IMU 外参：在
`p_imu = R * p_lidar + T` 定义下，`T=[0.007698, 0.014655, -0.00667]`、`R=I`；
来源和符号推导见 `docs/validation/2026-08-25_go2_extrinsics.md`。
`[0.171, 0, 0.0908]` 仍未被采用：旧 Co-Nav 将它同时用于 LiDAR→IMU 和
`base_link→utlidar_lidar`，没有证明其适用于当前 UTLiDAR 安装。identity 只有在
有明确来源时才可通过标定检查。

## DDS、数据和时钟 preflight

先 source `scripts/go2_network_env.sh`。自动选择规则是 0 个匹配接口失败、1 个
自动选择、多个失败；多网卡必须显式设置 `GO2_NET_IFACE`。固定环境为
`ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
`ROS_LOCALHOST_ONLY=0`，CycloneDDS 使用现代 `<Interfaces>` 配置。

`scripts/check_go2_qos.sh` 对 LiDAR、IMU 和三路 D435 topic 执行
`ros2 topic info -v`，检查 publisher/subscriber 的 reliability、durability、
history/depth。subscriber 必须按实测 publisher QoS 设置，并以至少连续收到三帧
消息
作为最终证据，不能只看 topic list。相机 QoS 在相机启动前保持 TBD；脚本不会
凭 RealSense 的常见默认值提前写死。

`go2_sensor_preflight --diagnostic-only` 只报告，不通过正式 gate。它检查
x/y/z finite、`time` 字段、点时间变化和帧内范围、header 单调、LiDAR Hz、IMU
finite/范数/最大值/跳变、IMU header 单调、共同覆盖窗口内的 LiDAR/IMU 时间关系，
并把整帧 `ring=1` 仅作为诊断现象。`scan_line` 不表示接收线数上限；
`scan_rate` 必须由 topic Hz 和 point-time span 联合确认。

`go2_camera_preflight` 只读订阅 RGB、aligned depth 和 color CameraInfo；检查
至少三帧、图像元数据、时间单调、RGB/depth 像素尺寸一致、CameraInfo 内参与
同一像素尺寸匹配，以及 RGB/depth 时间差。它不会接受
`/camera/camera/depth/image_rect_raw` 代替 aligned depth。

D435 当前经设备枚举和 60 秒实测选定的候选低带宽 profile 是 depth/color 均为
`640x480x15`；这是 D435 原生支持的离散 mode，不是软件插帧。只允许在 Go2
相机终端显式启动，不修改 Ranger profile：

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_sync:=true \
  align_depth.enable:=true
```

该 profile 当前是候选而非完整 gate PASS：Go2 本机 aligned-depth 60 秒约
14.23 Hz，未再出现 `USB SCP overflow`，但仍有启动期 libusb warning 和最长约
368 ms 的偶发间隔。VLM 必须拒绝超过同步门槛的单个 RGB-D pair，不能使用旧
depth 补配 RGB。

正式模式要求先用静止 rosbag 填写并审核 `config/go2_preflight.yaml` 中的 IMU
阈值和 LiDAR timing 阈值。系统时间先用 chrony/NTP 状态确认；单次
`ssh go2 date` 只能诊断，不能通过 gate。多帧
`local_receive_time - header.stamp` 的 minimum、median、jitter 继续记录，但只作
已知时间戳现象的观测，不参与 `healthy`、`SYSTEM_READY` 或 ARM gate。运行时消息
freshness 以本机接收/观测时刻计算，不用 header stamp 相对 `system_now` 的绝对
偏移判失败。正式 ROS/FAST-LIO/Nav2 启动前必须完成 NTP 同步，运行中禁止突然
向后校时。

正式 FAST-LIO timing gate 保持 fail-closed，但只针对实际测量时间：

- `point_time_semantics.validated=true`，并有证据说明
  `header.stamp` 表示扫描起点、中点或终点，以及 point `time` 的单位和参考方向；
- `lidar_imu_measurement_alignment.validated=true`，并由 bag/驱动证据证明每个点的
  实际采样时刻能与对应 IMU sample 对齐；
- point time finite、帧内变化和范围合理；LiDAR/IMU 时间戳单调，无明显跳变、
  漂移或 backward；
- 实测 LiDAR/IMU 相对 offset 与 jitter 在经审核的阈值内。

当前 IMU header 相对 callback `system_now` 的稳定正偏移降级为“已知时间戳现象，
持续观察”。它本身不阻塞 FAST-LIO，也不允许据此加入固定时间补偿。只有实际
LiDAR point 与 IMU 测量时刻出现稳定或漂移性错位时，才评审时间补偿。

## Nav2、6DoF 和 bridge

Go2 Nav2 明确使用 DWB，所有 y 方向最小/最大速度、加减速度和 smoother 数组均为
0，`vy_samples=1`，只允许 `vx + wz`。bridge 仍检查
`abs(linear.y) <= 1e-4`；明显非零 vy 会拒绝整条 command、报告 ERROR，并在
ARMED 时进入 FAULT，绝不静默丢弃。

首版保留 FAST-LIO 6DoF `odom → base_link`。动态 bag 专门检查 z/roll/pitch、
注册点云、LaserScan、costmap 和 footprint 是否随机体摆动异常；只有出现明确
证据才进入 Plan B。

源码 `twist_to_go2_sport_bridge` 运行在 x86 Humble 主机。重写源码的原因是可审计、
可重建、可测试，并能实现限幅、watchdog、ARM/DISARM/FAULT 和异常停车，而不是
ARM 架构兼容性。默认参数：`max_vx=0.20 m/s`、`max_vyaw=0.40 rad/s`、
`cmd_timeout=0.5 s`、`control_rate=20 Hz`、`dry_run=true`、DISARMED。

Move 使用 API 1008，JSON 为 `x=vx, y=0, z=vyaw`；StopMove 使用 API 1003。
watchdog 主要判据是 `last_valid_cmd_age > cmd_timeout`。DDS endpoint 只作辅助，
短暂消失由 debounce 吸收，ARMED 时持续消失才进入 FAULT。NaN/Inf、非零 vy、
非零 z/roll/pitch、输入超时、Sport endpoint 持续消失或软件故障进入 FAULT。
ARMED 转 DISARMED/FAULT 的边沿只发送一次零
Move 和 StopMove；FAULT 和重启都不会自动 ARM。节点退出 best-effort 停车，
但 SIGKILL、主机掉电、网络物理断开无法保证消息到达，首次运动必须保留操作者、
Unitree 遥控器及硬件安全手段。

## 固定依赖

`go2_dependencies.repos` 固定：

- `unitree_ros2@0dfa8f2...`：Humble verified、Go2 interface verified、message
  definition verified、Sport API verified。
- `spark-fast-lio@17b36d2...`：Humble verified、Go2 interface verified，待完成
  实测外参后 tested with current VLM_Nav integration。

固定的目的还包括 message definition verified 和 Sport API verified；硬件阶段
完成后补记 tested with current VLM_Nav integration 的 bag/报告 ID。依赖
不得自动跟随 upstream main。任何升级必须是独立改动并重跑全部离线和真机验收。

首次构建运行 `scripts/setup_go2_workspace.sh`。脚本先检查 ROS deb，再导入固定
commit，并在独立 `go2_ws/{build,install,log}` 中构建，不复制 Ranger 的
build/install/log。若缺包，脚本打印一条需要在可输入 sudo 密码的终端执行的
`apt-get install` 命令。Python API 环境继续使用
`/home/isee-pst/venv/co-nav-real`（Python 3.10、system-site-packages）和
`openai==2.32.0`；首版不安装仿真依赖、本地模型或模型权重。

## 6-Gate 快速实施与验收 Checklist

下面把原 19 个顺序步骤合并为六个不可跨越的 Gate，不删除任何安全条件。
每个 Gate 失败时只排查当前层，不继续叠加系统。

**Gate 1 — 基础与原始数据。** 构建固定依赖和 VLM_Nav，保持 Ranger 回归；
配置 DDS/NTP，保存 LiDAR、IMU、RGB、aligned depth、CameraInfo 的 QoS 报告；
只读验证 LiDAR/IMU finite、频率、时间单调、point-level time 和实际测量时间对齐。
稳定的 `header.stamp - system_now` 正偏移只记录，绝不进入 ready gate。

**Gate 2 — 外参与 LIO。** 使用有来源的 LiDAR↔IMU 外参，实测并唯一发布
`base_link → utlidar_lidar`；录制一段 60 秒静止 bag 和一段人工运动 bag，bridge
全程 DISARMED；用 bag 生成并人工审核候选阈值后再渲染 SPARK 配置。SPARK
独占 `odom → base_link`，静止 60 秒首版目标为平移漂移 < 5 cm、航向漂移 < 2°。

**Gate 3 — 不运动的导航链。** 依次启动 obstacle cloud、LaserScan、SLAM
Toolbox 和 Nav2，确认唯一 `map → odom`，验证障碍物方向、`vy=0`、costmap、
footprint 和 6DoF 姿态影响；只达到 NAV_READY，不 ARM。

**Gate 4 — 控制桥 dry-run。** 保持 `dry_run=true`，验证 Move 1008 / StopMove
1003 JSON、限幅、非零 `vy` 拒绝、0.5 秒 watchdog、endpoint debounce、FAULT
不自恢复、DISARM 停车及重启默认 DISARMED。该 Gate 不向真机 Sport topic
发送请求。

**Gate 5 — 人工监护低速运动。** SYSTEM_READY 后，在封闭场地保留 Unitree
遥控器/硬件安全手段，先显式 ARM 做极小速度 bridge 验证；再在 NAV_READY +
CONTROL_ARMED 下执行短 Nav2 goal，检查 roll/pitch/z、点云、LaserScan、costmap
和 footprint。若 6DoF 确认破坏 Nav2，才启用文中 Plan B。

**Gate 6 — VLM。** 标定并唯一发布 `base_link → camera_link`；验证 RGB-D、
CameraInfo、API 和三维投影，但先不发 goal。只有 NAV_READY + CONTROL_ARMED +
显式 VLM_ENABLED 同时成立后，才执行短 VLM 自动任务。

FAST-LIO 验收不能只看漂移，还必须检查 LiDAR/IMU finite、Hz、point time、单调
时间、掉帧；LIO 无 NaN、无突跳/回退、静止不持续漂移、运动地图不撕裂、注册
点云不拉花；TF 发布者唯一；Nav2 costmap 不随姿态跳动；bridge 未 ARM 不输出、
限幅正确、0.5 秒超时停车、DISARM/FAULT/endpoint 停止均停车且不自动恢复。

当前只读结果见 `docs/validation/2026-08-24_go2_readonly.md`。不得跨过其中的
阻塞项；尤其不得在未完成 dry-run/bag 验证、LiDAR point/IMU 实际测量时间、IMU
数据或外参异常，或相机未标定时进入相应自动运动阶段。
