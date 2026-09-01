# Go2 快速上线流程

本文是 `GO2_PORTING.md` 六个 Gate 的操作入口。它不会降低 gate，也不会改变
Ranger profile。所有真机运动相关命令默认关闭；候选参数不会自动写入正式配置。

## 充电期间已经可以完成的工作

- Go2 网络、QoS、原始传感器和时钟诊断代码已纳入仓库。
- LiDAR→IMU 已按 Unitree 官方 L1 几何切换为
  `[0.007698, 0.014655, -0.00667]`、`R=I`；`base_link→utlidar_lidar`
  保持 Unitree Go2 URDF 候选 `[0.28945, 0, -0.046825]`、
  `rpy=[0,2.8782,0]`。2026-08-27 静止真机复验使用旧 LiDAR→IMU profile，
  约 60 s 平移漂移 `1530993.37 m`、航向漂移 `56.01°`，因此新的组合 profile
  仍需重新验证；base 外参继续标记为未校准。
- point-level time 会对帧中每个点重建实际采样时间，并统计最近邻 IMU 对齐误差。
- `config/go2_preflight_candidates.yaml` 保存当前只读数据生成的候选包络，但明确为
  `candidate_only: true`、`validated: false`；正式 gate 配置没有被放宽。
- Go2 Sport bridge 保持 `dry_run=true`、默认 DISARMED。

## XT16 替换方案（2026-09-01）

当前采用“禾赛 XT16 点云 + 宇树 Go2 IMU”的最小替换链，不使用 Hesai
`/lidar_imu`：

```text
Hesai XT16 /lidar_points
  fields: x y z intensity ring timestamp
        │
        ▼ tools/hesai_to_fastlio.py
/lidar_points_fastlio
  fields: x y z intensity time ring
        │                         
        ├── SPARK LiDAR input
        └── preflight/safety LiDAR health input

Unitree IMU /utlidar/imu ──────── SPARK IMU input
```

XT16 配置约束：`lidar_type=2`（SPARK Velodyne/PointCloud2 路径）、
`scan_line=16`、`scan_rate=10`、`timestamp_unit=0`；LiDAR frame 为
`hesai_lidar`，IMU frame 为 `utlidar_imu`。ROS TF 使用独立的
`base_link→hesai_lidar`，FAST-LIO 内部继续使用独立的 LiDAR→IMU
`extrinsic_T/R`，两类外参不能合并。

本方案的 Hesai driver/converter 是 VLM_Nav 启动前的外部前置进程；VLM_Nav
不硬编码 Hesai 工作空间路径。正式启动前必须确认每个 topic 只有一个
publisher，避免重复 driver、converter 或 SPARK。

### XT16 + Unitree IMU Gate 2 复测

本次使用单实例 Hesai ROS 2.0.12 driver、converter、SPARK FAST-LIO，机器保持
静止，bridge/Nav2/VLM 均未启动。完成的是 60 秒静止在线 odometry 采集，不是
rosbag 回放；Gate 2 要求的人工动态段本轮未执行。实测证据：

| 项目 | 结果 | Gate 2 要求 |
|---|---:|---:|
| `/lidar_points` publisher | 1 | 唯一 |
| `/lidar_points_fastlio` publisher | 1 | 唯一 |
| `/utlidar/imu` publisher | 1 | 有真实消息 |
| `/odometry` publisher | 1 | SPARK 独占 |
| `/tf` publisher | 1，SPARK | `odom→base_link` 唯一 |
| `/tf_static` publisher | 1，XT16 static TF | `base_link→hesai_lidar` 唯一 |
| 静止时长 | 60.0 s | 60 s |
| odometry 样本 | 15359，约 256 Hz | 持续输出 |
| 平移漂移 | 0.02596 m | < 0.05 m |
| 航向漂移 | 0.1125° | < 2° |
| odometry timestamp 回退 | 330 次 | 0 |

结论：XT16 点云与宇树 IMU 能驱动 SPARK 输出 odometry，静止位置/航向漂移数值
低于阈值；但由于存在 330 次 odometry 时间戳回退，Gate 2 当前仍为
`FAIL/BLOCKED`，不能据此进入 Gate 3 导航链。同期观察到 Hesai driver 的
packet loss、点云帧大小波动，以及 SPARK 的 `Lidar loopback`、`No point, skip
this scan` 和点时间超过约 100 ms 警告。测试报告保存为
`/tmp/gate2_xt16_odom_60s.json`，SPARK 日志位于
`/tmp/spark_gate2_clean_roslog/`。

该结果说明当前主要 blocker 是 XT16 driver/转换链的帧完整性、时间单调性和
处理能力，而不是本次静止漂移阈值。Hesai driver 同时仍报告 firetime 文件缺失，
需要在后续稳定性复测前处理或明确其影响。

## 下一次连接 Go2：一次只读采集

先把脚本复制到 Go2；脚本只读传感器、TF、QoS 和录 bag，不包含 `cmd_vel` 或
`/api/sport/request`：

```bash
scp scripts/collect_go2_readonly.sh unitree@192.168.123.18:/tmp/
ssh -tt unitree@192.168.123.18
bash /tmp/collect_go2_readonly.sh \
  --output-dir ~/go2_readonly_evidence \
  --static-duration 60 \
  --dynamic-duration 45
```

脚本先采静止段，然后等待操作者确认再采人工运动段。整个过程 bridge 必须保持
DISARMED；“人工运动”是操作者安全地移动机器人或按既定人工方式采集，不授权
VLM/Nav2 自主运动。输出目录包含时间状态、QoS、TF、两段 bag 和 SHA256 清单。

采集完成后，在本机运行正式传感器诊断并生成新的候选配置：

```bash
source /opt/ros/humble/setup.bash
source ../go2_ws/install/setup.bash
source scripts/go2_network_env.sh
source /home/isee-pst/venv/co-nav-real/bin/activate

PYTHONPATH=. python -m vlm_nav.go2_sensor_preflight \
  --config config/go2_preflight.yaml \
  --duration 60 --diagnostic-only \
  --output /tmp/go2_raw_60s.json

ros2 run vlm_nav go2_derive_candidates \
  --point-report /tmp/go2_raw_60s.json \
  --raw-report /tmp/go2_raw_60s.json \
  --output /tmp/go2_preflight_candidates.yaml
```

生成结果只表示观测包络，不表示通过。必须检查原始报告、静态/动态 bag 和参数
来源，再人工修改正式配置中的对应项及 `validated` 状态。

## 最短真机路径

1. Gate 1：只读采集通过。
2. Gate 2：同事或实测确认 `base_link→utlidar_lidar`，SPARK 在 bag 和静止真机上
   通过，且 TF 发布者唯一。
3. Gate 3：不 ARM 启动 Ranger 同构导航链，达到 NAV_READY。
4. Gate 4：本机/联网 dry-run 通过，确认没有真实 Sport 请求。
5. Gate 5：有操作者和遥控器时才将 bridge 改为实发，先极小速度，再短 Nav2 goal。
6. Gate 6：相机安装外参标定后才启用 VLM 投影和自动任务。

当前仍需硬件确认的最小集合是：新 L1 + URDF 组合 profile 的静止真机验证、SPARK 动态地图质量、
`base_link→camera_link`，以及 Go2 6DoF 对 Nav2 costmap 的
实际影响。它们不能由离线代码猜测。

目前正式进度是：1/6 Gate 完整 PASS；Gate 1 已完成，XT16 + 宇树 IMU 的 Gate 2
已复测但因 timestamp 回退和帧稳定性问题保持 FAIL/BLOCKED，Gate 4 的离线部分已通过。

   Gate                     当前状态        尚缺
  ━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Gate 1 基础与原始数据    PASS            无；正式 60 秒 preflight healthy:true，
                                            动态 yaw/roll/pitch 方向一致性通过
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 2 外参与 LIO        FAIL/BLOCKED      XT16 `/lidar_points` + `/utlidar/imu` 静止漂移数值通过；
                                            但 60 s odometry 有 330 次 timestamp 回退，且存在 packet loss/不完整帧
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 3 不运动导航链      部分通过/阻断    静态 TF、SPARK 外参加载和 `/odometry` readiness
                                            obstacle filter、LaserScan 参数和 safety supervisor 已通过；
                                            但实测过滤后每帧仅约 2–62 点（中位数 15.5），SLAM 输入过稀并发生
                                            段错误；需决定 SLAM 是否使用更宽高度范围，尚未通过 Nav2
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 4 Bridge dry-run    离线部分通过    本机已验证限幅、watchdog、FAULT、
                                            StopMove 且无真实 Sport 输出；还需
                                            Go2 网络环境下 dry-run
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 5 低速运动          未开始          人工监护、遥控安全条件下的极小速度
                                            及短 Nav2 goal
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 6 VLM               未开始          相机安装外参、三维投影、API、VLM 自
                                            动任务

  下一步仍不是运动，而是先修复并重新验证 XT16 点云帧完整性、timestamp 单调性和
  converter 处理能力，再确认 `base_link→hesai_lidar` 外参及 SLAM/障碍过滤的
  分层高度策略并重新验证 Nav2 readiness；
  bridge 继续保持 DISARMED。

### 已完成

  - 建立独立 Go2 profile，未破坏 Ranger 默认配置。
  - 固定 Unitree ROS2 和 SPARK-FAST-LIO 依赖版本。
  - 完成 CycloneDDS、网卡选择、跨 Foxy/Humble QoS 检查。
  - 完成 LiDAR、IMU、D435 原始数据诊断代码。
  - 明确时间策略：稳定的 header.stamp-system_now 偏移不再阻塞，重点检查 point-
    time 与 IMU 实际对齐。
  - Gate 1 正式通过：60 秒 formal preflight 为 `healthy:true`，LiDAR/IMU
    point-level 数值对齐和动态 yaw/roll/pitch 方向一致性均已审核。

  - 找到并录入有来源的 LiDAR→IMU 外参。
  - 完成 SPARK、SLAM Toolbox、Nav2、障碍点云、LaserScan 的 Go2 配置与 launch 骨
    架。

  - Nav2 已约束为 vx+wz，vy=0。
  - 完成安全 Sport bridge：
      - 默认 DISARMED、dry_run=true
      - 速度限幅
      - watchdog
      - DISARMED/ARMED/FAULT 状态机
      - 非零 vy 拒绝
      - StopMove 边沿发送

  - 完成 SYSTEM_READY、NAV_READY、CONTROL_ARMED、VLM_ENABLED 分层。
  - 原 19 步精简为 6 个 Gate。
  - 增加一键只读采集脚本和候选参数生成工具。
  - 本机 dry-run 确认没有向真实 Sport topic 发布请求。
  - 验证通过：187 个 Python 测试、13 个 C++ 测试、构建及 launch 解析。

  ### 尚未完成
  6. 不 ARM 验证障碍点云、LaserScan、SLAM、Nav2 和 costmap，达到 NAV_READY。
  7. 验证 D435 RGB-D 配对及正式三维投影。
  8. 人工监护下完成 bridge 极小速度实验。
  9. 完成短距离 Nav2 闭环运动及 Go2 6DoF 影响验收。
  10. 最后验证 VLM API，并执行短 VLM 自动导航任务。

  下一步应从VLM_Nav/docs/GO2_FAST_TRACK.md:16开始，不应直接进入运动测试。
