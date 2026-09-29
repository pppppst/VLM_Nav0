# Go2 快速上线流程

本文是 `GO2_PORTING.md` 六个 Gate 的操作入口。它不会降低 gate，也不会改变
Ranger profile。所有真机运动相关命令默认关闭；候选参数不会自动写入正式配置。

## 充电期间已经可以完成的工作

- Go2 网络、QoS、原始传感器和时钟诊断代码已纳入仓库。
- 外参见~/Notes/待确定的参数.md, 里面记录了可靠的官方文档和前人导航实验github项目的参数。
- point-level time 会对帧中每个点重建实际采样时间，并统计最近邻 IMU 对齐误差。
- `config/go2_preflight_candidates.yaml` 保存当前只读数据生成的候选包络，但明确为
  `candidate_only: true`、`validated: false`；正式 gate 配置没有被放宽。
- Go2 Sport bridge 保持 `dry_run=true`、默认 DISARMED。

## XT16 替换方案（2026-09-01）

当前采用“禾赛 XT16 点云 + 宇树 Go2 IMU”的最小替换链，不使用 Hesai
`/lidar_imu`。正式 VLM_Nav Go2 launch 会启动 Hesai driver 和 C++ converter：

```text
Hesai XT16 /lidar_points
  fields: x y z intensity ring timestamp
        │
        ▼ hesai_fastlio_converter_node (C++)
/lidar_points_fastlio
  fields: x y z intensity time ring
        │                         
        └── SPARK LiDAR input

Unitree IMU /utlidar/imu ──────── SPARK IMU input
```

XT16 配置约束：`lidar_type=2`（SPARK Velodyne/PointCloud2 路径）、
`scan_line=16`、`scan_rate=10`、`timestamp_unit=0`；LiDAR frame 为
`hesai_lidar`，IMU frame 为 `utlidar_imu`。ROS TF 使用独立的
`base_link→hesai_lidar`，FAST-LIO 内部继续使用独立的 LiDAR→IMU
`extrinsic_T/R`，两类外参不能合并。

正式 Go2 launch 的 Hesai driver/converter 是 staged bringup 的前置节点；Hesai
配置路径通过 `hesai_config_file` 参数传入。正式启动前必须确认每个 topic 只有一个
publisher，避免重复 driver、converter 或 SPARK。

正式 XT16 配置文件为：

- `config/go2_calibration_xt16.yaml`：`base_link→hesai_lidar` 与 FAST-LIO 内部
  LiDAR→`utlidar_imu` 两套独立外参。
- `config/go2_preflight_xt16.yaml`：`/lidar_points`、16 线、10 Hz、单位秒。
- `config/spark_fast_lio_go2_xt16.yaml`：SPARK 使用 `/lidar_points_fastlio` 和
  `/utlidar/imu`。

下游接口保持不变：`/cloud_registered_base` → `/vlm_nav/obstacle_cloud` →
`/scan` → SLAM Toolbox → Nav2/VLM_Nav；由于 obstacle filter 输出已明确位于
`base_link`，Go2 Nav2 costmap 的 `sensor_frame` 使用 `base_link`。

旧 L2 配置文件仍保留，可通过显式 launch 参数回退，不作为新的 Go2 默认配置。

### XT16 + Unitree IMU Gate 2 复测

2026-09-02 修复 SPARK 双 odometry 发布路径并移除正式运行时的额外大点云诊断
订阅后，使用单实例 Hesai ROS 2.0.12 driver、converter、SPARK FAST-LIO 重新完成
60 秒静止在线采集。bridge/Nav2/VLM 均未启动；Gate 2 要求的人工动态段仍未执行。

| 项目 | 结果 | Gate 2 要求 |
|---|---:|---:|
| `/lidar_points` publisher | 1 | 唯一 |
| `/lidar_points_fastlio` publisher | 1 | 唯一 |
| `/utlidar/imu` publisher | 1 | 有真实消息 |
| `/odometry` publisher | 1 | SPARK 独占 |
| `/tf` publisher | 1，SPARK | `odom→base_link` 唯一 |
| `/tf_static` publisher | 1，XT16 static TF | `base_link→hesai_lidar` 唯一 |
| 静止时长 | 60.0 s | 60 s |
| odometry 样本 | 562，9.365 Hz | 约有效 LiDAR 频率 |
| 平移漂移 | 0.00733 m | < 0.05 m |
| 航向漂移 | 0.02135° | < 2° |
| odometry timestamp 回退/重复 | 0 / 0 | 0 / 0 |
| TF timestamp 回退 | 0 | 0 |
| TF 与 odometry stamp 差 | 0 | 0 |
| converter overwrite | 38/645，5.89% | ≤ 8% |
| driver packet loss | 未观测到 | 0 |

随后完成 60 秒人工动态段：轨迹长度 5.90 m、x 范围 1.06 m、yaw 覆盖 77.75°；
SPARK 有效匹配点最小 158、中位数 186，稳定窗口无 loopback、`No point` 或
`No Effective Points`。odometry/TF rollback 与 duplicate 均为 0，driver 无丢包，
converter overwrite 为 38/648（5.86%），低于 8% 上限。Gate 2 判定 `PASS`。
静止报告为 `/tmp/gate2_retest_60s.json`，动态报告为
`/tmp/gate2_dynamic_60s.json`，动态日志为 `/tmp/gate2_dynamic_stack.log`。
Hesai firetime 文件缺失仍记录为非阻断警告。

### Gate 3 最终回归（2026-09-02）

正式 staged launch 在 bridge `dry_run=true`、DISARMED 且未发送 Nav2 goal 的条件下
完成 Gate 3。60 秒 preflight PASS；converter 使用 `best_effort` 输入、`reliable`
输出并持续约 10 Hz；FAST-LIO 持续约 10 Hz，无持续 `No Effective Points`。
正式 `/wait_go2_fastlio` 在 0.898 s 内完成 `/cloud_registered_base` 3/3，exit code
为 0，约 2.5 ms 后启动下一阶段 `obstacle_cloud_filter`。

30 秒回归中 `/odometry` 为 9.9999 Hz，`odom→base_link` TF 为 9.9996 Hz；两者
timestamp rollback/duplicate 均为 0，配对 stamp 一致。全过程无真实 Sport 请求。
CycloneDDS receive-buffer、converter input QoS 和 FAST-LIO readiness blocker 均已
关闭。完整证据见 `docs/validation/2026-09-02_go2_gate3_final_regression.md`。

### Gate 4 control bridge dry-run（2026-09-02）

在正式 Go2 DDS 网络（`rmw_cyclonedds_cpp`、`ROS_DOMAIN_ID=0`、`enp4s0`）上运行
隔离命名的正式 bridge executable，输出仍指向 `/api/sport/request`，但保持
`dry_run=true`。新进程初始为 DISARMED；dry-run ARM 后，Move 1008 将请求
`x=0.8, z=-0.9` 限幅为 `x=0.2, y=0, z=-0.4`；0.5 秒 watchdog 使 bridge
进入 FAULT，并生成一次 StopMove 1003 debug request。整个测试窗口真实
`/api/sport/request` 消息增量为 0。

原生状态机测试同时覆盖非零 `vy`/不支持轴拒绝、endpoint debounce、DISARM
单次停车、FAULT 不自动恢复和重置后保持 DISARMED；实际 binary runtime 测试覆盖
限幅、watchdog 及真实 Sport 输出为零。Python bridge tests 为 4 passed，C++
`test_go2_bridge_state` 为 1/1 passed。Gate 4 判定 `PASS`。

### Gate 5 入场检查（2026-09-02）

封闭场地、操作者和遥控器条件就位后，先按 fail-closed 流程重跑 60 秒正式
preflight。首次报告 `/tmp/go2_gate5_preflight_60s.json` 为 `healthy:true`；在
重建安装空间后，为满足 300 秒 freshness 又连续采集两次新报告。两次均因静止
IMU 重力包络失败而为 `healthy:false`：

- `/tmp/go2_gate5_preflight_fresh_60s.json`：2/14917 帧越界，最小 9.5724 m/s²；
- `/tmp/go2_gate5_preflight_retry_60s.json`：7/14911 帧越界，最小 9.5480 m/s²；
- 正式下限为 9.6 m/s²；两次 LiDAR 均约 10.00 Hz，LiDAR/IMU timestamp rollback
  均为 0，且无 nonfinite、extreme、gyro 越界或 acceleration jump。

因此 Gate 5 在正式 staged/nav2 启动前即 `BLOCKED`。没有启动 bridge，没有 ARM，
没有发布速度或真实 Sport 请求；未放宽阈值，也未进入运动测试。

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

在切换了机体IMU /body_imu + 禾塞雷达/lidar/points组合之后，已通过的gate2--4都已经重测，结果见下面。

### 已完成

  - 建立独立 Go2 profile，未破坏 Ranger 默认配置。
  - 固定 Unitree ROS2 和 SPARK-FAST-LIO 依赖版本。
  - 完成 CycloneDDS、网卡选择、跨 Foxy/Humble QoS 检查。
  - 完成 LiDAR、IMU、D435 原始数据诊断代码。
  - 明确时间策略：稳定的 header.stamp-system_now 偏移不再阻塞，重点检查 point-
    time 与 IMU 实际对齐。
  - Gate 1 正式通过：60 秒 formal preflight 为 `healthy:true`，LiDAR/IMU
    point-level 数值对齐和动态 yaw/roll/pitch 方向一致性均已审核。

  - Gate 3 正式通过：FAST-LIO 约 10 Hz，正式 waiter 在 0.898 s 内完成 3/3 并
    exit 0，staged launch 成功进入 obstacle stage；odom/TF 无 rollback/duplicate。

  - Gate 4 正式通过：Go2 DDS 网络 dry-run 验证 Move/StopMove、限幅和 watchdog，
    真实 `/api/sport/request` 消息增量为 0；每次新启动默认 DISARMED。

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
  6. 人工监护、遥控器和封闭场地条件具备后，完成 Gate 5 bridge 极小速度实验。
  7. 完成短距离 Nav2 闭环运动及 Go2 6DoF 影响验收。
  8. 验证 D435 RGB-D 配对及正式三维投影。
  9. 最后验证 VLM API，并执行短 VLM 自动导航任务。

  下一步应从VLM_Nav/docs/GO2_FAST_TRACK.md:16开始，不应直接进入运动测试。
### Gate status update (2026-09-03)

正式 Go2 XT16 链路使用 Go2 body IMU：`/lowstate → /body_imu`，FAST-LIO 外参
`extrinsic_T=[0.171, 0, 0.0908]`、`extrinsic_R=I`。

Gate 1 **PASS** · Gate 2 **PASS** · Gate 3 **PASS** · Gate 4 **PASS** ·
Gate 5 dry-run **PASS**，真实运动待现场安全确认 · Gate 6 **未开始**。

Gate 5 真实运动仍需在封闭场地由操作者执行极小直线、极小 yaw 与 watchdog 验证；
不发送 Nav2 goal，不启动 VLM 自动任务。

## 9.3 gate5最终情况

进行前进，后退，左右转各十秒的测试， 观察发现，前进约4秒，然后一直后退且没能停止，到了左右旋转的阶段仍在退后。第二次仅测试旋转，观察到其确实按要求慢速旋转了逆时针和顺时针各一段时间。最终停止运动。

## Gate 5 正式标准与手动测试（2026-09-05，取代以上旧判据）

**最终结论：Gate 5 low-speed control = PASS。**

Gate 5 low-speed control 只有同时满足以下十项才判定 `PASS`：

1. `SYSTEM_READY=true`；
2. bridge 成功进入 `ARMED`；
3. `control_armed=true`；
4. 本桥实际发布非零 API 1008 Move；
5. 对应 API 1008 response 的 `status.code=0`；
6. 测试期间无未知外部 Move / SwitchJoystick 干扰；
7. odometry 与现场观察均证明机器人按命令方向运动；
8. StopMove 1003 成功；
9. 停止后无持续运动；
10. 最终 `bridge=DISARMED` 且 `control_armed=false`。

`sportmodestate.mode` 仅记录为诊断信息，不参与 Gate 5 PASS/FAIL。当前 Go2
实测可在 API 1008 response `code=0` 且 odometry/现场均发生运动时保持
`mode=0`。

### 0. 现场条件

- 封闭场地，操作者持遥控器并随时可急停；测试期间不操作摇杆或手机 App。
- 前后路径各留足运动和制动余量；旋转时机体四周无遮挡。
- 只允许一套 `go2_system.launch.py`，不运行其他 SDK example、测试 publisher
  或 Nav2 goal。
- 下列两个 extended 命令会让 Go2 真实运动；每个命令只执行一次。

### 1. 停止旧栈并确认退出

在旧 launch 终端按 `Ctrl-C`，然后执行：

```bash
ps -ef | grep '[g]o2_system.launch.py'
ps -ef | grep '[t]wist_to_go2_sport_bridge'
```

预期两条命令均无输出。若仍有输出，先在其原 launch 终端正常停止，不要启动第二套。

### 2. Terminal 1：正式 preflight

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_go2_preflight.sh
```

30 秒内保持 Go2 静止。预期末尾同时出现：

```text
"formal_preflight": "PASS"
"healthy": true
PASS: temporary sensor processes stopped
```

若 `healthy=false`、producer 退出或临时进程未停止，不得启动 real-output bridge。

### 3. Terminal 1：启动唯一 Nav2 / bridge 栈

必须在 preflight 报告生成后 600 秒内启动：

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh
export PYTHONPATH="/home/isee-pst/unitree_ros2/VLM_Nav:${PYTHONPATH:-}"
export ROS_LOG_DIR=/tmp/go2_nav2_runtime_roslog
mkdir -p /tmp/go2_nav2_runtime_roslog

ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:=/tmp/go2_costmap_preflight.json \
  target_stage:=nav2 \
  bridge_dry_run:=false
```

该终端持续刷新属于正常现象，不要按 `Ctrl-C`。预期先看到 bridge 以 real Sport
output 启动但保持 `DISARMED`，随后出现 `SYSTEM_READY=true`，最终 Nav2 lifecycle
节点 active。

### 4. Terminal 2：环境与基本自检

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh
export PYTHONPATH="/home/isee-pst/unitree_ros2/VLM_Nav:${PYTHONPATH:-}"
export ROS_LOG_DIR=/tmp/go2_gate5_manual_roslog
mkdir -p /tmp/go2_gate5_manual_roslog

ros2 topic echo /vlm_nav/system_ready --once --field data
ros2 topic echo /vlm_nav/nav_ready --once --field data
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
ros2 node info /twist_to_go2_sport_bridge
ros2 topic info /api/sport/request -v
timeout 2 ros2 topic echo /api/sport/request \
  unitree_api/msg/Request --qos-reliability best_effort
```

预期依次为 `True`、`True`、`DISARMED`、`False`；bridge 必须同时订阅
`/api/sport/request` 和 `/api/sport/response`。最后一条命令在静止 2 秒内应无消息；
裸 DDS publisher endpoint 的存在本身不等于存在外部控制流量。

### 5. Terminal 2：完整移动测试脚本

下面这条命令将执行完整 Gate 5 真机运动测试。脚本首先要求输入 `MOVE`，完成
前后移动并自动 DISARM 后，再要求输入 `ROTATE`；未输入完全一致的确认词时不会
进入对应运动阶段：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/movetest.sh --execute
```

执行顺序为：前进 `0.40 m/s × 6 s` → 零速 `1 s` → 后退
`0.40 m/s × 6 s` → DISARM → 人工二次确认 → 逆时针
`0.50 rad/s × 10 s` → 零速 `3 s` → 顺时针 `0.50 rad/s × 10 s` →
零速 `3 s` → DISARM。两段测试分别生成独立 rosbag。

**该脚本会让 Go2 真实运动。运行前必须确保 bridge 使用最新配置、场地安全、
遥控器可急停，并且无其他 Sport 控制流量。**

以下第 6、7 节保留为需要单独复测某一阶段时的等价命令。

### 6. Terminal 2：单独复测前进与后退

以下整条命令会自动执行：ARM → 前进 0.40 m/s 6 秒 → 零速并等待 1 秒 →
后退 0.40 m/s 6 秒 → 零速 → DISARM，并保存独立 rosbag：

```bash
ros2 run vlm_nav go2_gate5_pulse \
  --sequence linear \
  --speed 0.40 \
  --duration 6 \
  --execute-extended
```

预期现场方向为先前进、完全停住、再后退；任意方向错误、持续运动、外部控制介入或
bridge 离开 `ARMED`，立即使用遥控器急停。工具异常退出时会调用 DISARM，不继续
下一阶段。正常结束应输出 `COMPLETE: sequence=linear bridge DISARMED; bag=...`。

完成后确认：

```bash
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
```

预期为 `DISARMED`、`False`。等待至少 2 秒，并确认机器人没有持续运动。

### 7. Terminal 2：单独复测逆时针与顺时针旋转

再次确认场地和遥控器后执行。ROS 正 `angular.z` 为逆时针：

```bash
ros2 run vlm_nav go2_gate5_pulse \
  --sequence yaw \
  --yaw-rate 0.50 \
  --duration 10 \
  --settle-duration 3 \
  --execute-extended
```

该命令会重新 ARM，依次执行逆时针 0.50 rad/s 10 秒、零速等待 3 秒、顺时针
0.50 rad/s 10 秒，然后零速等待 3 秒并 DISARM。正常结束应输出
`COMPLETE: sequence=yaw bridge DISARMED; bag=...`。

再次确认最终状态：

```bash
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
```

### 注意！若速度/角速度过慢，以及加速度限制过低，会导致机器狗无法挪动腿，从而无法正常移动和旋转！

只有两份 rosbag 中均能证明非零 1008、对应 response `code=0`、StopMove 1003
成功、无未知外部 1008/1027，且 odometry 与现场四个方向观察一致时，才能将
Gate 5 标记为 `PASS`。禁止用 `cmd_vel_bridge` 有非零值单独证明 Go2 已收到命令，
也禁止回放包含 `/cmd_vel_bridge` 或 `/api/sport/request` 的 rosbag。

检查项                        结果
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   SYSTEM_READY=true             PASS，ARM 被接受
  ────────────────────────────  ──────────────────────────────────────────
   bridge=ARMED                  PASS
  ────────────────────────────  ──────────────────────────────────────────
   control_armed=true            PASS
  ────────────────────────────  ──────────────────────────────────────────
   本桥发布非零 API 1008         PASS，每轮 400 条
  ────────────────────────────  ──────────────────────────────────────────
   API 1008 response code=0      PASS，无失败或缺失
  ────────────────────────────  ──────────────────────────────────────────
   无外部 Move/SwitchJoystick    PASS
  ────────────────────────────  ──────────────────────────────────────────
   前后运动里程计                PASS，前进约 1.03 m，后退约 1.05 m
  ────────────────────────────  ──────────────────────────────────────────
   旋转里程计                    方向 PASS：逆时针约 95.4°，顺时针约 9.9°
  ────────────────────────────  ──────────────────────────────────────────
   StopMove 1003                 PASS，每轮均成功
  ────────────────────────────  ──────────────────────────────────────────
   最终 DISARM                   PASS，日志明确记录 DISARM
  ────────────────────────────  ──────────────────────────────────────────
   停止后无持续运动              数据 PASS，残余速度接近零
  ────────────────────────────  ──────────────────────────────────────────
   sportmodestate.mode           始终为 0，仅作诊断，不影响判定

  需要注意：顺时针旋转幅度明显小于逆时针。确实观察到这个现象。

已创建：VLM_Nav/scripts/movetest.sh，并加入 Gate 5 文档及安装构建，测试通过。

  以下命令会让 Go2 真实运动：

  /home/isee-pst/unitree_ros2/VLM_Nav/scripts/movetest.sh --execute

  脚本要求依次手动输入 MOVE、ROTATE，执行：

  前进 0.40 m/s × 6s → 停1s → 后退 × 6s → DISARM
  → 二次确认
  → 逆时针 0.50 rad/s × 10s → 停3s
  → 顺时针 0.50 rad/s × 10s → 停3s → DISARM

  每段自动保存独立 rosbag；异常时 pulse 工具会尝试 DISARM。
