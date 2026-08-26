# Go2 快速上线流程

本文是 `GO2_PORTING.md` 六个 Gate 的操作入口。它不会降低 gate，也不会改变
Ranger profile。所有真机运动相关命令默认关闭；候选参数不会自动写入正式配置。

## 充电期间已经可以完成的工作

- Go2 网络、QoS、原始传感器和时钟诊断代码已纳入仓库。
- 已按用户要求采用并接受 Co-Nav 历史 Go2 profile：LiDAR→IMU 与
  `base_link→utlidar_lidar` 均为 `[0.171, 0, 0.0908]`、单位旋转，并关闭
  gravity alignment。该 profile 已写入正式配置，但静止 bag 漂移约 0.06 m，
  静止 bag 漂移约 0.06 m；经用户验收判断视为满足当前要求。
- point-level time 会对帧中每个点重建实际采样时间，并统计最近邻 IMU 对齐误差。
- `config/go2_preflight_candidates.yaml` 保存当前只读数据生成的候选包络，但明确为
  `candidate_only: true`、`validated: false`；正式 gate 配置没有被放宽。
- Go2 Sport bridge 保持 `dry_run=true`、默认 DISARMED。

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

当前仍需硬件确认的最小集合是：Co-Nav profile 的静止真机重复验证、SPARK 动态地图质量、
`base_link→camera_link`，以及 Go2 6DoF 对 Nav2 costmap 的
实际影响。它们不能由离线代码猜测。

目前正式进度是：2/6 Gate 完整 PASS；Gate 1、Gate 2 已完成，
Gate 4 的离线部分已通过。

   Gate                     当前状态        尚缺
  ━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Gate 1 基础与原始数据    PASS            无；正式 60 秒 preflight healthy:true，
                                            动态 yaw/roll/pitch 方向一致性通过
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 2 外参与 LIO        PASS            Co-Nav 历史 profile 已采用并接受；
                                            静止真机重复验证仍建议执行
  ───────────────────────  ──────────────  ─────────────────────────────────────
   Gate 3 不运动导航链      未验收/待联网    当前 enp4s0 为 DOWN，暂不能刷新真机
                                            preflight；待恢复 Go2 网卡后启动 obstacle、
                                            LaserScan、SLAM、Nav2，检查 TF、costmap、
                                            footprint 和 6DoF
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

  下一步仍不是运动，而是启动 Gate 3 不 ARM 导航链并达到 NAV_READY；
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
