• ## 核心结论

  VLM_Nav 是“VLM 选语义目标、Nav2 负责运动、独立桥接器负责真机安全”的分层控制：

  VLM/RGB-D → 目标点或探索前沿
                    ↓
          ComputePathToPose / NavigateToPose
                    ↓
       NavFn 全局规划 + DWB 局部控制
                    ↓
  /cmd_vel_nav → velocity_smoother → /cmd_vel_bridge
                    ↓
  限幅、ARM、watchdog、安全门
                    ↓
  /api/sport/request：Move API 1008
                    ↓
           Go2 内置 Sport 步态控制器

  VLM 不直接输出线速度或角速度，也不控制关节。

  ## 1. 速度控制策略

  Go2 正式配置在 VLM_Nav/config/nav2_go2.yaml:3：

   控制量             最小非零值        最大值    说明
  ━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━  ━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━
   前向线速度 vx        0.40 m/s      0.40 m/s    自主导航只前进，不后退
  ───────────────  ──────────────  ────────────  ────────────────────────
   横向速度 vy                 0             0    明确禁止侧移
  ───────────────  ──────────────  ────────────  ────────────────────────
   角速度 wz          0.50 rad/s    0.50 rad/s    可正转或反转
  ───────────────  ──────────────  ────────────  ────────────────────────
   线加/减速度        ±0.50 m/s²
  ───────────────  ──────────────  ────────────  ────────────────────────
   角加/减速度      ±0.50 rad/s²
  ───────────────  ──────────────  ────────────  ────────────────────────
   控制频率                20 Hz                  DWB 和 smoother

  因此有效控制接近：

  vx ∈ {0, 0.40}
  wz ∈ {-0.50, 0, +0.50}
  vy = 0

  允许 vx=0.40 与 wz=±0.50 同时出现，即走弧线。

  自定义 ComponentwiseMinSpeedGenerator (VLM_Nav/src/componentwise_min_speed_generator.cpp:12) 会过滤掉“小而无效”的速度候
  选：任一非零平移必须达到 0.40 m/s，任一非零旋转必须达到 0.50 rad/s。

  velocity smoother 同样配置了 0.40/0.50 deadband，所以加速过程中的较小分量会被归零。实际 Sport 输出更接近“零速/有效速度跳
  变”，而不是连续低速爬行。设计意图是避开 Go2 对过小速度不响应的问题，但代价是无法慢速精细挪动。

  ## 2. Nav2 如何产生动作

  控制话题被明确重映射：

  DWB/Behavior Server
      └─ /cmd_vel_nav
            ↓
  velocity_smoother
      └─ /cmd_vel_bridge

  见 VLM_Nav/launch/go2_navigation.launch.py:56。

  运动规划分两层：

  - 全局规划：NavFn 根据 SLAM 地图生成路径。
  - 局部控制：DWB 在局部代价地图中模拟速度轨迹，用 ObstacleFootprint 检查完整 Go2 footprint 是否碰撞，再选择评分最好的速
    度。

  机器人 footprint 为约 0.76 m × 0.36 m，见 VLM_Nav/config/nav2_go2.yaml:67。障碍来源是经过高度过滤、自身裁剪和体素降采样后
  的 FAST-LIO 点云。

  Nav2 普通目标的完成条件是机器人中心距离目标不超过 0.25 m。

  ## 3. VLM 搜索与探索策略

  VLM_Nav 的探索状态机大致为：

  SCANNING
    ↓ 未发现目标
  FRONTIER_SELECTING
    ↓ VLM 选择前沿
  EXPLORING
    ↓ 到达检查点/前沿
  重新扫描或重新选择前沿
    ↓ 目标连续确认
  APPROACHING
    ↓ 提前取消并确认停车
  SUCCEEDED

  ### 初始扫描

  机器人执行 8 个等角度 Spin，覆盖一整圈。每个方向：

  1. Nav2 完成旋转；
  2. 发布零速度；
  3. 等待 0.3 秒稳定；
  4. 获取一张运动结束后的新 RGB-D；
  5. 等待这张图对应的 VLM 结果；
  6. 再转向下一方向。

  见 VLM_Nav/vlm_nav/vlm_navigator.py:2327。

  ### 前沿探索

  如果扫描未发现目标：

  1. 从 SLAM OccupancyGrid 中寻找“可达自由空间与未知空间的边界”。
  2. 小于 8 个栅格的前沿被过滤。
  3. 使用“前沿面积 − 0.35 × 距离”排序。
  4. 最多保留 16 个候选。
  5. 把候选地图和八视角图交给 VLM。
  6. VLM 返回候选 ID、理由和置信度，低于 0.5 的结果拒绝。
  7. 先调用 ComputePathToPose 验证可达，再发送 NavigateToPose。

  见 VLM_Nav/vlm_nav/geometry.py:500 和 VLM_Nav/vlm_nav/vlm_navigator.py:2577。

  长路径不会一次盲走到底：默认最多在 3 m 活动半径内执行一段；较长前沿路径通常先走约一半，然后重新观察、重新规划。到达最终前
  沿后再次执行全景扫描。

  ## 4. 发现目标后的接近策略

  VLM 返回目标像素后，系统利用 RGB-D 和相机外参投影到 map 坐标系。

  目标必须满足：

  - VLM 置信度 ≥ 0.60；
  - 目标、限定词、关系描述全部匹配；
  - 有可靠深度；
  - 连续 3 帧空间位置一致；
  - 三帧位置偏差不超过 0.35 m。

  确认后停止当前探索目标，再让 Nav2规划到目标参考位置。

  接近目标时：

  - 距目标约 0.89 m 时提前取消 Nav2 动作并连续发零速度；
  - 等待至少 3 个静止里程计样本；
  - 静止判定：线速度 ≤ 0.03 m/s、角速度 ≤ 0.05 rad/s；
  - 最终机器人中心距目标参考点 ≤ 0.81 m 才算成功；
  - 若停车后仍超距，允许重新规划一次。

  相关配置见 VLM_Nav/config/robot.yaml:94，停车确认见 VLM_Nav/vlm_nav/vlm_navigator.py:2879。

  ## 5. Go2 桥接与安全策略

  桥接器订阅 /cmd_vel_bridge，输出 Unitree Sport 请求：

  API 1008: {"x": vx, "y": 0.0, "z": wz}
  API 1003: StopMove

  桥参数见 VLM_Nav/config/go2_bridge.yaml:1：

  - 最终限幅：|vx| ≤ 0.40 m/s、|wz| ≤ 0.50 rad/s；
  - 控制频率：20 Hz；
  - 命令超时：0.5 秒；
  - 侧移容差仅 1e-4；
  - 启动默认 DISARMED；
  - 默认 dry_run=true；
  - 禁止配置自动武装。

  以下情况进入 FAULT 并发送零速度和 StopMove：

  - 命令包含 NaN/Inf；
  - vy/vz/roll/pitch 非零；
  - 活跃速度命令超过 0.5 秒未更新；
  - SYSTEM_READY 丢失或超时；
  - Sport 接口持续消失；
  - ARMED 时发现其他程序也在发布 Sport 命令。

  进入 FAULT 后不会自动恢复，必须人工 reset_fault，然后重新 ARM。状态机见 VLM_Nav/include/vlm_nav/go2_bridge_state.hpp:68。

  ## 6. 正式启动为什么不会突然运动

  正式入口 VLM_Nav/scripts/VLMNav-go.sh:1 按顺序执行：

  1. 检查相机、NTP、深度单位和 API。
  2. 做 30 秒传感器 preflight。
  3. 启动定位、SLAM、代价地图和 Nav2，此时桥未启动。
  4. 人工输入 START_NAV。
  5. 确认没有其他 Sport 控制器。
  6. 启动真机桥，但保持 DISARMED。
  7. 等待 SYSTEM_READY 和 NAV_READY。
  8. 人工输入 ARM。
  9. VLM 仍保持 disabled。
  10. 再人工输入 ENABLE_VLM，才开始扫描和探索。

  所以真实运动必须同时满足：

  传感器健康
  + 定位/地图/Nav2健康
  + Sport接口健康
  + 无其他控制器
  + 人工ARM
  + 人工启用VLM

  总体上，VLM_Nav 是安全门严格、Nav2 闭环、速度接近“全速或停止”的控制方案；它比 Co-NavGPT 的固定四动作完整得多，但当前
  0.40/0.50 的最小速度策略较激进，不具备低速精细移动能力。

