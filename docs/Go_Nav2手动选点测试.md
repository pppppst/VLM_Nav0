# Go2 Nav2 手动选点测试

本流程用 RViz `2D Goal Pose` 下发近距离目标。Nav2 规划成功后通过
`/cmd_vel_nav → velocity_smoother → /cmd_vel_bridge → Go2 Sport Move`
让机器人运动；规划失败时不应运动。全程不启动 VLM。

人工定位阶段不启动 bridge，允许使用遥控器控制 Go2；候选目标确定且遥控器
Sport 流量静默后，才启动一套全新的 bridge。RViz 的正式目标工具使用
`/goal_pose`，按钮名称为 `2D Goal Pose`。

完整流程可直接运行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/manualnav2.sh
```

脚本完成 preflight，等待 Nav2/RViz 就绪后要求输入 `START_NAV`，确认已用遥控器
移动到起点、机器人静止且遥控输入停止，随后检查并 ARM。ARM 后在脚本终端输入
`CANCEL` 可取消当前目标，全部测试结束后输入 `DISARM`。紧急停车可在
另一个终端随时执行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/stopnav2.sh
```

如果 preflight 前已有 `/lidar_points` 或 `/body_imu` 发布者，脚本会显示发布者
信息。必须在其原终端正常停止，输入 `RETRY_PREFLIGHT` 后脚本才会重新检查并继续；
脚本不会强制终止所有权不明的传感器进程。

以下章节保留为逐步执行和排障参考。

## 0. 安全条件

- Gate 5 已 `PASS`。
- 封闭、平整场地；首次目标距离为 `0.5–1.0 m`。
- 操作者手持遥控器，能随时人工急停。
- 测试期间不操作遥控器摇杆、手机 App 或其他 SDK 程序。
- 只允许一套 `go2_system.launch.py`。
- 任何方向异常、障碍物未识别或无法停止：先用遥控器急停。

## 1. 开机与网络

1. 启动 Go2 和禾赛 XT16，等待设备稳定。
2. 连接网线，确认电脑具有 `192.168.123.x` 地址：

```bash
ip -brief address
```

后续 source 网络脚本时应看到类似：

```text
Go2 DDS interface: enp4s0
```

## 2. 停止旧进程

若旧终端仍在运行，在原终端按 `Ctrl+C`。然后检查：

```bash
ps -ef | grep '[g]o2_system.launch.py'
ps -ef | grep '[t]wist_to_go2_sport_bridge'
ps -ef | grep '[r]viz2'
```

预期均无输出。如有输出，先在对应的原终端正常停止，不要再启动一套。

## 3. Terminal 1：30 秒正式 preflight

保持 Go2 静止，执行：

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH

/home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_go2_preflight.sh
```

必须看到：

```text
healthy: True
PASS: temporary sensor processes stopped; report: /tmp/go2_costmap_preflight.json
Start go2_system.launch.py within 600 seconds.
```

若 preflight 失败，不得继续。

## 4. Terminal 1：启动 Nav2 主栈，暂不启动 bridge

在 preflight 生成后 600 秒内执行：

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
export ROS_LOG_DIR=/tmp/go2_nav2_manual_roslog
mkdir -p /tmp/go2_nav2_manual_roslog

ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:=/tmp/go2_costmap_preflight.json \
  target_stage:=nav2 \
  bridge_dry_run:=false \
  start_bridge:=false
```

该终端会持续刷新，属于正常现象；不要按 `Ctrl+C`。预期现象：

- Hesai、body IMU、FAST-LIO、SLAM Toolbox 和 Nav2 依次启动；
- 此阶段没有 `twist_to_go2_sport_bridge` 进程，不存在本项目 Sport 输出；
- safety supervisor 正在等待 bridge，`SYSTEM_READY=false`、`NAV_READY=false`
  属于预期现象；
- Nav2 lifecycle 节点进入 `active`。

## 5. Terminal 2：基本自检

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

ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 topic info /local_costmap/costmap
ros2 topic info /global_costmap/costmap
ros2 node list | grep twist_to_go2_sport_bridge || true
```

三个 lifecycle 节点均必须为 `active [3]`；`/navigate_to_pose` 必须有 action
server；两个 costmap 的 `Publisher count` 必须为 `1`；最后一条命令必须无输出。

## 6. Terminal 3：启动 RViz

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh

/home/isee-pst/unitree_ros2/VLM_Nav/scripts/start_rviz.sh \
  /home/isee-pst/unitree_ros2/VLM_Nav/config/go2_costmaps.rviz
```

RViz 中确认：

- `Fixed Frame` 为 `map`；
- `SLAM Map`、`Local Costmap`、`Scan`、`Robot footprint` 正常；
- 工具栏同时存在 `Publish Point` 和 `2D Goal Pose`；
- 机器人位置稳定，地图不明显飘移；
- 机器人轮廓不落入障碍区；
- 首次目标的整条路径是已建图、可见的空旷区域。

Local 和 Global Costmap 的机器人几何 footprint 均为
`0.76 m × 0.36 m`（`x=±0.38 m`、`y=±0.18 m`），
`footprint_padding=0.0`，不额外扩张。

手动选点测试的 DWB 轨迹碰撞判定使用 `ObstacleFootprint`：在每个
模拟轨迹位姿放置上述完整 footprint 并检查障碍，不使用
`BaseObstacle` 的机器人中心点式判定。costmap 仅作为障碍数据源。

Local Costmap 的垂直体素范围为 `-0.20–0.80 m`（相对 odom），覆盖实测站立
LiDAR 原点约 `z=0.42 m` 以及下蹲约 `0.30 m` 的高度变化，并保留上下余量。

`Stereo is NOT SUPPORTED` 不影响地图或 Nav2。

## 7. Terminal 4：启动取证

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/record_go2_nav2_manual.sh
```

看到 `Press SPACE for pausing/resuming` 表示 rosbag 正在正常记录，保持该终端运行，
不要再粘贴其他命令。如某个路径 topic 当时不存在，warning 不会阻止其他 topic
记录。

## 8. 遥控器定位与候选目标选点

此时本项目 bridge 没有启动，允许使用遥控器让 Go2 站起、蹲下或移动到合适的
测试起点，同时观察 RViz 中的地图、位姿和 costmap。

位置确定后释放遥控器摇杆，使 Go2 完全静止。在 Terminal 2 执行：

```bash
ros2 topic echo /clicked_point --once
```

然后在 RViz 点击 `Publish Point`，在计划的目标位置单击。记录输出的
`frame_id: map`、`x` 和 `y`。

**`Publish Point` 只确定候选坐标，不会发送 Nav2 goal，也不会让 Go2 运动。
此阶段不要点击 `2D Goal Pose`。**

## 9. Terminal 5：遥控器静默后启动全新 bridge

停止遥控器输入后，先在 Terminal 2 监听 2 秒：

```bash
timeout 2 ros2 topic echo /api/sport/request \
  unitree_api/msg/Request --qos-reliability best_effort
```

该命令必须无 Move / SwitchJoystick 消息。然后新建 Terminal 5 并执行：

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh
export ROS_LOG_DIR=/tmp/go2_nav2_manual_bridge_roslog
mkdir -p /tmp/go2_nav2_manual_bridge_roslog

ros2 run vlm_nav twist_to_go2_sport_bridge --ros-args \
  --params-file /home/isee-pst/unitree_ros2/VLM_Nav/config/go2_bridge.yaml \
  -p dry_run:=false
```

保持 Terminal 5 运行。预期 bridge 以 real Sport output 启动并保持 `DISARMED`。

## 9.1 Terminal 6：只读观测发往 Go2 的实际速度

新建一个终端执行：

```bash
cd /home/isee-pst/unitree_ros2
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh
export ROS_LOG_DIR=/tmp/go2_nav2_speed_observer_roslog
mkdir -p /tmp/go2_nav2_speed_observer_roslog

ros2 topic echo /api/sport/request unitree_api/msg/Request \
  --qos-reliability best_effort \
  --field parameter \
  --filter 'm.startswith("{\"x\":")'
```

该窗口只订阅并显示 bridge 实际发布的 Move 速度 JSON：`x` 是前向
线速度（m/s），`y` 固定为 `0.0`，`z` 是 yaw 角速度（rad/s）。
`Ctrl+C` 只会停止该诊断订阅者，不会向 Go2 发送命令。
本轮验收时，任何非零 `x` 必须满足 `|x| >= 0.40 m/s`，任何非零
`z` 必须满足 `|z| >= 0.50 rad/s`。

## 10. Terminal 2：bridge 自检与 ARM

等待至少 2 秒，执行：

**下列命令必须逐条执行；等上一条输出并退出后，再执行下一条。**

```bash
ros2 topic echo /vlm_nav/system_ready --once --field data
ros2 topic echo /vlm_nav/nav_ready --once --field data
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
bash /home/isee-pst/unitree_ros2/VLM_Nav/scripts/inspect_go2_sport_publishers.sh
```

必须依次为 `True`、`True`、`DISARMED`、`False`，且脚本未发现活跃的外部
Move / SwitchJoystick。再次确认遥控器在手、目标方向空旷，然后执行：

```bash
ros2 service call /twist_to_go2_sport_bridge/arm \
  std_srvs/srv/Trigger "{}"
```

必须返回：

```text
success=True
message='ARMED'
```

继续确认：

```bash
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
```

必须为 `ARMED` 和 `True`。

**ARM 命令本身不会让 Go2 运动。执行本节后不要输入
DISARM，下一步是在 RViz 中选点。**

## 11. RViz：下发目标——这一步会让 Go2 运动

1. 点击 RViz 顶部的 `2D Goal Pose`。
2. 在第 8 节记录的候选目标附近按下鼠标左键。
3. 拖动箭头，使目标朝向与机器人当前 `+X` 大致一致，然后释放。

**释放 `2D Goal Pose` 箭头是本流程中触发真实运动的操作。**

Nav2 会先规划：

- 目标可行：生成 path，然后 Go2 直接开始闭环运动；
- 目标不可行：action 应失败/终止，Go2 不应移动；
- 不要在首个 goal 未终止时发送第二个 goal。

## 12. Terminal 2：运动期间监视

运动时可选择执行下列某一条，不要因盯终端而失去对机器人的观察：

```bash
ros2 topic echo /vlm_nav/go2_bridge_state --field data
```

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

正常现象：

- bridge 始终为 `ARMED`；
- Go2 平移速度不超过 `0.40 m/s`，非零角速度固定为 `0.50 rad/s`；
- 平移加/减速度限制为 `±0.50 m/s²`，角加/减速度限制为
  `±0.50 rad/s²`；
- DWB 逐分量过滤候选速度：平移只能为 `0` 或
  `speed_xy >= 0.40 m/s`，旋转只能为 `0` 或 `|wz| >= 0.50 rad/s`；
- DWB 用 `ObstacleFootprint` 检查每条模拟轨迹上的完整 Go2 footprint，
  footprint 与障碍接触的候选轨迹必须被否决；
- velocity smoother 将加减速过程中的 `|vx| < 0.40 m/s` 或
  `|wz| < 0.50 rad/s` 分量置零，因此实际 Sport 输出会在零与有效速度
  之间跳变，不应再出现持续的 `wz≈0.08 rad/s`；
- Go2 沿规划路径运动并在目标附近停止；
- `base_link` 中心与所选点的平面距离不超过 `0.25 m` 时，
  action status 进入 `SUCCEEDED`（status `4`）。

## 13. 取消或成功后继续选点

中途取消当前目标：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

等待 action 进入 `SUCCEEDED`（status `4`）或 `CANCELED`（status `5`），
并确认 Go2 停止。bridge 继续保持 `ARMED`，可重复第 8 节的
`Publish Point` 操作和第 11 节，选择并下发下一个目标。当前 action
仍为 `ACTIVE` 或 `CANCELING` 时不得下发新目标。

全部选点轮次结束后，在 Terminal 2 执行：

```bash
ros2 service call /twist_to_go2_sport_bridge/disarm \
  std_srvs/srv/Trigger "{}"
```

预期：

```text
success=True
message='DISARMED'
```

再确认：

```bash
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
```

必须为 `DISARMED` 和 `False`，且机器人停止后无持续运动。

最后在 Terminal 4 按 `Ctrl+C` 停止 rosbag，保留其输出目录用于验收。

## 14. 异常时的停止顺序

如果出现危险，立即使用遥控器人工急停。在终端可操作时，先 DISARM：

```bash
ros2 service call /twist_to_go2_sport_bridge/disarm \
  std_srvs/srv/Trigger "{}"
```

然后取消所有 Nav2 goal：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

不要在原因未明时重新 ARM。如 bridge 进入 `FAULT`，先查明日志中的
fault reason；仅在故障条件已消失后执行：

```bash
ros2 service call /twist_to_go2_sport_bridge/reset_fault \
  std_srvs/srv/Trigger "{}"
```

## 15. 验收标准

每轮 Nav2 手动选点测试按其终态验收：

1. 下发 goal 前 `SYSTEM_READY=true`、`NAV_READY=true`。
2. bridge 成功 `ARMED`、`control_armed=true`。
3. RViz goal 位于已知自由区，Nav2 生成有效 path。
4. `/cmd_vel_bridge` 与本桥 API 1008 Move 的方向一致。
5. API 1008 response `status.code=0`。
6. 无未知外部 Move / SwitchJoystick 干扰。
7. odometry 与现场观察均证明 Go2 沿路径向目标运动。
8. 成功轮次中 `base_link` 中心距所选点不超过 `0.25 m`，
   Nav2 action 成功；取消轮次进入 `CANCELED` 并安全停止。
9. StopMove 1003 成功，停止后无持续运动。
10. 最终 bridge=`DISARMED`、`control_armed=false`。

`sportmodestate.mode` 只作诊断记录，不作为 PASS/FAIL gate。

## 16. 正常关闭

1. 确认 bridge 已 `DISARMED`。
2. 在 Terminal 5 按 `Ctrl+C` 停止 bridge。
3. 在 Terminal 4 按 `Ctrl+C` 停止 rosbag。
4. 在 Terminal 3 按 `Ctrl+C` 关闭 RViz。
5. 在 Terminal 1 按 `Ctrl+C` 停止 `go2_system.launch.py`。
