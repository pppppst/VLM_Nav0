# Go2 现场接入准备与单机 adapter 验收

基准：`go2-cooperative-task-layer` / `1ba2589`。本轮读取代码、修正接口并做
无硬件测试；下面的 Domain0 命令交给现场执行，本轮没有连接真实 ROS 图、
Ranger、bridge，也没有发送真机目标。完整协同仍需两台真实 agent、标定和场景。

## 文件、当前缺项与填写方式

配置源目录：`/home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/config`。
独立检查实现：`multi_agent_nav/go2_check.py`；adapter：`adapters/go2.py`；
完整协同门槛：`config.py:live_gates`，单机门槛：`go2_local_gates`。

| 门槛 | 实际文件 / 字段 | 当前缺项 | 填写与通过条件 |
|---|---|---|---|
| 地图身份 | `alignment.yaml`: `robots.{go2,ranger}.{local_map_id,map_version}`, `alignment_id`; `robot_{go2,ranger}.yaml`: `map_path,map_digest` | ID/version/路径/摘要均未填；当前加载地图未查 | 每台 map_path 是本机真实 YAML 绝对路径；摘要含 YAML 与 image 文件，按下方离线工具输出填写。重建地图改 version 并重新标定。Go2 检查本机文件，Ranger 文件留给 Ranger PC 校验，不要求复制到 Go2 |
| 地图对齐 | `alignment.yaml`: `common_frame_id,validated,report,robots.*.transform` | Ranger transform/report/alignment_id 缺失 | lab_ref 数值等于 Go2 map，Go2 固定 `[0,0,0]`；Ranger 用实测对应点及独立检查生成 report，完整接口仍会重算 fit、核对 map identity/transform；不得只改 validated |
| 场景点位 | `scene.yaml`: `targets`, `viewpoints.*[{id,pose}]`, `wait_points.*`, `exit_points.*`, `resources.*.polygon`, 非 basket 的 `transit_exits.*` | A/B/C 文本为占位、点位/区域全空，enabled=false | 共同 lab_ref 下测量；pose=`[x米,y米,yaw弧度]`，polygon=`[[x,y],...]`。两台均需观察/等待/退出点，basket 必须存在，门需两台的通行出口。ID 不重复、数值有限、区域非退化；未知继续空，不填演示数值 |
| 执行路径源 | `robot_go2.yaml`: `path_topic,path_source_verified,path_max_age,path_start_timeout` | topic=null、verified=false | 需要与本机当前 NavigateToPose/FollowPath 执行及重规划对应的 map-frame `nav_msgs/Path`。`/plan` 或 ComputePath 预检结果不能单独作为证明；下方先发现来源，拿到现场证据后才填 topic/verified |
| 停止确认 | `robot_go2.yaml`: `odom_topic,stationary_linear_speed,stationary_angular_speed,stationary_duration,odom_max_age,cancel_timeout,stop_margin` | `/odometry` 已静态确认；阈值 0.02 m/s、0.02 rad/s、持续0.5s、最大龄0.5s、取消确认预算5s为当前参数；**stop_margin=null**，物理停止未测 | adapter 同时检查消息时间与接收时间；线/角速度分别用三维范数。必须 action 终态且连续新鲜低速，Cancel ACK、发零速、停止目标点都不是证明。停止余量包含真实速度/制动、反馈/监督/取消延迟及对齐误差，只填实测保守值 |
| 实时 costmap | `robot_go2.yaml`: `costmap_topic,costmap_source_verified,costmap_max_age,costmap_blocked_cost,footprint` | 实际 topic=`/local_costmap/costmap_raw`；verified=false；最大龄1s；阻挡阈值253；footprint继承实测±0.38×±0.18m，动态能力尚未验收 | `nav2_msgs/Costmap` 完整 payload、size/resolution/origin/时间戳有效；在 payload 时刻把 map 候选变换到 costmap frame，再处理 origin yaw。253/254及255未知格均阻止 footprint 接近，最后仍做 ComputePath；失效 TF/过期数据阻止推进 |
| 相机/识别/动态障碍 | `robot_go2.yaml`: `camera_calibrated,vision_validated,peer_obstacle_verified,model,endpoint,api_key_env,api_timeout`；旧 `config/robot_go2.yaml` 的 RGB-D、depth_units_report | 三个验证标记 false，model/endpoint未填；现有 `config/go2_calibration_xt16.yaml:camera_extrinsic.calibrated=false` | 实测相机外参/depth单位/ABC结果与投影、真实机身 marking/clearing 后再设置对应标记。凭环境读取 key，不写 YAML/日志。独立 scan 用于取得 vision 验收证据，因此不预先要求 vision_validated=true；完整协同仍要求它 |
| 运行许可 | 旧 `/vlm_nav/nav_ready`, `/vlm_nav/control_armed`；NavigateToPose/ComputePathToPose/Spin | 本轮未观察，未 ARM | 单机运动也要求现有许可、定位、相机/action/costmap健康。排除其他上层目标控制者；检查入口会拒绝与旧 vlm_nav 或协同节点并行运动，不发布任务消息，也不产生 Ranger READY |

仅开关 scene.enabled、verified 或 validated 不足以启动完整 live；配置路径拼错
现在会明确失败，不再悄悄加载默认模板。map_digest 算法是
`SHA256(yaml_sha256_hex + image_sha256_hex)`，拼接按 UTF-8 编码；摘要不能替代
确认实际 `/map_server.yaml_filename`。单机入口每次实际验收会查询该参数并核对路径。

**AMCL 动态障碍缺口：** 实际 `launch/go2_navigation.launch.py` 在 AMCL 下最后合并
`config/nav2_go2_static_map.yaml`。该文件把 global/local 的 plugins 均替换为
`[static_layer,inflation_layer]`；`config/nav2_go2.yaml` 中的 obstacle/voxel marking
和 clearing 因而不能仅凭源文件存在就视为已启用。旧成功栈保持原样；协同现场
必须另行审查并验证 active dynamic layer 后才设置 costmap_source_verified 与
peer_obstacle_verified。消息持续更新不能证明检测到了动态障碍。

## 构建与离线配置清单（本轮可执行，无 DDS participant）

先按 [go2_runbook.md](go2_runbook.md) 的同一 `/tmp/go2-coop` 输出目录重新构建，
使用 system Python 生成、co-nav-real 运行。然后：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source /tmp/go2-coop/install/setup.bash
ros2 run multi_agent_nav multi_agent_go2_check --help
ros2 run multi_agent_nav multi_agent_go2_check --offline
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready
# 最后一条当前预期 exit 1：列出完整协同缺项，无 ROS 初始化。
```

准备独立现场配置副本，避免直接修改安装模板；`/tmp` 丢失后需要重建/恢复：

```bash
mkdir -p /tmp/go2-field/config /tmp/go2-field/evidence
cp cooperative_ws/src/multi_agent_nav/config/{alignment,scene,robot_go2,robot_ranger}.yaml /tmp/go2-field/config/
export GO2_FIELD_CONFIG=/tmp/go2-field/config
```

库内找到 `maps/lab/lab.yaml`、`maps/lab_candidate_20261001_AINSKh/lab.yaml`、
`maps/lab_rear120_20261001_IwQjMX/lab.yaml` 三份候选；存在不等于正在加载。
确认活动路径后把它赋给 GO2_FIELD_MAP，再生成摘要，按输出填写副本的
robot_go2.map_path/map_digest 与 alignment.robots.go2 的 ID/version：

```bash
: "${GO2_FIELD_MAP:?请先填写实际活动地图YAML绝对路径}"
ros2 run multi_agent_nav multi_agent_go2_check --offline --config-dir "$GO2_FIELD_CONFIG" \
  --map-yaml "$GO2_FIELD_MAP" > /tmp/go2-field/evidence/map-and-gates.json
```

Ranger 对应点采集与 report 生成沿用
`ros2 run multi_agent_nav multi_agent_calibration references.yaml`；输入字段为
source_points/common_points/held_out_pairs/thresholds/identities。本轮不生成真实标定。

## 现场只读命令（本轮未运行）

只访问 Go2 本地域0/enp4s0，针对**已有且禁止运动的底层栈**。不要用
manualnav2.sh 或 VLMNav-go.sh 来启动“只读采集”：这些脚本含硬件/ARM流程。
下面只查询已运行接口，不启动底层、不 ARM、不启动协同或 bridge：

```bash
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /tmp/go2-coop/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_NET_IFACE=enp4s0
export CYCLONEDDS_URI=file:///tmp/go2-coop/install/vlm_nav/share/vlm_nav/config/cyclonedds_go2.xml
export ROS_LOG_DIR=/tmp/go2-field/evidence/ros
export GO2_FIELD_CONFIG=/tmp/go2-field/config
```

独立 adapter 默认 inspect，观察15秒，**没有速度发布器、不调用动作、不取消别人目标**：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --timeout 15 > /tmp/go2-field/evidence/adapter-readonly.json
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --timeout 15 --require-ready
```

普通 inspect 的 exit 0 只表示成功输出诊断；--require-ready 才要求本机
地图文件/活动路径、定位、静止和 costmap 就绪。报告单独保留所有完整 live 缺项。
`own_motion_terminal` 只描述这个进程持有的句柄，**不认证其他客户端动作已终态**。
入口使用 Humble 实际 GetParameters 服务，不依赖较新 rclpy 的 parameter_client。
只读 observer 使用独立节点名，不占用运动锁。运动入口用本机文件锁互斥，拒绝与
协同/旧目标控制者（包括 agent 默认名称）并行；发送前再次核对许可、静止、终态、
costmap 与控制者，规划期间失效不会先发目标再取消。

逐项交叉检查：

```bash
# 地图身份、TF。记录活动文件后，用离线命令计算其摘要。
ros2 param get /map_server yaml_filename
ros2 topic info /map -v --no-daemon
timeout 5 ros2 run tf2_ros tf2_echo map base_link

# 场景采点：在Go2 map/lab_ref记录实测点位；检查 footprint 与区域出口的整机间隙。
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint

# 路径源发现和BT：这里只观察，不发送 Navigate/FollowPath/Spin 目标。
ros2 topic list -t --no-daemon | rg 'nav_msgs/msg/Path'
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 action info /navigate_to_pose
ros2 action info /compute_path_to_pose
ros2 action info /spin
# 发现候选后填变量；发布者、frame、stamp和重规划关联均要验证。
: "${GO2_PATH_CANDIDATE:?填写发现的路径话题，不能猜测}"
ros2 topic info "$GO2_PATH_CANDIDATE" -v --no-daemon
timeout 5 ros2 topic echo "$GO2_PATH_CANDIDATE" nav_msgs/msg/Path --once --field header

# 停止确认：QoS与消息时间，连续静止由adapter判定；状态数组需匹配实际goal UUID。
ros2 topic info /odometry -v --no-daemon
timeout 5 ros2 topic echo /odometry nav_msgs/msg/Odometry --once --qos-reliability best_effort
timeout 5 ros2 topic echo /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray \
  --once --qos-durability transient_local
timeout 5 ros2 topic echo /spin/_action/status action_msgs/msg/GoalStatusArray \
  --once --qos-durability transient_local

# costmap：确认active plugins而非只看YAML；raw消息是Costmap，不是OccupancyGrid。
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /local_costmap/local_costmap global_frame
ros2 param get /local_costmap/local_costmap always_send_full_costmap
ros2 topic info /local_costmap/costmap_raw -v --no-daemon
timeout 5 ros2 topic echo /local_costmap/costmap_raw nav2_msgs/msg/Costmap \
  --once --field metadata --qos-durability transient_local
ros2 topic info /vlm_nav/obstacle_cloud -v --no-daemon
```

路径候选若来自 planner_server，要证明预检 ComputePath 不会冒充正在执行路径；
FollowPath 的实际目标是 action 请求，不是反馈中的距离/速度。没有可验证来源就保留
path_topic=null/path_source_verified=false，本轮没有新增或假装存在的 Nav2 exporter。
当目标为空、动作从未运行时，路径/status没有消息并不自动说明接口坏了，也不构成验收。

costmap验收在禁止运动时可用可控实物检查 marking/clearing，确认 full payload 下
footprint 接近点会被排除；随后断输入/使 TF 或 stamp失效应报告不就绪。需有 dynamic
layer，不能只观察静态地图。停止距离/取消后终态等涉及运动的验收另行开展。

## 无运动的 ComputePath 与候选检查（本轮未对真机执行）

填写现场测量的局部 Go2 map 坐标；这些动作会请求规划，但不会发送导航或旋转目标。
approach 的 X/Y 表示物体位置，筛选实际 costmap 后逐个验证本机可达性。

```bash
: "${GO2_TEST_X:?实测map x}" "${GO2_TEST_Y:?实测map y}" "${GO2_TEST_YAW:?实测yaw弧度}"
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action plan --goal "$GO2_TEST_X" "$GO2_TEST_Y" "$GO2_TEST_YAW" --timeout 15
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action approach --goal "$GO2_TEST_X" "$GO2_TEST_Y" "$GO2_TEST_YAW" --timeout 15
```

## 单机运动验收命令（本轮禁止执行，仅交接）

仅在单机运动现场条件和现有许可已满足、其他上层目标控制者退出后显式选择。
入口不会替用户 ARM；它没有资源授予权，导航路径/旋转停止包络一旦碰到任何受控门/C
区域即拒绝或停止。单机验收不需要 Ranger 的坐标、READY或标定，不启动 coordinator。
这不是完整协同 live 门槛的替代入口，也不修改共享任务状态。

```bash
# 相同实测坐标变量；Navigate另要求已验证执行路径源。
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action navigate --allow-motion --goal "$GO2_TEST_X" "$GO2_TEST_Y" "$GO2_TEST_YAW" --timeout 30
# 验证接受前/接受后的取消及物理停止；2秒是请求取消时刻，不是声称2秒必定停稳。
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action navigate --allow-motion --goal "$GO2_TEST_X" "$GO2_TEST_Y" "$GO2_TEST_YAW" \
  --cancel-after 2 --timeout 30
# scan会依配置scan_angles进行Spin，再请求ABC API；先填真实A/B/C描述、相机和API配置。
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action scan --allow-motion --timeout 120
```

退出/超时/SIGINT只取消本进程句柄并尝试停止；必须同时报告 stop_confirmed=true。
未确认停止返回失败，不发后继目标；报告 last_native_action_uuid 可对照原生 status。
本机 `/tmp/go2-coop-acceptance-motion.lock` 在正常完成后清空；停止未确认或进程被
强杀或异常清理时保留阻断标记（即使可能尚未发送动作，也需人工核对）。后续单机
运动入口拒绝运行，不能直接删除“解锁”。现场先按
原流程取消/停机并确认实际动作终态及机身停稳，再保留证据，人工清空标记：

```bash
# 仅用于现场已完成真实停止核对之后；本轮未执行。
truncate -s 0 /tmp/go2-coop-acceptance-motion.lock
```

若本进程停止未获确认，沿用现有现场停机/人工处理程序，不能把程序退出当作机身已停。
完整双机标定、Ranger adapter、bridge路由/隔离、互为障碍、门/C顺序仍是后续独立验收，
本轮完整协同模板继续关闭。
