# Go2 预设家具观察策略

实现入口仍是 `multi_agent_go2_check --action explore`，唯一运动执行链为
`go2_check → explore → Go2Adapter`。`fixed_viewpoints` 保持默认；
`--strategy semantic_furniture` 用带家具 ID 的地图选择家具，再解析预设观察点。
没有新增运动节点、global planner 或协同通信消息，coordinator 与 AgentMachine 不变。

## 当前配置与待确认项

[观察点目录](../../maps/lab_rear120_20261001_IwQjMX/observation_points_go2.yaml)
已按新记录的 1→2、3→4 分别建立 `table_01_view_01`、`table_02_view_01`，
绑定 table_01（四椅桌）与 table_02（并排桌椅）。坐标与朝向保留原点击，用户已完成 RViz 核对，当前 enabled=true；现场通过与否仍由实时门槛判定。原 go2_view_01/02 已移到 archived_viewpoints 保留，
不参与显示和候选选择；原 scene 固定点策略未变。
房间未标注，`current_room: null` 明确表示未知。

地图身份须与 alignment、robot_go2 的原图摘要一致；目录同时固定语义 YAML、标注 PNG
的 SHA256，修改标注后需人工核对并更新摘要。Go2 点位单位为米/弧度，直接属于本机 map，
不会再次套用 common-frame 变换；Ranger 不能复用这些坐标。

单机默认仍为检测到 A 后报告并结束。用户已授权本轮“寻找并接近灭火器”，新增显式
`--approach-target` 复用 Go2Adapter.select_approach 的安全候选、路径检查和原导航监督；
最新闭环版本按原 VLM_Nav 契约在0.89米请求停止、停稳后≤0.81米才成功，最多纠偏一次。
最新版本需加载 runbook 指定的 go2-arrival-contract-20261011 独立安装；
只有有效深度/TF投影才能接近，未定位的视觉线索会停止并报告。到达且确认停止报告
TARGET_APPROACHED；该本地实验不发布协同 READY。B/C 仍待现场填写。

本轮提示使用 `--search-hint '灭火器在 table_02 附近'`，只发给家具选择请求；
相机识别仍只接收目标“灭火器”，提示不能替代视觉确认。完整命令见 runbook 对应实验节。

## 离线检查

终端 4，仓库目录；以下不初始化 ROS、调用模型或运动。使用可导入本项目依赖的 Python：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD" \
  /home/isee-pst/venv/co-nav-real/bin/python -m multi_agent_nav.go2_check \
  --config-dir "$PWD/cooperative_ws/src/multi_agent_nav/config" \
  --offline --require-ready --strategy semantic_furniture
```

成功判据：退出 0，`semantic_search.points` 列出家具 ID → 观察点 ID → 本机位姿，
`single_robot_experiment_gates` 为空，`motion_sent: false`。
若退出 1，先修正报告中的配置问题再重复检查。
命令自动退出。此结果不证明现场可通行或相机/API有效。

构建和公共环境沿用 [runbook](go2_runbook.md)，不增加另一套安装目录。
未来获准的现场首轮沿用原四终端及 `--allow-motion` 入口，只追加
`--strategy semantic_furniture`，不自动开始实验。本轮不提供绕过现场门槛的脚本。

## 选择、观察与证据

模型只接受 `inspect_furniture`、候选家具 ID 和日志用 `reason` 三字段输出。
未知 ID、额外坐标、重复 JSON 字段、超时或非法响应不会生成导航目标。
程序先排除禁用/完成/冷却点，检查全局停车 footprint、ComputePathToPose 和资源范围；
同一家具有多个可达点时取路径长度最小者。模型返回后重新核对任务版本、停止状态、
取消代数和地图身份，选中点再规划一次；原导航期间安全监督仍有效。

复用原导航终态、物理停止、旋转、新鲜图像及深度/TF流程。到达不是观察完成。
`OBSERVATION_RECORD` 分别保存 unobserved/executing/observed/navigation_failed/
observation_failed；失败冷却 30 秒，无可用候选时报告 `WAITING_VIEWPOINTS` 并退出等待人工处理，
不会常驻轮询模型。有效观察过的点在本轮不会重选；全部有效完成报告 `EXHAUSTED`，
仅表示当前预设视角已检查，不能断言家具所有位置或目标不存在。

`events.jsonl` 中 `FURNITURE_REQUEST` 保存任务、候选、历史与地图图片路径，
`FURNITURE_SELECTED` 保存家具/点位/原因，失败保存 `FURNITURE_SELECTION_FAILED`。
选择请求与原 `vlm/` 相机帧及 ABC 识别证据分开；不把语义地图混入相机 scan 目录。
`result.json` 保留原执行结果与停止确认。API 密钥不写日志。

结束顺序沿用原实验流程：先让终端 4 停止并检查停止确认，再按 runbook 关闭观察界面和底层栈。
未确认停止时不能启动下一轮。该语义策略实现尚未进行真机/API验收。

## RViz 家具与停靠点核对（离线）

使用三个新 Bash 终端 A/B/C。每个终端先执行以下环境块；本流程不要 source
CoVLM_env.bash 或 go2_network_env.sh，它们用于 Domain0 真机环境。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$PWD/build/go2-field/evidence/annotation-ros-logs"
mkdir -p "$ROS_LOG_DIR"
```

终端 A：启动固定地图，保持运行。如已有本流程的 map_server，则复用，不重复启动。

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __ns:=/go2_point_annotation -r __node:=map_server \
  -p use_sim_time:=false \
  -p yaml_filename:="$PWD/maps/lab_rear120_20261001_IwQjMX/lab.yaml"
```

终端 B：依次执行 configure、activate，每一步显示 `Transitioning successful` 才继续。
已有节点先用 `ros2 lifecycle get /go2_point_annotation/map_server` 检查状态，active 时跳过转换。

```bash
ros2 lifecycle set /go2_point_annotation/map_server configure
```

```bash
ros2 lifecycle set /go2_point_annotation/map_server activate
```

然后检查地图消息；预期 width=222、height=312、resolution=0.05。无输出或不一致先排障，不继续。

```bash
timeout 10 ros2 topic echo /go2_point_annotation/map nav_msgs/msg/OccupancyGrid \
  --once --field info --qos-reliability reliable --qos-durability transient_local
```

同一终端 B 启动标记发布器并保持运行，预期打印“家具/观察点标记已发布”：

```bash
python3 scripts/show_semantic_viewpoints.py \
  --observation-yaml maps/lab_rear120_20261001_IwQjMX/observation_points_go2.yaml
```

终端 C：打开 RViz 并保持运行。

```bash
rviz2 -d config/go2_semantic_annotation.rviz
```

预期 Fixed Frame=map，Map 与 Furniture and viewpoints 两个显示均为 OK：

- 蓝色 table_01、橙色 table_02 是家具轮廓与 ID。
- 当前 table_01_view_01、table_02_view_01 箭头表示新停车位置与点击方向，分别呈蓝/橙色。
- `UNBOUND / disabled` 表示未绑定且禁用。箭头仅表示初始朝向，不代表相机视场或已验证视线。
- 配置填入已确认的 furniture_id 后，箭头使用对应家具颜色，并显示连接家具中心的细线；
  连线只表示绑定关系，不是导航路径。disabled 点同样可见，不需要为看图而启用。

核对后确认 table_01_view_01、table_02_view_01 的停车位置及箭头朝向是否符合预期。工具不按距离
自动绑定、不写 YAML、不发布导航目标。修改观察点 YAML 后，终端 B Ctrl+C 再运行同一命令，
MarkerArray 首先清除旧标记，RViz 自动刷新。Publish Point 工具保留，但标记程序不录点；
需要新点时另用既有录点脚本，按停驻中心→方向点采集，不混入家具角点记录。

结束：终端 B Ctrl+C 停止标记发布器，再关闭终端 C 的 RViz，最后终端 A Ctrl+C 停止地图。
这套流程不需要定位、导航、桥接或机器人节点。

只检查文件、不启动 ROS：在仓库目录执行上述标记命令并追加 `--check`；自动退出0且列出
两个原位姿为成功。摘要、地图版本、点位非有限或无效绑定会拒绝加载。此查看器允许未绑定
的 disabled 点，与执行策略的严格准入条件不同。


## 2026-10-13：generated_candidates_vlm

新策略接入同一个 `next_point` 位置；默认仍是 fixed_viewpoints，semantic_furniture 也保留。
公共模块 `multi_agent_nav/generated_candidates.py` 负责几何、双图和严格 ID 解析；
Go2 adapter 只增加现有 API 客户端的选点请求及全局旋转空间检查。
coordinator、任务协议、READY/C 阶段、底层传感器/定位/Nav2 参数未改。

配置位于 `cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml`：
`generated_candidates.semantic_yaml` 指向现有家具标注，`observation_distance=1.0` 米是
家具每条边中点到机器人中心的外侧距离，尚未现场认证；每边最多一候选，不沿边密集撒点。
对两种多边形绕向均生成外法线，朝向对应边，拒绝自交、退化及非有限标注。
按源地图 YAML 阈值读取 trinary occupancy；拒绝边界、未知、占用、家具范围相交及重复点。
任务内缓存几何，每轮及运动前核对地图/语义文件摘要，变化则停止，不沿用旧坐标。

检查使用现有矩形 footprint x±.38/y±.18、既有 footprint_padding=0，完整转动盘半径约
.420476 米；栅格方块与转动盘相交时也检查，不只检查格中心。实时 global raw costmap
沿用253/254/255否决、原新鲜度及TF门槛；ComputePathToPose 和 RegionGuard 继续检查路径。
stop_margin 仍只用于现有资源停止范围，不额外加到停车盘，也不更改 costmap 膨胀。
远处候选不要求落在 local costmap 窗口；本机当前位置仍需原局部健康/路径监护。
展示前再更新候选可用状态，最多12点；选择后重规划与重查，实际到点停稳后再核对转动盘。
Nav2 自身旋转碰撞否决保持有效，预检查不保证导航途中环境永不变化。

每轮在证据目录 `selection/<decision_id>/{occupancy,semantic}.png` 写新图，不覆盖地图。
两图同全图范围、3倍缩放、相同坐标转换，共用同一标记层：蓝色 Go2 点/朝向、橙色
P1…P12、灰色已观察点和不可选图例。语义图保留家具多边形、ID和名称。
标签优先放在家具空白处或宽敞空闲区，避开家具名称和点标记；必要时加细引导线。
图内没有坐标或数值坐标轴。实际地图离线样图位于
`build/go2-field/evidence/generated-candidates-20261013/preview-v2/`，其中机器人位姿为
演示输入、橙点只经过静态筛选；不是实机当前位姿或实时可执行候选证据，也未发送API。

`CANDIDATE_GEOMETRY` 保留内部 ID→家具/位姿，`CANDIDATE_REQUEST` 保存本轮展示 ID→内部 ID、
图像路径和观察状态。只有严格允许的 `{"candidate_id":"P3","reason":"…"}` 被接受；
重复字段、额外坐标字段、未知 ID、超时、取消、地图变化及旧版本响应不能触发导航。
请求通过现有 AsyncVision 绑定任务、唯一决策、版本和取消代数，沿用原 API 超时处理
及 SDK max_retries=0，不新增后台重试。给API的白名单只有双图、目标/提示、图例、允许P编号、
任务/决策身份及最近12条简短记录；候选位姿、路径、内部ID、家具多边形坐标不发送。
历史P编号附历史决策 ID，不能当作本轮同名点；已观察点在图中为灰色不可选。

新策略旋转识别 ABC；用户本轮指定 B=水杯、C=篮子，A沿用灭火器，只更新描述。
`scene.yaml` 的 A/B/C 必须填写真实目标描述，B/C占位会被明确拦截。
有效全方向ABC扫描才记为 observed；到达、旋转异常、图像/API/结构错误不算完成。
若其他方向/目标失败但已有有效 A 检出，保留该检出并进入原目标接近，观察点仍未完成。
接近沿用 vlm_nav.arrival 距离闭环与原停车/路径检查，不置 READY。
失败保留 navigation_failed/observation_failed 与30秒冷却；无可执行点即退出
WAITING_VIEWPOINTS（候选暂不可达或观察失败），不常驻轮询；全部完成为 EXHAUSTED
（当前视角已检查完），均不宣称目标不存在。

安装、离线检查及今后获授权后的启动顺序统一见
[runbook 新策略小节](go2_runbook.md#2026-10-13生成候选双图vlm选点)。
