# Go2 控制电脑会话交接（2026-10-08）

## 0. 来源、工作区及阅读顺序

当前电脑为 Go2 控制电脑；仓库 `/home/isee-pst/unitree_ros2/VLM_Nav`。
实查分支 `go2-cooperative-task-layer`，HEAD `ce2999a4ace8409a4244e4aac02080e9ad93f479`。
祖先：`2575a54`现场门槛、`1ba2589`任务层交付；历史分支名`VLM_nav_go`不是当前分支。
初次交接时未发现适用 AGENTS.md；后续已新增根目录[AGENTS.md](../../AGENTS.md)，作为长期协作规则入口。
初次写本文件前的历史工作区：`docs/multi_agent/dynamic_costmap_stage.md`已修改；
未跟踪：根目录两个输入方案、`scripts/run_ab.py`、`test/check_go2_diagnostic_monitor.py`。
后续另新增本文、AGENTS.md及退出排查补充；当前状态每次以git status为准。
保留上述文件及所有日志/bag；此前阶段仅整理文档。新增授权后的隔离验证及修改见第7节；不提交。

- 权威架构：[ranger_go2_codex_plan.md](../superpowers/plans/ranger_go2_codex_plan.md)。
- 权威执行方案：[2026-10-07-go2-cooperative-navigation.md](../superpowers/plans/2026-10-07-go2-cooperative-navigation.md)。
- 两份根目录输入文件与对应docs方案逐字节相同；根目录副本仍未跟踪。
- 运行/现场：[go2_runbook.md](go2_runbook.md)、[go2_field_checklist.md](go2_field_checklist.md)、[dynamic_costmap_stage.md](dynamic_costmap_stage.md)。
- 退出问题静态核对及新增隔离证据：[costmap_shutdown_handoff_20261008.md](costmap_shutdown_handoff_20261008.md)。
- 实现/历史验证：[progress.md](progress.md)、[test_results.md](test_results.md)、[field_preparation_results.md](field_preparation_results.md)、[ranger_handoff.md](ranger_handoff.md)。
- 原单机指引：[VLM_Nav初步启动指引.md](../VLM_Nav初步启动指引.md)。

方案中的“尚未实现”是撰写时状态，不代表当前代码缺失；runbook/现场旧记录中的
“动态层仍被覆盖”“未接Go2网线”及旧缺项数量也不是当前结论。下文分设计、实现、证据。

## 1. 当前架构与部署边界

**设计：**两机器人各有控制电脑、各自地图和Nav2；Go2地图数值作为`lab_ref`。
Ranger→lab_ref使用实测SE(2)，不共享occupancy、不桥接TF；Ranger真实标定未完成。
coordinator部署Go2控制电脑，使用RulePlanner，不增加global VLM planner。
Go2找A、Ranger找B；每次观察同时检查ABC并共享有效位置，各自判断本机可达性。
只有自身A/B当前主接近任务Nav2成功才READY；过门/观察/等待成功不置READY。
双方新鲜READY屏障后使用同一C；Go2先接近、模拟放入、整机退出，随后Ranger。
C许可贯穿放入与退出：EXIT保留原`parent_command_id`和token，整机退出确认后释放。
门/C互斥、失联暂停及本地图路径监护详见权威架构，不在此重新定义协议。

**实现：**`cooperative_ws/src/multi_agent_nav/multi_agent_nav/`内已有
`coordinator.py`、`planner.py`、`agent.py`、`resource_manager.py`、`region_guard.py`、
`frame_adapter.py`和`adapters/go2.py`；[coordinator.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/coordinator.py)、[agent.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py)维护许可身份。
新包`multi_agent_task_msgs`与独立任务话题已实现，旧`multi_agent_msgs`及
`/multi_agent/agents/{go2,ranger}/state`保留；旧里程计state不是任务READY。
新话题为`/multi_agent/{go2,ranger}/{state,observation}`和`/multi_agent/go2/command`。
**验证：**公共任务逻辑、Go2适配及双fake已有离线证据；真实任务适配运动、Ranger和联合任务未验收。

**当前部署约定（并非本次网络实测）：**

- Ranger PC Domain0 ↔其定向bridge↔共享Domain40↔Go2定向bridge↔Go2 PC Domain0。
- Go2 Domain0仅`enp4s0`；Domain40仅`wlo1`；coordinator与Go2 live agent均在本机Domain0。
- Domain31仅隔离mock/回放，不接bridge，不迁移已验证底层域；旧30↔40↔31不用于live。
- 定向路由按PC来源和话题区分，禁止两端双向回灌；待部署配置为
  [bridge_go2_pending.yaml](../../cooperative_ws/src/multi_agent_nav/config/bridge_go2_pending.yaml)、[bridge_ranger_pending.yaml](../../cooperative_ws/src/multi_agent_nav/config/bridge_ranger_pending.yaml)。
- DDS文件：`config/cyclonedds_go2.xml`（本地）、`config/cyclonedds_go2_multi_agent_bridge.xml`（按域网卡）。

**用户最新补充（2026-10-08，未做网络/进程现场核验）：**Go2 enp4s0=`192.168.123.222`，
Go2上位机=`192.168.123.18`；Go2 wlo1=`192.168.1.118`，Ranger PC wlo1=`192.168.1.108`。
Wi-Fi承载共享Domain40；Go2的Domain0不因控制电脑也有wlo1而改走Wi-Fi。
此拓扑与本机bridge XML的Domain0=enp4s0、Domain40=wlo1静态配置一致；不代表已部署或本次实测通过。
用户描述的`/odom`→ranger_agent_node、`/odometry`→go2_agent_node→`/multi_agent/agents/*/state`
是保留的旧里程计转发链，不替代新任务agent或新任务话题。
用户报告网页步骤中Go2电脑收到了小车底盘信息，但看不到Go2 `/odometry`，疑有两个订阅者、无发布者。
具体端点/进程记录未提供，订阅者身份、当时输入链状态及双向连通均待核实，不据此写通过。
静态核对`scripts/start_go2_agent.sh`仅启动旧agent；其源码
`/home/isee-pst/unitree_ros2/multi_agent_comm/multi_agent_comm/go2_agent_node.py`
只在收到`/odometry`时转发state，不启动传感器或FAST-LIO，也不自行生成里程计。
**网页端步骤待补：**先注明两端本地域环境及本地里程计前置入口；Go2须有Hesai/机身IMU、转换器与FAST-LIO输入链，
Ranger须有真实`/odom`源。先核实本地真实发布者、新鲜消息和QoS，再按授权验证旧agent、定向bridge及对端收到的新鲜state；
不直接使用含录包/运动流程的旧脚本作纯通信前置步骤。无发布者时停止推进；两个订阅者不等于数据链路连通。
本次只补文档，不启动驱动、ROS、桥接或跨机检查；现有完整协同live门槛保持关闭。
后续只读核对已把输入顺序、历史QoS、旧agent源码语义及有界核对步骤补入
[runbook的本地里程计前置核对](go2_runbook.md#go2-本地里程计与旧状态转发前置核对)。
旧agent输入请求RELIABLE，输出旧state为BEST_EFFORT，healthy固定true；静态确认旧安装的
`/home/isee-pst/unitree_ros2/install/multi_agent_comm/lib/python3.10/site-packages/multi-agent-comm.egg-link`
指向`/home/isee-pst/unitree_ros2/build/multi_agent_comm`，模块软链接解析到上述源码。
实际运行模块/端点及桥接QoS仍未核实，不能把旧state字段当定位健康或READY证据。

## 2. 当前静止链路、有效参数与证据范围

当前可用结论：**Go2感知—定位—地图—costmap静止链路已贯通**，不等于完整自主导航或双机协同通过。
本轮使用原版FAST-LIO；实际启动参数和可执行路径见最新`manifest.json`，未按源码默认值反推。
证据根目录`E=/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-field/evidence/fastlio-retest.1_h_1z2p`。
最新完整轮次`S=$E/static-chain-repeat.skw8amvz`；build目录未受Git保护，须保留。

| 环节 | 实际节点、话题与配置 |
|---|---|
| Hesai XT16 | 控制电脑`/hesai_ros_driver_node`→`/lidar_points`，frame=`hesai_lidar`；SDK配置`/home/isee-pst/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml`；`$S/driver-params.yaml`只指定config_path |
| 机身IMU | Go2已有`/lowstate`→本机`/lowstate_imu_adapter`→`/body_imu`，frame=`body_imu`；不是用雷达IMU；源码`src/lowstate_imu_adapter.cpp` |
| 转换器 | `/hesai_fastlio_converter`：`/lidar_points`→`/lidar_points_fastlio`，实际参数`$S/converter-params.yaml`；timestamp FLOAT64秒→逐点相对时间；输入best_effort、输出reliable，未改QoS |
| FAST-LIO | `/lio_mapping`，原`go2_ws/install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping`；参数`$E/spark_fast_lio_go2_xt16.yaml`；lidar/imu remap为上述两话题；输出`/odometry`、`/cloud_registered_base`等 |
| 障碍/scan | `/obstacle_cloud_filter`：`/cloud_registered_base`→`/vlm_nav/obstacle_cloud`；`config/robot_go2.yaml`；`/go2_cloud_to_scan`→`/scan`，`config/go2_laserscan.yaml` |
| 定位/地图 | `/map_server`加载下述lab.yaml，`/amcl`使用`config/go2_amcl.yaml`并显式`scan_topic:=/scan`；源码默认`/scan_amcl`未改；`/initialpose`来自RViz |
| 独立costmap | `/go2_costmap_acceptance/{local,global}_costmap/{local,global}_costmap`；各自`costmap_raw`同前缀；只有costmap及生命周期，无Nav2运动action服务器 |

TF链：`map→odom`由AMCL，`odom→base_link`由FAST-LIO；静态
`base_link→hesai_lidar` xyz=[0.14543,0,0.13312]、yaw=1.570796；
`base_link→body_imu` xyz=[-0.02557,0,0.04232]、rpy=0。
这些值来自`$E/go2_calibration_xt16.yaml`，不是Ranger标定或场景坐标。
FAST-LIO实际`lidar_type=2, scan_line=16, timestamp_unit=0, extrinsic_est_en=false`；
内部LiDAR→IMU外参与上述ROS静态TF不是同一个变换；`/**`参数文件仅传给该节点。
障碍过滤高度仍是base_link的[-0.20,0.20]m，未放宽；所有真机链路使用wall time。

最终costmap加载顺序：安装的`nav2_bringup/params/nav2_params.yaml`
→vlm_nav安装的`config/nav2_go2.yaml`→`config/nav2_go2_static_map.yaml`
→multi_agent_nav安装的`config/nav2_go2_cooperative_costmap.yaml`。
共享合并实现`vlm_nav/go2_nav2_config.py`；普通Nav2需显式`costmap_overrides_file`，旧默认未改。
静止launch从ament安装share读取后生成`/tmp/go2_costmap_acceptance_*.yaml`，不是直接读取源码YAML。
本轮查验源码/当前安装的四份任务配置、障碍/AMCL及最终costmap覆盖内容一致；
实际运行参数另存`$S/runtime-{local,global}-costmap.yaml`，local=static/voxel/inflation，global=static/obstacle/inflation。
动态源都是`/vlm_nav/obstacle_cloud`，`marking/clearing=true`；参数true不等于效果已定量验收。

Go2活动地图：`/home/isee-pst/unitree_ros2/VLM_Nav/maps/lab_rear120_20261001_IwQjMX/lab.yaml`，引用`lab.pgm`。
身份`lab_rear120_20261001_IwQjMX`；map_digest=`c9f354a26471032e7ff10b5898db627b65fce7526c28124891833b6a3a164c02`，版本为`sha256:`加该值。
本次只读重算一致；摘要规则为SHA256(YAML摘要字符串+图像摘要字符串)，见`multi_agent_nav/config.py`。

相机是Go2上位机既有D435，不是控制电脑USB；旧进程记录显示Foxy、Domain0、640×480×15、sync=true、alignment=false。
现有启动命令来自单机指引：`ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15 enable_sync:=true align_depth.enable:=false`（仅记录，不在本机执行）。
输入为`/camera/camera/color/image_raw`、`/camera/camera/depth/image_rect_raw`及各自`camera_info`。
视觉链路为`adapters/go2.py:make_observer/ABCClient`→已有`VLMNavigator`/RGB-D同步与`RealSenseGeometry`→图像时间TF投影→ABC结果；不是global planner。
本轮只确认相机进程/端点和参数，未运行ABC识别，也未验收RGB/深度/相机TF；
`~/.config/vlm_nav/raw_depth_units_verified.json`存在，不代表本次新鲜视觉验证。

最新窗口182.39秒、无录包/完整点云监测；AMCL接受实测初始位姿，地图与两个costmap激活。
障碍输入约10Hz、2692—3542点；基线后odom最大累计2.18cm、最大单步8.14mm（不是外部真值）。
操作者确认移动障碍时“原位置消失，新位置出现”：所观察costmap具有定性标记/清除响应，未指明local/global分别表现。
原全图新增格分散在8/16个区域，0/38、1/26不能当作该实物清除率；指定ROI定量验证仍待完成。
操作者确认RViz出现后由趴姿变站姿：约0.34m启动位移包含真实运动，不记为静止漂移；转换起止时间未记录。
最新节点已停止，已有机载相机未被停止；本交接不探测当前进程或网络状态。

## 3. 排障结论、证据索引与限制

- **已证实监测干扰：**临时两路完整点云监测参与异常；同初始10.93s窗口发布中位257.67→0.933ms、最大位姿单步0.590→0.0121m，跨度拒绝11→0。
  移除后唯一60s复测发布中位0.892ms，未见No Effective Points；证据`$E/subscriber-triage.20sk5e31/verified-result.json`。
- `scripts/run_ab.py`现默认只保留odom/lowstate监护，两路完整点云须`--full-cloud-monitoring`；离线构造检查通过。不要再运行build中的旧默认监测版。
  此开关仅控制两路额外诊断订阅，不关闭必要转换/定位/costmap输入；开启须在任务授权范围内，不能自行追加试验。
  必需转换器、LIO、障碍过滤及costmap订阅继续保留；不增加常驻完整点云、RGB或aligned-depth检查节点；逐点分析优先离线。
  具体DDS内部等待机制/真实慢调用线程尚未确定；禁用监测的raw/converted直接统计为未测，不把计数0写成零异常。
- **首点候选：**`break→return`仅独立构建、Domain31配对回放；异常首点10→0，但最大位姿跳变0.618→0.954m，没有证明解决定位发散。
  证据`$E/patch-validation.wfw0zqo0/paired-replay-result.json`；未部署，现场原版二进制/首点源码摘要保持。
- **历史输入时间异常：**SDK当前host-receive时间跨度不保证转换器0.05—0.15s契约；具体异常原因未全部确定。
  索引`$E/driver-framing._q99fm_e/framing-and-contract-result.json`、`$E/udp-time-prepared.pnio3u2u/timing-result.json`；该60s独立驱动未捕获旧异常，不能宣称全恢复。
- **诊断记录错误已修：**DiagnosticStatus.level在本机Python是bytes，转整数后实际回调离线检查通过；首次失败窗口`$E/static-chain.q6b7dbk3`保留。
- **costmap退出独立待办：**150s轮次`$E/static-chain-fixed.0id75ch3/runtime/costmaps.log`：SIGINT后清理/析构，子进程16882 exit−11，父launch返回0。
  最新轮次`$S/costmap-shutdown.log`先原生SHUTDOWN(command4)返回success=true，最终SIGINT时costmap17959/生命周期17961均exit−2。
  后续[退出排查补充](costmap_shutdown_handoff_20261008.md)确认代码存在重复SIGINT投递路径；−11崩溃点及最新−2的精确机制仍未确定，尚未修复。
  查`cooperative_ws/src/multi_agent_nav/src/costmap_acceptance.cpp`及临时`run_static.py`→`scripts/run_ab.py:stop_owned`的信号/析构顺序；父退出0不代表子正常退出。
- `Go2Adapter.get_health()`的`localization_valid`主要检查map→base_link TF取得及stamp年龄；不等于定位准确、相机标定有效或现场坐标正确。
- 本轮仅操作者授权的odom/lowstate诊断监护；正式`go2_safety_supervisor`需要新鲜formal PASS，不能写成正式安全监督通过。
- 旧采集137调查暂停，索引`build/go2-field/evidence/costmap-triage.json`；诊断bag不能替代三阶段验收。
  旧PASS复用记录待处理，后续采集重新核实实时输入，不复用历史结果充当本次通过。
- 最新解释优先读取`$S/static-result.json`、`clearing-review.json`、`posture-clarification.json`；后两份更正原结论，不覆盖原快照。
  历史基线/协同测试独立见`docs/multi_agent/dynamic_costmap_evidence/`：273 Python、21 gtest基线，84协同pytest及parser；本交接未重跑。

## 4. 环境、脚本与安装入口清单

以下命令是索引，不是本次执行指令。路径列均为本机已存在文件，另列明确“拟新增”。
后续获授权再运行时，当前真机安装环境为（构建/测试输出统一遵循[runbook](go2_runbook.md)，不混用旧`/tmp/go2-coop`）：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export REPO="$PWD" GO2_NET_IFACE=enp4s0
export GO2_COOP_OUTPUT="$REPO/build/go2-coop"
export CFG="$REPO/cooperative_ws/src/multi_agent_nav/config"
export E="$REPO/build/go2-field/evidence/fastlio-retest.1_h_1z2p"
export S="$E/static-chain-repeat.skw8amvz"
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
source "$REPO/scripts/go2_network_env.sh"
export VLM_NAV_EXTRA_SETUP_FILES="$GO2_COOP_OUTPUT/install/setup.bash"
export PYTHONPATH="$REPO:${PYTHONPATH:-}"
```

网络环境脚本检查rmem_max≥16MiB及192.168.123.x地址，再设置Domain0/CycloneDDS/enp4s0。
禁止最后source旧`unitree_ros2/install`覆盖当前包；任务安装入口shebang为`/home/isee-pst/venv/co-nav-real/bin/python`，ROS生成使用system Python3.10。
mock launch只改变其子进程环境；第二终端的mock客户端需另设`ROS_DOMAIN_ID=31`、
`ROS_LOCALHOST_ONLY=1`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`并unset CYCLONEDDS_URI，不能沿用上面的Domain0。

| 用途 | 绝对文件路径 | 入口命令 | 依赖/参数 | 可能运动 | 状态 |
|---|---|---|---|---|---|
| 原单机VLM | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/VLMNav-go.sh | `bash "$REPO/scripts/VLMNav-go.sh" "实测目标描述" map:=实际yaml` | 既有D435、单位报告、API、交互确认；会SSH、录包 | 是，ARM/ENABLE后 | 原成功栈入口，存在/调用核实；本轮未复验 |
| 原单机Nav2 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/manualnav2.sh | `bash "$REPO/scripts/manualnav2.sh" localization_mode:=amcl map:=实际yaml amcl_scan_topic:=/scan` | 自动预检、bag、Nav2/RViz、Sport桥接、交互ARM | 是 | 存在；不是本轮无录包静止入口 |
| 原入口环境 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/common.sh | `source "$REPO/scripts/common.sh"` | Humble→存在的Hesai/Ranger旧overlay→go2_ws→EXTRA | 否 | 存在，额外overlay次序须审查 |
| Go2网络环境 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh | `source "$REPO/scripts/go2_network_env.sh"` | 本机网卡/rmem检查 | 否 | 已核实 |
| 旧状态agent | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/start_go2_agent.sh | `bash "$REPO/scripts/start_go2_agent.sh"` | multi_agent_comm/go2_agent_node，旧overlay | 否 | 旧里程计状态；不是任务agent |
| XT16原生驱动 | /home/isee-pst/Documents/liang/hesai_xt16_ws/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node | `ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args --params-file "$S/driver-params.yaml"` | UDP2368、原SDK配置 | 否 | 最新静止轮次实际使用 |
| 完整静止入口 | /home/isee-pst/unitree_ros2/VLM_Nav/build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/run_static.py | 历史`/usr/bin/python3 "$S/run_static.py"`；静态`--check-plan` | 环境/原参数/已有相机；最长300s | 否 | 临时一次性脚本；已有runtime会拒绝复跑，不作长期入口 |
| 三阶段快照 | /home/isee-pst/unitree_ros2/VLM_Nav/build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/capture_costmaps.py | `/usr/bin/python3 "$S/capture_costmaps.py" 新目录` | 已激活的独立costmap；只收costmap | 否 | 临时排障，三阶段已用 |
| 独立costmap launch | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/launch/go2_costmap_acceptance.launch.py | `ros2 launch multi_agent_nav go2_costmap_acceptance.launch.py` | 有map/TF/障碍输入；按安装share合并覆盖 | 否 | 正式存在且本轮运行 |
| costmap宿主源码 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/src/costmap_acceptance.cpp | 安装`ros2 run multi_agent_nav multi_agent_costmap_acceptance`，优先由launch参数化 | 原生两个Costmap2DROS | 否 | 已运行，退出问题待处理 |
| coordinator源码 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/multi_agent_nav/coordinator_node.py | `ros2 run multi_agent_nav multi_agent_coordinator_node --ros-args -p simulation:=false -p config_dir:="$CFG"` | Domain0、RulePlanner、真实配置 | 可驱动agent任务 | 实现/离线验证；live未启动验收 |
| Go2任务agent源码 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent_node.py | `ros2 run multi_agent_nav multi_agent_agent_node --ros-args -p simulation:=false -p robot_id:=go2 -p config_dir:="$CFG"` | 完整门槛、真实Go2 adapter | 是 | 实现/离线验证；不是start_go2_agent.sh |
| 任务live launch | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/launch/go2_cooperative.launch.py | `ros2 launch multi_agent_nav go2_cooperative.launch.py config_dir:="$CFG"` | 仅coordinator/Go2 agent；默认IDLE，不启动底层/桥接 | 后续任务可能 | 已存在，live门槛关闭 |
| 独立检查源码 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_check.py | `ros2 run multi_agent_nav multi_agent_go2_check --offline --config-dir "$CFG"` | offline不初始化ROS；现场inspect默认只读但订阅RGB-D | 显式navigate/scan加allow-motion才可能 | 安装/离线核实；此阶段推荐offline |
| 独立检查安装入口 | /home/isee-pst/unitree_ros2/VLM_Nav/build/go2-coop/install/multi_agent_nav/lib/multi_agent_nav/multi_agent_go2_check | 同上一行；亦可绝对路径执行 | co-nav-real，同ABI | 同上 | 已存在，源码/安装位置分开 |
| 双fake launch | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/launch/cooperative_mock.launch.py | `ros2 launch multi_agent_nav cooperative_mock.launch.py` | 自设Domain31/localhost、唯一mock前缀 | 无真机运动 | 已有离线ROS证据；不得接真机 |
| 任务开始/暂停源码 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/multi_agent_nav/start_task.py | `ros2 run multi_agent_nav multi_agent_start_task start --scenario synthetic --prefix 实际mock前缀` | 仅隔离mock示例，另终端须同Domain31 | live调用可能 | 已实现；本交接不调用 |
| 标定候选工具 | /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src/multi_agent_nav/multi_agent_nav/calibration.py | `ros2 run multi_agent_nav multi_agent_calibration 实测references.yaml` | 实测拟合/独立检验点；只打印候选 | 否 | 已存在；Ranger未标定 |
| 完整离线验证 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/test_cooperative_no_hardware.sh | `GO2_COOP_OUTPUT="$REPO/build/go2-coop" bash "$REPO/scripts/test_cooperative_no_hardware.sh"` | 先独立构建；基线与新增分别记录 | 否，Domain31 | 历史已过，本次不运行 |
| 原基线入口 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/test_no_hardware.sh | `bash "$REPO/scripts/test_no_hardware.sh"` | 旧单机测试环境，见脚本 | 否 | 已存在，优先统一离线脚本 |
| 正式传感器预检 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_go2_preflight.sh | `bash "$REPO/scripts/run_go2_preflight.sh"` | 临时驱动/IMU及30s检查；有600s PASS复用分支 | 否 | 已存在；不是纯文件检查，后续核实新鲜输入 |
| 三阶段旧采集 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/collect_go2_cooperative_static.sh | `bash "$REPO/scripts/collect_go2_cooperative_static.sh" --dry-run --config-dir "$CFG"` | 真执行会调用RGB-D observer/rosbag；旧137未解决 | 否 | 已存在；不用于本轮无录包链路 |
| 诊断A/B入口 | /home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_ab.py | `/usr/bin/python3 "$REPO/scripts/run_ab.py" --self-check` | 真执行须session/reference/new-output；B组录包 | 否 | 已固化、未跟踪；不是完整静止启动器 |
| 默认订阅离线检查 | /home/isee-pst/unitree_ros2/VLM_Nav/test/check_go2_diagnostic_monitor.py | `/usr/bin/python3 "$REPO/test/check_go2_diagnostic_monitor.py"` | 替换ROS节点构造，不初始化DDS | 否 | 已通过、未跟踪；本次不重跑 |

原单机调用链：`VLMNav-go.sh`→common/venv/network→相机/NTP/单位/API检查→`manualnav2.sh`
→`run_go2_preflight.sh`→`record_go2_nav2_manual.sh`→`go2_system.launch.py`
→TF/IMU/Hesai/converter/安全监督/阶段waiter→FAST-LIO→障碍→scan→AMCL/map→`go2_navigation.launch.py`
→RViz→START_NAV确认→Sport bridge→ARM；VLM模式再camera TF/`go2_vlm.launch.py`→ENABLE_VLM。
源码默认waiter也会订阅点云，不能把原入口未经审查直接替代“无额外监测”静止轮次。
`start_hardware.sh`/`01_camera.sh`/`02_ranger.sh`/`03_livox.sh`/`04_fastlio.sh`是旧Ranger/Livox路径，非当前XT16 Go2组合。
单机成功背景来自用户及既有指引；已核对启动manifest，读到的9份均status=aborted，不能据此补写某轮完整任务PASS。

完整构建命令、source顺序及基线/协同验证统一见[go2_runbook.md](go2_runbook.md)的Build and offline validation；本交接不重复维护命令。
独立`--offline`仅检查文件；无`--require-ready`时退出0不代表live门槛通过。
现场默认inspect仍初始化ROS并订阅RGB-D等；Domain31 mock/离线测试也初始化ROS，三者含义不同，详见runbook。

**拟新增、当前不存在、不得当命令运行：**`/home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_go2_static_check.py`
（把一次性静止链路整理成可复用、正确生命周期退出的入口）；
`/home/isee-pst/unitree_ros2/VLM_Nav/scripts/analyze_go2_costmap_roi.py`（复用数据读写，按实物ROI离线比对）。
本次不创建/迁移/重构；需先解决退出问题，再决定最小实现，不复制第二套Nav2或视觉栈。

## 5. 配置缺项、下一步与本轮边界

以下来自当前实际`cooperative_ws/src/multi_agent_nav/config/`，不沿用旧“20/24个缺项”数量：

| 文件/字段 | 当前值或缺项 | 填写/验收依据 |
|---|---|---|
| scene.yaml | enabled=false；targets.A/B/C为“现场填写”；viewpoints.go2/ranger=[]；wait_points/exit_points/resources={} | 实物描述、实测观察/等待/C退出点、门与篮子多边形、各端transit_exits；不可填演示坐标 |
| alignment.yaml | alignment_id空，validated=false，report=null；Ranger local_map_id/map_version空、transform=null | Ranger真实地图身份及对应点/独立检验SE(2)；Go2 transform=[0,0,0]是基准定义 |
| robot_go2.yaml | path_topic=null、path_source_verified=false、stop_margin=null | 真正执行/重规划路径与native goal关联；实测取消终态/物理停止距离与包络 |
| robot_go2.yaml | costmap_source_verified/camera_calibrated/vision_validated/peer_obstacle_verified均false | ROI定量costmap、相机TF/单位/ABC、对方整机动态障碍及覆盖证据；不由本轮目视直接置true |
| robot_go2.yaml | model/endpoint=null | ABCClient允许既有环境回退；须核实实际model/endpoint/API可用，不能据null断言环境一定缺失；不打印密钥 |
| robot_go2.yaml | costmap_topic=/local_costmap/costmap_raw | 与本轮`/go2_costmap_acceptance/local_costmap/costmap_raw`不同；正式运动配置必须绑定实际源，不能认为此次独立host自动接入agent |
| robot_ranger.yaml | map_path/map_digest=null，adapter=not_installed | 后续由Ranger电脑独立适配与验收；此机不猜填 |
| config/go2_calibration_xt16.yaml、config/robot_go2.yaml | camera_extrinsic.calibrated=false且xyz/rpy待测；camera_extrinsic_calibrated=false | 旧manual脚本硬编码camera TF不等于当前协同标定通过 |
| 正式预检/安全门 | 新鲜formal PASS、nav_ready/control_armed等本轮未正式验收 | 当前实时输入重验和正式监督；独立诊断监护不是替代品 |

下一步按序：①Domain31退出修复已完成局部验证，见第7节；后续现场复验仍须保留子进程退出判据和单次父launch信号，不直接重启真机。
②明确实物ROI，分local/global验证marking/clearing；③有界检查实际RGB/raw depth/相机TF及既有ABC接口，不加常驻检查订阅。
④补齐本次新鲜预检、正式安全监督、真实执行路径、取消/物理停止能力及实测场景点位/区域；⑤门槛满足且获得运动安排后再做Go2单机运动分项。
跨机通信、Ranger标定/适配、桥接隔离和双机联合验收均未完成；不得用fake Ranger READY解锁真机任务。
Domain31隔离验证按第7节授权；不录包、不SSH、不连接Ranger、不启桥、不发送运动目标、不改验收标记，保留单机成功栈和所有修改/日志/bag。

新会话先读取本文件和AGENTS，重新核实分支/HEAD/status；按任务读取权威方案及专题记录；
以`clearing-review.json`、`posture-clarification.json`为最新现场解释；先核对第7节退出修复和实际安装，任何真机实验或运动需新安排。

## 6. 此前文档整理与授权范围

同一任务已有明确授权在未撤销且范围不变时跨轮延续；次数、时长限制仍有效，
已结束的一次性实验不授权新实验。文档命令、历史实验、旧PASS和“下一步”均不产生新授权。
此前odom/lowstate诊断监护不扩展为录包、
额外完整点云监测或运动；run_ab的`--full-cloud-monitoring`只控制额外raw/converted
监测，不关闭必需处理订阅，也不提供现场授权，B组仍会录包。

本轮只整理AGENTS、本文和go2_runbook，检查链接、命令格式及git diff --check。
不构建、不启动ROS、不做现场实验、不SSH、不连接Ranger、不启桥接、不发运动目标、
不改验收标记、不自动提交；runbook所列命令本轮均不执行。

Go2静止底层链路已贯通、目视支持标记/清除；完整正式静止验收仍缺local/global实物ROI、
新鲜正式预检/安全监督和异常退出闭环。完整自主导航及双机协同未验收，live继续关闭。

## 7. 新增授权后的costmap退出隔离验证

2026-10-08用户明确“继续，如果有必要可以构建或启动ros”。本任务据此执行本机
Domain31、localhost、rmw_fastrtps_cpp的合成输入验证和最小宿主修复；此前不构建/不ROS
限制在此任务范围内解除。SSH、Ranger、桥接、录包、运动及完整协同live仍关闭。

证据根目录：`build/go2-field/evidence/costmap-shutdown.o2f53dag/`。
旧安装及独立重建原代码均被新增退出断言捕获：SIGINT与生命周期SHUTDOWN后两项测试
均host=-11、manager=0。A—D各2轮旧宿主均SIGSEGV，证明重复信号不是充分根因。
GDB确认CallbackGroup弱引用控制块的虚表仍指向已卸载的liblayers.so；只清理内部
callback group仍会在默认NodeBase callback group析构崩溃，此中间尝试未保留在代码中。

最终修复仅限独立宿主：cleanup前用class_loader保留实际已加载的插件库，保持至所有
costmap节点及外层executor销毁后再释放；不改系统Nav2、参数、共享停止脚本或验收标记。
class_loader为本机已有ROS依赖，声明宿主直接依赖；lifecycle_msgs声明为测试依赖。
局部测试7项通过；修复后A—D各2轮host/manager均0、无强制终止或遗留子进程。
另1轮RESET→STARTUP→SHUTDOWN通过，重新配置后两个costmap仍标记/清除成立且退出0。
具体栈、矩阵和限制见[退出排查补充](costmap_shutdown_handoff_20261008.md#新增授权后的隔离复现与修复)。

新产物仅在`build/go2-field/evidence/costmap-shutdown.o2f53dag/isolated-build/install/`，
**未替换现场`build/go2-coop/install`，其旧宿主仍有退出缺陷。**仅局部构建multi_agent_nav，
根包/消息复用旧已验证安装，不重建外部overlay；完整命令见
[runbook](go2_runbook.md#costmap-退出修复的独立构建与局部验证)。
未重跑全量测试、未启动真机链路或提交；Domain0/CycloneDDS及真实静止退出仍待现场复验。
合成ROI清除不替代local/global实物ROI证据，完整自主导航和双机协同未通过。

## 8. Go2 独立人工点探索、记录与RViz标注

用户最新要求直接进行Go2连续探索，记录每次选择点的可达性并实时标注；随后明确
“先使用一个点”。此前仅用于退出排障的ROS授权不再是本项任务的全部范围：
本项允许准备/检查/构建独立探索入口及满足真实门槛后的Go2导航和扫描。
SSH、Ranger、定向bridge、录包及完整协同live仍未授权，不自动提交/部署、不改验收标记。
Go2是否站稳、现场急停操作者及真实底层启动状态尚未获得用户确认，不据采点推断。

用户两条map下点击依次为中心和朝向点：
`(-0.8235164284706116,2.656074285507202)`、`(1.6186352968215942,2.4998345375061035)`。
计算yaw=-0.063889弧度（约-3.660581度），已记录到源
[scene.yaml](../../cooperative_ws/src/multi_agent_nav/config/scene.yaml)的go2_view_01；A=灭火器。
enabled=false及原验收字段保持不变；点击z不作为二维pose。该点可达性仍未知。

实现沿原[multi_agent_go2_check](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_check.py)
扩展action=explore，复用Go2 adapter/独占运动锁/停止检查/执行路径与资源监护；
[go2_exploration.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_exploration.py)
串行选点、停车和规划检查、导航、确认停止、扫描灭火器、继续下一配置点。
不接coordinator或fake Ranger，不改READY/退出协议，不新增global VLM planner。
ABCClient仅在本入口启用single模式，扫描只记录A，不伪造B/C；原ABC协同行为保留。
未发现/UNCERTAIN可继续配置列表；规划失败记录本次结果而非永久不可达；
API/定位/执行路径/停止错误中止，检测目标即停止，不自动接近。
当前只有一个点，完成后停止并报告找到或EXHAUSTED，不自动生成下一层。

每轮必须指定不存在的output-dir，events.jsonl逐事件flush/fsync，result.json保存总结。
RViz箭头和文字话题为`/vlm_nav/go2_exploration/current_point`，可靠/瞬态本地，0.5秒刷新；
[config/go2_exploration.rviz](../../config/go2_exploration.rviz)仅Map和MarkerArray，无RGB/完整点云
订阅或导航/定位重置工具。显示本机map坐标，不桥接TF。
完整构建/局部测试及实机执行顺序见[runbook](go2_runbook.md#go2-独立人工点连续探索)。

本次独立产物：`build/go2-exploration.tLYst4/install/`；只构建multi_agent_nav，
消息与vlm_nav使用`build/go2-coop/install`已验证依赖，未替换旧安装。
局部测试66项通过，其中新增12项、相关既有54项；未重跑全量项目基线、gtest或桥接测试。
真实Domain31隔离MarkerArray晚加入订阅者收到最新PLANNED状态；仅消息测试、无动作服务器。
首次消息检查因默认/home/isee-pst/.ros/log只读退出，改用独立输出evidence/ros-logs后通过。

当前现场就绪证据目录：`build/go2-exploration.tLYst4/evidence/`。
`domain0-readonly.json`为本次5秒有界Domain0/CycloneDDS/enp4s0检查：
active_map_yaml=null、localization_valid=false、costmap_valid=false、rgbd_ready=false，
NAV_READY/CONTROL_ARMED=false，stopped=false（未取得新鲜停稳证据，不证明机器人正在运动）；
point footprint_free=null。没有收到活动地图/有效定位输入，不能把Domain31地图浏览当真实导航。
只读检查exit=1，motion_sent=false；报告里的默认pose不是实测位置。

已用新安装入口尝试启动单点探索，`startup-one-point/result.json`及`events.jsonl`
明确REJECTED，exit=1、motion_sent=false；在ROS初始化和运动锁前拒绝，未发Nav/Spin/速度命令。
实际缺项：basket保护区域、实测正停止余量、costmap来源验收、相机标定、已验证执行路径。
不以填true或演示区域解除限制。还须恢复本次真实底层输入、确认场地/急停与停止证据，
才能使用同一入口开展真机导航扫描。未连接Ranger、SSH、启动bridge或录包；未提交。

用户随后说明相机驱动已启动，要求依据现有scripts/代码给出其余实机组件的终端命令。
本轮仅核对入口和参数，不自动启动硬件。底层顺序为新鲜传感器预检→分阶段
go2_system（AMCL、Nav2及安全监督）→RViz实测初始位姿；本地Sport速度转换单独启动且
保持DISARMED，跨机定向bridge仍关闭。ARM/探索依赖真实门槛，不据启动命令推断通过。

用户提供`/home/isee-pst/Notes/待确定的参数.md`；该文档记录机身IMU/XT16变换，
与当前body-IMU profile数值一致，没有相机外参、停止余量或执行路径验收记录。
用户随后提供相机平移`[0.32715,-0.00003,0.04297]`、零旋转；已记录到源
[go2_calibration_xt16.yaml](../../config/go2_calibration_xt16.yaml)的base_link→camera_link字段。
坐标系及实测验证尚待确认，camera_extrinsic.calibrated与robot_go2.camera_calibrated
仍为false；未修改安装副本。启动命令须显式传入源calibration_file。
停止余量、basket保护区域、costmap来源和执行/重规划路径验证仍缺失。

## 9. 实机启动缺项核对与停车检查接口修复

用户提供的终端4输出确认当时活动地图匹配、定位有效、stopped=true、实时costmap有效
（odom，60×60），NAV_READY话题为true、CONTROL_ARMED为false；相机图像时间TF断链，
rgbd_ready=false。local_observation_ready=true只代表只读检查的子集，不代表可发运动。
health.nav_ready还包含ARM/RGB-D/动作服务器门槛；未执行导航前path为空不证明路径接口坏了。

已定位报告TypeError：inspect和explore三处给costmap_candidate_free多传footprint，
真实Go2Adapter只接受candidate且内部使用已配置footprint。测试替身此前允许额外参数，
现收紧为相同签名；修复前8项相关用例失败，修复三处调用后adapter/field/explore共47项通过。
不改停车算法、验收字段或架构。仅修改源Python；终端最后prepend仓库PYTHONPATH即可
加载修复，无需重启硬件或构建，安装副本仍为原版本，未提交。

本轮有界Domain0/CycloneDDS/enp4s0只读证据：
`build/go2-field/evidence/go2-startup-gaps.WchpLgI6/`中的inspect.json、graph.json、
camera-tf.json；不发规划/导航/Spin/零速度，不ARM或启动硬件。
与用户较早输出不同，本次只发现相机及诊断/daemon节点，地图/Nav2/Sport转换参数服务均
超时，未发现活动底层；其是否已停止或处于其他环境待确认，不能以旧输出代替当前状态。
相机640×480@15、enable_sync=true、align_depth=false已核实；camera_link与深度/彩色
optical frame均连通，base_link→camera_link不连通。源calibrated=false使go2_system跳过
相机静态TF，用户给定值的坐标定义/实测验证仍待确认，不自动将false置true。

当前独立explore配置仍有5个门槛：basket多边形、实测正stop_margin、动态costmap来源
验收、相机标定、已验证执行/重规划路径。Ranger地图/点位、对齐report、B/C目标、
wait/exit及peer_obstacle_verified等完整live缺项不阻挡此入口；独立入口确实仍继承
mandatory basket，待用户确认本次场地是否有共享保护区，不编造多边形解除限制。
视觉API模型/endpoint/key可来自终端环境，源配置为null不证明用户终端缺失；仍须核对。

另有已确认代码限制：停车检查只使用local_costmap，窗口之外也返回false，探索在规划前
跳过并记录FOOTPRINT_BLOCKED，不能将该标签当作实物障碍证据。当前默认有效Nav2参数
继承3×3米、0.05米分辨率窗口，与用户60×60报告一致；按用户位姿计算当前点距离约
3.06米，很可能超出窗口，但活动width/height/resolution服务本轮超时，仍待现场核对。
先用实测近点验证，或另行最小修复全局目标停车检查；不猜测性放大窗口。
TF/路径/costmap发现命令及复检判据见[runbook](go2_runbook.md#实机启动缺项核对)。

## 10. 无共享资源的单机探索与追加采点

2026-10-09用户确认“本次没有共享资源”，并要求多标注几个探索点，覆盖此前先用单点的
限制。scene.resources现有{}保持不变；不填写篮子、Ranger坐标或修改验收标记。
尚无新增点击或pose，本轮不编造或追加点；现有go2_view_01完整保留。

最小修复：region_errors只校验显式资源字典及已有多边形，允许{}；mandatory basket
检查移到完整live_gates，独立explore不再被虚构篮子要求阻挡。完整协同其余门槛保持。
RegionGuard原先空资源直接ALLOW会跳过路径/定位/footprint/停止余量检查，现先执行
原安全检查再返回ALLOW，空资源不能使失效执行路径继续运动。

新增7项配置/空资源安全/连续探索用例；修复前3项明确失败，修复后相关局部73项通过
（既有66项、新增7项），覆盖adapter/field/region/explore/agent/coordinator/safety。
未重跑根项目全量测试、未构建/部署/启动ROS现场验证或发送运动；未提交。
源Python仍须在终端最后prepend仓库PYTHONPATH，新检查进程才加载；安装副本未改。

本轮剩余独立配置门槛为实测正stop_margin、动态costmap来源、相机标定与真实执行路径。
相机TF、API、当前底层运行状态及停车检查局部窗口限制仍按第9节处理，无资源确认
不替代这些验证。采点使用RViz Publish Point的中心→方向成对点击，每次需重新选择
单次工具；按唯一ID追加并审查探索顺序。只有源scene更新后新一轮入口才读取新增点。
在线及Domain31保存地图隔离采点命令见[runbook](go2_runbook.md#追加人工探索点)；
仅采点不启动探索/ARM，地图标注不证明点可达。

## 11. 用户指定替换为三个新探索点

2026-10-09用户明确删除此前点位，并提供六次map下PointStamped点击；按相邻两次
中心→方向配对，以yaw=atan2(y_direction-y_center,x_direction-x_center)计算朝向。
已仅替换scene.viewpoints.go2，复用go2_view_01～03按提供顺序编号，旧
[-0.823516,2.656074,-0.063889]已从活动配置及runbook命令移除，历史证据保留。

| ID | 中心[x,y] | 方向点[x,y] | yaw（弧度） |
|---|---|---|---|
| go2_view_01 | [-0.5513970851898193,1.5923351049423218] | [-0.10403712838888168,1.5090758800506592] | -0.184007117515727 |
| go2_view_02 | [-0.11444089561700821,3.059781312942505] | [0.6346269845962524,2.9453001022338867] | -0.15165803996719165 |
| go2_view_03 | [1.1236017942428589,2.320854663848877] | [0.9571422338485718,2.112705945968628] | -2.2453631961781273 |

点击时间依次为1791531986.143565929、1791531992.103854735、1791531996.288110676、
1791532000.160126735、1791532005.055828789、1791532011.959983717。
六条frame_id均为map，点击z不用于二维位姿，方向点不加入目标列表。
地图身份、Ranger列表、resources:{}、A=灭火器、enabled=false和验收字段均保持。
运行中的探索不会热更新，下一轮须显式使用源配置目录；未修改安装副本。
仅静态核对配对/yaw/顺序/配置边界及文档链接、命令语法、git diff --check，
未构建、启动ROS、发送运动或提交；各点可达性及局部窗口覆盖仍待现场验证。

## 12. 相机已启动后的单机导航寻找链路启动核对

2026-10-09用户要求先核对启动缺项，若无缺项再给完整终端命令；说明Go2待机趴卧，
相机已手动启动。本轮只读检查源配置及已有Domain0图，不自动启动底层或运动。
证据目录：build/go2-field/evidence/go2-startup-check.d2DTFvPF/，
camera-and-local-graph.json记录5秒有界只读节点的参数/TF/发布者检查。

相机640×480@15、enable_sync=true、align_depth=false，RGB/原始深度各1发布者；
camera_link→camera_depth_optical_frame连通，base_link→camera_link与map→base_link不连通。
/lowstate有1发布者（不据端点计数证明新鲜IMU）；/map、/odometry、local raw costmap
发布者均为0，底层尚未在本次图中发现。enp4s0=192.168.123.222/24，
net.core.rmem_max=16777216，运动锁内容为空。未调用规划/导航/Spin/ARM或零速度。

源exploration_gates仍列4项硬缺项：stop_margin=null，costmap_source_verified=false，
camera_calibrated=false（相机calibration文件calibrated同为false，故launch跳过静态TF），
path_topic=null/path_source_verified=false。视觉model/endpoint源为null，支持终端环境
覆盖，但用户执行终端的配置及真实识别结果尚未确认，不读取或打印密钥。
新三个点和resources:{}已配置；Ranger、B/C、完整scene.enabled等不阻挡独立入口。
局部停车窗口仍是入口限制，不因点位已填就视为可达。完整启动条件尚未满足。

推荐人工起立在配置/环境检查之后、scripts/run_go2_preflight.sh之前；相机可趴姿启动。
30秒正式静止预检及随后的FAST-LIO/AMCL/Nav2就绪阶段保持站立静止，再按门槛ARM。
程序没有自动起立或站姿验收，ARM不发起立命令；站姿转换是真实运动，不能计作漂移。
本轮仅补充起立顺序文档并检查链接/命令格式/git diff --check，未构建、录包、提交。

## 13. 执行路径来源核查与缓存有效期修复

2026-10-09用户明确执行六步精简方案：只读核实既有Domain0路径源、按证据最小修补、
Domain31合成验证及记录。未授权启动底层、ARM、真机目标/速度、SSH、Ranger、bridge
或录包；本轮未执行上述动作，不自动提交/部署、不改任何现场验收标记。

证据：`build/go2-field/evidence/go2-path-source.bsSuvM/`。
`domain0-paths.json`为Domain0/CycloneDDS/enp4s0的有界端点/参数/Path采样报告：
只发现camera和本检查节点，无Path话题、Nav2或lidar_points/body_imu/odometry/map发布者，
controller_server及bt_navigator参数服务未在期限内可用。未发任何动作请求。
因此源`robot_go2.yaml`的`path_topic: null`、`path_source_verified: false`原样保留。
第一版临时脚本错误地断言节点完全无publisher，遇到rclpy内建parameter_events而退出；
改为只允许内建参数/日志publisher后重新有界核查通过，两份日志保留。不是ROS输入失败。

静态来源核对使用本机安装版本Nav2 1.1.20对应官方tag源码，下载副本在证据目录。
controller_server接受FollowPath路径及重规划替换时调用DWB.setPlan；
DWB发布received_global_plan，默认prune_plan=true时还在控制循环重发剩余全局路径。
循环重发保留原计划header，不能以20Hz收到消息推断header总是新鲜。

已确认并最小修复的adapter问题：无本进程已接受导航也会缓存路径，取消和终态后缓存
仍可能有效。当前on_path_update只接受已有handle、未取消且非SCAN_SPIN的活动动作；
取消和SUCCEEDED/CANCELED/ABORTED均清空缓存。保留map/时间/有限值检查，不改超时。
Path无原生goal UUID，当前导航存在仅是必要条件，独占源与目标对应仍待现场证明；
代码shortcut说明此限制，不能用该修复替代path_source_verified的实测验收。

调用方补齐：独立go2_check navigate先处理终态再做执行路径监护，避免成功后缓存清空
被误报为执行路径故障；原动作终态/物理停止确认保留。explore和AgentRunner原先先消费
终态，已核对；新增explore终态清空路径回归，未重写其循环。
本轮仅修改源Python及测试，未替换既有安装，后续需最后prepend源PYTHONPATH。
既有单目标视觉/配置/资源/退出等用户修改完整保留。

测试：修改前相关既有73项通过（baseline.xml）；新增路径测试在原代码上6失败/6通过
（red.xml），终态调用顺序另1失败（caller-red.log）。修复后最终相关87项通过
（final.xml）：既有73项、新增14项，覆盖待接受/空闲/Spin、取消/三种终态、重规划替换、
前目标旧路径、坏坐标系/非有限值/超时、未验证来源及终态后探索继续。
使用co-nav-real Python、源PYTHONPATH、禁pytest第三方插件；仅ROS测试参与者使用
Domain31/localhost/rmw_fastrtps_cpp，终端0环境用stub且不初始化ROS。
未重跑根项目全量测试、C++或桥接测试，未构建。

Domain31实际控制器验证：`isolated-controller-result.json`及controller日志。
复用源Go2 DWB controller参数，只给合成局部costmap移除传感器层，输入为合成TF/odom；
cmd_vel定向到`/go2_path_source_isolated/cmd_vel`，无底层或Sport桥接。
两个FollowPath请求初始路径与替换路径均从controller_server的received_global_plan收到，
frame=map；取消终态5(CANCELED)，deactivate/cleanup后进程SIGINT退出0、无强杀。
观察到重发header年龄超过1秒。该项未启动planner_server/BT/NavigateToPose，
不作为完整重规划或新协议验收；临时脚本不是长期入口。

待验：实际Domain0路径发布者/插件/QoS、真实NavigateToPose与当前native goal关联、
重规划和首路径时序、header与接收延迟、断流取消终态及物理停止。
保持path_max_age/path_start_timeout=1.0，不能猜填更大值或刷新旧路径时间戳。
启动并核对步骤只维护在[runbook路径来源小节](go2_runbook.md#执行路径来源核查2026-10-09)。
本轮没有真实路径来源，因此仍不能开始单机探索；其他门槛见第12节。

## 14. 用户指定动态costmap标记及停止余量、路径来源说明

2026-10-09用户明确要求动态costmap验收设true，并再次给出相机平移
[0.32715,-0.00003,0.04297]及零旋转；要求解释停止余量、推荐取值和路径来源确认方案。
已仅将源robot_go2.yaml的costmap_source_verified由false改true，记录为用户指定状态；
本轮未做新增现场验收，不能因此宣称本轮完成动态障碍或导航验证。
相机数字已在源calibration文件，未重复改写；父子坐标定义/实测投影待确认，
camera_extrinsic.calibrated与robot_go2.camera_calibrated保持false，launch仍跳过该TF。
既有安装与运行进程未更新，后续命令须显式使用源配置。

stop_margin仍null；其为整机外接圆之外的资源边界停止余量，不是Nav2制动/避障参数。
本次resources:{}没有资源几何效果，但仍要求实测正值。当前0.40m/s速度上限及
软件0.50m/s²减速度不代表实机已验收；假设反应1.5s、有效减速度0.50、误差预留0.10m，
估算0.86m、推荐1.0m作为待验证候选，不自动配置且不宣称安全保证。
最终值应覆盖触发停止到物理停稳的最大继续位移和定位/对齐误差及预留。

执行路径沿用第13节证据：DWB控制器received_global_plan优先于planner /plan，
但重发旧header超过1s已实证，无法据频率直接接入。先实际Domain0端点/参数核对，
再隔离完整NavigateToPose/BT/FollowPath的UUID、重规划、新鲜度和取消验证，
必要时另行最小设计活动路径快照而非修改旧时间戳/放宽超时；最后有界现场验证
独占源、目标关联、断流终态及物理停止，再填写path_topic/verified。
详细方案与停止余量说明见[runbook](go2_runbook.md#2026-10-09-用户指定配置与停止余量)。
本轮仅配置及文档修改；不构建、启动ROS、录包、发送运动、修改其他验收标记或提交。
静态校验确认robot配置仅上述一个字段相对HEAD改变，相机数值/三个点/resources:{}
完整保留；离线探索门槛现为停止余量、相机标定及已验证执行路径三项。
47条本地文档链接、41段shell命令语法及git diff --check通过；未初始化ROS。

## 15. 停止余量临时设为1.0米及相机标记说明

2026-10-09用户明确要求停止余量暂设1.0m，并询问相机外参标记为何仍false。
源robot_go2.stop_margin已由null改为1.0，记录为用户指定临时值，未经实机停车测量；
安装副本和运行进程未更新。相机数值及两处标记本轮未修改。
camera_extrinsic.calibrated控制go2_system是否发布base_link→camera_link静态TF；
robot_go2.camera_calibrated控制独立运动入口的相机验收门槛。false是此前人工保留的
配置状态，不是程序判定所给数值错误。给定数字已记录，尚未核实父子坐标定义与
静止图像投影；外参数值、TF连通及现场投影验收是不同证据，不自动互相替代。
离线门槛剩相机标定及已验证执行路径两项；临时停止余量不代表现场停止能力通过。
仅更新配置和说明，静态校验、文档链接/命令格式及git diff --check通过；
未构建、启动ROS、发送运动、录包或提交。

## 16. 明确沿用上一版相机外参与单源TF接入

2026-10-09用户明确授权沿用上一版外参并将对应标记设true，无需再确认；仅处理
Go2相机配置/TF接入及适当检查。依据：**用户确认沿用上一版 Go2 相机外参**。
已核对旧go2_calibration.yaml的camera.calibrated=true、旧manualnav2独立硬编码TF、
旧go2_vlm节点覆盖camera_extrinsic_calibrated=true；本轮不声称重新完成实测标定或投影。

实际加载链及修改：go2_system默认XT16 calibration_file；config/go2_calibration_xt16.yaml
的camera_extrinsic.calibrated现true、source写明采用依据，数值保持
base_link→camera_link、xyz=[0.32715,-0.00003,0.04297]、rpy=[0,0,0]。
root config/robot_go2.yaml的camera_extrinsic_calibrated及协同robot_go2.camera_calibrated
现true；make_observer先读实际root share的robot参数，再按协同config-dir覆盖，
已用真实只读VLMNavigator验证最终true、enabled=false、publish_stop_command=false。
go2_vlm默认改为同一XT16文件，继续最后覆盖节点标记true；manualnav2显式给两个入口
传同一配置，移除其独立相机TF硬编码。common.sh可能重载旧overlay，故manualnav2
在该步骤后检查当前包的相机所有权程序；缺失则在网络/预检/录包/硬件启动前退出。
Ranger默认入口、其他运动门槛及旧安装不改。

新增vlm_nav/go2_camera_tf.py仅从配置构造参数/launch动作，src/go2_camera_tf.cpp
实现camera_link来源检查与静态发布；独立go2_camera_tf.launch.py只接相机TF。
按publisher GID核对父frame/数值/来源数，复用正确已有来源；无来源时用同机同Domain
进程锁保证自有源唯一，多源/不一致退出1，集成launch关闭。不额外订阅图像或点云。
本机Humble rclpy回调及take_message元数据均不提供GID，故使用现有rclcpp原生接口，
只补直接tf2_msgs依赖，无新第三方库；CMake登记可执行文件和局部测试。
外部未更新的手工publisher后起仍可能短暂重叠，检测后撤回自有源或失败；
新旧已更新入口共用锁，不再直接启动安装边static_transform_publisher。

构建：新独立root overlay位于build/go2-camera-tf/install，colcon仅构建vlm_nav；
依赖复用旧已验证overlay，没有原地重建。已核对实际share指向新输出、安装文件与
源相机配置/三个launch逐字节一致；启动环境必须最后source该root overlay。
独立协同包继续用原安装和源PYTHONPATH/显式config-dir，额外setup链也须包含新root。
构建命令和TF检查顺序见[runbook](go2_runbook.md#go2-相机外参沿用与单源tf接入2026-10-09)。

验证：相关既有71项、新增9项共80项通过，final.xml位于build/go2-camera-tf/。
包括真实C++并发双入口单源、已有来源复用、后起外部来源撤回自有源、错误父frame/
数值、多源失败、禁止optical当安装子frame、实际观察节点覆盖参数、旧overlay拒绝。
一项旧field用例依赖costmap默认false，与第14节用户设置冲突，已显式构造false输入；
root profile原false断言随授权更新。最初Python实现因Humble缺GID接口失败，已删除，
未保留不可用的备用路径。最终局部测试、launch参数展开及安装/配置检查通过。

有界现场证据：build/go2-camera-tf/evidence/{before,with-camera-tf,after-cleanup}.json。
开始时仅驱动camera发布内部TF、安装边不存在；图像header实查分别为
camera_color_optical_frame和camera_depth_optical_frame。
只启动新相机TF入口后，base_link→camera_link平移完全一致、旋转[0,0,0,1]，
只有一个安装边源；/tf_static共两个发布者（安装边节点+驱动内部TF），不是重复。
base_link到真实RGB/depth optical均连通，驱动光学旋转保留。
当时map frame不存在，map→depth及图像时间投影验收未完成，不补启底层。
检查后关闭本次临时入口，图核对仅剩camera的static publisher，未留后台TF进程。

第14/15节相机false记录为历史，本节覆盖当前状态；离线探索仅剩真实执行路径来源
门槛，但现场停止能力、API/投影及底层完整链仍须按原要求验证。
未启动底层/导航/VLM API、ARM、运动、SSH、Ranger、bridge或录包；未重跑全量测试、
未自动提交。文档链接、命令格式及git diff --check通过。

## 17. AMCL 就绪门槛阻止 Nav2 启动的只读证据

2026-10-09用户提供target_stage:=nav2、localization_mode:=amcl的go2_system启动
命令及节点列表；列表有wait_go2_slam，无controller_server/planner_server/bt_navigator。
本轮仅做7秒有界只读检查，未设置初始位姿、启动底层或发送运动。
证据：build/go2-field/evidence/nav2-localization-gate.lwXSb8/localization-gate.json。
现场/map有效（map、222×312、分辨率约0.05m），收到70条/scan；AMCL及map_server
均active，AMCL实际scan_topic=/scan，set_initial_pose=false、tf_broadcast=true。
odom→base_link及base_link→hesai_lidar连通，但map→odom不连通，未收到/amcl_pose；
地图实际路径为maps/lab_rear120_20261001_IwQjMX/lab.yaml。

已确认阻塞项是map→odom：wait_go2_slam虽保留旧名称，运行参数是AMCL模式，
等待/map及map→odom，超时600秒；该进程成功退出后才启动Nav2。
尚未建立有效AMCL初始定位是当前推测，不能仅凭无pose断言用户从未设置过。
下一步由用户在RViz用真实地图位置/朝向设置2D Pose Estimate；若已尝试，则查
AMCL初始化/TF日志。现有go2_costmaps.rviz未包含SetInitialPose工具，需在UI添加。
manualnav2.sh已有map→amcl_bootstrap临时静态TF，仅用于地图显示，定位建立后关闭；
不得发布静态map→odom绕过门槛。原launch仍运行时不重复启动导航栈。

本轮未修改启动逻辑或执行路径配置。/path发布者lio_mapping是历史定位轨迹，
仍不能用作Nav2执行路径；/received_global_plan现场尚无发布者，来源/重规划关联仍
待验证。只读证据不是导航验收，未启动导航目标、ARM、桥接、录包或真机运动。

## 18. 探索 RViz 空地图的后续只读检查

2026-10-09用户以rviz2 -d "$REPO/config/go2_exploration.rviz"启动，反馈无地图及
INFO级Stereo is NOT SUPPORTED。源码配置有启用的/map显示，QoS为Reliable、
Transient Local，Fixed Frame=map；该立体显示提示不能证明地图链路故障。
再次7秒Domain0只读检查的证据：
build/go2-field/evidence/rviz-map.2zDZhh/localization-gate.json。
本次仅可见camera、rviz及监听/检查节点，/map、/scan、/amcl_pose均无发布者，
AMCL/map_server状态服务不可用，未收到地图/scan，TF仅相机内部边。
此状态不同于第17节，不继续假设当时地图发布者仍在；原launch是否超时退出需看
其终端日志。检测到rviz节点但无/map订阅，是否加载目标配置及该终端REPO变量仍
待核实，不能断言配置错误。后续先恢复/确认地图发布者，再用明确文件路径加载
RViz；若map尚不在TF树，复用上一节仅供显示的amcl_bootstrap后设置真实初始位姿。
未启动后台TF、底层或RViz，未改配置，未发布初始位姿或运动命令。

后续用户在~目录终端查询返回空节点列表和Unknown topic '/map'；按runbook加载
当前overlay及Domain0/CycloneDDS/enp4s0后再次CLI检查亦为空，证据位于
build/go2-field/evidence/domain0-graph.Jsnte7/{nodes,map}.txt。
该时间点整个本地域图均不可见，不能仅凭此断言进程退出；先查原go2_system终端
最后日志及退出状态，区分启动停止和发现环境/网络问题，再决定是否恢复启动。
首次CLI因沙箱不可写~/.ros/log未执行查询，已将ROS_LOG_DIR置证据目录后重试成功。

## 19. 设置初始位姿后确认控制器路径话题

2026-10-09用户反馈已设置初始pose，提供的新节点列表含controller_server、
planner_server、bt_navigator、两处costmap及lifecycle_manager_navigation；
/received_global_plan类型nav_msgs/msg/Path，唯一发布者/controller_server，
QoS为Reliable/Volatile、Keep Last(1)，当时订阅者0。用户输出确认Nav2已进入
运行阶段及路径接口来源，不代表已收到路径消息或完成执行/重规划验收。
cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml现填写该path_topic；
path_source_verified仍false，path_max_age/path_start_timeout仍1.0，未改运动门槛。
可靠/volatile订阅与发布QoS兼容；订阅者0只说明当时无订阅端点。
当前1.1.20 DWB重发可能保留旧header，持续发布不能代替header新鲜度，标准Path
没有目标UUID，实际NavigateToPose目标关联、重规划、新鲜度与断流停止仍待验证。
仅修改源配置和说明，未更新安装副本或运行进程，未发送规划/导航/速度或ARM命令。
验证：真实源load_config成功，assert确认话题、false验证标记、两个1.0秒超时及
原相机/costmap/停止余量值保持；exploration_gates仍报告唯一的已验证路径缺项。
检查未初始化ROS，git diff --check通过；未构建或重跑全量测试。

用户继续提供控制器参数：FollowPath.plugin=dwb_core::DWBLocalPlanner、
publish_global_plan=true、prune_plan=true。随后8秒有界只读采样再次确认上述参数、
controller_server唯一Path发布者及Reliable/Volatile QoS；两个节点use_sim_time=false。
活动BT为/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/
navigate_to_pose_w_replanning_and_recovery.xml，实查该树RateController为1.0Hz。
证据：build/go2-field/evidence/path-passive.yOHj5M/path-sample.json。
采样期间收到0条路径，不能据此判定故障；用户是否已发送导航目标尚待答复。
本轮订阅仅Path，参数服务只读，检查节点发布者assert仅parameter_events/rosout，
退出后销毁节点，未发规划/导航/速度或初始位姿命令。source_verified仍false，
实际路径frame/非空/时间戳及目标关联、重规划、断流停止未获得新证据。

用户随后询问如何验证后设true：核对现有go2_check确认false会拒绝navigate/explore，
不能把它当首次来源验收入口或先置true绕过。runbook补充人工验收条件：完整BT目标
关联、1秒首路径/双时间新鲜度、实际计划替换、隔离故障下取消清理及有界现场停止。
完整BT及独立现场验收入口仍未实现，本轮仅明确方法，未执行新导航或修改验证标记。

## 20. 1秒路径阈值来源与表述修正

用户询问1秒是否来自通道监护反应时间。git追踪确认：505e970的adapter已使用
path_max_age/pose_max_age回退值1.0；d5c9680新增path_start_timeout=1.0，目的是
处理首路径启动延迟，代码仅在仍静止及停止包络允许时等待；2575a54将path_max_age
等值显式写入配置。已查代码、提交记录与架构/进展文档未找到1秒的速度、通道距离、
停止能力或故障反应预算推导，因此只能称实现默认阈值，不能称经论证的安全时限。
path_max_age同时用于到达消息header年龄检查和最后有效消息接收超时，DWB保留
旧计划header时二者意义不同；阈值触发也不等于1秒内物理停车。
此前“不得放宽1秒”的绝对表述修正：当前配置约束仍需满足，但具体阈值应按消息
语义、更新/通信/调度延迟及取消/制动能力重新论证；不能为凑PASS直接增大或刷新
旧header。runbook已更新，参数及verified=false保持，本轮未启动ROS或执行运动。

## 21. 旧路径下当前位置包络的后备保护及当前部署限制

用户问旧路径一直保留时是否仍会无许可停在通道外。静态调用链确认RegionGuard
先拒绝无效定位/路径，再检查未许可区域与[当前pose]的外接圆+stop_margin相交，
然后才检查执行路径；所以有效但绕开通道的旧路径不能屏蔽当前位置STOP。
AgentMachine在NAVIGATING监护中将非ALLOW变为STOP，AgentRunner取消自有目标及
调用adapter.stop_motion；该行为依赖任务监护运行，不是底层资源许可速度联锁。
源码go2_nav/safety_supervisor只监护健康，不读取resource/token；当前scene.yaml
仍resources:{}，用户此前节点列表没有multi_agent_task_agent，仅Nav2运行不提供
这层通道许可保护。缓存长期存在也不等于path_valid一直有效，过期路径会请求停止。

本轮区域既有pytest 3项通过；另用纯Python合成几何验证：旧路径远离通道、远处pose
ALLOW，最新pose靠近停止包络时STOP，实际AgentRunner分发取消自有目标及停止调用；
空resources对此有效pose返回ALLOW，path_valid=false仍STOP。未初始化ROS或运动，
没有以合成坐标代替现场坐标。没有通道物理停止验收，1m stop_margin仍用户临时值；
旧路径后备不保证越界前物理停车，需测监护/定位/取消/制动总延迟与停止包络。


## 22. 单机实验取消计划年龄门槛，独立5秒接收监护（2026-10-09）

用户明确决定修改代码并完成必要检查，之后交付一次完整单机真机实验启动步骤；
不要求先搭完整Domain31，不逐项真机故障注入，不再重复方案确认。当前无共享通道
互斥，resources:{}保持；定位、避障、控制、独占owner与急停要求保持。
此前第19–20节的path_max_age/verified阻塞说明为历史状态，本节取代其单机启动规则。

实现：cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml删除path_max_age，
独立path_receive_timeout=5.0，path_start_timeout=1.0仍仅允许静止等首条路径。
adapter删除header年龄拒收，只以monotonic距最后有效执行路径接收计算断流；
缓存读取、非法消息不续期，header不改写。同一目标每条有效收到的路径更新缓存，
包括几何/stamp未变的重发；不以几何未变判定无重规划。accepted自有目标、map frame、
非空/有限坐标及请求stamp屏障保留，取消/成功/失败/任务切换清缓存。
5秒仅是用户指定的本轮实验初始值，未经过安全推导或现场停止测量。

单机navigate/explore不再用path_source_verified阻塞，源标记仍false；
config.live_gates完整协同要求未改。--offline --require-ready现在检查
single_robot_experiment_gates，仍报告完整live_gates的未完成项，退出0不代表现场通过。
无路径/接收超时走实际cancel_navigation/stop_motion和零速，终态+静止同时确认；
日志失败不阻断取消、停止观察或资源释放，evidence_write_failed如实输出。
复用events.jsonl记录目标、native UUID、接收间隔/原header、更新、取消终态及停止观察；
没有新增相机/点云诊断订阅或bag。Path没有UUID，独占/任务屏障支持关联但不是身份认证。

验证：最初8文件基线83通过、1失败（旧测试假定stop_margin默认null，与现配置冲突）；
将无效配置fixture显式设null。最终9文件101通过：88项既有检查（含上述fixture及
额外4项既有coordinator），13项新增检查；覆盖旧计划可收、单调5秒断流、同几何
更新、无路径实际取消/停止、缓存清理、单机配置门槛与证据失败清理。新增运动测试均fake/stub；2项既有observer测试短时初始化
localhost/Domain31节点并退出，其余为逻辑/传输stub，不构成完整Domain31或真机验收。独立只读审查发现2个日志错误，RED复现后
修复，再审无阻塞；日志在build/go2-path-monitor-20261009/evidence/。

本轮multi_agent_nav已在独立输出build/go2-path-monitor-20261009/install重新构建，
源码根与cooperative_ws/src均参与发现，仅更新该包；任务消息复用build/go2-coop/install，
根vlm_nav复用build/go2-camera-tf/install。已校验安装Python和配置字节等于当前源码，
包前缀正确且相机根overlay保留；安装入口offline --require-ready退出0，
plan_age_checked=false、receive_timeout_s=5.0、source_field_verified=false，live_gates
仍有协同缺项。全量测试、完整Domain31及真实运动未执行，未改验收报告、提交或部署。
启动完整命令、环境、ARM/取消/停止观察及条件分支在go2_runbook.md首节维护；
当前底层仍正常运行可复用，不重跑传感器预检。底层关闭时才按冷启动分支恢复。
真机运动由操作者按该顺序执行，出现具体问题再针对性排障。


## 23. 完整实验首次退出1：RGB-D启动TF缓存尚未建立（2026-10-09）

用户反馈终端A退出1。已读取本次build/go2-field/explore.kzZlaFGM/result.json及events：
motion_sent=false，REJECTED=existing NAV_READY/CONTROL_ARMED/RGB-D/action gates closed。
实际地图匹配、定位、静止、costmap、depth_units_verified、nav_ready_gate和control_armed_gate
均true，rgbd_ready=false；不是路径年龄或5秒接收超时，也未发送运动目标。
原现场日志build/go2-exploration.tLYst4/live-20261008-01/ros-logs/
python_33990_1791543146018.log有图像时间camera_depth_optical_frame→map外推等待。

针对该具体问题做本机有界只读检查：四个图像/CameraInfo话题真实单发布者，
相机enable_sync/color/depth为true；不订阅额外点云，不发动作/速度/ARM。
一次15秒inspect结束rgbd_ready仍false，不能当通过。随后复用既有read-only observer
进行12秒临时内部状态采样（无速度发布器，结束销毁），确认RGB/depth同步健康、
两个CameraInfo均有、外参存在、invalid_frames=0；前4秒图像stamp早于新TF缓存的
最早数据、无snapshot，约5秒起snapshot成立，后续至12秒持续ready=true。
证据rgbd-inspect.json/rgbd-state.jsonl及调试ROS日志位于
build/go2-path-monitor-20261009/evidence/。这是针对性只读观察，不是相机稳定性或真机
完整验收；第一次15秒未就绪的具体子条件未记录，不将其归因直接写成已证实。

已修复go2_check固定2秒即拒绝的启动竞态：若motion入口RGB-D未就绪，在现有
--timeout总预算内继续被动等待真实同步/图像时间TF，记录WAITING_INPUTS；
超时REJECTED，且即使输入在deadline才成立也不发目标。输入成立后原所有门槛仍检查，
不改相机TF/同步阈值，不改路径5秒或停止行为，不替换图像时间为最新TF。
新增3个stub回归先RED再GREEN：5秒就绪可进入、持续缺输入不运动、deadline才就绪
不运动。9文件104通过（本次基线101，新增3）；multi_agent_nav已在原独立输出重建，
安装入口字节核对通过，文档/差异检查通过。没有替用户重跑真机运动。
后续操作者保持底层及Sport/RViz运行，在终端A选新证据目录再启动完整探索；
不复用已有目录，不为此重启感知或AMCL，出现具体问题再读取该轮证据。


## 24. RGB-D修复后正常EXHAUSTED：三个目标都超出局部costmap完整覆盖（2026-10-09）

用户提供下一轮输出，实读build/go2-field/explore.ippg1h15/result.json/events.jsonl：
RGB-D、定位、静止、地图匹配、local costmap、NAV_READY、ARM均true，单机配置门槛空；
live_gates仅报告未启用的完整协同缺项，没有据此拒绝。入口result=EXHAUSTED、
motion_sent=false；三个viewpoint均SELECTED→FOOTPRINT_BLOCKED，plan_reachable=null，
没有ComputePath、NavigateToPose或扫描。EXHAUSTED只表示列表结束，不表示查找已完成。

针对具体跳过原因有界只读核对：local costmap实际width=3、height=3、rolling_window=true、
60×60格、分辨率0.05；global_costmap/costmap_raw有真实Reliable/TransientLocal发布者。
复用实际最新local payload及payload时间map→odom TF，3秒采样后退出，无RGB/完整点云
诊断订阅、无动作/速度/ARM。三个目标的完整车体角点均超出局部边界，适配器返回false；
证据build/go2-path-monitor-20261009/evidence/goal-local-coverage.json。
因此FOOTPRINT_BLOCKED在这次表示局部窗口无法覆盖，不能证明目标处有障碍或不可达。
GetCostmap一次性服务探针等待4秒后样本已陈旧，不用于目标free判断；全局目标停车
footprint是否空闲仍待检查，不能据发布者存在写成通过。

当前explore在ComputePath前及导航前调用local costmap_candidate_free，远目标无法
进入规划。这是原有局部窗口限制，和本轮路径接收5秒或verified=false无关；本轮没有
改变点位、扩大窗口、放宽障碍/未知格规则、改验收标记或发送导航目标。若保留既定三个
远目标，后续应将目标停车预检使用全局costmap，局部costmap继续用于运行健康和避障；
需要实现及测试该目标预检调整，不能重复原命令期待进入导航或用ComputePath可达代替
完整车体停车检查。此前runbook已有局部窗口限制说明，本节附本次真实证据。


## 25. 必要修改：探索目标使用全局costmap停车预检（2026-10-09）

用户明确授权修复第24节的远目标预检限制。仅单机explore启用global_goal_checks，
新增goal_costmap_topic=/global_costmap/costmap_raw订阅及独立缓存；其已确认的真实发布者
为/global_costmap/global_costmap，Reliable/TransientLocal，订阅QoS兼容。
复用既有full payload、stamp及单调接收年龄、payload时刻TF和完整车体矩形检查；
不扩大局部窗口、不改变障碍253/254/255未知格否决，不仅检查中心格。
选点后及ComputePath完成后发导航前，两次使用全局停车检查。全局缺失/过期/无TF/无效
数据报告输入失效，不冒充障碍；global footprint出边界或障碍/未知格仍跳过。
全局检查期间刚好过期的边界已通过RED复现后修复，不能误记FOOTPRINT_BLOCKED/EXHAUSTED。

局部costmap继续原get_health、explore运行健康、接近候选及Nav2避障；其他入口/协同
adapter默认不订阅全局目标grid，不改变其行为。三个点、定位/相机/控制门槛、
5秒执行路径接收监护、验收标记全部保留。只有robot_go2.yaml追加goal_costmap_topic，
其余键与修改前快照解析值一致。report记录goal_costmap_topic/goal_costmap_valid，
events记录目标预检来源及全局FOOTPRINT_BLOCKED原因。没有增加RGB/点云诊断订阅。

验证：修改前9文件基线104通过，新增19项，共123通过；检查远点局部外/全局内、
全局边界/253/254/255、缓存隔离、missing/payload/stamp/receipt/TF、规划后失效或障碍、
局部健康不可由全局替代、仅explore订阅及QoS、启动缺输入不运动、检查期间年龄边界。
测试运动/真实输入fake或stub；2项既有observer测试短时初始化localhost/Domain31，
未搭完整验收环境。一次只读审查发现年龄边界分类问题，修复后再审无阻塞。
证据build/go2-goal-costmap-20261009/evidence/{baseline,red,review-red,final}.log及XML。

最终安装位置改为build/go2-goal-costmap-20261009/install；root vlm_nav仍复用
build/go2-camera-tf/install，消息复用build/go2-coop/install。两源码根参与发现，仅构建
multi_agent_nav；最终重新构建退出0，安装三处Python和robot_go2/scene配置字节与源一致，
包prefix正确。安装入口offline --require-ready退出0：单机配置门槛空、运动false、
path_source_verified=false、接收5秒、完整live_gates仍有协同缺项，不代表现场通过。
runbook构建/运行路径及目标检查说明已更新；51个bash块语法和文档链接检查、
git diff --check通过。未跑全量测试、完整Domain31或真机动作，不自动提交/部署/ARM。

后续：操作者保持原底层/Sport/RViz，终端A加载该新包及相机root overlay，使用新证据
目录重启完整探索。预期goal_costmap_valid=true后按实际全局数据尝试停车检查/规划；
不保证三个点实际可停车/可达，不伪造报告或自动发目标，具体结果仍在整轮实验观察。


## 26. 重启后的EXHAUSTED仍加载旧探索代码（2026-10-09）

用户反馈遥控器操作可能解释前一轮外部Sport请求；这只是用户提供的可能来源，
未据此关闭foreign Sport保护。此前explore.5LPa6Ips在启动时同时缺全局目标costmap、
NAV_READY和CONTROL_ARMED，没有发运动。旧桥日志在19:04左右有foreign Sport触发FAULT；
重启后真实新桥日志twist_to_go2_sport_bridge_39021_1791545199853.log记录ARM accepted，
本机只读查询得到ARMED。不能把旧FAULT当作重启后仍未运动的原因。

最新build/go2-field/explore.uhCPKttl/result.json和events.jsonl：NAV_READY、ARM、RGB-D、
定位及local costmap均true，三个点都SELECTED→FOOTPRINT_BLOCKED，plan_reachable=null，
最终EXHAUSTED、motion_sent=false，无ComputePath/导航/扫描。本轮result和events完全
缺少新版goal_costmap_topic/goal_costmap_valid以及全局阻塞reason，与旧安装
build/go2-path-monitor-20261009/install中的局部预检实现一致；新版当前安装必写上述字段。
因此确认这轮未加载全局目标预检版本；具体是哪个旧overlay/启动终端覆盖无法由已退出
进程追溯。RViz点标记只证明选点，不能证明已发送导航；EXHAUSTED正常返回0。
未发送自己目标前的path_valid=false不是这轮拒绝原因，完整协同live_gates也未阻塞单机。

只读环境验证：清除PYTHONHOME/PYTHONPATH后按coop依赖→最新goal-costmap→camera root
的local_setup顺序加载，ros2 pkg prefix和两个实际Python模块路径均在
build/go2-goal-costmap-20261009/install/multi_agent_nav，检查退出0。
runbook加入同一终端的模块路径assert，避免仅核对包prefix仍误加载旧Python代码；
不必重建或重启正常底层。文档bash块语法及git diff --check通过。

另一个已观察事项：真实/global_costmap/global_costmap的update_frequency与
publish_frequency均为1.0；6秒临时必需global Costmap订阅样本在map下payload有效，
接收间隔包含约1秒及一次2秒，而当前全局目标检查复用costmap_max_age=1.0。
证据build/go2-goal-costmap-20261009/evidence/global-costmap-receipts.jsonl。
因此新版存在全局缓存间歇过期风险，但无法证明explore.5LPa6Ips失败瞬间确切子条件，
更不是这轮旧代码EXHAUSTED的直接解释。本轮未改新鲜度参数、障碍阈值、点位、
验收标记或运动代码；未自动ARM/reset/导航/API/录包，未重跑全量测试或真机实验。


## 27. 新版再次启动拒绝：ARM关闭及全局年龄预算；三个实际停车点受阻（2026-10-09）

最新结果explore.QPNuv21J（events wall_time 1791546041–1791546043）已包含goal_costmap字段，
确认不是第26节旧Python overlay问题。rejected两项：global目标输入无效和motion门槛关闭；
定位、静止、local costmap、RGB-D、node.nav_ready为true，但control_armed_gate=false，
health.nav_ready因此false；goal_costmap_valid=false，motion_sent=false，未规划/发导航。
前两次MiQ5i30g、XRRSsi1v也未ARM，其中MiQ那轮全局有效，证明控制锁独立阻塞。

真实桥新日志twist_to_go2_sport_bridge_39021_1791545199853.log在1791545807.945765
再次记录foreign Sport request observed while ARMED→FAULT，随后仍有外部请求。
本轮有界只读查询当前state=DISARMED，control_armed唯一真实桥发布者、QoS符合。
外部请求具体来源未认证，不能将用户的遥控器推测写成确定根因或关闭保护。
reset/arm的实际服务及前置条件已核对源码，并补入runbook条件分支，均由操作者执行。

已修复全局目标数据预算与1 Hz输入不匹配：新增独立goal_costmap_max_age=3.0，
on_costmap入缓存及costmap_transform同时用于原header与monotonic接收年龄检查；
缺省旧配置仍1.0。3秒是实测存在2秒gap后本轮初值，非安全推导；local costmap仍1.0，
执行路径接收仍5.0。缓存读取不续期、不改stamp、不放宽253/254/255及定位/ARM门槛。
配置正有限校验及report新值同步。新增8项先RED（8 fail）再GREEN，总131 pass
（本轮基线123+8新增）；8新用例覆盖独立header/receipt过期及六种非法值。
一次编辑误放旧测试断言导致NameError，已移回原测试、保留旧覆盖后重新全部通过。
只读代码审查无阻塞，独立14项纯stub通过。证据在
build/go2-goal-costmap-20261009/evidence/timeout-fix/{baseline,red,final,build}.log。

同一当前独立输出仅重建multi_agent_nav退出0；安装adapter/config/check及robot_go2配置
字节与源码一致。4秒只读订阅必需global Costmap，没有动作/速度/RGB/点云订阅：
首条真实header年龄1.615秒，在3秒预算内被接纳，后续真实1 Hz数据全部有效，
frame=map；证据timeout-fix/global-readonly.json。无法证明QPN失败瞬间的确切年龄，
但现有1秒判据与这次真实首条数据明确不匹配，不将全部globalfalse均归因年龄。

还确认三个点仍无法通过真实全局完整车体预检。另一次只读grid采样frame=map、
222×312、0.05米、origin=(-6.52,-5.34)，三个footprint均在边界内：
view01中心cost=74、body含253×18/254×1；view02中心173、含253×52/254×6；
view03中心253、含253×90/254×18。前两中心空闲不能代替完整车体检查。
证据timeout-fix/goal-footprint-details.json、global-grid.npz及goal-footprints.png。
这是当前Nav2数据下的判定，不是物理障碍根因/动态稳定性认证；不自动移动已测点，
不修改障碍阈值。需操作者看真实全局costmap重新测量可停驻中心/朝向再更新点列表，
否则即使ARM及global输入恢复也会合理EXHAUSTED，不能继续重复同一实验期待运动。

runbook补独立预算、条件化控制锁恢复及当前点受阻说明；bash块/链接与diff检查通过。
未执行完整Domain31、全量测试、运动/ARM/reset/API/录包，未改标记、提交或部署。


## 28. 用户要求采点RViz显示costmap、车体及点云（2026-10-09）

用户说明底层正在运行并要求显示local/global costmap、车体方框及点云，用于重新测量
探索点。只修改源config/go2_exploration.rviz和runbook，无导航/硬件代码及点位改动，
修改前配置快照保存在build/go2-goal-costmap-20261009/evidence/rviz-point-selection/。
旧配置只有Map/MarkerArray的历史记录仍保留；本节为当前配置变化。

新增两个Map显示/global_costmap/costmap、/local_costmap/costmap，使用costmap色表、
alpha=0.5/0.7，Reliable/Transient Local；Polygon显示实际
/local_costmap/published_footprint，绿色，Reliable/Volatile。只增加一个黄色
PointCloud2显示/vlm_nav/obstacle_cloud，Best Effort/Volatile、depth=1、Decay Time=0，
仅最新帧；这是Nav2已经使用的过滤障碍云，不启动新处理节点/完整原始点云诊断。
保留map固定坐标、俯视、当前点Marker及/clicked_point工具，新增Measure工具；
没有SetGoal/SetInitialPose。车体轮廓显示当前位置，不是远处候选点自动预览。

有界只读核对实际话题：两costmap及footprint各有一个真实Nav2发布者，QoS与配置兼容；
障碍云唯一obstacle_cloud_filter发布者为Best Effort。实际收到global grid header
frame=map（sec1791546939）、local grid frame=odom（1791546940）、footprint四顶点
frame=odom（1791546941）、障碍点云header frame=base_link（1791546826）。
只回显点云header，不打印/分析完整点；临时订阅均已退出。此处是输入证据，不是GUI
Status/TF渲染或长期稳定性验收。未自动启动第二个RViz或任何运动/ARM/reset/API/录包。

检查：YAML解析、所有显示类在本机Humble plugin XML存在、话题/QoS/启用项/单点云/
最新帧/map固定坐标/采点工具及无运动工具静态检查通过；runbook54个bash块、文档链接
及git diff --check通过。仅配置和文档改动，不新增镜像测试，不构建或重跑全量测试。
操作者可在现有RViz File→Open Config直接选择仓库源文件，或仅退出旧RViz后按
runbook在原Domain0终端重开，底层保持运行。四个新增显示Status=OK后才继续采点。


## 29. 四次点击替换为两个探索点及底层已运行的再实验指令（2026-10-09）

用户明确要求两点替代旧三点，并在底层已运行时给出RViz等终端顺序。
最初消息未包含坐标，已通过文本询问取得四条真实PointStamped；frame_id均map，
按中心/方向配对，header.sec依次1791547727、1791547731、1791547741、1791547743。
二维目标忽略点击z；yaw=atan2(hy-y,hx-x)，保留计算精度。源及当前安装scene现在只有：

| ID | x | y | yaw(rad) |
|---|---|---|---|
| go2_view_01 | -0.37453392148017883 | 1.9982248544692993 | 1.911663117782095 |
| go2_view_02 | 0.9363348484039307 | 2.5810399055480957 | 0.22931137952344943 |

原第三点删除，原三个点快照及用户四次xy/计算结果分别保存在
build/go2-goal-costmap-20261009/evidence/two-viewpoints/{scene.before.yaml,measured.json}。
其余scene键解析比较全部一致，目标灭火器、resources={}、enabled/验收标记及
robot_go2监护参数均未改；不伪造测量、报告或自动把点当成可停车/可达。

验证：两个中心与方向不重合、有限坐标/yaw、顺序/ID/长度、其余键未改、
measured_viewpoints入口检查及offline --require-ready退出0、单机静态门槛空。
已有test_go2_exploration.py 23项通过（无新增用例）；本轮不重跑全量或131项组合。
同一独立输出仅重建multi_agent_nav退出0，两个源码根参与发现，根vlm_nav及任务消息
依赖保留原安装；已检查installed scene字节等于源。runbook当前表、旧点证据说明及
底层已运行的A/B/C/D顺序已更新，55个bash块/链接与git diff --check通过。
上述报告和日志保存在two-viewpoints/{offline.json,tests.log,build.log}。

本轮只读节点列表显示相机、雷达/IMU/FAST-LIO、AMCL/Nav2及RViz运行，未发现
twist_to_go2_sport_bridge；6秒state查询未收到数据，不将底层运行当作已ARM。
操作者终端A保持底层；B复用RViz源配置；C已有唯一Sport转换则复用，缺失才启动并保持；
D加载当前overlay/源config，在现场静止/外部控制静默/急停条件下按状态reset或ARM，
只有ARMED后才新证据目录启动一次300秒完整探索。程序实际检查两点停车/规划、
执行路径/更新、取消终态/停止及目标扫描；已找到可提前结束，EXHAUSTED不等于全完成。
没有先做新点逐项真机测试，不自动启动运动/ARM/reset/API/录包，不提交或部署。


## 30. 两点真实运动后退出1：第二点Spin预测碰撞；API及朝向记录边界（2026-10-09）

用户反馈完成运动后停止、终端D退出1，并询问识别/API及持续旋转。
已读取本轮build/go2-field/explore.Lc9EsHBa/result.json及283条events，motion_sent=true，
两个VIEWPOINT_NAV均原生SUCCEEDED，stop_confirmed=true、motion_terminal=true；
第一点完成SCANNING/SCAN_RESULT→NOT_DETECTED，第二点开始SCANNING后首个
SCAN_SPIN原生ABORTED（seq279/native_status=6），随后RuntimeError→STOPPING→STOPPED。
这不是正常探索完成，但程序按已有Spin失败停止、返回1，与用户观察停止一致。
底层真实日志build/go2-exploration.tLYst4/live-20261008-01/ros-logs/
behavior_server_38728_1791545186485.log在1791548416.240454明确记录
Collision Ahead - Exiting Spin / spin failed / Aborting handle。
确认直接原因为Nav2旋转预测碰撞，不证明发生实际碰撞，也未判定障碍格来源。
本轮238次有效路径接收，最大已记录间隔0.052482812秒，没有PATH_RECEIVE_TIMEOUT。

第一点seq118 SCAN_RESULT保存4条A/灭火器结果，view_index=0..3，均NOT_DETECTED、
confidence=0.0、四个真实image_stamp，非API_ERROR。调用链明确Single ABCClient.infer_abc
→继承infer→_completion→OpenAI client.chat.completions.create→JSON/结构校验→Observation；
异常被AsyncVision转为安全泛化错误，tick中空results将产生API_ERROR，不会产生
NOT_DETECTED。因此可确认第一点四次识别返回有效负结果，不能据此证明图中无目标。
当前无逐次HTTP状态/request ID/model response/image文件记录，last_raw_response仅在
进程内存，退出后不能恢复。第二点在首次转向失败前可能已处理首视角，但没有整组
SCAN_RESULT，也未逐帧flush结果，不能推断该点API成功/未检出/检测到。
scan总是收集完4个配置方向再由explore判断检测；即使某方向已DETECTED，现有设计
仍会继续剩余方向，所以继续旋转本身不能证明没识别到。

核对实际controller参数：goal_checker_plugins=[general_goal_checker]，plugin=
nav2_controller::PositionGoalChecker，xy_goal_tolerance=0.25，yaw_goal_tolerance未设置。
本机安装头文件/opt/ros/humble/include/nav2_controller/plugins/position_goal_checker.hpp
明确仅检查XY忽略朝向。scan.initial取实际到点yaw，并非人工goal yaw：
点1goal yaw1.9116631178，实际停止yaw3.0096249931，差+62.90858°、XY差0.221026m；
点2goal yaw0.2293113795，实际yaw0.1232131880，差−6.07898°、XY差0.240765m。
可确认第一点首视角未对齐人工车头方向；这可能影响目标是否进入首图，不是漏检根因
定论，缺送入模型图像无法查验。后续若修复应优先Go2探索适配器显式以人工yaw对齐
首视角及逐视角API/图像证据，不直接更改既有单机Nav2成功栈或放宽Spin避障。

事件提取摘要保存build/go2-goal-costmap-20261009/evidence/field-scan-analysis/
derived-events-summary.json，标明原证据路径，未写入新验收结论或修改原实验文件。
Observation中的simulation=true是既有默认元数据未填，不是客户端切换mock的证据，
本轮单机日志仍不能当完整协同Observation消息验收。
本轮仅日志/代码/头文件读取、三个有限参数查询及交接更新，无代码修改、测试/构建、
API重调、运动、ARM/reset、临时图像/点云订阅、录包、提交或部署。

## 31. 单机探索逐视角VLM图像与响应记录（2026-10-09）

用户确认第一点到达时实际朝向已能看见目标，要求下一阶段保存VLM图像结果。
没有将第30节朝向偏差推断升级为漏检根因；此次限定于现有扫描证据，不改探索点、
目标描述、四方向扫描、检测判据、Nav2/避障或首方向选择。当前API返回文字/像素坐标，
结果图由本地按返回坐标生成，不宣称服务端返回了图像。

新增multi_agent_nav/scan_evidence.py，复用ArmImageRecorder.draw_target_annotation，
不使用其会清理旧会话的构造/保存流程，不增加依赖、RGB或点云订阅。
go2_check仅在explore中设置本轮output_dir/vlm；Go2Adapter从当前冻结snapshot取得
图像，在原AsyncVision worker写盘，ABCClient hook保留实际API JPEG字节和原始响应。
每方向独立随机目录，保存input.jpg、response.txt（有模型文字时）、result.png、
metadata.json，带点ID、view index、真实图像stamp/frame/sequence、提交时实际位姿、
人工导航目标/任务/native UUID、模型/request/response ID（实际SDK提供时）、耗时、
解析状态/像素坐标/置信度和安全错误类别。配置API密钥若被模型回显则脱敏；
不保存错误exception正文/请求头，不编造HTTP成功状态。文件flush/fsync，metadata原子替换；
独立目录不覆盖、不prune。已有现场实验未保存的原图/响应不能从旧NOT_DETECTED恢复。

每视角完成即记录VLM_VIEW_RESULT，consumer_status为APPLIED/ERROR/TIMEOUT/CANCELED，
后续Spin中止仍保留前方向图/响应；API/结构错误仍产生API_ERROR，不改为负检测。
正常写盘完成才推进，超时也等待旧worker结束才开始下一Spin，保持健康/整组deadline。
取消先实际cancel和zero，再记录取消事件。报告新增vlm_evidence_root及
vlm_evidence_pending：HTTP工作在取消/超时后可能晚返回，pending=true不代表证据完整；
晚结果不作为已应用识别，以事件为准，不能仅凭metadata校验成功当现场成功。
复审发现并修复scan=None后的晚写盘错误遗漏：独立_scan_evidence由tick在空scan前检查，
worker只传播evidence_write_failed/motion_fault，主循环继续实际停止；不在worker调用ROS。

本轮基线131项通过；新增15项本地SDK/图像/运动stub测试，合计146项通过。
覆盖精确提交JPEG、负/正结果标注、无效JSON、真实provider错误字段、密钥回显脱敏、
不可覆盖/写失败阻止请求、逐方向事件、后续Spin失败、取消先zero、worker线程写图、
超时不提前Spin、结束/取消后的晚写盘故障和后台失败标志。
既有2项observer测试短时localhost/Domain31节点检查，不是完整Domain31验收。
read-only reviewer独立运行新增15项并通过，晚错误修复复审无阻塞问题。
同一build/go2-goal-costmap-20261009输出仅重建multi_agent_nav退出0，两个源码根参与，
vlm_nav相机TF/消息依赖保留原安装；四个相关模块和robot_go2.yaml/scene.yaml源安装
字节一致、实际Python导入前缀正确。--offline --require-ready退出0，仍只是单机配置检查。
runbook更新文件/事件/pending语义及当前Python路径检查；55个bash块语法、本地链接和
git diff --check通过。真实证据存build/go2-goal-costmap-20261009/evidence/vlm-recording/
（baseline/red/integration-red/late-error-red/final/build/installed-check/offline）。

下一完整实验继续复用底层A、RViz B及唯一Sport转换C，D按runbook加载当前安装与
原有API/Domain0/enp4s0环境，现场静止/急停/ARM条件成立才以新证据目录执行探索。
观察第一点view_00的实际input.jpg和原始响应，按具体画面判断目标是否确实送入、
模型是否负检及像素坐标是否正确；不先扩展逐项故障注入或放宽Spin避障。
本轮未启动真实API/运动、ARM/reset、SSH/桥接、录包或新增现场订阅；没有提交，
尚未验证真机新增文件产出，未改path_source_verified/vision_validated等验收标记。

## 32. 用户Notes启动指引原地同步当前单机流程（2026-10-09）

用户要求审查并原地修改/home/isee-pst/Notes/协同VLM启动指引.md。
已保留原文于build/go2-goal-costmap-20261009/evidence/start-guide-review/guide.before.md，
修订版guide.after.md、guide.diff、environment-check.log及checks.json同目录。
原文仍加载go2-exploration.tLYst4、未加载相机TF overlay、旧goal/单点及完整协同门槛，
与当前实现不符。已原地改为go2-goal-costmap-20261009加go2-camera-tf，清旧PYTHONPATH，
区分底层复用与独立冷启动新预检报告，保留四终端角色/初始化条件/唯一Sport转换。
更新两个人工点、单机与完整live门槛区别、只读就绪检查、实际状态reset/ARM分支、
独立每轮证据目录、逐帧VLM图/响应/pending、路径5秒接收时限和停止/DISARM判据，
新增可选Publish Point记录。motion_sent=false时不声称存在stop_confirmed或认证其他客户端。

已检查15个Bash块语法、16个必要路径和本地链接，实际执行公共环境及纯Python路径检查；
四个模块实际导入当前安装且与源字节一致，两份配置源安装一致。写回前比对原文
未变化，原子替换保留文件权限；写回后核对内容/语法/链接/差异，无源代码改动。
未运行本指南的ROS查询/预检/launch/reset/ARM/运动/API/录包，未构建或重跑测试。
此前146项局部测试/包构建是第31节已有证据，不写为本轮新验收。

## 33. 固定 Go2 occupancy 地图家具语义录点（2026-10-09，等待人工分组）

用户授权仅对固定地图离线标注家具四角，明确不启动定位、底盘、导航、桥接或探索，
收到人工分组前不猜测家具对应关系。已核对分支go2-cooperative-task-layer、HEAD
ce2999a4ace8409a4244e4aac02080e9ad93f479及已有未提交修改，全部保留、不提交。

源maps/lab_rear120_20261001_IwQjMX/lab.yaml解析为同目录lab.pgm，222×312、
resolution=0.05、origin=[-6.52,-5.34,0]、mode=trinary、negate=0、阈值0.65/0.25。
YAML SHA256=b928cdb28dab483674c49ab269ee9818cf8cf5d4fd147f2314d193e93877532f；
图片SHA256=90ab030608cba960b42922507a851760a7de2405b27665a65781c40e99eabe36，
本轮复查两文件摘要未变。语义身份为source_robot=go2、frame_id=map、units=meters；
Domain31仅通信环境，不引入Ranger坐标一致假设。

新增scripts/record_semantic_points.py、scripts/render_semantic_map.py和
config/go2_semantic_annotation.rviz；复用已安装PyYAML/Pillow/rclpy，不新增依赖、
不改导航配置或原探索RViz。录点每次独立session/文件，逐点原编号、时间戳、frame、
有效性与flush，错误frame或非有限xyz仍保存但不用于家具。渲染模块build_semantic
接受明确分组并检查重复/无效/退化/自交/边界，渲染前复核源/记录SHA256和polygon
对应原point_indices；原图6倍最近邻放大，处理yaw/纵轴，彩色轮廓/轻填充/ID及引线，
PNG更新先保留.bak再原子替换。复用步骤见[家具标注说明](semantic_map_annotation.md)。

新增test/test_semantic_map.py的17项针对性测试通过，覆盖录点立即可读与无效点、
不覆盖、yaw/纵轴投影、分组错误、源图保护、摘要变化、原始点一致性、旧PNG备份和
ID重叠引线；AST语法、专用RViz YAML、文档Bash块及git diff --check通过。
本轮没有运行既有全量测试或构建。独立ROS自检在
build/go2-field/evidence/semantic-annotation.l6fgnsxj/selfcheck/保存四条合成消息，
有效性=[true,false,false,false]、编号1—4、原始stamp保留，Ctrl+C退出0；仅使用独立
/semantic_annotation_selfcheck话题，未向/clicked_point发合成点，不属于家具记录。

沙箱不同执行会话不能发现地图节点，同一执行环境configure成功，精确传输原因待验证。
沙箱内/dev/dri缺失且GLX BadValue；经自动批准的实际桌面只读glxinfo确认NVIDIA
OpenGL4.6正常，随后仅在桌面环境启动本次离线A/B/C窗口。过程中的首次CLI YAML
解析因末尾---失败，改为safe_load_all；一次启动检查误要求RViz固定节点名，
关闭了已能显示地图的窗口。用户已点击的2个有效原始点完整保留于
maps/lab_rear120_20261001_IwQjMX/clicked_points_20261009T132553.172783Z_988cd57e.jsonl，
另有早期空session记录，也保留；不自动合并或重编号。已向用户说明并恢复窗口，
启动就绪改用实际地图加载证据。

当前运行：Domain31、localhost_only=1、rmw_fastrtps_cpp，公共环境清除
PYTHONHOME/PYTHONPATH/CYCLONEDDS_URI，未加载go2_network_env.sh，也未发现
继承的Fast DDS profiles/discovery server覆盖。A固定map_server namespace为
/go2_point_annotation；configure、activate各收到Transitioning successful，实际
OccupancyGrid为222×312、resolution≈0.05、origin一致。C用专用RViz俯视地图，
截图已确认固定地图Status: Ok、尺寸及分辨率正确；无实时显示。B已就绪订阅
/clicked_point，BEST_EFFORT/Volatile兼容RViz发布，实际录点路径为
maps/lab_rear120_20261001_IwQjMX/clicked_points_20261009T132715.398576Z_38a467b1.jsonl。
已告诉用户可以每件家具按顺/逆时针依次点击四角，每次确认B的编号；新session从1开始。

当前证据：build/go2-field/evidence/semantic-annotation.3q4zom8i/，包含
configure.log、activate.log、map-info.yaml、rviz.log、manifest.json、rviz-status.png。
build/go2-field/evidence/semantic-annotation-current.json记录本次路径及桌面进程ID；
进程状态跨轮须重新核实，不能把manifest当当前运行事实。当前监督exec session=78587，
桌面A xterm PID52782、B xterm PID52884、RViz PID52883；只用于识别本次窗口。

**待办：**用户在B Ctrl+C结束录点并提供家具点号/ID/类别后，再读取实际JSONL，
校验四角与人工分组，保留旧文件后生成semantic_map.yaml和annotated_map.png，展示
图片等待对应关系核对。目前尚未生成这两份最终文件，不存在已确认家具数量。
用户可引用早期session，但任何跨session分组都须明确，不擅自补成四角。
结束顺序为B停止并确认保存→关闭C RViz→A Ctrl+C关闭离线map_server；xterm以
-hold保留退出输出，随后可关窗口。未启动任何真实动作/API/SSH/桥接/录包或更改验收标记。

**第33节采点进展：**当前session实际收到8个有效map点。用户说明1—4是“四张椅子的
桌子”，5—8是“并排的桌子和椅子”，随后明确选定ID=table_01/table_02、category=table。
几何校验确认1→2→3→4自交，5→6→7→8有效；1→2→4→3是有效候选，但没有擅自
采用。已展示同源底图clicked-points-review.png，红线原顺序/绿线候选/蓝线第二组。
用户明确选择重新点击第一组四角，已要求继续当前session沿外围顺/逆时针选点，
预计编号9—12，每次确认B打印编号，完成后B Ctrl+C并提供新点号。原1—4完整保留但
不用于table_01；table_02仍使用5—8。待收到重选点后再校验，不提前写最终YAML/PNG。

**第33节重选交付进展：**用户确认重选完成，当前session实际共12个有效map点。
9→10→11→12及5→6→7→8均通过四角/自交/退化/边界校验，原始顺序和坐标未改动。
用户虽完成采点，B的python3 PID52888仍在运行；核对其命令行、输出目录及Domain31/
localhost环境后，仅向该录点进程发SIGINT，正常退出并保留12条原始记录。RViz和A
离线map_server暂保留，未停止用户其他进程。

已在源地图目录生成semantic_map.yaml及annotated_map.png（1332×1872，原图6倍最近邻）。
table_01：category=table，name=放了四张椅子的桌子，point_indices=[9,10,11,12]；
table_02：category=table，name=并排的桌子和椅子，point_indices=[5,6,7,8]。
excluded_points明确记载原1—4因用户选择重选而排除，但原JSONL不删改。
原始选点session=20261009T132715.398576Z_38a467b1，记录SHA256=
4901299861118264479b54e3518eb899c440ed5e76b3674f3e783cc9ef114219。
生成前检查无同名旧交付；YAML经解析比对后原子保存，渲染器再次核对源/记录摘要与
原始point_indices。输出PNG已打开检查，ID清晰、两组不重叠。独立复核YAML可解析、
源地图两份SHA256保持第33节实查值、两组polygon精确对应原点击、PNG可解码，
git diff --check通过；工具代码没有新改动，17项测试是此前本轮已执行证据，未重复运行。

证据同semantic-annotation.3q4zom8i目录，新增confirmed-groups.json及annotation-report.json。
已展示完整标注图并询问用户核对：右侧蓝色table_01为四椅桌，左侧橙色table_02为
并排桌椅。**等待用户视觉确认，未将对应关系验收标为通过。**确认后可结束窗口：
B录点已停止→关闭C RViz→A Ctrl+C结束离线map_server，保留退出日志后关xterm。
本次无导航/定位/桥接/真机动作、API、构建、全量测试或提交。

**第33节人工核对完成：**用户明确确认右侧蓝色table_01为四椅桌、使用点9—12，
左侧橙色table_02为并排桌椅、使用点5—8。家具语义交付已获人工对应关系确认；
当前semantic_map.yaml、annotated_map.png及原始JSONL保持不变。annotation-report.json
及semantic-annotation-current.json的user_visual_confirmation更新为true，后者状态为
ANNOTATION_CONFIRMED。录点已正常停止；关闭窗口顺序仍为C RViz→A Ctrl+C结束
离线map_server，再关保留输出的xterm。此项仅完成离线家具标注，不产生导航验收结论。

## 34. 四终端公共环境、时间目录及终端输出精简（2026-10-11）

用户要求简化协同VLM启动方法。本轮保留当前Go2单机四终端流程及全部未提交修改，
在仓库根新增[CoVLM_env.bash](../../CoVLM_env.bash)。四个Bash终端均执行
`source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash`，脚本切换至仓库目录，
设置REPO、GO2_COOP_OUTPUT、GO2_EXPLORE_OUTPUT、GO2_CAMERA_OUTPUT、
GO2_EXPLORE_CONFIG、VLM_NAV_EXTRA_SETUP_FILES及ROS_LOG_DIR。按原顺序加载
Humble→XT16→Go2→go2-coop→go2-goal-costmap-20261009→go2-camera-tf，最后复用
go2_network_env.sh核对enp4s0及192.168.123.x、配置Domain0/CycloneDDS。
source前检查环境文件可读，任一加载/网卡检查失败返回非零；只加载环境，不启动节点，
不保存或打印API密钥。实际source检查通过，导入模块确认位于当前探索安装，未由旧overlay覆盖。

已原地更新/home/isee-pst/Notes/协同VLM启动指引.md及仓库go2_runbook.md。
每轮探索路径由时间生成：`$REPO/build/go2-field/explore.$(date +%Y%m%d_%H%M%S)`，
例如explore.20261011_143025；入口仍独占创建目录，已存在则拒绝，不预先mkdir或覆盖。
Notes中的冷启动live目录和可选坐标记录也改用时间，创建失败阻止后续预检/采点。
任务/native UUID等协议身份不变，VLM逐视角证据沿用原实现。

go2_check新增terminal_report，仅在emit时把footprint/path坐标数组替换成
footprint_count/path_count；保留拒绝原因、健康门槛、动作结果、停止确认、证据写入和
pending等排障字段。result.json保留完整坐标，events.jsonl保持原记录；无控制逻辑改动。
新增1项输出测试验证精简、故障字段和原报告不变；新增3项公共环境测试验证真实overlay
加载、Domain31旧环境被切换、旧PYTHONPATH清理、网络失败阻止继续及必须source。
网络探针使用stub，不连接机器人。既有field_preparation/exploration共87项加新增4项，
合计91项通过；这些运行检查使用fake/stub，不启动ROS节点或真实动作。

按runbook在原build/go2-goal-costmap-20261009输出仅重建multi_agent_nav，两个源码根
参与发现，构建退出0；vlm_nav和multi_agent_task_msgs复用既有overlay，colcon相应提示
符合预期，不重建外部安装。实际导入go2_check路径及源码/安装字节一致，已实测精简输出。
CoVLM_env.bash语法、Notes的16个和runbook的55个Bash块语法、本地链接、git diff --check
通过。Notes原文保存于build/go2-field/evidence/startup-simplification-20261011/guide.before.md，
同目录guide.after.md及guide.diff；写回前比对原文未变化，原子替换保留权限并复查内容。
未执行现场预检/launch/ARM/reset/导航/真实API/录包、全量测试或提交。

## 35. Go2 家具选择策略：离线实现，现场绑定待确认（2026-10-11）

新增 [Go2 语义搜索说明](go2_semantic_search.md)、`semantic_search.py` 和
`maps/lab_rear120_20261001_IwQjMX/observation_points_go2.yaml`。
唯一控制链仍为 go2_check → explore → Go2Adapter；没有新增运动节点、global planner、
双机协议或底层参数改动。fixed_viewpoints 默认，semantic_furniture 通过原入口 --strategy 选择。
复用原 API 客户端及单个 AsyncVision worker，发送标注地图和候选/历史，严格解析家具 ID，
程序解析本机 map 位姿；候选规划和选中点复查后才调用原 NavigateToPose/旋转观察。

新增地图与语义资产 SHA 校验、候选过滤、每视角观察状态、失败冷却、迟到/取消结果拒绝。
观察点到达不置 READY，不把 API 失败当未检测到。全完成 EXHAUSTED 仅表示预设视角检查完成；
暂时无候选 WAITING_VIEWPOINTS，退出等待人工处理。选择事件与原 vlm/ 相机证据分开。

**待用户回复的实际配置/范围问题**：go2_view_01、go2_view_02 分别对应 table_01 还是 table_02；
单机首次阶段是否保持原“发现 A 即结束”，还是本轮也增加目标接近。尚未收到答复。
原两个精确位姿已抄入观察点目录，家具 ID 留 null、enabled=false，语义策略离线检查预期拒绝；
不能猜测绑定后启用。当前实现保持单机发现即报告，未新增接近动作；双机 AgentMachine 原
APPROACH_PICKUP→READY 和 C 屏障不变。当前房间未知，scene 的 B/C 仍待现场填写。

证据目录 `build/go2-field/evidence/semantic-search-20261011/`：before/ 为本轮前已有探索代码快照；
baseline.log 为原119项测试通过，tests.log 为同119项加新增28项，合计147项通过。
新增覆盖非法/额外字段、地图/图片变更、缺失坐标、模型超时和取消迟到结果、候选路径失败、
原生 map 坐标不二次转换、fake 两点反向选择/观察/去重、图像API失败区别、离线不初始化ROS。
独立只读审查的3项P2均修正，超时边界、标注资产更换和旧固定配置无关性均补测。
在原 go2-goal-costmap-20261009 输出只构建 multi_agent_nav 成功，4个安装模块字节与源码一致，
未重建外部 overlay；build.log 记录预期依赖复用提示。offline.json 确认真实目录绑定未填时
退出1、motion_sent=false。本轮所改跟踪文件 diff --check 通过；仓库整体检查在用户已有
docs/测试与运行日志go.md:143/147 报空格+Tab，保留未修改。
未运行真实 API、ROS 节点/导航/旋转、双机现场实验或全量测试，未提交。

## 36. 离线 RViz 家具/观察点叠加层（2026-10-11）

用户要求补可视化层以核对家具绑定。新增 scripts/show_semantic_viewpoints.py，读取原语义图
和 observation_points_go2.yaml，校验源图摘要/版本、家具多边形和有限位姿，允许未绑定的
disabled 点用于人工查看。仅发布 /go2_point_annotation/semantic_markers MarkerArray，
reliable/transient_local，并每秒重发；无导航、速度发布、订阅或配置写入。
原家具蓝/橙轮廓与ID，停靠点朝向箭头、UNBOUND/disabled 标签；已绑定点显示家具颜色和
关联线，关联线不是规划路径。重启发布器清除旧标记。原图、点位和绑定状态均未修改。
config/go2_semantic_annotation.rviz 已添加 MarkerArray 显示，保留 Publish Point。
Domain31 三终端启动、成功判据和退出步骤见 [语义搜索说明](go2_semantic_search.md) 的 RViz 节。

新增 test/test_semantic_viewpoints.py 9项，连同既有 semantic_map 17项，26项通过；使用
Humble 消息类构造验证，不初始化ROS。--check 对真实两点目录退出0；文档Bash块语法和
链接通过。未运行RViz GUI/ROS节点、真实机器人或API，未构建/提交。屏幕上的文字可读性
及实际家具绑定仍待用户查看确认，不把可视化当现场可达性验收。


## 37. 新家具观察点绑定（2026-10-11）

用户在 RViz 判断原两点不适合家具观察，重新按中心→方向点击并提供
clicked_points_20261011T094800.286779Z_d37f638f.jsonl。四条均为有效 map 有限坐标。
按约定1→2绑定table_01，3→4绑定table_02，新增table_01_view_01与table_02_view_01：
(1.1407206058502197, 1.5659018754959106, 0.1139940814558879)、
(-0.4178990423679352, 1.4312951564788818, 2.10969554928474)。yaw为弧度，未擅自转向家具中心。
observation_points_go2.yaml 保存原始session、点号、文件SHA256；新点暂时disabled，
待RViz与现场验证。原两点移到archived_viewpoints保留，源文件另留时间命名.bak；
原scene固定点列表、原图、语义家具多边形和原始JSONL不变。
已检查两中心在地图内且不在家具多边形内，中心像素254为空闲，仅为静态单格检查，
不表示整机footprint、路径、旋转或相机视野通过。可视化--check通过；重启终端B标记程序
即可显示两个新点和绑定线，旧点不显示。未启动ROS或真机运动。

## 38. 用户授权寻找灭火器并接近：执行准备完成（2026-10-11）

用户已确认RViz新点并明确授权“提示灭火器在table2附近，完成寻找并走到灭火器旁边”。
后续明确现场有人可急停、底层未启动。已要求按既有启动指引0–3节冷启动，保持DISARMED，
实际AMCL初始位姿由现场按实物设置；目前等待该步骤完成，尚未运行现场检查/ARM/运动/API。
授权为本轮Go2单机实验，不扩展Ranger或双机协同。

新两点已enabled=true，原文件另有时间命名.bak，validation_status明确现场仍待验证。
新增显式--search-hint仅传家具选择，--approach-target在有效几何检测后调用原select_approach，
保留0.9m standoff并校验其正有限性；使用共用navigate监控/停止逻辑，重查局部/全局footprint、
规划和资源范围。未定位检测不接近；TARGET_APPROACHED且stop_confirmed才算完整成功，
开启接近时其他终态退出1。默认未传开关仍保持发现即结束，不发布READY。
修复首次家具决策前started=false导致AsyncVision未poll的问题，只轮询结果，不触发动作。

相关旧156项加新增接近5项和距离边界5项，共166项通过；安装构建成功，离线语义目录门槛为空。
配置了API密钥/模型/接口（仅检查布尔存在性，不记录内容）。独立只读审查通过，提出距离校验已补。
本轮命令及前置顺序见go2_runbook末尾“2026-10-11：家具提示搜索并接近灭火器”。
未运行真实API或运动，不能报告实验成功；等用户确认冷启动就绪后先做inspect，再根据Sport状态
处理ARM并运行一次300秒实验，保留时间命名证据，结束确认停止并DISARM，不自动重复实验。

## 39. 首轮家具选择→灭火器识别→接近实机完成（2026-10-11）

用户确认底层就绪后执行一次授权实验。先只读inspect：定位/静止、地图匹配、costmap、
RGB-D及深度单位有效；Sport DISARMED。ARM服务success=true后运行semantic_furniture，
目标“灭火器”，search_hint“灭火器在 table_02 附近”，--approach-target，timeout300。
证据 build/go2-field/explore.20261011_181136/，进程退出0，实际事件约45秒。

VLM因提示选择table_02→table_02_view_01；VIEWPOINT_NAV成功且停止确认。
四个观察方向的前三个NOT_DETECTED，第4个(view_index=3)DETECTED、confidence=.95、
position_valid=true。图像vlm/aac8b5a07d7b4d15934841f5dfc24ede_view_03/result.png可见标记灭火器，
原input.jpg、response.txt、metadata.json齐全，另三视角也均留存。
投影目标本机map=(-1.0869788114,2.8084497316)。计划接近位姿
(-0.4505827083,2.1720536285,2.3561944902)，名义standoff=.9m；PICKUP_NAV返回SUCCEEDED。
最后停止时定位约(-0.3508609335,1.9471544507,2.0992570279)，据投影目标估算距离约1.13m，
不是实尺距离，不能把计划0.9m当实际停车精度。未为缩短距离追加运动或改Nav2容差。

最终result=TARGET_APPROACHED、stop_confirmed=true、evidence_write_failed=false、
vlm_evidence_pending=false。完成后调用disarm，success=true/DISARMED。底层保持运行，
未重复实验。experiment_summary.json记录关键结果。仅证明本轮Go2单机流程完成，
不更新双机READY/C、vision_validated等通用验收标记。后续新实验需新的明确授权。

## 40. 到达距离闭环复用：源码和隔离构建完成，未实机执行（2026-10-11）

用户要求复用原VLM_Nav闭环以满足原距离值；未授权新一轮运动。核对原配置为
approach_cancel_radius=.89、target_success_radius=.81、arrival_retry_limit=1，
并非先前协同候选点的standoff=.9。新增vlm_nav/arrival.py共享距离、停止半径及到达判据，
原VLMNavigator改为调用共享函数，原控制流程保持。Go2 --approach-target 现在在
Nav2终态/距离触发取消后确认停止，再使用新鲜定位计算到投影目标的平面距离；
≤.81才TARGET_APPROACHED，否则最多一次安全候选重规划，再失败为APPROACH_DISTANCE_FAILED。
纠偏以实际超距缩小候选半径，不能通过costmap/路径/资源检查则失败，不绕开避障或修改Nav2。
原select_approach新增可选keyword距离覆盖，旧调用保持默认；原配置文件列表在robot_go2关联。

复核246项既有相关用例和10项新增到达/纠偏/边界测试，总256通过；独立只读审查无阻断项。
两源码根、vlm_nav与multi_agent_nav在build/go2-arrival-contract-20261011隔离构建成功；
已从该输出导入两模块，配置门槛通过，实测安装判据将历史1.133m判为失败。
构建仅有PCL CMake开发提示、根包未用COOPERATIVE_RUNTIME_PYTHON提示及接口包依赖复用提示。
未覆盖go2-camera-tf等原安装，未修改CoVLM_env.bash默认，不切换运行节点、ARM、运动或调用模型。
下次实验必须最后source新安装（命令见runbook新小节），不能继续用旧安装的宽松完成判据。
根本限制：此契约基于本机定位/深度投影，不保证实尺精度；历史实验记录保留其当时口径。

## 41. 用户授权3秒裁剪后点云诊断（2026-10-12）

用户目测头部到前障碍约20cm，图上仅1–2格；明确授权一次3秒采集。订阅已有过滤输出
/vlm_nav/obstacle_cloud，3秒窗口31帧（首末2.997596s）、91635点；同步map、local/global raw
costmap、footprint、odom快照和TF后销毁订阅。无完整点云/RGB-D常驻订阅、参数更改或运动。
证据build/go2-field/evidence/cloud-clearance.20261012_205145/，含原PointCloud2 CDR、逐帧npy、
合并PCD、参数dump、TF、对照图、分析脚本及report.txt。

实际运行footprint x±.38/y±.18、padding0；自裁剪box x±.45/y±.25/z[-.4,.2]，voxel .05；
三图resolution .05、inflation .55/cost_scaling5。以footprint前缘而非未测量鼻端为距离基准，
|y|≤.18：点云最近间隙7.3–14.2cm/中位9.2cm，局部/全局致命格中心8.7/9.2cm，近边约6cm；
原/map占用格中心24.9cm、近边21.6cm，非一两格。裁剪盒内0输出点，但盒外x .45–.60范围
144点在31帧均出现、z .014–.070m，导致costmap近距致命格；膨胀进一步扩展代价。
不能把该现象仅归因于inflation。静态地图上近点可能是RViz实时层叠加；当前数据未证明地图
原图障碍有同样近距。近点是自身残留还是真实突出障碍尚待实物确认，不能贸然加大裁剪盒。
原/map服务路径已核对为lab.yaml。最后odom速度0；完整轨迹未录，合并PCD未补偿运动。
用户障碍类型/目测参考部位问题尚待回复。原图和所有导航配置保持不变。

### 2026-10-12 强度链路与原生过滤只读审计

详见 [intensity_audit_20261012.md](intensity_audit_20261012.md)。复用已有31帧CDR和历史全链路bag；转换器27帧强度/通道逐数组一致，FAST-LIO base输出保留强度但丢ring，障碍VoxelGrid平均强度（2、10→6离线断言通过）。用户确认地图障碍实测偏差≤5cm；疑似ROI144点全零，但地图对照3517点中5%也为零，37点标签重叠另列歧义。暂不设软件阈值/ROI，不修改生产代码。雷达自身只读Web查询：PandarXT-16，SW1.0.15/FW1.2.8，InterstitialPoints OFF，Linear Mapping。未改变硬件设置、未新建ROS订阅或现场实验。证据 `build/go2-field/evidence/intensity-audit-20261012/`；仍缺同距离黑色障碍保留、通道对应与现场标记/清除验证。

### 2026-10-12 21:25 约25cm位置复查

用户恢复雷达/FAST-LIO后，主机Domain0发现障碍过滤输出，短时采集31帧/2.9988秒/87903点，订阅已退出。证据 `build/go2-field/evidence/cloud-clearance.20261012_212555/`，含CDR、当前过滤参数、XYZI、统计与预览。自裁剪和5cm体素参数未变。前方|y|≤.18近点x=.4521–.4824m，中位.4685；按历史footprint前缘x=.38参考，间隙7.2–10.2cm、中位8.8cm（不是实测鼻端距离）。先前疑似ROI仍31/31帧出现，共200点、z=.0129–.0687m、强度0–2/中位.25，其中98点为0；与旧144点全零相比仍为低强度但并非仅零强度。未改变过滤器或雷达设置，无运动。最后odom速度为零，但无全时段轨迹证据。当前无costmap，采集未获取map→base_link，不能以旧定位对齐新点或确认地图对照标签。前一次21:19恢复前尝试为0帧，不算有效采集。

### 2026-10-12 强度≤3 A/B准备（未现场验收）

用户明确改为全输出“不过滤/删除≤3”对照，并要求低反射临时物体保留、costmap清除和旋转定位连续性验收。实现见 [intensity_ab_20261012.md](intensity_ab_20261012.md)：现有节点增加默认关闭、启动只读的intensity_filter_enabled/intensity_threshold，固定体素后，Go2 launch可覆盖。错误强度字段/非有限强度拒绝帧并报错。原始雷达/FAST-LIO不变，/scan消费者会受影响。实时查询AMCL未发现节点，未部署/重启底层/启用滤波或旋转；物体材质尺寸与摆放仍待现场提供。

隔离构建 `build/go2-intensity-ab-20261012/install` 成功（仅vlm_nav根包，两处源码搜索）；默认环境未切换。C++6项通过（原2项+新增4项）；相关Python40项通过、1项已有注册清单差异失败（CMakeLists已注册test_go2_camera_tf.py而旧expected未列，本轮未改这两处）；全仓diff检查另报既有运行日志Markdown两处缩进，未修改无关内容。新增脚本/launch语法检查通过。离线同数据阈值比较：旧91635→63566、新87903→59100；疑似ROI144→0、200→0；旧地图对照删除1204/3517=34.23%，尚不证明真实障碍持续标记。脚本 `scripts/compare_obstacle_intensity.py`，结果各采样目录ab_le3.json。现场A/B未完成，不能宣称通过。

补充验证：source同一隔离install，在Domain31/localhost/FastRTPS运行原障碍过滤运行时测试，1项通过（TF未就绪时节点保持存活），未连接真机Domain0。该测试不替代现场强度与定位验收。

### 2026-10-12 22:04 黑布灭火器指定区域观测

用户放置黑布包裹灭火器，约右前45°/base_link距离1m；按中心(.7071,-.7071)、水平半径.30m采集3秒现有裁剪输出，30帧84760点。证据 `build/go2-field/evidence/cloud-clearance.20261012_220447/`，完整CDR、filter.yaml、target_statistics.json、target_xyzi.npz、analyze_target.py、target_ab.png。当前节点未加载新增强度参数，基线仍原过滤（高度[-.2,.2]、voxel .05）。圆内1143点、每帧36–39；离线删除≤3后988点，86.44%保留、每帧31–34，30/30帧均有点。强度中位30.13、P25=10、P75=42.37、最大53。区域点均值(.909,-.662)，范围x[.816,.988]/y[-.858,-.519]/z[-.159,.041]；均值不是目标几何中心。区域可能含背景，未通过移除差分单独确认所有点属于灭火器/黑布。当前未获取map→base_link、无costmap，故未验收标记清除/旋转定位。未启用现场阈值、无运动、短时订阅已退出。22:02恢复前0帧不算有效观测。

### 2026-10-12 强度现场A/B中止于定位错位

详见 [intensity_ab_field_20261012.md](intensity_ab_field_20261012.md)。证据 `build/go2-field/intensity-ab.20261012_222102/`。用户确认有人急停并授权导航旋转；灭火器已移动，随后取消该目标测试，改用table_01/02观察点。原地静止A/B曾启用≤3（仅过滤节点），随后恢复旧二进制/原参数。A table_01导航+四向扫描成功，但map→odom单次修正21.26cm；A table_02到点后首个Spin因Collision Ahead中止，停止确认true。到点扫描阶段map→odom单次修正42.91cm，TF stamp在Spin开始之前，不能认定旋转导致。用户明确确认明显定位错位，停止后续运动。B两点运动未执行，阈值验收未通过。

最终Sport桥已请求DISARM成功，关闭本轮启动的桥；不过滤节点恢复、其他底层保留。恢复过滤节点由独立进程运行（工具session14612），不是原launch子进程，后续停底层须一并结束以免重复发布。无新的完整原始云订阅，bag仅分析必需话题并已关闭。不能以旧ROI静态致命格证明灭火器仍在/清除失败；静态地图自身覆盖该区域。下一步先定位排障，再恢复同位姿A/B，不能把重新定位后的B与错位A直接算单变量通过。

### 2026-10-12 22:46 静止IMU检查，B组再次未运动

用户重启要求B后发现旧恢复过滤22070与新24112重复，清旧并将新实例替换为≤3进程24857；未ARM，只读门槛未通过。用户中断改查静止漂移，期间底层再次重启形成24857与新25486重复；已终止本轮B实例24857，保留新原过滤。之前恢复独立实例22070已清，不再存在。B证据 `build/go2-field/intensity-B.20261012_224047/`，无运动。

12秒IMU/lowstate/odom/TF诊断证据 `build/go2-field/evidence/imu-stationary.20261012_224629/`，含bag、分析脚本/JSON/图、实际参数和FAST-LIO日志、report.txt。5887条IMU约498.9Hz，无非正stamp间隔、最大9.56ms；加速度模长9.61m/s²，gyro均值约[-.094,-.094,.191]deg/s；FAST-LIO初始化归一化重力、估计gyro偏置，不能由原始均值判故障。同期odom XY净漂移.8cm、X/Y波动2.2/1.6cm，map→odom不变；未复现几十厘米漂移。odom twist始终零不等于pose不动。IMU接收时戳不证明硬件同步。日志启动有Lidar loopback clearing buffers且之后有No point skip，需要后续跟踪雷达/去畸变链，根因未定。无新点云订阅、无参数改动或运动；短时bag已退出。

### 2026-10-12 22:50 B组两点完成（有限场景候选）

[详细结果](intensity_B_result_20261012.md)。用户恢复地图后授权B；证据 `build/go2-field/intensity-B.20261012_225049/`。≤3真实启用、其他参数保持，table_01/02导航+四向观察全部成功/停止确认。table1首包SQLite锁异常，后两次旋转缺包；保留并用resilient录包补做一次table1，完整包与table2均integrity ok。两点扫描原疑似ROI(0.45<x<.6,|y|<.18)点数和local/global致命格均0；map→odom最大单次平移11.57cm/6.10cm、角度1.64°/2.93°，用户确认未见明显错位。只支持当前两点阈值候选，不是定位精度/所有真实障碍验收；A2错误定位，B重启重定位且table1复测，不能作严格同初值因果比较。

Sport已DISARM并关闭。B过滤停止，原不过滤恢复；默认环境未改。恢复进程本轮绑定底层launch PID25313寿命，launch退出时自动SIGINT清理（工具session43713），避免再出现重复发布者。低反射灭火器动态标记/清除按用户要求未测，禁止把旧保留率当全面避障PASS。后续启动B仍需显式实验overlay/参数。

### 2026-10-12 23:03 table_02 B组人工观察复测

用户明确要求再测table2并人工观察拉丝重点区域。证据 `build/go2-field/table2-B-review.20261012_230302/`；唯一过滤节点切为体素后≤3，其他参数不变；既有执行器到点成功、四方向扫描成功、stop_confirmed=true。用户明确反馈“未见近处致命障碍”。离线扫描窗口原ROI(0.45<x<.6,|y|<.18)点数、local/global致命格min/median/max均0；map→odom最大单次平移5.876cm、角度2.622°。支持table2当前场景拉丝抑制效果，不扩展到所有实物保留或全场景定位精度。

录包resilient正常结束，含必要输出话题，无原始完整云额外订阅。结束DISARM成功、关闭本轮Sport桥，恢复原不过滤节点，恢复节点随当前底层launch25313退出（工具session见本轮cleanup.json）。默认阈值未永久启用。用户可据此选择显式≤3候选用于后续单机实验；黑布临时障碍动态标记/清除仍未测试。

### 2026-10-12 用户指定Go2默认过滤≤3

修改config/robot_go2.yaml默认intensity_filter_enabled=true、threshold=3.0，go2_system launch同名默认true。保留裸节点/Ranger默认false。独立构建build/go2-intensity-default-20261012/install成功；CoVLM_env.bash新增GO2_FILTER_OUTPUT并最后source此overlay，VLM_NAV_EXTRA_SETUP_FILES同样更新。原实验overlay未覆盖。6项C++过滤测试+31项Python Go2配置/定位测试通过，bash语法及安装配置默认值断言通过。

当前运行旧过滤节点未重启，仍不过滤；下次重新source环境并正常重启底层才默认开启。回退在原Go2 launch追加intensity_filter_enabled:=false。runbook保存完整构建/生效顺序。用户只要求更改默认，本轮未运动或重新现场验收，范围仍限已测单机场景。

### 2026-10-13 探索RViz改为白→灰→黑障碍梯度

用户要求取消亮色，并明确空闲白色、障碍程度增加时从白到灰到黑。
config/go2_exploration.rviz 的local/global costmap改用map色表，原/map同样使用map色表；
保留原Alpha .5/.7，叠加后黑色障碍可能显示为深灰。内置map色表的未知值-1仍为灰绿色，
不属于障碍梯度；没有增加自定义插件或转换节点。车体轮廓RGB(80,80,80)，过滤障碍云黑色。
探索点发布源码改为默认中灰(.45)、ARRIVED/TARGET_FOUND/TARGET_DETECTED_UNLOCATED深灰(.2)、
原失败/停止状态黑色，原状态文字保持。
YAML解析、Python AST检查通过；既有marker坐标/标签测试1项通过（35项未选择），无新增测试。
未构建、部署、启动ROS/RViz或运动。重新打开仓库源RViz配置可加载显示配色；探索点颜色修改
仅在源码，CoVLM_env默认使用的go2-goal-costmap-20261009安装副本仍为旧彩色，须后续更新
安装并在下一次已授权任务启动时加载才生效，不应为改色重启正在执行的任务。

### 2026-10-13 接近实验未检出：离线相机/API证据核对

用户反馈Image-time TF pending和持续扫描，怀疑图像上传不及时；本轮目标灭火器，用户仅肉眼观察，未查看相机实时画面。
本轮build/go2-field/explore-approach.20261013_173653：两个fixed_viewpoints各四方向，8次API均返回、结构校验成功、consumer_status=APPLIED，模型均NOT_DETECTED；最终EXHAUSTED、stop_confirmed=true、写盘/pending均false。
启动警告1791884213.828514929对应图像超出TF最新数据23.670ms；本机日志仅一条匹配，不能据此判上传失败。
8张实际请求input.jpg摘要互异；metadata/events时间换算估计图像stamp到request_started为0.101–0.173s，API相关处理latency为2.217–3.029s；不等同服务端上传完成时间。
人工离线查看输入图以桌椅、墙面和设备为主，部分模糊，未见清楚可辨灭火器；上一轮explore.20261013_172315/view_03成功图可清楚看到墙边红色灭火器。
当前证据支持继续核对相机视野/拍摄方向，模糊为可能因素；未确认实物位置、四向扫描盲区或定位误差根因，不改阈值/扫描动作。
报告及8图拼图：build/go2-field/evidence/vlm-input-audit.20261013_173653/{report.json,submitted_images.png}。仅离线读取和生成诊断图/报告，无ROS订阅、API调用、ARM、运动或部署。

### 2026-10-13 接近阶段APPROACH_UNREACHABLE只读分析

最新实验build/go2-field/explore-approach.20261013_175044，第4视角DETECTED/.95/position_valid=true，原result.png清楚标注墙边灭火器；投影map目标(1.3758433843,3.8639094103)。
事件1791885096.7868104 APPROACH_PLANNING→1791885096.7883098 APPROACH_UNREACHABLE，reason=no safe standoff candidate；无PICKUP_NAV请求，最后stop_confirmed=true、写盘/pending均false。
select_approach代码会先校验local costmap/TF、绕目标生成8候选并逐个检查整机footprint，再检查规划；当前日志丢弃ApproachResult.reason且没有失败时costmap快照/逐候选原因，不能确认是越界、障碍/未知、数据过期还是路径失败。1.5ms返回提示早期候选否决，不能当具体根因证明。
本轮无ARRIVAL_CONTRACT事件，行为与旧go2-goal-costmap-20261009的直接approach_target路径一致；新版arrival-contract会先记录该事件，故不能宣称新距离契约已生效。安装来源未由本轮模块路径直接确认，可能环境加载未保留，须下轮前核对。
startup图像TF警告只超前17.063ms，随后RGBD/nav/control门槛均通过，四次视角API均返回并应用；本轮直接失败发生在候选选择，非上传失败。仅离线读取已有事件/图片/代码和补充交接，无新ROS订阅、参数改动、API、ARM、运动或部署。

### 2026-10-13 初始位姿后静止匹配不准：运行参数及5秒只读观察

用户确认现象为设置后静止即有明显位置/角度偏差。实际GetParameters：AMCL scan_topic=/scan、base_link/odom/map、likelihood_field_prob、do_beamskip=false、update_min_d=.25、update_min_a=.2、transform_tolerance=1、500–2000 particles、use_sim_time=false；地图仍lab.yaml。
实际过滤节点强度开启≤3、height[-.2,.2]、voxel .05。launch的/scan来自过滤障碍云；源码scan范围±120°，5秒实测50帧/481束/240°，最后有效434束、年龄约.079s。
5秒临时订阅/scan、/amcl_pose、/tf，无新增RGB/点云检查：AMCL pose更新0次，map→odom50样本均不变；odom→base_link51样本最大相邻位移.00603m/转角.10585°。临时节点已销毁。
最新amcl_18823_1791885901735.log记录9次initialPoseReceived，各次随后Setting pose；初始位姿时odom TF未来外推约74–166ms，Humble官方AMCL代码该路径使用identity继续初始化，不代表设置被拒。
当前证据支持静止更新门槛会保留单次匹配偏差；扫描被裁剪、低反射过滤、家具重复几何及人工base_link中心/朝向误差为待核对因素，未建立扫描对地图残差或实际位姿真值，不能唯一归因或宣布标定故障。未调用nomotion更新/发布初始位姿/改参数/重启/ARM/运动。

### 2026-10-13 18:15实验扫描旋转碰撞中止（区别于接近不可达）

当前实验build/go2-field/explore-approach.20261013_181503：1791886517.117295 VIEWPOINT_NAV成功；view_index0/1/2均NOT_DETECTED、API结果APPLIED，有3套图片证据。
第三次SCAN_SPIN于1791886538.512494请求，behavior_server_18955_1791885917842.log在1791886543.013597998报Collision Ahead - Exiting Spin并Aborting handle；事件1791886543.0166886原生status=ABORTED/native_status6。
适配器以RuntimeError(scan spin failed)结束未完整扫描，入口仅记录异常类型RuntimeError；最终STOPPED、stop_confirmed=true、evidence_write_failed=false、vlm_evidence_pending=false。第4方向没有取图/调用API，未进入TARGET_FOUND或接近阶段。
这是Nav2旋转碰撞检查否决的已确认退出链，与18:09/17:50的APPROACH_UNREACHABLE不同。配置矩形footprint x±.38/y±.18，转动角点距中心约.42m，因此停车姿态可通过不意味着旋转各朝向均通过。
该实验未保存失败时local costmap/footprint/TF快照或bag，无法从事件/图片定位具体致命格，真实障碍、静态层定位错位、动态虚假/残留格及范围/未知问题仍待验证；不能直接归因AMCL或放宽碰撞阈值。仅分析已有证据并补交接，无新ROS订阅、参数修改、API或运动。

### 2026-10-13 初次匹配视觉偏差继续排查：二维扫描与三维显示差异

用户明确判断基准为实时障碍点与地图墙面不重合，估计几厘米、角度不能量化（小于40度）。一次6秒只读/scan和TF采样得到11帧有效scan-time map变换，AMCL pose更新0；/scan与/amcl_pose各唯一发布者，/tf发布者lio_mapping/amcl符合分工。
证据build/go2-field/evidence/initial-alignment.20261013_181918/：samples.json、alignment_report.json、scan_map_comparison.png、analyze_alignment.py和summary.json。当前是在18:15实验后的停止位姿，不是初次设置的历史快照。
按max_beams120规则抽样首帧104有效束、11帧扫描与静态地图最近障碍距离≤.10m比例约86.7–91.1%；附近±.6m/±20度离线端点似然搜索最佳相对调整约(+.05m,0,-1度)，不在搜索边界，仅小幅改善，不是实测真值或纠偏指令。现有证据不支持大幅雷达坐标整体错误；不把裁剪/强度过滤推测当根因。
实际map→base_link TF pitch约-6.9度（不是实尺姿态）；为区分RViz三维障碍云与AMCL二维scan，另做5秒临时订阅，仅采已有/vlm_nav/obstacle_cloud过滤输出3帧/1529–1544点，无原始完整云/RGB。完整scan-time TF与同点仅yaw投影XY差异中位约1.2cm、P90约3.0–3.3cm、最大5.3–5.7cm；filtered_projection.json及3个npz保存原始过滤XYZ和两种投影。
投影差异与地图/体素5cm离散化可解释几厘米视觉错位的一部分，不证明AMCL的实际精度，也不能还原初次收敛。静止更新门槛.25m/.2rad会保留已有偏差；需要捕获一次初始点击/协方差与随后扫描、粒子/pose才能判初次收敛，尚未操作。未发布initialpose、调用nomotion_update、改参数、重启、ARM、导航/旋转；临时订阅均退出。

### 2026-10-13 18:29实验：第一点未检出、第二点全局footprint受阻

本轮build/go2-field/explore-approach.20261013_182954：go2_view_01导航及3次Spin均SUCCEEDED，四视角API均返回/APPLIED、NOT_DETECTED；实际input图墙面/桌椅设备为主，未见清晰灭火器，submitted_images.png留存拼图。
1791887440.6684623 go2_view_02 FOOTPRINT_BLOCKED，reason=global costmap: footprint outside coverage or intersects obstacle/unknown；没有发第二点导航/接近。随后EXHAUSTED、stop_confirmed=true、写盘和pending均false，非18:15的Collision Ahead异常。
离线用原lab.pgm和候选(.9363348484,2.5810399055,.2293113795)、同整机矩形保守栅格检查，139格中致命/未知均0；静态地图本身未否决该候选。
结束后约147s（采样1791887587.2688975，grid stamp1791887585.662284）取得已有global raw costmap一次3秒只读快照，frame map、origin[-6.52,-5.34]、222×312/res约.05。候选footprint内1格254和26格253、无255，致命格中心map≈(.90500011,2.73500012)。
raw数组/元数据及静态对照图保存在本轮global_costmap_after_experiment.{npz,json}/goal_footprint_after_experiment.png；属于后续状态，不是失败瞬间快照。支持实时障碍层仍阻塞候选，不确认障碍实物身份、历史格精确一致、定位偏差或虚假点根因。未通过改footprint/膨胀/阈值绕过门槛。仅只读既有记录和短时costmap订阅，无新点云/RGB订阅、API、ARM或运动。


### 2026-10-13 generated_candidates_vlm：离线实现及隔离验证完成，未现场启动

用户要求复用刚成功的导航/旋转观察/目标接近，只新增自动候选与VLM选点。本轮读取根AGENTS、
最新交接和实际代码；分支仍go2-cooperative-task-layer，HEAD仍ce2999a4ace8409a4244e4aac02080e9ad93f479。
保留全部已有未提交修改、日志/bag和既有安装，无commit、reset、部署或运行节点切换。
默认fixed_viewpoints和semantic_furniture保留，coordinator、协同协议/READY/C逻辑未改。

新增公共generated_candidates.py：读取现有occupancy/semantic_map，家具每边外侧中点候选朝向家具，
距离配置1.0米，几何任务内缓存；身份/源摘要校验、有限简单多边形、边界/未知/占用/家具碰撞/
重复过滤。使用当前footprint完整旋转盘半径.4204759208米、padding0和原253代价否决，
全局costmap/TF新鲜度、原ComputePathToPose及资源路径检查；远点不要求进入local窗口。
每轮最多12可执行未观察点，VLM返回后重新规划/检查，实际到点停稳后再查旋转空间。
保持原局部路径监护、运动取消、原生终态和物理停止确认。

生成occupancy/semantic两PNG，共用位置/标签层、全图范围及缩放；Go2蓝点/朝向、橙色P编号、
灰色已观察点不可选，家具范围ID/名称及图例；标签避名称、优先家具空白/宽敞区域。
新图保存selection/<decision_id>，不写源地图。API复用ABCClient/AsyncVision原超时与max_retries0，
仅发送双图、目标/提示、允许P编号、图例、任务/决策及短记录，不发送候选坐标/内部ID/多边形。
程序保持展示→内部→位姿映射，严格JSON拒绝重复/额外/非法字段；版本、取消代数、摘要、
超时/迟到及当轮集合重检。临时导航/观察失败与完成分开，30秒冷却，无点退出等待，不轮询模型。
有效全方向ABC才完成观察；到达/API失败不算完成。已有有效A检出不因其他方向API失败而丢弃，
仍可进入原接近，但观察点保持未完成。成功接近复用vlm_nav.arrival，不发布READY。

用户本轮补充：B先填“水杯”、C先填“篮子”，A沿用“灭火器”；只更新scene.yaml描述，
没有填坐标或重新开放scene。以后若ABC描述为占位则新策略门槛拒绝执行。
当前--offline --require-ready新策略退出0、single_robot_experiment_gates=[]、motion_sent=false；
实际lab地图在1.0米配置下静态筛选得到3候选，此数不证明实时可达/可观测。
完整双机live_gates仍不通过，不把单机离线结果改写为双机验收。

构建：隔离build/go2-generated-candidates-20261013/{build,install,log}，vlm_nav和multi_agent_nav
两包成功；multi_agent_task_msgs复用build/go2-coop/install。首次未source go2_ws导致缺Sport/IMU，
已补正确依赖并强制重新配置，安装两可执行文件均已核对。剩余仅PCL开发提示和接口包复用提示。
CoVLM_env.bash默认不改，使用时须最后source新install并更新子脚本EXTRA，不能再加载旧overlay。
已从/tmp核对新增模块及vlm_nav.arrival均来自该安装，避免仓库源模块遮蔽验证。

验证：同一安装输出下6个相关测试文件，共180项通过（新增候选29项、既有相关151项），
报告build/go2-generated-candidates-20261013/related.xml；非colcon条目数，也未重跑无关全量基线。
覆盖双图标记一致/地图不变、两种绕向/静动态整盘/边界未知/重复禁用、全局远点、12点上限、
非法/重复字段/取消/迟到无运动、fake选点→观察→下一点、API失败不记完成、实际停止姿态重查、
有效检出进入原接近（含其他目标API失败）。系统Python/Pillow9离线绘图也已单独通过；
最初测试环境缺ROS/ament安装的问题已使用正确环境解决。
Python AST、文档Bash语法、相关本地链接和限定范围git diff --check通过。
只读审查发现的Pillow版本兼容、有效检出丢弃、实际停车旋转空间三项已修复并测试。

证据build/go2-field/evidence/generated-candidates-20261013/offline.json及preview-v2双图；
preview-metadata.json明确样图机器人位姿为演示输入、只经过静态筛选，未发API/运动，不当实测。
详细实现见[go2_semantic_search.md](go2_semantic_search.md#2026-10-13generated_candidates_vlm)，
构建/环境/核对过的未来启动顺序只在[runbook](go2_runbook.md#2026-10-13生成候选双图vlm选点)维护。
待验证：真实API选择质量、1.0米候选视野及实时可达性、完整ABC识别和新策略真机接近；
本轮无SSH/桥接/录包/新常驻诊断订阅/API调用/ARM/真机运动。下一次运动仍需明确授权。


### 2026-10-13 RViz新策略点、RGB与10次实验包（源码/安装完成，未现场启动）

用户要求原两行CoVLM_env+go2_exploration.rviz入口隐藏以前模式点、显示生成点与VLM选点、
RGB，并统一记录每次上传/返回最多10次实验；相机仅通过SSH在Go2上位机运行指定raw profile。
保留用户全部已有工作、地图、旧日志/bag，无提交、真机运动、API、SSH或ROS bag录制。

RViz移除旧current_point显示，新增独立generated_candidates/selected_candidate话题；原旧话题
仍为其他消费者保留。程序生成点保留map位姿，橙色本轮可选/P编号、灰色待检查/完成/暂不可用，
绿色选点箭头不跟随接近目标位姿漂移。每轮清旧选择、启动清旧标记；固定模式不会冒充新点。
RGB直接订阅已有/camera/camera/color/image_raw，raw/best_effort/volatile/depth1；此RViz订阅
为用户本轮明确要求，不添加额外常驻RGB/点云诊断或相机转发节点。

请求证据复用ScanEvidence：相机新增request.json记录实际messages含提示词/编码图像；
家具/候选选择也保存request.json、已脱敏原文response.txt、解析/失败metadata，与双图一起
放selection/<decision_id>，不污染上一相机目录。磁盘写失败抑制请求/执行，不记为未发现。
新策略不传output-dir时默认build/go2-vlm-experiments/experiment.<时间>.<ID>，事件、结果、
selection和vlm全部一包；experiment_records.py只清理有自身管理标记的直接子目录，保留
最多10个，删除最早非活跃包。文件锁保护活跃进程和迟到worker，10个全活跃拒绝第11包；
跳过符号链接与未标记历史目录，不触碰旧build/go2-field数据。
外部显式output-dir仍可兼容，故不受管理清理；要使用统一10次限制按最新默认入口不传外部目录。

scripts/go2_camera_ssh.sh已添加可执行权限，通过unitree@192.168.123.18，加载原远端setup.sh
及ros2_ws/install/setup.bash，exec原rs_launch.py depth_module.depth_profile/rgb_camera.color_profile
640x480x15、enable_sync true、align_depth false；TTY保留终端控制。脚本未执行，不重复启动已有
正常驱动，不改变Ranger/旧01_camera.sh。本轮仅用本地SSH替身检查命令/环境/参数及Bash语法。

隔离multi_agent_nav构建build/go2-exploration-display-20261013/{build,install,log}成功；
vlm_nav复用前一轮generated-candidates根包、消息复用go2-coop。CoVLM_env现在最后加载新协同
输出，EXTRA同步更新；仅source不启动节点，已有运行节点没有切换。
同一安装输出相关206项通过（本轮新增11、既有195），另环境加载3项通过，共209；
报告related.xml/environment.xml位于此新输出。覆盖request实际内容/原文/脱敏/失败、10次
清理/旧记录/符号链接/活跃worker/满容量、默认包入口、标记位置/清除/颜色及SSH参数。
只读审查发现失败点冷却后重新可选却显示灰色，已按本轮selectable修复且回归通过。
Python AST、Bash语法、YAML/文档链接及限定diff检查通过。
没有可用Xvfb虚拟显示，未运行真实RViz窗口或相机图像显示验收；没有SSH/真实API/运动。

当前入口、执行目录/终端/环境/成功判据按[runbook最新小节](go2_runbook.md#2026-10-13rviz生成点选点rgb与10次实验记录)
执行；前一轮显式最后source旧generated协同安装不再适用，会覆盖本轮新模块。
待现场验证：真实相机SSH启动（须先确认不重复）、RViz RGB持续刷新、真实新策略发布、每次
模型上传/返回与实际显示的对应关系；打开RViz本身不会生成点、调用模型或运动。


### 2026-10-13 本地启动指引同步（仅文档）

按用户要求更新 `/home/isee-pst/Notes/协同VLM启动指引.md`，依据当前CoVLM环境、相机SSH脚本、
Go2底层launch、Sport服务、generated_candidates_vlm及10次记录入口。原344行精简为217行，
按A相机、1底层、2RViz、3速度接口、4检查/探索列出依赖顺序、作用、输入、预期结果及保持运行要求。
移除旧安装路径/旧显示/固定点默认探索与接近语义矛盾，保留冷启动/复用分支、实测初始位姿、
FAULT→reset→DISARMED→ARMED分支及实际停稳后DISARM。
原文备份 `build/tmp/start-guide-20261013/original.md`；用户指定Notes文件已写入并与草稿核对一致。
18个Bash块语法、引用文件/本地链接、空白检查通过。没有构建或重跑测试，没有执行SSH、ROS启动、
预检、API、ARM/reset/运动/录包；此文档更新不产生现场授权或新的实机验收结论。


### 2026-10-13 20:12 终端4退出1：只读证据核对

用户要求检查当前实验失败原因。当前分支go2-cooperative-task-layer、HEAD已为
bdbb64c004b46c8de163710688834ecd4cbb4f0d（用户已提交前序实现），本轮保留交接未提交修改。
最新experiment.20261013_201239.45cd3ae3：20:12:44.990377 CANDIDATE_REQUEST，约8.923ms后
CANDIDATE_SELECTION_FAILED，20:12:45.000399 STOPPING，result为
motion owner/localization/Nav2/costmap health lost。motion_sent=false，没有发送导航/旋转，
stop_confirmed=null不认证其他控制器；不是最新实验碰撞或目标接近失败。

同决策24f182475871431d8f48816b8139cbd3的selection metadata已保存正常API返回、
validation_success=true、合法P2（本轮映射table_01/edge_4），worker耗时约4.217s。
响应晚于任务因健康门槛取消，未应用、未触发运动。result中vlm_evidence_pending=true是退出时
快照，现在response/metadata已写完，不把该字段当永久API卡住。最初相机TF警告请求比当时
buffer首样本早约19.8ms，随后启动门槛/RGBD/map/costmap已通过，不据此认定上传失败。

失败分支把owners、nav_ready、localization_valid和局部costmap/TF归为同一错误；nav_ready
又包含外部ARM/NAV门槛/新鲜度、action服务、视觉/深度单位和局部costmap。
现有日志未保存失败时各分项，result.health等为启动快照，不能用其中true断言失败时也正常。
具体失效条件仍待验证，不能直接归因AMCL/costmap/API或猜测性调宽时限/碰撞阈值。
源码与当前go2-exploration-display安装的go2_check/go2_exploration/adapters/go2逐字节相同。

另核对此前experiment.20261013_200426.8b85dea8：两个观察点8次相机ABC请求均api_returned=true，
但validation_success=false/error_type=ValueError，程序消费者为ERROR/API_ERROR，观察失败，
最后第三次选点同样健康门槛取消且停止确认true。保存的8个响应均为A/B/C直接grounding字段，
parser要求每目标status+grounding，离线重放全部invalid target structure；只加诊断包装后
严格grounding验证8/8通过，其中table_01/edge_1 view0返回A可见，其余A未检出。
诊断包装不修改/应用历史响应、不补投影或运动，不把原API_ERROR改写为有效观察。
实际请求提示同时要求status/grounding并嵌入3份单目标JSON指令，提示冲突为需修正事项；
它与最新实验选点阶段健康门槛失效是两个独立问题。

只读报告：build/go2-field/evidence/exploration-failure-audit.20261013_201239/report.json。
本轮仅读文件和离线校验，不修改产品代码/参数，不重启、ROS实时订阅、SSH、API、ARM或运动。
下一步应细分失败瞬间的健康分项证据，并统一ABC响应提示与解析协议，再按授权验证；
不凭通用退出1或健康总错误直接重跑/改底层。

### 2026-10-13 健康分项与ABC协议修复、一次只读检查

用户“继续”授权落实上述离线修复/相关构建和一次有时限只读inspect，不授权新运动实验。
分支/HEAD仍为go2-cooperative-task-layer/bdbb64c004b46c8de163710688834ecd4cbb4f0d，保留已有交接修改。
ABC请求统一为A/B/C各含status+grounding，并复用现有客户端schema及worker；单目标提示词
对6组目标/坐标模式与HEAD字节一致。旧扁平响应仍被拒绝，不回填历史观察、不放宽像素验证。
健康检查保存外部门槛/TTL、三个action服务、定位TF年龄、RGB-D、深度单位、局部costmap
接收/头时间/TF失效原因；探索失败事件HEALTH_FAILED记录实际失败瞬间的分项和owner。
运动门槛、超时、Nav2/定位/传感器参数未改。inspect复用已有临时observer，无新增常驻订阅。
输入稳定摘要仅含定位、局部costmap、RGB-D、深度单位、odom新鲜度，要求就绪后观察至少2秒
且多于1个样本、无后续输入失效；ARM/服务等仍在完整分项中，摘要不认证运动就绪。

隔离输出build/go2-health-abc-20261013/{build,install,log}两包vlm_nav/multi_agent_nav构建成功。
同安装/tmp相关协同225项、根客户端/环境35项，共260通过；其中新增诊断/入口回归19项，
既有相关241项。本轮改进环境断言，核对两个Python模块来自新安装。XML在新输出
related.xml/client-environment.xml；源码相关174项通过不叠加计数。只读审查两项诊断缺陷
（ARM误计、odom遗漏/晚就绪误报稳定）先复现失败，再修复回归，复核无阻断问题。
CoVLM_env最后加载新输出，保持原source入口；已有节点未切换。完整构建及检查命令见runbook。

一次Domain0/enp4s0只读inspect，timeout=10，79个样本，全窗口未收到map/定位TF/costmap/
RGB-D/odom，三个action服务不可见；退出1、motion_sent=false、non_motion_inputs_stable=false。
node list也为空。用户随后明确底层和相机“已停止，稍后按指引重启”，解释本次输入缺失；
不能将该结果当20:12瞬时失效的根因，历史分项缺失仍不可追溯。证据目录
build/go2-field/evidence/health-abc-readonly-20261013/：nodes.txt、inspect.json、exit-code.txt、
summary.json；inspect.json末尾含ros2run退出文本，纯JSON摘要为summary.json。
没有API、SSH、ARM/reset、运动、录包或参数调整。未执行真实API新格式/重启后的健康检查，
没有自动提交。下一步用户按本地指引重启原输入、只读检查通过后再按原ARM/探索流程；
再失效时取HEALTH_FAILED.failed_gates，而非猜测性改底层阈值。
本地Notes启动指引已同步新安装路径/稳定检查判据；runbook新增本轮完整构建/相关测试命令。
新runbook小节5个Bash块、Notes18个块、CoVLM语法、4个受改模块源码/安装字节一致性、
AST与git diff --check均通过。Notes与仓库内暂存稿health-abc.md核对一致。

### 2026-10-13 20:55 新实验启动即退出：已定位RGB-D同步门槛

用户询问为何刚开始又失败。本轮仅读新日志/代码、离线计算，不启动ROS/API/运动、不改参数。
最新experiment.20261013_205504.8f852ffd中20:55:09.201595 CANDIDATE_REQUEST，8.989ms后
HEALTH_FAILED，failed_gates=[rgbd_ready,nav_ready]；motion_sent=false。失败瞬间分项明确：
rgbd_sync_healthy=false，其余ARM/NAV/TTL、三个action服务、定位、局部costmap、深度单位均true，
owners=[]；RGB-D帧年龄81.9ms、相机TF81.3ms、位姿139ms、odom67ms，均新鲜。
这是运行期间同步健康判据失效，不是此前底层停止的无输入检查，也不是API选点格式失败。
selection在3.627s后返回合法P1，validation_success=true，但任务已取消，未触发运动。
程序按每个RGB/深度消息回调重算独立30帧窗口的一对一配对率（slop=.05、min samples=10、
min rate=.9），并在任何一次输入false时立即结束探索。离线人工时间戳例可复现：
27/30=true→单RGB回调26/30=false→深度回调27/30=true，说明窗口/到达顺序可能瞬时翻转；
人工例不是实测根因。当前日志未保存真实matched/total/rate或两流时间戳窗口，不能确定
本次具体是丢帧/延迟/窗口边界；不要猜测性改相机/Nav2参数或直接忽略同步失效。
证据：build/go2-field/evidence/exploration-failure-audit.20261013_205504/report.json。
下一步可在既有observer诊断中保存完整sync状态/窗口时间边界，再区分持续异常与回调瞬态；
后续修复应保持运动前安全门槛，不重放本次迟到结果。原始实验资料保持不变。
