# Go2 控制电脑会话交接（2026-10-08）

> 最新修改及安装见文末2026-11-05全流程等待恢复交付；任务模式仍为独立C。此前资源许可与顺序退出记录属于历史版本。


最新实施范围见本文末尾 **2026-10-20 空地候选及家具可选**；本轮私有C要求覆盖前文共享位置/共同C/跨地图对齐的任务要求，历史证据不改写。

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

当时Go2活动地图（2026-10-18已切换源码默认，见末节）：`/home/isee-pst/unitree_ros2/VLM_Nav/maps/lab_rear120_20261001_IwQjMX/lab.yaml`，引用`lab.pgm`。
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

### 2026-10-13 RGB-D成功/失败版本对比与配对边界修复

用户要求仅对比配对、取消、初始化及执行方式，并复用旧流程离线修复，禁止新真机实验。
对比最近完整成功18:37 fixed_viewpoints/TARGET_APPROACHED与20:55 generated失败；
详情见[rgbd_pair_health_20261013.md](rgbd_pair_health_20261013.md)。历史启动指引指向旧
goal-costmap协同安装，成功实验无逐进程模块路径清单，不能补造确定加载来源；但四份根包
安装的配对/缓存/TF等待/就绪函数AST一致，相关配置一致，旧两个协同安装也都持续检查
输入健康并在false时取消。选点未清缓存，API原本异步，单线程主循环可在两路回调之间读健康。

本轮仅将统计更新从单路record_rgbd_stamp移入既有on_rgbd完整配对回调。复用旧matcher、
统计函数、缓存及期限；不改取消条件、不放宽阈值、不新增同步模块或订阅、不改底层参数。
原离线人工27/30→26/30→27/30序列用真实matcher重放，RGB/深度各先到时旧实现2例失败，
修复后7个新增回归全通过；持续低匹配率、启动不足样本、停流/单流和图像/TF超时仍拒绝。
没有原始现场到达序列，人工回放证明该机制缺陷，不证明20:55的具体丢帧/延迟来源。

仅vlm_nav构建build/go2-rgbd-pair-health-20261013/{build,install,log}成功。
相关安装检查36项通过（新增7、既有29）：rgbd-runtime.xml 13、health-cancel.xml 19、
environment.xml 3、image-tf.xml 1；均为离线，不初始化ROS、不连接设备/API。
根包新模块与源码字节一致，协同仍来自health-abc；CoVLM_env最后加载GO2_RGBD_OUTPUT，
VLM_NAV_EXTRA_SETUP_FILES同步更新，旧安装和既有运行进程保持原样。只读复核无阻断问题。
对比摘要/函数摘录在build/go2-field/evidence/rgbd-pair-comparison-20261013/。
工作期间用户HEAD更新为eb32b45（go侧），已保留该提交；本轮未自动提交或现场复验。

### 2026-10-15 10:35 无探索点与Image-time TF past警告：只读核对

最新experiment.20261015_103513.01dceb07仅WAITING_INPUTS→STOPPING/operator interrupt，
等待75.245秒后人为中断，motion_sent=false；无CANDIDATE_GEOMETRY/REQUEST/SELECTED、
无selection/vlm请求资料。因此RViz候选/选择项尚无生成数据，并非已证明显示配置故障。
本地配置两显示项均启用；运行阶段必须先通过RGB-D/图像TF输入门槛才进入候选生成。

python_13826_1792031713829.log仅两条图像TF警告：图像时刻14.064613/14.198987，
TF缓存最早15.053060（共同秒前缀17920317），图像早0.988447/0.854073秒。
这是新observer本地缓存缺少对应历史变换的已确认事实；不能用最新TF替代该图像时刻。
原队列会先等待旧帧，直到收到时长超过5s后丢弃，past类型当前不会立即丢弃，因此启动可能
多等几秒。正常持续新帧/TF到来应能恢复；两条启动警告不足以解释整段75秒未就绪。
本次中断发生在启动收集后、最终health快照之前，结果缺少持续等待时的sync计数/帧龄/TF龄；
不能断言相机断流、同步差或TF持续故障，也不能将本次当20:55健康取消同一根因。
离线报告：build/go2-field/evidence/exploration-startup-audit.20261015_103513/report.json。
下一步需在原输入保持运行时做有时限只读inspect并核对健康分项，不直接重跑探索。
本轮仅日志/源码/配置检查和文档更新，无ROS/API/SSH/运动、构建或测试。
另按既有comparison.json实值纠正前轮对比文档队列容量笔误：实际16，测试fixture的4非部署值；
未修改产品配置。保留当前所有未提交修改，git diff --check通过。

### 2026-10-15 10:40 用户inspect与相机UVC watchdog证据

用户提供10秒inspect：93次采样RGBD全部false，定位约1s恢复、局部costmap约1.2s恢复；
最终地图匹配/定位/静止/costmap有效，但完整帧年龄7.016s、相机TF最后成功年龄3.750s、
sync_healthy=false。sync在早期一度true，不能解读为始终无配对；camera_tf_age是最后成功
查询年龄，不是TF发布本身年龄。local_observation_ready仅旧定位/停车/costmap门槛，true
不代表相机就绪，仍须检查rgbd_ready和non_motion_inputs_stable。三候选是离线几何结果，
inspect不会发布探索marker，也不代表全局规划与旋转筛选通过。

相应本地python_14398_1792032028969.log仅两条startup TF警告。用户随后提供相机上位机
10:40:33 uvc-streamer.cpp:106 watchdog triggered endpoint130，与本次inspect同一时段。
官方librealsense src/uvc/uvc-streamer.cpp：有效传输回调kick watchdog，超时报告并reset端点，
表明相机UVC/USB采集链路出现数据停顿；不能仅凭端点号确定RGB/深度或断言USB线/供电故障。
来源 https://github.com/realsenseai/librealsense/blob/master/src/uvc/uvc-streamer.cpp 。
该证据与完整图像不刷新一致，应先检查相机主机USB/驱动输入，不能通过放宽健康期限解决。

另核对运动许可：twist_to_go2_sport_bridge_13595_1792031663109.log在10:37:12.937
记录foreign Sport request observed while ARMED并进入FAULT，随后SYSTEM_READY=false，
NAV supervisor明确failed=['system_ready_value']。因此inspect的NAV_READY=false不是
相机门槛派生，也不是单纯未ARM；外部请求来源仅有GID，不能断言来自遥控器或某进程。
故障发生在10:35实验被人工中断之后，不能倒推为该次WAITING_INPUTS的原因。后续运动前
需先排除外部控制源，再按原reset/DISARMED/ARM流程，禁止自动重置或反复ARM。
本轮只读现有日志/代码及官方源码，没有新增订阅、连接SSH、重启相机、改参数、ARM或运动。

### 2026-10-15 用户补充USB拓扑与内核日志

Go2上位机lsusb -t显示D435所在Bus02/Port2/Dev2各Video接口为5000M、Driver=usbfs，
说明检查时协商USB3，不能据此证明持续传输稳定或将usbfs本身判为驱动错误。
用户dmesg末100行在10:35:17、10:40:32记录PID3270 realsense2_came对usb 2-2
端点0x82调用USBDEVFS_CLEAR_HALT；0x82等于watchdog端点130，后一条与10:40:33
watchdog同一时段，符合端点恢复行为，但不是触发停流原因的证据。官方streamer源码
start/stop也调用reset_endpoint，不能把每条CLEAR_HALT都认定为一次watchdog故障。
所给片段未见异常时段USB disconnect/重新枚举；10:26:19–20大量xudc usbphy报错
属于更早记录，xUDC为设备角色控制器、相机路径为xHCI，现有证据不足以关联本次停流。
EXT4错误统计也不能直接归因为相机故障，应保留为独立存储问题，不在线执行fsck。
下一步先在相机主机只读核对/dev/bus/usb/002/002的占用者和相机进程，排除重复采集。
若只有原驱动，再由用户停止并按原参数重启相机、观察并复查RGBD输入；尚未执行这些步骤。
未改运行代码、参数或健康门槛，未SSH、重启驱动或启动运动；仅补充交接并检查文档差异。

### 2026-10-15 10:55 实验在第二观察点退出：扫描健康取消，分项未保存

experiment.20261015_105514.c907f4e6已完成P3/table_02/edge_4导航与四视角ABC观察，
10:56:14记录observed/NOT_DETECTED，随后选择P2/table_01/edge_4，10:56:29到达。
第二点view0、view1正常返回；view2前旋转在10:56:58.190记录SUCCEEDED，
10:57:00.391 STOP_COMMAND_SENT reason='scan health lost'，该视角推理被取消，
最终observation cancelled、stop_confirmed=true。两段导航与五次Spin均SUCCEEDED。
view2 API取消后约5.276秒返回合法A=DETECTED（仅模型识别，非确认目标位姿/接近成功），
未被应用；不存在由此启动接近的证据。本次不是API超时记录或Nav2动作失败记录。

实际触发分支为adapters/go2.py tick内nav_ready/localization_valid检查，取消前未保存
last_health_details。外层go2_exploration因scan future已cancelled直接退出，未产生
HEALTH_FAILED分项快照。result.json中的health/health_details在开始探索前填入，
不是退出时快照，不能据其rgbd_ready=true排除运行期间相机/同步瞬态。
本次控制桥仅启动DISARMED及ARM accepted，未见此前foreign Sport/FAULT；相关
SYSTEM_READY/NAV_READY日志未见失败时段下降，仍不足以排除本进程接收TTL超时。
现有证据可确定扫描健康保护取消，无法确定具体为RGBD、TF/定位、costmap、外部门槛
接收时效或action服务。相机无终端报错也不等于这些消费端门槛持续有效。
下一步最小改动应在既有扫描取消分支保存触发检查同一次健康快照及scan phase/index，
不要先改同步模块、放宽门槛或启动新真机实验。本轮仅读日志/代码并补充离线审计和交接，
未实现上述日志改动。证据build/go2-field/evidence/exploration-stop-audit.20261015_105514/report.json。

### 2026-10-15 按用户要求改为45度旋转8次

仅修改源config/robot_go2.yaml的scan_angles为π/4至2π共8项，每次相对上一方向约+45°，
第一帧在首次45°旋转并停稳后获取，第八次回到初始朝向；不是0°起拍再旋转7次。
沿用现有角度归一化、Spin/停稳/新帧/ABC异步识别及取消逻辑，单点scan_timeout显式240s
以容纳8次识别；不改API/健康/底层参数。整轮命令timeout300仍可能先于多点扫描预算到期。
CoVLM_env的GO2_EXPLORE_CONFIG直接指向源配置，下次原入口启动生效，无需重建或覆盖旧安装。
离线验证：现有test_go2_scan_evidence.py 15项通过；调用当前安装adapter的start_scan/tick，
stub运动输出，在初始yaw=0、3.1、-3.1三种情况下均发出8个π/4 Spin目标、总计2π并正常结束。
该检查隔离图像/API阶段，验证角度配置与跨±π执行语义，不替代实际8视角现场验收。
未启动ROS节点、API或真机运动，未提交代码；扫描健康分项日志缺口仍未修复。

### 2026-10-15 11:07 启动即拒绝：外部Sport请求使桥接进入FAULT

最新experiment.20261015_110726.44013148于11:07:31.288 REJECTED，motion_sent=false。
启动快照RGBD/sync、图像TF、定位、local costmap及三个action服务均通过；只有
nav_ready_gate/control_armed_gate=false，且两门槛接收新鲜。尚未进入候选或45度扫描。
此前11:07:10.840桥接PID16384在ARMED时收到非自身Sport请求，api_id1008，
GID eb:2c:43:73:10:0a:7e:ec开头，立即StopMove并记录
bridge entered FAULT: foreign Sport request observed while ARMED；随后SYSTEM_READY下降，
11:07:11.229 NAV_READY下降，nav supervisor failed=['system_ready_value']。
外部请求持续到11:07:21.721；截至本次启动无新的reset/ARM记录。故此次启动失败已明确
是运动许可链关闭，不是相机或45度配置错误，且不同于10:57扫描健康分项未知的退出。
具体外部发布者尚未映射到进程/设备，不能直接归因遥控器。下一步先核对该时段其他
控制操作及/api/sport/request发布者，排除后按原指引reset→DISARMED→ARM，不自动操作。
本轮只读日志/源码并更新交接，无ROS/SSH/重启/运动及代码修改。

### 2026-10-15 外部Sport手动接管桥接实现（未现场启用）

用户明确要求两次实验间可用遥控调整位置，外部请求优先。源go2_bridge.yaml将既有
fault_on_foreign_sport_request设false；该分支由原来的忽略冲突改为yield_to_external_control：
ARMED→DISARMED，清空缓存速度和停止边沿，不发Move/StopMove覆盖手动控制；
CONTROL_ARMED=false使活动任务按原健康检查取消，不自动续跑/重放。外部请求自身已到Sport
话题，无需再次转发。自身GID过滤保持；其他FAULT不因外部请求被清除。true仍保留原FAULT策略。
外部流量静默至少2秒、系统门槛有效后才能显式ARM；正常手动接管不需reset_fault。

改动限状态机、桥接、Go2配置、相关测试、公共环境及文档。先运行新增dry-run接管用例，
旧二进制在期望DISARMED处失败；修改后原生19项、Python8项均通过（新增3，既有24）。
Python包括Domain31/localhost/FastDDS隔离dry-run新旧策略各1、配置3、环境3；覆盖不发停止
干扰手动、残余自动指令不输出、静默期拒绝ARM、重新ARM不重放及原故障策略。未发真实Sport。
构建仅根包vlm_nav，输出build/go2-manual-takeover-20261015/{build,install,log}，完成；
仅有既有PCL/CMake与overlay提示。测试XML为bridge-state.xml、bridge-runtime.xml。
安装的vlm_navigator.py及go2_bridge.yaml与源码一致，保留此前RGB-D修复。
CoVLM_env最后source GO2_BRIDGE_OUTPUT，EXTRA_SETUP同步；协同仍health-abc。
原安装保留、运行中的旧进程未重启。完整命令在runbook末尾“手动接管桥接”。
本地Notes/协同VLM启动指引.md已更新：首次须停稳后结束旧桥接再重开唯一实例；修正协同包
prefix为实际health-abc，正常遥控后按DISARMED直接ARM。无SSH、ARM、真机运动或提交。
既有10:57扫描健康分项日志缺口仍未修复，本改动不宣称解决该问题。

### 2026-10-15 重复实验灰圈：用户重启对照、恢复后基线及启动接入遗漏

用户反馈只重开RViz灰圈仍在、重启底层后消失。11:16截图不含原始格/TF数据；不能用
重启后状态补造当时现场。经Domain0本机只读发现后，复用旧capture脚本短时采集3秒30帧
过滤障碍点云，30帧均获取严格点云时刻map TF，并保存map/global/local/odom/footprint。
订阅已退出；参数服务只读查询global/local和filter。证据
build/go2-field/evidence/ghost-obstacle.20261015_112232/含CDR、XYZ、TF、参数、分析脚本/图。
恢复后global lethal3449格，其中静态free上467格；234格距静态障碍>10cm，均有当前
点云并集10cm内支持。这是恢复后基线，不证明截图灰圈的具体点来源，也不证明实物身份。
实际inflation_radius两侧均0.55m，marking/clearing同用过滤云、observation_persistence=0；
后者不等于格自动TTL。Nav2 obstacle layer依赖raytrace清除，未经过清除射线的历史格可保留。

代码确认：原单机VLMNavigator enabled启动调用require_initial_costmap_clear，其navigation_tick
调用ensure_initial_costmap_clear_done等待服务完成及新costmap；协同make_observer启用
observation_only跳过该timer，当前go2_check/adapter未调用这段流程；Sport ARM也无清图。
所以当前协同多轮实验沿用底层costmap状态；该遗漏同时存在于协同固定点和生成点入口，
不能描述为仅本轮45度或桥接改动引入。它解释旧格跨实验保留的机制，不能单独证明初始
标记由噪点还是定位投影产生。旧实验包原始格/点云/TF记录缺失是本次无法回溯的实际限制。
本轮未改运行代码、Nav2/定位/传感器参数，未清图、ARM、运动或录长期bag。
后续应复用原任务边界清理/刷新流程（停稳且运动所有权成立后），并先保留清理前格与失败
健康快照；不能直接反复清图掩盖输入误点或定位问题。尚未实现/现场验证该接入修复。

### 2026-10-15 顺时针扫描、RGB双订阅复现与预览规避

用户要求45°×8改顺时针，排查勾选RViz RGB后的失败，并确认现场终端保持运行。
当前分支go2-cooperative-task-layer、HEAD eb32b45；保留此前未提交修改，未提交代码。
最新experiment.20261015_112422.0a4e4b4e导航及6次Spin成功，11:25:36.066679
scan health lost取消；历史具体分项缺失。现已修复取消分支漏记：一次get_health快照，
先停再写SCAN_HEALTH_FAILED及RGB-D窗口/匹配率/帧年龄/待TF数、其他健康门槛、phase/index。
不放宽门槛、不修改同步器、不新增常驻诊断订阅。

本机Domain0无运动对照复现：原观察器单独健康；临时第二原始RGB订阅加入后配对22/30，
继而帧龄2.101s，深度时间戳断档10.152s；移除后恢复。同期定位/costmap正常。
改订阅已有1 Hz camera_preview，同样38s对照就绪后313样本/33.642s无输入健康失败，
预览12秒收到12帧；相机原始RGB仅观察器1个订阅。支持额外原始RGB订阅触发异常，
但未采远端负载，不能细分驱动/DDS/主机瓶颈，也不能补造历史退出瞬间的分项。
完整证据、局限及文件索引见[专项记录](rviz_rgb_scan_audit_20261015.md)。

源Go2 scan_angles改为-π/4至-2π；RViz Go2 live RGB (1 Hz preview)改用
/vlm_nav/camera_preview。此话题已有，不加新节点；仅inspect/探索进程运行期间刷新。
预览停更可能保留旧帧。现有RViz需重开读取配置；相机/终端1无需因本改动重启。
仅构建multi_agent_nav到build/go2-scan-health-20261015/{build,install,log}，成功；
CoVLM_env GO2_EXPLORE_OUTPUT及ROS_LOG_DIR改为该输出，根包仍manual-takeover。
相关84测试通过（新增3，既有81）；实际安装adapter与源码一致，离线3种初始朝向
各验证8次-π/4；offline配置通过。Notes启动指引与runbook同步，无真机运动/API/SSH。
未执行真实顺时针运动或RViz GUI重开验收，灰圈任务边界清理仍为前述未完成项。


### 2026-10-15 检出即结束扫描：源码及独立安装完成，默认入口未切换

用户要求Go2等待当前模型回复后才进行下一动作，未检出继续旋转、检出则停止扫描并接近。
本轮改动限本地探索执行链：start_scan增加可选stop_on_target，explore指定主目标A；
INFER逐帧串行，A=DETECTED立即返回已完成视角（无有效位置也停止，交给上层报告）；
有有效位置并启用原--approach-target时复用停稳/健康/规划/到达检查。超时或错误停止本轮，
不再继续旋转或切换点位；取消后迟到结果不会启用导航。提前返回允许不完整方向集合，
SCAN_RESULT保存scan_complete，不把未扫方向/整点记为observed。双机及独立scan入口
默认stop_on_target=None，完整ABC扫描不变。Fake同步接口、只返回请求的目标。
未改扫描角度、Nav2、传感器、健康阈值或此前健康失败日志/RViz预览等用户修改。

测试先行：新测试对旧代码观察到15个失败（提前检出不返回、错误后未终止、探索未传目标、
正常提前完成误标观察失败），再实现。最终新增17项，覆盖等待/未检出/其他目标、首帧
检出/无深度、超时/非法回复/缺失结果、取消后迟到检出、完整ABC、1/8方向接近及安全拒绝。
源码相关143通过（新增17、既有126）；独立安装全套297项，296通过、1既有断言失败
（新增17全通过；既有280项279通过）。失败名称
`test_no_verification_flags_are_set_by_static_costmap_profile`，原因是旧测试仍断言
costmap_source_verified=false，而当前配置为true；本轮没有修改该标志/测试。
首次扩大测试仅source旧go2-coop时还遇到2个ROS日志目录只读和2个旧costmap二进制退出-11，
改用当前CoVLM完整环境、可写ROS_LOG_DIR后复核这4项通过；不将测试环境失败称为本功能回归。

仅multi_agent_nav构建到build/go2-stop-on-target-20261015/{build,install,log}，成功；
根包/消息复用既有安装，两处源码基路径均传给colcon。实际安装adapter及exploration与
源码逐字节一致；在/tmp测试安装代码，Domain31/localhost/FastDDS隔离，没有现场输入。
证据scan-source.xml、installed-suite.xml、costmap-baseline.xml在该输出根目录。
只读代码审查未发现阻断项。没有API、SSH、真实运动、默认环境切换或自动提交。
CoVLM_env仍指向go2-scan-health-20261015，现场进程未更新；独立构建不等于默认入口已生效。
完整构建/检查及终端4手动加载顺序见[runbook新增小节](go2_runbook.md#2026-10-15等待识别结果检出主目标即结束扫描)。
待验证：实际真机首帧检出后停止继续旋转并接近；健康丢失仍应立即取消，不被等待模型覆盖。

### 2026-10-15 11:53检出后仍旋转：默认入口遗漏，已统一安装

用户反馈另会话完成提前停止扫描后仍出现检出0.95、observation_failed和继续旋转，要求
对比原成功接近链并修复；明确要求原地更新Notes/协同VLM启动指引.md。
当前分支/HEAD不变，保留另会话源码、测试、独立构建及所有旧实验资料。

最新experiment.20261015_115348.99b2a74c：事件106在11:54:12.730277接受A=DETECTED、
confidence=.95、position_valid=true、reason=ok，位置(0.852678,2.888399)。事件107约10.7ms后
再次SCAN_SPIN；事件109 ABORTED。Nav2 behavior_server_29747_1792036287901.log在
11:54:14.142明确Collision Ahead - Exiting Spin。事件112把P3记observation_failed，
随后选择/导航P2继续扫描，最后operator interrupt停止，无PICKUP_NAV。
此次有效检出和定位已完成，后续多余旋转失败使旧扫描future异常，旧上层catch后continue
丢弃该次已识别目标并切点。未采当时原始障碍格，不能判断碰撞检测对应实障碍还是残留格。

核对默认CoVLM_env仍加载go2-scan-health，源/独立go2-stop-on-target已有正确提前返回和
失败终止分支但未接入默认source。现场事件与旧分支一致；旧记录无逐进程模块路径清单。
10/13 18:37成功fixed_viewpoints实验完成四方向后走TARGET_FOUND→APPROACH_SELECTED→
PICKUP_NAV→TARGET_APPROACHED；本次旧流程在扫描返回前被Spin失败打断。
原单机handle_worker_result接受有效定位后也立即进入confirm_target、离开扫描等待。
当前新源码将上述检出提前终止语义接入本地探索，接近函数无需回退或重写。
AST核对新旧approach_target/approach_attempt/select_approach完全相同；四个运行模块
（go2/base/fake/exploration）源码与stop-on-target安装逐字节一致。

本轮仅修改默认环境、环境回归断言、runbook及原地Notes指引，统一GO2_EXPLORE_OUTPUT为
build/go2-stop-on-target-20261015，EXTRA_SETUP/ROS_LOG_DIR随变量一致；vlm_nav仍manual-takeover。
复用已有构建，未修改运行模块或重建。测试118项通过（原有相关115+更新环境断言3），
环境断言先复现旧入口失败；在/tmp验证实际安装的检出立即返回/无位置/超时/取消/原接近流程。
offline配置通过。XML default-entry.xml/default-entry-scan.xml及审计default-entry-audit/
在stop-on-target输出内。未重跑全量297项、启动ROS订阅、API、SSH、ARM或真机运动，未提交。
Notes第6步要求旧实验退出停稳后重新source并核对新prefix；第9步明确最多8方向、等待
当前回复、检出即接近、异常停止不换点。相机及终端1/2/3可保持运行；真实新入口接近待现场验收。

### 2026-10-15 12:05实验：提前检出接近已执行，末端距离未达标且补偿无候选

用户实验experiment.20261015_120530.7bc81355：两次SCAN_SPIN均成功、两个视角API均有效返回
并APPLIED。view0于12:06:10.775为NOT_DETECTED；view1于12:06:21.720检出A=.95、
position_valid=true，目标地图位置(0.859705,2.906451)。随后立即结束扫描，进入TARGET_FOUND
及原PICKUP_NAV；没有第三次扫描请求。本次现场证实提前结束扫描及接近衔接已生效，
但不是完整TARGET_APPROACHED验收通过。

初次接近点距目标0.81m，goal=(0.286948,2.333694)，12:06:25.463 Nav2返回SUCCEEDED，
controller日志也报告Reached the goal。停稳base=(0.081323,2.234871)，距导航goal约0.22814m，
距语义目标1.028055m，大于target_success_radius=.81；任务层因此RETRY。原补偿将下一次
standoff降至.591945m，12:06:25.470返回no safe standoff candidate，APPROACH_UNREACHABLE→
APPROACH_FAILED。没有发出第二次PICKUP_NAV。final stop_confirmed=true，证据无写入失败/待返回。
--approach-target只将TARGET_APPROACHED作为退出0，故本次返回1。无HEALTH_FAILED或扫描健康
失败事件；这是导航goal成功与任务目标距离验收的差异，不是本轮识别/扫描衔接失败。
当时原始costmap及每个补偿候选拒绝理由未保存；仅凭通用结果不能区分格占用/未知/越界、
costmap/TF瞬态或路径不可达，不能断言真实障碍或灰圈。未因此放宽成功半径或安全检查。

vlm/恰好2个目录，各含input.jpg/request.json/response.txt/result.png/metadata.json，
api_returned及validation_success均true，耗时约7.706/7.244s。第一次停稳识别未检出才发
第二次旋转；第二次检出后不再扫描。导航过程中的转向不逐方向发识别请求；另外1次双图选点
请求在selection/。只读离线核对事件数、结果顺序及文件完整性通过，证据保存于
build/go2-field/evidence/approach-audit.20261015_120530/report.json。
本轮只查日志/代码并更新交接，无运行代码/参数修改、ROS/API/SSH/真机运动或全量测试。

### 2026-10-15 用户授权减小协同膨胀：配置0.45米，未热改运行节点

此前实际参数只读查询确认全局/局部radius=.55、scaling=5.0。用户随后明确要求减小
膨胀并允许灰色区停靠/接近，本次授权覆盖之前“不改Nav2参数”中的这一限定范围。
仅最终协同nav2_go2_cooperative_costmap.yaml两侧增加inflation_radius=.45；衰减仍5.0，
footprint/253阻挡阈值、到达0.81m验收及其他profile不变。0.45为初始小幅调整，非现场验证值。

核对当前adapter已允许原始代价值1～252用于局部/全局停车、旋转空间及接近候选规划；
253/254/255、未知/越界/失效输入仍拒绝，不能将RViz灰色一概放行。本次无需修改检查代码。
减小外层膨胀可降低外围路径代价，但不能自动消除上次导航停远的0.228m偏差，也未证明
上次.592m补偿候选全部被拒绝是膨胀半径造成。

相关16项通过（新增6个灰区/危险值边界用例，既有10项配置加载及footprint/costmap用例）。
两配置加载断言先在旧值.55上失败，再修改配置通过；验证实际launch最终overlay优先级，
legacy仍.55、其他非costmap配置不变。新增测试在现有安装adapter验证1/128/252允许、
253/254/255拒绝；测试夹具曾误用不存在的on_global_costmap，改回既有on_costmap(...,
global_goal=True)后验证通过，无运行代码变更。XML inflation-config-and-parking.xml及
inflation-launch.xml在go2-stop-on-target输出。配置由启动指引显式源路径读取，无需重建。
Notes第3步及runbook已更新停止/DISARM、终端1按原预检重启及两侧0.45参数核对步骤。
本轮未热改节点、启动ROS诊断订阅、SSH、重启底层、ARM或运动；现有进程不会自动加载该配置。

### 2026-10-15 12:17最新失败：首次Spin碰撞中止，运行膨胀仍0.55

最新experiment.20261015_121734.e9d511ce：P3导航12:17:48.178 SUCCEEDED，停稳后首次
顺时针-0.79rad Spin于12:17:48.190开始，12:17:52.193 ABORTED（native_status=6）。
behavior_server_32429_1792037069821.log在12:17:52.191明确
Collision Ahead - Exiting Spin，随后spin failed/Aborting handle。探索因此observation_failed→
STOPPING scan failed→STOPPED，stop_confirmed=true，没有继续换点；符合新错误停止语义。
仅1次SCAN_SPIN、0次VLM_VIEW_RESULT，扫描未完成停稳抓帧前就中止，无识别API漏发或超时证据。
两次停稳快照净顺时针约40.0°，地图位移约0.072m；不能由此判断实际碰撞格来源。

本轮实时只读参数查询：/global_costmap/global_costmap及/local_costmap/local_costmap的
inflation_layer.inflation_radius仍均.55。源最终overlay为.45；新参数没有加载到当前底层，
本次不是.45的现场验证。终端1/behavior_server日志仍为原12:04进程。不得仅凭改文件称已生效。
旋转前检查用全局costmap，Nav2 Spin运行中检查局部预测碰撞；当时原始局部格未保存，
无法确定实障碍/残留格/输入投影的具体贡献，不直接归因.55或减小半径就能通过。

前一次experiment.20261015_121710.9c955c21为启动拒绝：仅CONTROL_ARMED=false，
其余启动checks=true、motion_sent=false；12:17:30.752 bridge ARM accepted后才开始最新实验。
这与最新旋转碰撞失败不同。审计保存build/go2-field/evidence/scan-collision-audit.20261015_121734/report.json。
本轮只读日志/参数并离线核对次数/终态、更新交接；无代码/参数修改、运动或重启。
下一步按Notes第3步在结束实验、停稳、DISARM后重启加载新配置，查询两侧.45后再继续原检查。

### 2026-10-15 12:23实验：Nav2两次成功后，任务距离验收失败

用户要求区分终端4提前报错还是Nav2到点后目标距离不达标。最新
experiment.20261015_122308.bc281a2b为APPROACH_DISTANCE_FAILED、stop_confirmed=true。
12:23:36.300首个PICKUP_NAV SUCCEEDED，随后STOP/停稳，12:23:36.302测目标距离
.926834m>.81m并RETRY；补偿standoff=.693166m，12:23:36.318发第二次PICKUP_NAV，
12:23:36.381已SUCCEEDED，之后STOP/停稳，12:23:36.384测距.896642m>.81m，
1次重试用尽而FAILED，12:23:36.388结束。无HEALTH_FAILED或APPROACH_DISTANCE_STOP事件。
对应controller/bt日志两次Reached the goal/Goal succeeded，第二次controller接目标约0.75ms
即Reached the goal。实时只读查询general_goal_checker.xy_goal_tolerance=.25m；两次停稳
位置距各自导航goal约.153362m/.220622m，均在该容差内，但距识别目标仍超任务成功半径。
因此是Nav2按导航goal容差结束后任务距离失败，不是终端4在Nav2结束前抢先报错；
“Nav2成功”不表示精确抵达停靠点坐标。此轮不是上轮无安全补偿候选，两次接近都实际提交。
事件顺序离线核对通过，审计build/go2-field/evidence/arrival-order-audit.20261015_122308/report.json。
本轮只读日志/单项参数、更新交接，无代码/参数修改、运动或重启。

### 2026-10-15 接近任务到达预算对齐（离线完成，未启动真机）

用户要求核对旧成功/当前到达参数、按实际路径终点+控制容差+余量≤0.81m筛选、
统一map/base_link平面距离记录、拒绝无改善重试。保留扫描、碰撞检查、停稳与最终距离验收。

证据：旧 `explore-approach.20261013_183747`请求/执行终点距物体0.90m，
旧goal-costmap安装在Nav2 SUCCEEDED时直接记TARGET_APPROACHED，没有ARRIVAL_VERIFIED；
这不能证明0.81m已满足。旧配置xy=.25、Navfn继承上游tolerance=.50，但旧运行参数快照缺失。
12:23实际生成的 `/tmp/vlm_nav2_go2_ml91o9xu.yaml`已复制到
`build/go2-arrival-budget-20261015/evidence/nav2-launch-122204.yaml`，同样为.25/.50。
上轮实时读到xy=.25；本轮enp4s0无192.168.123.x地址，未得到新在线参数。
12:23两次PATH_RECEIVED实际终点距物体.81/.6931655543；加.25+.03分别1.09/.9731655543，
均不能保证最终.81。对比与计算存 `build/go2-arrival-budget-20261015/evidence/comparison.json`。

保留Nav2 xy=.25、planner=.50及请求停靠上限.90，Go2配置新增approach_distance_margin=.03。
接近前使用短期参数客户端/定时器等待发现、异步读取实际容差和map/base_link，最多5s；
完成/失败/取消销毁客户端和定时器，不新增图像订阅，不写参数。
当前候选生成半径最多.81-.25-.03=.53，选择器用原8方向，检查实际返回路径终点；
预算超标/落入当前Nav2容差/无保证改善的点不发送，继续筛其余候选。
请求点与终点均保留全局/局部footprint和路径guard；发送前再次规划并复核。
请求/返回路径必须map且坐标有限；定位仍用map→base_link。
接近距离停止界限取min(原stop_radius,.81)，避免原.89提前取消；保留原取消/停稳与.81最终验收。
原1次重试上限保留，重试仍须通过上述改善条件。
日志新增具体拒绝原因及请求点/规划终点/停稳点三组位姿、物体平面距离；
规划终点来源ComputePathToPose，实际执行/重规划终点继续由PATH_RECEIVED记录。
不将预规划预算等同最终成功；后续重规划及定位/目标测距误差仍可能失败。

仅构建multi_agent_nav到 `build/go2-arrival-budget-20261015/{build,install,log}`，成功。
根包复用go2-manual-takeover、消息包复用go2-coop；未重建外部overlay。
实际安装相关165项+环境3项通过（新增20、既有148）；XML在该输出目录。
覆盖预算越界/非有限路径、正确三距离、参考系转换、真实停稳偏差后重试/最终失败、
落入已有Nav2容差的无效重试拒绝、参数发现/错误/取消清理、路径frame拒绝及既有碰撞条件。
源码首轮新回归确认旧实现失败后修复。未重跑无关全量基线。
AST对比前一个stop-on-target安装：adapter仅read_approach_parameters/compute_path/select_approach变化，
扫描及其他运动方法一致。`git diff --check`通过；Notes指引21个bash块语法通过。

CoVLM_env默认指向新安装，环境回归验证真实导入路径；原地更新
`/home/isee-pst/Notes/协同VLM启动指引.md`第6节与runbook末节，完整构建只在runbook。
用户下次：结束旧终端4并停稳后重新source→核对新包→只读查询.25/.50→按原指引继续。
本次无需重启相机/终端1/2/3；不代表获得新实验授权。未提交、未运行现场运动。
待现场：0.03m余量和.53m候选的实际可达性未验证；若原碰撞检查拒绝全部候选则停止，
不能为了距离条件放行障碍或把任务距离验收改成Nav2成功。

### 2026-10-17 协同握手等待与第二轮任务序号修复（源码及纯逻辑验证）

用户明确要求修复这两项；本轮不启ROS、bridge、SSH/Ranger或真机动作，不提交、不部署。
核对分支 `go2-cooperative-task-layer`、HEAD `eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`；
保留已有全部未提交修改。本轮实现改公共状态机
[agent.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py) 和
[coordinator.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/coordinator.py)，
回归补入 `tests/test_agent.py`、`tests/test_coordinator.py` 和 `tests/test_mission_mock.py`。

- 原因一：接纳首次SYNC后，`peer_received=None` 在下个tick立即触发停止。
  现使用现有 `lost_timeout`（默认3秒）作为首次对端状态的固定等待窗口；重复SYNC不延长窗口。
  等待期间拒绝EXECUTE，且要求本机停稳、动作终止、定位/Nav2有效；超时仍STOPPING→确认停止→PAUSED。
  当前任务首次对端状态到达后恢复原接收时间失联监护，不为后续断流追加等待窗口。
- 原因二：新任务只清内部last_sequence，外发sequence/command_id及PAUSED运行状态仍残留。
  现重建任务状态，保留地图/标定身份与本机位姿、健康和停车字段，清理旧命令、序号、资源请求及动作上下文。
  新任务另清对端心跳，必须重新收到当前任务状态才接受执行；上一轮序号不再导致新回执被coordinator拒收。
  复核另确认暂停后重开时，coordinator在新任务握手前误把上轮PAUSED当本轮故障。
  现仅对当前task_id的STOPPING/PAUSED触发该故障判断，其他健康/新鲜度检查与握手超时保留。
  未改消息接口、桥接路由、调度频率和单机入口；持有token时禁止换任务、停止确认、旧任务和迟到结果隔离保留。

验证使用源码路径和 `/usr/bin/python3`，设置 `PYTHONDONTWRITEBYTECODE=1`、
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`、源码及tests的PYTHONPATH，pytest禁用cacheprovider；不初始化ROS。
首批新增10项中5项修前失败，分别复现首次误停、重复SYNC下误停、新任务残留（正常/暂停后）及第二轮任务失败；
另5项安全边界检查原已通过。复核后再补2项暂停重开回归，均先确认失败再修复。
最终同一组既有28项+新增12项共40项通过，覆盖protocol、agent、coordinator、resources、
safety_regressions、mission_mock；正常两轮双fake均DONE且各自从序号1执行，暂停后的新一轮也DONE。
未重跑无关单机全量基线；未构建或更新安装overlay。当前默认单机环境仍引用
`build/go2-arrival-budget-20261015/install` 的multi_agent_nav，不将源码测试当安装或现场验收。
本轮未修复观察消息晚于扫描完成状态时可能提前判定搜索耗尽的问题；新路由部署、Ranger适配及跨机验收仍待完成。

### 2026-10-17 协同发令前重检与运行失联保护（独立构建完成）

用户要求握手后重新确认真实停稳及现有运动预检，未停稳等待、定位/许可故障暂停，
并阻止取消/失联后的晚到导航与VLM结果推进。保留前节修复及其他用户未提交修改；未提交。
协同入口仍为go2_cooperative.launch.py → agent_node/coordinator_node，动作链为
AgentMachine → AgentRunner → Go2Adapter；单机explore不接入coordinator。

- agent.py提取原健康/通信监护为check_health，供周期tick与发令复用；未停稳只等待，不误当定位故障。
  定位（含非有限pose）或nav_ready失败仍停车暂停，真正发令前必须重新读取现有Health。
- coordinator.py保留双方当前task_id屏障；无人可发令但有人未停稳时等待，不误报搜索耗尽。
- runner.py先监督/STOP，再adapter.tick及结果消费；发令前读新健康样本后取当前单调时间，
  复用定位、Nav2/许可/costmap和真实停止判据，重查command身份和paused；停稳后重做路径检查。
  取消清理等待动作及pending future，发送返回时不把STOPPING/CANCELED覆写为NAVIGATING/EXECUTING。
- adapters/go2.py仅在协同runner注入motion_guard时，于实际send_goal_async前执行同一闸门，
  覆盖扫描内部Spin准备或日志耗时跨过失联期限的边界；未发送请求不计为active motion。
  拒发返回None，导航等待重新规划，扫描保持原阶段；未注入guard的单机行为不变，未改扫描角度或策略。
  adapters/base.py同步返回约定；mock.py注入模拟时钟，无新增配置参数。

只构建multi_agent_nav到 `build/go2-task-control-20261017/{build,install,log}`，成功；
消息依赖复用build/go2-coop/install/multi_agent_task_msgs，根包复用build/go2-manual-takeover-20261015/install/vlm_nav。
未覆盖旧overlay，未切换CoVLM_env默认单机环境。完整构建只维护在[runbook末节](go2_runbook.md#2026-10-17-协同任务发令闸门握手及失联取消验证)。
从/tmp核对新安装prefix和实际agent/coordinator入口解释器及import；agent、coordinator、runner、mock、
adapters.go2、agent_node、coordinator_node七份实际安装模块均在新输出，且与源码SHA256一致。

回归先确认原实现失败再修复；实际安装78项通过（既有相关63、新增15），
XML为 `build/go2-task-control-20261017/installed-task-control.xml`，不重复计算源码测试。
覆盖握手成功但未停稳、规划后定位/许可变化、检查期间取消/超时、重复SYNC不延期、
运行失联后旧成功/迟到VLM/恢复通信、实际_motion发送边界和资源token保留；无ROS初始化。
独立复核重现2.99s预检→3.01s实际Spin先发再停；修后同一重现只有STOP、无Spin或残留motion。

仍用原握手/失联3秒及整任务600秒；搜索计时起点、扫描、到达距离、Nav2参数、传感器链路未改。
stop_confirmed未因握手或Nav2成功置真；停车仍需实际Health.stopped与motion_terminal确认。
未启动ROS、bridge、SSH/Ranger、录包或真机动作，不改现场验收标记。
观察消息晚于扫描终态的已知问题、新路由部署、Ranger适配和跨机验收仍未完成。

### 2026-10-18 候选标注缺失与Go2不动：RGB输入未到达观察器

用户要求分析当前启动实验后RViz无候选标注且Go2不动；本轮不修改运行代码、配置或安装，
不启动/重启驱动、不SSH、不调用API、不ARM、不发运动。核对分支及HEAD仍为
`go2-cooperative-task-layer` / `eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`，保留已有全部改动。

最新 `build/go2-vlm-experiments/experiment.20261018_143937.17e26c31` 于14:39:39记录
WAITING_INPUTS，14:44:37按原300秒时限REJECTED，原因startup input wait timed out，
motion_sent=false；没有CANDIDATE_GEOMETRY、VLM选点或导航事件。
最终result.json健康快照：RGB时间戳窗口为空、depth窗口30帧，RGB-D matched=0/total=0、
rgbd_ready=false，无有效snapshot，pending TF帧为0。NAV_READY/CONTROL_ARMED、外部门槛时效、
定位、local costmap、三个Nav2 action服务、深度单位及motion_fault_clear均正常。
**已确认直接阻断：本轮观察器始终未收到RGB，不能组成RGB-D对，启动等待门槛未通过；
不是已经生成候选后被规划拒绝，也没有证据把这次等待归因于图像TF。**

代码顺序为go2_check.run_ros先等待vlm_input_ready，再进入explore生成候选。
启动时generated_display为空，generated_exploration_markers仅返回DELETEALL；14:44:33起
7秒有界小消息采样收到13条、均仅action=3。RViz订阅和实验发布者话题一致，双方为
RELIABLE/TRANSIENT_LOCAL，故当前无标注与未运动由同一输入阻断解释。
默认安装的go2_check.py/go2_exploration.py与源码SHA256一致。

补充本地域有界只读检查：RGB与depth原话题各有1个camera发布端及1个实验订阅端，
发布RELIABLE、订阅BEST_EFFORT，无可靠性不兼容；发布端存在不代表新鲜数据到达。
14:42:57起6秒彩色CameraInfo无消息、深度CameraInfo收到新时间戳；稍后7秒窗口两路
CameraInfo均无消息，不据此宣称深度持续正常。读取现有相机参数enable_color/depth/sync
均true，两路profile均640x480x15，未写参数。没有新增Image/PointCloud诊断订阅；
临时CameraInfo/MarkerArray/diagnostic小消息订阅均已退出。observation_only本来不启动
diagnostics定时器，未收到该诊断不能另判节点失活。

前一轮14:33:36实验 `experiment.20261018_143336.dd206535` 曾生成3候选、选择P2并提交导航；
14:33:57因RGB-D帧年龄2.006s超过2.0s而取消，stop_confirmed=true。该时刻同步率仍29/30，
NAV_READY外部门槛、定位及costmap正常；不能将健康中的nav_ready=false误读为底层Nav2故障。
用户补充相机终端异常、显示约十分钟前的时间；这是用户观察，不单凭终端日志时间认定相机时钟偏移。
远端彩色驱动/USB状态与DDS传输原因尚未区分，未因本次诊断放宽RGB-D门槛或自动重启相机。

审计保存在 `build/go2-field/evidence/startup-input-audit-20261018/report.json` 及
`metadata.144440.json`（build不受Git保护，须保留）。离线断言核对事件顺序、无运动、
RGB/depth窗口、其他健康门槛及前一轮超龄取消均通过；未构建或重跑无关测试。
下一步先检查相机终端实际报错/远端彩色流，在恢复后按runbook做单观察器inspect；
不能在旧实验仍运行时叠加原始RGB检查订阅，也不把恢复输入当作获得新实验运动授权。

### 2026-10-18 相机重启后的新失败：检出目标后接近候选受局部窗口限制

用户报告重启相机后恢复、新实验突然报错。本轮只读日志/源码和离线复现，无新ROS节点、
传感器订阅、参数写入、SSH、API或运动；仅补证据及本文，保留已有未提交修改。
最新 `build/go2-vlm-experiments/experiment.20261018_144454.0fe0b41a`：
14:45:16观察点P2导航SUCCEEDED，后续4次Spin均SUCCEEDED；14:45:55第4视角检出
A=灭火器、confidence=.95、position_valid=true，地图目标=(.744896,2.911190)。
随后立即结束扫描并停稳，TARGET_FOUND→ARRIVAL_CONTRACT→APPROACH_PLANNING→
APPROACH_UNREACHABLE(no safe standoff candidate)→APPROACH_FAILED，stop_confirmed=true。
没有PICKUP_NAV、HEALTH_FAILED、SCAN_HEALTH_FAILED或APPROACH_CANDIDATE_CHECK事件。
不是这轮识别API或Spin失败；result.health为启动快照，不将它当失败时的健康实测。

接近参数实际读值xy=.25、planner=.5、成功半径=.81、margin=.03，生成圆半径=.53。
事件297停稳base_link=(.266456,-.103364)，距目标3.052285m；圆上任一点距机器人
至少2.522285m，实际8点最近2.530200m。运行生成配置
`/tmp/vlm_nav2_go2_zqtdc2jo.yaml`（14:38:35）为local width=height=3m、resolution=.05、
rolling_window=true；与启动报告60×60格一致。居中3×3窗口半对角线仅2.121320m，
因此该圈候选即使无障碍也在局部覆盖范围之外，不依赖map/odom相对旋转判断。

当前select_approach在调用compute_path之前，先用local costmap_candidate_free检查
每个候选整车矩形；出边界即false。探索观察点已改用全局停车预检，但接近候选仍保留
此局部窗口门槛，外层candidate_allowed的全局检查也无法救回已在内层排除的点。
这解释了本轮接近预检约2.4ms即返回、没有进入路径预算检查和接近导航的现象。

使用现场venv、从/tmp导入默认实际安装，调用原select_approach/costmap_candidate_free，
用记录的目标/base位置和全空3×3局部格离线复现：8点均拒绝、planner_calls=0、
reason=no locally reachable candidate。系统Python首次导入缺httpx，未改依赖，改用已有
`/home/isee-pst/venv/co-nav-real/bin/python`完成上述复现；未初始化ROS。
该人为全空网格不是故障时原始costmap回放；当时原始格及逐候选拒绝原因未保存，
不能证明目标处真实障碍格情况或断言改用全局后一定可达。
证据 `build/go2-field/evidence/approach-local-window-audit-20261018/report.json`，
同目录保留实际运行配置nav2-launch.yaml；build证据不得清理。
尚未修改接近逻辑、扩大局部窗口、放宽安全/距离条件或重新实验；后续若修复应保留
局部运行健康与避障，补足远端候选的全局完整车体/路径检查及对应回归。

### 2026-10-18 单机指引的一条命令入口（仅制作及离线验证）

用户要求按 `/home/isee-pst/Notes/协同VLM启动指引.md` 制作脚本，并明确选择完整流程：
真实初始位姿仍人工设置，必要SSH/底层启动以及ARM/一次探索保留现场确认。
新增 `scripts/start_go2_vlm.sh` / `scripts/start_go2_vlm.py`；使用仓库源入口，
一条命令及依赖顺序见[runbook新增节](go2_runbook.md#2026-10-18-go2-单机一条命令启动)。
复用CoVLM_env、现有相机SSH、正式预检、go2_system、当前Sport转换及generated_candidates_vlm入口；
不修改导航/接近算法、配置或现有安装，不启动双机链路。保留全部已有未提交修改。

实际实现包括唯一发布者/已有owner检查、当前安装prefix检查、冷启动独立新报告、
人工AMCL定位屏障、NAV_READY/四个Nav2参数及单观察器inspect稳定输入检查、
按状态确认reset/ARM、一次300秒探索、确认stop_confirmed后DISARM。
Ctrl+C转交本次探索并等待其停车报告；停止未确认时不清理锁、不自动重启或关闭底层。
新建检查日志在 `build/go2-field/single-vlm.<随机ID>/`；原实验托管/保留策略不变。
各硬件终端继续运行，停止顺序及异常处理已写入runbook；`--prepare-only`不ARM/不探索。

验证：新增23项离线编排测试、既有相关6项（VLMNav入口及CoVLM环境）均通过，共29项；
仅替换外部命令，不初始化ROS。Bash/Python语法、帮助入口和git diff --check均通过。
新增编排测试覆盖冷启动/复用、RGB-D不稳定/定位/停稳/其他owner拒绝、
服务失败、探索失败/证据未完成、未确认停车不DISARM及取消后停车再DISARM。
复核发现只结束ros2 run包装进程可能残留inspect观察器；真实父子进程测试先复现失败，
修为独立进程组并在检查超时/中断时清理整组后通过，不用真实ROS模拟该生命周期。
本轮没有SSH、传感器或ROS启动、API、ARM、录包或真机运动；不构建、不切overlay、不自动提交。
上述2026-10-18现场输入/接近失败的历史限制仍有效；启动脚本不构成这些问题的修复或新现场验收。

### 2026-10-18 接近候选局部窗口限制修复（独立构建，未切换默认入口）

用户明确要求修复：全局完整footprint检查、局部覆盖不足不单独否决但覆盖区碰撞仍拒绝，
地图/TF异常保留明确失败；保持8候选、0.53m对应距离预算、扫描及运行避障，本轮不启动真机运动。
分支/HEAD仍为go2-cooperative-task-layer / eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75，保留所有原有改动。
本轮前快照在build/go2-approach-coverage-20261018/before/，便于与既有dirty状态区分；未提交。

- adapters/go2.py复用原坐标变换、车体矩形及栅格交叠检查，增加结构化costmap_candidate_check。
  原costmap_candidate_free仍严格要求完整覆盖；新approach_candidate_free要求全局free且
  local为free/coverage_insufficient，局部覆盖区253/254/255仍拒绝。缺失/无效/超龄/TF失败
  为unavailable，不等同覆盖不足。select_approach在规划前后复用新检查，保持原8点及距离排序。
- go2_exploration.py在原请求点/实际规划终点预算复核处使用同一全局+局部检查，去掉残留的
  局部完整覆盖门槛；传播ApproachResult.reason并明确记录规划失败。
- go2_check.py独立approach及agent_node.py的Go2 live入口开启既有global订阅，补齐共享调用方。
  原单机explore已有该订阅；未改变其他动作/消息协议或传感器订阅。
- 接近日志新增APPROACH_FOOTPRINT_CHECK（global_check/local_check、accepted）及
  APPROACH_PATH_CHECK（reachable/accepted、reason）；coverage_insufficient的reason为“局部检查覆盖不足”，
  collision、全局out_of_bounds与unavailable分开。规划失败也无条件重检输入，不掩盖同期map/TF失效。

实际运行Nav2参数只读核对时间14:57：local plugins含voxel_layer，enabled=true、
observation_sources=go2_cloud、topic=/vlm_nav/obstacle_cloud，marking/clearing均true；
3×3m/.05m/rolling_window=true保持。障碍过滤器发布端与local/global costmap订阅端均存在，
BEST_EFFORT兼容；没有新增完整点云/RGB检查订阅，也没有写运行参数。
配置/端点证据为新输出的runtime-obstacle-input.json；不将端点存在或启用配置当作动态避障验收。

仅multi_agent_nav构建到build/go2-approach-coverage-20261018/{build,install,log}，成功；
根包复用go2-manual-takeover-20261015、消息包复用go2-coop，未覆盖旧overlay。
从/tmp核对四个实际安装模块（adapters.go2、go2_exploration、go2_check、agent_node）均在新输出，
与源码SHA256一致。完整构建、同输出测试及人工选用步骤只维护在runbook新增的2026-10-18小节。

初始相关基线165项通过。实际安装最终256项通过（新增36、既有220），XML为
installed-coverage.xml（253项）及installed-dynamic.xml（3项静态配置/launch检查）。
新增首批25项修前失败、1项既有边界通过；另补非有限位姿日志3项和末候选规划失败同时
local/global map或TF失效4项，均先失败再修复。复核发现的输入故障短路日志问题已修复。
覆盖全局障碍/未知、局部交叠碰撞/未知、完整车体越界、旋转TF/网格原点、地图缺失/无效/
header与接收超龄、规划期间变化、远端请求点及返回终点复核；新旧scan、tick、停止、导航、
compute_path、执行路径与costmap输入加载函数AST一致。git diff --check通过。

CoVLM_env仍默认go2-arrival-budget-20261015；本轮未部署或更新运行中的进程。
后续选用新输出需先退出旧实验并停稳，再按runbook加载并核对prefix；原输入/授权门槛仍适用。
未运行原生costmap标记/清除测试、无关全量基线、API、SSH/Ranger、桥接、ARM或真机运动。
未验证远目标实际可达、定位/语义位置误差及动态避障的现场效果；不把单元测试当任务验收。

### 2026-10-18 15:06实验仍为旧接近链路，默认入口尚未切换

用户要求分析当前报错；本轮仅查现有日志/代码及本地导入，不初始化ROS、不SSH/API、不改运行配置或运动。
保留另会话新增的start_go2_vlm脚本、测试及所有其他已有改动；未自动接入新安装。
实验 `build/go2-vlm-experiments/experiment.20261018_150602.909f2a46` 于15:06:18到P3
Nav2 SUCCEEDED；15:06:24首次Spin SUCCEEDED；15:06:32首视角检出A=.95、position_valid=true，
目标map=(.790566,2.867219)，API returned/validation_success均true。随后
TARGET_FOUND→ARRIVAL_CONTRACT→APPROACH_PLANNING→APPROACH_UNREACHABLE，旧通用原因
no safe standoff candidate；最终APPROACH_FAILED、stop_confirmed=true，无PICKUP_NAV、
HEALTH_FAILED或SCAN_HEALTH_FAILED。无新增APPROACH_FOOTPRINT_CHECK/APPROACH_PATH_CHECK。

当前CoVLM_env第11行仍无条件指向go2-arrival-budget-20261015；在/tmp source默认环境后，
实际multi_agent_nav prefix及adapters.go2/go2_exploration均来自该旧安装，adapter无
approach_candidate_free、探索无新helper调用。修复版go2-approach-coverage-20261018安装仍存在，
两个关键模块与源码逐字节一致；前节已明确本次独立构建未切默认入口。
新scripts/start_go2_vlm.sh第8行也会重新source CoVLM_env，若使用它，事先手动加载新overlay
仍会被该默认赋值覆盖。未发现与本轮对应的一键启动日志目录，不能断言这次用了该脚本。
实验未保存逐进程__file__；日志行为与旧adapter+旧探索一致，当前默认导入路径是本轮只读实测，
不将其写作历史逐进程模块路径实测。

本轮末停稳base=(-.318146,1.232401)，距目标1.975316m；与前一轮3.052m不同，
不能复用上轮“整圈必在局部窗口外”的几何证明。旧版未保存每候选拒绝理由及失败时原始costmap，
无法从该实验区分边界、障碍/未知或输入瞬态；只能确认接近未发出且修复版完整链路未运行。
审计 `build/go2-field/evidence/approach-version-audit-20261018_150602/report.json`；
事件顺序、无接近运动、新日志缺失、默认旧模块及新安装摘要离线断言通过，git diff --check通过。
下一步需按runbook选用新安装并核对实际prefix；若希望一键入口默认使用新版本，应同步处理
CoVLM_env及入口校验，不能仅在运行前source后又被脚本覆盖。本轮只分析，未切换默认入口或重跑实验。


### 2026-10-18 默认实验入口已接入接近覆盖修复安装（无现场启动）

用户明确授权先接入新安装、核对加载路径，原地修改Notes启动指引及scripts/start_go2_vlm.sh；
本轮未授权新实验或运动。保留原分支、HEAD和所有已有dirty修改，不重建或覆盖现有安装。

- CoVLM_env.bash的GO2_EXPLORE_OUTPUT由go2-arrival-budget-20261015切为
  go2-approach-coverage-20261018；EXTRA_SETUP_FILES及ROS_LOG_DIR随同切换。
  相机/过滤/生成候选/RGB-D/Sport覆盖层及顺序保持。已运行实验进程不会自动更新。
- scripts/start_go2_vlm.sh在source之后、进入原Python启动器之前校验两个真实模块路径
  （adapters.go2/go2_exploration）必须位于预期prefix，且Go2Adapter含approach_candidate_free。
  不匹配明确失败并阻止启动流程；--help仍直接退出、不加载环境。原启动器现场/ARM确认未修改。
- /home/isee-pst/Notes/协同VLM启动指引.md已原地更新日期、新prefix、无ROS模块核对命令和
  全局/局部候选安全语义及新日志。先准备并核对完整diff/语法，再摘要核对原文件未被并发修改后写回。
  原文件快照在build/go2-approach-coverage-20261018/entry-integration/before/，不清理。
  旧实验退出且确认停稳后，终端4须重新source并按更新指引第6步核对，原相机/1/2/3可继续运行。

实际从/tmp加载默认环境，ros2 pkg prefix multi_agent_nav为
/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-approach-coverage-20261018/install/multi_agent_nav。
四个模块adapters.go2、go2_exploration、go2_check、agent_node均来自该安装且与修复源码SHA256一致。
root vlm_nav仍来自go2-manual-takeover-20261015。独立执行一键入口的source/导入检查段通过，
没有进入Python启动器；新安装multi_agent_go2_check shebang为既有co-nav-real解释器。
实际ros2 run新入口--offline --require-ready输出single_robot_experiment_gates=[]，未初始化ROS。
证据均在上述entry-integration目录：default-import-audit.json、entry-import-check.log、offline-config.json。

本轮基线26项通过；更新环境断言及新增4项入口测试后30项通过（既有26＋新增4），
覆盖正确安装、错误路径/缺修复拒绝、help不加载环境。修改前新增断言4项失败、2项既有边界通过，
XML为baseline.xml、red.xml、installed-entry.xml；未因环境改动重跑前节256项候选/协同测试。
Notes全部22个Bash命令块及入口脚本语法、模块核对命令、链接和git diff --check通过。
完整构建命令仍只在runbook，新增默认入口接入小节记录本轮实际路径与验证。

未运行真实一键启动、相机/驱动/ROS节点、API、SSH/Ranger、桥接、ARM或真机运动。
本次仅完成默认文件/环境接入；目标实际接近、局部动态障碍停止/重规划效果仍待后续授权现场验证。


### 2026-10-18 15:20实验在首次识别等待期间因RGB-D快照超龄取消

用户确认问题对应experiment.20261018_152003.83586c9e；本轮仅读取已有日志/代码并离线审计。
15:20:18.856 VIEWPOINT_NAV在P3 SUCCEEDED；15:20:21.687首次SCAN_SPIN SUCCEEDED。
15:20:24.180 SCAN_HEALTH_FAILED发生于INFER/view_index=0，仅failed_gates=[rgbd_ready]。
最新有效RGB-D快照接收年龄2.009976s，超过既有2.0s；camera_tf_age=1.872490s<3.0s，
同步窗口30/30匹配、rate=1.0，pending TF=0，定位/局部costmap/odom/外部gate及action服务器检查均通过。
两路已记录header窗口止于15:20:22.179785；说明观察端新鲜快照出现约2秒更新空窗，
不能仅凭此判定相机退出、USB故障、网络丢包或接收处理阻塞。启动时的past extrapolation警告
（15:20:04）后已有正常导航/快照和TF，不是此刻失败门槛；不将其当直接根因。

原scan健康保护先StopMove/取消观察，再记录失败；最终observation cancelled，stop_confirmed=true。
VLM未超20s API预算，约8.002s后于15:20:29.803正常返回并校验通过，A=DETECTED/.95，
但任务早5.62s取消，故VLM_VIEW_RESULT为CANCELED，无TARGET_FOUND/接近规划/候选检查或PICKUP_NAV。
result.json的vlm_evidence_pending=true是退出报告时后台证据未结束；之后metadata已完成，不能判API失败。
该轮未进入接近候选链路，不能据此判定覆盖修复现场通过或失败。通用health.reason里的
execution path missing or receive timeout也不是触发门槛：导航终态会清除执行路径，本次仅rgbd_ready失败。

审计build/go2-field/evidence/rgbd-stale-audit-20261018_152003/report.json及离线断言通过；
核对get_health/vlm_input_ready/tick、实验events及VLM metadata，未改代码/阈值、未重跑测试或实验。
git diff --check通过；未初始化ROS、额外订阅图像/点云、SSH、API、ARM或运动。
后续根因仍需故障时发布端相机日志和接收端时序证据；本次不以放宽阈值或重启代替定位。


### 2026-10-18 15:28实验因网线断开导致SYSTEM_READY丢失（用户现场确认）

最新experiment.20261018_152827.2e0abea6在15:28:45.435到P1导航SUCCEEDED，
15:28:49.158首次SCAN_SPIN SUCCEEDED；15:28:49.611 Sport桥记录
bridge entered FAULT: SYSTEM_READY lost while ARMED，随后拒绝cmd_vel并保持FAULT。
15:28:49.614 SCAN_HEALTH_FAILED位于FRAME/view_index=0，仅control_armed_gate=false；
RGB-D age=.595719s、同步30/30、TF及局部costmap/定位检查通过，不是上一轮RGB-D超龄。
雷达转换器15:28:49.619起received=1339/published=1338，后续多条5秒周期报告均不再增长；
FAST-LIO最后帧日志15:28:49.081。失败样本odom_age=.530363s>.5s，无法依新鲜里程计确认停稳。
SYSTEM_READY监督器只记录Bool变化，不能从该样本指定其内部某一个最先失败的输入条件。

本地只读检查时enp4s0 link/carrier仍UP，但无IPv4，NetworkManager显示30(disconnected)、无连接。
用户先确认未人为操作网络/电源，随后明确确认“是网线断了”，据此根因确定为网线断开引起底层通信中断。
链路为输入中断→SYSTEM_READY下降→Sport桥锁存FAULT/control_armed=false→扫描取消。
未进入VLM识别或接近候选规划；最终observation cancelled、STOP_UNCONFIRMED、stop_confirmed=false。
只证明曾发送StopMove，不证明该轮实际停稳；不得当作自动恢复运动依据。

离线断言及审计build/go2-field/evidence/system-ready-audit-20261018_152827/report.json通过，
其中已补用户现场确认；未更改代码/网络/阈值、未初始化ROS或额外订阅、未SSH/API/ARM/运动。
系统journal普通权限不可读，sudo -n需密码，未取得对应内核/NetworkManager事件；没有要求用户提供密码。
git diff --check通过。接线后输入恢复、实际停稳及既有reset/ARM门槛尚未验证，本轮未执行恢复或重试。


### 2026-10-18 本轮 C 全程不共享坐标，各机独立寻找（源码及独立构建完成）

用户确认“本轮全程不共享、靠自己寻找”，场内只有一个实物 C，并批准简短设计：
各机提前识别 C 后保存在本机，仅同步本机是否已定位；双方 READY 后各用自己缓存，缺少的一方继续找，
仍保留 Go2 先放入并整机退出、Ranger 后放入和退出及原资源许可。该明确指令覆盖旧设计的本轮共享 C 策略。
未授权部署、SSH/Ranger、桥接或新现场实验；保留原分支 HEAD 及所有此前未提交修改，未提交。

- scene.yaml 设置 `share_c_coordinates: false`；缺省 true 保留旧共享流程，mock 原基线不改。
  make_machine 将策略传入本机 context，配置检查拒绝非 boolean。
- AgentMachine 增加当前任务内存 C 缓存，只有当前未取消扫描的有效 C 可缓存；新任务清空，重启不复用。
  外发 C 的位置及 reason 均清空，只有有效缓存才发 `CACHED`、position_valid=false。
  A/B 共享不变；agent_node 私有模式忽略对端 C。没有新增相机/点云常驻诊断订阅。
- 协调器只记录每台自己的 C observation_id，不接受 C 坐标。双方 READY 后仅派未有本机 C 的一方继续搜索；
  双方都有 C 后，APPROACH_DEPOSIT 仅传各自引用、两个位置字段为空、position_valid=false。
  agent 仅从自己本轮缓存解析后在本机规划，拒绝缺失缓存或附带坐标的私有 C 命令，不用零值后备导航。
- 独立复核复现同一扫描 API_ERROR 后重试成功被旧记录遮住，已允许同机同引用非 CACHED→CACHED 升级；
  端到端回归先失败再通过。未改变普通观测去重或取物逻辑。
- 补 `SYNC_LOCAL_C` 模式握手（现有 string 动作字段，无消息字段变更）：
  旧 agent 或新版共享配置拒绝私有同步，私有配置拒绝普通同步，建立任务身份前检查。
  双方 task_id 屏障未满足则不发扫描；独立复核实测旧版 agent 也不会进入本轮任务或发出 C。
  两端公共任务代码及配置必须一致；外部 Ranger live adapter/安装未更新，不能声称双机已实际关闭共享。

独立输出 `build/go2-local-c-20261018/{build,install,log}`，仅构建 multi_agent_nav 成功；
依赖复用 go2-coop/install/multi_agent_task_msgs 及 go2-manual-takeover-20261015/install/vlm_nav，未覆盖它们。
从 /tmp 加载该输出核对十个模块实际路径/SHA256 和 scene=false；真实 ROS 消息序列化往返及
接收端 C 忽略、A 保留的无初始化断言通过，证据 `installed-audit.json`。
未切换 CoVLM_env 或一键入口，默认仍为前节 go2-approach-coverage-20261018；运行进程未热改。
构建环境及复现命令仅维护在 [runbook 本轮 C 小节](go2_runbook.md#2026-10-18-双机各自定位-c本轮不共享-c-坐标)。

验证：原核心基线 52 项通过；最终新安装核心 **85 项通过（原52＋新增33）**，
`installed-core.xml`，含完整双机 fake、两端各找 C、独立引用、重开缓存失效、重试恢复、
模式混用不扫描及放入/退出许可。新增测试先确认缺少功能时失败；独立只读复核两项问题修后关闭。
扩展源码检查修正日志环境、明确排除项后 **413 通过、5 deselected**，见 `source-selected.xml`，
不与核心检查累加计数。没有宣称全量测试或完整现场验收通过。

首次扩展 `source-offline.xml` 为406通过/6失败/1deselected，当时新增28项：
两项 launch 静态检查因 ~/.ros/log 不可写，改为可写日志后通过；
flags 旧断言与当前用户已验证配置不符、现场 motion_slot 锁已有16字节标记，两项保留未修；
两个 Domain31 合成原生 costmap 退出检查用旧 go2-coop host 出现 -11，未在本轮排查原生根因或重跑。
具体全部失败用例名及最终5个deselected/ROS smoke未执行项见runbook，未隐藏首次失败。
扩展检查还创建并销毁 Domain31 localhost 的只读观察节点；合成 costmap进程均已退出。
本轮不能写为“完全未初始化ROS”，但未连接真机、SSH/Ranger、bridge、真实API、录包、ARM或真机运动。
未清理现场停车锁，前节网线断开后的STOP_UNCONFIRMED对账仍待完成，不能据本轮测试恢复运动。

剩余：后续按授权核对两端安装并部署、Ranger任务适配及现场闭环；当前缓存不做跨进程磁盘持久化。
观测晚于扫描终态可能提前耗尽的既有问题未在本轮扩展；本轮唯一C假设由用户确认，未从现场重新验证。


### 2026-10-18 双机旧状态通信检查脚本（仅制作及离线验证）

用户要求Go2、Ranger各一个通信验证脚本；承接此前旧里程计state链路讨论。
新增 `scripts/check_go2_comm.sh`、`scripts/check_ranger_comm.sh` 和共用
`scripts/check_multi_agent_comm.py`，没有改启动入口、底层/桥路由、DDS配置或新任务协议。
从源码直接运行，不需构建；两个入口依赖已加载的本机Humble/旧multi_agent_msgs/Domain0 DDS环境。
Go2旧接口独立安装环境实查为
`/home/isee-pst/unitree_ros2/install/multi_agent_msgs/share/multi_agent_msgs/local_setup.bash`，
只加载该包，不用旧整套overlay覆盖当前包。Ranger自身安装和网卡路径仍待本机核实。

检查依次为本机真实里程计（Go2 `/odometry` / Ranger `/odom`）、本机旧agent状态、对端旧状态。
每层默认10秒，最大龄2秒，唯一发布者/消息类型/本机agent来源/robot_id必须匹配；
至少3条且时间戳推进2次，拒绝零/倒退/超龄/未来时间，并比较真实agent里程计订阅QoS。
早先订阅保持更新，退出前复核新鲜度、发布者和agent订阅QoS。输出端点和QoS、收包/时间戳推进次数和最新年龄。
退出0是本机检查通过，1是数据/端点失败，2是配置/运行错误，130是中断；两端均PASS才能确认旧链双向连通。
不启动驱动、agent、桥、coordinator或运动，不订阅图像/点云，不把旧state当READY或新任务验收。
操作前置条件、Go2环境顺序及Ranger复制文件说明见
[runbook通信脚本小节](go2_runbook.md#2026-10-18-双机旧状态通信检查脚本)。

新增 `test/test_check_multi_agent_comm.py` 并注册CMake pytest项。
新增33项离线测试通过，相关既有 `test_go2_docs.py` / `test_go2_offline_tools.py` 15项通过（共48项）。
覆盖时间戳/身份/新鲜度、缺失/重复发布者、Go2/Ranger完整检查流程、QoS不匹配、输入中断、退出和清理；
ROS I/O使用测试替身，没有初始化DDS。只读复核发现退出前遗漏端点复查，动态图4项回归先失败后修复通过；
另真实Humble消息/API导入、QoS的OK/ERROR/WARNING三种无初始化断言通过。
最初QoS探针把双方SYSTEM_DEFAULT liveliness错误预期为OK而失败，查实际返回为WARNING；
脚本原本就拒绝未知兼容性，没有为通过探针放宽现场门槛。
两入口Bash语法、Python编译和git diff --check通过。
未全量测试、构建或部署，没有执行实际通信检查、初始化ROS、连接Ranger/SSH、启动桥接/驱动、录包或真机动作。
两端输入/agent/桥须先按已有授权和部署就绪；当前IP、Ranger配置及实际双向通信均待现场验证。


### 2026-10-18 根据用户通讯记录重制新任务PAUSED脚本（未现场执行）

用户提供 `/media/isee-pst/MNGS2/Temp/3.通讯验收.md`，要求按其中Ranger命令及本地Go2代码重做两端脚本。
已读原文件：本轮目标是 **multi_agent_task_msgs新协议的PAUSED状态传输**，不是前节旧里程计state链。
该文件给操作命令而非实际收包输出；不能据此声称今早或当前双向通信已通过。
本轮仅获制作/离线验证授权，未扩大为部署、Ranger连接、ROS参与者、桥接、agent或运动实验。

原 `scripts/check_go2_comm.sh`、`scripts/check_ranger_comm.sh` 与共用Python脚本已原地改为新协议；
runbook原旧协议小节标为历史，保留此前33项结果，不把旧结果改写为新版测试。
当前使用方法及逐终端依赖顺序在
[新任务PAUSED通信脚本](go2_runbook.md#2026-10-18-新任务paused通信脚本)。

- Go2入口source本地 `CoVLM_env.bash`，配置取GO2_CONFIG，否则取其既有GO2_EXPLORE_CONFIG。
  Ranger入口按新记录source `/home/isee601/ws/VLM_Nav/RangerVLM_env.bash`，配置取RANGER_CONFIG；
  不再依据此前GitHub旧main中的 `/home/isee-cdh/ws/install` 入口猜测Ranger当前环境。
  Ranger入口和 `check_multi_agent_comm.py` 须同目录，未自动复制。
- 默认preflight仅环境/接口导入、读取配置/网卡/Git，不调用rclpy.init；打印实际包/模块路径、
  三种消息的有序字段及SHA256、实际网卡和真实gates。两端须人工核对三个指纹。
  拒绝本地域DDS走共享网卡、错误域/额外路由、旧消息和非Reliable/Volatile；不自动改网络/配置。
- 显式bridge/agent动作重复检查真实gates非空，并用ROS CLI查同名/前缀节点防重复后exec文档命令。
  bridge使用各自按域XML并unset ROS_DOMAIN_ID；agent用Domain0、simulation=false、/multi_agent，
  节点名分别multi_agent_go2_agent / multi_agent_ranger_agent。不启coordinator/start或硬件。
- check默认12秒订阅本机和对端state；检查schema1、身份、simulation=false、PAUSED及非空reason、
  ready/deposited=false，stamp至少推进2次、至少3条，新鲜度和最大间隔按配置lost_timeout。
  stamp按协议仅追踪，不以两机墙钟差替代接收监护；阻塞态sequence可一直0，不要求定位/pose有效。
  采样结束检查本机agent和本机task bridge导入的发布端/类型/QoS，不认证采样期间每条消息来源。
- `check --expect-peer-stop` 先确认有正常数据，再提示人工Ctrl+C退出对端agent，
  保持本机agent及两桥以观察停流；期间状态异常、发布端变化或未停流均不通过。
  先验Go2接收端，再恢复Ranger agent后验反方向，避免同时停两端；停流还须结合人工退出时刻记录。
  退出0区分PREFLIGHT_OK与DATA_OK，不代表定位/Observation/READY或完整任务验收。

验证：改写前旧协议33项测试在前轮已通过；改写后先确认新协议首批30项失败，再实现通过。
最终新版脚本 **48项通过**，相关既有test_go2_docs/test_go2_offline_tools **15项通过**（共63，不与历史33累加）。
覆盖Ranger与Go2流程、当前五话题配置、各自DDS接口、开放gates拒绝及预检后再次拒绝启动、
真实新字段/sequence0、错误身份/状态/来源/QoS、重复发布者、接收中断、人工停流及退出清理。
所有流程测试使用ROS/网络测试替身；独立只读复核的当时45项亦通过，未发现阻断缺陷。

另单独加载既有go2-coop、go2-manual-takeover-20261015及go2-approach-coverage-20261018的安装环境，
检查实际config/agent模块均来自go2-approach-coverage-20261018/install/multi_agent_nav/local/lib/python3.10/dist-packages。
静态确认settings按local_robot检查gates，AgentNode非空gates分支不创建adapter；
以当前源码配置目录计算14条真实缺项，包含vision_validated、basket、对齐report、scene disabled、
Ranger地图身份/路径/digest、wait/exit/viewpoints及path_source/peer_obstacle验证项。
真实ROS AgentState对象的PAUSED/sequence0断言通过，仅导入/构造消息，没有初始化ROS或创建节点。

实际尝试source CoVLM_env时因enp4s0无192.168.123.x地址失败，未继续执行现场preflight、未改网络/参数或猜测新根因。
两入口Bash语法、Python编译、文档命令块/链接和git diff --check已核对；不构建、不全量测试、不提交/部署。
未执行ROS CLI查重/bridge/agent/check、SSH/Ranger连接、录包、API、ARM或真机运动。
Ranger环境文件/安装实现及两端网络、收包和停流仍待按授权现场核对。


### 2026-10-18 用户报告两端新任务状态均DATA_OK

用户在运行新版PAUSED通信脚本后报告“都收到了DATA_OK”。这是用户现场报告，非本轮工具直接收包。
按当前脚本判据，记录为两端新任务PAUSED状态双向采样通过；不改写为定位、Observation、READY或完整协同通过。
尚未提供两端完整输出、消息指纹、实际安装路径/当前IP和采样参数，故不补写未知字段或声称独立核实运行版本。
前节source CoVLM_env时enp4s0无IPv4的错误保留为当时记录，不据此否定用户本次DATA_OK，也不推测现场如何恢复。

下一步仍为runbook中的人工停流核对：先在Go2接收端启动check --expect-peer-stop，待提示再人工退出Ranger agent，
本机agent和两端bridge保持运行；如需反向核对，先恢复Ranger阻塞agent，随后在Ranger接收端等待并按提示退出Go2 agent。
两端不能同时停agent，也不能把初始无消息当停流通过。当前未收到STOP_OBSERVED或人工退出时刻记录。
本轮只追加用户反馈及说明下一步，未运行ROS、连接Ranger、启停节点或发送任何运动/任务命令。


### 2026-10-18 固定地图切换为 lab_rescan_20261018_155656_0pa6gm（仅文件及离线核对）

用户指定固定地图改用 `maps/lab_rescan_20261018_155656_0pa6gm`。
已读AGENTS.md并核对分支go2-cooperative-task-layer、HEAD eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75；
保留此前全部未提交改动及新地图文件，不提交。此次改动前文件快照在
`/tmp/go2-map-switch-20261018-pi0f59io/`，不是长期证据目录。

新地图YAML为 `/home/isee-pst/unitree_ros2/VLM_Nav/maps/lab_rescan_20261018_155656_0pa6gm/lab.yaml`，
引用同目录lab.pgm；实读472×578、resolution=0.05、origin=[-8.09,-20.8,0]、mode=trinary、negate=0、
occupied_thresh=0.65、free_thresh=0.25。
YAML SHA256=`a4d4a35ed4ba2203cb3decfb8b3a991ff57cb90294ab1c351c063eaef4ad7955`；
PGM SHA256=`0085c2d7ba8868f8d4ff76eb27aa911e2ca5324ea531f77347d0a1f1a9241520`；
按现有map_file_info规则得到map_digest=`6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279`。

- 源robot_go2.yaml更新map_path/map_digest；alignment.yaml更新Go2的local_map_id及sha256版本。
  Go2等于lab_ref的基准定义保留；既有validated=false/report=null及scene.enabled=false未解锁。
- scripts/start_go2_vlm.py冷启动地图参数及runbook中的冷启动/隔离标注路径同步新图。
  一键入口继续显式读取源码GO2_EXPLORE_CONFIG，无需构建；没有修改已有安装或热切运行节点。
  不带--config-dir的其他入口仍须核对其实际安装配置，不能据源码修改宣称安装已更新。
- 新图目前只有lab.yaml/lab.pgm；scene.yaml中的两个旧图Go2探索点移出活动列表，列表置空。
  原始精确点位仍保留于本文此前记录、runbook历史表和改前快照；旧地图目录/语义/观察点文件均未修改。
  generated_candidates.semantic_yaml及semantic_search.observation_points改指新图目录待采集文件。
  未复制旧家具坐标、推算点位、伪造标注或沿用旧地图坐标；新图探索点及家具标注须重新实测采集。

验证：相关既有test_start_go2_vlm.py/test_go2_docs.py基线32项通过；更新冷启动地图参数断言后仍32项通过，
没有新增用例或把断言数当用例数。独立无ROS初始化断言核对地图/图像可读、尺寸、摘要/身份/版本一致、
map_file_gates为空；固定点加载器拒绝空列表，家具/生成候选策略拒绝缺少新图标注，完整live门槛仍阻止运行。
默认fixed_viewpoints的--offline摘要不检查空列表，不可把其退出0当可探索；实际explore入口先检查measured_viewpoints。
文档命令块语法、当前地图链接及git diff --check通过。

未构建、部署、初始化ROS、启停或切换map_server/AMCL/Nav2、连接SSH/Ranger、启动桥接、录包、API、ARM或真机运动。
当前运行地图、AMCL在新图上的定位、新探索点及双机地图对齐均待按授权验证；历史现场结果不转移到新图。


### 2026-10-20 公共本地执行核心（Go2/公共层＋fake Ranger）

用户明确“进入实现”，并补充远端候选使用全局costmap/完整footprint/规划路径，local覆盖不足本身不否决；
只实施本机独立搜索、READY屏障和原许可链。已阅读AGENTS、单机框架及相关代码，分支仍为
`go2-cooperative-task-layer`、HEAD仍`eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`。
保留此前全部dirty修改、地图、日志及运动锁，不提交、不部署、不切换默认启动环境。
改前相关源码/配置/测试快照：`build/local-execution-20261020/before/`，按仓库相对路径保存；不是新Git提交。

**实现及修改位置：**

- `go2_exploration.py`提取`explore_steps`，原`explore`同步驱动，协同AgentRunner逐tick驱动同一循环。
  本地候选、VLM选ID、导航、8×顺时针45°、停稳新RGB-D、图像时刻TF、ABC同图及自身目标提前结束保留。
  `generated_candidates.py`按robot_id读取自己的地图/身份/几何，图例也标本机。
- 单机继续读原0.81m合同；`agent_node.py`/`runner.py`给协同注入1.00m。
  候选预算/补偿/停稳验收同一门槛，实时XY=0.25、余量0.03时预算为0.53/0.72m。
  当前目标导航SUCCEEDED、身份有效、真实停稳、action terminal、实际base_link距离达标才READY/完成放入；
  不用已有近距离或距离触发取消替代协同当前导航成功。
- `planner.py`/`coordinator.py`只发本地目标意图，不派固定点。Go2找A、Ranger找B，各自成功才READY，
  双方新鲜READY后进入C；C提前只本机缓存，CACHED只报引用，缺自己C则自己继续找。
  无共同C绑定、无跨机坐标转换，所有网络目标/任务位置均无效且为零。
- `models.py`在私有模式拒绝坐标式EXECUTE，防止绕回旧pickup验收；
  `frame_adapter.py`本地路径不做数值变换，保留各端本地地图身份/版本；
  `config.py`新模式仅取消跨机对齐依赖，仍完整检查本地地图、候选/扫描/到达参数、资源/退出及真实验证项。
  `agent_node.py`接收端检查本机任务、私有模式、对端健康和固定地图身份，任务内地图变化暂停。
- `agent.py`/`runner.py`将内部导航交回原AgentMachine路径/门/篮子许可链。
  门通行释放后恢复原主任务，C许可贯穿放入与EXIT，EXIT保留deposit的parent和token，Go2退出后才放行Ranger。
  非C接近阶段不申请basket，避免持篮子许可停在READY导致屏障死锁；允许door走原通行退出链。
  VLM等待交回executor；失联/取消关闭当前核心，迟到Future/goal不能推进；
  证据清理异常不跳过native cancel/stop，执行器初始化异常也停机回报。
- `adapters/base.py`列明公共异步/成本图接口，`fake.py`/`mock.py`提供双替身及安装后资源路径加载。
  `adapters/go2.py`本轮仅补本机图例文字；此前全局/局部覆盖保护实现保留。

**验证（无ROS初始化或现场操作）：**

最初较窄的165项基线162过/3失败：2项因overlay环境，补环境通过；
1项测试依赖已被清空的现场固定点，改成测试内部合成点，不为真实新图补点。
新增核心/严格距离和目标任务检查先失败后实现；只读复核发现并复现配置漏检、坐标命令旁路、
清理异常跳过停车及basket/READY死锁，已修复并补相关检查。
最终安装相关集合 **276项通过：既有235＋新增41**，排除1项会初始化ROS的observer测试；
未运行全仓ROS smoke/native节点/现场运动锁测试。原单机扫描、接近及远端候选检查包含在该集合中。
新增3项core、36项local_tasks、2项runner异常保护；不是colcon条目数或断言个数。
同样0.90m：单机失败、协同通过；双机>1m/未停稳/取消不READY或放入；
双fake验证两端VLM选点、心跳/迟到丢弃、独立坐标、A/B→READY→各自C→顺序退出，门/篮子许可不绕过。
只读复核最终118项也通过，范围仅公共代码和替身。

**独立安装和证据：**

`build/go2-local-execution-20261020/install/multi_agent_nav`已构建（仅1个变化包）。
根包vlm_nav依赖复用`build/go2-manual-takeover-20261015/install/vlm_nav`，
消息复用`build/go2-coop/install/multi_agent_task_msgs`，没有修改消息字段、桥接路由或源配置数值。
从/tmp导入14个安装模块与源码摘要相同，7个配置文件与本轮改前/安装逐字节一致；
SYNC_LOCAL_C/CACHED真实ROS类型序列化往返通过，rclpy未初始化，完整live gates仍关闭。
安装版fake首次因源码相对路径查到达文件而失败，已改用已安装vlm_nav资源目录后备加载，最终276项通过。
证据为`build/go2-local-execution-20261020/installed-related.xml`和`installed-audit.json`；
源码测试、构建/安装日志及改前快照位于`build/local-execution-20261020/`，不受Git保护，不清理。
构建、source、安装核对和无节点测试完整命令统一维护在[runbook本轮节](go2_runbook.md#2026-10-20-公共本地执行核心本轮入口差异与离线复现)。

**剩余边界和交接：**

默认CoVLM_env/启动安装仍未切换，不能据源码实现声称当前运行进程已更新。
新图仍缺家具标注/候选资产，本地basket/门/退出点和真实验证项尚待现场补齐，live gates继续阻止任务。
本仓库Ranger live分支仍明确要求在Ranger PC安装自身adapter，本轮只fake验证，未接入Ranger硬件。
最小同步清单、公共接口/命令/状态、私有地图要求及原五话题方向已写入
[Ranger本轮交接](ranger_handoff.md#2026-10-20-本轮最小接入说明覆盖旧共享-c-要求)。
不得整体覆盖Ranger分支或复制Go2硬件配置；后续在Ranger电脑保留自身adapter/地图/参数接同一核心，再按新授权真实跨机验收。
本轮未初始化ROS、连接SSH/Ranger、启停桥接、真实VLM请求、额外订阅/采集/录包、ARM或真机运动；未自动提交。


### 2026-10-20 空地候选及家具可选（单机/协同公共生成器）

用户明确当前地图不录入家具label，并确认“空地为基础、家具可选”“优先当前位置附近、逐步扩展已知地图”。
本轮基线分支go2-cooperative-task-layer、HEAD 6a786c1497ca2215f8181b39782c03139fc67d7d，改前工作区干净。
用户已批准会话方案并要求实施；本轮不提交/推送、部署或切换默认环境。

实现：generated_candidates.py合并已知自由网格点与原家具点，网格间距max(1m,2×完整旋转半径)向上取整到地图分辨率，
从半步格子中心采样，正确应用map原点/旋转，空地朝向地图X轴。继续完整旋转圆、边界/障碍/未知和家具占据区检查。
保留家具ID，空地点ID由地图行列确定，furniture_id为空，距家具候选半步内去重。
未配置semantic_yaml、文件不存在或合法空家具均允许，不创建/伪造标注；已有非法文件/摘要/身份不符仍报错。
地图/图像/标注资产出现、删除或变化使当前选择失效。

go2_exploration.py每轮按距当前base位置的距离与稳定ID排序，再经原实时costmap/路径/资源检查取最多12点。
原观察记录、失败冷却、地图变化/取消迟到保护保持。adapters/go2.py更新提示词允许无家具选空地点；
保留双图和只返回ID/理由的接口，无家具时第二张图不含家具层。go2_check.py无标签路径用null报告。
没有改变根包、协议、Nav2配置、单机0.81m/协同1m、8×45°/ABC扫描、时限/重试、C缓存或许可链。
固定点和旧semantic_furniture策略未改。家具可选不表示basket/门/退出点或验证项可省略。

验证记录：改前相关基线276项通过，1项初始化ROS的observer测试排除。新增16项无节点用例，
首批10项先因旧实现不支持无家具而失败，修后16项全部通过；原断言只允许4个家具点的测试改为区分家具/空地点。
观察全部候选的fake测试总预算由60秒改为600秒以覆盖新增候选，真实入口时限未改。
最终集合扩展覆盖旧semantic_furniture策略；源码相关320项通过，其中既有304项、新增16项，仍排除1项observer测试。
只读审查独立运行82项通过，未发现本轮新增阻断问题。未执行全仓ROS/native节点/现场运动锁检查。

独立构建位置build/go2-free-space-20261020/install/multi_agent_nav；只重建变化包，根包和消息复用原已记录依赖。
当前新图家具标签是可选输入，不再要求补录。其他资源/退出实测和验证缺项仍阻止live；Ranger仍待真实adapter接入。
当前默认CoVLM_env及单机一键脚本仍重载1018安装，新能力须显式source本轮新输出，原1020公共核心安装也不含本次变更。
构建/source/安装核对完整命令统一在[runbook空地候选节](go2_runbook.md#2026-10-20-空地候选与可选家具标注)。
外部Notes/启动指引/VLM_Nav协同指引_1020.md同步去除家具必填并给出新安装路径；Ranger交接补充公共同步范围。
本轮未初始化ROS、连接SSH/Ranger、启动桥接/硬件、真实VLM、额外采集/录包、ARM或真机运动。

独立安装最终核对：从/tmp加载4个变化模块与源码摘要一致，7个配置与HEAD/安装一致，rclpy未初始化；
当前新图无家具标注生成518个静态候选，离线单机门槛为空而完整live门槛仍关闭，motion_sent=false。
安装相关320项全部通过（304既有＋16新增），1项ROS observer排除；证据目录
build/go2-free-space-20261020/evidence/内保留baseline/source/installed XML、installed-audit.json及构建日志。
这些build证据不受Git保护，不清理；本轮既未自动切换默认安装，也未更新Ranger控制电脑。

### 2026-10-20 独立C与取消保护区（最新）

用户明确去除篮子保护区和门限制，并选择“取消先后顺序，双方各自完成C”。本节覆盖旧许可链、共同退出及Go2先行要求。
基线分支/HEAD未变；改前空地候选实现已由用户暂存。本轮保留index，只追加工作树修改，不自动提交/推送或切换环境。

- planner/coordinator：READY屏障保留。进入C后一方有本机引用即可接近，另一方可继续找C，不等双缓存；两端可同时执行。
  不申请资源，不发GRANT、DOOR_TRANSIT、EXIT。双方deposited+stopped且当前命令按身份结束才DONE，不置exited。
- agent/runner/go2_exploration/go2_check：删除活动RegionGuard/许可/固定退出链，普通路径与完整footprint检查统一保留。
  真实stop/action terminal、路径时效/启动等待、costmap、定位健康、失联停车和取消迟到隔离不变。
  停在C接近位置，没有固定退出/退让，也没有任务层两机互斥。deposited仍表示接近验收/模拟放入。
- config/mock/models/coordinator_node：不要求资源几何、区域stop_margin和退出点，消息保留旧空字段；EXIT/GRANT明确拒绝。
  旧工具类保留兼容，活动链不调用。不改源YAML、协议、桥路由、Nav2参数、0.81/1.00m、预算/扫描/重试/时限。

新增9项：6项独立C用例、1项DONE停稳/当前命令回归、2项独立检查入口无区域/旧区域兼容，均先红后绿。
扩展相关集合首次442过/16失败，失败是既有stub FakeAdapter.vision缺poll，补齐测试替身接口后通过；未为此改生产逻辑。
最终源码相关461项通过（既有452＋本轮9），2项真实初始化ROS的observer排除。独立只读复核发现DONE过早和检查入口残留，已修复；复核94项通过1项排除。
独立构建build/go2-independent-c-20261020/install/multi_agent_nav，仅1变化包；根包/消息复用原依赖，不能把构建条目数当用例数。
安装版同一集合461项通过（既有452＋本轮9），2项ROS observer排除。证据在build/go2-independent-c-20261020/evidence，含改前快照与用户暂存diff，不清理。
完整命令在[runbook](go2_runbook.md#2026-10-20-取消保护区与独立c执行最新)，Ranger同步清单在[最新交接](ranger_handoff.md#2026-10-20-独立c与取消保护区最新覆盖)。

默认CoVLM_env和一键入口仍旧安装；旧free-space输出不含本轮删除逻辑。真实scene.enabled和vision/path/peer验证仍关闭，未自动解锁。
本仓库仍无Ranger实际adapter；本轮fake不代表其已接入，实际适配须在Ranger控制电脑保留自身硬件栈完成。
没有初始化ROS、连接SSH/Ranger、启停桥、真实模型、额外采集/订阅/录包、ARM或真机运动，未触碰当前运动锁。

最终安装核对：从/tmp加载12个相关模块与源码摘要一致，7个源/安装配置与HEAD逐字节相同；
当前地图无家具仍有518个静态候选，单机离线门槛为空，协同仅剩scene.enabled与真实vision/path/peer验证缺项；rclpy未初始化。
证据为evidence/installed-audit.json、source-related.xml、installed-related.xml和build.log。
Notes/启动指引/VLM_Nav协同指引_1020.md同步独立C流程及新安装路径；没有运行其中的现场命令。

### 2026-10-21 静止通信启动前 KeyError('go2') 排障

本轮分支 `go2-cooperative-task-layer`，HEAD `4fc673156c5c985c4d9a9149cfe0b08b1582d90b`。
改前 alignment.yaml 已有用户修改，maps/lab 两文件已删除，协同环境/脚本及双机文档未跟踪；均保留。
已确认源配置 robots 下两个 ranger 键、无 go2；safe_load 后仅留下后一个空 Ranger 条目。
安装版 GeneratedCandidates 与 live_gates 均离线复现 KeyError('go2')，对应 G1/G2 预检及 G0 check，尚未到通信采样。
仅修正 alignment.yaml：按 robot_go2.yaml、实际地图摘要和 HEAD 恢复 Go2 身份；
用户填写的 Ranger ID/version 移至唯一 ranger 条目，保留 transform=null，未宣称远端身份已实测验证。
未改 Cooperative_env.bash、cooperative.sh、启动指引、DDS、验证标志或启动顺序；源配置由 COOP_CONFIG 直接读取，无需重建。
离线验证：YAML 无重复键、Go2 身份/摘要一致；报错中的 independent-c 安装生成 518 个候选；
在显式离线环境执行原脚本 transport 预检 Python 体通过，blocked_config 正常返回原 4 项 live gates。
源码既有 test_check_multi_agent_comm.py + test_live_gates.py 共 49 项通过，未新增用例；git diff --check 通过。
直接运行 go2 preflight 在本轮环境因 enp4s0 无 192.168.123.x 地址提前退出，未进入其 Python 预检；未改网络。
未初始化 ROS、SSH/连接 Ranger、启停桥/agent/硬件、录包、请求模型或运动；真实静止通信仍待现场按原流程复验。

### 2026-10-21 按双机文档离线审查 Go2 启动链

用户确认审查依据为 docs/双机协同启动与验收.md，Go2 已关机断连；禁止因连接失败改流程，不确定项留待审核。
交付：[审查与修改说明](go2_startup_review_20261021.md)、[待确认清单](go2_startup_pending_20261021.md)。
本轮未改公共启动脚本、环境、双机文档、DDS、协议或参数值；仅纠正 robot_go2.yaml 中“必须采集家具标注”的过时注释。
上一轮 alignment 修复保持，Ranger 身份仍是用户填写值而非本轮远端验证。
实际安装协同 27 个/底层 24 个 Python 文件及 8 个 launch/配置资源与源码一致；17 个 YAML 无重复键。
显式离线检查确认 518 个候选、4 项真实 gates 仍关闭，最终 costmap 动态层和 0.45m 膨胀正确；实际 agent venv 依赖可导入。
22 项入口替身断言通过；真实 bridge 配置解析器接受两侧路由，没有创建 ROS 节点。
既有回归 187 passed、1 failed；失败为 obstacle_chain_config 的测试注册白名单漏列两个已注册文件，CMake/测试与 HEAD 相同，未顺手修改。
证据保留 build/go2-startup-review-20261021；入口用替身网络探测和 ros2，检查器未实际采样，不能作真实通信通过证据。
待用户审核目标 B（水杯/纸巾）、远端地图/协议、4 项真实门控、相机示例/标定描述、报告复用和缺失配套资料，未擅自改值。
未初始化 ROS、SSH/连接任一机器人、运行传感器/相机/bridge/agent/coordinator、模型请求、录包、ARM、导航或运动；不提交、不构建、不部署。

### 2026-10-21 Go2 静止通信前硬件合并入口

用户明确要求把双机文档中的 Go2 硬件、预检、RViz 到静止通信前步骤合为一个脚本，授权本轮制作入口，未授权执行现场启动。
新增 `scripts/start_go2_cooperative_hardware.sh`；复用现有 `start_go2_vlm.py` 编排器，增加仅协同硬件准备模式，不复制整套启动逻辑。
主/组件终端均加载 Cooperative_env go2，沿用 COOP_CONFIG 和 COOP_SETUP_FILE；当前地图来自离线配置报告，不写死或借用 Ranger 地图。
相机沿用已成功的 go2_camera_ssh，原冷启动预检、底层、人工 AMCL 定位、RViz、Sport 与 inspect 检查保留；有效完整组件复用，异常/重复时停止。
默认传感器报告仍为 /tmp/go2_costmap_preflight.json，保留原 600 秒复用；本入口强制仅准备，Sport 须 DISARMED，不 ARM、不请求模型、不启动任务通信。
输出 HARDWARE_READY 后主终端退出，组件终端保持，操作者再按双机文档第5节继续。原单机模式保持，Ranger/公共协同脚本及参数未改。
双机文档 Go2 部分增加整合用法，逐终端方式保留；使用合并入口时不要再叠加执行手动整套命令。runbook 同步入口差异。
10 项新增测试先红后绿；最终相关 85 项通过（既有 75＋新增 10），含原单机行为及 PAUSED 检查器回归。
Shell 语法、无环境 --help、文档新增链接和 git diff --check 通过；证据为 build/go2-startup-review-20261021/hardware-launcher-tests.xml。
Go2 仍断连，本轮没有实际 SSH、ROS 初始化、硬件/组件启动、模型请求、ARM 或运动，也没有构建/部署/提交；实际桌面窗口和现场输入仍待操作者验证。

### 2026-10-21 按用户要求启用四项 Go2 live 配置开关

本轮21:00:05的Go2 agent日志记录四项live gates阻塞，真实adapter未激活；安装版离线复核得到同样四项，候选518个。
用户随后明确要求“全部改成true”：仅将源配置scene.yaml的enabled，以及robot_go2.yaml的
vision_validated、path_source_verified、peer_obstacle_verified设为true；保留其他已有改动。
这些值由用户要求设置，不代表本轮补做或通过了对应现场验证；此前历史验证缺项记录保留。
Cooperative_env默认COOP_CONFIG指向源配置，无需重建；已运行agent/coordinator不会自动重载，后续须按原流程重新启动才生效。
本轮不重启节点、不启动实验、不ARM或运动，不修改Ranger配置。
验证：使用independent-c安装版读取源配置，四项均为布尔true，Go2 live_gates=[]，仍生成518个候选；git diff --check通过。
未检查实际API环境、对端状态或coordinator服务，配置检查通过不代表整轮start或真实任务通过。

### 2026-10-21 READY 后 NAV_READY 超时：编排等待顺序修复

用户报告合并入口在 READY 后等待 /vlm_nav/nav_ready 的 true 消息 120 秒超时。
已确认脚本缺陷：原 Launcher.start 在 Sport 启动前等待 NAV_READY，而 go2_nav_supervisor 依赖 SYSTEM_READY，
go2_safety_supervisor 又要求 Sport 正常状态、真实接口就绪和新鲜心跳；无已有 Sport 的冷启动必然不能通过该等待。
原整合测试把 nav_ready 无条件模拟为 true，漏掉这个依赖；本轮补入依赖后4个冷启动参数场景先失败，复用场景不受此缺陷影响。
最小修复仅把既有 Sport 唯一性/启动或复用/状态检查块移至人工 READY 确认后、NAV_READY 等待前，公共编排器根因处修正。
不延长120秒、不改就绪门控、不ARM、不改Ranger及现场配置，保留用户刚设置的四个true标志；已有窗口/进程不自动结束。
修复后相关85项测试通过，Shell语法及git diff --check通过；证据 build/go2-startup-review-20261021/hardware-readiness-order-tests.xml。
双机文档和runbook追加依赖说明。未初始化ROS、启动/关闭现场组件或运动，未证明这是现场唯一阻塞原因；若仍超时应查System的NAV readiness failed项。

### 2026-10-21 live gates 齐全后的通信与任务入口

用户要求继续将双机文档第5节之后制作脚本，并指出当前验证标志齐全。
本轮安装版离线读取源配置确认 Go2 live_gates=[]、四项布尔true；保留用户开关和目标值，不重写成旧PAUSED配置。
判断：只跳过PAUSED专用采样/停流测试，不能跳过真实bridge、agent和新鲜状态核对。Ranger仍在本机按原流程人工启动，本轮未连接远端。
新增 scripts/start_go2_cooperative_task.sh/.py，复用原Launcher的环境终端/命令/ARM服务及cooperative.sh，不改公共协同脚本或协议。
prepare：live预检→Go2 bridge→明确ARM确认/原服务→真实agent→Go2侧两端IDLE核对→coordinator/watch；不自动start。
真实IDLE检查仅临时订阅两条高层state；复用原发布者/QoS检查，每端至少3条新鲜推进心跳，检查健康/停稳/地图身份且无旧任务或命令。
此检查不要求两电脑绝对时钟同步，也不证明Ranger已导入Go2状态；R0仍须原ranger peer核对反向链路。
start独立执行，在START确认后再次读两端IDLE，调用原start并输出accepted/task_id；pause不做live预检，disarm须人工STOPPED确认并核对服务/状态。
不自动reset/kill/重启旧节点，不自动处理Ranger或关闭组件；重复/不完整组件保留原窗口处理，启动结果不明不反复重发。
双机文档第5节改为live/PAUSED分支并补终端顺序与失败处理；原手动6～8节保留，runbook同步。硬件入口成功提示改为按当前gates选分支。
新增37项无硬件用例先因入口缺失而失败、实现后通过；合并既有85项最终122项通过，源配置/硬件/原PAUSED行为未改。
Shell语法、无环境help、Python语法、文档新链接及git diff --check通过；证据 build/go2-startup-review-20261021/cooperative-task-entry-tests.xml。
没有初始化真实ROS、SSH、启动通信/agent/coordinator、ARM、模型请求、录包或运动；只制作脚本与离线测试，不部署/构建/提交，现场仍待操作者执行。

### 2026-10-21 start 前置检查拒绝：拆分诊断信息

用户报告 cooperative-task.0m68qqez 在 live预检通过后被“唯一coordinator及ARMED”前置条件拒绝，未发送start请求。
该次目录仅有preflight.log，本机未见新入口prepare成功日志；最近Sport日志只记录初始DISARMED，不能据此确定报错当时的实时状态。
原判断使用短路or，无法从旧提示分辨coordinator数量异常还是Sport未ARMED；本轮只拆分诊断为实际数量和实际Sport状态。
保留原顺序和拒绝条件：coordinator数量不为1时仍不查询Sport，不自动prepare/ARM/reset或重新启动组件。
补4项诊断断言先红后绿，相关126项通过；证据 build/go2-startup-review-20261021/task-start-precheck-tests.xml，git diff --check通过。
未查询实时ROS图、ARM、启停组件或发送任务；HARDWARE_READY仅为硬件完成，需prepare得到COOPERATIVE_READY后才能start。

### 2026-10-21 按用户要求增加 Go2 完整自动入口

用户明确要求“一条命令执行，也不用我手动输入READY等”；本轮制作入口，运行入口时自动ARM及start，不执行现场实验。
新增 scripts/start_go2_cooperative.sh，加载原Cooperative_env后进入现有task编排的run动作。
新增AutomaticLauncher只替换确认词并串联原硬件start、prepare、start_task；原单机、硬件及分阶段入口的人工确认保留。
完整顺序：live预检→相机/传感器预检/底层/RViz/DISARMED Sport→真实NAV_READY/inspect→bridge→ARM→agent→两端新鲜IDLE→coordinator/watch→一次start。
CURRENT改为读回实际Sport参数dry_run=false、require_system_ready=true、fault_on_foreign_sport_request=false；不自动确认RESET。
NAV_READY、地图/健康/停稳/QoS/任务身份与原SYNC门槛保留；原硬件启动锁和task prepare锁同时持有。
当前live_gates=[]只跳过PAUSED专用采样，用户true标志和目标不改；不操作Ranger、不改公共cooperative.sh或协议。
原位姿仍由操作者在RViz按实车设置，SSH未免密仍需凭据；原生双向SYNC仍必需，不把Go2侧IDLE当Ranger反向链路验证。
重复/旧任务组件、FAULT及失败停止，不reset、清锁、结束旧进程或重发start；退出主入口不暂停已发起任务，暂停/停稳/关闭沿用原流程。
双机文档开头改为完整一条命令及判据，人工详细流程保留；runbook同步，Ranger原启动流程未改。
新增14项替身测试（既有126项），相关140项通过；证据build/go2-startup-review-20261021/cooperative-full-entry-tests.xml。
没有初始化真实ROS、SSH、启停硬件/bridge/agent/coordinator、模型请求、ARM、录包或运动；未构建、部署或提交，实际桌面/现场结果待验证。

### 2026-10-22 双机联动前必要检查

用户确认两端已完成迁移，本轮只授权必要检查、已证实问题的最小修复和针对性离线测试；不启动运动、不自动提交。
分支 go2-cooperative-task-layer，HEAD 4fc673156c5c985c4d9a9149cfe0b08b1582d90b，保留已有全部未提交改动。
完整结论、Ranger清单、静止通信/最小联动顺序及构建待办见[runbook本轮节](go2_runbook.md#2026-10-22-双机联动前必要检查)。

生产代码仅修改公共agent.py：旧导航成功进入ARRIVAL_STOP，实际停稳、动作终态、当前目标导航类型匹配、距离≤1m后才READY/deposited；
私有意图未接本地执行器直接拒绝，删除回退旧坐标导航的路径。现有私有执行核心、探索、VLM、Nav2、定位及参数未改。
独立C、双READY、CACHED只报自身引用、另一台继续搜索、双方当前命令终态且停稳才DONE均有离线证据。
资源许可/门/basket/EXIT已按当前版本停用，没有恢复旧机制或增加协议。心跳/握手/重发/取消及失联停车逻辑保持。

基线231通过/5失败，最终266通过/同5失败，新增覆盖35项全部通过（新文件29、Ranger参数化6）；
5项均依赖旧live gates关闭假设，逐项比较无新增失败，未修改用户已启用的true值或无关测试。
旧路径回归先26失败/2通过，再修复后关联86项通过；独立只读代码复核无新增问题。
Shell/Python语法及git diff --check通过。证据build/dual-preacceptance-20261022含XML、日志、test-comparison.json、installed-audit.json，不清理。

默认Cooperative_env仍加载go2-independent-c-20261020安装，其agent.py与HEAD一致、与修复后源码不一致；现有preflight会拒绝。
未构建/部署/切换默认环境；验收前须按runbook新输出构建并在各终端显式COOP_SETUP_FILE，核对matches_source及进程版本。
用户所述Ranger迁移尚未由本轮访问实码验证，本地Ranger占位分支不代表远端实现；需在其本机同步修复并完成协议、严格到达及实际安装核对。
Go2当前live_gates=[]是配置事实，不是新增现场证据。现场仍须核对B=水杯、双向状态、真实目标距离/停稳、失联停车和C处局部避障。
本轮未初始化ROS、SSH/连接机器人、启动bridge/agent/coordinator/硬件、模型请求、录包、ARM或运动，未自动提交。

### 2026-10-22 按用户要求构建更新 Go2 独立安装

用户报告完整入口被源码/安装一致性检查拒绝，静态复核六个模块仅agent不匹配；旧默认安装仍为independent-c-20261020。
用户随后明确要求“进行更新并给出脚本命令”。按runbook独立输出构建multi_agent_nav，实际输出build/go2-dual-acceptance-20261022，退出0。
复用go2-coop消息及manual-takeover根包；未覆盖旧安装，未改公共默认overlay、启动脚本、协议或Ranger流程。
显式COOP_SETUP_FILE选择新安装，执行原preflight退出0，六模块matches_source全true，live_gates=[]、API环境齐全、候选518、motion_sent=false。
实际任务入口解释器为/home/isee-pst/venv/co-nav-real/bin/python；更新验证不初始化ROS。
新安装相关离线回归174passed/1failed，唯一失败为既有test_independent_c.py::test_no_gate_requires_resource_geometry_or_stopping_envelope_margin，
与此前五项基线失败之一相同，依赖已过时的vision_validated关闭假设，未修改用户配置或无关断言。
证据在新输出log及evidence目录，包含preflight.log、installed-preflight-summary.json、installed-targeted-tests.xml。
runbook及双机文档的完整启动命令显式带新COOP_SETUP_FILE；父/子终端继承，运行时仍会自动ARM并发起真实任务。
旧进程不会自动更新，不自动结束旧进程或重试任务；实际停稳/关闭、RViz真实位姿和SSH凭据沿用原要求。
本轮未启动完整入口、SSH、硬件/bridge/agent/coordinator、模型请求、ARM、录包或运动，未提交；现场与Ranger端版本仍待操作者验证。

### 2026-10-22 按用户要求等待 Ranger 启动而不因15秒退出

用户要求任务准备时持续等待Ranger消息，避免对端尚在启动就结束检查。
仅修改scripts/start_go2_cooperative_task.py共享wait_idle默认timeout=None，每5秒显示等待原因；prepare/start均沿用，显式有限timeout仍供原测试使用。
原两端各3条新鲜推进心跳、健康IDLE/停稳/身份及发布来源/QoS条件保留，不以一条或不健康消息放行。
保持同一临时高层订阅，不重启原组件、重发start、reset或清除PAUSED；原agent/coordinator运行后失联停车逻辑未改。
Ctrl+C可退出并销毁临时检查节点，保留已有窗口/进程；已ARM不自动DISARM，旧运行实例不热更新、旧退出实例不自动恢复。
入口命令和新overlay不变，无需构建，未改Ranger流程、公共cooperative.sh、配置或协议。
新增3项无硬件用例先红后绿，验证20秒后对端出现/变健康以及持续等待中断清理；相关143项通过。
证据build/go2-dual-acceptance-20261022/evidence/startup-idle-wait-tests.xml，双机文档与runbook同步等待语义。
没有初始化真实ROS、SSH、启停组件、ARM、API请求、录包或运动；未构建/部署/提交，真实现场等待与状态恢复仍待操作者验证。

### 2026-10-22 现场只读定位及授权重启 Sport/Go2 agent

用户在等待Ranger阶段先要求重启agent，随后取消并说明已重新执行完整入口；未执行该次agent重启。
主机进程检查须用授权的主机执行环境，沙箱PID空间只可见自身；只读ROS采样使用运行实例环境，仅订阅高层state/就绪Bool和已有诊断，不新增RGB-D或完整点云观察器。
现场11:04采样确认两端各持续收到6条state，Go2/Ranger均停稳、无task/command/goal、均PAUSED；Go2原生NAV_READY/SYSTEM_READY=true，但control_armed=false、Sport=DISARMED、agent nav_ready=false。
桥日志10:58:35记录external Sport control took over; DISARMED until explicit ARM；用户明确确认外部Sport请求来自其遥控。
证据build/go2-field/readiness-diagnosis.msalocdo/live-status.json及bridge-transitions.log；不能把原生NAV_READY与agent汇总nav_ready混为一项。

用户随后明确要求重启Sport及后续依赖。确认无coordinator/start进程、state停稳且无任务后，暂挂原等待编排PID18920，
向旧Go2 agent PID20741、旧Sport PID19804分别发送SIGINT并等待正常退出，未强杀；按原环境/配置在新桌面终端启动唯一Sport。
确认新Sport=DISARMED及NAV_READY=true后，通过原arm服务恢复ARMED/control_armed=true，再用原cooperative.sh go2 agent启动新agent。
最终连续至少3条新鲜Go2状态为IDLE/localization_valid=true/nav_ready=true/stopped=true、reason及任务身份均为空，发布者唯一。
原等待编排已SIGCONT恢复，未人工发送coordinator/start请求；旧组件终端保留退出提示，新Sport/agent窗口保持运行。
未重启相机/底层/定位/Nav2/任务bridge，也未连接或操作Ranger；不改生产代码、配置、门槛或协议，不构建/提交。
操作和前后状态证据build/go2-field/sport-agent-restart.p0cm6fo_/operations.json、arm.log；本轮实际进行了授权ARM及组件重启，不是纯离线测试。
剩余：采样时Ranger为PAUSED/nav_ready=true/stopped=true、reason=peer localization/navigation invalid，当前Go2继续等待其真实IDLE；
未自动解除Ranger暂停、伪造状态或绕过IDLE检查，需Ranger本机按原恢复流程处理后再核对两端状态。

### 2026-10-22 任务前 PAUSED 锁存恢复及原完整入口的一轮结果

用户随后报告Go2仍PAUSED。主机只读采样确认Ranger已恢复IDLE/nav_ready=true，Go2也nav_ready=true但PAUSED，Sport保持ARMED，两端停稳且task/command/goal为空。
已确认触发机制：agent_node.peer在task_id仍为空时也会因对端localization/nav无效调用stop；后续健康消息只刷新peer_received，不解除PAUSED。
因此一端重启时发布的短暂初始化不健康可能暂停另一端；本轮没有修改该生产逻辑、门槛或自动清除暂停。
沿用用户本轮“重启Sport和后续依赖”授权，在无任务/无coordinator且停稳前提下暂挂原等待编排，先SIGINT正常结束Go2任务bridge，再正常结束并启动Go2 agent。
待Go2自身连续新鲜健康IDLE后，按原入口恢复Go2任务bridge；核对两端各至少3条健康IDLE、正确唯一发布者和空任务身份后恢复等待编排。
Sport/硬件/Ranger均未重启，无额外ARM或人工start；此恢复仅用于本次无任务准备阶段，不能用于任务运行中屏蔽失联。
操作证据build/go2-field/startup-agent-recovery.ob5at07b/operations.json，恢复前后状态均保留。

原完整入口随后自动创建coordinator/watch并成功调用其原本一次start：accepted=True，task_id=d90f753459334fdca0b2c778780a6d87，日志build/go2-field/cooperative-task.3l2x2wq0/start.log。
本轮Go2实际到达选中候选，events记录ARRIVED/navigation_status=SUCCEEDED/stop_confirmed=true，随后进入SCANNING并observation_failed。
最新只读状态：两端均PAUSED/stopped=true/goal_id为空；Go2原因local execution failed: scan failed，Ranger原因coordinator pause/cancel；
coordinator为PAUSED，reason=go2: health/heartbeat lost。扫描失败的具体底层门槛尚未定位，不把此轮记为完整任务通过。
证据为上述恢复目录progress.json及build/go2-vlm-experiments/d90f753459334fdca0b2c778780a6d87_go2_1/events.jsonl。
没有额外重发start、重试实验、自动恢复旧任务或DISARM；原本一轮已有真实导航/VLM活动，后续新实验须另有相应授权。

### 2026-10-22 11:20 新一轮等待中的状态新鲜度核对

用户表示自行重新启动后见Ranger PAUSED，询问是否旧状态。本轮仅只读核对，不重启、不ARM、不start。
11:20:12连续4秒采样：Go2收到8条、Ranger收到9条，双方stamp严格推进，末条接收年龄分别约0.38s/0.10s，发布者各唯一且分别为本机agent/导入桥。
采样末态Ranger已IDLE/nav_ready=true/localization_valid=true/stopped=true、reason及任务身份为空；Go2仍PAUSED但nav_ready=true/stopped=true，reason=peer localization/navigation invalid，任务身份为空。
因此当前不是通信缓存旧包；新的心跳可携带agent此前锁存的PAUSED原因，且用户贴出状态与采样时刻可能不同。当前阻塞在Go2暂停锁存，不能据旧行继续要求重启Ranger。
证据build/go2-field/peer-paused-freshness.0vasa1ip/states.json；未改生产代码/配置/流程，未解除暂停或发起新实验。

### 2026-10-22 用户新一轮完整启动的 Go2 PAUSED 恢复

用户表示Ranger已正常，但当前自行重启的完整入口仍因Go2 pre-task PAUSED等待。沿用本轮用户授权的Go2依赖恢复，未修改协议/门槛或生产代码。
核对新一轮等待PID23489、Go2 agent PID25313、任务bridge PID25039，无coordinator；新鲜状态为Ranger健康IDLE、Go2健康但PAUSED，双方停稳且task/command/goal均为空，Sport=ARMED。
暂挂等待编排，正常SIGINT结束Go2任务bridge和Go2 agent；用同一已验证overlay/config重启agent，待本地连续健康IDLE后恢复任务bridge。
重新核对两端各至少3条推进的新鲜健康IDLE、正确唯一发布者、空reason和任务身份后SIGCONT恢复原等待脚本。
未重启Sport/硬件/Ranger、未新增ARM、未人工发送start；证据build/go2-field/startup-agent-recovery.dtycorel/operations.json。
原完整脚本随后自行受理本轮start，accepted=True，task_id=60483e15cceb426abc8b911265d309d8，日志build/go2-field/cooperative-task.fynm1l1k/start.log。
本轮完整任务结果尚未验证，不重复start或追加实验。启动前peer invalid可锁存PAUSED的生产逻辑尚未修改，交替重启仍可能复现；本次为无任务时的受控恢复，不是永久修复或运行中失联保护的替代。

### 2026-10-22 11:26 只读分析 task 60483 的运行期暂停

用户要求分析watch所示60483e15cceb426abc8b911265d309d8、PAUSED、ranger: health/heartbeat lost；本轮只读采样及日志分析，不恢复/重启/ARM/start或重试实验。
证据build/go2-field/task-60483-analysis.ksjtxb0w/live-status.json、analysis.json。
11:26:04结束的8秒采样，两端各80条心跳，stamp严格推进，最大接收间隔Go2约0.367s、Ranger约0.415s；两条state发布者各唯一、Reliable/Volatile、simulation=false。
两端当前localization_valid/nav_ready均true，run_state=PAUSED、stopped=true、goal_id为空、READY/deposited=false；原生NAV_READY/SYSTEM_READY=true，Sport仍ARMED/control_armed=true。
Go2理由peer localization/navigation invalid、command_id为本任务/go2/1、reply=CANCELED；Ranger理由coordinator pause/cancel、command_id为空、sequence=0、reply=CANCELED。
coordinator仍为原任务PAUSED，reason=ranger: health/heartbeat lost，online_robots两端均在，local_c为空。

源码coordinator.tick同一摘要覆盖：无状态、接收超lost_timeout、健康无效、或同任务run_state=STOPPING/PAUSED；进入PAUSED后不自动更新为恢复状态。
因此当前摘要不是当前断网的证明；只读窗口未见失联，不可反推首次触发时无短时失联。Go2原因对应peer处理曾收到Ranger定位或nav无效状态，但具体首次是哪项、底层原因及是否伴随网络间断，需要Ranger本机历史证据。
本轮本地events只有CANDIDATE_GEOMETRY→CANDIDATE_REQUEST→CANDIDATE_SELECTION_FAILED，无CANDIDATE_SELECTED/ARRIVED/SCANNING，不能引用上一轮d90f的到达/扫描记录作为本轮成果。
本轮模型metadata确认api_returned=true、validation_success=true、error_type=null、耗时约4.21秒，返回候选P6；候选流程失败不是模型调用失败的充分证据。
start.log文件记录11:23:25受理，events最后记录约11:23:29，模型metadata约11:23:30完成；结合生成器取消路径，更符合暂停先取消选择、模型有效结果随后返回的情形，此因果细节属于推断。
当前整轮未完成A/B接近、未进入C、非PASS；两端状态停稳不替代本机原生动作终态/现场停车证据。Sport仍ARMED，未擅自DISARM。
下一步需核对Ranger本机11:23:25～11:23:30对应任务的agent及定位/Nav2健康转换日志，先定位触发项，不以重启或提高失联阈值代替排障。

## 2026-10-22 异步准备与任务握手源码实施（未部署）

用户提供五阶段状态机及四项约束，授权进行公共任务逻辑/Go2入口的必要修改和离线验证；本轮没有构建、更新现场安装、SSH/ROS/bridge/ARM/start或运动。
分支go2-cooperative-task-layer，HEAD 4fc673156c5c985c4d9a9149cfe0b08b1582d90b；原有用户修改、配置标志及maps/lab删除保留。

当前私有坐标模式本机独立准备，健康停稳即可WAITING_PEER；双方等待状态可相互确认STANDBY。心跳接收与健康分离，初始化/单端重启不锁存PAUSED。显式start后SYNCING，接受SYNC立即发布task_id；双方本轮干净SYNCED后派工，保持严格A/B READY及私有C语义。同步等待受600秒总任务上限约束，执行期3秒门槛、异常停车和PAUSED锁存不变；共享坐标旧模式保持原启动逻辑。
候选几何、地图摘要、选择图渲染复用现有worker并yield，模型原异步路径保持。独立审查发现不对称首条EXECUTE竞态、同步地图摘要阻塞，以及后台等待后的运动复核缺口，均用先红后绿回归修复；导航前健康/路径/停稳/终态/footprint检查位于最后等待之后。

最终离线集合465通过/5项原有失败（基线269通过/同5失败，XML逐项相同）；新增启动握手35项、受控阻塞8项及最终地图等待后运动复核3项通过。证据build/async-startup-20261022/，包括源码快照source-before、baseline.xml/final.xml与红绿日志；不清理。
详情及Ranger待核对清单：[异步启动修改说明](async_startup_changes_20261022.md)。本机此前build/go2-dual-acceptance-20261022/install未更新；源码与安装不一致预检拒绝是预期，不得绕过。两端须同步更新实际安装和自定义状态映射后，另按现场授权验证；不能称为已远端迁移或现场通过。完整构建命令仍只在runbook维护。

## 2026-10-22 依据 Ranger 最终契约补全 Go2 并隔离构建

用户指定 docs/multi_agent/ans_async_startup.md 为接口/参数依据，并说明Ranger已依契约修改；本轮未访问对端核验。
分支go2-cooperative-task-layer，HEAD仍4fc673156c5c985c4d9a9149cfe0b08b1582d90b，用户原有修改、真实验证标志、地图删除保留，没有提交。

Go2已补两种正式SYNC编码和脏字段验证，仅phase=SYNCING可以建立新task；SYNCED必须同时run_state/reply=SYNCED，coordinator只接受规范ACK，先SEARCH_AB同步后派工。历史ACK与当前就绪分开，保持乱序/重复/首条丢失保护；提交后未收到首EXECUTE也严格执行期监护。固定预期peer地图身份并报告错schema/模式/地图，分别保留传输和当前task接收时间。旧worker未结束时新工作yield等待，可取消；最终运动检查不变。
独立审查发现旧PAUSED缓存误取消新任务，补回归修复；新coordinator可在操作者显式start下替换停稳PAUSED旧任务，agent仍本地确认动作终态。旧共享坐标模式初始宽限保留。

参数heartbeat_hz=2.0、lost_timeout=3.0、task_timeout=600.0和双方地图身份原已匹配契约，无需猜测修改。新隔离安装build/go2-async-contract-20261022/install/local_setup.bash，colcon退出0，27模块逐字节一致，三消息摘要契约一致；默认与显式preflight退出0，两种SYNC=true、live_gates=[]、API齐全、候选518、motion_sent=false。
Cooperative_env.bash的Go2默认已选该新安装，显式COOP_SETUP_FILE覆盖仍有效；Ranger分支未改。旧安装未覆盖，旧运行进程不会自动更新，已有终端的旧COOP_SETUP_FILE应在后续正常退出/启动时替换。

基线465通过/5失败；实际新安装相关回归551通过/同5失败，新增86项全部通过（契约72、入口10、worker2、默认/覆盖安装2）。基线5项均为旧关闭gates假设，未改真实配置转绿。证据build/async-contract-20261022/，实际安装XML build/go2-async-contract-20261022/evidence/installed-tests.xml。修改/未验收项见[本轮说明](go2_async_contract_changes_20261022.md)，构建命令仅在runbook维护。

本轮只构建和离线检查，未初始化真实ROS、SSH、连接Ranger、启动任何硬件/bridge/agent/coordinator、ARM/start/录包或运动；没有恢复此前暂停任务。下一步现场必须按原流程先确认停车/动作终态、退出旧任务节点再加载最终版本，不能把本轮离线结果当跨机静止/运动验收通过。

## 2026-10-22 启动流程文档与脚本整理

用户要求更新启动文档、制作启动脚本。沿用现有start_go2_cooperative.sh作为正式完整入口，补齐四阶段、状态/双ACK/计时判据、入口选择、故障后续和关闭顺序，未再造另一套编排或更改硬件顺序。分步prepare/start仍人工确认，完整入口不输入READY/ARM/START等确认词，保持只发一次任务。
发现必要修复：Cooperative_env.bash加载默认新安装时没有导出COOP_SETUP_FILE，后续硬件检查可能回退到旧安装期望。Go2分支现成功加载后导出实际路径，默认/显式覆盖均传到子进程；Ranger分支不改，现有隔离安装无需重建。
证据build/startup-docs-20261022/environment-red.log（默认路径子进程为空）、environment-green.log；test_cooperative_environment/test_start_go2_cooperative_task/test_start_go2_vlm合计110通过，18段文档Bash语法、本地链接和diff --check通过。没有执行文档中的启动命令；未构建/ROS初始化/SSH/硬件/bridge/ARM/start/运动，也未提交或覆盖旧工作。
正式入口和每步继续判据见[启动与验收](../双机协同启动与验收.md)。本轮仅入口/环境传递修复与文档，不把上一轮551/5结果当作本次重新全量验证。

## 2026-10-22 15:37 冷启动传感器预检失败只读分析

用户贴出run_go2_preflight.sh退出1。本轮只读现有报告/源码/日志，未重跑预检、初始化ROS、SSH、重启组件、ARM/start或运动，未修改代码/阈值/启动顺序。
最近完整入口日志build/go2-field/cooperative-task.u0sstt0k仅有preflight.log；/tmp/go2_costmap_preflight.json生成15:37:35，healthy=false、imu.healthy=false。采样300帧雷达、10.0028Hz、cloud_errors=[]、14967个IMU样本；IMU重力范围超限42、角速度模超限33、极值超限6、加速度跳变18、无NaN。
重力模8.2909～10.998（门槛9.0～10.3 m/s²）、角速度模最大0.177915（门槛0.09 rad/s）、角速度单轴最大0.173638（门槛0.11 rad/s）、加速度跳变最大1.7690（门槛0.5 m/s²）。formal_preflight=PASS不覆盖healthy=false；采样器据healthy返回1，父脚本按set-e退出，是门槛真实拒绝，不是Ranger等待或已ARM的证据。
失败在Launcher.start冷启动临时传感器采样，System/Sport和后续ARM/任务步骤尚未由本次入口执行；末尾ARM/start不明提示为通用失败兜底，不能当作本次已ARM。没有读取当前桥状态，不反推整机当前使能状态。
静止期间是否运动/调整姿态/振动或原始IMU异常尚不确定，汇总不足以判传感器硬件故障；未放宽阈值。临时目录/tmp/go2_preflight.ctL8ny日志见收到退出信号，但未另行检查现场进程。
证据快照：build/go2-field/sensor-preflight-failure.p1ti5cjl/（report.json、thresholds.yaml、临时sensor日志和analysis.json），避免/tmp报告后续被覆盖。

## 2026-10-22 task 997eb API连接异常只读排查

用户报告本轮watch PAUSED、go2 agent local execution failed: vision...。本轮只读取本任务文件/源码和agent必要环境，检查模型服务DNS/TCP/TLS及一次同venv/httpx无凭据GET；未推理模型、重发任务、ROS初始化、SSH或机器人操作，没有改代码/阈值/启动流程。
任务997eb520a32d4e1db5647f54fac1a638，start日志build/go2-field/cooperative-task.ao9pv5rg/start.log显示accepted=True；准备快照两端STANDBY、定位/Nav2/stopped均true，预检安装一致/gates=[]/两种SYNC=true。
决策9590155fc4b64cc09408b7cabb9c7554的metadata记录qwen3-vl-plus、api_returned=false、error_type/provider_error_type=APIConnectionError、http_status/request_id=null、latency_s=0.15117、parsed_selection=null。只有CANDIDATE_GEOMETRY→CANDIDATE_REQUEST→CANDIDATE_SELECTION_FAILED，motion_sent=false，无CANDIDATE_SELECTED/NAVIGATING/ARRIVED/SCANNING；不是20秒超时或模型回答格式不合格。
异常由AsyncVision.poll统一脱敏成vision request/validation failed，runner锁存暂停，coordinator保留go2 agent暂停首因。原始网络异常链未落盘，不能断言具体DNS/TLS/连接重置，也不能把此前60483有效响应但后续取消的情况套用本轮。
当前从运行agent的配置查出endpoint/model完整、key存在且无外侧空白/控制字符，未设置代理；ABCClient明确trust_env=False。DNS/TCP/系统TLS均通过；同co-nav-real venv httpx对base路径做无凭据GET返回HTTP404，仅证明当前HTTP传输可达，不证明真实鉴权模型请求恢复。没有打印/写入API密钥，没有发送凭据或图像。
完整根因仍需当时底层异常链或后续独立模型链路诊断，未猜测改网络/重试/门槛。证据：build/go2-field/task-997eb-analysis.zf0cgqgb/analysis.json及metadata.json，原事件/输入仍在build/go2-vlm-experiments/997eb520a32d4e1db5647f54fac1a638_go2_1/。没有采样当前ROS状态或Sport使能状态，不能宣称当前实体停车/解除使能已验收。

## 2026-10-22 最近一轮 520b 探索与缺少 views 的重新核对

用户确认核对最近一次探索，报告实际持续运动、曾原地转向且目标在眼前未停。本轮只读本机历史日志、源码/安装及进程元数据，不连接Ranger，不重发任务/调用模型/启动硬件或运动。
任务520b4506478b4f6d926269244e3a9529：选点模型约4.109秒有效返回P3；15:48:25.720开始向(7.435,-15.275,0)导航，15:49:45.740取消，持续80.021秒。
controller有77次Passing new path、4次ObstacleFootprint轨迹拒绝；15:49:02全局规划失败并清除global costmap，但上述日志不足以确定持续不到点的完整根因。
本轮无导航成功、ARRIVED、SCANNING或behavior_server Running spin，也无input.jpg/views。缺少事件不等于机器人没有运动，不能把原地转向直接认定为视觉扫描。
源码与实际overlay六个相关模块一致：私有任务只在观察点导航成功、停稳和帧就绪后调用相机ABC；observation_only不运行原单机定时识别。因此本轮没有进入相机目标识别路径，沿途目标出现不能触发视觉停车；不能把地图选点成功视作相机识别通过或把本轮判为模型漏检。
同任务15:52保存状态为PAUSED/local navigation not ready、agent nav_ready=false，原生/vlm_nav/nav_ready=true。正式AgentRunner/agent_node没有绑定adapter.record_event，缺少运动/健康分项事件，具体失效子门槛仍待验证。
计划的6秒低流量元数据订阅尚未启动，原agent PID11440已退出；后续本机进程只读核对未见agent/Nav2/Sport，不据此宣称实体停车或解除使能已验收，未重启补采。
更新报告与原日志副本：build/go2-field/task-520b-vision-recheck.o7uaxr5q/report.json；报告脚本断言确认80秒、无扫描/图像和源码安装一致。未修改生产代码/参数或执行ROS测试，不混用上午d90f扫描Spin失败或997eb网络异常的证据。

### task 520b 排查报告与待审核项

报告及待确认修改方向已整理至[本轮诊断](go2_task_520b_diagnosis_20261022.md)。离线控制导航Future的80秒验证确认：到点前不调用scan，到点成功后才进入识别。用户观察到转向不能替代本轮SCANNING/相机识别证据；当前路径不支持沿途目标触发停车。
历史低流量采样文件仍在build/go2-field/task-520b-analysis.75gaybmd/，与上述后来因agent退出而取消的新采样计划区分；彩色CameraInfo两次无消息只是待核实线索，不能断言原始RGB停止或首次暂停首因。15:55:48外部Sport接管/DISARM晚于15:49:45导航取消，不混作前因。未据不确定事项修改代码、参数或启动流程。

## 2026-10-22 人工23顶点 Keepout 掩膜草稿（未接入）

用户提供build/go2-field/keepout-boundary-points/clicked_points_20261022T083443.986516Z_133838bc.jsonl，明确采用编号1至23。
23点均为map有限坐标，顺时针、无自交、全部在原图范围内；使用原始XY顺序并闭合23→1，不移动顶点，原始XYZ记录保留。
独立产物位于maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261022_133838bc/：boundary.yaml、原始点副本、keepout_mask.pgm/png/yaml、boundary_preview.png、validation.json。
掩膜保持472×578、0.05m、origin=[-8.09,-20.8,0]；白255加载为0（不额外限制），黑0加载为100（禁行）。边界内原有墙体及未知语义不由此清除。
连续多边形面积108.53218087035569m²；栅格化保守禁止跨界格（中心距边界需大于半格对角线约0.03536m），未加整机尺寸缓冲。允许42383格，禁行230433格，掩膜无未知值。
离线核对源/点记录摘要未变、原编号/闭合顺序、矩形及凹多边形例、顺逆序一致、独立点内判定及允许格四角、PGM/PNG往返、trinary加载分类；证据脚本build/go2-field/keepout-mask-build.9s5m8q7x/generate.py。
与旧决策24be031b664344359ce76214fcc56922比较：P1中心内距边界约0.09071m，P5中心在外约0.03410m；按已有约0.42048m整机旋转范围检查，旧12点仅P10通过此掩膜，尚未叠加动态障碍或重新生成候选。P编号仅属于旧决策。
待核对人工边界预览，尤其P1/P5附近；未为保留旧点而移动边界或放宽footprint。未修改原地图/YAML、生产源码、Nav2配置或安装，未初始化ROS/启动mask server、部署Keepout、ARM/start或运动。
此前原图只读统计为灰度0=4914格、205=210517格、254=57385格；活动lab.yaml的free_thresh=0.25会把205判自由，离线以0.196重分类则旧12点仅P1/P5/P10通过原图旋转检查。该阈值尚未修改，Keepout草稿不代表原图分类问题已修复。

### 用户作废23点并重新采集

用户明确指出点选错误并要求清除重来，以上23点及对应Keepout草稿已作废，不可部署。原点记录和整个keepout_boundary_20261022_133838bc目录已移至build/go2-field/keepout-boundary-discarded.xi810c3t/，共8个文件摘要逐一确认不变，archive_manifest.json记录原路径、备份路径及作废原因。
build/go2-field/keepout-boundary-points/现为空，原maps下的草稿目录已移出；原地图/录点脚本/导航配置未改。只读进程核对未见旧录点程序、地图server或RViz；本轮没有代启新录点或ROS节点，等待用户按同一Domain31离线流程从编号1重新采集。

### 第二轮23点 Keepout 草稿（29ab5c38，未接入）

用户重新提供build/go2-field/keepout-boundary-points/clicked_points_20261022T084824.581164Z_29ab5c38.jsonl，共23有效点；沿用完整编号1→23→1，无坐标调整。旧133838bc继续作废，不恢复。
新产物maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261022_29ab5c38/含掩膜PGM/PNG/YAML、boundary.yaml、原点副本、boundary_preview.png和validation.json。
多边形顺时针、无自交/越界、面积108.88009757245527m²。掩膜472×578、0.05m、origin=[-8.09,-20.8,0]；白255不额外限制42497格，黑0禁行230319格，保留上一轮的保守跨界栅格规则，无额外整机缓冲。
旧决策24be031b664344359ce76214fcc56922的12点中，P1/P5/P10通过此掩膜的约0.42048m整机旋转检查，其余9点在多边形外；这不是动态costmap、实际路径或现场可达性验收。
复用上一轮离线生成脚本，仅替换新记录和输出路径：build/go2-field/keepout-mask-build.636gw137/generate.py。几何/独立点内判定、所有允许格四角、顺逆序一致、图像往返及trinary分类检查通过；源地图/新记录与各产物摘要、原始1至23的XY顺序再次核实一致。
当前仍是待核对预览的独立草稿，未修改原地图/分类阈值、生产源码或Nav2配置/安装，未初始化ROS、部署Keepout或启动运动；现场边界与接入效果待后续核对。

## 2026-10-22 第二轮边界已接入Nav2（隔离验证，未现场启用）

用户随后确认预览并明确要求“接入nav2”。采用29ab5c38的23顶点掩膜，第一轮133838bc继续作废。robot_go2.yaml新增keepout_mask；协同硬件入口将其传入System/Navigation，两个原生mask/info服务先激活，全局/局部costmap配置KeepoutFilter。保留原地图、阈值、动态层、Ranger及空掩膜旧单机行为。
候选流程原本已在渲染/上传前用全局costmap检查停车、整机旋转和路径，本次直接使该检查覆盖禁行区。白色掩膜仅不额外限制场内，未把全部未知格改为障碍。
原生KeepoutFilter缺少掩膜时仅告警，因此增加独立就绪证据：supervisor要求两服务ACTIVE、对齐/新鲜的全局完整成本图且全部场外格为254；adapter复用既有全局costmap订阅进行同样检查，包括只读inspect/scan模式。缺失/旧header/迟到后静默过期/未应用/坐标不符均拒绝就绪。复用旧底层还核对supervisor的mask参数，不自动重启。

新成对安装build/go2-keepout-nav2-20261022/install/local_setup.bash，vlm_nav和multi_agent_nav构建退出0，Python树/launch/config共56文件逐字节一致。Cooperative_env.bash的Go2默认及硬件期望切到新安装，显式覆盖仍有效；preflight增加实际根包Keepout和adapter字节匹配，并排除工作目录源码遮蔽，不为检查adapter而导入系统Python缺少的SDK。原有overlay未覆盖。
构建、环境及用户执行顺序仅在[runbook最新Keepout节](go2_runbook.md#2026-10-22-人工边界-keepout-接入nav2)维护；[正式启动文档](../双机协同启动与验收.md)中的Go2命令已统一新安装。旧终端的COOP_SETUP_FILE、旧System/Nav2及任务进程不会热更新；需按原关闭顺序正常退出后，在后续有现场授权的一轮重新准备。

证据build/keepout-nav2-20261022/evidence/：新安装专项16通过（15判断/启动＋1真实Nav2），扩大相关回归288通过/3失败。旧async安装baseline-three.xml独立复现同3失败，均为测试假设现有vision/path/costmap认证标志关闭；未改真实配置凑通过。不是全库测试，也不把历史551/5当本轮结果。
真实Nav2测试只用localhost Domain31的合成全自由底图、TF和稀疏点云，无运动服务：全部230319个全局场外格为254、局部场外采样254；旧P1/P5/P10未禁行，P3为254；动态标记/清除和4个测试子进程正常退出均通过。单射线在不同栅格origin上取整留下邻格，测试改窄扇形合成清除，不改生产参数；P10有合成障碍膨胀成本，未要求所有场内格恒为0。
审查发现并修复只读模式漏订阅global、header新鲜度遗漏及迟到年龄预算，均有先红后绿证据。默认/显式实际preflight均退出0，keepout路径/472×578正确、源码匹配、SYNC两项true、live_gates=[]、API环境完整、motion_sent=false；只读预检未发送API请求。

本轮没有SSH、连接Ranger、初始化现场ROS、启动硬件/bridge/agent/coordinator、ARM/start、录包或运动，没有恢复任务；人工边界、定位误差和实车边界避让仍待现场验收。仅启动了隔离Domain31合成测试ROS，不能写成“未初始化任何ROS”。没有自动提交或清理用户改动/日志。

### 用户要求“进行更新”后的生效核对

再次核对默认Cooperative_env、硬件/完整入口及实际离线预检，确认已选择go2-keepout-nav2-20261022成对安装，源码匹配、掩膜为29ab5c38、live_gates=[]、motion_sent=false；新证据为build/keepout-nav2-20261022/evidence/preflight-update.log。没有重复重建已验证安装。
本机/proc只读核对未发现System/Navigation launch、Nav2服务器、mask/info服务、supervisor、协同agent/coordinator、Sport转换、RViz或FAST-LIO/雷达进程，因此没有可原位切换的现有Nav2；下一次由正式协同入口准备时加载新安装。未启动一轮新的硬件准备或运动实验，未SSH或初始化现场ROS；本机进程缺失不构成实体停稳/DISARM的验收证据。

### 2026-10-22 17:29 冷启动 Node not found 已修复（未重新现场启动）

用户报告完整入口在ros2 param get /go2_nav_supervisor keepout_mask失败。只读最新System launch日志2026-10-22-17-29-57-052386…28026确认：map_server/AMCL及wait_go2_slam已启动，尚无controller/planner/supervisor启动记录；amcl_28421日志持续提示Please set the initial pose。System实际调用顺序为AMCL就绪waiter完成才启动Navigation/supervisor，而AutomaticLauncher的READY只提示人工定位并立即继续。新增Keepout参数查询放在NAV_READY等待之前，导致冷启动尚未定位时查询不存在节点。这是脚本时序问题，不是已发生supervisor崩溃的证据。
已将Keepout参数查询移至既有NAV_READY=true等待之后，保留参数一致性校验、DISARMED门槛及所有后续检查；脚本直接从仓库执行，无需重建/覆盖已验证overlay。回归先增加“未ready禁止查询supervisor”约束，复现冷启动/复用两项失败，再修复后启动相关111项全部通过。证据build/keepout-startup-order-20261022/含源脚本before、System/AMCL日志副本、red.xml/green.xml。
本次入口代码路径在prepare/ARM/start之前失败；末尾“已ARM或start结果不明”是通用提示，不代表该轮已执行ARM。没有从本机进程情况推断实体停车/解除使能。未重跑现场脚本、未SSH/ROS现场采样/硬件重启/ARM/start/运动；仅离线替身测试及diff检查。下次正常准备仍须在RViz初始化真实位姿；已有有效定位不重置，现有NAV_READY等待上限120秒未放宽。

### 2026-10-22 17:35 agent同名误判已修复（独立安装，未重新现场启动）

用户随后报告“multi_agent_go2_agent 重复”。源脚本仅在component启动后的节点列表匹配数>1时输出该准确文本；agent同一进程创建AgentNode和默认名multi_agent_go2_rgbd的VLMNavigator，前者/后者均继承进程级__node:=multi_agent_go2_agent，导致同名。python_30888_1792661716071.log同时有同名rosout已注册警告及agent启动记录；Domain31 localhost真实rclpy回归用相同全局重命名复现默认/自定义观察器名丢失，2失败2通过。
最小修复在VLMNavigator构造：observation_only时use_global_arguments=False，继续接收显式node_name/parameter_overrides；普通单机模式保留全局参数。保留component重复保护，不通过去重隐藏真实重复；同一修复覆盖正式脚本和Go2 launch的多节点进程。原有observer调用及参数来源已核对。

新成对安装build/go2-agent-identity-20261022/install/local_setup.bash，两包构建退出0，56个实际Python/launch/config文件与源码一致；新安装针对性165项通过，包含真实rclpy默认/自定义名及重命名组合、RGB-D/TF、定位、Keepout、启动流程，未做全库测试。Cooperative_env.bash、Launcher缺省期望、环境测试和正式启动文档统一新安装。preflight只读比较实际安装vlm_navigator字节，不导入系统Python缺失的SDK；默认/显式新路径均退出0，旧Keepout路径按预期退出1。旧已验证overlay未覆盖。证据build/go2-agent-name-20261022/evidence/；构建/环境命令仅在[runbook最新节](go2_runbook.md#2026-10-22-go2观察节点重名修复)维护。

与上一轮Node not found不同：此轮Sport日志1792661709.417701有ARM accepted（17:35:09），错误发生于prepare的agent出现检查之后、idle/coordinator/start_task之前，没有本轮start.log；不能把入口退出当DISARM。本机/proc后续只读核对未见相关进程，不证明实体停稳/解除使能。现场恢复按原关闭流程确认停稳和DISARM后再用新版本，不盲目重复完整入口。
本轮未SSH、连接Ranger、初始化现场ROS、采集相机/完整点云、停止或重启旧进程、ARM/start、录包或运动；仅读取历史日志、构建、离线preflight和隔离Domain31测试。没有提交或清理用户工作。

### 2026-10-22 Go2搜索先看当前相机（已实现/离线验证，未真机启动）

用户要求“刚启动就上传相机图像给VLM，识别到目标就导航，否则再按探索点选点”，追加静态检查及完成时电脑较响提示音。按既有流程小改：仅live Go2正式本地搜索开始、停稳和健康门槛通过后，在候选准备前调用现有ABC扫描的current_view_only模式，等待当前朝向的一帧新鲜RGB-D，无Spin。有效DETECTED复用原接近规划/Keepout/双costmap/Nav2成功/距离验收；NOT_DETECTED/UNCERTAIN且结果完整才回原探索；API故障/缺结果暂停，未定位返回TARGET_DETECTED_UNLOCATED。
初始ABC仍缓存C；APPROACH_DEPOSIT沿用缓存，Ranger/mock不改。原独立go2_check和沿途持续识别未扩展，本次是协同搜索开始前的一帧。static preflight生成几何候选仍保留，不等于已调用VLM选点。
沿用已有views图像、原始回复和metadata，不新增相机/完整点云订阅。初始事件INITIAL_CAMERA_START/RESULT带真实任务/指令，viewpoint_id=initial_camera；审查发现last_motion会使初始证据空身份/归旧任务，已改为显式本轮task_id/command_id、空导航身份，worker测试确认保存结果正确。

新成对安装build/go2-camera-first-20261022/install/local_setup.bash，两包构建退出0，56个Python/launch/config文件逐字节一致；Cooperative_env、Launcher缺省路径、环境测试及正式启动文档同步新路径，旧overlay保留。完整构建/检查顺序只在[runbook最新节](go2_runbook.md#2026-10-22-go2搜索前先识别当前相机)维护。
基线234通过/3失败（另一次未source消息underlay的早期基线缺模块，已纠正环境并保留日志）；最终实际安装421通过/相同3失败，新增22项全部通过。初始分支19项+扫描证据29项共48项专项通过，包含合成完整任务先看图→PICKUP→同帧缓存C→DEPOSIT→DONE且零探索点。测试使用新安装并禁用源码conftest；没有把合成通过写成现场验收。证据build/go2-initial-camera-20261022/evidence/。
实际默认/显式入口preflight在环境阶段被拒绝：enp4s0无192.168.123.x地址；未改网卡、造IP或绕过现场门槛。提取同一preflight Python并按实际配置设置GO2_NET_IFACE=enp4s0后，独立离线配置检查退出0，installed全匹配、两种SYNC=true、live_gates=[]、keepout=29ab5c38、API环境齐全、motion_sent=false；这不证明当前网络或传感器可用。
全部检查结束后已按用户要求以80%输出音量播放3次完成提示音，音频命令退出0，并恢复原音量及静音状态；证据notification.json。144个Python文件AST语法、Bash/21段文档命令、14处本地链接及diff检查通过。没有SSH/连接Ranger、现场ROS采样、真实模型请求、硬件/任务启动、ARM/start/录包/运动或自动提交；当前运动状态未验收。

### 2026-10-22 19:01 rgbd_ready / non_motion_inputs_stable 失败只读分析

用户运行新入口后报告上述两项。最新报告build/go2-field/cooperative-task.a53do5xi/inspect.log显示94个样本的rgbd_ready全部false，first_input_ready_elapsed_s=null，因此始终没有连续2秒输入稳定窗口。末尾RGB/深度配对matched=30,total=30,rate=1.0、深度单位通过，但rgbd_frame_age_s/camera_tf_age_s都为null，16帧等待图像时刻TF。
报告首行精确报错：图像请求时刻1792666944.923529（本地显示19:02:24.923529），TF仅到1792666857.640898（19:00:57.640898），相差87.28263092041016秒；日志写出时钟为19:00:57。已确认图像时间戳超前TF，导致map→camera_depth_optical_frame在图像时刻不可用，无法形成可定位FrameSnapshot。这不是RGB与深度互相不同步的证据；map匹配、localization/costmap/Keepout在检查末尾均通过。
相机由Go2上位机192.168.123.18发布，TF主要在控制电脑生成，优先怀疑两端时钟偏移或RealSense设备时间到系统时间转换；未采两端UTC/NTP，不能把主机时钟失同步定为唯一根因。没有SSH、初始化ROS、订阅新相机/点云、改系统时间/驱动/参数、重启或运行实验。本次入口在Launcher输入检查处停止，尚未到本轮prepare/ARM/start/新增初始VLM识别，不能套用17:35那轮已ARM的事实。
证据副本及摘要：build/go2-field/rgbd-time-mismatch-20261022/inspect.log、analysis.json，原报告SHA256及87.28秒差值/94次失败已用离线断言核对。下一步只需先在控制电脑和相机上位机核对UTC及NTP状态；时钟一致时再查相机驱动时间域，暂不放宽门槛或换最新TF掩盖图像时刻不匹配。

### 2026-10-22 task a944 最新实验暂停只读排查（终端保留）

用户报告新实验疑似报错退出，要求保留终端排查。本轮任务a9445385f49c4f39aa0284cfdd7fbf49，入口build/go2-field/cooperative-task.gosi8gdo，start.log accepted=True。该轮inspect末尾rgbd_ready=true、matched30/30、TF年龄约0.067s，输入稳定5.64s，地图/定位/成本图/Keepout通过；不沿用19:01图像超前87秒的根因。
新策略确已执行：本任务events.jsonl仅INITIAL_CAMERA_START，views/c680fba2fda34599a64052cf3076d92a_view_00/保存input.jpg、response.txt、metadata.json、result.png等。qwen3-vl-plus在8.411675s返回有效ABC：A灭火器DETECTED、confidence0.95、归一化像素420/580换算268/278；B/C未检测到。没有INITIAL_CAMERA_RESULT/探索选点/接近规划或导航事件，不能把API验证通过当成定位/导航通过。

本次用户明确授权只读排查，进行了6秒本地Domain0低流量任务/状态采样，无RGB/完整点云订阅：coordinator及两个agent仍持续发布，任务为PAUSED，并非整套进程退出。Ranger锁存reason='local health invalid: localization_valid'，Go2锁存reason='peer localization/navigation invalid'，coordinator保留'go2: agent paused: peer localization/navigation invalid'。两端当前报告localization_valid/nav_ready/stopped=true，但暂停不自动恢复。尚需Ranger原始日志确定其瞬时定位失效的细项，不能断言TF超时或数据源中断。
Sport日志19:08:11初始DISARMED、19:09:19 ARM accepted、19:11:50.808920发StopMove；模型完成晚于StopMove约3.2426s，因此取消后晚返回的识别只落盘，没有继续导航。采样窗口Sport持续报告ARMED、Go2本地NAV_READY/SYSTEM_READY=true；没有执行pause/disarm/start/ARM服务，不能把PAUSED当已解除使能或把状态stopped报告当实体停车验收。

重要证据边界更正：工具exec的/proc是隔离PID命名空间，只显示当前sandbox进程，无法用它判控制电脑真实ROS进程退出。此前“/proc未见进程”只能表示此隔离视图未见，不再作主机进程判断；本轮已以实际ROS graph和持续状态确认节点仍在。首次coordinator echo误用TRANSIENT_LOCAL导致DURABILITY不兼容，按实际发布者VOLATILE修正后采样成功；该诊断订阅错误不是实验暂停原因。
证据build/go2-field/task-a944-diagnosis-20261022/含live-state.json、analysis.json、agent事件/VLM metadata/回复、Sport/idle副本及摘要；原始图像保留在实验目录。已向用户请求Ranger agent首次暂停前后原始文字，等待补充。未SSH、连接Ranger私有ROS、改生产代码/参数/时钟、重启/关闭终端、真实模型请求、新任务/ARM/start、录包或运动。


## 2026-10-22 RViz目标、候选、最终选点及路径

用户要求小改动，在RViz标注识别目标、候选探索点、VLM最终选点及生成路径，颜色柔和、标签小。新增exploration_display复用已有generated_exploration_markers，正式live Go2 AgentNode通过runner.on_event接通，独立go2_check explore也复用；原Ranger和模拟行为保留，无新依赖/传感器订阅。
仅本轮上传VLM的至多12候选显示灰蓝小点及P编号；VLM最终点为灰褐短箭头/短标签；有效定位的ABC目标转换到本机map后显示灰绿小点；未知/非有限坐标不画虚构点。PATH_PLANNED仅在选中路径通过现有运动检查后发出，执行重规划复用Go2Adapter原on_path_update合法当前导航回调；RViz细线显示。新任务清旧显示，同任务子命令保留目标；换轮、STOP及adapter清路径同步清活动标记/路径。显示发布异常仅告警，停止先执行物理停指令再清标记。SCAN_RESULT记录全部scan_results，使同帧ABC均可显示，未改变识别或导航判据。

新成对安装build/go2-rviz-markers-20261022/install/local_setup.bash，初次两包18.6秒、最终Python同步两包0.68秒；旧camera-first等安装未覆盖。Cooperative_env默认、Launcher兜底、环境测试和正式文档已同步新路径，preflight增加新display模块安装匹配；源与安装Python/launch/config含RViz共58文件一致。
证据build/go2-rviz-markers-20261022-evidence/：源码117项通过（新增显示8项、相关原有109项）；新安装扩展相关回归275通过，无失败，不是全库。默认实际离线preflight退出0、所有源码匹配、live_gates=[]、motion_sent=false；只读审查发现并补齐STOP/路径清理回归后复核无阻断。构建/环境/使用命令仅在[runbook最新RViz节](go2_runbook.md#2026-10-22-rviz目标选点和路径)维护。

当前旧现场终端/任务未触碰，旧显式COOP_SETUP_FILE仍会覆盖默认值；下次按原流程确认停稳、终态并退出旧任务后，统一新安装再正常准备，RViz重新加载config/go2_exploration.rviz。未新启现场ROS/实验、未SSH/ARM/start/录包/调用真实VLM；未做现场渲染及运动验收。上一轮a944暂停的Ranger localization_valid根因仍待Ranger侧日志，不因本次可视化修改宣称已修复。

### 2026-10-24 task c5aa 执行期对端任务心跳丢失（本地离线分析）

用户报告任务c5aa1784f95f4ee0a6f12f5860e9aea3反复显示PAUSED、reason截断为go2: agent paused: peer heartbeat lost for current...。本地入口build/go2-field/cooperative-task.xthuxhhf/start.log确认accepted=True；preflight使用go2-rviz-markers-20261022成对安装、源码匹配。实验events.jsonl共9条：初始相机ABC均NOT_DETECTED，模型校验通过、耗时8.054645s；随后模型有效选择P11，规划通过、193点路径，最后为OBSERVATION_RECORD executing，没有到达/后续扫描完成记录。

实际日志build/go2-approach-coverage-20261018/ros-logs/中bt_navigator_10901_1792815296452.log记录北京时间12:16:05.659818开始从(4.68,-13.14)导航到(9.44,-13.28)，12:16:29.680573记录Goal canceled；controller同时记录客户端取消。Failed to get result for follow_path in node halt!出现在取消阶段，不能据此认定为首因。Sport本轮12:15:26记录ARM accepted，后续有StopMove，但本次未采样当前状态，不能认定实体停稳或DISARM。

源码agent.py唯一对应报错为peer heartbeat lost for current task：提交/执行后，coordinator SYNC未超时，而peer_task_received缺失或超过lost_timeout=3s，触发STOP并锁存PAUSED。默认心跳2Hz；使用time.monotonic的本机接收回调时间，不直接用对端stamp判断。旧任务状态不刷新本任务心跳；身份错误、对端健康失效和空任务重启通常有各自reason。coordinator按2Hz重复发布锁存原因，因此三条相同PAUSED不表示三次独立故障，心跳恢复也不自动续跑。

根因边界：已确定报告指向Go2的当前任务Ranger状态接收链；尚无故障窗口逐条接收记录或Ranger本轮原始日志，不能区分Ranger执行阻塞/停发、两端bridge/Wi-Fi/DDS传输、Go2回调调度或其他task_id状态。已请求Ranger在12:16:20至12:16:35的首次暂停reason及日志，待补充；不沿用a944定位失效或早先相机时钟偏移作为本轮根因，也未放宽3秒门槛。

证据build/go2-field/task-c5aa-diagnosis-20261024/analysis.json和inputs/含15份入口/事件/VLM metadata与回复/agent/coordinator/bridge/Nav2/Sport日志快照及SHA256。实际安装的执行故障锁存离线测试8项通过；额外纯状态机断言复现“SYNC新鲜、旧任务消息持续到达，但本任务心跳超时”的准确reason，并验证恢复消息不能自动解锁。未初始化ROS、订阅现场话题、SSH/连接Ranger、改生产源码/参数、构建、重启或关闭终端、ARM/start/录包/运动；未自动提交。

### 2026-10-24 Go2 当前状态窗口（已实现，未现场验证）

用户批准将原watch替换为自动“Go2 当前状态”窗口。scripts/start_go2_cooperative_task.py的prepare共用此窗口，完整入口仍沿原启动/ARM/start顺序；scripts/cooperative.sh go2 watch改为直接运行仓库scripts/show_go2_status.py，无需重建或切换现有go2-rviz-markers-20261022安装。原有任务和旧watch未触碰；下次正常启动自动使用新显示，也可按[runbook最新状态窗口节](go2_runbook.md#2026-10-24-go2-当前状态窗口)单独只读审查，不重跑prepare/start。

查看器仅订阅本地Go2 state和coordinator status，Reliable/Volatile；不创建控制发布者或服务客户端，关闭rosout与参数服务。显示完整run_state/reason、phase/reason、各自task_id及定位/nav/stopped报告。变化立即打印，未变化每5秒，0.5秒检测消息年龄；接收单调时钟超过实际scene lost_timeout（当前3秒）时标明最后报告，不把失联推断成PAUSED。任务ID不同提示分别审查；异常JSON及错误Go2身份可见且不刷新最后有效报告接收时间。Ctrl+C清理本查看器，不暂停/解除使能或改变现有锁存逻辑；Go2消息新鲜不证明Ranger心跳已恢复，停稳报告不替代实体停稳/DISARM验收。

证据build/go2-status-window-20261024/evidence/保存改前文件、baseline/red/green日志及XML、离线preview.txt与进度记录。原启动/环境/硬件入口离线基线111项通过；最终相关132项通过，新增21项，非全库。覆盖完整长原因、各路独立过期/恢复、配置阈值、任务不一致、异常JSON/身份、节流、两只读订阅、Ctrl+C及setup中断清理、watch路由和窗口接入。独立审查无阻断，其复跑的92项为上述相关用例子集，不重复计数。

首次shell路由红测遗漏ros2替身，旧watch误调用真实ros2并因~/.ros/log只读在日志初始化时报错退出，未取得现场消息；保留red-initial证据，补齐命令替身后重新完成红/绿测试，后续均为离线替身。未SSH/连接Ranger、新启硬件/bridge/agent/coordinator、ARM/start/录包/运动、重启旧终端或自动提交；没有现场接收、图形终端渲染或实体运动验收。


### 2026-10-24 Go2选点增加RGB同编号投影（已实现，未切换现场）

用户批准选点时发送带P编号的RGB，并明确禁止新增相机订阅；此前明确接受宽松投影，不增加深度可见性验证。Go2Adapter只读既有观察节点latest_snapshot，复用原vlm_input_ready门槛；选点地图worker追加第三张相机图，ABC识别仍使用未修改的原始RGB。相机内参及图像时刻camera→map均来自冻结快照，投影取逆，map z=0沿用二维导航平面作为近似显示，不宣称实测物理地面。原图不变，画面内有限前方点画空心圈及同轮P编号；相机后方/画面外点不钳入RGB、仍可在地图选择。提示词明确可能叠在桌面等遮挡物上，不是检测或运动安全证据；Ranger/mock保留原双图流程，候选/取消/健康/路径及导航前复核不变。

现有decisions/<decision_id>/追加camera_rgb.png、camera.png及camera_projection.json，保留真实发送图、原图、图像/RGB时间、序列、内参、变换、像素对应和未投影编号，request.json记录实际三图。未新增相机/RGB/深度/完整点云订阅、依赖或常驻诊断；raw模式沿用已有几何对未验证颜色畸变的拒绝，不猜测畸变模型。

新成对安装build/go2-candidate-rgb-20261024/overlay/install/local_setup.bash，两包19.3秒构建退出0，53个Python文件与源码一致。未覆盖旧安装，未修改Cooperative_env/Launcher默认值或切换现有终端，当前默认go2-rviz-markers尚无本功能。后续按原正常关闭及现场授权流程统一显式新overlay；构建/环境/检查完整顺序仅在[runbook最新RGB节](go2_runbook.md#2026-10-24-go2选点附带rgb候选投影)维护。

证据build/go2-candidate-rgb-20261024/evidence/保存四份改前源码、精确task.diff、baseline/red/green/installed日志及XML、build.log、install_matches.json和纯合成preview/camera.png。基线201通过，新增9项先失败，源码最终及新安装各210通过（非全库）；独立审查无Critical/Important/Minor，复跑52项为上述子集，不重复计数。覆盖逆变换、后方/边界/非有限、空可见集合、冻结RGB不变、失效几何、既有快照复用/未就绪、三图请求及视野外仍可选。合成预览已目检，不代表真实相机效果。

未初始化现场ROS、调用真实VLM、SSH/连接Ranger、启动/切换现有任务或终端、ARM/start/录包/运动、自动提交或部署；现有运行状态未验收。未做真实投影/模型选择效果或全库测试；历史Ranger任务心跳丢失根因待办不因本次改动消失。

### 2026-10-24 RGB选点安装更新为默认（入口已更新，未重启现场）

用户随后要求“进行更新”，授权将刚验证的build/go2-candidate-rgb-20261024/overlay/install/local_setup.bash设为默认。已更新Cooperative_env.bash、scripts/start_go2_vlm.py的协同硬件安装预期、环境选择测试及正式docs/双机协同启动与验收.md全部18处当前安装路径；文档增加RGB投影说明并链接最新runbook。旧安装保留，不重建已验证overlay，不新增相机订阅。

入口相关基线112通过；环境默认切换及Launcher默认成对安装测试先红，最终113通过（新增1项，原环境默认/显式覆盖测试更新，不是全库）。实际env -u COOP_SETUP_FILE默认preflight退出0，13项installed matches_source全true、路径均指向新安装、两种SYNC=true、API环境齐全、live_gates=[]、motion_sent=false。证据build/go2-candidate-rgb-default-20261024/evidence/保存改前入口/正式文档及baseline/red/green日志和XML、default-preflight.log；首次新Launcher红测缺COOP_CONFIG，补齐测试环境后确认真实旧默认路径失败，再改生产默认。

现有父终端和运行进程未切换/重启；显式旧COOP_SETUP_FILE仍优先。下一次按正常旧任务退出、实体停稳和动作终态流程后，以正式文档新路径统一各Go2终端。环境及默认只读预检命令仅在[runbook默认更新节](go2_runbook.md#2026-10-24-go2选点rgb安装设为默认)维护。不根据离线预检宣称实时RGB/TF、Ranger链路或运动通过；未SSH、现场ROS初始化、真实模型调用、ARM/start/录包/运动、自动提交。

### 2026-10-24 14:04 start被拒绝：Go2导航就绪抖动（本地离线分析）

用户报告StartTask_Response accepted=False、task_id为空、reason='go2: fresh healthy stopped state required'及ros2run退出1。最新入口build/go2-field/cooperative-task.wp34wl62/preflight.log使用candidate-rgb新安装，源码匹配、live_gates=[]；idle.json在14:04:01.579写出，两端均STANDBY且定位/nav/stopped为true，Go2样本stamp为14:04:00.999530。拒绝不建立新task_id，start客户端按accepted=false主动返回1，不代表coordinator或整套ROS崩溃。

同轮supervisor原始日志build/go2-approach-coverage-20261018/ros-logs/python3_27644_1792821705682.log确认：14:04:02.721952唯一失败项keepout_costmap_fresh，全局图有效header年龄2.003987s超过maximum_data_age_s默认2秒，NAV_READY=false；14:04:03.221876年龄0.504033s又恢复true，14:04:04.722再次过期。用户随后提供14:06:53/54的状态窗口：Go2 PREPARING/nav_ready=false与STANDBY/nav_ready=true交替，定位/stopped均true、两路消息新鲜、任务ID为空且coordinator IDLE；与日志14:06:53.721850过期2.004030s、14:06:54.221851恢复0.504136s对应。由此高度支持导航就绪不稳定导致本次拒绝，不沿用c5aa的执行期Ranger心跳丢失解释。服务受理时coordinator内部快照未落盘，不能把通用reason单独当成某一子条件的直接记录。

实际本轮临时Nav2参数/tmp/vlm_nav2_go2_r6_q4mz7.yaml确认global update/publish均1Hz、always_send_full_costmap=true；global/local Keepout均配置。supervisor使用图像header年龄抵扣接收后预算；为何名义1Hz仍反复出现约2.004秒有效图龄，尚需区分发布/更新周期、header及回调时序，不凭现有证据断言网络丢包或唯一调度根因。修复方向应核对全局成本图实际时序与2秒新鲜度预算，不绕过Keepout、不放宽start条件或自动重试。本次未修改生产代码/参数。PREPARING下reason仍为空是当前agent未填具体局部健康原因，不能用“原因：无”推断健康全通过。

新证据build/go2-field/start-health-rejection-20261024/analysis.json及inputs/含入口、idle/inspect、supervisor/agent/coordinator/Sport日志和实际临时参数快照与摘要。实际新安装纯状态机断言复现nav未就绪、定位无效、未停稳、旧状态和缺状态均返回相同拒绝且保持IDLE/空task_id；健康状态才进入SYNCING，合计6种情形，未初始化ROS。start.log缺失另由当前Launcher.command在非零退出先抛错、cooperative尚未写日志解释，原错误只在主终端输出；不是未尝试start的证明。

本轮Sport日志14:02:15.685035记录ARM accepted；start拒绝不触发自动DISARM，当前实体停稳/解除使能未验收。本次仅本地文件分析与纯状态机检查，未SSH、采样现场话题、重新start/ARM、重启/关闭现有窗口、改现场配置、录包或运动；未构建或跑全库测试，未自动提交。

### 2026-10-24 14:13 控制电脑网络只读核对

用户要求核对控制电脑网络。当前enp4s0为UP/LOWER_UP、1000Mb/s全双工、192.168.123.222/24，相机192.168.123.18路由经enp4s0；wlo1为UP、192.168.137.65/24，默认路由经192.168.137.2。项目配置仍是本地Domain0=enp4s0、桥Domain40=wlo1；没有用隔离/proc进程环境推断实际运行进程绑定。

14:13:40.671至14:13:50.683的10.012秒计数差值：enp4s0约RX219.69Mb/s、TX10.18Mb/s，RX/TX errors/dropped、missed/CRC及carrier_changes均零新增，驱动错误计数为0；UDP InErrors/RcvbufErrors/SndbufErrors和IP分片重组失败亦零新增，可见UDP socket内存丢弃计数均0。lo约486Mb/s为本机流量，不计作有线占用。wlo1新增1个rx_dropped，链路无切换；当时iw信号先后-85/-82dBm，缓存NM信号34/100，无线质量应保留为协同链路风险，单个丢弃不能归属Ranger/DDS或某任务。14:14:43所读Wi-Fi当前关联从14:12:02.506开始，晚于14:04拒绝，不能倒推拒绝时已断连。

同一采样窗口supervisor在14:13:41.722/44.722/46.722/49.722四次仅keepout_costmap_fresh失败，全局图龄约2.004秒，均在约0.5秒后新图龄0.504秒恢复；同期local costmap和SYSTEM_READY新鲜。因此当前有线链路错误/接收缓冲溢出没有支持本次持续过期的证据，仍优先查本机全局成本图更新/发布/header/回调时序；这不排除短时突发延迟或故障时历史异常。

证据build/go2-field/network-check-20261024/含before/after/delta.json、network-details.json、同窗supervisor-window.log及analysis.json，计数差分和同窗失败项已离线断言核对。NetworkManager/内核历史日志普通用户无权读取；脱离沙箱只读查询后仍无读取组权限，sudo -n要求密码，未提供密码、未改用户组或网络，故不能声称故障时无重连。没有抓包或直接订阅ROS成本图，实际逐条发布/接收间隔仍待验证。

本次只读网卡、路由、驱动统计、Wi-Fi关联和已有日志；未ping/SSH或连接Ranger、未初始化ROS/订阅相机或完整点云、未重启组件/ARM/start/改网络/运动，未自动提交。部分ethtool受限字段返回Operation not permitted，但速率/双工/link及驱动错误统计已可读，不将该提示认定为链路故障。

### 2026-10-24 14:23 更换Wi-Fi后10秒最小复测

用户更换Wi-Fi并要求再次最小测试；按上一轮同范围，只做14:22:59.370至14:23:09.382的10.012秒被动计数、关联状态及已有supervisor日志对照，没有新start或运动。新AP的wlo1地址为192.168.1.118/24、默认网关192.168.1.1，信号由之前-85/-82dBm改善为-46至-37dBm；enp4s0仍192.168.123.222/24，相机192.168.123.18路由仍经有线。两张网卡RX/TX errors/dropped及carrier_changes均零新增，UDP InErrors/RcvbufErrors/SndbufErrors亦零新增；有线RX约219.18Mb/s、TX10.16Mb/s。

同窗14:23:01.722、03.722、06.722仍各有一次仅keepout_costmap_fresh失败，年龄2.003851至2.004194秒，均约0.5秒后恢复到0.504秒。因此更换为强信号Wi-Fi后本机成本图周期性过期仍存在，不能用原Wi-Fi弱信号单独解释；实际发布/header/回调时序问题继续待查。

额外边界：Wi-Fi station的rx drop misc从56增至58（和接口rx_dropped是不同计数），IP OutNoRoutes本次增51，来源未知；ss看到一个UDP socket仍绑定旧无线地址192.168.137.65:58282，未用隔离PID视图将其归属到DDS或某进程。当前新地址已有socket，但不证明Ranger同网或Domain40通信恢复。没有采样任务话题、抓包、连接Ranger或重启bridge，不宣称双机协同通过。

证据build/go2-field/wifi-retest-20261024-142259/保存before/after/sample.json、network.json、udp-sockets.txt、supervisor-window.log与analysis.json。仅复测网络及日志，不改生产代码/参数或网络、不初始化ROS/新订阅传感器、不ARM/start/录包/运动、不自动提交；10秒结果不排除其他时段异常。

### 2026-10-24 14:29 全局成本图发布/header时序已定位（25秒本地只读ROS采样）

用户明确要求检查实际发布和时间戳，执行一次14:29:06.206至14:29:31.222的25.016秒本地Domain0/enp4s0采样。临时诊断节点仅订阅/global_costmap/costmap_raw、/vlm_nav/nav_ready及/multi_agent/go2/state；只保存元数据，不保存完整栅格，不订阅相机/RGB/深度/点云。仅调用global costmap及supervisor的GetParameters读取参数，无写服务或控制发布。探针正常退出并释放节点；首次环境加载因配置路径非绝对而在ROS初始化前拒绝，改为绝对路径后仅运行这一轮25秒采样。

实际GetParameters确认global update_frequency=1.0、publish_frequency=1.0、always_send_full_costmap=true、use_sim_time=false；supervisor maximum_data_age_s=2.0。global raw发布者唯一/global_costmap/global_costmap，Reliable/TransientLocal。收到17帧，16个相邻header间隔各有8次约1秒、8次约2秒；首帧为TransientLocal缓存，年龄0.491秒，不纳入稳态到达年龄。其余16帧header至回调8.185至14.844ms，header至DDS source timestamp仅0.139至0.278ms，DDS发送至回调8.045至14.693ms，未观察到秒级发送后延迟。Humble RMW received_timestamp为0，不能再拆DDS传输与Python执行器延迟；metadata.update_time同样为0，本检查使用header.stamp，不把其误作旧图时间。

同窗NAV_READY=false共8条，与supervisor日志8次仅keepout_costmap_fresh失败对应；每次均在约2秒间隔的新帧回调前约2.9至9.6ms判图龄超过2秒，之后在下一0.5秒检查恢复。根因已定位为当前配置下发布节拍与新鲜度预算冲突：Nav2发布检查嵌在更新循环内，条件为last_publish + publish_cycle < current_time；两者同为1Hz时，微小周期抖动会使某次检查未满足严格小于，延至下一秒发布，形成实测1/2秒间隔。2秒有效header年龄预算没有给这一最长周期和回调延迟留下余量，产生就绪抖动。不是实测到2秒网络传输，也不是header与本机时钟存在秒级偏移。

已核对本机ros-humble-nav2-costmap-2d版本1.1.20-1jammy.20260804.204611；本地libnav2_costmap_2d_core.so反汇编确认时间相加、严格operator<及条件publishCostmap调用，与[官方同版本更新循环](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp#L425-L465)及[发布header代码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_publisher.cpp#L142-L175)一致。离线整数时间模型复现同频跳轮；将发布检查周期设为0.5秒、更新仍1秒时，同一模型每次更新均可发布，实际发布上限仍由更新循环限制。这是最小修复候选（仅Go2 global publish_frequency=2.0），本轮未改参数或部署，保留2秒新鲜度及Keepout门槛，后续效果须另行验证。

证据build/go2-field/costmap-timing-20261024/含一次性probe.py、started.json、samples.jsonl、真实参数/publisher/统计summary.json、costmap-timeline.csv、同窗supervisor-window.log、本机Nav2反汇编及analysis.json。计数、header/source/callback延迟、唯一发布者、8次失败和离线严格比较模型均已断言核对；没有重跑现场采样。一次性探针按当前Humble实现保留被默认执行器丢弃的RMW消息时间信息，不作为长期入口；未测内部updateMap耗时，不将25秒窗口当长期稳定性或任务验收。

本轮初始化过现场ROS用于已授权的限时只读采样，不能表述为“未初始化ROS”。没有SSH/连接Ranger私有ROS、真实VLM请求、相机/完整点云诊断、新start/ARM/录包/运动、停止/重启现有任务、写生产代码/参数、构建或自动提交；实体运动及DISARM状态未验收。

### 2026-10-24 Go2全局publish_frequency已改2.0（未重启现场）

用户明确要求将Go2全局发布频率参数改为2.0、更新保持1.0Hz、保留2秒保护，必要时更新环境。仅在cooperative_ws/src/multi_agent_nav/config/nav2_go2_cooperative_costmap.yaml的global ros__parameters增加update_frequency=1.0和publish_frequency=2.0及同频边界原因注释；local、旧单机、传感器/动态层/Keepout未改。supervisor文件与改前逐字节相同，maximum_data_age_s默认仍2.0，没有修改运行中参数。

正式入口scripts/start_go2_vlm.py将COOP_CONFIG源码目录下的该文件显式传为costmap_overrides_file，最后合并；所以本轮无需构建、切换环境或覆盖旧overlay。离线使用现有candidate-rgb已安装的Navigation launch与go2_nav2_config加载器（排除工作目录源码遮蔽），确认最终global更新1.0/发布参数2.0，local仍5.0/2.0、两张图Keepout保留，旧单机global仍1.0/1.0。实际话题受1Hz更新循环限制，不将发布参数2.0称为实测2Hz。已运行/被复用的旧Nav2不会热更新；只有后续正常退出旧底层、再创建Nav2节点时读取新文件。使用显式COOP_CONFIG副本时也须同步这两个global字段；安装内旧配置副本保留，不据此宣称所有独立入口默认均更新。说明见[runbook频率修正节](go2_runbook.md#2026-10-24-go2全局成本图发布频率修正)。

证据build/go2-costmap-publish-20261024/evidence/含改前文件、scope、baseline/red/green日志与XML及installed-launch-check.json。原相关基线59通过/1失败；两项既有参数合并/实际launch输出测试增加约束后先因发布值1.0失败，配置修复后这两项通过，最终仍59通过/相同1失败。失败为test/test_go2_keepout.py::test_readonly_and_scan_adapters_subscribe_global_map_when_keepout_required，系统Python缺httpx的导入错误，未改依赖或生产代码绕过；3项排除为2个原生ROS测试和1个已有验证标志假设测试。没有新增测试函数，非全库；独立只读审查无阻断。

本轮只改配置、相关既有测试及说明，做离线参数生成/检查，没有初始化现场ROS、执行launch动作、动态参数写入、SSH/连接Ranger、重启/关闭现有终端、新ARM/start/录包/运动或自动提交。当前运行Nav2仍按旧参数，实际频率及NAV_READY稳定性未复验；实体停稳及DISARM状态未验收，不以配置落盘当现场修复通过。

### 2026-10-24 task 1564：局部轨迹碰撞、恢复Spin后路径保护暂停（只读分析）

用户报告协调器reason='go2: agent paused: invalid pose or executing path'。最新实际任务1564b92847bf4dba9e76e9169b3bca70，入口build/go2-field/cooperative-task.a6utqz3s/start.log确认accepted=True。本轮初始VLM成功返回灭火器A DETECTED、confidence0.95、耗时8.599秒，B/C未检测；events.jsonl共9条，已定位A、接近候选检查通过、规划219点路径，选中map目标约(0.902,-12.078,-1.571)，没有后续到达成功记录，也没有进入探索选点的decisions目录。API/规划通过不等于目标物理定位和实际通行验收。

原始Nav2日志确认北京时间14:58:18.950从(6.17,-13.38)开始接近导航；14:58:36.422开始DWB连续报告No valid trajectories out of 3、ObstacleFootprint/Trajectory Hits Obstacle，随后多次Controller patience exceeded，Nav2清理成本图并重试。14:58:40.330为controller最后一次Passing new path to controller记录；40.551开始原生恢复Spin，45.551收到客户端取消Spin及顶层NavigateToPose。最后路径日志至取消约5.221秒，Spin持续约5.000秒。这些障碍拒绝早于用户所报暂停，不能把最终通用reason直接定性为定位丢失。

当前Go2 path_topic=/received_global_plan、path_receive_timeout=5.0；adapter只有收到有效当前导航路径才刷新单调接收时间，超过5秒令path_valid=false。agent在顶层NAVIGATING持续监控路径；Nav2内部恢复Spin期间顶层导航仍可活动，但FollowPath/replanning暂停，未必继续产生执行路径。本轮时间线高度符合“局部避障受阻→原生恢复→路径5秒未刷新→agent停止并锁存PAUSED”。纯状态机在定位/nav/两种心跳有效且已获得路径时，将path_valid置false，复现准确reason并在停稳后锁存PAUSED；有效路径保持执行。正式agent未接入adapter.record_event，因此没有暂停瞬间Health/最后PATH_RECEIVED时间记录；仍不能完全排除空/非有限/被拒绝路径导致同一reason。一般localization失效会先由check_health报local localization invalid，本轮未见对应暂停快照。

本轮实际Nav2临时参数/tmp/vlm_nav2_go2_au4i8z02.yaml已加载global更新1.0、发布参数2.0；supervisor python3_36127_1792824966103.log在14:56:09.641606就绪后，没有后续NAV_READY=false记录，不沿用上一轮全局成本图周期性过期作为本轮首因。这只证明本轮加载及日志情况，不把它升级为长期稳定性验收。障碍的具体来源（实物、动态点云标记、静态地图、Keepout或膨胀）缺当时成本图/位姿记录，尚未定位；不能凭DWB碰撞日志断言特定层误报。

证据build/go2-field/task-1564-path-diagnosis-20261024/analysis.json、inputs/及实际临时参数副本，含入口/idle/inspect、事件、VLM metadata/回复、controller/planner/behavior/BT/supervisor/Sport/agent日志快照和SHA256。已离线核对时间差、事件及纯状态机条件，实际安装agent与源码相同。未改生产代码/参数、构建、初始化现场ROS、订阅传感器/路径、SSH/连接Ranger、重启/关闭现有终端、新ARM/start/录包/运动或自动提交；当前实体停稳和DISARM未验收。Sport本轮有14:56:36 ARM accepted及后续StopMove，不能由PAUSED或StopMove日志认定已解除使能。

用户随后补充现场观察：碰撞障碍来自Ranger运动到Go2附近；这是用户提供的物理场景事实，非本机历史格图独立重建。复核本轮controller共20条Passing new path，首次碰撞14:58:36.422之后仍在36.521、37.513、39.263、40.330更新路径。当前全局静态/点云障碍/膨胀层、局部静态/点云voxel/膨胀层均启用，数据源为既有/vlm_nav/obstacle_cloud；实际global更新1Hz，local更新5Hz，DWB控制配置20Hz；默认导航树RateController按1Hz调用ComputePathToPose，失败有清图、Spin、Wait、BackUp恢复。故已有动态障碍重规划机制且本轮确有后续路径消息，仍不能仅凭更新日志断言路径几何改变、Ranger已完整标入全局图或存在可行绕行空间。暂停仍需结合前述恢复/5秒路径监护证据解释，未据本轮答疑更改控制参数或启动新验证。

### 2026-10-24 task ff3b：到达成功但前缘接触目标（只读排查，真实间距未验收）

用户报告最新Go2找到目标后停得过近，并补充当时前缘完全挨着灭火器；要求不先归因为初始pose错误。用户先称目标未移，随后明确更正“灭火器已经移动，go也移动了”。因此不以当前实物位置或与task1564的跨轮目标坐标差推断定位漂移；此前交流中的约2.09米跨轮差不作为根因证据。物理接触是用户对当时现场的观察，不是本轮工具测距所得。

最新任务ff3b4b2cff1d4d498c55d66fab6087ba，入口cooperative-task.zw1a1j4d accepted=True。VLM初始单帧识别A DETECTED、0.95，15:23:33.438开始接近，15:24:13.688 Nav2 Goal succeeded；事件12条包括APPROACH_STOPPED、ARRIVAL_VERIFIED及TARGET_APPROACHED，navigation_status=SUCCEEDED、stop_confirmed=true。估计目标map=(0.390738,-10.776004)，计划base_link停靠点=(0.899855,-11.285121)，到估计目标0.720米；最终记录base_link=(1.009256,-11.497338)、yaw=2.745265，到估计目标0.950204米、到导航goal0.238757米（小于0.25容差）。最终软件位置比计划离估计目标更远，记录不支持“冲过设定点才贴近”的单一解释。

将保留的FAST-LIO JSON按同进程SPARK_DIAG_FRAME帧号关联ROS日志时间（诊断处理时间，不等于精确传感器stamp），停止后约1/2/4秒XY仅变化0.0061/0.0097/0.0108米，不支持约半米量级的制动越界。按配置±0.38×±0.18矩形，最终姿态到估计目标点的几何间隔约0.5299米；这只是配置模型的点距，不是实测机身/实物表面间距。它与用户观察的前缘接触不一致，目标深度投影、相机外参、定位变换、实际机体包络和目标状态的一致性仍须核对。

已确认的距离设计缺口：成功只检查base_link到缓存目标点<=1.0米，没有物理目标表面或机身前缘的最低安全间隔，arrival_decision对0米和0.1米也返回SUCCESS。standoff配置0.9被min(0.9,1.0-0.25-0.03)压至0.72米，0.03是“保证在最大到达半径内”的预算，不是最低安全距离。单看XY容差，理论可接受中心半径降到0.47米，按正对目标的矩形前沿约只剩0.09米，任意朝向保守点距界约0.05米，尚未扣定位/深度/机体误差；本轮实际记录0.95并未走到这个最小界。全局/局部footprint检查不等于已认证物理目标的最小安全间距。

本命令只在初始相机处生成一次3D目标，接近过程中没有复观测校正目标；运行raw_depth_mode，使用RealSense SDK颜色像素到原始深度像素投影，再用图像时刻TF映射map。当前views证据保存RGB/回复/metadata，但没有原始深度、匹配depth_pixel/raw值、完整内参外参和T_map_camera，无法离线重算这次三维定位。二维标注目检在灭火器上，只证明识别像素大致落对，不证明深度或map坐标正确。AMCL日志两次人工设初始位姿都在图像采集前，无之后人工重置记录；正常定位修正不能仅靠该日志排除。基于两时刻map pose与FAST-LIO的粗略二维对照有约0.306米残差，但保持初始变换时目标距离约0.969米，与记录0.950米仅差约0.019米；不是完整三维/精确stamp的TF重建，不能据此定AMCL跳变。初始固定刚体偏移本身会在同坐标系距离中抵消，后续定位修正/漂移才可能破坏缓存目标一致性；初始pose错误仍是候选因素。

另发现独立停稳验证风险：Go2Adapter.on_odometry只按odom.twist判断stationary，而本机go2_ws FAST-LIO publishOdometry源码未对twist赋值。尚无本轮实际接收twist历史，不能仅从源码认定运行消息必为零，但stop_confirmed不应直接当实体停稳证据。此风险不等于已证明本轮接触由制动过冲导致，前述位置轨迹未见大幅过冲。

证据build/go2-field/task-ff3b-close-approach-20261024/analysis.json、inputs/、odom-comparison.json及FAST-LIO发布源码片段，含事件/图像/VLM回复/metadata、入口、AMCL/controller/BT/Sport快照与摘要；已断言核对距离、goal容差、到达函数无下界、odom小位移及源码条件。没有改生产代码/参数、现场ROS初始化/订阅、复用移动后的现场测距、SSH/连接Ranger、新ARM/start/录包/运动、重启或自动提交。本轮软件到达成功不等于物理安全停靠通过；根因仍需固定相对位置的离线证据或后续另行授权的静止几何核对。

### 2026-10-24 Go2第二轮任务脚本模式（仅离线验证）

用户要求将Go2第二轮命令合并成执行脚本。复用scripts/start_go2_cooperative_task.sh，新增restart动作；Python TaskLauncher.restart依次请求pause、提示确认两端实际停稳/原生动作终态并从原窗口正常退出旧任务组件（CLOSED）、复用disarm_task（STOPPED）、等待NAV_READY最多120秒、复用inspect及require_inputs、prepare（ARM）、start_task（START一次）。保持健康相机/System/RViz/Sport，不新增自动进程终止、Ranger远程操作、故障复位或自动重试。restart与prepare/run共用准备锁，开始前要求交互式终端及图形桌面。

用法、前提和失败分支见[双机协同启动与验收第6.1节](../双机协同启动与验收.md#61-第二轮-go2-执行脚本保留健康底层)。旧coordinator须仍能响应pause，旧组件退出和实体停稳须由操作者确认；Ranger操作者负责本机检查及任务bridge/agent重启与反向peer核对。中途失败/取消不自动DISARM，已ARM或start结果不明时按原关闭流程处理，不盲目再次restart。本模式直接运行源码脚本，不要求构建或更改overlay。文档完整启动命令原有缺失续行符一并补正。

验证：修改前相关基线113通过；新增restart行为在实现前10项预期失败（缺少动作/方法），实现后同组相关测试125通过（原113项和新增12项，包含参数化用例，不是colcon条目数）。范围为test/test_start_go2_cooperative_task.py、test/test_start_go2_vlm.py及test/test_cooperative_environment.py，覆盖流程顺序、阶段失败不继续、无硬件重启/单次start、非交互或无桌面拒绝及shell动作分派。另检查Bash/Python语法、help无环境路径、文档文件链接与空白；不宣称全库通过。

本轮未执行restart/prepare/pause/disarm现场入口、初始化现场ROS、SSH/连接Ranger、重启/结束现有窗口、ARM/start/录包/运动、构建或自动提交；现场双机新任务和运动效果未验收，既有现场风险与待办不因脚本落盘而消除。

### 2026-10-24 task 0916：local navigation not ready，当前RGB接收中断（只读排查）

用户要求排查local navigation not ready。最新任务0916e17890f44d7cb363fb17f2d41e84，入口build/go2-field/cooperative-task.byfm4g7s/start.log为accepted=True。事件共9条：初始相机成功识别A、通过候选/路径检查并选定接近点，没有到达成功。Nav2日志17:08:06.108开始接近，06.130起DWB连续ObstacleFootprint/Trajectory Hits Obstacle，06.430 controller patience exceeded，06.448清局部图并重试，07.201/08.201/09.251仍有路径更新，09.779顶层收到取消。障碍碰撞已确认，但具体障碍来源及与随后健康暂停的因果未确定，不把通用健康reason等同于碰撞reason。

本轮supervisor python3_50562_1792832636204.log自17:03:59.242 NAV_READY=true后无新的false；SYSTEM_READY亦无运行期false。实际临时Nav2参数/tmp/vlm_nav2_go2_a469ly0s.yaml及本轮只读GetParameters均确认global更新1.0/发布参数2.0、local更新5.0/发布参数2.0，不能沿用旧global同频过期原因解释。

为区分supervisor与agent内部综合health，执行一次17:19:55.379至17:20:07.384的12秒本地Domain0/enp4s0只读元数据/TF采样，无相机/点云订阅或控制调用：120条当前任务Go2状态均PAUSED、reason=local navigation not ready、localization_valid=true、nav_ready=false、stopped=true；25条NAV_READY和241条CONTROL_ARMED均true，发布者各唯一且正确。TF缓存预热2秒后87个检查点两图TF均可查，local最大header/receipt年龄0.528/0.528秒，global最大0.997/0.988秒。探针正常释放节点。首次仅source成对local_setup缺消息包，在ROS初始化前导入失败；改用正式Cooperative_env后执行上述唯一12秒采样。

随后仅运行一次现有10秒只读inspect（无VLM请求/运动；临时接收RGB-D，退出即释放）。独立观察器初始化后仅rgbd_ready失败：最终rgbd_stamp_windows.rgb=[]、depth窗口30条持续推进且跨度1.934秒；同步matched=0/total=0、无有效snapshot，non_motion_inputs_stable=false。depth_units、三种导航action服务器、使能/外部门槛、新鲜定位、本地costmap及Keepout检查均true。注意inspect默认退出0只代表受检本地观测条件，不能替代require_inputs的RGB-D/稳定性检查；脚本restart仍调用require_inputs，不会据此自动通过。

另做3秒本地图端点发现，无图像/深度订阅；RGB及depth均有唯一/camera/camera发布者，RELIABLE/Volatile，agent订阅BEST_EFFORT/Volatile，QoS兼容，发布者存在不证明流在发/已到达。本轮已定位当前持续阻塞为RGB接收中断、深度仍有数据→无法同步RGB-D→综合Health.nav_ready=false；尚不能区分机载相机彩色流停止与RGB传输路径故障，未SSH读取机载日志。原agent未公开暂停瞬间last_health_details；新观察器并非原agent内存，不能据其motion_fault_clear=true排除原agent曾锁存其他故障，故最初暂停分项仍保留该证据限制。

证据build/go2-field/task-0916-nav-ready-20261024/含inputs日志/实际参数副本与摘要、12秒probe.py/samples.jsonl、inspect-current.log、camera-graph.json、analysis.json。已离线断言核对状态计数、RGB空窗口/深度推进、分项检查、唯一端点及QoS。没有修改生产代码/参数、重启/停止现有窗口、SSH/连接Ranger、新ARM/start/录包/运动、构建、全库测试或提交。现场ROS已用于上述限时只读采样，不表述为未初始化ROS；CONTROL_ARMED实测仍true，PAUSED/stopped报告不认证实体停稳或DISARM。

### 2026-10-24 RGB中断深入排查：机载USB断连/重枚举，未见订阅过多或持续过载证据

用户要求继续核对RGB中断是否由监听过多/负载引起，并明确授权仅只读SSH到Go2机载unitree@192.168.123.18，读取CPU/内存/USB和现有相机日志，不重启、不改参数、不启动新的ROS订阅或运动。该授权不包含相机复位、拔插、驱动/固件更新、任务重启或新的运动实验。

本机17:43:42.512起一次10.012秒被动CPU/网络计数：32逻辑核总忙9.507%，最忙核25.723%，无内存压力；enp4s0当前1Gb/s全双工，接收105.844Mb/s、发送10.123Mb/s，网卡错误/丢包、UDP InErrors/RcvbufErrors/SndbufErrors/MemErrors、IP重组失败及可见UDP socket丢弃均零新增。wlo1另增1个rx_dropped，不归属走有线的相机。lo约484.84Mb/s是本机流量。当前本地PID命名空间不含宿主ROS进程，未由ps假称测得各ROS进程CPU；主机整体/分核计数有独立实测。

另做5秒本地图发现，不订阅图像/点云：RGB与depth各唯一camera发布者、唯一multi_agent_go2_rgbd订阅者；RViz只订阅/vlm_nav/camera_preview。原始/lidar_points仅转换器订阅、/lidar_points_fastlio仅LIO订阅、/cloud_registered_base仅障碍过滤订阅；过滤后的障碍点云5个订阅者为既有Nav2/监督/scan/RViz消费者。无先前额外两路完整点云监测；话题/节点多不等于高带宽额外订阅。3秒当前agent状态31条仍为同任务0916/PAUSED/local navigation not ready，nav_ready=false、localization_valid=true。

授权后的首次只读SSH采样10.042秒：机载8核总忙2.588%，最忙核10.611%，realsense2_came PID2705占单核8.465%、RSS约136.96MiB，loadavg约0.04/0.08/0.09，内存充足；eth0接收/发送无错误丢包，发送约75.86Mb/s，与配置640×480×15 Z16深度载荷73.728Mb/s加开销相符。控制电脑相对先前约219Mb/s少掉约114Mb/s，也接近RGB8载荷110.592Mb/s；这是RGB流中断的带宽旁证，不是单凭带宽证明具体上游故障。

直接日志证据：机载/tmp/realsense_roslog/realsense2_camera_node_2705_1792832525344.log显示17:02:13成功开启Depth Z16和Color RGB8，均640×480×15；17:08:08.285报告The device has been disconnected!，停止/关闭Depth与RGB；17:08:11.903重新发现同一SDK序列号D435，physical ID由2-2-2变2-2-3。普通用户dmesg受限，但随后只读journalctl -k可读历史：17:08:07出现usb_submit_urb returned -19和usb 2-2: USB disconnect, device number 2；17:08:08新SuperSpeed设备3重新枚举，之后多次uvcvideo: Failed to set UVC probe control : -32。时间上先于本任务17:08:09.779顶层取消，并与后续深度仍到达/RGB空窗口一致。已确认故障位于机载USB相机断连及恢复环节，当前证据不支持监听过多或持续CPU/接收缓冲过载作为解释。

只读USB现状为设备3、speed=5000、runtime_status=active、power/control=auto、autosuspend=2秒、当前runtime_suspended_time=0。不能据auto字段断定自动休眠致故障；当前设备计数不重建旧设备历史。线缆/接头、供电、端口或驱动/设备复位的具体触发机制尚未证明；未测USB电压、未做换线/拔插或运动复现。kernel中的tegra-xudc gadget控制器-517仅保留上下文，未当作D435 host断连的确定原因。相机ROS日志当前4096字节且尾部半行，未信号进程强制刷盘，不能以缺少后续行断言重连后哪一步停止。

证据build/go2-field/rgb-load-20261024/保存两端采样、ROS端点、agent状态、两次只读SSH结果、onboard-camera.log、onboard-kernel-usb.log、analysis.json及摘要；事件/计数/订阅者/负载均离线断言核对。未修改生产代码/配置、增加图像/点云订阅、重启/停止组件、改USB电源管理、SSH到Ranger、调用运动/控制服务、ARM/start/录包或提交；未做新的现场修复验收。仅重开任务层不能保证恢复断连后的RGB流。


### 2026-10-24 ABC目标改为纸巾／灭火器／椅子

用户要求改成A=纸巾、B=灭火器、C=椅子。已仅修改cooperative_ws/src/multi_agent_nav/config/scene.yaml的三个目标描述，并同步正式启动文档的当前目标说明；场景其他字段与改前逐项相同。正式协同入口显式使用源码配置目录，agent启动时将scene.targets传入runner/ABC识别，不需重建或覆盖已验证overlay。运行中的组件不会热更新目标；本轮未重启任何任务。Ranger本机配置未远程修改，后续新一轮两端须核对同一组目标。

实际load_config显式读取源码目录，确认新ABC映射；只读完整preflight在环境阶段因enp4s0无192.168.123.x地址退出1，尚未进入配置报告，不因此宣称整体预检通过。未改网卡/验证标志，未初始化现场ROS、增加订阅、请求VLM、SSH/连接Ranger、构建、ARM/start/录包/运动或提交。证据build/go2-targets-20261024.enmkh40o/含scene.before.yaml、preflight.log、validation.json；仅配置/说明变更，不为此运行全库测试。

### 2026-10-24 task 818d：Ranger路径监护报错传播至Go2，双机锁存暂停（离线分析）

最新实际启动任务818da5c1bb41489285350161ed0f62d1，入口build/go2-field/cooperative-task.ywukpdlk/start.log确认accepted=True，目标为A纸巾/B灭火器/C椅子。Go2初始相机ABC均NOT_DETECTED，候选VLM有效选中P6（free/150/170，map约0.435,-13.275）；19:38:58.044开始导航，19:39:07.654顶层Goal succeeded。19:39:07.743开始首段扫描Spin约-0.78rad，19:39:11.443成功，首个扫描图像stamp约19:39:11.496。三次保存的VLM请求（初始识别、候选选择、首视角识别）均api_returned/validation_success=true、error_type=null；初始和首视角ABC均未检测。事件共11条，最后为SCANNING，无完整SCAN_RESULT。Go2本轮controller未见Trajectory Hits Obstacle；supervisor首次NAV_READY=true之后无新的false记录，不沿用此前USB断连或局部碰撞解释本轮暂停。

用户提供19:46:36状态窗口：Go2 PAUSED，reason为peer paused: execution path monitor: execution path missing or receive timeout；coordinator PAUSED，reason为go2: agent paused: peer paused: execution path monitor: execution path missing or receive timeout；Go2 localization_valid/nav_ready/stopped均true，Go2及coordinator消息新鲜。这是用户提供的现场状态文本，非本轮新ROS采样。核对实际安装与源码agent/agent_node/runner/coordinator/adapters.go2一致；AgentMachine.receive_peer只在执行期接到同任务对端STOPPING/PAUSED时拼接peer paused，stop保留首次原因。因此可确认Go2的直接触发为收到Ranger的路径监护暂停，随后协调器联动暂停。协调器按Go2优先检查且锁存首个reason，其go2前缀不表示路径故障源头在Go2。纯状态机在本地定位/导航均有效时准确复现两级完整reason及STOP效果，并确认对端恢复后不自动续跑。

尚未获取Ranger本机实际adapter/路径配置、最后有效路径接收时间、报错瞬间run_state及原生导航/恢复/扫描终态。故只能将上游故障定位为Ranger报告的执行路径缺失或接收超时，不能确定是导航中实际断流/拒收，还是结束导航后、扫描或恢复阶段仍被监护；不能把Go2的/received_global_plan或5秒阈值当Ranger实参，也无依据修改Go2参数。下一步应在Ranger本机对齐首次监护报错、有效路径接收/清空及原生动作状态，核实实际topic、QoS和超时门槛；本轮未连接Ranger。

证据[analysis.json](../../build/go2-field/task-818d-peer-path-20261024/analysis.json)、同目录user-status.json及inputs/，保存入口/idle、事件、三份metadata和相关Nav2/Sport/agent/coordinator/supervisor日志快照与SHA256。已执行离线证据断言、安装一致性和纯状态机传播核对；首次自检夹具在未设置peer前进入执行健康检查而正确触发心跳丢失，修正夹具初始化后通过，不是生产代码修复。仅追加交接和分析证据，未改生产代码/配置、初始化ROS、订阅传感器/状态、SSH/连接Ranger、重启/结束组件、ARM/start/录包/运动、构建或提交；未做现场修复验收，PAUSED/stopped仍不认证实体停稳或DISARM。


### 2026-10-24 RViz候选/选点/目标/路径辨识更新（已构建，仅离线验证）

用户要求为当前start_go2_cooperative入口补齐RViz候选、VLM最终点、识别目标位置及导航路径，并最小修改、更新环境和脚本。已核实其candidate-rgb安装原有四路显示及live agent事件接线；复用该实现，仅提高颜色与尺寸辨识：蓝色P候选、橙色VLM选点朝向、绿色ABC已定位目标、青色路径。无有效地图位置的二维识别不画目标；停止/取消清候选/选点/路径、保留本任务目标，新任务清旧显示，语义沿用既有实现。未改导航、选点或传感器订阅。

本轮新成对安装为build/go2-rviz-20261024/overlay/install/local_setup.bash；Cooperative_env默认、scripts/start_go2_vlm.py的协同硬件安装预期与正式启动文档已统一更新，显式COOP_SETUP_FILE仍优先，Ranger默认不改。完整入口help及复用已有RViz提示说明原窗口File → Open Config加载源码config/go2_exploration.rviz；运行中的agent不会热更新。旧输出完整保留，未代为暂停/重启或启动下一轮。构建环境/命令、离线检查与后续操作判据见[runbook新增节](go2_runbook.md#2026-10-24-rviz标记辨识与默认环境更新)。

验证：修改前基线161通过；增强既有断言后7项预期失败，修改后实际新安装同组161项全部通过，未新增用例函数。成对构建退出0，两包成功；两包Python/RViz共54文件安装与源码逐字节一致，默认离线preflight退出0、installed各项matches_source=true、API环境齐全、live_gates=[]、motion_sent=false。证据build/go2-rviz-20261024/evidence/。未初始化现场ROS、增加相机/点云订阅、SSH/连接Ranger、真实模型请求、新ARM/start/录包/运动、重启/关闭现有终端或提交；现场RViz显示及真机导航未验收，历史硬件/安全问题仍未因此消除。


### 2026-10-24 安装不一致报错复核：旧显式overlay可复现，新环境通过

用户报告ValueError“安装与源码不一致”，要求更新新环境。只读核对当前分支/HEAD与默认环境后，对用户最初命令中的旧candidate-rgb、新go2-rviz-20261024显式路径及不带覆盖的默认路径分别执行离线preflight。旧路径退出1并准确复现该ValueError；其go2_exploration.py和exploration_display.py与当前源码不同。新显式路径和默认路径均退出0，13个受检项matches_source=true，实际导入位于build/go2-rviz-20261024/overlay/install，live_gates=[]、motion_sent=false。未获取用户报错终端的实际环境，故旧显式覆盖是已复现的原因，不声称已读取该失败进程的COOP_SETUP_FILE。

新成对环境已安装且默认配置正确，无需重复构建/覆盖旧输出。后续在启动终端export COOP_SETUP_FILE为上述新路径，再执行cooperative.sh go2 preflight；预检通过且按原流程正常结束旧任务后才启动完整入口，各组件窗口保持运行。仅source或改默认值不会覆盖用户已export的旧显式路径；显式覆盖本身仍保留。

证据build/go2-rviz-20261024/evidence/environment-recheck/中的三份preflight日志及results.json。未修改生产代码、跳过一致性检查、构建、重启/关闭组件、初始化现场ROS、SSH/连接Ranger、ARM/start/录包/运动或提交；用户现场失败终端切换后的实际启动仍待验证。


### 2026-10-25 A目标改为灭火器

用户要求将A改为灭火器。本轮仅将源码scene.yaml的targets.A由纸巾改为灭火器，同步正式启动文档；当前A=灭火器、B=灭火器、C=椅子。历史目标记录保留。实际load_config读取源码目录核对目标映射，并逐项确认其余场景配置不变；git diff --check通过。运行中组件不会热更新，Ranger本机目标配置仍需同步核对。本轮未构建、初始化ROS、重启任务、连接Ranger或执行运动，未做现场验证。


### 2026-10-25 选点提示词补充边界与信息量偏好（仅源码）

用户要求说明语义地图黑色边框为边界，尽量选择信息量大且不贴近边界的观察点，例如拐角附近。仅在adapters/go2.py的ABCClient.choose_candidate系统提示词中增加三句：黑色边框表示探索区域边界；结合历史优先预期新增信息量大、能看见更多尚未观察区域的候选；拐角附近或通道交汇处可优先，但不为靠近拐角而紧贴边界或墙角。保留本轮allowed_candidate_ids、JSON输出和不判断运动安全等约束，未新增距离阈值或修改候选生成/导航安全判定。

离线基线test_generated_candidates.py及test_candidate_camera.py共49通过、3失败，失败均为test_global_rotation_space_rejects_side_obstacle_and_stale_grid[253/254/255]缺少builtin_interfaces导入；修改后同环境结果相同。加载/opt/ros/humble/setup.bash，并以仓库根目录和cooperative_ws/src/multi_agent_nav为PYTHONPATH前缀后，使用/home/isee-pst/venv/co-nav-real/bin/python及PYTEST_DISABLE_PLUGIN_AUTOLOAD=1重跑原两文件，52项全部通过；未新增用例。提示词静态核对与git diff --check通过。

未构建或覆盖默认build/go2-rviz-20261024/overlay/install，默认安装中的adapters/go2.py目前与源码不同，后续启动前须按runbook在新输出目录构建并核对安装一致性；不要跳过一致性检查。运行中任务未热更新。未初始化ROS节点、请求真实VLM、连接Ranger、重启任务或执行运动，实际选点偏好效果待验证。


### 2026-10-25 更新选点提示词安装（完成安装，完整预检受网络门槛阻塞）

用户明确要求更新安装。新成对输出build/go2-prompt-20261025/overlay/install/local_setup.bash构建退出0，两包成功；旧build/go2-rviz-20261024及其他输出保留。Cooperative_env.bash默认、scripts/start_go2_vlm.py协同硬件安装预期、两份原测试夹具和正式启动文档已统一为新路径，显式COOP_SETUP_FILE继续优先。包含本轮新增边界/信息量提示，安装scene.yaml当前为A=灭火器、B=灭火器、C=椅子。

默认路径改前两份入口/环境测试基线41项通过；新安装从/tmp运行候选、RGB、环境、硬件入口和协同任务入口五文件共177项通过，11条第三方弃用告警，未新增用例，未跑全库。两包实际前缀/导入指向新输出，53个Python文件与源码逐字节一致；安装场景及提示词核对通过。bash语法及git diff --check通过。证据build/go2-prompt-20261025/evidence/，完整构建/测试环境、命令、后续终端切换顺序及判据见[runbook](go2_runbook.md#2026-10-25-选点提示词安装更新)。

默认完整离线preflight退出1，当前enp4s0无192.168.123.x地址，在基础环境加载阶段阻塞，未进入配置/installed/live_gates报告；不得把独立安装检查与177项离线测试写成完整预检通过。未改网络或绕过检查，未初始化ROS节点、连接Ranger、真实VLM调用、重启现有任务、ARM/start/录包/运动或提交。用户既有终端的旧COOP_SETUP_FILE需显式切换；运行中进程不会热更新，真实选点效果待现场验证。

### 2026-10-25 Go2协同改为目标坐标请求Nav2、实际终点导航（仅源码）

用户确认三步流程，并明确不设最低安全距离。Go2协同接近A/本机C时不再调用圆周8点select_approach，直接用识别目标map坐标请求Nav2；完整返回路径与终点通过机身/未知格/Keepout检查及上限距离预算后，以实际路径终点导航。物体坐标不强制作为可停车点检查。保留Nav2成功、停稳、定位有效、motion_terminal和距目标≤1m的最终门槛；0.03m仍是上限预算余量，不是最低距离。取消初始在目标1m内时的圆周缩半径，仍须真实Nav2成功；有限重试重新请求目标坐标。原单机和Ranger流程保留，不新增参数或依赖。

独立审查发现并已修正两项必要接线/性能问题：runner会对选定终点再次规划，现复用同一验收回调覆盖二次规划和重试，拒绝/异常进入stop并清理回调；逐点全图检查原会阻塞单线程回调并自致local costmap超龄，现只计算机身邻近网格。300组离线结果与原算法一致，实际472×578地图的200点检查原触发receipt_stale，修改后约0.026秒，独立复核约0.030秒，不当作实时性能认证。二次审查无遗留重要问题。

验证：原6文件基线208通过、3失败；新增21项先失败再通过，另4项真实runner故障测试先复现错误下发再通过，4项真实costmap测试覆盖free/inscribed/lethal/unknown。最终10文件306通过、显式排除上述3项基线失败（其中新增29项）；旧失败是当前vision_validated=true及已取消资源/退出区门槛与旧测试预期不一致，未顺手修改，未宣称全库通过。证据[目录](../../build/go2-target-plan-20261025/evidence/)保存改前文件与各轮日志，设计/范围见[实施记录](../superpowers/plans/2026-10-25-target-nav2-endpoint.md)，完整离线命令和安装限制见[runbook](go2_runbook.md#2026-10-25-go2协同以目标坐标请求规划仅源码未更新安装)。

尚未构建/安装本次源码，默认仍为build/go2-prompt-20261025/overlay/install，三个生产模块已与旧安装不同；下一步使用前必须新目录成对构建、验证安装和预检，不能跳过一致性检查或原地覆盖旧输出。未改Nav2参数/运动门槛、初始化现场ROS、SSH/连接Ranger、请求真实模型、新增传感器订阅、重启/停止组件、ARM/start/录包/运动或自动提交；运行中任务不热更新。离线测试不能证明本轮现场目标已经可达，不把此前APPROACH_UNREACHABLE写成已修复验收。

### 2026-10-25 目标坐标规划安装更新（构建及离线预检通过）

用户明确要求更新安装。新成对输出build/go2-target-plan-20261025/overlay/install/local_setup.bash构建退出0，两包成功，18.9秒；保留go2-prompt及所有旧输出。Cooperative_env默认、硬件Launcher默认预期、两份测试夹具及正式双机启动文档全部Go2显式路径已统一，显式COOP_SETUP_FILE仍优先，Ranger默认不改。正式文档原有不同位置的旧rviz安装路径及一处过时A=纸巾说明已同步为新安装和当前A=灭火器/B=灭火器/C=椅子。

验证：路径更新前两份原测试基线41通过；从/tmp加载新安装后13文件回归最终431通过、3项已知基线失败明确排除、11条第三方弃用告警、17.92秒，未新增用例。首次绝对路径--deselect不匹配实际nodeid而得到431通过/3项原失败，已保留日志并改用名称筛选，未修改生产门槛或旧失败测试。两包实际prefix及导入均为新输出，53个Python文件、安装scene.yaml和robot_go2.yaml与源码一致。Bash/Python语法、正式文档链接与路径及git diff --check通过。

默认离线preflight本轮退出0：13项installed全部matches_source=true，API环境齐全、live_gates=[]、motion_sent=false；这次已越过网络环境门槛，不沿用上一轮无有线地址的结果。仅离线检查，不认证现场停稳或运动效果。证据build/go2-target-plan-20261025/evidence/install-update/含改前文件、build.log、安装检查、两次回归和default-preflight.log。完整命令、环境与旧终端显式路径切换顺序见[runbook](go2_runbook.md#2026-10-25-目标坐标规划安装更新)。

旧终端若已export旧COOP_SETUP_FILE须显式切换；运行中组件不会热更新。本轮未初始化现场ROS、连接Ranger/SSH、请求真实VLM、新增传感器订阅、ARM/start/录包/运动、暂停/重启/结束现有组件或提交；新目标规划真机效果仍待后续授权实验验证。

### 2026-10-25 task 1b9d：Nav2返回路径，但机身footprint与禁入/静态障碍相交（离线分析）

用户要求分析最新报错。最新任务1b9de9a9317345a1ad9b93d9b9b9549c，入口build/go2-field/cooperative-task.11rn1_qo/start.log确认accepted=True，13项安装检查均匹配且指向go2-target-plan-20261025。事件8条，初始相机识别A=.95、位置有效，VLM api_returned/validation_success=true、error_type=null。目标map=(0.082685568,-11.243604499)，APPROACH_CANDIDATE_CHECK明确requested_pose等于目标、requested_distance=0；实际路径终点=(-0.017314426,-11.243604492,yaw=0)，距目标约0.10m，含XY容差.25与余量.03的上限预算约0.38m，满足1m门槛。唯一拒绝reason为planned path or endpoint footprint rejected，随后APPROACH_UNREACHABLE→APPROACH_FAILED，无APPROACH_SELECTED/NAVIGATING；尚未进入runner二次规划或接近导航。

离线用本轮地图和Keepout mask、实际参数footprint矩形x±.38/y±.18复核：终点中心所在格为静态free且不在禁入区，但保守机身检查覆盖136格，其中20格为Keepout、13格为静态occupied（两者有重叠，不可相加成33格）。用实际新安装Go2Adapter.costmap_candidate_check在重建的仅Keepout栅格及仅静态障碍栅格上分别检查，两者均返回collision。只是将重建地图的TF固定为map恒等、跳过现场时间戳，不初始化ROS；此复现不是故障瞬间动态costmap快照。源地图SHA与本轮inspect一致，inspect已确认keepout_applied=true，planner日志已收到mask，supervisor启动NAV_READY=true后无false记录。

本轮实际Nav2参数/tmp/vlm_nav2_go2_puop11au.yaml为GridBased=nav2_navfn_planner/NavfnPlanner、tolerance=.5、allow_unknown=true，矩形footprint与agent一致。NavFn按代价格规划，不保证矩形机身各朝向可通过；参见[Nav2官方footprint说明](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/)。因此“返回路径”与后续整机检查拒绝并不矛盾。planner的No planners specified...will use only plugin GridBased警告明确选择了唯一插件，不是本次规划失败。距离门槛不是本轮触发原因，也不是新增最低安全距离。

证据build/go2-field/task-1b9d-footprint-20261025/含输入快照及SHA256清单、可复核analyze.py、analysis.json和endpoint-footprint.png。确认终点本身足以违反现有地图安全条件；但完整拒绝路径/首个失败点及当时两幅动态costmap未保存，协同入口仍未接adapter.record_event，因此不能认定运行时首先拒绝的恰为终点，或排除路径中途亦有碰撞。未改生产代码/参数、构建/重装、初始化现场ROS/新增订阅、SSH/连接Ranger、请求模型、重启/停止进程、ARM/start/录包/运动或提交；未认证实体停稳、DISARM或现场修复通过。

### 2026-10-25 task 2bb4：右上墙角局部动态障碍与目标附近碰撞分开核对

用户报告APPROACH_FAILED但现场可通行，随后提供11:56:21截图指出右上墙角三角形障碍。本轮任务2bb4631f6010472b9af06217dc821b09（入口cooperative-task.zpgyhi_r）不同于上一轮1b9d，不能沿用上一轮静态/Keepout碰撞结论。事件明确Nav2返回终点map=(.276056621,-9.704722048,yaw=0)，距目标(.326056614,-9.754722056)约.07071m，预算.35071m≤1m；拒绝reason仍为planned path or endpoint footprint rejected，未进入接近导航。仅静态地图与Keepout复核本轮终点机身均无碰撞。

执行两次有时限的本地域只读采样：第一段约3.06秒订阅local/global原始costmap、TF及任务state，实际取得11:56:02网格，任务身份仍为2bb4、PAUSED，nav_ready/localization_valid/stopped均true；第二段约.98秒只读local voxel_grid和TF，取得12:05:18体素快照。均未调用服务/action、控制命令或新增相机/点云订阅，节点正常销毁。它们是失败后的快照，不能冒充11:51:58故障瞬间证据；state也不认证实体DISARM。

截图右上区域map x∈(.8,2.5)、y∈(-12.2,-10.7)内，11:56局部costmap有23个代价254格位于静态free且非Keepout区域；按同帧TF映射到几乎同时的global网格，这23处均非254（8处为0，其余为膨胀代价）。12:05局部体素层仍有6列marked体素位于该区域静态free、非Keepout处，其中5列在odom z第6层，中心约.20625m（不是地面高度判定）。这是局部动态层存在额外障碍标记的直接证据，不是单纯RViz叠图或正常墙体膨胀；标记周围膨胀再被3m局部窗口截断，与截图不规则楔形外观对应。当前参数local=VoxelLayer、global=ObstacleLayer、同用/vlm_nav/obstacle_cloud，膨胀半径.45；安装头文件确认膨胀距种子0为254，非零最多253。历史标记未被射线清除是优先怀疑，持续错误点云/TF等仍待验证，不能仅凭静态free断言实体不存在，亦未确定原始错误输入来源。

另一个独立事实：11:56全局网格上，终点矩形覆盖4个254格，坐标约(.335,-9.825)、(.385,-9.825)、(.435,-9.825)、(.385,-9.775)，靠近识别目标。这些格在静态地图中没有，但可能就是目标灭火器本身，不能和墙角疑似幽灵障碍混为一谈。用实际安装checker检查返回yaw=0、采集yaw≈1.499和走廊yaw=π/2均返回global collision，均覆盖这4格；终点在local窗口之外，local返回coverage_insufficient。因此仅更改朝向、放宽253或缩小膨胀不能消除当前终点的254碰撞。NavFn找到距目标7cm的中心线终点，不证明该处可停放整机。原拒绝路径和首个失败点未记录，不能断言墙角三角区域就是这次最先触发拒绝的点，也不能声称清除该区域即可成功。

证据build/go2-field/task-2bb4-footprint-20261025/：probe.py/probe.json、两幅原始costmap.npy、compare_layers.py/layer-comparison.png、local-classification.npz、probe_voxels.py/voxel-probe.json/local-voxel-grid.npy、analysis.json、实际nav2参数副本。离线断言核对23个额外局部障碍格、终点4个lethal格及后续6列体素均在静态free/非Keepout处通过；git diff --check通过。未改生产代码/参数、构建/安装、清图、重启、SSH/Ranger/bridge、录包、真实模型请求或运动，未验证修复。下一步应区分动态层残留与持续错误观测；即使墙角恢复，还须单独处理目标中心附近实际机身停车可行性，保留用户要求的机身/禁入区/≤1m门槛。

### 2026-10-25 Go2协同动态障碍：0.8秒×连续3周期确认（已实现、隔离验证，未部署）

用户明确要求平面地图同一栅格连续3个0.8秒周期探测到障碍才添加。实现位于新增temporal_obstacle_filter.hpp/.cpp及现有obstacle_cloud_filter节点：按云原始时间戳、云时刻map TF和/map实际原点/方向/分辨率投影计数，每周期每格最多一次，第三个完整周期结束才输出当前帧仍命中的确认点；漏周期/断流/时间回退/地图元数据变化重置，无历史点重放。原空间过滤云不变，Go2新增obstacle_cloud_confirmed；仅协同最终overlay将原云设为clearing-only、确认云设为marking-only，覆盖global ObstacleLayer和local VoxelLayer。Go2确认配置为enabled=true/period=.8/cycles=3，Ranger默认关闭，原单机costmap标记配置不变。静态/Keepout不受影响。

新增确认的map TF使用有界MessageFilter队列（沿用tf_queue_size=20，每帧默认最多等待.8秒），原云先发布且后续清除回调不受TF等待阻塞；缺map/TF不放行确认点，诊断WARN。审查发现原诊断仍报OK，已RED→GREEN修复。审查最初推断TF晚20–50ms会立即reset，但150ms晚到实际测试在修改前已通过，核对PCL PointCloud2重载内有1秒等待，故不采纳该原始根因；进一步回归确实复现同步等待阻塞后续raw云，改队列后通过。记录未按审查意见猜测性修改其他导航门槛。

构建输出build/go2-temporal-20261025/overlay/install/local_setup.bash仅用于本轮验证，默认入口仍为go2-target-plan-20261025，Cooperative_env和硬件/任务默认未更改。首次两包构建26.3秒，最终成对增量构建退出0；root新增nav_msgs编译依赖（原已有运行依赖），无新系统包。不能混用新协同参数和旧过滤节点。完整构建/检查命令、加载顺序及成功判据仅在[runbook新增节](go2_runbook.md#2026-10-25-go2协同障碍连续周期确认隔离构建未切换默认安装)。

验证：原配置基线最初28通过/3失败，修正underlay及/tmp ROS日志目录后其中1项环境失败通过，剩余2项为旧注册清单与旧验证标志期望，未修改。最终14个C++用例（新增8）、35个Python/Domain31用例（新增3）通过，2项既有失败明确排除。Domain31真实过滤节点覆盖缺map/TF、确认延迟、运动坐标一致性、原字段/header保留、TF晚到不阻塞后续原云、诊断WARN；真实costmap覆盖raw不能mark、confirmed可mark、raw可clear及正常退出。未把ctest条目数当用例数。证据build/go2-temporal-20261025/evidence/保存输入快照、失败与成功日志、审查修正、最终diff/SHA；git diff --check通过。

通常增加约1.6～2.4秒确认等待，实际入costmap另受TF/云/地图刷新影响，快速移动或细小障碍可能不满足同格连续命中。已存在的障碍仍靠Nav2射线清除，本次不主动清图、不增加自动过期。未切换现场安装、重启现场组件、初始化Domain0、连接Ranger/SSH/bridge、录包、请求真实模型或执行运动；无额外现场点云订阅。真实幽灵障碍抑制、移动障碍和导航效果仍待现场验证，不宣称此前APPROACH_FAILED已解决。

### 2026-10-25 Go2 接近迁移冻结单机行为（仅源码，离线验证）

按用户[附件计划](../superpowers/plans/2026-10-25-go2-single-robot-parity.md)，从完整 SHA `32eab6244ec4b483807f8ed68e8667f5495874bb` 的真实函数迁移接近状态与回调。起始 HEAD `df1a519df222c40f32e5222801d42fe20d8c4c44` 与附件协作基线一致，原工作区干净；在 `/tmp/go2-single-parity-20261025` 独立副本实施后同步限定源码。每个源函数、新位置与接口变化见[来源对应表](../superpowers/plans/2026-10-25-go2-single-robot-parity-sources.md)。

仅真实 Go2 且 share_c_coordinates=false 的 A/本机 C 接近启用新内部执行器：请求目标 x/y 和机器人当前 yaw，规划成功/非空后导航原请求目标，不替换终点、不作协作新增 footprint/距离预算否决、不二次规划或叠加 ABORTED 重试。内部停稳样本、取消终态与 0.81m 验收沿用源分支；首次 0.89m、原有一次距离重试，重试停止阈值按原函数取成功半径。用户补充确认的半径/动作超时仍读取 max_travel_radius + 0.50 与 goal_timeout，保留 active_motion_origin、goal_started 的原更新时机。参数验证仅做类型/有效值/基线核对，无新余量或运动准入规则。

内部成功和协作放行分开：外层身份、健康、停稳、动作终态、≤1m 尚未满足则等待，不能 READY，也不再导航/追加重试。保留双 READY、失联/PAUSE/CANCEL 与定位门槛；真实 Go2 本地模式的两层首条/5秒执行路径消息停止条件去除，观察点导航也适用，路径记录与真实 path_valid 保留。Ranger/mock/共享模式、候选选点/扫描、导航与障碍参数、地图、传感器保持。

接通被动动作/接近日志，记录原目标/规划终点/导航目标、原生状态、取消/终态、样本、年龄、最终距离及重试；迟到事件保留原任务记录器，新增诊断异常告警而不改变运动决策。独立审查发现漏掉原有其他运动所有者检查，已先复现再修复入口/规划回调/运动过程窗口，并防止扩大成任务开始前暂停；没有剩余已确认的重要审查问题。

验证：原五文件基线130通过/3失败；扩大原代码基线365通过/4失败；最终15文件428通过/相同4项旧失败，新增63项均通过，未deselect。第四项为扩大覆盖后复现的 independent_c 旧vision_validated预期，其他为local_tasks既有验证标志/资源/退出点门槛预期，均未顺手修改。差分参考来自冻结源码真实方法，但隔离了外围ROS/传感器依赖，只证明覆盖的接近子流程；真实adapter/runner/agent离线集成另行覆盖。命令、完整失败名、退出码和环境见[runbook](go2_runbook.md#2026-10-25-冻结单机接近行为迁移离线源码验证)，证据 `build/go2-single-parity-20261025/evidence/`。语法、文档链接及git diff --check通过。

未构建/安装/切换默认环境（仍为go2-target-plan-20261025），源码与默认安装不一致；运行中组件未热更新。未初始化ROS节点、Domain0/31现场流程、SSH/Ranger/bridge、录包、真实模型、ARM/start/运动、提交或推送。现有动态障碍三周期确认改动保留；疑似残留障碍、实际接近效果和双机避碰仍未验证，不把离线迁移写成现场问题已解决。

### 2026-10-25 单机接近迁移安装更新（构建通过，完整预检受有线网络门槛阻塞）

用户要求构建更新，并保持其 `start_go2_cooperative.sh` 实验流程。新成对安装为 `build/go2-single-parity-20261025/overlay/install/local_setup.bash`，两包构建退出0、23.1秒；包含本轮单机接近迁移及此前连续三周期障碍确认，旧安装/证据/进程保留。Cooperative_env的Go2默认、硬件Launcher安装预期、两份原测试路径夹具和正式双机文档统一新路径；显式COOP_SETUP_FILE仍优先，用户旧export须自行替换。

脚本仅增加新接近模块的Go2安装一致性检查，并替换两处默认路径。完整入口 `start_go2_cooperative.sh` 和调度 `start_go2_cooperative_task.py` 与改前逐字节一致，启动/ARM/握手/退出流程未改。新安装的share脚本与文档副本已同步，未覆盖旧overlay。

验证：路径变更前两份原测试41项通过；从/tmp加载新安装后18文件回归553通过/4项旧失败（沿用上一节完整失败清单）、11条第三方弃用告警，未排除用例，本轮未新增用例。新构建的两个CTest条目对应14个C++用例全部通过；未初始化ROS节点或跑Domain31现场流程。两包prefix及实际导入指向新安装，54个Python文件、根包config、协同scene/robot_go2配置和安装内入口脚本/正式文档均与源码一致，冻结接近参数核对通过。Bash/Python语法、文档链接和git diff --check通过。命令、环境、结果与用户执行顺序在[runbook](go2_runbook.md#2026-10-25-单机接近迁移构建与默认安装更新)，证据 `build/go2-single-parity-20261025/evidence/install-update/`。

默认完整离线preflight退出1：基础环境检查报enp4s0无192.168.123.x地址，只读核实该网卡DOWN/无地址，尚未进入installed/live_gates/API报告。独立安装核对通过不能当作完整preflight通过。未更改网络或绕过检查；用户恢复原有线环境后先按runbook显式export新安装并执行preflight，通过后再使用原完整入口，旧任务仍按原流程退出，不能叠加运行。

本轮未启动完整协同入口、SSH/Ranger/bridge、真实VLM、ARM/start/录包/运动，未重启/停止运行中组件或自动提交。旧进程不会热更新；实际接近、动态障碍与双机避碰仍未现场验收。

### 2026-10-25 取消 Go2 协同接近的 3.5 米半径终止（仅源码）

用户明确要求取消此前报错对应的3.5m半径保护。本轮任务2045f339c60749448a3093b879cfd42b的events.jsonl记录接近路径起点(3.31000017,-14.34999990)、目标(-0.06992071,-10.07189194)，直线距离5.4522m；失败原因为Robot exceeded active rolling-path radius，当时距目标1.8518m。规划允许直接导航远目标，但运行中仍限制离该段起点超过max_travel_radius+0.50，导致本次接近中途取消；这不证明目标投影或实际定位正确。

仅删除single_robot_approach.py中上述半径失败分支，适用于真实Go2私有目标接近；120秒动作超时、到目标停止、取消终态、停稳样本与外层健康/身份/双READY条件保留。原单机vlm_nav/vlm_navigator.py及共享配置未修改，参数基线校验与原字段生命周期保留。此处是对冻结单机迁移的明确行为差异，覆盖前述保留半径保护的当前要求；历史实施记录不改写。

验证：改前test_single_robot_approach_parity.py基线57项通过；将旧半径相等性用例替换为2项新回归，先在旧源码得到2项预期失败。删除分支后，接近差分、runner、target_nav2_endpoint和arrival_contract四文件110项全部通过（原保留108项、新增2项）；验证越过原半径仍继续接近、进入目标停止范围仍取消，以及超时仍失败。环境为source /opt/ros/humble/setup.bash及build/go2-coop/install/local_setup.bash，源码根目录与cooperative_ws/src/multi_agent_nav置于PYTHONPATH前部，使用/home/isee-pst/venv/co-nav-real/bin/python、PYTEST_DISABLE_PLUGIN_AUTOLOAD=1和pytest --noconftest -q -p no:cacheprovider。未初始化ROS节点或运行全库测试。

未构建、安装、切换默认环境、重启任务或现场验证；默认build/go2-single-parity-20261025/overlay/install仍含旧保护，与当前源码不一致，后续须按runbook在新输出目录构建更新，不能跳过安装一致性检查。运行中进程仍执行旧逻辑。未改用户已有的docs/双机协同启动与验收.md未提交内容，未自动提交。取消后远距离接近不再由这一固定半径终止，实际导航与避障效果待现场验证。


### 2026-10-25 取消接近半径保护后的构建安装与环境更新

用户明确授权构建安装并更新环境。新成对安装为build/go2-no-radius-20261025/overlay/install/local_setup.bash，两包构建退出0、23.4秒；包含取消Go2协同接近3.5米半径终止的修改，其余接近保护保留。Cooperative_env默认、硬件入口安装预期、两份既有测试路径夹具与正式启动文档已同步；显式COOP_SETUP_FILE仍优先。旧安装、现场进程、用户原有未提交文档内容均保留，未自动提交。

验证：路径变更前两份基线41项通过；从/tmp加载新安装后7文件235项通过，11条第三方弃用告警，本轮未新增用例；两个CTest条目对应14个C++用例全部通过。安装两包prefix/导入均位于新输出，54个Python文件、配置和入口/正式文档副本与源码一致。默认离线preflight退出0、14项matches_source=true、live_gates=[]、api_environment_complete=true、motion_sent=false。未跑全库或旧18文件回归，历史4项既有失败未据此销账。

完整构建/检查命令、环境与后续用户切换步骤见[runbook](go2_runbook.md#2026-10-25-取消接近半径保护后的安装更新)，证据build/go2-no-radius-20261025/evidence/。本轮未初始化ROS节点、SSH/Ranger/bridge、真实模型请求、录包、ARM/start/运动或重启/停止任务。旧终端须更新显式COOP_SETUP_FILE，旧进程不会热更新；新的长距离接近及避障效果未现场验证。


### 2026-10-25 人工19顶点新掩膜草稿（已生成，未应用）

用户提供build/go2-field/keepout-boundary-points/clicked_points_20261025T121903.172648Z_7be2a662.jsonl，19点均有效；按此前录点流程使用全部编号1→19→1，原始XY不移动、不重排，原始XYZ和时间戳完整保留。复用既有离线算法生成到maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261025_7be2a662/，包含boundary.yaml、clicked_points.jsonl、keepout_mask.pgm/png/yaml、boundary_preview.png、validation.json。旧掩膜及原地图保留。

顺时针、无自交、全部在地图内；连续多边形面积103.24200813012945m²。保持472×578、分辨率0.05m、origin=[-8.09,-20.8,0]。白色40300格不额外限制、黑色232516格禁行，无未知掩膜格；沿用半格对角线保守栅格化，无额外机身缓冲，不清除原地图墙体或动态障碍。矩形/凹多边形自检、顺逆序一致、独立点内判定、允许格四角、PGM/PNG回读与Nav2三值解码均通过；源文件及产物SHA256复核、实际KeepoutMask加载通过。二维预览正常，matplotlib仅报告不使用的Axes3D导入告警。

上一轮2045任务P3坐标(3.435,-14.275)中心落在新掩膜禁行格；新掩膜应用后该点应被筛除，不为保留旧候选调整人工边界。预览需用户核对，几何检查不替代现场可达性验证。证据生成脚本及consumer-check.json位于build/go2-field/keepout-mask-build.20261025_7be2a662/，该脚本是本轮一次性证据，不是长期入口。

当前robot_go2.yaml仍指向旧keepout_boundary_20261022_29ab5c38，未修改配置、安装、地图阈值或现场组件，未初始化ROS、订阅传感器、SSH、ARM/start/运动。此次仅产出可核对的新掩膜，尚未应用。


### 2026-10-25 用户确认19点掩膜并切换源码配置

用户明确确认7be2a662预览并要求切换。已仅将cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml的keepout_mask从旧keepout_boundary_20261022_29ab5c38改为maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261025_7be2a662/keepout_mask.yaml；解析前后逐项核对，其余配置完全不变。正式启动说明的当前掩膜标识同步为19点，保留用户已有未提交内容。旧掩膜和原地图完整保留。

已复核新产物SHA256，默认入口（清除COOP_SETUP_FILE/COOP_CONFIG覆盖）离线preflight退出0，报告config_dir为源码cooperative_ws/src/multi_agent_nav/config，keepout为7be2a662、472×578；14个安装模块matches_source=true、live_gates=[]、API环境齐全、motion_sent=false。证据build/go2-field/keepout-switch.20261025_7be2a662/含改前配置/说明、change.json及预检日志/退出码，git diff --check通过。仅配置修改，不构建或跑全量测试。

正式协同入口显式读取源码配置，故无需重建代码；默认overlay仍为go2-no-radius-20261025，已验证安装share中的旧配置/文档快照未覆盖。直接使用安装share配置或其他COOP_CONFIG副本的入口不会自动取得本次新路径，须核对实际config_dir。新掩膜validation.json与预览中的未部署标记保留为生成时历史状态，本节仅确认配置选择，不声明运行期costmap已应用。

未重启/停止硬件、Nav2、supervisor或agent，未初始化ROS节点、连接Ranger/SSH/bridge、ARM/start/录包/运动。运行中旧mask server/costmap不热更新，仅重开任务层不足以替换掩膜；下次须按原退出流程确认停稳和原生终态后正常退出，并重新启动加载新掩膜的硬件/导航及任务流程。现有入口在NAV_READY后核对supervisor掩膜路径，旧路径复用应被拒绝，不跳过该检查。真实范围应用及可达性仍待现场验证。


### 2026-10-25 task 3d38：Ranger定位失效触发暂停，Go2另有持续碰撞拒绝

用户要求分析当前实验暂停原因。最新任务3d38acc1d1b14826a8cb93ae21b8486b，入口build/go2-field/cooperative-task.noqzjd99/start.log受理成功，预检为go2-no-radius安装及新19点7be2a662掩膜。离线读取时任务仍在更新，随后20:36:30.435 Nav2收到客户端取消；events末尾STOP_COMMAND_SENT明确为peer localization/navigation invalid，最终VIEWPOINT_NAV原生CANCELED。

为区分导航不动和任务暂停，只执行一次4秒本机Domain0/enp4s0高层状态订阅；有效消息窗口20:36:44.899—20:36:48.522，Go2/Ranger各40条、协调器8条，均当前任务。协调器PAUSED原因为ranger: localization invalid；Go2 PAUSED/peer localization/navigation invalid；Ranger PAUSED/local health invalid: localization_valid。两端所有受采样状态localization_valid/nav_ready/stopped均true，goal_id为空，说明采样时健康字段已恢复，但暂停锁存。结合本地coordinator/agent的定位门槛与暂停首因保留逻辑，直接触发链为Ranger定位健康失效→Go2停止/协调器暂停；恢复字段不自动续跑。没有Ranger本机故障瞬间TF/定位日志，不确定失效具体子项，不将其归为Wi-Fi或AMCL漂移。

独立的Go2执行问题：起点1帧和两个到达点的各8帧扫描共17次识别均APPLIED、无API错误，A均NOT_DETECTED；未进入APPROACH，故不是已取消的3.5米接近保护。20:32:38.896首次DWB ObstacleFootprint/Trajectory Hits Obstacle，此后控制器patience exceeded、后退和恢复Spin也预测碰撞退出。任务记录7次观察点NAV_FAILED/ABORTED，随后下一点被对端健康暂停取消；多次Nav2起始位姿约(4.72,-12.73)。事件中的PATH_RECEIVE_TIMEOUT为记录项，本地单机迁移模式不据此暂停。碰撞格来源尚不能从日志区分新掩膜、静态墙体、动态障碍或定位误差，不能仅因刚换掩膜就归因掩膜过窄；更换目标也未解除当前局部堵塞。

证据build/go2-field/task-3d38-analysis-20261025/含status.json、analysis.json及本轮事件/入口/Nav2日志快照与SHA256；已离线断言核对状态原因、计数、识别结果、7次失败和最终取消。未修改生产代码/配置、SSH到Ranger、请求真实模型、订阅图像/点云、清图、重启/恢复任务、ARM/start/录包或发送运动指令；临时高层订阅已销毁，未跑构建/全量测试。stopped报告不认证实体停稳或DISARM。后续应分别核对Ranger首次定位门槛失效及Go2失败位置的实际costmap/footprint，不把两条故障链混为一个根因。


### 2026-10-25 选点提示补充避免低信息死胡同（仅源码）

用户要求新增“避免拐进死胡同”并优化表述。仅在adapters/go2.py的choose_candidate系统提示中增加：“优先选择通向连通区域、便于继续探索的观察点；避免仅能进入封闭尽端、到达后必须原路折返且预计新增信息有限的死胡同。”沿用候选白名单、原JSON输出和原导航检查，不增加新筛选算法或运动参数。该提示不是可达性/避障保证。

加载Humble与go2-coop消息underlay，以源码根目录和cooperative_ws/src/multi_agent_nav作为PYTHONPATH前缀，co-nav-real Python执行test_generated_candidates.py和test_candidate_camera.py，既有52项全部通过，未新增用例；git diff --check通过。未构建安装、请求真实模型、重启任务或运行真机；默认go2-no-radius-20261025安装中的adapter尚为旧提示，与当前源码不一致，下次启动前需新目录构建更新，不跳过一致性检查。实际模型选点偏好待验证。


### 2026-10-25 死胡同选点提示构建安装完成

用户要求构建安装。新成对安装为build/go2-deadend-prompt-20261025/overlay/install/local_setup.bash，vlm_nav与multi_agent_nav两包构建成功（23.4秒）；包含避免低信息死胡同的选点提示、此前取消协同接近3.5米半径终止及已确认的19点7be2a662掩膜配置。Cooperative_env默认、硬件入口安装预期、两份既有测试路径夹具与正式启动文档同步；显式COOP_SETUP_FILE仍优先，旧终端须更新export。旧安装与用户未提交工作保留，未自动提交。

验证：改前入口/环境基线41项通过；从/tmp加载新安装后候选/相机、接近与入口共6文件235项通过（13.17秒、11条第三方弃用告警），本轮未新增用例；两个CTest条目对应14个C++用例全部通过。两包实际prefix/导入均为新输出，54个Python文件、配置、入口及正式文档副本与源码一致，明确核对安装adapter含新增提示及安装配置的新掩膜。默认离线preflight退出0，14项安装一致、live_gates=[]、api_environment_complete=true、motion_sent=false，读取源码配置和7be2a662掩膜。未跑全库，历史4项旧失败未据此销账。

完整命令、环境与后续切换顺序见[runbook](go2_runbook.md#2026-10-25-死胡同选点提示安装更新)，证据build/go2-deadend-prompt-20261025/evidence/。未启动现场ROS、SSH/Ranger/bridge、真实VLM、录包、ARM/start/运动或重启/停止组件；旧进程不会热更新，新提示不构成可达性保证，真实选点与避障效果待验证。


### 2026-10-25 task 782a：离开首个观察点后持续旋转，未进入目标扫描

用户要求只诊断仍在实验中的旋转/未识别，并随后确认旋转发生在对端失效之前、失效后已停止。最新任务782a6716d8954035a009966836f4e4ad，入口cooperative-task.gkl8in0m使用go2-deadend-prompt安装和7be2a662掩膜。仅离线读取现有日志与FAST-LIO诊断文件，未订阅ROS、请求模型、暂停、重启或发送运动。

已确认：起点唯一一次目标识别返回APPLIED、A/B/C均NOT_DETECTED，API延迟6.87秒；其后两次请求为观察点选择，不能计作目标识别。首个P10=(0.435,-10.275)导航20:59:33成功，实际停靠(0.611639,-10.414852)、误差0.2253m，随即SCAN_SPACE_BLOCKED，跳过扫描。第二个P12=(4.435,-13.275)在20:59:40开始VIEWPOINT_NAV，至21:01:28因peer localization/navigation invalid取消，持续108.47秒，未到达。全程无SCANNING、无SCAN_SPIN；behavior_server仅初始化、无恢复Spin执行记录，不能把实体旋转归为45度视觉扫描或恢复动作。

从现有/tmp/spark_fast_lio_timeseries.jsonl按frame与SPARK_DIAG_FRAME日志时间关联，第二段1084个里程计样本净航向变化约-5.17圈；21:00:20—40与21:00:40—21:01:00各净转约一圈、净平移仅0.0490/0.0471m，支持用户反馈的近原地连续旋转。21:01:30—21:02:00净航向仅0.0104圈，与用户确认停止一致。该结果为LIO估计、非外部真值。

持续旋转发生于DWB导航控制阶段：controller有10次ObstacleFootprint/Trajectory Hits Obstacle，报No valid trajectories out of 3，但为间歇拒绝，未产生导航失败终态。任务只有导航成功且停稳/空间检查通过才启动扫描，因此第二段一直未提交目标识别，不是VLM迟迟不返回或扫描跳过等待。配置中非零线速度下限/上限均0.40、非零角速度下限/上限均0.50，自定义生成器限制低速候选，是需核查的控制约束，尚不能据此定为唯一根因。SimpleProgressChecker门槛0.10m/15秒；LIO代理计算后68秒13次超过0.10m、最长间隔7.90秒，提示转动伴随小位移可能刷新进展计时，但缺完整map/TF轨迹，不能声称精确复现Nav2判据。

离线原静态地图及新keepout在计划点/实际停靠点的0.42048m旋转圆盘均自由；不能据此排除融合costmap中的膨胀、动态障碍或TF差异，也不能直接归因新掩膜。缺该时段cmd_vel、DWB候选评分及融合costmap历史，尚未区分持续转向选择的确切触发原因。下一步应核对控制命令/里程计与局部代价图、轨迹评分，而非调整VLM等待时间或移除空间检查。

证据build/go2-field/task-782a-rotation-20261025/含events、入口/Nav2/Sport桥/LIO日志、LIO序列和生成参数快照、SHA256及analysis.json。事件数量/终态/扫描缺失和里程计窗口断言通过；无生产代码或参数修改、无构建/全库测试、无现场操作或提交。

### 2026-10-25 下一轮持续旋转记录命令已准备（未执行）

用户要求若当前证据不足，给出start_go2_cooperative.sh之前的记录命令。已核对现有record_go2_nav2_manual.sh缺evaluation/原始costmap与协同状态，本轮不修改该手动入口、不新增脚本或控制参数；在[runbook](go2_runbook.md#2026-10-25-持续旋转的下一轮定向记录)提供终端A先录、终端B原入口启动、正式停止/DISARM后结束录包并检查消息数的顺序。使用当前成对安装、Domain0/enp4s0、现有TF静态/map QoS覆盖、显式话题白名单和1GiB分卷；包括速度链、Sport、里程计、TF、DWB evaluation/路径、raw costmap、动作状态/反馈和协同状态，不记录完整点云或图像。

已读本机ros2bag参数实现及--help（未初始化ROS），核对Humble DWB官方默认evaluation发布、实际版本1.1.20与本机命名空间/速度重映射。命令语法、QoS解析和白名单静态核对通过；没有启动录包、参数服务或现场实验，现场消息是否完整与录制开销仍须本轮bag验证。源码及默认安装未变，无需构建。

### 2026-10-25 Go2 独立入口对齐（本worktree源码与隔离安装）

用户附件 `/home/isee-pst/Downloads/2026-10-25-go2-standalone-parity.md` 已实施。
当前交付目录 `/home/isee-pst/.codex/worktrees/a897/VLM_Nav`，已有隔离detached worktree，起始干净；
起始/结束HEAD均为 `1acae1d56e4e43dac29514f2244d439b760520bf`，未提交、合并或同步原仓库。
原仓库 `go2-cooperative-task-layer` 和旧安装保持；运行中组件不会热更新。

共享LocalApproachExecution仅提取原runner生命周期，复用未改动的SingleRobotApproach；共用原本地外层验收。
独立入口先观察当前画面，描述以--target为准，有效目标直接接近；否则沿用原探索/错误分支。
接通初始扫描轮询、odom关联、唯一结果消费者、取消迟到防护和提前返回后的停车确认。
外层未停稳只等待，验收后才输出TARGET_APPROACHED；独立结束不依赖peer、不进入C。
真实协同的身份、失联、双READY保持；限定模式沿用原路径接收监护选择，真实path_valid仍记录。
独立任务/动作诊断异常仅告警，VLM证据故障门槛不变。

新成对安装为本目录 `build/go2-standalone-parity-20261025/overlay/install/local_setup.bash`。
两包构建23.1秒、退出0；本worktree独立/协同Go2默认均切到该pair，保留COOP_SETUP_FILE覆盖，父子窗口核对实际前缀/模块。
不再加载10月18日功能overlay；消息underlay仍复用原仓库go2-coop。默认配置使用本worktree协同config，
arrival YAML中的原仓库robot.yaml→robot_go2.yaml未改，已核对与新安装逐字节一致及合并顺序。
独立总时限300s、协同总时限600s和执行器120s监护分别保留。

验证：改前18文件553通过/4失败；扩大5文件原源码137通过/2失败；新安装从/tmp执行24文件742通过/同6项旧失败、
11条第三方告警、退出1，新增52项全部通过，不排除旧失败。两类差分99项通过；新构建14个C++用例通过；
55个Python模块、根config与arrival来源安装核对通过。完整命令、六项失败名、实际安装和用户按序启动步骤见
[runbook本轮章节](go2_runbook.md#2026-10-25-go2-独立入口对齐隔离构建)，
[来源表](../superpowers/plans/2026-10-25-go2-standalone-parity-sources.md)记录固定SHA映射与任务复选框。
证据 `build/go2-standalone-parity-20261025/evidence/` 须保留。

未做全库/现场/Domain31节点测试、SSH、跨机桥接、录包、真实模型调用、ARM/运动、停止旧组件或自动提交。
真机接近、动态障碍消除与双机避碰仍待现场验证；不把本轮离线一致性当现场验收。


### 2026-10-27 单机与协同源码同步及共同安装

用户要求同步两侧更改。已以同一HEAD为基线，将a897单机对齐和原仓库后续协同修改三方合并，21个限定文件同步回原仓库
`/home/isee-pst/unitree_ros2/VLM_Nav`。保留双方未提交内容/历史文档；原分支go2-cooperative-task-layer及HEAD不变，未提交或推送。
原仓库build/go2-unified-20261027/evidence/snapshots保留两边修改前快照；旧安装、地图、日志和bag未清理。

单机和真实Go2本地协同共用探索及接近执行器；同时继承已确认的无3.5m接近半径终止、死胡同提示、19点7be2a662掩膜。
保留超时、取消、停稳/原生终态及外层验收，单机300秒/协同600秒及协同双READY差异保留。
FAST-LIO/AMCL/TF/Nav2和底层标定未改，未宣称持续旋转或现场障碍问题修复。

共同安装为原仓库build/go2-unified-20261027/overlay/install/local_setup.bash；两包22.9秒构建通过。
原仓库两个入口默认均为该pair；旧COOP_SETUP_FILE显式覆盖仍优先，启动前须更新。
a897只保留同份源码及前期证据，本轮未在a897构建同名pair，后续使用原仓库正式入口。

同步前24文件742通过/6旧失败；合并后源码及新安装743通过/相同6旧失败，安装20.07秒、退出1。
本轮新增两个入口长距离后到达验收测试均通过；更新既有半径预期并合入原仓库已有长距离/超时测试，不能将净增用例数当新增数。
针对性150通过，14个C++算法用例通过；55个实际导入模块、包prefix及参数来源/顺序核对通过。只读审查无阻塞问题。
完整环境/构建/证据/下一次启动步骤见[runbook同步安装节](go2_runbook.md#2026-10-27-单机与协同同步安装)。
未初始化ROS、启动现场预检、SSH/Ranger/bridge、录包、真实模型、ARM/运动或停止/重启已有组件；旧进程不热更新。


### 2026-10-27 正式文档三入口命令同步与新构建

用户指出正式文档完整启动/restart/单机命令仍混用。已保留用户新增三个入口的意图，统一为原仓库路径与配置，每块可独立复制；
移除先完整启动再预检并再次启动的重复顺序，明确按场景选择。单机旧a897路径移除，restart显式指定安装/配置，第6.1节与其他Go2命令同步。
默认环境、Launcher及对应测试夹具统一新 `build/go2-unified-entry-20261027/overlay/install/local_setup.bash`；原go2-unified-20261027保留。
源码同步回原仓库，分支/HEAD与用户未提交工作保留；未修改探索/接近、FAST-LIO、AMCL、TF、Nav2或恢复逻辑。

两包22.7秒构建退出0；修改前、修改后源码和新安装的入口/环境/restart三文件均132项通过，无新增用例；55个实际导入模块及参数来源核对通过。
完整命令、环境与切换条件见[runbook本节](go2_runbook.md#2026-10-27-三个入口命令统一与重新构建)，证据在原仓库build/go2-unified-entry-20261027/evidence。
未重跑全库或C++算法测试，既有6项失败没有销账；未执行文档中的现场命令、重启组件、SSH/桥接/录包/ARM/运动或提交。旧进程不热更新。

### 2026-10-27 VLM真实请求延迟抽样

用户要求测试VLM延迟。本轮使用已有API环境、qwen3-vl-plus及正式ABCClient，api_timeout=20秒、max_retries=0、trust_env=False。
源码vlm_client.py与adapters/go2.py逐字节匹配go2-unified-entry-20261027安装；未修改生产代码或配置。
复用历史e6a403任务640×480图像（A/B=灭火器、C=椅子），分别调用infer与infer_abc；复用782a任务决策538e1335的占据图、语义图和标注RGB调用当前choose_candidate。
选点三图与历史request.json实际提交字节一致；本轮不采集新图像。三类按轮交替执行，每类复用一个客户端及相同输入，各3次，无预热剔除。

| 调用 | 三次耗时（秒） | 平均（秒） | 中位数（秒） | 成功 |
| --- | --- | --- | --- | --- |
| 单目标识别 | 5.400 / 3.686 / 3.260 | 4.116 | 3.686 | 3/3 |
| ABC三目标识别 | 9.006 / 7.681 / 9.631 | 8.773 | 9.006 | 3/3 |
| 三图观察点选点 | 7.168 / 4.930 / 4.310 | 5.469 | 4.930 | 3/3 |

计时使用perf_counter包围正式客户端调用，包含输入编码、网络、完整响应生成及本地结果校验；不包含相机采集、ROS排队或证据落盘，不是首字延迟或纯模型计算时间。
成功指请求和结构校验成功，不代表识别正确或导航安全。全部低于配置20秒超时，但未设置独立性能验收阈值；固定输入小样本不能推出长期P95/P99或现场闭环延迟。
证据[results.json](../../build/vlm-latency-20261027/results.json)保存逐次结果、统计、时间及输入/源码SHA256；同目录probe.py仅为本轮一次性测量脚本，不是长期入口。
独立复算均值、9条成功断言、输入/源码摘要及安装一致性检查通过。未构建或跑全库测试，未初始化ROS、订阅传感器、SSH/Ranger/bridge、录包、ARM/start/运动或重启组件。

### 2026-10-27 Go2 五项最小可靠性修改与新成对安装

按用户附件 `/home/isee-pst/Downloads/Go2_minimal_reliability_codex_plan.md` 在控制电脑原仓库实施。
起止分支 `go2-cooperative-task-layer`、HEAD `f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1`，未提交/推送。
保留开始时本交接文档中未提交的VLM延迟抽样记录；快照在本轮evidence，旧日志、bag及安装未清理。

五项行为：生命周期查询超时后失效并取消当前Future、下周期重查，迟到回调受身份隔离；
动态costmap中心拒绝253/254/255、矩形机身及完整旋转圆盘仅拒绝254/255，原几何/覆盖不足分类保留；
Go2内外接近共用adapter新鲜里程计停稳结果，平面≤0.03m/s、yaw≤0.05rad/s、连续新样本0.5秒，
odom_max_age仍0.5秒，不再以3条样本或纯等待宣布停稳；停稳且动作终态、其余条件健康时可等待暂失RGB-D/图像时刻相机TF，
nav_ready真实失败字段不改，任务/扫描/规划原超时、取消和协同上层暂停保留；主目标检测但无可靠位置同朝向最多额外2帧，
默认target_depth_retries=2，必须更新RGB-D重新识别，attempt独立记录且每目标每方向仍一条代表结果，
补帧用尽无位置保留DETECTED证据并继续原方向/候选点。有效位置仍固定坐标接近。

停止和扫描退出语义是本轮有意改变，不再宣称与10月6日冻结版完全一致。
DWB/速度、底层定位/传感器、地图/Keepout/膨胀/机身、Ranger和协同协议无修改。
只读审查找出的待决规划恢复、等待期120秒规划超时、等待期API错误处理均已补red/green回归并复核；
不增加恢复状态机或额外距离/重试预算。正常成功模型响应仍用自身采集时同步深度/TF，恢复先确认当前感知健康，
不因模型消费时帧龄增加无条件重识别，也不把旧像素用于新深度。

新pair `build/go2-minimal-reliability-20261027/overlay/install/local_setup.bash`，vlm_nav与multi_agent_nav两包23.0秒构建退出0；
55个实际导入模块、两包prefix、根配置/arrival来源和8个协同配置文件核对一致。
默认环境脚本/Launcher仍选择旧go2-unified-entry-20261027，本轮未自动部署或切换进程；
使用新版本须显式COOP_SETUP_FILE，正式启动文档命令已明确新pair。旧安装与当前源码不一致，不能绕过安装检查。

基线26文件765通过/6旧失败，补核旧安装scan evidence29通过。最终源码及新安装30文件均870通过/相同6旧失败，
全部876项执行、pytest退出1，11条第三方弃用警告；新增76项全部通过。原规划/取消/终态/距离及协同暂停断言保留，
旧253边缘拒绝预期改为新语义，未删除相关覆盖来换取通过。两个CTest对应14个C++用例全部通过。
六项既有失败名、完整构建/测试环境、逐项行为与用户后续顺序见[runbook本轮章节](go2_runbook.md#2026-10-27-五项最小可靠性修改)。
证据 `build/go2-minimal-reliability-20261027/evidence/` 包含附件、开始时交接快照、逐项失败/通过、审查修复、JUnit、安装核对。

未执行全库、真实VLM/现场验收、SSH/Ranger/bridge、录包、ARM/reset_fault、运动或停止/重启现场组件。
已有隔离ROS单元测试不等于现场连通或实体停车验收；旧进程不热更新。下次先按原流程停稳/原生终态/DISARM和正常退出，
再按runbook在新终端显式选新pair、只读安装核对成功后走原单机人工确认入口；不据此保证真机不再出错。


### 2026-10-27 默认切换至五项可靠性版本

用户后续明确要求切换默认版本并更新正式文档第一节。CoVLM_env.bash、Cooperative_env.bash及start_go2_vlm.py
现默认选择build/go2-minimal-reliability-20261027/overlay/install/local_setup.bash，覆盖前节“默认仍为旧安装”的状态。
单机ROS_LOG_DIR同步本版目录，旧日志/安装保留；显式COOP_SETUP_FILE仍优先，旧终端请用正式文档第一节的完整命令覆盖旧值。
三个命令块（完整协同/restart/单机）已核对新pair和源码配置，默认说明同步；历史实施记录保留。

新路径夹具先复现3失败/45通过，修改后入口/环境/restart132通过、11条第三方弃用警告。
仅安装脚本/文档副本同步，不重新编译算法；55模块及参数来源只读核对继续通过。未重跑全库/五项行为全套，历史六项失败仍保留。
真实默认环境source因本机enp4s0没有192.168.123.x地址而被原预检拒绝；未改网络或绕过门槛。
需恢复原有Go2有线连接/地址后再按正式流程启动；离线默认选版测试不代表现场可启动。
未初始化ROS、SSH/桥接/录包/ARM/运动，未停止或重启运行组件，未提交/推送。
详见[runbook后续切换记录](go2_runbook.md#同日后续切换默认版本及第一节命令)，证据在本版evidence/default-switch/。

### 2026-10-27 选点reason最多20字符、构建更新及真实复测

用户要求压缩reason并构建复测，明确确认最多20个字符。当前任务观察点/家具选点提示新增中文短语、含标点最多20字符；
两解析器在原有字段/类型/白名单校验后`strip()[:20]`，超长不拒绝合法选点，原始response.txt沿用脱敏后完整保留，
解析结果和下游reason受限。ABC识别无reason，其提示/字段/max_tokens及旧单机frontier接口未改；不降低输出token上限以免截坏JSON。
本轮保留开始时已有五项可靠性源码、测试和文档修改，未调整导航行为；修改前diff/限定文件快照已保存。

新pair为build/go2-reason20-20261027/overlay/install/local_setup.bash；vlm_nav/multi_agent_nav两包22.8秒构建退出0。
CoVLM_env、Cooperative_env、Launcher默认及正式文档各入口已切换新pair；显式COOP_SETUP_FILE仍优先，旧进程不热更新。
旧安装保留。源码基线7文件223通过；新增12项边界测试并增强原始/解析证据断言，先复现7失败，最终源码及新安装各235通过，
11条第三方弃用警告。55模块、两包prefix、根配置/arrival来源核对通过；未重跑全库或C++测试，前节六项旧失败未销账。

从/tmp加载新安装，以同一qwen3-vl-plus、20秒超时、无重试及上一轮固定历史图像复测各3次，9次请求全部成功并通过格式校验。
单目标均值3.536秒（3.073–4.374）；ABC均值7.547秒（7.168–7.987）；三图选点均值2.695秒（2.356–3.185），
上轮选点5.469秒，本轮样本均值低50.7%。三次选点原始reason为17/15/15字符，均P12且无需截断；超长截断由离线回归验证。
第三次选点命中5504 cached_tokens，前两次为0；网络/服务和缓存未控制，不能据小样本保证加速幅度或归因全部改善。
本轮保存usage，ABC固定输入文本2286 tokens、图像302 tokens，输出262 tokens；与纯字符数口径不同。

证据build/go2-reason20-20261027/evidence/保存源码/安装测试、构建、安装核对、probe.py/results.json及输入/源码SHA256。
原图/选点三图与上一轮摘要一致；9条成功、reason上限、统计独立复算、输入/源码摘要检查通过。
完整构建记录和更新说明见[runbook](go2_runbook.md#2026-10-27-选点reason限制20字符)。文档完成后同步新安装中的文档副本。
未初始化ROS节点、采集新图、SSH/Ranger/bridge、录包、ARM/start/运动、停止/重启组件或提交；家具真实请求和现场闭环未测试。


### 2026-10-27 随机6张历史上传RGB的VLM复测

用户要求随机选6张以前上传过的图片再次测试。本轮按ABC三目标识别执行：从build/go2-vlm-experiments的真实views证据中，
仅纳入request.json图像字节与input.jpg一致、api_returned=true且有完整ABC描述的记录，按JPEG SHA256去重得63张。
无放回随机选6张、来自5次任务；随机种子14316737157508612738，候选池/抽样顺序/图片路径及摘要均保存在results.json，可复现抽样。
每张保留原记录的ABC目标描述：2张灭火器/水杯/篮子，4张灭火器/灭火器/椅子。每张仅请求一次、不预热、不重试。

从/tmp加载go2-reason20-20261027安装，实际ABCClient文件与源码一致；模型qwen3-vl-plus、20秒超时、max_retries=0，
同一个客户端顺序调用infer_abc。历史JPEG解码后按正式客户端重新编码；来源JPEG摘要与实际提交JPEG摘要分别记录，未宣称二者字节相同。
计时包含编码、网络、完整生成及格式校验，排除磁盘读图、相机采集/ROS和证据落盘；未运行三图选点，ABC无reason字段。

按抽样顺序延迟8.160、7.465、6.859、6.960、6.551、6.773秒；6/6成功，均值7.128秒、中位数6.910秒，最小6.551、最大8.160秒。
六次API报告cached_tokens均0；成功仅指请求与结构校验，不代表识别准确率。固定6张小样本不推导P95/P99或现场闭环表现。
证据[results.json](../../build/vlm-random6-20261027/results.json)包含原始脱敏响应、解析结果、token用量、逐张耗时和来源；
同目录probe.py为一次性测量脚本。抽样复现、六张不同图像、来源摘要、6条成功及均值/中位数独立复算检查通过。
本轮无生产代码/配置修改，无构建或全库回归；未初始化ROS、采集新图、SSH/Ranger/bridge、录包、ARM/start/运动或重启组件。

### 2026-10-27 暂时取消选点附带RGB，更新默认安装

用户明确只取消观察点选点第三张RGB，保留占据图、语义图及目标识别所需RGB。
正式探索选点不再调用selection_camera_snapshot，不传camera_snapshot给render_selection，故不产出本轮相机投影图/元数据；
choose_candidate仅读取/发送images前两项，并删除RGB相关提示和图例。即使旧snapshot仍带第三张相机路径（含文件不存在），也不会读取上传。
目标识别RGB、感知健康/停稳/导航门槛、候选白名单及reason≤20字符保持。投影辅助函数/测试及历史证据保留，未增加配置开关。

已保留开始时全部未提交修改与旧安装；新pair为build/go2-map-only-20261027/overlay/install/local_setup.bash。
两包23.1秒构建退出0，默认环境、Launcher和正式文档均切换；显式COOP_SETUP_FILE仍优先，旧终端/旧进程不自动更新。
源码基线9文件276通过；更新原有两项行为测试，增加第三图缺失场景，先复现3失败，最终源码及新安装各277通过，11条第三方弃用告警。
测试核对只上传两图的实际字节、旧第三路径不读取、探索阶段不取相机/不生成相机证据；原识别/扫描证据与入口回归保留。
其中有原有Domain31本地ROS替身测试，不是现场节点或真机验收。55模块、包prefix、根配置/arrival来源安装核对通过；
AST复核ABCClient除choose_candidate外全部方法未变，目标识别接口保持。未跑全库/C++测试，六项历史失败未销账。

证据build/go2-map-only-20261027/evidence/保存修改前快照、基线/red/green、构建和安装回归/核对；
完整环境及更新步骤见[runbook](go2_runbook.md#2026-10-27-选点暂时取消相机rgb)，最终文档同步新安装副本。
未请求真实VLM、采集新图、连接现场ROS/SSH/Ranger/bridge、录包、ARM/start/运动、停止/重启组件或提交。
当前两图选点真实延迟及效果待测；目标识别仍需图像，不能据取消选点RGB跳过相机健康门槛。


### 2026-10-27 两图选点真实延迟复测

用户要求再次测试延迟。本轮从/tmp加载go2-map-only-20261027新安装，复核ABCClient和基础VLM客户端与源码一致，
对此前782a任务538e1335决策相同的occupancy.png/semantic.png、目标/候选/历史记录顺序请求6次，使用同一个客户端、不剔除预热。
模型qwen3-vl-plus、20秒超时、max_retries=0、enable_thinking=false、选点max_tokens=512及reason≤20字符保持。
逐次在实际调用前断言仅2张上传图片，提交字节SHA256与两张原地图及前轮证据一致；不上传相机RGB。

6/6成功，依次2.570、2.469、2.134、2.765、2.049、2.252秒；均值2.373秒，中位数2.360秒，最小2.049、最大2.765秒。
均选择P12，原始reason均“通道交汇处，视野开阔，新增信息量大”（17字符），无需截断。
本轮输入5546 tokens（图像5020、文本526），输出26 tokens；此前三图输入5949 tokens。
cached_tokens前两次0、后四次5120。相较此前三图3样本均值2.695秒，本轮均值低11.94%；
小样本、跨时间请求和缓存未控制，不能将全部差异归因去掉RGB或保证长期加速幅度。
计时包含读取/编码图片、网络、完整生成及校验，不含地图渲染、ROS排队和证据落盘。

证据[results.json](../../build/vlm-map-only-latency-20261027/results.json)保存实际上传图像摘要、逐次返回/usage、安装路径/摘要和统计；
historical_context仅用于记录之前三图请求的原始上下文，不是本轮实际上传图像列表；本轮images/image_count明确为两张地图。
同目录probe.py为本轮一次性测量脚本。6条成功、两图数量/提交字节、reason上限、均值/中位数独立复算及原图摘要检查通过。
未更改生产代码/配置、构建或跑全库；未重测ABC识别、初始化ROS节点、采集新图、SSH/Ranger/bridge、录包、ARM/start/运动或重启组件。


### 2026-10-27 新安装三目标识别6张同图复测

用户要求也测试三目标识别。本轮从/tmp加载go2-map-only-20261027新安装的ABCClient及基础VLM客户端，均与源码逐字节一致；
沿用vlm-random6-20261027抽取的相同6张图、顺序及各自ABC描述，每张一次，同一客户端顺序请求、不剔除预热。
模型qwen3-vl-plus、enable_thinking=false、20秒超时、max_retries=0、ABC max_tokens=1200；每次实际上传1张RGB，取消选点RGB不影响此接口。
输入历史JPEG解码再由正式客户端编码；本轮实际提交JPEG摘要、提示字符数和目标描述均与上次逐项一致。

6/6成功且通过格式校验，依次7.874、6.942、6.858、6.043、7.282、6.536秒；均值6.922秒、中位数6.900秒，最小6.043、最大7.874秒。
API报告cached_tokens六次均0；相同图上次均值7.128秒，本轮约低2.9%，小样本/跨时间服务波动下不解释为识别代码优化效果。
每张请求输入2588或2590 tokens，输出234或262 tokens。耗时包含编码、网络、完整生成及校验，排除读图、相机采集/ROS及证据落盘。

证据[results.json](../../build/vlm-abc-retest-20261027/results.json)保存逐次脱敏原始响应、解析ABC、usage、图片/安装来源和统计，
同目录probe.py为一次性脚本，verification.json记录输入一致性、六条成功、原图/上传JPEG摘要及均值/中位数独立复算检查通过。
本轮无生产代码/配置修改、构建或全库回归；未初始化ROS节点、采集新图、SSH/Ranger/bridge、录包、ARM/start/运动或重启组件。
成功仅指请求及格式校验，未人工标注核验识别准确率或验收现场闭环。

### 2026-10-27 仅VLM请求子目录改为递增序号

用户明确“只改views/decisions内的请求子目录”。新请求改为<原实验目录>/views/0001、0002…和decisions/0001、0002…，
两个父目录独立计数，已有最大数字名称后续号；排他mkdir处理并发占号，数字文件/符号链接不会被覆盖或跟随。
历史UUID目录不重命名/清理。顶层runner/go2_check命名及原managed保留策略与本轮开始时完全相同；任务/命令/扫描/决策ID仍保留在原元数据/事件。

共享分配函数放在已有experiment_records.py；ScanEvidence.begin仍在原视觉worker创建目录，不搬到ROS回调，
未分配目录的取消记录evidence_dir=null。render_selection返回序号目录中的两图，snapshot.evidence_directory指向该目录，
不再用decision_id拼文件路径；家具选点复用分配函数。旧诊断入口vlm/selection请求使用同一子目录规则，显式输出路径不变。
两图选点、reason≤20字符、识别RGB、任务身份、感知/导航门槛及原证据写入异常处理保持。

新pair为build/go2-numbered-results-20261027/overlay/install/local_setup.bash，两包23.2秒构建退出0；
默认环境、Launcher及正式文档已切换，显式COOP_SETUP_FILE仍优先，旧进程不会热更新。旧安装及所有开始时用户修改保留。
基线14文件411通过/3旧失败；最终15文件源码及新安装各416通过/相同3旧失败，新增5项编号用例全部通过，11条第三方弃用警告。
3项均为test_local_tasks.py的local_assets、malformed resources和malformed exit_points旧门槛预期，完整名字及环境见
[runbook](go2_runbook.md#2026-10-27-vlm结果目录序号命名)。未跑全库/C++测试，其他历史失败未销账。
55模块、prefix和参数来源安装核对通过；顶层命名文件/测试逐字节与本轮基线一致，原experiment_bundle及保留函数AST一致。

证据build/go2-numbered-results-20261027/evidence/包含修改前快照、红/绿日志、构建/安装回归、安装核对和增量diff。
用户确认只改子目录之前曾在工作区尝试顶层编号，已在构建前撤回，未部署或触碰真实结果目录。
本轮包含原有Domain31本地ROS替身单测，未请求真实VLM、采集新图、迁移旧目录、连接现场ROS/SSH/Ranger/bridge、录包、ARM/start/运动或重启组件；未提交。
最终文档同步新安装副本，真实下次请求采用新命名需按原流程使用新安装启动。

### 2026-10-28 Go2按2m有效视距排除重复探索候选

用户报告当前Go2相机有效视距可达2m，要求修改探索点选择减少区域重复；本轮按“完整扫描后排除2m内无遮挡候选中心”实现，
不是两次视场圆盘完全不相交。保留原约1m候选网格和家具观察距离，不用4m稀疏网格，以免漏掉转角/窄区域。
robot_go2.yaml的generated_candidates新增coverage_range=2.0；未配置默认0（关闭），Ranger不启用。

仅generated_candidates_vlm、ABC各方向结果齐全且无API/验证错误时计算覆盖；扫描方向在圆周上的最大间隙须≤45度，
当前正式8方向满足。以导航成功后、扫描前的实际停靠map位姿为圆心，检查候选中心≤2m，并按静态地图全部非自由格
（含unknown）及家具多边形检查视线遮挡；贴栅格边界亦保守检查。计算复用视觉worker，避免在ROS回调执行逐格几何。
覆盖点单独记为covered、记录covered_by及scan_pose，不伪造该点的扫描结果；后续候选白名单、图像灰色标记、RViz和耗尽判定同步识别该状态。
提前发现目标的未完整扫描、失败扫描和非环扫不扩展覆盖；原目标接近/安全门槛保持。

验证：补齐ROS Python underlay后的三文件修改前基线134通过；未加载underlay的首次运行127通过/7个缺ROS模块的环境失败，
补环境后全部消失。新增首批8项先失败后通过，贴未知格边界用例另先复现失败后修复。
最终六文件197通过，其中新增13项，见[runbook](go2_runbook.md#2026-10-28-go2探索候选按2m视距去重)；git diff --check通过。
证据build/go2-coverage-20261028/evidence/保留两个核心模块的修改前快照及red.log/green.log。
保留全部用户原有未提交修改，无提交、构建、安装切换、VLM请求或现场验证；默认安装仍为go2-numbered-results-20261027，
本轮源码需后续构建后才能在已安装入口生效。

限制：这是以机器人中心和静态平面视线近似相机覆盖，45度方向间距沿用正式扫描配置，未重新实测相机水平视角/外参/动态遮挡。
视场仍可部分重叠，不能作为逐像素可见性或无漏搜保证；本轮不宣称现场探索验收通过。

### 2026-10-28 2m探索去重双包构建及默认环境切换

用户追加授权构建、切换环境并视需要更新第一节命令。实际文档为docs/双机协同启动与验收.md；本轮更新其第一节及
其他分步命令的安装路径，CoVLM_env/Cooperative_env/Launcher默认同步为build/go2-coverage-20261028/overlay/install/local_setup.bash。
旧安装和用户已有改动保留，显式COOP_SETUP_FILE仍优先，旧进程不会热更新。

两包24.7秒构建退出0，消息接口复用build/go2-coop/install/multi_agent_task_msgs。源码及新安装各245通过，
其中探索相关197项（含上轮新增13项）及环境/Launcher48项；安装回归在/tmp运行，不添加源码PYTHONPATH。
55个模块及prefix/根配置/arrival来源核对通过，安装协同配置与源码逐字节一致、coverage_range=2.0。
新安装直接--offline报告518个静态候选，go2_local_gates/live_gates/single_robot_experiment_gates均为空、motion_sent=false。

正式默认preflight因当前enp4s0无192.168.123.x地址在环境加载阶段退出1，不能报告完整预检或环境source通过；
未修改网络或绕过门槛。安装路径选择已切换，但现场完整启动须先恢复真实网络条件，旧任务仍按原流程停稳/DISARM/退出。
完整构建环境、检查与限制见[runbook](go2_runbook.md#2026-10-28-2m探索去重构建与环境切换)，证据仍在build/go2-coverage-20261028/evidence/。
文档第一节Bash语法、安装路径及git diff --check通过，最终文档同步到新安装share。
未跑全库/C++测试、请求模型、采新图、SSH/Ranger/bridge、录包、ARM/start/运动、重启组件或提交。

### 2026-10-28 完整入口inspect反复失败：定位断流与当前百兆链路

用户报告完整入口先等待NAV_READY 120秒超时，随后两次multi_agent_go2_check检查失败。本轮只读日志/源码及本机网卡计数，
未启动ROS检查订阅、SSH、bridge、录包、ARM/start/运动、网络重配或进程重启；未改生产代码和安全门槛。
第一次21:08启动的AMCL持续提示Please set the initial pose，属于初始位姿未设置；后续轮次已完成定位并短暂NAV_READY=true，不能混为同一根因。

21:16轮用户贴出的检查报告：odom_age_s=9.919（门槛0.5s），定位与local costmap变换均ExtrapolationException；
costmap消息本身仅约0.384秒，active_map_matches/keepout_applied/depth_units_verified=true。
RGB最后时间21:16:19.264、depth仍到21:16:28.318，RGB-D不新鲜；stopped=false是无新鲜里程计不能证实停稳，不证明实体移动。
DISARMED为准备阶段正常状态；execution path missing为未下发任务时的通用reason，不是此次inspect退出条件。

最新目录cooperative-task._mvfc4gt（21:21:42）仅preflight.log；现有Launcher.inspect在command非零异常后尚未写inspect.log，
所以失败JSON仅原终端有，不能假称读取到了这次完整报告。
最新检查器python_63986于21:23:29产生图像TF等待日志；supervisor 21:23:23.697就绪，21:23:30.697起
system_ready_value失败，随后obstacle/scan/odom_to_base_link/map_to_odom不新鲜，21:23:40.197恢复。
转换器21:23:31.999→21:23:36.997期间received 546→596、published一直518、incomplete 28→78：该5秒50帧全部缺点拒收。
上一轮21:16:20→25同样received 396→446、published固定383、incomplete13→63。
已核对外部转换器源码：incomplete计数对应TooFewPoints（输入点数低于min_points），不是凭计数推定某个具体丢包位置。

本轮ethtool和/sys/class/net/enp4s0/speed双重核实当前100Mb/s full，地址192.168.123.222/24；本端及对端均advertise 1000baseT/Full。
现有相机入口配置640×480×15，按此前确认RGB8/Z16原始载荷估算合计184.32Mb/s（不含协议/雷达），超过当前百兆接收能力。
两次检查器开始接收图像即缺点、结束后恢复与带宽瓶颈相符，是当前首要原因线索；失败当刻的链路重协商历史不可读，
不能宣布已证明是某根网线或交换机损坏，也不能排除机载USB等并发故障。
被动8.008秒采样当前RX38.04/TX87.13Mb/s、本机网卡errors/dropped零新增；采样发生在失败之后，不是失败期间流量，
也不排除上游拥塞丢包。journalctl当前权限不可见系统内核记录，未提权。

证据build/go2-field/nav-ready-diagnosis-20261028/保存两轮关键日志及link-evidence.json/ethtool.txt。
待现场按原流程停稳/DISARM后检查线缆/接头/中间设备，恢复1000Mb/s再做输入验收；不强制速率或放宽门槛，不盲目重复完整任务。
本轮未构建/跑代码测试；文档git diff --check通过。当前真实数据链路仍未验收。

### 2026-10-28 最新10292c任务DEPOSIT与peer暂停分析（只读）

用户报告双方已完成A/B、Ranger C输出TARGET_APPROACHED，Go2仍找C时显示DEPOSIT，最终协调器
PAUSED reason=go2: agent paused: peer paused: peer heartbeat timeout。
已核对最新start.log：build/go2-field/cooperative-task.hpshhawu，accepted=true，task_id=10292c45743a45b08cb598c830ff7be0。
本机agent/agent_node/coordinator/planner/runner五模块与go2-coverage-20261028安装逐字节一致。

阶段语义：share_c_coordinates=false；双方ready且命令完成后SEARCH_AB→SEARCH_C，只要任一机器人已有本地有效C观察，
协调器即可进入DEPOSIT。该阶段内尚无本地C的机器人仍执行SEARCH_VIEWPOINT(C)，已有本地C的执行APPROACH_DEPOSIT；
仅双方deposited且stopped、无未完成命令才DONE。所以DEPOSIT不表示Go2已到C，也不证明阶段跳转错误。

Go2第1命令events.jsonl第551行TARGET_APPROACHED，实测目标距离0.6811667m，A完成有本机证据。
第2命令目标椅子：第一处8方向C全NOT_DETECTED；第二处7个NOT_DETECTED、1个DETECTED，
该C confidence=.85但position_valid=false、reason=insufficient_valid_depth_samples:0/1；第722行TARGET_DETECTED_UNLOCATED后继续选第三点。
不能把DETECTED当作C已可靠定位；本机没有C的TARGET_APPROACHED记录。
第1457行STOP_COMMAND_SENT reason='peer paused: peer heartbeat timeout'，monotonic=36271.995289582；
第1459行原生导航CANCELED，36272.018077664。最后路径header约21:54:23.553，属于Go2第三点VIEWPOINT_NAV取消，非C接近失败。
取消前12秒259条执行事件，最大相邻记录间隔0.0903s：未见Go2执行循环长停顿，但这不能证明跨机心跳成功送达。

按本机receive_peer代码，“peer paused:”是对端同任务STOPPING/PAUSED reason的转发；本机直接心跳超时文案为
'peer heartbeat lost for current task'。因此现有证据支持Ranger先报告peer heartbeat timeout，Go2接收后停车，协调器传播全局PAUSED。
具体是Go2发布、Go2 bridge、Wi-Fi Domain40、Ranger bridge、Ranger回调/消息校验哪一环，暂无Ranger端原始记录/历史心跳证据，不能定唯一根因。
Ranger TARGET_APPROACHED仅用户报告，尚未核对其协同deposited=true；完成C后仍须保持agent/bridge及心跳，待双方完成。
已向用户索取Ranger TARGET_APPROACHED至超时的原始日志及agent/bridge是否持续运行，未擅自SSH远端或启动订阅采样。

当前本机enp4s0 speed=1000、wlo1=192.168.137.128（本次只读时值），不可将此前百兆传感器异常套用于这次高层心跳。
证据摘要build/go2-field/peer-pause-10292c-20261028/key-events.json保留对应事件及原行号。
未修改生产代码/门槛，无构建或测试重跑，无重启、录包、SSH、bridge/ARM/start/运动；git diff --check通过。


### 2026-10-29 完成状态保留、2.5m去重及新安装

用户确认要求：全局完成且本机动作终止后，保留完成结果，后续健康异常独立显示；Go2视野假设更新为2.5m。
开始分支go2-cooperative-task-layer，HEAD f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1；保留所有原有未提交修改。

已确认代码缺陷：旧独立C agent在全局DONE后仍保留committed/executing及task_started监护。
离线MockMission约1.6秒DONE；健康心跳持续到任务启动602秒，Go2因task timeout变PAUSED，Ranger跟随暂停，协调器仍DONE。
DONE后失联同样可复现。用户旧实验证据不足以断言那一次就是600秒超时；只能确认代码机制及其与现象相符。
修改前证据build/go2-coverage-20261029/evidence/done-result.json、check_done.py、completion-red.log（初始14失败/1通过）。

agent增加同任务完成锁定：有效SYNC(DONE)+本机ready/deposited、IDLE/SUCCEEDED、停稳/动作终态、无活动目标或本地运动，
清除已结束执行上下文。后续健康字段保持实时，reason前缀post-completion health，只诊断不改成功。
新任务仍需新task_id和健康SYNCING握手；迟到结果/旧命令不复活任务。仅一方完成仍执行原监护。
显式stop仍派发adapter停车，runner不将完成展示改成STOPPING；agent退出等待停稳/终态；协调器DONE后暂停/退出保留DONE。
默认2Hz状态发布定时器不变。协议消息未改；Ranger实际独立实现尚需同步修复，未SSH/远端部署。
Go2 coverage_range从2.0改2.5，只覆盖完整成功环扫后2.5m内无遮挡候选中心；墙后、失败和不完整扫描保留。

新默认pair build/go2-completed-20261029/overlay/install/local_setup.bash，根vlm_nav与multi_agent_nav构建24.1秒成功；
默认env、Launcher及正式启动文档第一节和分步命令均切换，旧overlay保留。旧终端显式覆盖和旧进程不会热更新。
完整构建记录及行为说明见[runbook](go2_runbook.md#2026-10-29-完成状态保留与25m去重安装)。
新增18项完成测试+4项边界参数用例通过；源码503通过/5失败另Launcher84通过，安装对应587通过/5失败。
5失败在未修改任务模块的隔离基线同样复现，为现有live_gates与旧断言不一致；详情runbook，未顺手修改配置或断言。
55安装模块字节/prefix/arrival一致；安装配置2.5；offline三类gates=[]、motion_sent=false。
默认环境实际source被enp4s0无192.168.123.x地址阻止，退出1；只确认默认安装已切换，完整现场环境未通过。
证据build/go2-completed-20261029/evidence/；未改网络、启动现场/真机运动/真实VLM/录包/SSH/bridge或自动提交。
下一步为Ranger同等完成状态修复，以及有现场授权、网络恢复后两端实机完成后长时间状态验证；离线通过不能替代现场验收。

### 2026-10-29 信箱联合复核昨日10292c心跳暂停

按用户指定会话01a18280-d8ef-7fc1-9ad6-f0d23b6713fa追溯，实验task_id为
10292c45743a45b08cb598c830ff7be0，信箱任务继续使用link-check-20261029。
Ranger通过信箱在其本机核对原始日志；Go2读取本地原始事件、当时start.log及保留的
build/go2-coverage-20261028旧安装，并独立重算Ranger传回的67条时间戳摘录。

Ranger证据目录（远端路径，不是Go2本地文件）：
/home/isee601/ws/VLM_Nav/build/ranger-local/acceptance-20261015_161341/cooperative-startup-20261028_211058-Zj2NEQ。
state.log:265090消息stamp=1793195663.3753123（北京时间21:54:23.375312），
reason实际位于265115，为peer heartbeat timeout；本机deposited=true、健康字段true。
peer.log:168862消息stamp=1793195665.2837443，Go2 reason为peer paused: peer heartbeat timeout。
21:54:15～28，peer.log有67条同任务Go2消息，最大发送stamp间隔1.499368668秒；
Ranger报告state.log有56条，最大间隔0.501472235秒。两机stamp差不能当作网络延迟。
本机events.jsonl:1457/1459确认收到peer暂停后停车并CANCELED，与历史分析一致。

关键限制：peer.log来自独立ros2 topic echo，没有本机到达时间，也不是agent回调记录。
Ranger源码以本机time.monotonic()更新/检查peer_received，3秒阈值，不直接使用消息stamp；
C完成不清空该值。Ranger报告离线输入67条原消息均通过当前内容过滤并刷新计时，
3.000秒不超时、3.001秒超时断言通过；Go2没有在本机重跑Ranger测试。
Ranger agent与observer共用SingleThreadedExecutor，agent QoS为RELIABLE/VOLATILE/depth10；
echo未显式指定QoS，昨日实际端点QoS/进程环境未保存。preflight匹配及mtime支持版本追溯，
但Ranger没有昨日源码哈希快照。Go2对应bridge日志0字节，agent日志没有故障窗接收记录。

结论：已确认Ranger判定当前任务peer心跳过期后暂停，Go2收到暂停后跟停；尚未证实
双向同时断流，无法从现有日志唯一定位到传输延迟、DDS订阅路径或回调处理。
当时Go2未完成C、全局未DONE，不把今日DONE完成锁定修复当作该轮根因修复。
原始信箱往来及Ranger摘录保存于build/go2-field/mailbox-peer-diagnosis-20261029/，
其中mailbox-transcript.json、ranger-evidence.txt及go2-evidence-121317.json为本轮证据，build不受Git保护。
本机仅日志/代码核对与摘录重算，没有改生产代码、启动ROS、改网络或复现实验。

用户追加允许令Ranger补充未来通信诊断日志；已发送请求4db156d0f658478cbf53b1007dfd687f，
要求最小接收/过滤/更新时间与超时快照、限制日志量、必要离线测试，不改阈值/QoS/执行器，
不部署、不重启、不启动现场采样。此处仅记录请求已投递，实施结果待对端回报。

Ranger回复4391bf94c5f643a69c1956eb969eec62确认其本地用户也已授权日志补充，
拟复用agent.log、有界接收/过滤摘要、首次超时单调计时快照。随后Go2连续三次45秒等待
无新消息，按信箱规则结束本轮收信；尚未收到修改diff及测试结果，日志功能仍标记实施中、
未验收，不能报告已部署或后续实验自动生效。继续时先按同task收信核对交付，不重复实施。


### 2026-10-29 白色桶目标C

用户要求将C改为白色的桶、构建并切换新环境，必要时更新正式启动文档。
`cooperative_ws/src/multi_agent_nav/config/scene.yaml`中C由椅子改为白色的桶，A/B仍为灭火器。
新默认pair：`build/go2-white-bucket-20261029/overlay/install/local_setup.bash`；环境入口、Launcher、相关测试夹具及
[双机协同启动与验收](../双机协同启动与验收.md)的安装路径和目标说明已同步。旧安装及用户已有改动保留，包含当前工作区构建内容。
两包构建23.9秒成功；环境/Launcher修改前48通过、修改后48通过，无新增用例；安装来源/字节及目标配置一致检查通过。
新安装--offline三类gates=[]、motion_sent=false。详细命令与验证见[runbook](go2_runbook.md#2026-10-29-白色桶目标c与新安装)。
默认环境实际source仍因enp4s0无192.168.123.x地址退出1；已切换默认选择，未完成现场环境加载。
旧终端显式覆盖和旧进程不会热更新；Ranger端C描述也须一致，本次未操作远端，待核实。
证据`build/go2-white-bucket-20261029/evidence/`；未全库/C++测试、真实VLM、SSH/bridge/录包/运动或重启现场进程，无提交。


### 2026-10-29 相机断联等待

用户要求现场输入检查忽略rgbd_ready及相机临时未就绪，初始/其他观察位置停稳等拔插恢复；
明确保留整轮600秒（含相机等待），仅取消相机等待自身错误/超时。后续授权构建、切换环境和更新正式文档第一节。
入口/inspect稳定窗口不再阻断临时相机缺失；adapter导航健康与RGB-D就绪分离，静态标定和深度单位仍检查。
扫描停稳等待、恢复再识别；相机等待不消耗扫描预算，真正的Spin失败立即终止；已完成规划等待不被120秒watchdog误杀，
未完成规划/运动和整轮期限、定位/costmap/停稳/ARM/心跳保护保留。具体边界见[runbook](go2_runbook.md#2026-10-29-相机断联等待与新安装)。

新默认pair为`build/go2-camera-reconnect-20261029/overlay/install/local_setup.bash`；两包23.4秒构建成功，
CoVLM_env/Cooperative_env/Launcher及[正式启动文档](../双机协同启动与验收.md)第一节三种入口、分步命令均已更新。
构建包含当时工作区已有修改；保留旧安装及未提交工作。C=白色的桶、coverage_range=2.5未变。
修改前四文件170通过/2失败；最终19文件源码/新安装各600通过/2失败，新增9项全部通过。
两失败在基线即存在：field_preparation的live_gates及offline_single_robot_config旧预期，完整名称见runbook。
55模块安装来源/字节和配置检查通过，--offline三类gates=[]、motion_sent=false。
本次默认环境实际source退出0，enp4s0检查通过且两包prefix来自新pair；此前网卡失败为历史记录，不覆盖或删除。
旧进程/显式覆盖的旧终端不会热更新；本次未启动现场/真机/真实VLM/SSH/Ranger/bridge/录包，也未全库/C++测试或提交。
真实拔插恢复仍待现场验证；相机驱动进程若已退出，插线本身不保证恢复。证据`build/go2-camera-reconnect-20261029/evidence/`保留。


### 2026-10-29 16:04故障只读排查（中途转入环扫修改）

最新task_id=1f62c81455d24bf487dbfd3fbf4eaac8，启动日志build/go2-field/cooperative-task.lmi6efsx/start.log。
本机events.jsonl（build/go2-vlm-experiments/1f62c81455d24bf487dbfd3fbf4eaac8_go2_1/）第2/3行确认初始相机等待约132.8秒后恢复，之后继续观察。
第1226–1228行确认扫描中取消，STOP_COMMAND_SENT reason=local navigation not ready，原生Spin终态CANCELED。
新环境ros-logs/python3_33126_1793260687666.log在16:04:53.290记录SYSTEM_READY=false；
twist_to_go2_sport_bridge_33445_1793260689436.log第438行记录SYSTEM_READY lost while ARMED进入FAULT。
python3_33672_1793260729140.log第7/8行16:04:53.679首次NAV失败项为system_ready_value，之后odom/scan/obstacle等陆续过期。
用户16:05:38 watch显示STOPPING（最后报告）及ddsi_udp_conn_write失败，不能当作该时刻实体已停稳确认。
只读查询enp4s0为NO-CARRIER、carrier=0、无IPv4；本机本地域DDS绑定该接口。
现有证据确认本地底层健康丢失及当时接口断链，未唯一定位物理线缆、供电或网卡操作原因；系统NetworkManager日志无读取可见条目。
没有启动订阅/ROS、改网络或重启/复现实验。用户随后转向按整轮交替环扫的工程修改；未把方向修改宣称为已验证的断链修复。

### 2026-10-29 环扫方向交替

用户确认Go2按整轮环扫奇数顺时针、偶数逆时针；初始单帧不计，每个新task_id重置，同任务A/C共用序号。
已修改Go2Adapter方向和计数，explore_steps及runner显式传task_id；原有8方向、提前检测结束、深度补拍、相机等待/超时及运动保护保留。
图片metadata.json记录scan_number/scan_direction。接受过的环扫取消或提前结束不回退计数，进程重启不恢复旧序号。

新默认pair为build/go2-alternating-scan-20261029/overlay/install/local_setup.bash，两包23.8秒构建成功。
CoVLM_env/Cooperative_env/Launcher及[正式启动文档](../双机协同启动与验收.md)第一节和分步命令已同步，旧安装/用户未提交工作保留。
修改前基线157通过；新增6项先失败后通过；源码与新安装均为469+31=500项通过。
已修审查发现的dual_acceptance测试替身漏传task_id；未跑全库/C++，之前已知旧断言失败未销账。
55模块字节/来源及配置一致，offline三类gates=[]、motion_sent=false；详见[runbook](go2_runbook.md#2026-10-29-环扫方向交替与新安装)。
实际source默认环境仍因enp4s0无192.168.123.x地址退出1，仅默认选择已切换，完整现场环境未加载成功。
未改网络/放宽门槛，未启动ROS/真实VLM/SSH/Ranger/bridge/录包/真机运动或重启现场组件，无提交。
真实交替方向及现场连接待验证，证据build/go2-alternating-scan-20261029/evidence/保留。


## 2026-10-29 距离验收失败后继续观察

用户要求：Go2停靠距离不在阈值内且retry无效时，不因该次目标结果结束任务，视为未观察到可接受目标并继续新观察。
本轮修改只在终态+停稳+有限距离>0.81m+一次retry耗尽时分类为可继续探索的距离拒绝，
初始单帧、环扫与缓存C入口均继续生成/选择后续候选，失败观察点跳过、拒绝的C缓存移除。
原始检测/接近失败证据保留，不置READY/放置完成；未完整环扫不扩展覆盖。600秒原期限、真实健康故障、
动作取消/停稳失败和候选耗尽语义保留；相机等待、交替旋转及白色桶目标不变。

新默认安装：`build/go2-distance-rescan-20261029/overlay/install/local_setup.bash`；CoVLM_env.bash、
Cooperative_env.bash、Launcher和[正式文档第一节](../双机协同启动与验收.md#1-完整启动一条命令)已同步。
完整构建命令见[runbook新安装节](go2_runbook.md#2026-10-29-距离验收失败后继续观察与新安装)。
保留原未提交修改、旧安装和日志，无提交、远端部署或新运动授权推定。

验证：修改前接近/入口基线126通过；新增12项距离拒绝用例覆盖有限距离、终态/停稳边界、初始/环扫/缓存C后继续观察、
生成候选去重/保留未完成扫描证据、其他错误不吞掉及只移除被拒绝C缓存，其中6项先失败再通过。
原overshoot入口用例改为检查继续选点；其跨入口完整停止轨迹比较不再包含overshoot，因协同夹具只包接近、单机夹具还包后续探索，
两者此后流程不同；底层冻结接近轨迹测试仍保留。源码18文件577通过，双机离线31通过，共608个不同用例；
新安装从/tmp清除源码PYTHONPATH后同范围577+31通过。两包23.1秒构建成功，仅既有CMake/接口包复用提示。
55模块安装路径/源码字节检查和安装配置一致性检查通过；--offline三类gates=[]、motion_sent=false。
Bash语法、正式文档23个命令块/本地链接和git diff --check通过。

本次默认source Cooperative_env.bash go2退出0，DDS接口enp4s0；multi_agent_nav包前缀确认为新pair。
这是环境加载结果，不是相机/导航/通信或现场运动验收；未启动ROS节点、真实VLM、SSH/Ranger/bridge/录包或新实验。
未跑全库，历史field_preparation两个旧断言失败未纳入本轮，不据此销账。旧终端/旧进程不会热更新，
已有任务按正式文档正常结束并确认停稳后，再选择第一节的一个入口。证据在
`build/go2-distance-rescan-20261029/evidence/`，包括before、baseline/red/green、source/installed、dual-*、
build、check-install、offline-config、default-env和本轮增量task.diff；目录不受Git保护，须保留。


### 2026-10-29 16:57 最新实验失败只读排查（task 53ec7cb）

用户要求排查最新实验失败；本轮只读排查日志/代码及进程是否存在，未操作ROS节点或真机、未改运行逻辑/参数。
最新任务53ec7cb894ba44fbab6c1b1bf183928f，启动证据为
`build/go2-field/cooperative-task.kpxpikq1/start.log`：受理成功，安装为go2-distance-rescan-20261029，记录的模块matches_source=true。
失败发生在本机第2条命令（找C），不是A距离验收失败：

- `build/go2-vlm-experiments/53ec7cb894ba44fbab6c1b1bf183928f_go2_1/events.jsonl:315–317`：A距离验收通过，TARGET_APPROACHED实测0.7983289124882551m，retry=0。
- 第2条命令当前单帧C为NOT_DETECTED，模型响应有效；选择P12/free/170/210，目标坐标(2.435,-12.275,0)，规划路径6.8714m。
- `..._go2_2/events.jsonl:1283–1286`：VIEWPOINT_NAV成功、确认停稳后进入SCANNING。
- `build/go2-distance-rescan-20261029/ros-logs/controller_server_52174_1793263945991.log`：北京时间16:56:59.143886 Reached the goal。
- 同目录`behavior_server_52180_1793263945992.log:23–27`：16:56:59.271680 Running spin，Turning -0.79；16:57:01.272053 Collision Ahead - Exiting Spin，spin failed并Aborting handle。
- 第2命令events.jsonl:1287–1292：首轮顺时针Spin接受约2秒后ABORTED(native_status=6)，观察点记observation_failed，任务停车原因local execution failed: scan failed。

直接原因已确认是Nav2旋转碰撞保护中止，任务层把Spin失败作为观察异常结束本地执行。调用链为
adapters/go2.py的SPIN_WAIT非SUCCEEDED→RuntimeError('scan spin failed')，go2_exploration.py读取future异常→
ExplorationStopped('scan failed')，runner.py→machine.stop。第2命令未出现HEALTH_FAILED、SCAN_HEALTH_FAILED、
WAITING_CAMERA、TARGET_REJECTED或APPROACH_DISTANCE_FAILED；本轮未触发刚增加的“距离验收失败后继续观察”分支。
同批nav/safety supervisor日志只有启动阶段false→true，没有失败时刻再次失效记录。
agent在16:55:47有phase must be synchronized before execution的拒绝记录，但随后第2命令已执行并成功到达P12，
该警告不是此次16:57停车的直接触发。

代码确认的检查边界：选点及到达后parking_free(actual_pose)通过global costmap检查，
Go2Adapter.observation_candidate_free也只检查global_costmap整圈圆盘；没有对local costmap做同等整圈预检。
因此“全局检查通过”不能证明旋转全过程无碰撞；运行中障碍变化也可能导致Spin拒绝。
实体障碍/局部残影/瞬时点云或定位误差的具体原因仍待证据，不能把碰撞保护解释为已发生实际碰撞，
也不能认定方向交替有误（此次首轮-0.79符合奇数顺时针）。当前保存记录没有该时刻local costmap/点云快照；
build中最新已有db3录包时间为10月25日，与此次实验不对应。排查时上述behavior/controller/agent PID均已退出。
建议后续修复范围是让到点预检同时核对局部旋转空间、阻挡点跳过并重选；不关闭Nav2碰撞保护。
本轮仅提出基于证据的改进方向，没有实施或重新构建；具体障碍成因和现场恢复仍未验证。


### 2026-10-29 到点局部旋转空间检查

用户在task 53ec7cb失败排查后明确要求：不能因目标超出局部costmap范围就不前往；
确认“选点/前往只用全局，到点再检查局部，覆盖不足等待、确认受阻换点”的规则并授权修改。
Go2Adapter新增local_rotation_check，仅在探索执行器到达观察点并确认停稳后使用，按实际位姿/时间戳TF/旋转网格原点检查完整旋转圆盘。
覆盖不足或unknown等待，保持原600秒期限；确认障碍记scan_blocked、排除本次本地探索中的该点并重选。
初始单帧不旋转不受此门槛影响；远候选、Ranger、全局规划和Nav2碰撞保护均保留。
Spin ABORTED仅在确认停稳及局部障碍后允许继续；CANCELED及其他异常、未停稳、Nav2/定位/TF/costmap数据流健康故障仍停车，
不能把真实健康丢失当作地图覆盖等待。覆盖不足与障碍、数据健康失效有独立结果。

独立审查发现等待期间定位修正后可能沿用旧scan_pose写覆盖，已补用例先复现后修复：
局部预检完成后重新取健康位姿再作为scan_pose；审查复核关闭该问题。
构建/边界及验证见[runbook](go2_runbook.md#2026-10-29-到点局部旋转空间检查与新安装)。
新默认pair：`build/go2-local-scan-20261029/overlay/install/local_setup.bash`；环境入口、Launcher及
[正式启动文档第一节](../双机协同启动与验收.md#1-完整启动一条命令)同步。保留旧安装、日志和用户未提交工作，无提交。

本轮验证：修改前generated_candidates/approach_costmap/go2_exploration三文件166通过；新增18项测试，
首批14项先失败后通过，另新增审查发现的等待位姿修正回归也先失败后通过。
最终20文件源码644通过，双机离线31通过；新安装从/tmp清除源码PYTHONPATH后同范围644+31通过，
各合计675个不同用例，未把重复运行计作新用例。覆盖远点不受局部窗口否决、整圈侧向障碍/未知区域区别、
TF/网格旋转与陈旧数据、到点后覆盖等待恢复、原期限/健康/停稳/动作终态、阻挡点不重复选及Spin ABORTED/CANCELED边界。
两包27.3秒构建成功，仅既有CMake及接口包复用提示；55模块安装来源/源码字节检查通过，安装配置一致，
--offline三类gates=[]、motion_sent=false。Bash语法、正式文档23个命令块/本地链接和git diff --check通过。
默认source Cooperative_env.bash go2退出0，DDS接口enp4s0，包前缀指向新pair；这不等于现场验收。
最终文档已同步新安装share/docs。

未启动现场ROS节点、真实VLM、SSH/Ranger/bridge/录包、ARM/start或真机运动；没有重启旧进程。
未跑全库/C++测试，历史field_preparation旧断言失败未纳入本轮，不据此销账。旧进程/显式旧环境不会热更新，
使用新版时按正式文档正常结束旧任务并确认停稳后选择一个入口；实际障碍来源和本次恢复效果仍待现场验证。
证据`build/go2-local-scan-20261029/evidence/`保留before、baseline/red/green、review-red、source/installed、
final-changed、dual-*、build、check-install、offline-config、default-env和task.diff，不受Git保护，须保留。


### 2026-10-29 18:04 心跳失联联合排障（task 1c6b）

用户先要求更新overlay/脚本，随后要求当前heartbeat lost联合排障。按新增AGENTS信箱约定读取README/角色，
本机go2、peer ranger、hub local，doctor通过；同信箱task heartbeat-1c6b-20261029联合。
实验task_id=1c6b7c6027924890bdb1a17c758e190f，start.log为build/go2-field/cooperative-task.4sr1bran/start.log。
本轮首次宿主复核时三进程Go2 agent66826、coordinator67099、bridge66538均实测使用go2-local-scan-20261029；
环境入口、脚本默认指向本版，55模块安装源码检查通过，未发现overlay滞后，故本轮不重复构建。

已确认的本机故障：events.jsonl第1449行，steady21804.100058663/北京时间18:04:13.978416，
STOP_COMMAND_SENT reason=peer heartbeat lost for current task。之前Spin18:04:04.180203已成功；VLM后台等待随后取消，
晚到返回不推动运动。A已接近成功，C搜索未完成。此处是本机当前任务peer计时失效，不是转述对端暂停。

联合证据：Ranger信箱消息892d88e5d9e2453ab381e850936c0b26提供本轮首次超时原始JSON：
reason=coordinator heartbeat timeout，now26702.308814966，lost_timeout3，sync_received年龄3.043973396秒，
peer年龄2.788119880秒。本机独立重算32条peer历史均accepted，最大callback间隔0.272671490秒，
没有内容过滤导致丢心跳的迹象。Ranger记录首次报警北京时间18:04:13.906955；两机monotonic不直接比较，
也不依据约71ms壁钟先后断言传播因果。Ranger报告自己state在故障窗连续，最大发送stamp间隔0.500626秒，
独立peer echo出现18:04:11.129090→18:04:15.346092的4.217003秒发送stamp空窗；echo无本机接收时刻。

Go2本机15秒并行只读高层state采样（没有传感器订阅或发布控制）：
- Domain0：Go2状态150条，Ranger0条；可见go2_cooperative_task_bridge_0的Ranger发布端但没有数据。
- Domain40：Ranger28条，最大接收间隔0.614721秒；Go2 state没有发布端。
- 采样时双方同任务PAUSED，Go2 ready=true/deposited=false，Ranger ready/deposited=true，双方nav/localization/stopped报告true。
- 以上仅消息报告，不认证实体停稳或DISARM。

定位到旧Go2 bridge的地址绑定失配：采样时wlo1=192.168.137.142；bridge66538仍绑定UDP fd15
192.168.137.138:57225，Domain0 fd8=192.168.123.222:35743。Ranger同期邻居表也见.142与Go2 MAC
c0:35:32:d3:c6:53，.138 INCOMPLETE/FAILED。bridge进程存在不等于共享域正常转发。
排障中Ranger用户告知双方已设置静态IP；Go2再次实测wlo1=192.168.137.128/24、nmcli ipv4.method=manual，
当时再次宿主只读查询确认同一个bridge PID/fd仍绑定.138（输出也已通过信箱661d256e81b44a0283eb2d639ec9ec47同步Ranger）。
因此静态配置完成时旧DDS参与者没有自动重新绑定；不能把信箱/SSH恢复说成DDS恢复。最终进程状态另见下段。

历史根因边界：证据强烈指向Go2无线改址后的桥接/DDS连接失效，但本机故障窗没有agent接收/发布快照，
NetworkManager系统journal当前账号不可见，无法证明地址切换精确时刻或唯一判定该瞬间的底层触发。
本机普通沙箱进程视图看不到宿主进程，已通过授权的宿主只读/proc检查确认仍在运行；不能据沙箱“PID不存在”宣称进程退出。
先前53ec7cb排障中的PID退出说法亦未用宿主视图确认，仅保留为当时观察，不作为退出证据。

恢复方向：现场确认停稳并按既有退出流程后重新创建Go2 bridge/DDS参与者，使共享域使用当前静态地址；
再只读核对双向state和SYNC新鲜后，按正常流程另起任务，不能自动解除当前PAUSED。
本轮没有重启bridge/agent、改网络/阈值/代码、ARM/start或发送运动命令；未验证恢复效果。
证据根build/go2-field/heartbeat-1c6b-joint-20261029，含双域原始接收、端点/间隔摘要、进程socket、静态地址复核、
日志快照及SHA256、信箱来往和Ranger超时JSON；目录不受Git保护，须保留。
此前Ranger补充诊断日志交付c708e4b6f8b448c7bd343ed0b82dc63c已保存并ack，本次实际超时JSON证明Ranger端已生效；
未将对端交付视为Go2本机已补充同类日志，未把历史10292c根因改写为已修复。

最终现场复核 2026-10-29T18:45:11.039327+08:00：Go2 wlo1仍为manual 192.168.137.128/24；宿主/proc未发现task agent、coordinator或domain_bridge进程。
本轮未执行kill/退出/重启，不能据此推断谁关闭了组件；旧socket失配为此前已确认事实，不再写作当前仍运行。
最终静态地址与进程列表保存在static-ip-confirmation.json。恢复后的DDS双向通信未验证；后续应在固定IP后按正常流程启动组件并先检查高层双向心跳。


### 2026-10-29 19:52 对端导航健康瞬时失效联合排障（task 774a）

用户报告peer localization/navigation invalid并要求联合排障。本轮使用Codex信箱task peer-health-774a-20261029，
实验task_id=774a19a84fa945b1809715bc734700d6，start.log为build/go2-field/cooperative-task.jdm_y9cj/start.log。
只读核对双方日志、Go2宿主进程及15秒Domain0高层state；不改网络/阈值/代码、不重启、ARM/start或发送运动命令。

本机确定时序：`...774a..._go2_2/events.jsonl:3886–3890`为导航取消及STOP_COMMAND_SENT，
steady28276.672438396 reason=peer localization/navigation invalid，原生导航随后CANCELED。
Sport日志twist_to_go2_sport_bridge_83146_1793274114712.log:1463提供wall1793274726.550799151，
即19:52:06.550799。bt_navigator在19:52:06.572才报halt中的follow_path结果获取失败，晚于任务层取消，
不作为最初根因。停车前12秒PATH_RECEIVED203条、最大间隔0.057690325秒；本轮无HEALTH_FAILED/SCAN_HEALTH_FAILED，
Go2本机supervisor从启动ready=true后没有故障转换。A已接近成功（0.766441548m），C仍在搜索。

Ranger回复9a29ca7f956743f3bf8bc2ff5c460051定位直接触发项：定位有效，local_costmap header_stale使nav_ready短暂false。
远端证据根（不是Go2本地路径）：
/home/isee601/ws/VLM_Nav/build/ranger-local/acceptance-20261015_161341/cooperative-startup-20261029_193955-3M5Cl7。
agent.log:128，wall1793274726.524568999，failed_checks仅local_costmap，localization_valid=true；
header_age_s=1.085712194442749>1.0，但receipt_age_s=0.19932491199870128，last_update.header_age_s=0.0011718273162841797。
按对端数值重算，消息时钟年龄与单调接收年龄不一致约0.885215455秒。
Ranger系统journal同秒记录systemd-resolved Clock change detected. Flushing caches及systemd-timesyncd
Initial synchronization to time server 91.189.91.157:123 (ntp.ubuntu.com)。证据指向NTP校时跳变造成header年龄突增，
不是实际0.199秒的数据传输间隔超过1秒，也没有定位失效证据；不据此放宽costmap健康门槛。
state.log:111343首个false stamp1793274726.524804，localization=true/nav_ready=false，PAUSED local health invalid: nav_ready；
111541 stamp1793274726.6202612 reason变peer paused during task；111574 stamp1793274726.6602066 nav_ready恢复true，
仍PAUSED；本机重算false窗口约0.135402441秒。
对端随后通过7c5b58386c9d4c43b1d537c208eaf24a补齐完整健康JSON、journal JSON及实际install源码片段，已保存并离线重算。
clock change的journal wall=1793274726.523532，距首个nav_ready=false约1.272毫秒；
timesync初次同步的源时间_SOURCE_REALTIME_TIMESTAMP=1793274726.523498（journal收录时间4726.526952）。
Ranger安装nav2.py链接到其源码，文件修改时间早于当前agent启动；costmap_transform先按monotonic检查receipt，
再按abs(ROS now-header stamp)检查header，后者返回header_stale，与完整健康JSON一致。
远端原始内容由信箱提供，未直接访问远端文件系统；本地副本为证据目录的ranger-health.json、ranger-clock-journal.jsonl及ranger-7c5b58386c9d4c43b1d537c208eaf24a.txt。

代码链已核对：Go2 receive_peer在同任务localization/nav false时先stop(peer localization/navigation invalid)，
该检查早于peer STOPPING/PAUSED转述；stop锁存。离线复用状态机夹具分别输入一次localization=false/nav_ready=false，
均重现停车，后续true仍PAUSED；无ROS初始化。Ranger报告其_peer_fault将Go2此旧文案归为peer paused during task，
并撤销原local nav_ready的自动恢复资格；本轮不跨端迁移自动恢复语义。
Go2 HEAD=f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1，agent.py有既有未提交修改，实际安装agent.py与源码逐字节一致。
当前Go2无is_nav_ready_pause函数；实际片段及hash通过6d29b963abc84af5ae0c5cd87d18a0d2发Ranger核对。

Go2时钟核对：同一Sport日志1481条wall/steady配对，偏移全程极差0.000014782秒，故障窗90条稳定；
未见与Ranger约0.885秒同量级跳变。Go2当前NTP/NTPSynchronized均yes，但系统journal当前账号不可见，
“没有可见条目”不等于没有系统校时事件；两机monotonic不直接比较。

后续15秒只读采样双方各150条，同任务、localization/nav/stopped均true但PAUSED；
Go2 reason=peer localization/navigation invalid、ready=true/deposited=false；Ranger reason=peer paused during task、ready/deposited=true。
当前消息恢复不改写当时瞬时false；停稳字段只是消息报告，不认证实体停稳或DISARM。
宿主bridge83834绑定当前192.168.137.128/192.168.123.222，agent84123/coordinator84396及bridge使用go2-local-scan-20261029；
本轮没有旧IP绑定失配证据，历史1c6b问题不套用本轮。

建议后续从两端启动前完成并稳定时间同步、避免运行中系统时间跳变入手；若修改代码，应明确处理时钟跳变，
仍保留单调接收新鲜度和真实costmap/TF健康检查，不以扩大超时或自动解除PAUSED替代根因处理。
本轮仅诊断，没有修复部署或现场恢复验证。证据build/go2-field/peer-health-774a-20261029含本地日志快照/SHA256、
当前状态/端点、宿主进程socket、Ranger信箱摘录、时钟偏移重算、暂停锁存离线自检和causal-summary.json，须保留。

### 2026-10-29 接收可恢复暂停修改意见（尚未在Go2实现）

本轮用户要求“接收codex信箱中的修改消息”。doctor通过，接收并回复/ack同一task peer-health-774a-20261029的两条消息：
74e172f781004626a5045cbc2e73dc3a确认前述诊断闭环；151d42ff92a949fb8a4d9b6830e0f7e1转达修改意见原文：

> 将此类的navigation invalid设为可恢复,遇到后不直接锁存pause而是可以在恢复正常后继续.以及当heartbeat timeout,则两边都进入暂停状态;恢复正常heartbeat则可继续任务.不因为对端的pause而锁存pause, 避免恢复后死锁.据此进行修改,并将此修改意见原话传递到codex信箱

Ranger报告开始修改本机恢复逻辑与回归测试，尚未交付实现结果。Go2本轮仅接收、核对现有触发位置和回执，未修改、构建或部署恢复逻辑。
后续实施需核对nav_ready、peer/coordinator心跳、双方停稳与任务身份，以及人工停止/定位失效等锁存条件；不能将对端已开始修改写成双端已经兼容。
原文及回执保存于build/go2-field/peer-health-774a-20261029/modification-messages.json、ranger-151d42ff92a949fb8a4d9b6830e0f7e1.txt、modification-receipt.txt。
修改意见回执14bb5f726dd94d1598585dbf5a7ef205投递成功，原消息ack成功；没有启动机器人或恢复当前任务。

### 2026-10-29 可恢复暂停实施（源码完成，未构建部署）

用户明确“就按照此要求修改”，即上一节信箱意见。分支go2-cooperative-task-layer，HEAD仍f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1；已有dirty工作保留，不提交。
本次改models.py、agent.py、coordinator.py、runner.py、go2_exploration.py，新增test_recoverable_pause.py，更新test_startup_handshake.py的旧“永不恢复”契约断言，更新runbook及正式文档第1.4节。

实现限定独立地图协同（share_c_coordinates=false）；纯nav_ready失败、peer/coordinator心跳超时可恢复。
AgentMachine.stop保存恢复前状态并仍取消动作；收到新鲜同task的SYNC、peer状态，定位/nav恢复、本机stopped+motion_terminal、peer停稳且不STOPPING，恢复原阶段内同一未完成command。
coordinator保存resume_phase及commands/ack，双方fresh/healthy/stopped/可恢复PAUSED后恢复原phase；暂不消费恢复中的CANCELED为任务失败。
双方恢复先公布活动且停稳，再等对端退出PAUSED才推进执行；runner延迟effect和generator，原生motion_guard继续兜底，避免先恢复端移动阻塞后恢复端的停稳门槛。
已成功的command不重跑，ready/deposited/C缓存和adapter环扫计数保留；当前未完成探索执行器重建，候选历史不保证延续。
整轮600秒含暂停，deadline/started不重置。人工PAUSE/CANCEL、定位/身份错误、未知原因、清理失败可覆盖软暂停成为硬锁存。

双端编码与Ranger消息f151207d3fa84472af3b6f53b2aecd87确认：PAUSE kind=NAV_READY_WAIT；具体is_recoverable_pause原因白名单在models.py，兼容附加` | nav_ready:`诊断。
旧peer localization/navigation invalid混合文案不自动恢复；Go2拆成peer localization invalid和peer navigation not ready。
Ranger确认相同的不对称恢复等待，报告50项定向测试通过（仅对端报告，不记为Go2验收）；Go2实际HEAD/dirty、文件hash及健康异常捕获片段通过3cf194c75f58499b912e2d55910a7de8投递。
信箱task保持peer-health-774a-20261029。后续两端仍需核对实际安装和现场消息序列，不能把约定或源码成功当双机已部署。

独立只读代码审查发现并修复3项边界，均RED→GREEN：真实SingleRobotApproach提前进入生成器导致guard拒绝；explore内部首次发现纯nav失效需NavigationNotReady专门分类；generator.close清理异常必须升级硬锁存且仍发物理stop。
新增32项通过，包含三类故障恢复到DONE、双向不对称恢复、停稳/终态/新鲜度门槛、失联期间短暂定位失败不可被掩盖、未知混合原因、人工升级、期限、成功拾取不重跑、旧VLM结果废弃、真实Go2 adapter受控future。
核心修改前基线94通过/1旧失败；广测协同目录1007通过/14失败（当时含29新例，后3例在最终范围通过）；14项用原生产文件快照逐项复现，未新增失败。
最终核心/恢复/入口范围256通过/1旧失败（test_covlm_env仍要求20261018旧overlay顺序，退出95），也用原代码快照复现。失败逐项名单和证据见build/go2-recoverable-pause-20261029/evidence/verification.md。
广测含Domain31 localhost合成ROS，costmap/ROS smoke等失败保留；本轮没有Domain0现场节点、真实VLM、ARM/start、真实运动或重启。

证据根build/go2-recoverable-pause-20261029/evidence，包含before快照、基线/red/green、审查red/green、广测、最终定向、原代码差分、协议原文及modified-sha256.json；必须保留。
没有构建、切换overlay或修改启动脚本；当前默认仍build/go2-local-scan-20261029/overlay/install，不含本次代码，源码一致性预检可能拒绝启动。
后续若更新安装，须按runbook成对构建根包与协同包到新目录，核对Ranger已同步协议后再走正常退出/启动流程。旧进程不热更新，未做真实恢复验收。

收尾补充：Ranger通过4f7e590451564f9c9f1952c63a0bd905发送HEAD ab922276108c21f44612b7805e26fc0560300528 dirty实际diff，报告97项定向通过且未部署。
Go2保存ranger-implementation.json及原文，从diff提取12项RECOVERABLE_PAUSES，用本机实际分类函数逐项验证原因及实际诊断后缀接受，旧混合/定位/人工原因拒绝，protocol-check.log通过。
核对NAV_READY_WAIT及恢复等待编码一致；双方内部异常类型和重启/检查点策略不同，未直接访问Ranger文件系统或做跨机运行验收。已回复并ack。

### 2026-10-29 可恢复暂停新overlay构建完成（默认未切换）

用户明确要求“将修改构建overlay”。按runbook同时构建根包vlm_nav与cooperative_ws/src中的multi_agent_nav，输出新目录：
`/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-recoverable-pause-20261029/overlay/install/local_setup.bash`。
两包22.8秒完成，退出0；没有覆盖旧安装。接口schema未变，multi_agent_task_msgs复用build/go2-coop/install。
从/tmp清除源码PYTHONPATH并最后source新overlay，55模块安装路径/源码字节一致检查通过；安装配置一致，离线三类gates=[]、motion_sent=false。
24文件相关安装回归731通过（699既有+32恢复新增）、退出0、25.54秒、11条第三方弃用警告；未把重复验证计为新增测试。
证据build/go2-recoverable-pause-20261029/evidence包含build/check-install/offline/installed及Junit报告；本次未跑全库/C++测试，上一轮14项广测和旧env断言失败仍保留。

仅构建与安装验证，没有修改CoVLM_env/Cooperative_env/start_go2_vlm默认值；默认仍go2-local-scan-20261029。
正式文档新增overlay状态和runbook构建命令已更新，文档同步进新安装share/docs。
没有重启/切换运行中的进程、启动ROS现场节点、真实VLM、ARM/start或真机运动；新旧双端混用不等于恢复通过，现场验收待后续。

### 2026-10-29 默认入口overlay选择修复

当前分支go2-cooperative-task-layer，HEAD已由用户提交至bce9de1ee4e9d2dd3c4ac6288703335767bb9ae9（10.29），开始时工作树干净。
用户报安装与源码不一致，实际复现scripts/cooperative.sh go2 preflight默认调用报相同stdin第23行错误，退出1；显式指定新recoverable-pause安装退出0，55模块一致，确认构建本身完好。
已将CoVLM_env.bash、Cooperative_env.bash、start_go2_vlm.py默认值和README/正式文档命令切为build/go2-recoverable-pause-20261029/overlay/install/local_setup.bash。
Go2日志目录同步go2-recoverable-pause-20261029/ros-logs。显式COOP_SETUP_FILE仍优先；旧终端须用runbook所列export覆盖旧值。
更新两个入口测试的期望安装：先3失败/46通过，改默认后49通过。默认预检退出0、installed全部匹配/live_gates=[]/motion_sent=false；默认source后从/tmp校验55模块路径及源码一致通过。
Bash语法、正式文档23块脚本与文件链接、git diff --check通过；同步安装share的启动脚本及文档。
无需重新构建Python/C++运行包，没有重启进程、ROS现场节点、ARM/start或运动。原731安装回归历史保留，本轮仅验证入口范围。
证据build/go2-recoverable-pause-20261029/entry-fix/包括before、旧默认失败、新显式成功、默认修复后成功、red/green及安装检查。

### 2026-10-29 NAV_READY启动等待调整

用户要求将120秒改为240秒。已修改start_go2_vlm.py的启动就绪等待和start_go2_cooperative_task.py的restart就绪等待，正式文档对应说明与分步timeout命令同步240秒。prepare阶段原有10秒即时复核及整轮600秒期限保留。两份入口回归129项通过，未执行现场启动；脚本和文档同步新overlay的share副本。

### 2026-11-03 Go2 RoboRSI / Codex CLI 可选接入（离线完成）

按 `/home/isee-pst/Notes/Go2_RoboRSI_CodexCLI_Implementation_Plan.md` 执行。
仍在 `go2-cooperative-task-layer`，HEAD `bce9de1ee4e9d2dd3c4ac6288703335767bb9ae9`；
未切分支/worktree、提交或推送。原 staged 二进制 diff 完全一致，原12份 dirty 文件保留。
仓库外完整 dirty 快照：`/home/isee-pst/unitree_ros2/go2-agent-baseline-20261103-205731/`；
本轮证据：`build/go2-agent-adaptation-20261103-20261103-205731/`，含基线 argv、摘要、测试及仅本轮差异。

新增 `tools/go2_agent` 单次会话管理器、RoboRSI runtime/config/一个静态搜索技能；新增
`scripts/start_go2_agent_task.sh`、三份适配测试和 [Agent runbook](go2_agent_runbook.md)。
只在当前 dirty `scripts/start_go2_vlm.py` 增加默认关闭的 `--agent-framework` 派发与宿主结果接收，
原测试文件末尾追加12项，不重写原用例；本交接只追加。原入口日常使用不依赖框架或Codex。

两入口共享 Launcher.check 构造的同一搜索 argv、原环境和同一安装，Agent仅追加证据目录；
原搜索/接近/ABC/重试/停止参数不变，NAV_READY 240秒保留。
管理器启动前原子消费机会，重复/并发/不同目标/旧会话不重放；目标原文 argv、shell=False。
完成只采纳退出后可信原报告及SESSION_END；模型 finish 不构成成功。
SIGINT/SIGTERM/框架异常/提前finish走一次SIGINT及30秒原收尾等待；搜索独立于Codex进程树。
宿主不把通用TERM/KILL策略用于适配器或搜索，停止未知不自动DISARM、不删锁、不重启。
原DISARM及读回仍决定整轮完成，框架结果与原任务结果分别保留。

RoboRSI HEAD固定 `0208cd72ec7c885a7219040cd2be459d127e374e`，原`.venv` Python3.10.12；
Codex绝对路径版本检查为0.166.0，原生二进制完整摘要见preflight.json；模型gpt-6-astra/high，
max_turns/max_tokens=null，planner1200秒/action600秒。实际LocalToolSpec不受action timeout约束，取消靠管理器。
未升级依赖，未更改CODEX_HOME/认证/原视觉提供方。框架与ROS环境分离，秘密仅由私密管道/环境继承。
原生CLI只声明工具的权限限制尚未验证；read-only/ignore-rules和提示词不等于网络/机器人隔离证明。

最终离线测试：新增44项适配测试在框架venv通过；原启动器129项+新增入口12项在co-nav-real通过（141项、11条既有第三方弃用警告）；
原独立执行及接近parity106项通过。共291个不同用例：新增56、既有235，未将重复运行计为新增。
曾用框架venv跑旧入口组，8项缺cv2失败；改回项目原定co-nav-real后全部通过，未增装视觉依赖。
独立审查发现退出与最终事件读取竞态，已以fake poll写文件的失败用例复现，补退出后再次读取并验证通过。
另补日志/持久化I/O失败不阻止SIGINT、启动前取消防迟到start。原有源/安装549文件摘要未变，55个Python源/安装副本及配置/launch逐字节一致。
Python编译、shell语法、git diff --check通过。未修改ROS包，依方案未重建或改写overlay。

阶段结论：离线通过；真实模型尚未验证；真机尚未验证；原adapter终态、物理停稳证明及异常崩溃停车缺口未解决。
本轮/proc隔离，未核对宿主现有实验状态，不操作运行服务。未跑全库/C++/现场测试。
后续需用户明确授权，先纯假机器人后端真实Codex调用/取消/权限，再原流程人工ARM单机搜索，最后两入口同目标配置对照。
日常回退仅切回原入口；共享入口若需恢复，按完整接入前快照撤销本轮差异且保留原dirty和后续工作，不做Git回退。
未来算法仍修改共同源码，按原runbook共同构建新overlay、核对两入口、确认旧轮停车后重启；不承诺源码热更新。

### 2026-11-03 单机相机与冷启动预检自动继续

用户要求运行入口后直接打开相机终端和预检，无需输入确认字符。
在原dirty工作树上，仅移除共享Launcher.camera的CAMERA确认，将冷启动传感器预检前START确认改为状态提示。
单机原入口和Agent入口共同生效；缺失相机才打开SSH窗口，已有发布者/底层仍复用，重复发布者和不完整底层仍拒绝。
READY/CURRENT/ARM及故障RESET确认保留；SSH凭据提示不属于此次移除范围。共享协同硬件准备调用也使用此逻辑，协同任务start的START确认不变。
两份启动器测试141通过、11条既有第三方弃用警告；修改前新期望2失败，修改后通过。Python编译和git diff --check通过。
仅源码启动脚本及文档/测试修改，两种入口直接执行该源码脚本；未改ROS包、未重建或改写受保护overlay，未启动相机/预检/ROS/模型/运动，未提交。


### 2026-11-04 单机等待恢复与诊断终端

用户明确要求运行门槛等待恢复，运行/暂停分开累计，不增加门槛、不改入口文件，并确认保留1270秒宿主上限。
分支go2-cooperative-task-layer，HEAD仍bce9de1；原未提交工作保留、不提交。
新增experiment_recovery.py/experiment_status.py，修改go2_check.py与go2_exploration.py及定向测试。
单机运行条件异常先停止，等待定位/Nav2/控制/控制权/代价地图/动作终态/停稳恢复后续行同task；不自动ARM/reset_fault。
已完成观察保存，未完成目标重新观测；相机/旋转地图/视觉工作线程等待不消耗运行预算，独立窗口显示实时状态与三种时间，不打印配置或额外订阅ROS。
人工取消、程序异常、无效静态配置与运行预算耗尽仍退出。启动器预检及Agent外层1200秒planner/1270秒宿主墙钟限制未改；双机协议和Ranger未改。
新安装选择方式及构建命令见go2_runbook同日章节。入口不自动切换默认安装，需在原启动终端显式export COOP_SETUP_FILE；旧进程不热更新。
证据目录build/go2-wait-recovery-20261104/evidence含修改前快照、红绿测试、审查回归、构建及安装验证；最终验证结果在该目录verification.md。
本次没有真实模型、相机、机器人启动或运动。真实窗口显示及真机恢复待现场验证；不把离线通过称为现场通过。

最终验证：双包新overlay-final构建23.7秒成功；57模块来源/字节及配置一致；源码广测1183通过/14项已记录历史失败，新增32项全通过。
从/tmp加载新安装关键回归268通过（新增32、既有236）。入口/环境/Agent配置5文件与本轮前快照一致，git diff --check和新增Bash文档语法通过。
历史失败逐项见上述verification.md；不宣称全库全绿，未进行现场恢复/真实模型验收或自动启动新安装。


### 2026-11-04 启动安装检查清单静默

用户反馈启动终端显示installed/arrival_config_files/config_dir大段JSON，要求丢弃。
来源为scripts/start_go2_vlm.py的check_install成功报告；主shell预检及Launcher.window子终端预检直接继承stdout。
仅在这两个调用处增加stdout到/dev/null的重定向，保留stderr、检查和失败退出码；直接--check-install仍可保存报告。
57项入口回归通过；隔离执行主/子终端覆盖成功与退出7，清单不显示、失败信息保留且不继续启动；Bash语法通过。
双包新构建build/go2-quiet-start-20261104/overlay于23.9秒成功，入口安装副本一致；既有CMake及消息包复用提示。
构建命令沿用runbook等待恢复章节，仅GO2_RECOVERY_OUTPUT换为本次overlay路径；证据在build/go2-quiet-start-20261104/evidence。
源码入口下次启动即生效，环境默认选择及1270秒宿主上限不变，旧安装不重建；未启动真机或重启现有终端。


### 2026-11-05 全流程等待恢复交付

用户批准全流程方案并要求实现；需要用户答复时提示音、完成代码修改播放5秒提示音乐。起始工作树干净；分支go2-cooperative-task-layer，HEAD仍为323d5c52737477962a874ae012db117be5a93f34，未提交。实现、故障分类、完整三包构建命令见[runbook](go2_runbook.md#2026-11-05-全流程等待恢复与三包构建)及[计划](../superpowers/plans/2026-11-05-go2-resilient-experiments.md)。

已实现：Go2单机/协同/旧VLM软故障保持任务等待，等待/STOPPING/恢复握手冻结有效任务钟；输入新鲜度、单次动作和取消仍用真实钟。Sport新增有实例和代次保护的本地恢复服务，部分软FAULT可PREPARE/RESUME；首次显式ARM、人工停止/接管/硬错误保留。取消无需等待在途RESUME，服务端只允许撤销该恢复直接后继，不能碰后来的人工ARM。旧VLM保留扫描/接近阶段与完成进度，跟踪真实native request/result而非清空句柄当终态。单机与协同共用精确软错误分类，地图/身份变化及未知异常不可续接。

入口动态检查无限等待、不重复ARM/start、不自动重启退出组件；手动RViz目标不重放。总入口早期开诊断、主终端仅必要提示和日志路径；manual/VLMNav-go/preflight会最后加载同一新overlay并传给子进程，不再用源码PYTHONPATH覆盖安装。Agent移除1200/1270秒总墙钟限制，框架失败后守护已启动搜索，原任务/框架结果分别记录。必要证据断档或原生动作/恢复取消不确定，明确不能完整验收。

新安装：`/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-resilient-experiments-20261105/overlay/install/local_setup.bash`。接口multi_agent_task_msgs、根vlm_nav及multi_agent_nav共同构建；CoVLM_env.bash、Cooperative_env.bash、Launcher和Go2旧入口已切换默认，显式COOP_SETUP_FILE仍优先。历史overlay未重建，旧终端/进程不会热更新。最后从/tmp核对59个Python模块与三包prefix、新srv及98个脚本/launch/config字节；证据check-install.json、installed-paths.txt。

验证分开报告（以下分组有重叠，不相加当唯一用例总数）：

- C++ bridge 35项通过：原19项、本轮新增16项（包括CANCEL/RESUME乱序和人工代次隔离）。
- 新恢复客户端14项、手动ActionStop 4项、shell恢复6项、旧VLM新增35项通过；连同C++在本输出CTest为94项通过，0失败。首次CTest默认系统Python缺httpx，已复用runbook指定的现有运行venv，未安装新依赖；复跑依据为colcon-targeted-final.log与colcon-results.log。
- 旧VLM纯离线广集222通过（新增35、既有187），1项会初始化ROS的测试排除。协同最初定向147通过（新增37、既有110），后续共同异常分类及parity定向111通过，含精确已终态的Nav2软失败和未知异常硬失败。
- Agent runtime/session33通过。入口/环境/恢复/诊断组合186通过；后补Go2旧入口overlay加载用例与相关脚本/客户端23通过。配置/脚本25通过，2项ROS初始化测试排除。Bash语法10文件及git diff --check通过。
- 协同最初广集1071通过、11失败、7排除；11个失败用原HEAD包和原配置同名复现，详情task2-report.md及task2-baseline-failures.log。安装广集首次额外暴露7个parity失败，已修复3个精确软原因并将软故障夹具对齐真实Adapter错误；未知RuntimeError/TypeError新增硬失败用例，不将未知异常泛化为软错误。最终安装广集结果附后。

证据根目录 `build/go2-resilient-experiments-20261105/evidence/`，保留所有中途红灯、基线和最终日志，不删除。审查报告task1-review.md、task2-review.md、task-final-root-review.md；最后两处CANCEL新故障代次与manual信号处理问题已修复，并有客户端回归。legacy同样在退出前保留ROS context完成有限取消确认。

信箱TASK_ID=`go2-resilient-experiments-20261105`，本机go2/对端ranger，doctor通过。方案/原因/计时及最终实现消息均送达；最后实施消息ID=51dee2e993864b529e6287c05c922df5，基线更正消息ID=6eaeaeb1b556405186e2c55b2157294d。连续三次45秒无回复后停止主动等待，最终receive仍空；未把未回复当兼容通过。Ranger需核对三条新增软原因、wait_recovery显式开关、有效任务钟和同task恢复握手，保留自身硬件差异。

未执行真机、Domain0节点、SSH硬件操作、桥接、录包、真实模型或双端恢复验收；仅信箱通信获授权。普通Twist不能识别所有DDS迟到包，依赖原生终态/所有权/清缓存；未知native send可能已送到服务端但无可取消handle，必须人工确认原动作/实体停止。断电、执行器进程退出、内存进度或必要证据丢失不能可信续接。本轮不自动部署、重启现场组件或提交。

最终安装广集：1089通过、11失败、7排除，日志installed-cooperative-final.log；11个失败ID与原HEAD/config基线集合逐项完全相等（baseline-comparison.txt），新增parity失败已消除。三包最终构建退出0（build-delivery.exit）；必要定向CTest真实用例数94，非CTest条目数5。最后补手动待命/人工取消后的诊断持续刷新，避免窗口停在旧启动状态；随后同步构建及安装检查，未启动ROS现场。


### 2026-11-05 Ranger信箱回信与兼容性核查

此前“尚无回复”为历史状态。现已收到Ranger报告45d470f327c84ce49082295bbe49e9d9及协商请求c2313f2c7e8849e78a2335053c544a5d；Ranger报告基线a1c1b4c136b285b98732570a63ef2f1c3548eff3，工作树有既有未提交内容，仅只读核查。对端报告当前没有wait_recovery、新三条软原因被判hard、协同STOPPING/PAUSED/awaiting_peer_resume仍消耗总预算。Ranger报告80项离线通过，但本机未读取Ranger实际代码或独立重跑，不能当成本机验证。当前组合明确尚不兼容长暂停自动恢复。

Go2已逐项核对实际models/agent/coordinator/runner/exploration及mock构造入口，复跑相关四文件116项纯离线通过，未初始化ROS。通过信箱回复0fc3bf26ebbf4ac18b04c03583b1747d，delivered=true，原五条消息已确认处理。来信及回复正文保存于build/go2-resilient-experiments-20261105/evidence/ranger-contract-incoming.json、go2-contract-reply.txt；验证日志mailbox-contract-check.log。

约定方向：精确软原因与cause分类、有效任务钟、硬故障优先、同task停稳/原生终态及peer恢复握手、保留成功command和平台各自检查点。任务消息schema未改，Sport新srv只Go2本地。Ranger同步不能仅加配置，须核对实际构造入口及Nav2恢复计时消费者，不能整体覆盖有平台差异的adapter。

明确待解决/待验证：双方实际混合版本状态序列回放尚未完成；远端新hard尚未送达时，本机可能仍按上次有效状态行动，现协议不提供全局原子恢复或零延迟同步停车。已收到的hard会锁存、撤销待发恢复，但不能宣称感知尚未收到的消息。必要证据定义也待对齐：Go2 runner.start_local_execution.record在single_robot_mode下吞掉events.jsonl/展示异常并仅warning，外层_motion_event无法据此标记缺口；若events.jsonl为必要验收证据，该路径尚需补缺口记录/最终验收拒绝。选点/图像必要证据失败已有硬错误路径，普通诊断窗口写失败不应妨碍停车。该缺项已如实告知Ranger，不能声称所有记录异常均已被最终验收捕获。

本轮只做信箱通信、代码核查、离线复验及本交接追加；未修改运行代码/配置，未构建、部署、连接机器人、启动桥接或运动。等待Ranger确认约定边界；其本轮只读授权不因收到消息扩大。


### 2026-11-05 双端恢复同步：Go2本地完成、等待Ranger新版本

用户新增要求为“协商完成同步，使双机实验不会出现协同矛盾”，随后确认Ranger继续处理。继续原授权范围内的代码/离线工作，不启动现场。计划见[双端同步](../superpowers/plans/2026-11-05-dual-recovery-sync.md)。Ranger消息978b36515c6846b68e64c21e216d9ec8明确其本机用户已授权最小同步及前后七文件源码交换，实际旧工作树快照整体SHA256为82ff9d28795a69a3571fe3ae35d3efb4d07fb8356e17d40bd4d19ddb312d52d2，七文件逐项校验保存ranger-old-manifest.json，未当成HEAD导出。

本机证据缺口已修：runner先归档必要events，写/序列化失败锁存adapter.evidence_write_failed，后续补写不能清除；on_event、path/marker、最终stdout独立捕获，仅记录展示异常。物理SUCCEEDED继续保留，诊断独立INCOMPLETE/EVIDENCE_INCOMPLETE；单机finally的result失败返回非0，已停稳正常释放motion_slot，停车未确认仍抛StopUnconfirmed并保留对应标记，不被展示错误遮盖。未加wire字段或运动门槛。独立审查dual-sync-evidence-review.md中的最终emit问题已修复，无未处理重要项。

新安装为 `/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-dual-recovery-sync-20261105/overlay/install/local_setup.bash`，完整命令见[runbook](go2_runbook.md#2026-11-05-双端恢复同步与证据完整性)。三包首次26.4秒构建成功，默认环境/Launcher/旧总入口切换后增量三包0.92秒成功；上一安装未重建。核对59模块、98脚本/launch/config、三包prefix、srv及bridge二进制。首次手动安装检查未设置COOP_SETUP_FILE被父子终端一致性检查拒绝；补齐与expected-setup相同的显式环境后通过，未为绕过检查修改代码。

定向源码和/tmp加载新安装分别328通过、3排除，分组为本轮新增19与既有309（不把两次相加）。排除1项实际ROS初始化测试及2项已有基线断言失败；安装日志installed-targeted.log。入口/环境71项通过。此次未重跑全库，上一全库11项基线失败未宣称修复。构建/安装日志在build/go2-dual-recovery-sync-20261105/evidence/；源码故障注入、审查与信箱/回放证据继续在build/go2-resilient-experiments-20261105/evidence/。

实际Go2新+Ranger旧回放11场景：5通过、6不兼容。软恢复navigation/single_link/delayed_cancel在旧Ranger约601秒触发task timeout；localization/compound被旧定位原因判hard；execution因不认新原因判peer paused during task。三hard交错与两native迟到/已完成场景通过。单向丢包1300 tick保持暂停/等待确认，无运动推进effect。Go2+Go2同源码11通过仅是驱动自检，不能当双新。全部wire字段严格比较，内部API仅桥接构造及receive_peer签名，未改快照。详情dual-version-replay-report.md及dual-version-ranger-old.json。

驱动发送首次被自动审批拒绝，理由为缺少针对源码文件/接收方的明确授权；已提示音询问，用户明确“允许发送给Ranger”后重送成功，消息77eb3f4afa6342b99a08ec885fc597be。包dual-version-replay-driver-transfer.json约8KB，仅测试代码/说明，SHA256=9248100331ee8e2d5cea9665d8f287a3d647e15827efe5ec9a3f68c0e4cba1f6。此授权不扩展到启动机器人。

当前待Ranger实际新快照、本地构建/验证和同驱动双新回放，尚不能声明同步闭环。没有启动ROS现场、Domain0/40桥接、录包、ARM或运动；没有提交。异步通信的远端新hard尚未送达窗口仍存在，不能承诺零延迟全局原子停车或绝不冲突。必要历史证据丢失也无法靠恢复磁盘后补成完整验收。


同步过程续记：用户随后明确“Ranger正在本地执行相关任务，不要因此断联”，因此按最新指示保持信箱协作，不因此前三次无回复结束任务。离线驱动独立审查发现v1遗漏LOCAL_EXECUTION统计、成功command内部重跑、旧active被新状态覆盖及部分暂停仅终点检查；修订v2已逐项关闭，六场景各1300tick检查、硬故障按各端实际接收时刻保护，旧active实际参与一次决策。v2自检仍11/11、实际新旧仍5/11；额外10项内存错误effect注入均被断言捕获。两个native结果场景仅覆盖Go2状态机，未扩大为Ranger原生执行器验证。v2包SHA256=547729f82ffdbe0e7c67dabc5231df472bb82e0c37b1665e0cad26c5c33dcea3，消息2307ca01fae34fe2b3fa587d279b347e送达，替代先前v1；源码/旧证据保留。审查报告dual-version-replay-review.md。本轮新增文档链接无缺失；历史文档中指向旧文件名“../双机协同启动与验收.md”的31处链接在原HEAD已缺失，未以本轮检查掩盖，详情新证据目录document-links.txt。


为核对Ranger本端软故障路径，驱动v3增加同六类故障的Ranger侧镜像，共17场景；本机同源码自检17/17，实际新旧5/17（12不兼容），16项内存错误effect注入均被捕获，独立复核无新增问题。v3代码外发被自动审批拒绝，理由为新增场景超出此前具体源码发送授权；采用安全替代方案，v3仅本机执行，对端仍用已授权送达的v2复验，通过信箱只报告新增检查结果（消息140b2ee54bd64e60b4a2b99825a4672b）。没有绕过拒绝外发v3，也不因此中断等待新快照。v3包SHA256=c360a26d0f3796fca5d33dd77c6cbcb86420ca4afecce00228b6e6f6c1fe7069，实际结果/回放代码继续保存在原证据目录。


### 2026-11-05 双端恢复同步：实际final2互验

收到Ranger候选1（消息a13eff3c0cbf4b7c9c97f84b875171cd）后，本机对实际七文件完成17项双新回放。独立审查随后发现首EXECUTE排队与同周期hard交错：Ranger状态已PAUSED，tick仍可返回LOCAL_EXECUTE、STOP。三个真实快照反例均复现；不据此推断实体已运动，Ranger报告runner已有paused分派拦截。问题经消息2c0071e4fc224a288462f4c6268d632e反馈，Ranger新增三参数红绿测试并修复stop清队列，以及soft→hard以携带pause_goal_id的hard STOP替换旧队列，不改变停稳/原生终态或检查点条件。

Ranger最终候选2来自消息57aa269b6f5247aab589e67dbe852038，整体SHA256=3c9eefcc29ee5b61f3258f1e7bbe38631f3fdbc56ae0abfc73243224f5e658ae，agent.py SHA256=caaf2ac403335e2be6ac4ec4dcb1cb080f9973c03d33bc50ccbd3117256b2ff7，其他六文件与候选1相同。原始内容和逐文件来源在ranger-final2-source.json、ranger-final2-manifest.json。本机实际Go2新+Ranger final2七文件：冻结v3的17/17通过，独立三个首EXECUTE+hard反例3/3通过，三例输出仅STOP。两个native场景仍仅Go2状态机，不能代替Ranger原生动作执行器验证。运行前后hash不变、未导入ROS；独立最终限定审查无剩余重要项。汇总ranger-final2-local-verification.json，场景dual-version-ranger-final2-v3.json与ranger-hard-queue-final2.json，原红灯与候选1证据保留。

本机load_config(当前源码config)→新安装make_machine(go2,False)另确认wait_recovery=True、scene=True、share_c_coordinates=False、simulation=False，actual-constructor.json记录。Ranger报告actual current-acceptance/config→make_machine(ranger,False)及coordinator开启wait_recovery且private=True；source build/ranger-local/install/setup.bash后7个有关模块resolve到本仓库源码，无旧overlay，Python安装使用symlink，未重启现场进程。该安装与执行器证据为对端报告，本机独立运行的范围是收到的七个纯状态文件。

Ranger本机报告runner/adapter恢复事务性、取消RPC异常仍发零速、同task固定目标/三次旋转预算保持、必要证据与展示隔离专项通过；扩回归704通过、8个field配置断言失败已用其HEAD配置/实现复现，另排除1实际ROS初始化项。这些数字不是Go2本机运行结果。对端v2候选1实际双新11/11、旧混合5/11；final2最终回归与冻结确认仍待其回复。Go2已发送final2的17+3结果与hash（182ecdc8b923447a984874d33fd7481c），持续信箱协作。


### 2026-11-05 双端恢复同步最终闭环

Ranger最终确认消息2ae972e901194063a8dbeed43e7f650a：final2七文件与前述整体及agent hash完全不变，不需等待新快照。本机已实际复验17/17双端状态场景及3/3同周期hard队列反例，独立审查关闭已发现窗口；Ranger报告独立审读v2、使用实际Go2模块与arrival.py复验final2 11/11通过（旧实际混合5/11），额外soft→hard派发前后、重复hard及完成标记保留断言通过。双方确认本轮“规则协商、最小实现同步及限定离线互验闭环完成”。

最终Ranger数字以该确认替代前述阶段报告：27文件定向707通过、8个原基线失败、1项现场接口初始化旧测试排除；8项用其HEAD config/exploration/check独立进程同名复现。新增兼容21、runner停车优先10、恢复安全20、必要证据20为对端专项分组，不与707相加。runner恢复事务/取消RPC失败零速、固定目标和三次旋转预算跨同task保持、记录完整性专项由Ranger本机完成；Go2未运行Ranger实体adapter。对端证据在build/ranger-local/diagnostics/go2-wait-compatibility-20261105/validation-manifest.json。

Ranger最终报告实际current-acceptance-run配置→make_machine(ranger,False)/make_coordinator的wait_recovery均True、private=True；source build/ranger-local/install/setup.bash后模块和share/scene.yaml、robot_ranger.yaml均resolve当前源码。其prefix为build/ranger-local/install，纯Python/配置链接安装，无新overlay、无运行进程重启。Go2本机新安装及328定向（新增19/既有309）/71入口验证、59模块/98文件来源核对见前述节及新输出evidence；本轮未重跑Go2全库，历史11项基线失败未宣称修复。两端数据来源明确分开，不把回执当成测试。

Go2已回复最终确认并ack，双方本轮停止等待新快照。默认安装为build/go2-dual-recovery-sync-20261105/overlay/install，原已验证overlay未重建；最终启动文档和安装内文档同步。v3源码外发拒绝未被绕过：本机完成新增测试，仅共享结果，对端独立使用已授权v2。

本轮未部署或重启现场、未初始化现场ROS、未桥接/录包/ARM/运动，未提交，保留既有dirty工作。现场DDS/RPC、物理停车和恢复运动仍须实际验收；收到hard后的锁存已验证，但远端新hard尚未传播时不能保证全局原子停车。人工停止/接管须人工处理；进程/内存任务丢失、身份或地图变化、原生动作终态无法确认不能可信自动续接；必要历史证据丢失即使恢复写盘也不能补成完整验收。

最终收尾：5秒WAV完成提示已播放成功；git diff --check与修改Bash语法通过，59安装模块hash未变，新增链接无缺失。最终信箱回复cd43bb1d4087470c8a007e257799605c送达，Ranger最终确认已ack。


### 2026-11-05 Agent入口文档旧安装路径更正

用户按正式启动文档Agent示例显式export了build/go2-wait-recovery-20261104/overlay-final/install，安装校验报No module named 'vlm_nav.go2_control_recovery'。根因是此前更新最新入口时漏改该示例一行；CoVLM_env.bash保留显式COOP_SETUP_FILE优先，因而覆盖了新默认安装。旧安装确实没有该模块。未启动实验，以同一--check-install只读调用复现旧安装退出1/同错误；改为build/go2-dual-recovery-sync-20261105/overlay/install后退出0，59模块安装来源/源码校验通过。

本轮仅更正docs/10.22双机协同启动与验收.md的Agent示例并说明覆盖旧export，记录本交接；同步这两份文档到当前安装share副本，不改运行代码、不重建旧或新overlay。原错误命令停在安装校验前置，不需安装pip包或绕过检查。用户需在原终端显式export新安装后再按原Agent入口运行。仅检查命令格式和diff；未启动ROS、模型、ARM或运动。

### 2026-11-06 Piper 机械臂使能与导航保持记录

**来源与结论：**用户在 Go2 上位机执行本会话提供的命令并回传终端输出；使能后用户确认“机械臂能保持姿态”，随后确认当前姿态可行、行走中的保持效果良好。本轮“通过关节使能改善导航时机械臂被动摆动”的目标据此完成。现场保持与行走效果为用户报告，本机未通过 SSH 连接、独立采样或执行运动；不能扩大为高速度、长时间或携带物体后的稳定性验收。本次“记录”仅授权保存交接，不产生新的现场操作授权。

**设备与环境（用户回传）：**

- 机械臂 AGILEX Piper，铭牌 S/N：`MA100101000019005Y03487`；硬件版本回复 `H-V1.2-1`，软件固件 `S-V1.8-2`。版本后的 `251121` 未确定其含义，不并入软件版本。
- USB-CAN 接 Go2 上位机，不接本项目所在的笔记本。上位机登录提示 `unitree@ubuntu`，用户使用 SSH 别名 `unitree-go`；未由别名推断实际 IP。
- 上位机 Python `3.8.10`、架构 `aarch64`，安装目录 `/opt/ros` 中有 `foxy`、`noetic`；本次采用独立 Python SDK，不套用 Humble ROS 驱动。
- USB ID `1d50:606f`，驱动 `gs_usb`，接口 `can0`，速率 `1000000`；启用后为 `UP` / `ERROR-ACTIVE`，用户回传的总线错误及 bus-off 计数为 0。收到 Piper 状态、位姿、关节、夹爪及电机反馈，`0x4AF` 固件查询也收到真实回复。早期 RX dropped 较高，未据此计算物理链路丢包率，亦未完成专门丢包分析。
- Python 环境 `/home/unitree/piper-venv`；使用 `/home/unitree/piper-venv/bin/python -I` 隔离 ROS 的外部 Python 路径。普通 pip check 曾出现 `launch-ros` 缺 `pyyaml`，加 `-I` 后为 `No broken requirements found.`，未为此修改 ROS 或补装其依赖。
- 在线安装失败的明确证据是 `Temporary failure in name resolution`，不是已证实的 SDK/CPU 不兼容。按 `piper_sdk==0.6.2` 离线方案安装；用户随后成功运行 `C_PiperInterface_V2` 查询及使能代码，但未单独回传最终 pip 版本清单。
- 本机会话曾准备 SDK 及依赖共 6 个 wheel：`piper_sdk 0.6.2`、`python-can 4.5.0`、`msgpack 1.1.1`、`packaging 26.2`、`typing_extensions 4.13.2`、`wrapt 1.17.3`，核对 Python 3.8 元数据、aarch64 wheel、离线依赖解析及文件完整性。历史离线包 `/tmp/piper-offline-0.6.2-py38-aarch64.tar.gz`，SHA256 `c2d84584f80a6e2feceb8dd6056269d71864a10a79758dc73824d4f33491816e`；临时文件不作为长期可用安装源，后续检查时原缓存 wheel 已不存在。

**使能前后证据（用户粘贴输出）：**

1. 使能前，两次采样六关节均 `enabled=False`、`faults=[]`；状态 `STANDBY` / `NORMAL`、错误码 0、示教关闭，关节限位及通信异常标志均 False。电压反馈关节 1～5 为 24 V，关节 6 为 23 V；未报告欠压，不代表负载下供电已验收。
2. 使用 V2 接口 `ConnectPort(piper_init=False)` 建立连接，单次调用 `EnableArm(7)`；该操作未附带夹爪、归零或目标角度命令。使能反馈时间戳 `1793951722.3950057`，六关节均 True，输出 `ENABLE_CONFIRMED: all six joints report enabled.`，状态正常、错误码 0。
3. 使能脚本以 `DisconnectPort()` 结束，不发送失能。退出后重新运行查询，反馈时间戳 `1793951820.684835`、`1793951821.6904502` 持续更新；两次采样六关节均 True、`faults=[]`，状态正常。无需让本次一次性使能脚本持续运行；该结论不等同于掉电或所有故障情况下仍保持姿态。
4. 用户明确说明中途 CAN 接口关闭是上位机重启所致，重新启用后恢复；无须重装 SDK。CAN 初始化与机械臂电机使能是两个不同步骤，重启/重新连接后须重新检查实际接口状态。

**姿态采样记录：**以下为上述退出后两次查询中相同的关节反馈，原始单位为 0.001 度；是本会话已有采样，不是本次记录时重新读取的角度。用户后续认可当前姿态与行走保持效果，但未提供行走后的关节快照，不把下表直接写成自动回位或导航默认配置。

| 关节 | 原始反馈（0.001 度） | 换算角度（度） |
| --- | ---: | ---: |
| J1 | -4307 | -4.307 |
| J2 | -7802 | -7.802 |
| J3 | 279 | 0.279 |
| J4 | 20856 | 20.856 |
| J5 | 31922 | 31.922 |
| J6 | -67391 | -67.391 |

**后续待办与边界：**尚未创建长期启动脚本、开机自动使能或持久姿态配置，未修改导航入口、参数、固件或电机增益。下一步可在明确范围内整理 CAN 初始化、状态读取、按需使能及反馈核验入口；需要回到上述姿态时仍须确认安装、限位及路径，不能将角度表直接作为未经验证的运动命令。抓取所需机身到机械臂基座/相机外参、TCP、夹爪、负载及碰撞模型尚未配置或验收。本次仅追加文档，保留已有未提交工作，不构建、不部署、不启动 ROS 或真机验证。

### 2026-11-06 Piper 开机初始化与使能实现（待 sudo 安装及冷启动验收）

用户明确要求实现 CAN 开机初始化和开机使能，并确认 Go2 首次运动是开机完成后由用户遥控或启动导航。本轮授权范围内已通过 `unitree-go` SSH 只读核对环境并上传服务候选，没有发送使能、运动或重启命令。当前源码仍为分支 `go2-cooperative-task-layer`、HEAD `323d5c5`，保留已有 dirty 工作，不提交。

上位机实查 systemd 245、SDK 0.6.2、python-can 4.5.0；Piper USB-CAN 唯一序列号 `0046002F5246570520323934`、VID/PID `1d50:606f`。新脚本 `scripts/go2_piper_boot.py` 按该身份寻找接口，对关闭接口设置 1 Mbps；已启用且速率错误或 BUS-OFF 时等待处理。逐个核对状态帧和六关节反馈新鲜度、已有故障/示教状态，正常且未全使能时调用 `EnableArm(7)`，清空确认缓存后等待完整新反馈；成功退出，不后台反复干涉后续抓取或人工失能。`--inspect` 不配置 CAN、不发送使能。停止/退出不发送失能、回位、夹爪、复位或清错。

独立服务 `config/piper_boot/piper-boot.service` 使用 Type=simple、RemainAfterExit=yes，避免故障等待阻塞 multi-user.target；以 unitree 和 CAP_NET_ADMIN 运行，上位机实际入口为 `/home/unitree/piper-venv/bin/python -I -u /opt/piper-boot/go2_piper_boot.py --config /opt/piper-boot/piper.json`。状态与原因类写入 journal；人工独立诊断入口 `/opt/piper-boot/piper_boot_status.sh`。active(running) 可能仍等待，active(exited) 与 READY 仅证明该次开机流程完成，不是持续健康承诺。不改变 Go2 导航/底盘启动，不添加导航运动门槛。

安装器 `scripts/install_go2_piper_boot.sh` 预验证 SDK、Python/JSON/unit，拒绝覆盖既有部署并加安装锁；首次安装失败撤销本次新建内容，安装日志 `/var/log/piper-boot-install.log`。仅 enable，不 --now、不重启。上位机 sudo -n 明确需要密码，故服务尚未安装到系统目录或启用；最终安装必须由用户在上位机 Bash 终端输入 sudo 密码完成，不收集密码。候选目录 `/home/unitree/piper-boot-20261106/`，最终从本次构建产物打包并校验 SHA256SUMS；步骤见 [runbook](go2_runbook.md#2026-11-06-piper-开机-can-初始化与关节使能)。

验证：新增 `test/test_piper_boot.py` 24 项通过，覆盖身份选择、逐关节新鲜度、故障与控制状态、只读、使能后新反馈确认、只读超时后重连、畸形帧、设备晚到及 main 退出。独立审查提出的 oneshot 阻塞启动、失败覆盖旧部署两项已修复，复核无剩余重要静态缺陷。真实上位机 Python3.8 双次只读 session 均 READY，发送计数前后均 3，未发送 CAN 控制报文；证据 `build/go2-piper-boot-20261106/evidence/upper-readonly.log`。systemd245 unit 校验仅出现宿主已有 netplan/snapd/varlink 警告，不能据此宣称真实 service 已启动。

按 runbook 已完成本轮新输出 `build/go2-piper-boot-20261106/overlay` 的三包首次构建（27.2 秒），原 overlay 未重建。最终增量构建、定向 ctest 及部署清单见后续收尾记录；独立上位机服务使用本轮安装产物，不切换无关导航 CoVLM 默认 overlay。未跑全库、未做安装器实际失败注入、未做系统安装/冷启动或异常恢复真机验收。历史姿态表仍不是自动运动配置。

收尾补充：新增 socket 排队旧帧反例，先复现错误 READY，再按内核接收时间戳过滤本次连接/使能之前的报文，最终 24 项通过（均为本轮新增，不含既有测试）；不把 colcon 的 pytest 包装条目计作额外用例。

最终交付：三包增量构建0.95秒成功；同一输出定向ctest通过，实际pytest为24项（colcon汇总25含包装条目）。本轮仅新增测试，未重跑既有全库。五个服务资产由新安装share复制打包，源码/安装/上传SHA256逐项一致，清单build/go2-piper-boot-20261106/evidence/deployment-SHA256SUMS。最终上位机脚本Python3.8语法、两次无线程只读连接和重连均通过，六关节READY，TX前后仍为3；upper-readonly-final.log及upper-package-check.log记录。独立最终审查复现排队旧帧红灯并确认修复，24项复核通过，无剩余重要静态项。系统目录尚无本次服务，需用户在/home/unitree/piper-boot-20261106运行sudo bash scripts/install_go2_piper_boot.sh后确认is-enabled；不把上传或unit静态校验当成安装/冷启动成功。

### 2026-11-06 Piper 开机服务已安装并启用（待冷启动验收）

用户随后报告 sudo 安装成功、显示 enabled。本机通过 `unitree-go` 仅做只读核验：`UnitFileState=enabled`、`ActiveState=inactive`、`SubState=dead`，ExecStart 未运行，unit 来源 `/etc/systemd/system/piper-boot.service`。实际 ExecStart 使用 `/home/unitree/piper-venv/bin/python -I -u /opt/piper-boot/go2_piper_boot.py --config /opt/piper-boot/piper.json`。已安装 Python 入口、诊断脚本、JSON、systemd unit，以及保留在上传目录的安装器，均与部署 SHA256SUMS 一致。

本条更新此前“待 sudo 安装”的状态：系统安装和开机启用已完成；当前 inactive 符合仅 enable、未 --now 的安装设计，不是启动失败。下次用户安排正常开机后，按 runbook 在上位机独立诊断终端运行 `bash /opt/piper-boot/piper_boot_status.sh`，检查该次开机日志 READY 与 active(exited)；需要核对当前真实状态时再运行只读 --inspect。尚未执行自动服务首次启动或冷启动验收，本次没有发送使能、运动或重启命令。仅追加文档，不重建代码。

### 2026-11-06 Piper 姿态调整完成、重新使能及自启动保留

用户要求暂时失能以手动调整姿态，随后重新使能并保留开机自启动；本会话提供了上位机操作步骤，本机未通过 SSH 执行。用户最终确认“已经调整好姿态”。未提供调整后的关节角度，因此前文历史角度表不代表当前姿态，也未保存为自动回位配置。

用户回传 `systemctl show piper-boot.service -p ActiveState -p SubState -p UnitFileState -p Result` 输出：`Result=success`、`ActiveState=active`、`SubState=exited`、`UnitFileState=enabled`。上位机 `/home/unitree/piper-venv/bin/python -I /opt/piper-boot/go2_piper_boot.py --config /opt/piper-boot/piper.json --inspect` 随后由初始 `WAIT_FEEDBACK` 转为 `READY`，六关节反馈全部 True。用户粘贴的 journal 还记录了 17:18:05 `ENABLING`、17:18:06 `READY`。这些证据确认该次服务已执行成功、检查时六关节已使能且开机启用配置保留；更新此前“服务尚未执行”的状态，不等同于冷启动验收。

诊断脚本使用 `journalctl -f` 持续显示日志，不自动退出属于预期；开头 systemd 状态仅为启动查看时的快照。Ctrl+C 只结束日志查看。服务仅初始化 CAN 并使能，不记录或恢复指定关节角度；掉电后姿态若改变，下次开机不会自动回到本次姿态。

后续目标为 Go2 单机发现地面物体后抓取，用户候选为水杯或积木，具体物体、尺寸和重量未定。当前仍待确认站姿下地面可达性、调整后姿态的保持及行走效果、机械臂外参/TCP/夹爪和实际抓取；单机入口尚未接入抓取。本轮仅追加交接，不修改代码、不构建、不部署、不启动 ROS 或真机验证。

### 2026-11-06 17:24 调整后姿态只读实测

用户明确要求“读取”当前姿态，本机通过 `unitree-go` SSH 使用上位机 `/home/unitree/piper-venv/bin/python -I` 读取，按 `/opt/piper-boot/piper.json` 的 USB 身份确认 `can0`。SDK 连接使用 `piper_init=False, start_thread=False`，只接收并解析反馈；每次采样均等待连接/采样起点之后的状态帧、六关节使能反馈及三组关节角反馈，不使用排队旧帧。未调用使能、失能、夹爪或运动接口，CAN TX 计数前后均为 5。

上位机时间 `2026-11-06T17:24:46.215279+08:00` 和 `17:24:47.241150+08:00` 两次新鲜采样完全一致，所需帧最大接收年龄分别为 0.0201 秒、0.0178 秒；两次均 `READY`、六关节全部使能。

| 关节 | 原始反馈（0.001 度） | 当前采样角度（度） |
| --- | ---: | ---: |
| J1 | 1786 | 1.786 |
| J2 | -7740 | -7.740 |
| J3 | 2129 | 2.129 |
| J4 | 0 | 0.000 |
| J5 | 21456 | 21.456 |
| J6 | -44527 | -44.527 |

本表补充前一节“未读取调整后角度”的缺口，仅为当前姿态记录，不是自动回位配置或安全轨迹。未读取末端坐标、未验证地面抓取或行走保持，也未执行冷启动。SSH 出现 known_hosts 自动更新受只读文件系统限制的提示，但连接、两次采样及正常退出均成功；未修改 SSH 配置。

### 2026-11-06 地面抓取离线几何初筛

用户提供侧面照片，随后要求按安装面离地 60 cm、基座轴到雷达/护架前缘水平 50 cm 讨论并继续。两尺寸为讨论假设，不作为实测标定。候选官方 URDF 与固定版本 SDK 0.6.2 的正运动学完成 302 组离线位置交叉核对，最大差异约 0.14 mm；模型和实机的零位/限位/TCP 仍未完成标定。

假设基座水平、目标水平距离 50 cm、夹持点离地 5 cm，肩部比安装面高 12.3 cm，肩部到目标约 83.84 cm。按模型连杆长度的三角不等式，法兰到夹持点距离至少需要约 21.07 cm 才满足任意朝向的必要条件；假设工具长 10/15/20 cm 时均超出上界。此为有条件的排除，不是所有地面位置都不可达的结论。公开夹爪模型版本不同，未把手指关节原点当实测 TCP，也未通过照片确定版本。当前实测 J2/J3 略超公开 URDF/SDK 默认范围，原因待核实，未改限位或校零。

方法、来源、限制及下一步见 [Piper 地面抓取可达性初筛](piper_ground_reachability_20261106.md)。计算脚本、模型快照、哈希、result.json 和图位于 `build/piper-reachability-20261106/`，仅为离线证据，不是长期机器人入口；build 不受 Git 保护，不清理。下一步须实测安装几何和法兰到夹持中心长度，并选择具体物体；再核对可达候选、真实模型与整条无碰撞路径。本轮无 SSH、ROS 或控制命令，未修改产品代码或安装产物，不构建、不切换 overlay；尚未完成 IK、碰撞和真机抓取验证。

### 2026-11-06 用户手动伸展：正前方约 10 cm 以上、肩部侧方约 5 cm

用户在收到临时失能与恢复步骤后报告已做伸展检查：沿中轴线从头部相机前方向下，距地面约 10 cm 或更多；从头部侧方、肩膀附近向下，距地面约 5 cm。用户进一步确认两数值均为地面到夹指最下端（指尖）的距离，不是夹持中心高度。对应侧别、水平距离、关节角、机身姿态、碰撞余量及当前使能状态均未采样，不能视作完整空间极限或正式抓取成功。本机本轮未执行 SSH 或控制操作。

后续优先核对侧方候选：检查物体可夹持部位与指面有效接触区域的重叠，再核对前腿/机身干涉及停车后相机可见性，不直接把 5 cm 当作 TCP 高度或可抓物体高度阈值。详细解释已补入 [可达性初筛](piper_ground_reachability_20261106.md)。仅更新文档，未构建或改变导航/机械臂运行配置。

### 2026-11-06 独立夹爪调试工具完成（未上传、未真机执行）

用户明确尚未实物尝试，选择目测高约20cm、宽约7cm的杯子，随后确认夹爪宽度可用、不再讨论这一点，要求开始调试。本轮按此范围完成独立工具，不把宽度目测改写为实物抓取通过，不自行控制机器人。

新增 `scripts/go2_piper_gripper.py` 与 `scripts/piper_gripper_debug.sh`：上位机 Python3.8/SDK0.6.2，显式加载同目录 `go2_piper_boot.py`，复用USB身份、只读CAN与现有使能/故障反馈检查。`inspect`被动持续监测，不发CAN；`set --width-mm ... --effort-nm ...`查询已有驱动范围后，最多尝试一次GripperCtrl，故障/断流/重连不会重发已尝试的动作。查询不会改行程、零位或负载；不调用六关节、底盘、失能或清错接口，也不改开机服务。Ctrl+C退出监测而非物理停止/释放。诊断明确反馈与实物结果的区别，grasp_confirmed始终false。

主终端只显示日志路径；有图形会话时尝试打开诊断窗口，SSH无图形时第二终端执行 `bash piper_gripper_debug.sh diagnostics` 持续跟踪原子切换的current.log。真实日志位于上位机 `~/.local/state/go2-piper-gripper/`；set持有按USB身份划分的本用户控制锁。入口故障可等待恢复，不通过额外杯宽验收解锁。命令参数有效性与驱动已配置范围仍核对，SDK力矩单位为N·m，不是指尖夹持力N。

验证：新增 `test/test_piper_gripper.py` 42项，既有 `test/test_piper_boot.py` 24项，合计66项通过；Python3.8 AST语法与Bash语法通过。先复现再修复：清理异常覆盖KeyboardInterrupt导致继续重连；SDK0.6.2异常夹爪帧被丢弃后缓存不更新，入口仍标新鲜。现在保存解析前时间戳数值，只有SDK确实更新0x2A8缓存才登记新鲜，拒绝时移除对应seen。独立审查复核66项，无剩余重要静态缺陷。未跑全库，未实机验证Python3.8/反馈/夹爪动作、杯子夹持或抬升。

按runbook新输出 `build/go2-piper-gripper-20261106/overlay` 三包构建26.8秒成功（既有CMake/PCL等警告保留）；同输出定向ctest通过，colcon汇总68含两个pytest包装条目，实际用例66。三个包前缀均核对本次输出；独立上位机工具不切换导航默认overlay。安装share中三脚本与源码字节一致，从share打包到 `build/go2-piper-gripper-20261106/package/`，附SHA256SUMS；证据位于同级evidence目录。构建、上位机部署及两终端依赖步骤见 [runbook夹爪调试节](go2_runbook.md#2026-11-06-piper-独立夹爪调试入口)。尚未上传或部署，不覆盖/opt/piper-boot，不自动发送现场动作；用户下一步先上传核验、inspect，再空载打开，之后逐步原地夹持/释放。示例70mm/0.2N·m、后续68mm只是显式调试目标，不是已验收抓取参数。

### 2026-11-06 夹爪不动作：实物合拢与反馈108.1mm不一致

用户随后部署到上位机 `/home/unitree/piper-gripper-debug-20261106`，两次执行 `set --width-mm 70 --effort-nm 0.2` 后均报告不动作，明确允许本机自行读取日志并继续排查。SSH只读核实SDK0.6.2及部署脚本；第二轮日志 `~/.local/state/go2-piper-gripper/set-joaxi5f3.log` 从时间戳1793968102.000 STARTING、1793968102.641 COMMAND_ATTEMPTED后持续反馈108.1mm，未见SDK发送错误。日志快照保存在 `build/go2-piper-gripper-20261106/evidence/upper-retry-set-joaxi5f3.log`，共292行，末行EXITED/USER_EXIT，表示该轮监测退出，不是物理停止或失能。

只接收CAN的2秒采样（结束1793968274.205）收到399帧0x2A8、400帧0x2A1及各关节新反馈；夹爪原始位置0x0001A644=108100、状态0x40，enabled=true、homing=false、故障位全零；arm ctrl_mode=0、arm_status=0、teach_status=0、err_code=0。TX计数7→7，无查询或控制发送。SDK实际发送类是 `piper_sdk.hardware_port.can_encapsulation_v0_4_0.C_STD_CAN`，不能用同包旧 `can_encapsulation.C_STD_CAN` 替代分析。仅内存替身编码 `GripperCtrl(70000,200,1,0)` 得0x159/0001117000c80100，确认单位与编码，不代表当时总线报文或固件接受证据。源码及采样在同级 `zero-mismatch-readonly.txt`。

用户进一步确认当前夹指“基本合拢或指尖接近”。**已确认的是实物与反馈开口明显不一致；优先假设为夹爪零位偏移/未正确标定。**当前70mm相对反馈108.1mm是进一步闭合的目标，因此可能已被物理闭合端阻挡。该解释尚待正确校零后的小开口动作验证；homing=false本身不足以判根因，ctrl_mode=0也未证实是阻断原因。未发送实际控制帧抓取，未验证0.2N·m是否足以驱动，不能提前宣称故障已修复。

厂家[Q&A](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/Q%26A.MD)说明闭合时反馈偏大应设夹爪零位；[V2校零示例](https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/demo/V2/piper_set_gripper_zero.py)先仅失能夹爪、等待1.5秒，再以失能状态set_zero=0xAE。下一步顺序：原操作终端退出set并确认没有其他控制进程；空载，仅发送夹爪失能0x159(code=0,set_zero=0)，核对新鲜反馈已失能；操作者确认夹指自然完全合拢后，再单独发送夹爪校零(code=0,set_zero=0xAE)，核对新鲜反馈接近0；随后用小开口（例如10mm、保持0.2N·m）做动作验证。每步依赖上步结果，不把“基本合拢”直接当精确零位，不在张开或夹物时校零。六关节保持使能，开机服务不改；不重置整臂、不改模式、不改行程、不自动提高力矩。

本轮未执行失能、校零或运动，未部署修改。仅记录证据并修正文档中将70mm无条件称为打开/释放的表述；未改代码、不构建、不重跑既有测试。真实校零和夹爪动作仍待现场操作。

后续用户明确“现在是空载，授权验证”。本机按顺序仅发送一次 `GripperCtrl(0,200,0,0)`，未发送六关节控制或校零。发送前新鲜反馈READY、六关节使能全true、夹爪enabled=true；发送后2秒新鲜采样确认夹爪enabled=false、status=0，六关节仍READY且全true，位置仍108.1mm、homing=false；TX7→8。证据 `build/go2-piper-gripper-20261106/evidence/gripper-disable-before-zero.jsonl`（采样时间1793968661.126及1793968663.128）。未观察到0x159本机回环，不以此否认已发生的使能位变化。已向用户询问手动自然完全合拢与手移开确认；该物理前提尚未确认，当前不发送set_zero=0xAE，也不执行10mm打开测试。授权在范围内延续，无须再次询问相同授权。

### 2026-11-06 夹爪校零及10mm验证：位置偏差已纠正，动作故障尚未解决

用户确认“已完全合拢”后，在夹爪新鲜反馈失能、六关节READY条件下发送一次 `GripperCtrl(0,200,0,0xAE)`。发送前107.3mm，发送后2秒新反馈0.0mm；夹爪仍失能、六关节全使能，TX8→9。homing反馈仍false，不能称回零标志已置位或重启后持久化已验证。证据同目录 `gripper-zero.jsonl`。

随后按已授权10mm/0.2N·m通过实际已部署入口执行一次set，约8秒后SIGINT结束监测；日志 `set-n0p351s9.log`。发送前夹爪失能，发送后enabled=true，但位置始终0mm；仅确认使能变化，没有到位。用户因未看清明确要求“重新进行”，再执行一次相同目标（发送前已使能），观察8秒；日志 `set-o1vvqj7q.log`，独立接收的1601帧0x2A8均为0mm，用户现场确认“完全没有动作”。两次均无自动重发、无SDK发送报错；查询及命令TX依次9→11→13。两次程序均以SIGINT正常退出监测（返回130），退出不撤销目标，夹爪保持使能。原始0x47E=6464010000000000，即现有最大行程100mm；0x2A1全零，仍待机模式。未更改行程配置、力矩、控制模式或六关节目标。证据 `gripper-open10-after-zero.txt`、`gripper-open10-enabled-retest.txt`。

结论：闭合位置的107.3mm偏差已被纠正，但不能将原动作故障归结为零位并宣称修复；在已使能状态再发送同一10mm目标仍不动，“第一条命令只完成使能”不能单独解释。下一单变量候选为检查0.2N·m是否足以驱动，厂家独立夹爪示例使用1N·m但这不证明实机最低要求。拟在空载、10mm目标不变条件下单次试0.5N·m并观察8秒，不自动升到更大值；该力矩变更尚未执行，待用户确认。当前不采用整臂模式切换、清错或重置作为猜测性修复。仅更新证据/文档，无产品代码修改、不构建。

### 2026-11-06 10mm/0.5N·m测试与用户要求重试：仍无位置变化

用户明确“执行”，授权10mm/0.5N·m一次测试，随后又明确“重新尝试”，授权相同参数再试一次。两次各观察8秒，每轮实际部署入口仅尝试一次位置命令，无自动重发；每轮后SIGINT结束监测，日志均EXITED，子进程返回130。第一轮 `set-wx7nizcr.log`（1793969091.006至1793969098.958），TX13→15；第二轮 `set-ceis7es8.log`（1793969161.046至1793969169.001），TX15→17。每轮各1601帧0x2A8，宽度最小/最大/最后均0.0mm，夹爪状态始终0x40（使能、故障位全零）；扭矩反馈范围分别0.004~0.075N·m、0~0.070N·m。六关节反馈正常/使能，arm状态帧仍全零；原配置最大行程100mm，未更改。最新一轮现场目视结果尚待用户补充，不能替用户声称已经看清。

证据：`build/go2-piper-gripper-20261106/evidence/gripper-open10-effort05.txt`、`gripper-open10-effort05-retry.txt`。结论仅是将请求力矩从0.2提高到0.5N·m仍未到达10mm目标，不能据此排除所有力矩/机械问题，也不能认定是控制模式或发送频率根因。实际SDK `_create_bus()`明确设置 `receive_own_messages=False, local_loopback=False`（`sdk-local-loopback.txt`），因此独立同机CAN接收器没有观察到0x159回环符合发送端配置，不作为未发送证据。TX计数和SDK无报错也不等同于固件接受位置命令。

本轮未继续加力矩、改整臂模式、清错/重置或重新校零。当前夹爪和六关节均保持使能；监测退出不撤销已发目标。后续应核对固件接受位置命令的条件/发送时序，不能把重复相同命令视作排障进展。仅记录文档及证据，无产品代码修改、不构建。

### 2026-11-06 继续只读排查：固定版官方发送周期与夹爪电机固件线索

用户要求继续排查；本轮未再发运动/查询/配置命令。时间1793969395.056只接收采样仍为六关节READY全使能、夹爪status=0x40、width=0mm、homing=false，arm ctrl_mode=0/arm_status=0/err_code=0；TX17→17。证据 `build/go2-piper-gripper-20261106/evidence/further-readonly-20261106.txt`。在上位机实际SDK0.6.2中以内存发送替身编码 `GripperCtrl(10000,500,1,0)`，生成0x159/0000271001f40100，未接真实发送端；确认参数编码，但不是之前真实物理总线抓包。

已核对与部署版本对应的[0.6.2官方夹爪demo](https://github.com/agilexrobotics/piper_sdk/blob/0.6.2/piper_sdk/demo/V2/piper_ctrl_gripper.py)：循环内每次GripperCtrl后sleep(0.005)，与本工具一次发送不同；示例还包括1N·m、失能清错等差异，因此不能把差异直接当根因，也不原样运行无限开合demo。独立夹爪demo未调用MotionCtrl_2；实际EnablePiper仅调用EnableArm(7)，不会隐式切换控制模式。因此仅凭当前ctrl_mode=0不足以要求切整臂模式。

厂家仓库[issue40](https://github.com/agilexrobotics/piper_sdk/issues/40)报告夹爪使能、低扭矩反馈、无故障但示教软件/SDK位置命令不响应，最终通过升级**夹爪电机固件**解决；这是其他设备用户的报告，SDK版本/硬件不同，不能直接认定本机同根因。主控S-V1.8-2不等于夹爪电机固件版本，当前公开SDK接口查到的是主控固件方法，没有已确认可用的独立夹爪电机版本读取方法。用户对历史是否正常开合回答“不确定”，不能记录为之前正常或一直损坏。另一个[issue17维护者说明](https://github.com/agilexrobotics/pyAgxArm/issues/17#issuecomment-4222517310)针对旧主控从MIT切位置速度模式的限制；本会话未进行此类切换，无证据适用，不据此重置/升级。

下一候选实验已限定为只改变发送周期：空载、目标10mm、力矩0.5N·m、使能code=1、set_zero=0保持不变，按5ms间隔持续最多2秒且最多400次，再只读观察6秒；反馈失效/故障或到达目标即停止后续发送，不重连重播、不改六关节/控制模式、不清错校零。周期发送次数超出此前每轮单次授权，尚未执行，需用户明确允许此有界测试。若响应，才继续定位/修正单次发送策略；若仍不响应，优先核对夹爪电机固件/厂家示教软件，不继续靠加力矩推断故障。本轮仅追加证据和文档，未改产品代码、不构建。

### 2026-11-06 用户授权有界周期测试：373次发送仍无位置变化

用户明确“执行”后，实际使用上位机SDK0.6.2，独占既有USB控制锁，仅查询一次已有夹爪行程上限（100mm），新鲜反馈六关节READY、夹爪已使能且无故障后，发送恒定 `GripperCtrl(10000,500,1,0)`。只在本次进程内包装SDK传输函数校验0x159载荷必须为0000271001f40100，非SEND_MESSAGE_SUCCESS立即停止；没有绕过真实发送、写SDK安装文件或部署代码。接收采用内核时间过滤、逐帧新鲜度及SDK夹爪缓存更新检查；每次发送前检查原有反馈/故障条件，到位或异常立即停止后续发送，无重连重播。

2秒时长上限先到，实际373次调用，首尾跨度1.994853秒，最小调用间隔5.252ms，约187Hz；不能记为实际精确200Hz或400次。停止原因duration_limit，随后只读观察6秒。共1740帧夹爪反馈，width_min=width_max=0mm，状态均0x40，扭矩0~0.074N·m；结束时六关节全使能且READY，夹爪enabled=true，ctrl_mode=0。TX17→391，增量374与1次查询+373次控制一致；传输调用均未报错，但不等于固件确认执行位置目标。过程1793969575.131至1793969583.132，证据 `build/go2-piper-gripper-20261106/evidence/gripper-open10-effort05-burst.jsonl`。

本次条件下周期发送未解决故障，不将单发策略视为已证实根因，也不以一次阴性测试排除所有频率/固件/力矩组合。用户本轮目视结果尚未提供，软件结论限定为无位置反馈变化。未增加0.5N·m上限、切换整臂模式、修改六关节目标或刷固件；实验退出后夹爪和关节仍使能，退出连接不撤销目标。下一步优先核对厂家示教软件下的夹爪响应及独立夹爪电机固件兼容性，尚未验证或升级，不自动启动新硬件实验。本轮仅更新证据与文档，无产品代码修改、不构建。

### 2026-11-06 固件兼容性评估：尚无候选包，不直接升级最新版

用户要求核对后再考虑升级，确认尚无厂家包/候选版本，准备询问曾使用者。本机重新查询0x4AF：实际主控S-V1.8-2、硬件H-V1.2-1、标识ARM_MC；TX391→392只增加一条版本查询，无控制/刷写。上位机Python3.8.10、SDK0.6.2、python-can4.5.0，SDK无V3类，开机服务active/exited且enabled。夹爪电机独立固件、硬件与内部ID仍未知。安装资产哈希和版本回复保存在 `build/go2-piper-gripper-20261106/evidence/firmware-installed-baseline.txt`、`firmware-fresh-query.jsonl`。

官方要求主控1.8-8及以后使用V3；新ID10夹爪要求主控至少1.8-8，旧ID7仍可用。本机实际夹爪ID未确认，不由EnableArm(7)或主控回复数字推断。建议先询问是否有保留主控1.8-2/SDK0.6.2的独立夹爪电机修复包；没有具体目标包、型号/ID/标定保留与回退说明前不承诺升级无影响。实际产品SDK依赖集中在Piper开机/夹爪入口及安装器（固定SDK0.6.2），未接入现有导航抓取栈。

离线从PyPI wheel加载0.6.2/V2与0.7.0a1/V3（预发布候选，仅分析），控制电脑Python3.10.12中禁止CAN socket，真实反馈回放/内存发送替身两组比对：EnableArm(7)与10mm/0.5N·m编码一致，宽度模式READY/AT_TARGET均通过；但V3角度模式mode=1、数值10000被现脚本误判为10mm到位。升级V3需补模式语义验证，不能只换import/pip。两SDK的Python3.8 AST语法通过，不是Python3.8/aarch64实测。候选wheel、SHA256、comparison.json和回放代码文本在 `evidence/sdk-compatibility/`；依赖含本机cp310/x86_64产物，不作上位机部署包。未修改上位机环境、产品代码或构建，未重跑既有66项测试。

详细版本矩阵、依赖、向曾使用者/厂家索取的问题和未代发的摘要见 [runbook固件兼容性评估](go2_runbook.md#2026-11-06-piper-固件升级兼容性评估未刷写)。当前待取得同一设备正常开合时的主控、独立夹爪电机、硬件/内部ID、SDK/接口或示教软件版本，以及操作顺序/更换升级历史；用户要求将这些问题整理成可转发选项。未知版本仍要求原始字符串/截图或明确未知，不根据选项猜测。未发送外部消息、安装刷写工具、重启或追加动作实验。

### 2026-11-10 机载 Piper 抓放与携带接入方案

用户确认使用Go2身上的相机与机械臂，首版原地pick and place，后续抓起后携物导航、到目的地再放置；要求整理整个`~/机械臂资料`工具包与当前环境的匹配情况和两端对齐事项。本轮完成[接入设计草案](../superpowers/specs/2026-11-10-go2-piper-pick-place-carry-design.md)，两端明确为当前控制电脑与机载Piper执行电脑，不是Ranger。推荐保留Humble导航、独立RoboRSI与机载SDK，通过隔离的视觉/IK计算和有限任务接口接入；工具、执行协议、状态和阶段划分均为设计，尚未实施或批准部署。

同会话本机离线核实：默认仍为`build/go2-dual-recovery-sync-20261105/overlay/install/local_setup.bash`，59个Python模块与当前源码一致，但缺少后来新增的`config/piper_boot/piper-boot.service`及`piper.json`，完整`--check-install`退出1；最新构建`build/go2-piper-gripper-20261106/overlay/install/local_setup.bash`同样59模块一致且完整检查退出0。框架HEAD仍为`0208cd72ec7c885a7219040cd2be459d127e374e`且工作树干净；实际Codex CLI为0.167.0，与配置固定0.166.0不符，框架预检失败。采集时enp4s0为DOWN、无192.168.123.x地址，环境入口网络检查失败。没有把这些结果改写为现场故障根因或自动修改版本/默认overlay。白名单环境、接口schema、三包prefix和源码摘要保存在`build/agent-piper-inventory-20261110/`。

资料核对补充：原操作Agent直接用RealSense SDK/CAN，ROS1主要在标定采集层；primitive3导入即创建Piper并使能，不能直接导入现有框架进程。原TCP入口没有完整任务取消/分帧契约；goto_joint超时仍返回目标关节，不能当到位。标定scripts/1入口与write_pose_file参数不匹配，安装规则亦指向通用scripts入口；Pyroki要求Python≥3.10，不能直接复用机载Python3.8环境。方案列出18项双端对齐条目及证据要求，分别覆盖身份/版本、图像/时钟、坐标/TCP/单位、模型/命令、归属/结果/取消恢复、诊断及携带交接。

机载版本、固件和夹爪不动作仍引用11月6日历史记录，本轮未SSH复核。物体、起终区域、实际外参/模型、标定目标、停止与持物语义仍待确定；不把旧样本、0.135m工具偏移、旧角度或旧外参当当前机载标定。仅新增方案并追加本交接，保留所有原未提交工作；不安装、不改产品代码、不构建、不启动ROS/模型/SSH/CAN或真机动作，不提交或外发。

文档验证：方案20个本地链接有效、18项对齐条目完整、代码围栏与表格结构检查通过；新增方案及交接增量的diff空白检查通过。交接原始字节前缀完整保留，其余79份已有改动文件摘要未变。独立审读核对Python/ROS边界、RGBD坐标单位、取消/重启及持物导航交接，无剩余重要设计缺口；此为方案静态审读，不是执行器测试或现场验收。

### 2026-11-11 Piper独立状态/关节/夹爪入口：离线完成，未运行真机

用户缩小当前范围：固定Go2机身，先完成独立状态读取、六关节和夹爪控制，为后续手眼标定准备；本轮不接导航、携带、视觉抓放或IK。用户随后明确不得修改原代码异常处理。本轮在 `/home/isee-pst/unitree_ros2/piper` 新增 `piper.sh`、`standalone/{piper_control.py,piper_common.py,requirements.txt,README.md}` 和 `tests/test_standalone.py`；仅为原 `桌面操作和标定.md` 增加新入口提示，原文档备份于该目录 `.standalone-backup-20261111/`。原 `agent/`、`calibration_toolbox/`、`piper_calibrate/` 及SDK安装文件保持不变；没有包装或替换SDK发送函数，没有修改原异常处理逻辑。

入口提供 `check`（离线依赖/配置检查）、`inspect`（只接收反馈，无查询或控制发送）、`joints`（显式六关节角与速度）、`gripper`（显式开口与力矩限制）及 `diagnostics`。使用独立Python解释器与 `-I`，不导入原视觉/标定Agent，不source ROS overlay。运行目标依赖SDK0.6.2/python-can4.5.0；默认解释器优先历史机载 `/home/unitree/piper-venv/bin/python`，否则系统Python，可用PIPER_PYTHON指定。默认读取既有 `/opt/piper-boot/piper.json` 的USB序列号，不配置CAN或改变开机使能服务。使用说明及按依赖顺序的终端步骤在piper的 `standalone/README.md`。

新入口沿用已有USB身份与健康检查；反馈按帧新鲜度过滤，SDK拒绝异常关节/位姿/夹爪帧后不复用旧缓存冒充新数据；控制目标核对设备现有限位/行程，显式命令仅尝试一次，不自动重放。等待、故障与超时在独立诊断日志显示原因及状态；抛出异常导致部分写入时保持结果不确定。遵照用户限制保留SDK0.6.2底层发送失败可能只记日志的行为，`COMMAND_ATTEMPTED`不证明发送成功，`AT_TARGET`仅表示反馈到位。没有夹持成功或独立停止判据；Ctrl+C/SIGTERM只退出监测，不撤销已发目标、不自动释放/失能/回零。夹爪显式命令包含夹爪使能位，不自动使能六关节。

验证：从实际安装路径执行新增38项离线测试，全部通过；禁止socket后使用实际SDK0.6.2 wheel、历史反馈与合成报文核对编码、单位、缓存失效、原发送失败日志行为及无重发。两个生产Python文件编译、Python3.8 AST、Bash语法与跨目录help检查通过。安装前81份原文件摘要均未变；安装后除有备份的原文档外80份摘要均未变，7份安装文件与审阅版本一致。独立审读无剩余阻塞问题。证据在 `build/piper-standalone-20261111/evidence/`；早期red/review-red/recovery-red日志保留开发过程，最终结果以tests-final、sdk-offline-final和installed-verification为准。

没有运行旧硬件demo或全目录pytest，没有安装依赖、SSH、上传机载端、启动ROS/CAN/运动或采集标定数据。独立Python入口编译已完成；未改ROS包及导航启动入口，不做colcon构建或切换既有overlay。Python3.8/aarch64实机运行未验证；历史夹爪不动作未解决，不能将离线结果称为抓放成功。下一步是在实际连接USB-CAN且依赖齐备的电脑按README先check、再inspect，取得新鲜反馈后再按现场授权验证关节与夹爪；随后单独进行机载相机手眼标定，不复用旧外参。


### 2026-11-11 Ranger RoboRSI 协商与 Go2 CLI 版本兼容

用户先授权通过Codex信箱商定Ranger接入RoboRSI+CodexCLI，随后明确要求Go2侧agent适应CLI持续更新。本轮分支 `go2-cooperative-task-layer`、HEAD `323d5c52737477962a874ae012db117be5a93f34`；保留原dirty、日志和证据，不提交。信箱doctor通过，角色go2→ranger，任务 `ranger-roborsi-codexcli-20261111`。历史未处理消息未当本轮任务执行。

**本机修改与原因。**原 `tools/go2_agent/config.yaml` 精确固定0.166.0，今日实际CLI为0.167.0；框架宿主预检和真正planner预检均会比较精确版本。改为 `minimum_version: codex-cli 0.166.0`，接受不低于基线的稳定版本，拒绝过旧、无法解析和预发布版本；用本地--version及分别的顶层/exec --help检查必要参数。每轮把实际解析的绝对路径、版本和SHA-256传给原RoboRSI，保留第二次严格预检及启动前摘要检查，下一轮重新发现更新，不自动升级/降级/回退CLI。框架仓库仍为干净的 `0208cd72ec7c885a7219040cd2be459d127e374e`，没有修改框架、认证或导航/取消策略。

**构建与实际入口。**新输出 `build/go2-codex-compat-20261111/overlay/install/local_setup.bash` 三包构建成功；CoVLM_env、Cooperative_env、Launcher及公共诊断入口默认路径同步，显式COOP_SETUP_FILE仍优先。完整--check-install退出0，59个Python模块来自本轮安装；三包prefix及6份脚本/launch安装副本核对通过。Agent runtime/config仍走源码入口。完整命令与环境仅见 [runbook新节](go2_runbook.md#2026-11-11-codex-cli-版本兼容)，旧进程不会热更新。

**验证。**适配基线44项通过，最终59项通过（新增15）；入口/环境基线149项通过，加入既有恢复入口6项后的最终155项通过，11条既有第三方弃用警告。新增回归先复现新版被固定版本拒绝，独立审查另发现顶层/exec帮助混用误报并以3项失败用例复现，已分开检查；真实框架第二次预检拒绝同版本内容替换的SHA-256错误也已验证，搜索未启动。实际0.167.0原生二进制预检退出0，摘要 `cdcb45aaa01d0f14e298985e09465aebccd47f64ecd298aea44814af7b62bbda`，仅本次身份，不永久钉死。0.166.0及更高版本矩阵其余项使用假CLI，不能当对应真实版本模型验收。证据 `build/go2-codex-compat-20261111/evidence/` 包含before快照、基线、red/review-red、最终测试、构建、preflight和安装核对；build不受Git保护。未重跑全库/C++测试，未运行真实模型、ROS节点、SSH、桥接、录包或运动。

**信箱已商定范围。**Ranger消息 `9660104700434faf8c464acbfbb89467` 说明其用户目标为双机接入；Go2已逐项核对并回复 `22746896f3ba4314aaad1fdc39b648ab`。Ranger确认消息 `6cc41175395145adbbd8e3659cc5fbc9` 同意：Go2保留唯一coordinator及原start/pause；Ranger只读观察既有task_id，不把READY/deposited/模型finish当全局成功，未验证的结果/停车保持UNVERIFIED。Go2本轮不新增双机外层实现。待定集中在Ranger仓库 `docs/multi_agent/cooperative_roborsi_pending.md`；远端文件/112项回归及CLI0.167.1属于Ranger报告，本机未复制或执行其实现。两端兼容策略不同，不宣称字节一致或联合验收通过。当前任务收到的三条Ranger消息均在回复投递成功后ack。

**已核对的双机限制。**AutomaticLauncher.run会自动ARM并发一次start；分阶段prepare不start。StartTask等待5秒，回执仅print，正常返回才写 `build/go2-field/cooperative-task.*/start.log`，无结构化持久化请求ID或幂等键；超时须保持受理结果UNKNOWN，不重发。pause是软件暂停，不是整轮取消，Trigger成功不证明停车；已DONE保留DONE。本地coordinator/status含task_id/phase/reason等但没有统一持久化全局result，当前Coordinator没有主动赋值FAILED路径；双方AgentState不含motion_terminal，Go2诊断status也未绑定task_id，不能单独作为整轮证书。未来可讨论Go2本地只读证据输出，但本轮未实施；远端动作终态及证据完整性缺口不能由本机推断补齐。双机模型控制、至多一次请求、超时UNKNOWN、pause/cancel语义及整轮验收仍待后续明确需求。

**剩余边界。**CLI参数预检记录 `model_protocol=unverified`，不证明MCP/JSON事件或真实工具隔离；未来破坏性接口变化仍可能需要适配。已纠正runbook中--ignore-rules的说明：跳过execpolicy .rules，不等于跳过AGENTS.md。未修改既有硬件授权或真实验收结论。

交付检查：git diff --check、Python编译与Bash语法检查通过；新增6个本地链接有效，文档代码围栏成对；原交接字节前缀和其余未涉及的dirty文件摘要均保留。独立复核无剩余产品代码阻塞问题；提示音乐已播放5秒。最终确认回复 `0b2c537088164be89e6292464639c958` 已投递。

### 2026-11-11 Piper入口解释器路径报错修正

用户报告piper.sh第12行找不到 `/home/unitree/piper-venv/bin/python`。本机该路径不存在，显式设置PIPER_PYTHON为此值可复现相同报错；不设置该变量时原入口已能回落系统Python并显示help。用户终端环境本身未读取，不将复现等同其环境取证。修正piper.sh：显式无效解释器在exec前报告PIPER_PYTHON及unset修复方法，不静默切换用户指定环境。README本机首步改为unset而非机载路径export；原控制Python、SDK及异常处理未变。原文件备份位于piper/.standalone-backup-20261111/python-entry/。

安装后原38项加新增2项回归共40项通过，Bash语法通过；变更哈希、补丁在build/piper-python-entry-fix-20261111/。仅修改独立shell入口、测试及说明，不涉及ROS包，不构建/切换导航overlay。清除错误解释器变量仅解决路径问题；本机Piper依赖、默认机载配置及CAN仍不齐全，check不等于硬件可用。没有安装依赖、SSH、启动CAN或执行动作。

### 2026-11-11 Piper本机独立依赖补齐：离线CHECK_OK

用户提供check-lz72l433.jsonl，实读为CHECK_FAILED，原因是当前Python缺piper-sdk元数据。在piper项目新建 `.venv`（本机Python3.10/x86_64），从既有SDK兼容性证据目录的本地wheel离线安装piper-sdk0.6.2、python-can4.5.0及依赖，不联网下载、不修改系统/导航/机载环境。pip及检查使用-I隔离ROS PYTHONPATH；临时探针最初未加-I的pip check看到了继承ROS包而报缺项，加-I后无依赖冲突，未为此修改ROS依赖。机载aarch64不能直接复制本机虚拟环境。

piper.sh默认解释器顺序更新为项目.venv、历史机载环境、系统Python；显式PIPER_PYTHON仍优先。README同步本机步骤；原控制Python和SDK异常处理未改。默认配置仍为机载 `/opt/piper-boot/piper.json`，本机不存在；本机check需显式指定已有 `/home/isee-pst/unitree_ros2/VLM_Nav/config/piper_boot/piper.json`，没有伪造USB身份或修改系统配置。

从安装路径验证40项基线及1项项目环境选择回归，共41项通过；Bash语法、Python编译、实际SDK导入及隔离pip check通过。实际 `env -u PIPER_PYTHON bash /home/isee-pst/unitree_ros2/piper/piper.sh --config /home/isee-pst/unitree_ros2/VLM_Nav/config/piper_boot/piper.json check` 退出0，check-ql8goa5o.jsonl为CHECK_OK，解释器指向项目.venv，SDK0.6.2/CAN4.5.0；日志副本、变更摘要与补丁在build/piper-local-venv-20261111/。未改ROS包、不构建或切换overlay；未SSH、上传机载、初始化CAN或运行机械臂。USB序列号只做配置读取，真实设备连通、运动和手眼标定仍未验收。

### 2026-11-11 Piper默认配置统一与非运动自动检查完成

用户报告WAIT_CONFIG仍指向不存在的/opt配置，并明确授权自动完成非运动的代码环境配置与检查。原因是此前只在check调用显式传配置，随后不带参数启动inspect仍选旧默认路径。本轮把原Piper配置原样复制到piper/standalone/piper.json；Python统一解析规则为显式--config优先，否则现有/opt/piper-boot/piper.json优先，无该文件则使用程序同目录piper.json，所有子命令一致。默认运行不再依赖VLM_Nav目录或额外参数；CHECK_OK和STARTING记录实际config/python。保留所有except处理节点、run/session逻辑及原SDK行为；旧agent/标定代码未改，不创建/opt文件，避免改变旧等待进程的条件。保留用户刚加入README的本机标定目录说明。

已自动验证：实际项目.venv的pip check无冲突、依赖可导入、Python编译及3.8 AST/Bash语法通过；原41项加5项配置回归共46项通过。从/tmp跨目录、不设置PIPER_PYTHON且不带--config运行实际check，退出0，日志check-4z02d498.jsonl为CHECK_OK，加载piper/standalone/piper.json。实际被动inspect运行3秒，状态STARTING→WAIT_IO（USB身份匹配0）→EXITED，未再出现WAIT_CONFIG，command_attempted=false；仅向本次测试进程发SIGINT退出，未改变机械臂状态。diagnostics短时跟踪日志通过并已退出。

只读本机sysfs确认当前无CAN接口、无1d50:606f USB适配器。软件准备完成不等于设备连通；未修改网卡、未SSH或部署机载端，未查询/使能/失能/校零/移动机械臂，未采集标定样本。之后需要真实USB-CAN连接或在实际接臂电脑运行；不能凭软件修复宣称夹爪故障解决。旧运行进程不会热加载新默认路径，旧WAIT_CONFIG日志不是本轮结果。新日志副本、运行检查摘要、完整前后哈希、补丁及验证脚本位于build/piper-config-complete-20261111/；修改前文件备份于piper/.standalone-backup-20261111/config-default/。仅独立Python和配置变化，已编译检查，不构建或切换导航ROS overlay。

### 2026-11-11 Piper部署Go2上位机并通过SSH启动

用户明确授权参考相机SSH入口，部署独立机械臂代码到实际连接机械臂的Go2上位机并从其环境启动；此前非运动限制继续有效。已读取go2_camera_ssh.sh、Piper开机安装器/服务/实现及runbook。相机目标unitree@192.168.123.18与既有SSH别名unitree-go一致；本机enp4s0为UP、192.168.123.222/24，至目标走该网卡。SSH密钥连接成功。首次OpenSSH尝试更新known_hosts因沙箱只读发出警告但连接成功；后续用UpdateHostKeys=no避免写入，不禁用主机身份校验。

机载实测ubuntu/aarch64、Python3.8.10、piper-sdk0.6.2、python-can4.5.0，SDK导入及pip check通过。沿用 `/home/unitree/piper-venv/bin/python` 与 `/opt/piper-boot/piper.json`，未复制本机虚拟环境、未安装/升级机载依赖。新建 `/home/unitree/piper-standalone-20261111/`，只上传8份明确的入口/控制/helper/配置/说明/测试文件及SHA256SUMS；Python3.8实际编译、Bash语法及远端全文件摘要核对通过。未覆盖/opt/Piper开机部署或相机/导航代码。部署中发现本机README被用户同时改为“类似相机SSH，部署到go上位机”，已保留该修改并同步远端，不覆盖用户工作。

本机piper目录新增 `piper_go2.sh`，沿相机方式使用SSH -tt连接unitree-go，调用已部署目录piper.sh；参数经printf %q转义，清除远端遗留PIPER_PYTHON后由已部署入口选实际机载解释器。check/inspect/diagnostics及后续显式关节/夹爪参数共用此入口，不自动部署或重放动作。49项本机测试通过（原46+3项SSH参数/转义回归）；机载未装pytest，未为此加开发依赖。原控制Python及异常处理未改，独立入口无需ROS构建或切换导航overlay。

实际经本机新SSH入口执行check退出0，远端check-vvndogha.jsonl为CHECK_OK，明确机载解释器及/opt配置。随后经同入口运行inspect约12.7秒，并从第二个SSH入口diagnostics查看日志；状态STARTING→WAIT_IO，未出现WAIT_CONFIG，command_attempted=false。Ctrl+C经SSH PTY正确到达远端，inspect返回130并记录EXITED；诊断SSH也结束，复核无遗留独立监测进程。关节/夹爪目标、使能、清错、回零、失能均未发送，没有启动相机或导航。

当前硬件阻塞已在上位机重新确认：没有CAN接口，USB枚举未见VID/PID 1d50:606f。已有piper-boot.service为enabled、active(running)，journal反复WAIT_CAN/USB identity match count=0，尚未完成使能，不把Result=success或active当READY。原服务会在指定USB出现后配置1Mbps并EnableArm(7)，收到六关节新使能反馈后退出；本轮没有重启、禁用或修改服务，前后/opt代码和配置SHA一致。设备接线、供电或枚举原因待现场核对，历史夹爪不动作仍未解决。软件部署/远程启动通过不等于真实状态或动作验收。

证据在build/piper-go2-deploy-20261111/evidence/：远端环境/服务/进程/摘要、check/inspect日志副本、部署SHA256SUMS、本机前后摘要及49项测试结果；使用步骤已同步piper/standalone/README.md。后续控制电脑在piper目录以piper_go2.sh inspect/diagnostics开两个Bash终端并保持，观察真实OBSERVING与新鲜数据；WAIT_IO时先核对物理连接，不重启程序发送目标来试探。piper.sh仍代表本机执行，不能混用两端日志。


### 2026-11-11 信箱续接：双模型抓放任务契约商定（未实现）

本机用户要求“通过信箱继续对接”。沿用 `ranger-roborsi-codexcli-20261111`，doctor通过；本机分支/HEAD仍为 `go2-cooperative-task-layer` / `323d5c52737477962a874ae012db117be5a93f34`，dirty保留。本轮只读核对与文档商定，不改产品代码，不启动模型/ROS/SSH/桥接/机械臂或运动；上一轮CLI适配测试/构建结果不作为本轮新运行证据。上轮两条收口消息已在续接消息 `2a565ad014464a9890feae0c42c30aa6` 确认收到后ack，历史无关任务未执行。

**需求来源与方向。**Ranger消息 `71095515823543368b4259b7e29bf678` 转述其用户确认六项新约束：两端各一个RoboRSI+Codex CLI管理本机技能，Go2唯一确定性coordinator和唯一整轮start；任一执行模型异常须由确定性程序请求双机暂停，导航和机械臂分别确认停止；同task_id两端真实放置并满足执行结束条件才整轮成功；模型会话重建保留原任务/步骤；工具由确定性分发检查本机/任务/阶段权限。模型不得额外重试失败技能、重启整轮或调整参数；原导航内部恢复与模型重派分开。本轮审读这些目标，不代表Go2已获控制实现或现场授权。此前Ranger只读观察分工是当前实现边界，新目标是尚待实施的双执行模型架构，两者不能混称。

**本机已确认差距。**[runtime.run_session](../../tools/go2_agent/runtime.py)在单机模型失败/提前finish后继续守护搜索，尚不执行新双机暂停策略。[AgentMachine.complete_local_execution及tick](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py)两条路径都会在导航到达后设置ready/deposited；[RulePlanner](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/planner.py)没有真实抓放步骤，不能把旧字段直接当物理证据。旧显式pause不是同任务恢复接口，现有_resume绑定原导航阶段及recoverable快照；不能pause后重发start冒充原任务恢复。单机SearchSession会话task_id也不是协同task_id。Go2独立Piper当前COMMAND_ATTEMPTED不是受理ACK，AT_TARGET仅位置反馈，grasp_confirmed=false/stop_confirmed=null；退出监测不停止已发动作。相关事实通过回复 `f9089d95b640434f96935a828e82a4a6` 的A-G项反馈，不修改SDK原异常处理。

**已审读的商定草案。**Ranger通过 `1aedcb43fee74ccea734affc28569b14` 和 `aeff39ab2e084223825efba8cf9968b9` 提供集中待定文档及两张表。Go2同意业务交接依次为所属任务/步骤绑定→原探索接近A/B→真实夹取及允许携带→携带探索接近C→真实放置及本机执行结束→同任务双端证据收敛；不靠READY后抢发pause实现交接，不重写探索/靠近算法。异常表区分模型异常、暂停受理、各执行器停止确认、断网或证据缺失、会话重建、人工接管、技能失败/受理未知；UNKNOWN不能判停止，人工接管不自动续跑，失败步骤不能由模型自行重派。表为业务/证据草案，尚未定义新ROS字段、控制API或硬件阈值。

**补充边界与待定。**Go2回复 `df0de91b50ae4e5a982f12289fcf0dd4` 与 `10dc115bcfbd463fb15fa52a4adbf4e7` 要求补入：AgentState.sequence是命令序列而非心跳计数；接收新鲜度在各端本地计时，不跨机比较monotonic或把旧文件重读时间当新证据。暂停中旧动作许可失效，查询/停止由确定性层管理，并对已受理操作执行真实停止/保持流程；kill进程或松爪不作为通用停止。模型重建后旧会话动作权限失效，迟到调用不能仅凭相同task_id执行；受理未知不自动补发。剩余集中为两端执行器受理查询/物理完成/停止保持接口、携带许可成立和失效条件、模型异常检测与人工恢复授权、持久化及受理未知消歧。未凭建议字段补造硬件能力。

权威待定文档继续由Ranger在其仓库 `docs/multi_agent/cooperative_roborsi_pending.md` 单处维护；本机只记录交接摘要，不复制并行演化方案。Ranger状态/源码/验收报告与本机事实分别标注。当前收到的新需求、文档和两表三条消息均在回复投递成功后ack；最后两条补充边界的归档确认尚在等待。核对文件身份记录在 `build/go2-roborsi-contract-20261111/evidence/readonly-baseline.json`；本轮无需构建或重跑代码测试。

本轮结束状态：最后补充消息10dc115bcfbd463fb15fa52a4adbf4e7投递成功后，连续三次各45秒等待均无新消息；再不带task收信核对仅有历史无关任务，未发现任务号不一致，按约定停止等待。最后两条边界是否已写入Ranger文档仍待对端确认，不能以delivered当已归档。git diff --check、新增3个本地链接及文档格式检查通过；所核对10份代码/接口/配置摘要未变，原交接字节前缀保留。未构建、运行代码测试或播放代码完成提示音（本轮仅文档）。

### 2026-11-12 用户恢复接线后只读复核：有线已通，上位机地址仍不可达

用户确认Go2上位机已开机、连接控制电脑且USB-CAN已插上。本机实测enp4s0为UP/LOWER_UP、192.168.123.222/24、NetworkManager connected；至既有上位机192.168.123.18的路由正确走enp4s0。相机脚本与unitree-go别名仍指向.18。两次SSH均No route to host，绑定enp4s0的3次ping均由本机返回Destination Host Unreachable，.18邻居项FAILED。网线载波恢复与目标上位机可达是两项不同事实；当前不能重新检查远端USB或沿用昨天未插入状态当本轮结论。邻居表另见.20，但无该地址属于上位机的证据，未擅自切换目标。

已请求用户在上位机本地终端运行只读 `ip -br -4 addr`，确认本次开机实际地址；不要求重复检查已确认的接线。证据build/piper-go2-link-20261112/network.json。未修改网络/IP/SSH配置、机械臂代码或异常处理，未重启服务、启动模型/ROS或发送动作。仅追加交接，无代码变更，不构建或重跑代码测试。

### 2026-11-12 SSH恢复后的USB枚举排查（待现场换口反馈）

用户报告SSH已恢复但仍WAIT_IO；通过enp4s0实际连接unitree-go复核，最新check-berej63s为CHECK_OK，inspect-i7xcq23f仍USB identity match count=0，之后已有USER_EXIT。上位机USB列表仅含GenesysLogic USB2/USB3 Hub、RealSense435（2-2.2）及ASIX网卡（2-2.4）和根Hub，没有1d50:606f模块，也没有CAN接口。gs_usb.ko文件存在，modinfo别名明确支持1D50:606F，USB根端口authorized_default=1。当前不是已发现设备后的序列号不匹配；设备尚未枚举，未据此重装SDK、改身份或加载/重启驱动。内核日志中另有tegra-xudc启动告警，但尚无证据将其定为本模块根因。

用户确认USB-CAN通过扩展坞连接、无可用上位机直连USB口。已请求仅把USB-CAN换到扩展坞另一空闲口（无空闲口则原口重插），保持扩展坞主线、相机和网卡不变，并提供指示灯状态。只读udevadm监测45秒结束，未捕获USB/net事件；截至记录尚未收到实际换口/重插完成回复，不能把该观测写成换口无效。证据build/piper-usb-enumeration-20261112/usb-snapshot.json。原开机服务仍active(running)/WAIT_CAN，设备恢复后会继续原使能流程，已告知用户；本轮未改服务、网络、SDK或异常逻辑，未发送控制命令。待现场动作与灯态后继续核对枚举，暂不能确认线缆、端口或模块故障。

### 2026-11-12 换扩展坞插口后USB恢复，待sudo恢复新建CAN接口

用户确认已换口后实测USB 1-2.3出现candleLight USB to CAN adapter，VID/PID为1d50:606f，序列号0046002F5246570520323934与原配置一致；gs_usb创建can0。内核记录该模块多次断开/重枚举，其中一次probe报err=-71，不能仅凭一次换口将故障确定为原插口损坏。原开机服务在boot单调时刻1572.956889发送EnableArm，1573.960362记录READY/六关节全使能并退出，当前active(exited)；后续1718秒附近USB断开、1744秒再次枚举，当前ifindex17的can0为DOWN/STOPPED、TX/RX均0。历史READY不等于此刻接口可读。

未重启使能服务。按用户此前非运动环境配置授权尝试 `sudo -n ip link set can0 type can bitrate 1000000`，实际拒绝原因为sudo要求密码，未发生配置修改。已请求用户在自己的SSH终端执行 `sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`，密码仅在其终端输入；待执行结果后再自动做被动反馈检查。此步骤只配置CAN，不额外发送使能或目标。当前尚未采集到恢复后的关节/夹爪反馈，未执行任何动作验证。状态及带单调时刻的服务日志保存在build/piper-usb-enumeration-20261112/after-port-change.txt；无代码变更，不构建/测试或播放代码完成提示音。

### 2026-11-12 用户启用CAN后：机载Piper真实只读反馈通过

用户确认已完成sudo CAN配置后，SSH实测can0为UP/LOWER_UP、ERROR-ACTIVE、1000000bit/s，USB序列号仍正确。开机服务为active(exited)。实际从已部署 `/home/unitree/piper-standalone-20261111/piper.sh inspect` 被动读取6秒，状态STARTING→WAIT_FEEDBACK→连续11条OBSERVING→EXITED；missing_feedback=[]，关节/SDK末端/夹爪数据均持续更新。使用SIGINT仅结束本次检查，退出130。TX计数0→0，RX113199→131611，收发错误均0；command_attempted=false，未发送查询、使能、目标或其他控制帧。

新鲜反馈六关节使能均true，arm_status=0、err_code=0、ctrl_mode=0；夹爪enabled=true、反馈开口0.0mm。六关节反馈度数[2.472,8.017,0.935,7.244,-3.78,-12.528]，SDK末端毫米[68.369,2.195,204.918]，仅记录观测，不作为待执行目标、法兰/TCP或已标定位姿。物理停止及夹持成功判据仍未具备；本轮未动作，不能宣称历史夹爪无响应已修复。

远端日志 `/home/unitree/.local/state/piper-standalone/inspect-ncu3q4yz.jsonl`；含完整观测、时刻和前后计数的本机证据 `build/piper-usb-enumeration-20261112/readonly-after-can-up.json`。本次确认此前WAIT_IO已在真实机载链路消除，结论限于本次6秒窗口，不代表USB长期稳定性或抓放/手眼验收。后续继续用控制电脑piper_go2.sh inspect与diagnostics；若再次拔插生成新的DOWN接口，已退出的开机一次性服务不会自动重做初始化，需按既有非运动CAN恢复步骤处理，不能把重启使能服务当无副作用检查。无代码修改，不构建或重复离线测试。

### 2026-11-12 手眼标定脚本制作：用户明确暂停，待下次恢复

用户要求按手眼标定方法给出步骤并制作脚本，随后确认RealSense固定Go2机身、ArUco ID25刚性固定末端、机身不动；黑色标记正方形外边长14.2cm（142mm，不含白边）。用户描述“边长7像素”，不足以唯一确定字典；必须通过完整图像识别，不直接假定DICT_7X7或旧ArUco字典。本轮两次有限RGB取图中，第二张已见ID25标记，但黑码下边和右边被裁掉，扫描DICT_ARUCO_ORIGINAL及4/5/6/7格常见字典均未检出完整码，不能据此判码无效。用户要求传回画面，已保存 `/home/isee-pst/Pictures/piper_handeye_board_20261112_162400.png` 并提供链接。随后用户明确“摆放角度不佳，记住当前进度，现暂停，待下次恢复”。本轮已停止实现/部署/相机/硬件验证，仅保存以下断点，不自行继续。

**断点位置。**临时工作 `/tmp/piper-handeye-20261112` 已复制到持久目录 `build/piper-handeye-20261112/checkpoint/`（stage代码、测试、scope、原文件hash、失败回归日志、两张图、status.json）。新脚本全部是草稿，尚未写入实际piper源码目录或部署上位机；没有改原控制、SDK或标定代码的异常处理逻辑。stage包含handeye/core.py、capture.py、board.json（dictionary=null、marker_id=25、marker_size_mm=142、camera_serial=151222077264），piper_handeye.sh、piper_handeye_go2.sh、tests/test_handeye.py，并带原standalone只读helper副本。旧piper工具包代码基线在before.json，恢复时须重新核对并保留用户改动。

**环境新事实。**本次上位机 `/usr/bin/python3 -E` 可导入用户目录cv2 4.12.0、pyrealsense2 2.55.1及系统NumPy1.17.4，缺can/piper_sdk；已有piper-venv中SDK0.6.2/can4.5就绪但缺cv2/NumPy/pyrealsense2。`-I`系统Python不加载用户目录的RealSense。拟创建独立标定venv引用这些已验证库，不改已有环境，尚未创建。RealSense全局 `/home/unitree/.realsense-config.json` 是有效JSON，但context字段为空字符串，默认rs.context()报“expecting an object”；官方v2.55.1源码允许context({"inherit":false})仅对本进程忽略全局配置，已实际验证成功，未改原文件。当前直接SDK枚举到D435序列号151222077264（与之前USB记录153223022502不同，以本次采集身份为准，不复用旧外参）。实际640x480/15Hz彩色内参fx605.864、fy605.519、ppx326.521、ppy243.543，模型Inverse Brown Conrady但5项畸变均0；完整精度应在每个实际样本中读取，不从本摘要抄参数。两次有限取图结束后pipeline均stop，未留下采集进程、未移动机械臂。

**实现方向与已验证范围。**固定相机方程 `B_T_E E_T_M = B_T_C C_T_M`；输入SDK mm/deg转换米、Rz(RZ)Ry(RY)Rx(RX)，将B_T_E求逆后传OpenCV calibrateHandEye PARK，输出彩色光学系到Piper基座及末端到标记变换，不能称已确认法兰/TCP。训练集拟合，独立保留验证集仅报告残差，physical_validation=NOT_PERFORMED。原工具箱ROS1依赖/自动回放流程不直接复用。collector设计只被动收CAN并以图像前后静止反馈窗口配对，保留原始时间戳/域，明确不是硬件同步；单次capture/preview，不包含任何运动API。捕获质量默认窗口>=0.8秒、平移变化<=1mm、旋转/关节变化<=0.5度，仅为样本质量检查，不控制机器人。对非零且不支持的RealSense畸变模型拒绝冒充OpenCV Brown；板PnP检查重投影/平面二义性。样本不可覆盖，sample.json最后作为完成标记，失败清理本次半成品允许重试。新capture实际运行、相机/CAN时间配对、完整文件流程仍未验证。

**测试与审读。**本机 `/home/isee-pst/venv/co-nav-real/bin/python`（cv4.11/NumPy1.26）下最新21项离线测试通过：合成已知外参恢复、单位/方向、纯平移/单轴退化拒绝、SE3/NaN、ID25合成标记、畸变、静止/新鲜度、覆盖保护、训练验证重叠、采样板尺寸不一致、隔离-I入口导入及中途写入失败恢复。新测试先失败再修复；原17项、两轮各2项新增结果分别保留red/review-red/publish-red。独立审读发现-I下capture导入core失败、训练验证重叠、半成品目录和板配置一致性四问题，已补测试修正，但尚未做最终二次审读。审读未发现当前变换方向/被动CAN发送问题，不代表实机采集通过。stage原语法曾通过Python3.8 AST，最新改动恢复后需重新检查实际安装代码。

**恢复顺序。**先确认用户恢复任务；让标记四角及白边完整入镜，有限取图识别字典，写入准确配置。随后复核草稿、完成独立解释器和CLI端到端验证/说明文档，再部署并做非运动采样smoke；smoke不得冒充正式样本。正式建议15组训练+5组独立验证，姿态需不同方向与位置；只能由操作者按已确认方式变换机械臂姿态，不自动回放旧26点、不编造轨迹、不硬掰使能关节。本轮还未写完用户操作步骤、未完成部署、未采集有效手眼样本、未求解任何真实外参或发布TF。来源：OpenCV官方calibrateHandEye文档（https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html）、SDK0.6.2及旧采集器sxyz约定、librealsense v2.55.1 src/context.cpp。用户暂停优先，不因这些待办自动续跑。


### 2026-11-12 恢复手眼标定：脚本两端安装，完整板识别与静止采样通过

用户明确恢复任务；沿用此前非运动配置、SSH部署授权，不执行机械臂动作、不修改原控制/SDK异常逻辑。首次恢复时can0消失，USB枚举/属性读取阻塞，用户同时报告内核`unregister_netdevice: waiting for eth0 to become free. Usage count = 40`。SSH曾超时。未据此确定扩展坞/线缆/驱动根因，未重启网络、使能服务或设备。用户重新连接并再次要求继续后，实测SSH恢复、USB 1d50:606f与RealSense重新可见，can0 UP/ERROR-ACTIVE/1Mbps。当前窗口恢复不代表长期稳定性通过。

**本轮安装。**本机 `/home/isee-pst/unitree_ros2/piper/handeye/{core.py,capture.py,board.json,README.md}`、`piper_handeye.sh`、`piper_handeye_go2.sh`、两份新增测试已安装。全部原Piper文件与暂停前before.json哈希相同，原控制/SDK/旧标定异常逻辑未改。机载新目录 `/home/unitree/piper-handeye-20261112`，只部署11份明确文件，包括原样standalone helpers/config和新代码/测试/说明，SHA256SUMS核对；没有覆盖旧piper-standalone、/opt启动服务或导航。部署源及证据位于 `build/piper-handeye-20261112/resume/`，暂停checkpoint仍保留。

两端各建`.handeye-venv`，解释器以-I运行；没有安装/升级原环境的库，通过.pth复用本机co-nav-real与piper/.venv包，以及机载piper-venv、用户Python3.8和系统dist-packages。机载实际Python3.8.10、SDK0.6.2、CAN4.5、cv4.12、NumPy1.17.4、RealSense2.55.1导入通过；本机cv4.11、NumPy1.26.4、CAN/SDK导入通过。环境引用是依赖关系，后续升级被引用库需要重新验证；不是拷贝aarch64/x86二进制环境。仍用进程内inherit=false绕开Viewer配置类型错误，未改全局文件。

**标定板与相机。**重新连接后的有限RGB抓图，实际D435 serial151222077264；常见字典扫描只命中DICT_ARUCO_ORIGINAL/ID25，黑方块四角完整入镜。已将字典与用户确认的142mm写入两端board.json。新图 `/home/isee-pst/Pictures/piper_handeye_board_20261112_resumed.png`。画面中的板挂在支架上，尚未确认刚性装到末端，因此仅作为识别与软件smoke。正式采集必须先固定板、机身和相机，并由操作者按已确认方式调整姿态。已异步询问准备状态，尚未收到回复时不启动动作或正式多姿态采集。

**验证。**新增标定23项、原独立控制49项分别在co-nav-real解释器中通过。曾将两组同进程运行，旧测试因sys.modules已有cv2而失败；这是旧“纯控制入口不加载视觉库”测试与新视觉测试的同进程冲突，未改旧测试，README明确分进程执行。生产标定环境没有pytest，测试继续使用既有测试解释器；没有为测试添加依赖。Python3.8 AST/Bash语法及机载Python3.8实际compileall通过，机载合成20姿态恢复已知变换在1e-7容差内（非物理精度）。审读补充发现CAN积压旧帧可能被接收时间冒充新鲜，已仅在新Reader检查每帧/每必需ID内核时间年龄≤0.1s、保存can_timestamp_range_unix；新增50ms通过/700ms拒绝回归先红后绿，最终只读复核未见剩余阻塞。原helper没有修改。

实际本机`piper_handeye.sh identify`成功，仅输出本机日志路径，日志候选为DICT_ARUCO_ORIGINAL。实际SSH入口`piper_handeye_go2.sh check`退出0，远端`run-gWzQ9SGH.jsonl`为CHECK_OK，明确使用机载新目录.handeye-venv；SSH末尾Connection closed是单次命令正常退出，须结合退出码判断。实际新capture完成一次静止smoke，退出0、SAMPLE_SAVED、command_sent=false；远端日志`/home/unitree/.local/state/piper-handeye/run-JnR9uKy3.jsonl`，样本`/home/unitree/piper-handeye-20261112/smoke/stationary01`，证据smoke-evidence.json已回传。CAN TX1→1，RX1847776→1861425，收发错误0。图像640×480/15Hz彩色，零畸变，板PnP重投影RMS0.662503px，反馈静止窗口1.055795s，窗口CAN最大报文年龄约24.83ms。此为静止时间窗口配对，不是硬件同步；该样本role=train仅用于程序流程测试，位于独立smoke目录，板未刚性装末端，不得并入正式data。

**后续步骤。**详细顺序、终端/目录、成功判据和失败分支见 `piper/handeye/README.md`。正式建议15组训练+5组预先保留验证，至少两个独立旋转方向；不自动运行旧26点，不猜测动作目标，不硬掰使能关节。输出T_base_color_optical（米）及T_end_marker，保留physical_validation=NOT_PERFORMED；不发布TF，不混同SDK末端与真实法兰/TCP，不接入导航。尚无正式多姿态样本、真实外参或实物精度验收，也没有验证历史夹爪不动作问题。所有变化为独立Python/脚本/文档，未改变ROS包，未构建或切换导航overlay。未提交Git。


### 2026-11-12 手动示教保持已现场确认，逐张预览与人工收录完成

用户选择手动摆姿态并要求每姿态返回图片，明确“能操作按钮，标定板尚未固定”。随后询问示教后使能固定再采样的循环；已解释使能不等于保持当前位置，厂家手册对示教切CAN有零位/待机约束，没有执行模式/使能命令。用户明确“现在测试”，实际按实体示教按钮摆动，再按一次，反馈“机械臂固定在终止状态”。本轮35秒被动观测对应ctrl_mode=2、arm_status=0、teach_status由1变2、err_code=0；末5秒六关节和SDK XYZ跨度均0。CAN TX1→1，RX6617034→6724528。证据`build/piper-handeye-manual-20261112/manual-button-observe-1794489601.jsonl`。因此本机本次可以采用单击示教→手摆→单击结束保持→读图/位姿循环，无需额外EnableArm或切CAN；不推广为所有负载/姿态的保持保证。用户后续再次要求人工确认取舍，已落实下述approve，建议20个有效姿态（15train+5独立validation），废片不计。

实现与入口：本机piper新增`piper_handeye_review.sh`和`handeye/review.py`，现有piper_handeye.sh增加review-local分派；新标定capture.py增加review模式及teach_status记录。旧standalone/helper/SDK、使能服务与原异常处理均不变。只在采样判据中接受正常或明确示教记录状态；不把离线回放/故障/超速当有效样本。审读发现并修复ctrl_mode7误接受及短暂坏状态被过滤的问题：每个接收到的0x2A1坏状态立即留存，即使位姿不完整；采样窗口含坏事件就无效。review仍保存原图和原因，缺CAN/不静止/板没识别也可看图；相机缺失时无法产生新照片，继续等待原因。

控制电脑piper目录、无需ROS，`bash piper_handeye_review.sh snapshot --sample trial01`用于未固定的试拍；正式固定后`bash piper_handeye_review.sh snapshot --sample pose01 --board-attached`。每次取新编号，图片和元数据自动回传本机`~/Pictures/piper-handeye/manual01/<sample>/`。传输失败可恢复远端已完成的同一帧，日志明确RESUMING_EXISTING_SNAPSHOT和原拍摄时间；不覆盖旧图。人工查看四角/白边/清晰度，且ready_for_approval=true后，`bash piper_handeye_review.sh approve --sample pose01 --role train`（验证用validation）才收录本机`piper/handeye-data/manual01/<sample>/`，使用同一帧和同期位姿，不重拍；保留来源图像/metadata哈希及人工确认时间。不满意直接不approve，以新编号重拍；试拍/无效图不能计数，拍摄时未固定不能事后改成已固定。新批次可两条命令均在子命令前加--session manual02。诊断仍用本机piper_handeye.sh diagnostics，终端只打印日志路径。详见piper/handeye/README.md。

安装前发现用户同步把旧预览说明view01改成view02，已合并保留，没有覆盖；首次预检因此安全退出且未修改文件。第二次按reconciled-before核对安装成功。原始与合并前hash、备份、stage和检查位于`build/piper-handeye-manual-20261112/`；远端备份`/home/unitree/piper-handeye-20261112/.manual-review-backup-20261112/`。两端14份部署文件SHA256一致；本机Python3.8 AST/Bash语法、远端实际Python3.8 compileall通过。新标定47项通过（此前23+本轮24）、旧独立控制49项分进程通过；独立最终审读无剩余阻塞。只改独立Python/脚本，不构建或切换ROS overlay，不提交Git。

真实入口取图`trial-hold01`成功，图位于`/home/isee-pst/Pictures/piper-handeye/manual01/trial-hold01/color.png`；本机日志run-zhIFELbw.jsonl、远端run-2GfG6tSQ.jsonl。SDK静止窗口通过，ctrl2/arm0/teach2/err0；TX仍1→1。图中板靠近镜头，上边和左边裁切，未检出ID25；用户尚未固定板，review_issues两项且ready_for_approval=false。已向用户返回原图，说明需远离相机让四黑角和白边入镜。实际调用approve对该无效预览退出2/WAIT_REVIEW，没有创建正式样本目录，证明它不会计入标定。正式样本数仍0，尚未求解真实外参、验证精度或接入导航；后续待固定板与人工逐姿态取舍。五秒代码完成提示音按项目要求播放。


### 2026-11-12 续采两张后的OK/NO循环脚本完成

用户要求将剩余采样循环脚本化，每张回图后OK收录、NO跳过，并说明已采两次。实际核对本机`~/Pictures/piper-handeye/manual01/pose01`及pose02均为role=review、board_attached=true、review_issues=[]，尚无human_approved或正式data；不是已收录2组。新增循环优先人工判断这两张，不自动算通过；若均OK则剩13train+5validation。未固定且裁切的trial-hold01不作为pose系列待判断样本。

新增本机 `piper/handeye/loop.py`、`piper_handeye_loop.sh`、`tests/test_loop.py`，已有piper_handeye.sh只增加loop-local分派，README加入循环步骤。旧review.snapshot/approve、capture、SDK/standalone及其异常处理全部未改。循环是控制电脑交互程序，复用现有SSH远端采集；机载代码本轮无需修改或部署。本机操作：交互式Bash进入`~/unitree_ros2/piper`，保持运行`bash piper_handeye_loop.sh --session manual01 --board-attached`，无需ROS。板及相机/机身安装须在批次内不变。

已有待判断图按pose数字顺序显示，带编号/路径/拍摄时间；图形环境调用eog --single-window（无则xdg-open），无图形环境提示按路径自行查看。提示OK/NO/Q经/dev/tty直接交互，详细JSON及子进程输出仍进原诊断日志，满足用户最新人工输入需求。OK调用原approve对同图同位姿复核并收录，NO保存decision.json和源图/metadata哈希、保留原图但不计数；判断文件原子链接发布且不覆盖。不同字符不做决定。IO失败等待修复后回车，取图失败重试同编号/远端已完成快照；不会假装新图。质量失败的OK不收录，说明原因并继续等待NO/Q。新姿态每次都先等待用户摆好、单击结束保持后按回车，不能自动连拍或移动机械臂。

数量每轮从正式data重读，前15有效样本train，随后5validation；达到15+5显示COLLECTION_COMPLETE并退出，不自动求解。Q/Ctrl+C/EOF暂停不产生默认接受，同一命令续跑。批准后决策落盘失败仍以已提交正式样本为准，不重复收录；同session用本地flock阻止两个循环重叠。已拒绝图不重复询问；源图被修改或批准数据缺失时报告原因而不猜测修复。应选择不同位置/朝向，独立验证样本不复制训练姿态。

测试：本轮9项循环回归通过（已有两张优先、NO不计数和递增编号、续跑跳过已判断、补足15+5、非法输入与无效OK、EOF、网络失败同编号重试、决策覆盖/图变化、批准后写盘故障恢复）；连同既有47项标定共56通过，旧控制49项单独进程通过。Python3.8语法及Bash检查通过。独立审查无阻塞。安装前hash核对并备份，验证其他原代码文件、已有两张图片及metadata全部未变。实际安装入口通过PTY启动，显示0/15、0/5并展示pose01的OK/NO/Q提示；仅输入Q，退出0，日志run-3bQeNt5U.jsonl为PAUSED、command_sent=false。未替用户输入OK/NO，未新拍或连接机器人、未改变正式样本数；查看器发起打开但未人工确认窗口显示。

证据/备份/stage在`build/piper-handeye-loop-20261112/`，计划`docs/superpowers/plans/2026-11-12-piper-handeye-loop.md`。当前正式样本仍0，pose01/02待用户判断。只改本机独立Python与入口/说明，不构建或切换ROS overlay，不提交Git。按要求播放5秒代码完成提示音；后续正式循环由用户在交互终端运行，不自动启动无人值守采集。


### 2026-11-12 USB-CAN再次掉线；用户要求暂停并确认有效数量

用户报告`robot: USB identity match count=0 expected=1`。本轮按systematic-debugging只读取证：上位机SSH可达，RealSense/ASIX网卡/扩展坞仍枚举；1d50:606f适配器消失、无CAN接口。/opt配置仍为0046002F5246570520323934，与历史枚举一致，不是序列号被改。journalctl内核记录21:45:04 USB 1-2.1 disconnect device4，随后gs_usb can0 Couldn't shutdown device err=-19；根因（接口/线缆/供电/驱动等）尚不能确定。piper-boot为active(exited)，未重启或改变原使能服务，未尝试使能/运动。40秒udev只读监听无新事件并结束，不把它当重插失败证据。

已请求仅重新插紧USB-CAN，但用户改为“确认目前已经有效的图片数量，下次从此处继续”，因此暂停后续硬件检查/采集，仅确认并保存断点。未收到重新插紧完成确认，不自动继续。当前用户的交互循环可能仍在等输入，可在其窗口Q退出；没有擅自输入OK/NO或终止其他用户进程。

**已确认正式样本2组：pose01、pose02，均train且human_approved=true、board_attached=true、原quality issues为空。**本轮离线重新检测两图、核对静止窗口与已确认图像SHA256通过，PnP RMS分别0.816719px、0.454389px；此为单样本复核，不是手眼外参精度验收。验证样本0组，目标15train+5validation，因此还需13train+5validation，共18组有效姿态。正式数据在`/home/isee-pst/unitree_ros2/piper/handeye-data/manual01/`，预览和决定在`/home/isee-pst/Pictures/piper-handeye/manual01/`。原图和判断未更改。

pose03和pose04已由用户NO拒绝；pose05尚无decision且缺同期机器人位姿，即使USB恢复也不能对旧图补选OK。下次恢复先检查USB身份、CAN UP/1Mbps和新鲜反馈，再用原session继续循环；需要对pose05选NO，然后以pose06或下一个实际未用编号采新图。若期间相机/机械臂基座/标定板末端固定关系变化，旧2组不得与新安装混用，需新批次。标定尚未求解，不发布TF。

暂停断点：`build/piper-can-disconnect-20261112/resume-checkpoint.json`，含样本、判断、计数、硬件阻塞、路径和续跑命令；另存formal-samples.json、loop-log.txt、udev-monitor.txt。原循环入口`cd ~/unitree_ros2/piper`后`bash piper_handeye_loop.sh --session manual01 --board-attached`，恢复时按上述先决条件检查，不盲目拍摄。仅文档/状态记录，无代码修改，不构建/部署、不重跑代码测试或播放代码完成提示音。


### 2026-11-13 重启后重新夹板：应使用新批次manual02

用户说明Go2已重启、机械臂已使能但尚未夹板（本轮未SSH核验该硬件状态），询问是否需整臂失能，以及manual01仍显示旧两组。实读脚本确认session是持久批次，manual01正式保留pose01/02两组train；本机未发现manual02图片/数据目录。旧两组不是新夹持批次的进度；重新夹持位置改变后应固定板再用`bash piper_handeye_loop.sh --session manual02 --board-attached`从0/15、0/5开始，保留manual01历史数据，不混用、不删除。manual02尚未启动或创建。

不需要仅为夹板而整臂失能；夹爪开合与六关节使能是不同操作，整臂失能可能失去支撑。当前独立gripper入口通过GripperCtrl控制夹爪，仍要求原健康/使能/控制模式条件；示教模式下原入口会WAIT_CONTROL，不为了夹板绕过或修改。夹爪历史开合效果尚未验证，本轮不提供猜测力矩/目标，不执行运动、模式、使能或夹爪命令。重新夹牢后再进行既有手摆→结束保持→取图流程，--board-attached只是人为声明，不会使能或夹板。本轮只读代码/本机目录并记录说明，没有更改程序或硬件。


### 2026-11-13 标定板掉落后按用户要求清除当前批次，重启manual03

用户表示已采11次，但板掉落、重新夹取后角度不能保证，要求清除重新开始。实际完整落盘为manual02共10组预览（pose01～pose10），其中7组已人工OK为train（pose03/04/05/07/08/09/10），验证0；第11次无完整样本。本机循环日志末尾PAUSED/user_interrupt，.loop.lock可独占获取；远端未见活动capture进程。未替用户判断未落盘数据，也不把拍摄尝试数当有效样本数。

按授权采取可恢复清理：本机manual02的图片、判断与正式样本全部移出活动路径，归档`/home/isee-pst/unitree_ros2/piper/handeye-archive/manual02-board-drop-20261113/{previews,formal}`；archive.json记录原因/旧新批次/原路径/文件SHA256，45个文件移动前后哈希一致。远端`reviews/manual02`移至`/home/unitree/piper-handeye-20261112/review-archive/manual02-board-drop-20261113`，20文件哈希一致并写归档清单。没有永久删除原数据，没有修改manual01历史记录、程序或异常逻辑。

已创建空的本机`~/Pictures/piper-handeye/manual03`、`piper/handeye-data/manual03`及远端`reviews/manual03`。实际调用已安装loop.counts/pending核对为0train/0validation/0待确认，新编号从pose01开始；两端旧manual02活动目录已移走，不复用其远端旧帧。新一轮须用户先牢固固定板，再在控制电脑交互Bash进入`~/unitree_ros2/piper`运行`bash piper_handeye_loop.sh --session manual03 --board-attached`，保持终端，预期0/15与0/5，按原回车取图、OK/NO判断流程重新收集15+5。本轮未启动循环或相机采集，未发送机械臂动作/使能/夹爪命令。新断点`build/piper-handeye-restart-20261113/restart.json`；此记录取代先前manual01/manual02续采建议。仅数据归档、空批次准备与交接，不改代码、不构建或重跑代码测试。


### 2026-11-13 manual03收齐并离线求解：验证误差偏大，尚未验收

用户报告数据收集完毕。实读COLLECTION_COMPLETE，正式15train+5validation共20组，从29次预览中人工选择。训练pose01/02/03/05/08/09/10/11/12/13/15/16/19/20/22；验证pose24/25/26/28/29。全部human_approved=true、board_attached=true、review_issues为空。本轮离线复核源图片SHA256、批准元数据哈希、OK决定、静止窗口、正常/示教状态、SDK mm/deg转换与保存T_base_end，20组均通过；40个原始图片/JSON检查后哈希不变。

通过实际`piper_handeye.sh solve --dataset .../handeye-data/manual03 --output .../handeye-data/manual03-result.json`执行PARK固定相机求解，退出0，本机日志run-VCkx0kGS.jsonl。仅15组训练拟合，5组验证未参与拟合/参数选择。训练平移RMS13.218659mm、max20.926612mm，旋转RMS2.149121°、max3.292242°；验证平移RMS34.161689mm、max48.322774mm，旋转RMS7.973296°、max9.755556°。这些是手眼方程一致性残差，不是实测定位精度。结果physical_validation=NOT_PERFORMED，不作为已通过抓放标定，不发布TF/部署控制，不隐藏或剔除验证样本来凑通过。

候选T_base_color_optical平移（米）[0.3726449653,0.0697180508,-0.0317474590]；完整矩阵和T_end_marker在结果JSON。训练相对旋转向量奇异值[0.649888,0.275617,0.185021]rad，训练任意两姿态最大旋转差18.502466°，位置跨度[78.004,141.47,57.568]mm。逐一留出一个训练样本仅用于敏感性诊断，相机平移解最大变化30.005863mm；未据验证集成绩选新解。20图PnP重投影约0.168～1.400px，5组验证均出现较大闭环误差，不能当仅一张坏图。离线相对旋转角不变量差RMS训练1.804607°、验证1.497715°、跨组2.423602°；不能由此唯一判断根因。

用户确认本批没有人为调整板，但不排除未发现的滑动；随后询问是否转动夹爪与本体连接处关节影响结果，进一步明确是正常腕部J6，而不是安装法兰松动。已说明正常J6转动是预期标定动作，不会单独使数据失效，前提是板相对夹爪固定且位姿反馈正确。实读J6训练86.487～95.949°（变化9.462°），验证5组均106.699°，确有记录，不能说采集器忽略J6。验证腕角未被训练覆盖，可能暴露不稳定解；板微滑动/固定性、姿态覆盖和实际末端约定仍待区分，不能把相关性当成J6转动错误或滑动已证实。夹爪反馈开口训练从16.1至16.4mm，验证16.4mm；没有据0.3mm变化断言板滑动。

分析交付`/home/isee-pst/unitree_ros2/piper/handeye-data/manual03-analysis/`：quality-summary.json（NEEDS_INVESTIGATION、recommended_for_robot_control=false）、residual-analysis.json、residuals.csv、manual03-residuals.png、rotation-invariant-check.json、sample-audit.json；工作证据`build/piper-handeye-solve-20261113/`。原始manual03全部保留，不因正常J6旋转清除。尚未做外参实物精度验证或真机动作；本轮未SSH、未重拍、未改算法/异常逻辑，未构建或切换ROS overlay。本轮为数据分析与文档，无生产代码修改、不重跑代码测试。

### 2026-11-13 manual04新批次完成：平移一致性改善，旋转仍待排查

用户报告新数据采完。正式20组（15train+5validation），训练pose03/04/08/10/12/13/15/16/17/18/19/20/21/22/24，验证pose26/27/28/29/30。逐组复核人工OK、源图/元数据哈希、board_attached、静止窗口、状态和SDK位姿转换通过；分析后40个原始文件SHA256不变。实际入口离线solve退出0，日志run-8mgrz0qU.jsonl；候选结果`/home/isee-pst/unitree_ros2/piper/handeye-data/manual04-result.json`。只用15训练拟合，未剔除样本或用验证成绩选择模型。

训练平移RMS12.050924mm/max23.643512mm，旋转RMS2.299471°/max4.807072°；验证平移RMS13.164172mm/max19.094855mm，旋转RMS6.284133°/max6.658176°。相较manual03验证34.16mm/7.97°有所改善，但不是实测抓取精度，physical_validation仍NOT_PERFORMED、recommended_for_robot_control=false。训练最大相对旋转52.965101°（旧18.502466°），位置跨度[108.212,160.461,97.445]mm；逐一留出训练样本，相机平移解最大变化24.127270mm，仍敏感。候选T_base_color_optical平移米[0.4159578390,0.0529010551,-0.1253200610]，未发布TF/写入控制配置。

离线对照已安装piper_sdk的C_PiperForwardKinematics默认dh_is_offset=1：manual04关节计算末端与采集SDK反馈最大差0.009797mm、0.002313°，内部位姿转换一致；不代表真实机械模型精度已经验证。另一offset=0约9.95mm差异仅诊断，不改变配置。本批夹爪宽度反馈全为25.3mm，不能据此证明板没有滑动。5验证旋转残差均5.01～6.66°；正常J6运动不是错误，剩余根因（板固定/平整度、视觉测姿/内参、真实机械模型）尚未确定，不再次盲目要求全批重采。下一步建议先核对刚性安装/固定标记，再做少量静止重复拍摄与返回原姿态复测；现场复测尚未执行。

交付`/home/isee-pst/unitree_ros2/piper/handeye-data/manual04-analysis/`：report.txt、quality-summary.json、sample-audit.json、residual-analysis.json、residuals.csv、fk-cross-check.json、manual04-residuals.png，7文件复制哈希核对通过。工作证据`build/piper-handeye-manual04-20261113/`；仅临时分析器的mount_xyz单位标签纠正为mount_xyz_m（数值本来为米），未改生产算法。没有SSH、运动/使能/模式指令或新采集；原数据和异常处理未改。仅离线分析与文档，不构建ROS、不重跑代码测试或播放代码修改提示音。

### 2026-11-13 少量复测retest01已开始，等待调整可见姿态

用户授权少量复测，并确认manual04后板夹持、机身和相机安装未变，已结束示教并静止。SSH实际可达，can0 UP/ERROR-ACTIVE/1Mbps，librealsense枚举D435序列151222077264；未启动或重启控制服务。复用现有review-local snapshot入口，单独session=retest01，未改生产代码/异常处理。已拍a01并回传`/home/isee-pst/Pictures/piper-handeye/retest01/a01/color.png`；本机日志run-Kgt5YIPX.jsonl，远端run-tgHjY9Zz.jsonl。反馈ctrl2、teach2、arm_status0、err0，静止位移/转角/关节跨度均0；command_sent=false。

实看a01标记大部分超出画面上沿，检测ID25数量0、ready_for_approval=false，因此停止后续重复拍摄，未批准/混入manual04；当前可用复测0组。已给用户原图链接并请求保持夹持不变，通过示教调整整个机械臂，使黑色四角和白边完整可见，再结束示教保持。异步问题尚待回复；不能把等待时间当完成确认。恢复后用新编号a02试拍，不覆盖a01。计划先同姿态3张、人工OK/NO，再少量改变姿态/返回复测；无软件运动指令。断点`build/piper-handeye-retest-20261113/checkpoint.json`。本轮仅采集和记录，无代码改动、构建或实物精度验收。

### 2026-11-13 retest01静止组已拍，待人工OK/NO

用户回复“已经调整”后，以原被动snapshot入口完成a02/a03/a04，均review_issues=[]、command_sent=false，图片在`/home/isee-pst/Pictures/piper-handeye/retest01/a02/color.png`（a03/a04同结构）。实际查看标记四角完整、靠右下。尚无人工OK/NO，用户询问路径，已给控制电脑实际路径；不得把路径询问当批准。三个本机日志分别run-ODFlwf9j.jsonl、run-gS70cXW9.jsonl、run-flYkgDja.jsonl；没有发送运动命令或改代码。

离线初步重复性（未人工批准、不参与拟合）：三图marker相对位置最大差0.717274mm、旋转最大差0.488827°；机器人反馈跨图最大差0.184399mm、0.116247°，a03/a04反馈相同。重投影RMS0.390403/0.065525/0.209178px。不能据3张证明物理精度，也不能将板位姿变化全部当相机噪声。证据`build/piper-handeye-retest-20261113/static-a-analysis.json`，含重新检测、两两差异和6文件哈希。checkpoint更新WAIT_HUMAN_IMAGE_REVIEW。下一步先收用户对a02～a04的OK/NO，再引导手动换B姿态并确认就绪后拍b01等；尚未进行换姿态或返回原姿态复测，manual04保持原样。

### 2026-11-13 retest01静止A组三张已人工确认，等待B姿态

用户明确“三张均OK，但三张照片一致”。已解释同姿态重复拍摄是预期设计，不是三个新标定姿态；三图SHA256不同、采集时刻不同。复用review.approve与loop.decide原逻辑，a02/a03/a04正式批准到`/home/isee-pst/unitree_ros2/piper/handeye-data/retest01/`，role=validation，仅诊断，不加入manual04或重新拟合。批准前复核6源文件hash不变，批准后human_approved和role正确，3个decision.json均OK。

已请求用户通过示教摆B姿态（改变板朝向、完整可见，保持夹板/相机/机身安装不变），结束示教保持后回复；尚未收到就绪答复，未采B。下一编号b01。之后考虑返回A附近的复测，按实读关节差判断返回程度，不假定手摆能精确回到原位。checkpoint=WAIT_USER_POSE_B，3组已批准；分析static-a-analysis.json已注明人工批准和仅诊断用途。无生产代码修改、软件运动指令或TF发布。

### 2026-11-13 retest01 B组完成并批准，等待返回A附近

用户确认B摆好后，经现有snapshot入口采b01/b02/b03，全部review_issues=[]、command_sent=false；实看四角完整，三图均返回`~/Pictures/piper-handeye/retest01/bNN/color.png`。本机日志run-G5fr4JlB.jsonl/run-OHI8XKe9.jsonl/run-vamKOKVI.jsonl。用户明确“B组三张均OK”，批准前核对源hash，通过原review.approve/loop.decide保存到handeye-data/retest01，role=validation、不参与拟合。累计6组人工批准。

离线重新检测：B组机器人反馈完全相同，视觉位移最大差0.784475mm、旋转最大差0.697864°，重投影RMS约0.875489/0.916978/0.857327px。A→B机器人旋转角15.222712～15.267476°，视觉13.934564～14.251362°，角度不变量差0.971350～1.288148°；不由此单独判定滑动或模型错误。marker与robot原点平移长度不应直接相等，不能拿两者长度差当误差。分析`build/piper-handeye-retest-20261113/ab-analysis.json`含6样本、两两差异和12源文件hash。

已请用户参考a02图、手动示教回A附近，整个臂形/板朝向尽量接近，保持夹持不变；不要求精确回位，之后用真实关节差判断可比性。尚待就绪答复，未采C。checkpoint=WAIT_USER_RETURN_A，下一编号c01。无软件运动命令、无拟合/部署、未改生产代码或异常处理。

用户随后确认已返回A附近并静止。试拍c01完成（本机run-Jl5omwYS.jsonl、远端run-K4CK6M2b.jsonl），但实图标记右侧被裁切、检测ID25为0，未批准；停止后两张。已回传c01图片链接并请用户保持夹板不变、微调整个臂姿让画面中标记向左/稍远、完整入镜。等待新就绪，checkpoint=WAIT_USER_C_VISIBILITY，下一c02；原A/B六组保留不变。

用户确认微调后试拍c02（本机run-taRAm3DV.jsonl、远端run-hqOV3HCK.jsonl）。实图已向左，但标记下侧黑角被裁切，ID25仍未识别、review_issues非空，未批准。已给c02图片并请保持夹持、示教让板离相机远一些/缩小居中；下一编号c03，等待用户再次静止确认。没有持续无人值守重拍或改检测阈值。

### 2026-11-13 retest01少量复测采集/分析完成：9张批准，根因未定

中断恢复后核对已有状态，未重复已完成动作。用户明确可拍c03后完成c03/c04/c05（本机日志run-LBOQ6ZM6.jsonl/run-0INSB9Eo.jsonl/run-p7CEJXzA.jsonl），均review_issues=[]，实看标记完整。用户明确“C组三张均OK”，通过原review.approve/loop.decide保存为独立retest01验证数据。累计A(a02/03/04)、B(b01/02/03)、C(c03/04/05)共9张人工批准，只有三个近似姿态；a01/c01/c02裁切试拍保留但未批准。18个预览源文件hash和9个正式图源一致性验证通过；未重新拟合或把复测加入manual04。

组内视觉最大两两差：A 0.717274mm/0.488827°；B 0.784475mm/0.697864°；C 1.020836mm/0.956124°。B/C机器人反馈完全相同，A反馈最大0.184399mm/0.116247°。跨姿态旋转角不变量差A-B 0.971350～1.288148°，A-C 2.126875～2.812692°，B-C 0.352067～1.078079°。C实际未精确返回A：末端位置差23.847919～23.970895mm，朝向差18.445649～18.475640°，单关节最大差17.309°；因此不能把C当相同物理姿态来判断滑移。不能要求用户继续盲目追同姿态或把手摆回位假定精确。

固定manual04候选计算复测闭环：A平移26.60～27.01mm/旋转5.09～5.30°；B 25.46～25.52mm/5.11～5.53°；C 21.65～22.27mm/6.03～6.20°。均非实物定位精度。此次短时波动小于持续存在的旋转残差，但仅3张/组，不证明完整噪声分布、无慢漂移或无滑动。仍需区分板平整/刚性、视觉系统偏差/内参、机械臂物理模型、历史固定关系变化；不确认唯一根因，不把正常J6旋转归因错误。建议下一步先实物核对黑方块横纵尺寸是否均142mm、平整刚性和透明覆盖物反光/弯曲情况，再决定后续；调整安装须新批次。没有进一步采集安排。

报告保存`/home/isee-pst/unitree_ros2/piper/handeye-data/retest01-analysis/`：report.txt、abc-analysis.json、residuals.csv、checkpoint.json，4文件复制hash通过。工作证据`build/piper-handeye-retest-20261113/`，checkpoint=RETEST_COLLECTION_AND_ANALYSIS_COMPLETE（仅复测任务完成，标定尚未通过）。没有运动/使能/夹爪/模式指令、TF发布或生产代码改动，原异常处理未变，不构建ROS/重跑代码测试/播放代码修改提示音。

### 2026-11-13 按用户要求准备manual05：10组求解＋5组验证

用户要求重新采集“10次采集5条验证”，按10个不同有效训练姿态+5个独立验证姿态处理。最小修改本机piper/handeye/loop.py：新增`--train-count {10,15}`默认15，collect接收train_count并用于显示、角色切换及完成计数；验证仍5。旧命令保持15+5，所有异常处理AST逐一核对完全一致，采集/批准/控制模块及shell入口hash未变。不修改ROS、远端代码或异常逻辑。

新命令：控制电脑交互Bash进入`/home/isee-pst/unitree_ros2/piper`，执行`bash piper_handeye_loop.sh --session manual05 --train-count 10 --board-attached`。板先刚性固定、机身/相机安装不变；不需source ROS。保持终端，手摆不同姿态、结束示教静止后回车取图，再OK/NO；前10个有效样本train，后5个validation，NO不计数。Q暂停，续采必须保留同一完整命令含--train-count 10，不要在批次中改变目标参数；旧样本不重新分类。不会自动求解、使能或运动。

两端manual05此前不存在：本机Pictures与正式data无目录，SSH只读检查远端reviews/manual05为MANUAL05_UNUSED。安装后实际PTY入口显示训练0/10、验证0/5和pose01待采提示，仅输入Q退出0；日志run-80JbLAYs.jsonl为PAUSED/command_sent=false。本机现仅创建空Pictures/manual05及.loop.lock，没有新图片/正式样本，远端未启动采集。此前manual04、retest01全保留。

测试先增2项（10+5全流程含NO、10+4续采最后validation），原版9通过/新增2因不支持参数失败；修改后循环11通过。已安装标定套件58通过（原56+新增2），pytest仅有两条缓存目录无写权限警告，不影响测试；独立控制基线49项另进程通过。Python3.8语法、Bash语法、原异常处理AST和控制/采集/入口hash验证通过。首次stage测试曾因未链接standalone导入失败，补齐测试stage原依赖路径后完成红绿验证，未改生产导入。备份/stage/原hash在`build/piper-handeye-10plus5-20261113/`。安装3文件loop.py、test_loop.py、handeye/README.md；无需重建ROS或远端部署，入口按原root路径调用已验证解释器和本次loop.py。未提交Git；当前仅准备好新采集入口，尚未采样或标定通过。

### 2026-11-13 manual05收齐10+5并离线求解：较manual04未改善

用户报告完成采集，实读日志COLLECTION_COMPLETE=10train/5validation；从23次预览选出训练pose01/02/03/04/06/08/13/15/16/17、验证pose18/19/21/22/23。15组人工OK、固定声明、源图/元数据/decision哈希、静止窗口、状态和SDK转换复核通过；30原始文件分析后hash不变。

实际入口`piper_handeye.sh solve --dataset handeye-data/manual05 --output handeye-data/manual05-result.json`退出0，本机run-UUk0EIyJ.jsonl。训练平移RMS34.993990mm/max48.927004mm、旋转RMS5.184146°/max6.869636°；验证平移RMS33.916591mm/max48.582316mm、旋转RMS7.708157°/max12.215974°。manual04验证13.164172mm/6.284133°，本批未改善；不能仅两批对比认定样本数减少是唯一原因。都是方程闭环残差，不是实物精度。physical_validation=NOT_PERFORMED，recommended_for_robot_control=false，未部署。

训练最大两姿态旋转52.914262°，位置跨度[139.772,299.767,69.403]mm，15张重投影RMS均<1px。逐一留出训练样本仅用于诊断：留出pose04时core.solve拒绝非合法刚体变换，其他9次最大相机平移变化65.746181mm、旋转17.017885°（均留出pose13）；完整10条求解成功，未剔除样本或按验证成绩选择新模型。临时分析脚本首次因该ValueError终止，随后仅在临时分析器记录失败项再完成其余诊断，没有更改生产异常处理或算法。

夹爪宽度反馈20.4～23.3mm，pose01=21.8、pose06=23.3、pose08=23.2、pose13=20.4mm。已询问现场，用户确认“没有主动调整，但可能有滑动”。这是待排查线索，不是已证实滑动根因；正常J6运动不当作错误。建议先改善板与夹爪刚性固定并核对平整度/尺寸，再少量复测，暂不盲目要求再次整批重采；安装变化应新批次。

保存候选`/home/isee-pst/unitree_ros2/piper/handeye-data/manual05-result.json`；分析目录`manual05-analysis/`包含report.txt、quality-summary.json、sample-audit.json、residual-analysis.json、residuals.csv、manual05-residuals.png。工作证据`build/piper-handeye-manual05-20261113/`。6文件复制hash核对，用户答复随后同步补入两份报告。只有离线分析和文档；未SSH、未拍图、未发送运动/使能或发布TF，未改生产代码/原样本，不构建ROS或重跑代码测试。2D残差图已查看；matplotlib有3D扩展不可用提示，不影响2D图。

### 2026-11-13 用户观察夹爪自行变宽，询问独立使能

只读核对SDK0.6.2 GripperCtrl文档/实现及现有standalone接口：gripper_code=0x01为使能，调用同时携带目标开口与力矩，set_zero=0x00不设零；不是无目标的机械锁定，不能把默认开口0当纯使能。现有gripper动作已调用GripperCtrl(*target,0x01,0x00)，common.assess遇teach_status非0或ctrl_mode不在0/1时WAIT_CONTROL，未绕过/改逻辑。

manual05全部15个采样snapshot的gripper_enabled均true，ctrl_mode=2、teach_status=2；开口20.4～23.3mm，力矩反馈约-0.025～-0.046（现有SDK换算值）。采样时使能不等于整个示教切换过程始终保持同一目标；反馈力矩也不是配置的夹持目标力矩，不能据此断言力矩设置过小。需区分示教期间控制变化、负载/接触滑动、机械间隙或其他控制来源，未确认根因。手眼标定建议将板刚性固定到末端不随开合移动的结构（合适安装支架/法兰），若用活动夹指则仍需验证开口/板固定关系。未SSH、未发使能/开合/清错/设零命令，不改代码。外部核对来源为agilexrobotics/piper_sdk官方gripper示例，现场以本机0.6.2源码为准。

### 2026-11-13 新现场观察：J6旋转时夹板接触位置变化

用户报告J6转动时可明显观察夹爪抓取板的位置变化，认为保持J6不变会好一些。应区分相机视角中板整体旋转与板相对夹指接触位置真正移动：后者违反固定末端到标记变换假设，是重要现场证据；尚不能把所有历史残差归于单一根因。可尝试手动保持J6角度，用其他关节提供至少两个不平行方向的末端旋转及位置变化；手眼标定不要求每个关节都变化。依据OpenCV4.11 calibrateHandEye文档的非平行旋转轴要求。保持J6不能代替板刚性固定，其他关节运动时也需检查接触位置；发生滑移前后不可混用。未将J6软件锁定、未修改脚本或发动作；后续采集批次与现场就绪尚未确认。

### 2026-11-13 按用户要求准备manual06：保持J6角度，10+5重新采集

用户要求再采一轮10条试验+5条验证，沿用10train/5validation，新批次manual06。两端只读检查本机Pictures/handeye-data及远端reviews/manual06此前均未使用。复用已验证入口，无代码修改：`cd /home/isee-pst/unitree_ros2/piper`后`bash piper_handeye_loop.sh --session manual06 --train-count 10 --board-attached`。实际PTY显示0/10、0/5及pose01待采，仅输入Q退出，日志run-Y8iX99lZ.jsonl；未启动相机/采样或发运动命令。本机只准备空Pictures/manual06及锁文件，原批次全保留。

用户手动示教时尽量保持J6角度不变，其他关节提供不同位置/至少两个不平行旋转方向，每次结束示教静止再回车拍摄和OK/NO；保持夹板位置不变，若出现接触滑动则暂停该批次。没有软件锁定J6或新增运动门槛，后续需按采样反馈核对J6实际变化。续采使用同一完整命令含--train-count 10，10+5齐全后自动结束，仍需离线验证。此轮只准备入口，不构建/重跑测试。

### 2026-11-13 用户明确采用manual04，保持代码不变

最新用户指令“采用manual04”，后续接入以`/home/isee-pst/unitree_ros2/piper/handeye-data/manual04-result.json`为选定标定来源，不再把manual05或尚未完成的manual06作为默认替代。原文件SHA256=b8f843b4aee21b530e53c642d1ceaa16868c7680b16610504157135596433151；本轮核对三个SE3矩阵、正逆变换乘积、米制单位及15train/5validation数量通过，未改原结果或数据。

采用的相机到Piper基座变换字段为T_base_color_optical（color optical -> SDK arm base），不是Go2 base_link/TCP变换；相机序列151222077264。适用前提是相机与机械臂基座安装关系保持不变，当前未重新实测安装。重新夹板后不能直接复用旧T_end_marker到新的板安装关系。用户采用决定不代表实物验收通过；保留physical_validation=NOT_PERFORMED及验证RMS13.164172mm/6.284133°原值。

更正此前措辞：没有预先约定整轮残差的硬性验收阈值；后提出5mm/1°只是建议目标，不可回溯当作旧门槛。此前分析中的NEEDS_INVESTIGATION/不推荐控制为工程判断，不是脚本自动验收失败。现在按用户选择采用manual04作为后续接入来源，原分析证据不改写。代码搜索未见当前独立采集/控制栈读取手眼结果的运行配置，因此本轮完成选用记录，未宣称控制链路已加载；未改代码、未发布TF/启动控制或自动停止用户的manual06采集进程。旧批次全部保留。

### 2026-11-13 静止Go2视觉抓取代码只读盘点

用户询问Go2保持静态时后续抓取如何实施、piper目录有何可复用代码；本轮只读，不视为动作或改代码授权，继续保留原异常处理。已读实际main4/primitive3/aloha/ik_solver/pointcloud2/util及standalone和既有方案。

可复用：piper_go2.sh→机载standalone/piper_control.py的check/inspect/joints/gripper及诊断；handeye/capture.py设备身份和被动反馈逻辑（现仅color，无depth）；agent/pointcloud2.py已有RGB-D、align到color、读取depth_scale和内参，但默认相机151222076721非实机151222077264，旧context需按已验证方式适配；agent/primitive3.py:grasp_object为开爪→预抓取→接近→闭合→抬升顺序参考；agent/ik_solver.py提供Pyroki solve→6关节度数；main4/vlm_agent可参考识别/分割/任务拆分。

不可直接运行旧整链：primitive3导入即PiperRobot；piper_arm构造ConnectPort/EnableArm(7)，TCP固定0.135m；open/close会写MotionCtrl_2并用0/70mm固定目标；goto_joint超时break后仍返回目标，调用者不能据返回断言到位；抓取原语固定抬升0.18m且直接记持物，未充分证明抓住。util含旧相机外参、旧写死内参，未读manual04；IK路径/home/isee604且target link6、URDF旧版，须核对真实模型/TCP/单位，不把IK解当整条路径无碰撞。main4写死cuda:0/旧模型路径。旧main_robot是192.168.31.201:9999 socket方案，当前Go2 runtime仅注册导航start_find_target/get_task_status/cancel_task，没有抓取工具。

本机co-nav-real用find_spec只读检查：jax/jaxlie/jaxls/yourdfpy/pyroki/autolab_core/segment_anything/pyrealsense2/pinocchio/casadi未找到，torch/open3d可找到；不是实际导入或GPU复验。Pyroki项目声明Python>=3.10，机载沿用历史Python3.8设备层，建议计算留控制电脑、RGB-D与CAN留Go2，用现有SSH通路传快照/明确命令；不逐帧SSH发控制，不改导航overlay。

建议首版先人工指定图中目标→对齐深度取米制3D→manual04的T_base_color_optical变到Piper基座→实测TCP与姿态→IK及路径预览→分段抓取并确认持物；随后用VLM自动目标选择替换人工输入、封装独立抓取工具，暂不接导航/携带。需先验证夹爪正常模式的保持能力与示教退出流程，采用已选manual04但不声称实物精度已验收。没有额外引入残差硬性门槛。

已异步询问首个物体类型/大小及台面或地面位置，尚待答复。若地面，先实测可达性，既有ground_reachability文档结论基于假设H/D不能当当前事实。未安装依赖、未执行旧入口、未SSH或发运动，本轮未改生产代码。

### 2026-11-13 夹爪使能后可轻易手动开合：代码及输入离线复核

用户报告机械臂与夹爪使能后，夹爪仍几乎无阻力地被手动改变开口。本轮只读核对本机standalone/piper_control.py、piper_common.py、piper_go2.sh、仓库开机/夹爪工具及已安装SDK0.6.2；未SSH或获取当前实机反馈。已询问此次具体使能入口及是否结束拖动示教，尚待回答。不能从历史使能位或当前手感单独判定硬件正常/损坏。

确认开机工具仅调用EnableArm(7)，以六关节新反馈确认READY，没有GripperCtrl；独立夹爪入口在piper_control.py:188调用GripperCtrl(*target,0x01,0x00)，同时携带目标开口和力矩，不清错、不校零。输入宽度须有限且0..100mm，再检查设备70/100mm行程；力矩须有限且0.001..5N·m，转换均乘1000。SDK直接调用若只传gripper_code=1，默认effort仍为0；现有CLI要求显式正力矩，可避免这个遗漏，但数值合法不证明足够夹持。不能把反馈力矩当请求力矩，AT_TARGET也不是保持能力验收。

模式语义补充：SDK0.6.2的ctrl_mode=2表示示教控制模式，teach_status=2明确表示结束示教记录（退出拖动示教模式），不能把teach_status非0直接解释成仍在拖动。现有common.assess在teach_status非0或ctrl_mode不为0/1时返回WAIT_CONTROL、不发送夹爪目标；本轮用替身确认(2,2)/(1,2)均等待，(0,0)/(1,0)允许尝试。此为现有行为，不证明当前实际模式，也未改原异常/等待策略。历史manual05全部15条使能为真且模式2/2，仅作线索。历史2026-11-06曾在模式0/0、10mm/0.5N·m目标及有界周期发送下仍无位置变化，不能仅凭示教或单发差异解释全部现象。

本轮验证：独立控制基线49项、开机/夹爪基线66项全部通过；另一次性内存检查10组非法输入拒绝、3组合法单位转换、4组模式分支均通过（不是新增生产测试）。复跑既有sdk_offline_check.py，禁止socket、替身发送，0x159编码0000271001f40100（10mm/0.5N·m/code1/不设零）核对通过；其ERROR输出为预期故障注入，不是真实总线错误。未修改生产代码或部署，未重建ROS、播放代码修改提示音或重新进行实物试验。判断：正常位置保持下持续近乎无阻力且不恢复目标值得排查；仅使能不能证明保持功能正常，当前缺具体入口/请求值/实时模式和物理响应，根因待验证，不自动加力矩、切模式、重发或刷固件。


### 2026-11-13 将用户选定 manual04 外参写入本机 Piper agent

用户明确要求“将manul04的结果写入代码”，完成后输出相机到Piper基座外参。已将 `/home/isee-pst/unitree_ros2/piper/handeye-data/manual04-result.json` 的 `T_base_color_optical` 转换为现有 translation（米）及 quaternion（xyzw），更新 agent/util.py、agent/vlm_agent.py、agent/get_symmetric_center.py 三处重复常量。方向是 RealSense 彩色光学坐标系到 Piper SDK 基座；不是 Go2 base_link，也不是 TCP 或 T_end_marker。仅六处赋值和来源注释变化，全部其他 AST（含异常处理）保持一致；历史标定文件和结果状态未修改。

安装后离线验证三文件语法、单位四元数、重建矩阵、逆矩阵与三个测试点转换全部通过；与源矩阵最大元素差 1.055e-15。源 JSON SHA256 仍为 b8f843b4aee21b530e53c642d1ceaa16868c7680b16610504157135596433151。备份、stage、原文件哈希及 verification.json 在 build/piper-manual04-apply-20261113/。纯 Python 常量修改，无 ROS 构建/overlay/启动入口变化；未导入含硬件副作用的旧 agent、未 SSH、未部署机载端、未发送控制或验证实物抓取。

边界：选用 manual04 不改变其验证残差（平移 RMS13.164172mm、旋转 RMS6.284133°）或代表实物精度通过。旧抓取依赖、相机内参/序列号、TCP 尚需匹配。另只读发现 vlm_agent.axes_in_camera_from_tool_rotation 将全局四元数直接解释为世界到相机，且实际返回 x_w/y_w/z_w；本轮未修改该历史工具轴显示逻辑，后续接入需核对调用者及方向，不能仅因外参已写入就宣称旧抓取链可运行。


### 2026-11-13 矿泉水瓶抓取现场只读检查：视野、距离与夹爪反馈待核实

用户要求Go2静止抓取地面直立矿泉水瓶（约高23cm、宽6.4cm），倾向左前侧且无法精确摆放；给出分段验证方案后用户报告已放置并要求继续。本轮实际执行SSH取图、短时RGB-D采样和被动CAN反馈，无关节/夹爪/使能/模式/校零命令，无导航/站姿改变。未部署或修改生产代码。

工作证据`build/piper-bottle-ground-20261113/`。复用既有review.snapshot采view01（机载run-P4L09zg6.jsonl），原地面场景只有瓶盖/上部入镜；用户随后垫约7cm盒子，view02（run-5vzC9Ijk.jsonl）可见上半瓶身，但底部/盒子仍裁切。RGB-D rgbd01共3帧；随后用户报告把瓶子移远，重新取rgbd02共3帧及被动配对机器人窗口，最新图为`rgbd02/frame00-color.png`。相机151222077264，640x480@15，真实depth_scale=0.0010000000474974513，原始深度与SDK对齐深度均保存；没有常驻订阅。每组7个源文件复制SHA256核对通过。机载临时源目录分别/tmp/piper-bottle-rgbd-lyt784da、/tmp/piper-bottle-rgbd-j03zh0ps，均保留。垫高场景不能记为地面抓取通过。

离线用原始深度反投影、设备depth-to-color外参（column-major正确重建）、用户选定manual04转换；三帧结果接近不代表绝对精度。移远后可见瓶身表面中值基座坐标约[0.864,0.105,-0.159]m，标签表面约[0.853,0.107,-0.198]m，均不是瓶轴/TCP。候选模型肩部[0,0,0.123]m至法兰任意方向长度上界0.627741899m；肩部至可见瓶身约0.915m，因此候选模型需要至少约0.287m工具长度才能通过必要条件，旧未实测TCP为0.135m，当前初筛不通过。实际模型/TCP与manual04实物精度仍未验收，不把此初筛写成实机绝对不可达。此前让瓶移远仅改善视野，已明确修正建议、不再让用户继续盲目挪动。等待用户粗量Piper基座中心到瓶身中心的水平距离以核对视觉估计。

两次图像配对反馈均ctrl_mode=0、teach_status=0、arm_status=0、err_code=0、六关节及夹爪使能true、无缺失反馈；机械臂窗口静止。但gripper_width_mm=-27.5。两端snapshot均直接按SDK grippers_angle/1000转换，未加偏移。被动CAN原始检查gripper-passive-03.log有20帧显式核验ID0x2a8、新鲜、标准数据帧；前4字节ffff9494按现有V2有符号int32转换均-27500，即-27.5mm。未确定零点/硬件/协议等根因，不取绝对值、裁零或自动校零。已询问实际两夹指内侧间距。临时第一次诊断误传str给load_serial导致Path类型异常，发生在CAN连接前；修正调用后成功，原始日志保留。第二次记录缺少frame ID且有一条不同值，不能逐条认定0x2a8，已由第三次显式ID检查取代；未修改原控制或异常处理。

checkpoint=WAIT_PHYSICAL_DISTANCE_AND_GRIPPER_GAP。没有生成抓取中心/IK/无碰撞路径、没有实际开合或抓取，旧标定数据及结果不变；手眼review中缺marker警告是瓶子照片的预期，不批准或加入标定集。下一步先核对用户实测两值，再决定可达放置区域及夹爪排障；不要把历史动作授权或本轮被动采集当作自动校零授权。纯证据/交接更新，不构建ROS、不重跑产品测试、不播放代码修改提示音。


### 2026-11-13 瓶子移近后候选定位完成；等待单次夹爪校零授权

用户粗量原位置基座中心到瓶身约87cm，与视觉估计一致；同时确认夹爪实际闭合，-27.5mm不能当实际间距。不是对外参毫米级精度或完整机械模型的验收。没有自行加27.5mm软件补偿。

离线候选URDF FK与当前SDK末端位置差0.074614mm（只一姿态、非物理TCP验证）。固定水平接近的9组候选未找到精确解；改为斜向下30/45/60/90度、距离55/60/65cm和基座Z=-0.16/-0.18m的24组，16组找到模型内姿态解。使用旧未实测TCP沿link6+Z 0.135m和公开URDF限位，无碰撞/整条路径检查，计算容差仅求解诊断、不新增现场门槛。JSON在build/piper-bottle-ground-20261113/candidate-placement-ik.json与candidate-placement-tilted-ik.json，不是可执行轨迹。

据此请用户连同7cm盒子向机器人移近20～25cm、保持左前侧；用户已调整。采rgbd03（三帧及静止机器人窗口），7文件源SHA256核对通过。瓶盖/上中瓶身可见、底部仍裁切；不再要求为了全瓶入镜外移。标签上缘有效深度约91～92%，深度中值0.181m，透明瓶身ROI有效约65～70%。按用户近似半径0.032m、瓶轴近似基座Z假设，对标签上缘局部圆弧拟合，三帧候选中心约[0.63892,0.07863,-0.16005]m，水平64.37cm；拟合RMS0.45～0.51mm只是局部模型拟合，不是绝对抓取精度。需实测TCP/姿态/接近与碰撞验证；三帧未生成真机命令。证据rgbd03-surface-points.json、rgbd03-candidate-bottle-center.json。当前仍是垫高场景，不记地面抓取通过。

两端SDK0.6.2在禁止socket且替身发送条件下验证GripperCtrl(0,0,0x00,0xAE)，仅编码0x159/00000000000000ae，即夹爪失能并设当前位置为零，不清错、不重新使能、不动六关节。结果zero-proposal-local-offline.json及zero-proposal-go2-offline.json。这一步尚未实发；已明确询问用户是否授权对空载完全闭合夹爪执行一次并随后只读核对，因其更改硬件零点，不能把先前抓取开合授权当作校零授权。checkpoint=WAIT_GRIPPER_ZERO_AUTHORIZATION。仍无生产代码改动、无关节或夹爪动作。


### 2026-11-13 用户授权单次夹爪校零：反馈归零，保持失能，未抓取

用户明确选择“授权一次夹爪校零并读取反馈”，范围为当前空载完全闭合夹爪，单次失能+当前位置设零，随后仅被动读取；不清错、不重新使能、不发送六关节动作。实际遵守此范围，未将历史抓取计划用于追加开合。

执行前无其他已知Piper控制进程，指定USB对应can0为1Mbps/ERROR-ACTIVE；共用现有每USB控制锁。使用既有Reader的piper_init=False、start_thread=False连接，实发前保存唯一receipt，传输包装仅允许一次0x159/00000000000000ae；SDK GripperCtrl(0,0,0x00,0xAE)返回SEND_MESSAGE_SUCCESS。机载receipt=/tmp/piper-bottle-zero-20261113-01.json（O_EXCL，不重用/重发）。实际本次仅1帧；can0累计TX由1变2。约0.5秒内反馈从-27.5mm/使能true变为0mm/使能false；之后11条新鲜观察为0mm/false，持续约2.5秒，无缺失反馈、err_code=0。前后六关节角及SDK末端反馈完全一致，未发六关节指令。断开只结束观察，不重新使能。

证据build/piper-bottle-ground-20261113/zero-authorized-execution.log、zero-authorized-remote-receipt.json、can-after-zero.json，远端receipt与本地最终记录逐字段一致并完成一次发送/目标帧/11条归零/关节不变断言。checkpoint=GRIPPER_ZERO_OBSERVED_DISABLED_NO_GRASP。只证明当前闭合位置零点反馈已恢复，不证明原负值唯一根因、重启持久性、开口比例、夹爪保持或抓取通过。当前夹爪保持失能，六关节原使能状态不变。

瓶子仍在约7cm盒子上、最后估计中心水平64.4cm。未修改或部署产品代码、未重建ROS、未重跑产品测试。后续需在明确操作范围下验证夹爪实际开合/保持，并核实TCP和完整路径；不能直接使用候选坐标/旧0.135m工具参数执行抓取。


### 2026-11-13 校零后10mm小开口未执行，用户建议核对校零流程

用户说“继续”后，明确说明并执行一次10mm/0.2N·m夹爪目标，未动六关节。实际使用已部署piper-standalone-20261111/piper.sh gripper，三文件SHA256与本机一致；沿既有每USB锁、现有反馈/故障/实时行程检查。运行前can0正常，无其他已知控制进程；用唯一目录/tmp/piper-bottle-open10-20261113-01防止重复入口，12秒SIGINT结束监测（timeout状态124，程序EXITED），不将退出说成物理停止。尝试后夹爪保持使能，目标可能仍被控制器保留，未自动失能或重发。诊断终端按已有piper_go2.sh diagnostics打开。

远端日志/home/unitree/.local/state/piper-standalone/gripper-stu9ve69.jsonl，本机open10-runtime.jsonl：1次COMMAND_ATTEMPTED，其后22次WAIT_TARGET，宽度全部0mm、使能true、ctrl_mode0/err_code0，无缺失反馈，六关节全程不变。can0累计TX2→4，与1次行程查询+1次夹爪命令一致，不是固件动作应答。用户也确认夹爪没有张开，随后两次提出“可能校零没做好”。未提高力矩、连发、清错、回零或修改原错误处理。

已回应：刚才只确认反馈零点，不证明机械闭合基准正确，不能认定用户操作错误。SDK0.6.2官方piper_set_gripper_zero.py是GripperCtrl(0,1000,0x00,0)，等待1.5秒，再GripperCtrl(0,1000,0x00,0xAE)。本轮先前使用合并失能+设零，存在顺序差异，但未证明其为根因。拟按官方两步顺序、在失能后由用户确认空载完全合拢再单次设零并读取；不做六关节回零/重新设六关节零点、不清错或自动开爪。前一次单次校零授权已用完，因此已请求这次重新校零授权，尚未执行。

官方同版本piper_ctrl_gripper.py调用ConnectPort/EnablePiper、夹爪失能清错及使能，然后周期发送夹爪目标；没有显式MotionCtrl_2。本地EnablePiper只EnableArm(7)，故ctrl_mode0是线索，不能直接认定待机导致夹爪不动；不执行整个官方demo，避免清错、循环50mm开合等越界动作。官方piper_ctrl_go_zero.py会发送六关节零角目标，是运动命令，不是本次需要的夹爪零位设置。来源：https://github.com/agilexrobotics/piper_sdk/blob/0.6.2/piper_sdk/demo/V2/piper_set_gripper_zero.py 及同目录两个示例。

两步候选报文在本机SDK0.6.2禁止socket、替身发送下验证为0x159/0000000003e80000和0000000003e800ae，结果official-two-step-zero-offline.json；仅编码自检，无实发。checkpoint=WAIT_OFFICIAL_TWO_STEP_ZERO_AUTHORIZATION。无生产代码修改或构建；原目标仍为静止抓取，当前瓶子在约7cm盒子上、候选中心水平64.4cm，尚无物理抓取通过。


### 2026-11-13 官方两步校零仅完成失能；用户随后自行按教程回零/校零

用户授权“按两步流程校准一次”后，第一步仅发送GripperCtrl(0,1000,0x00,0x00)，唯一receipt=/tmp/piper-bottle-official-zero-20261113-01-disable.json，本地official-zero-disable-execution.log。仅一条0x159/0000000003e80000，观察约3秒确认夹爪使能false、宽度0mm、六关节不变。随后用户明确回复“空载完全合拢”，但在第二步设零执行前会话被中断，用户又报告“按照教程对机械臂进行了回零（校零）操作”。第二步从未发送，不能在恢复后自动补发或重做第一步。

恢复后只读核对：机载当前启动时间2026-11-13 21:01:36，boot_id=851af1ce-00f5-4e38-a004-a3def6af6a4b；此前/tmp单次操作receipt已不存在，不能因文件消失推定动作未执行或重发。SSH别名本轮实际走备用地址192.168.1.100，SSH_CONNECTION为192.168.1.222到192.168.1.100，机载eth1；沿USB序列号发现can0，接口UP/1Mbps/ERROR-ACTIVE，累计TX为0（新启动/接口计数，不否定此前操作）。没有其他已知Piper控制进程。未改网络或服务。

被动读取2秒收到40条完整反馈，最新六关节[0,0,0,0,0,0]度，SDK末端[56.127,0,213.266]mm、欧拉[0,84.999,0]度；ctrl_mode=1、teach_status=0、arm_status=0、err_code=0、六关节使能true，夹爪宽度0mm/力矩0/使能false。证据after-user-homing-01.json，command_sent=false。与此前待机mode0不同，但不据此认定旧故障根因或操作性质。

已询问教程操作是六关节运动回到零姿态、重写六关节当前位置为零点，还是仅夹爪校零；不确定时提供教程链接/命令/按钮名。此前用户“空载完全合拢”确认已收到，但在其后新的硬件操作后不继续旧设零。checkpoint=WAIT_USER_HOMING_PROCEDURE_CLASSIFICATION。若只是回零运动，不据此要求重做相机外参；若重写关节零点，应核对新的反馈/模型物理对应，不能仅因为矩阵文件存在就继续旧轨迹，也不直接断言相机与基座的物理外参失效。原抓取目标保留，尚无抓取动作或通过结论。未修改产品代码/构建。


### 2026-11-13 ArmRobotUA仅回零运动后，CAN模式小开口复测仍失败且夹爪未使能

用户澄清：在另一台电脑使用ArmRobotUA让六关节运动回零姿态，没有重写关节零点；期间重启设备，机械臂从Go2换接该电脑，现已接回Go2。不能把这次操作当成重新标定关节零点，或据此要求重做固定相机外参。用户没有在ArmRobotUA里单独测试过夹爪。此前官方两步校零的第二步仍未执行，不补发。

用户明确授权在回零后再做一次相同10mm/0.2N·m夹爪复测，观察约10秒；不校零/清错/六关节动作/自动重发。实际使用与本机SHA256一致的原standalone入口，唯一目录/tmp/piper-bottle-open10-20261113-02，12秒SIGINT结束只读监测。远端日志gripper-84861e7z.jsonl，本机open10-retest02-runtime.jsonl。1次COMMAND_ATTEMPTED、22次WAIT_TARGET；全部宽度0mm、力矩0、使能false、ctrl_mode1、err_code0、完整新鲜反馈，六关节始终0。无SDK ERROR或Traceback；can0累计TX0→2，与一次参数查询+一次目标一致，但不证明夹爪驱动收到/执行。用户确认未张开，且现能自由开合；此前使能时不能自由开合。不能用此前“使能true但不动”概括当前状态。

随后被动采2秒CAN：0x2a1有401帧0100010000000000，0x2a8有400帧且全部0000000000000000，0x261有80帧00f000261f400000；该窗口无0x150/151/159/470/471控制报文。证据after-homing-passive-bus.json；只证明窗口内未见覆盖命令，不证明完全不存在其他控制源。SDK解析与原始报文一致，夹爪全零不能单独证明电机在线或机械零位校准正确，也不直接确诊供电/接线故障。

用户说明已接回Go2后，再被动读取2秒40条完整状态，USB序列0046002F5246570520323934、can0匹配既有身份，六关节0/使能true、ctrl_mode1/teach_status0/err_code0，夹爪0mm/力矩0/使能false。证据after-user-reconnect-01.json。当前无已知控制程序，只有只读tail诊断；piper-boot.service为active/exited，不重启或修改服务。读取到的服务时间戳与启动时间不同，不据此推定新根因。

已采更新场景rgbd04，三帧RGB-D/机器人窗口，7文件源hash验证通过；图像瓶子仍在近处、底部裁切，旧rgbd03候选坐标不可直接沿用为运动目标。当前暂停追加机械动作，建议下一步在ArmRobotUA的夹爪页面核对独立控制/驱动状态，先确定实际UI字段，再做官方软件对照；用户尚未执行该对照。SDK/固件/供电/末端通信等原因仍待区分，不继续盲目设零、加力、清错或刷固件。官方快速手册确认ArmRobotUA用于本体控制和外网反馈，说明六关节回零/模式转换与末端工具控制区别；其版本2024.09不作为当前UI/固件完全一致的保证。

checkpoint=WAIT_ARMROBOTUA_GRIPPER_DIAGNOSTIC_VIEW。全程无产品源码修改/部署/构建，没有抓取成功。原请求仍为Go2静止夹取矿泉水瓶；当前瓶底约7cm垫高，不算地面验收。所有已结束单次动作授权均不得自动复用。


### 2026-11-13 收到ArmRobotUA夹爪页截图：未使能、目标和扭矩均为0

用户提供本地截图，已实际查看，证据另存build/piper-bottle-ground-20261113/armrobotua-gripper-user-view.jpeg；原图文件名包含会话查询参数，不在文档复述。软件V1.4.1.250905_release，已连接，底部CAN/正常/MOVE J。夹爪目标间距0.000mm，扭矩输入0.000（UI标签N/m）；右侧实际距离0.000mm、扭矩0.000、状态码0x0，夹爪状态控件显示“失能”。左侧行程限制输入70mm，右侧设备反馈100mm，不能将输入框当已生效配置或将100mm当机械实测开口。

这些只说明截图当时没有有效开爪目标/使能状态，不能反推此前Go2的10mm/0.2N·m未发出，也不能认定零点故障。下一步给用户官方UI单独夹爪对照步骤：保留其他参数，先将扭矩数值设0.200，再设间距10.000mm，然后仅用夹爪状态控件切到使能一次，观察实际夹指、右侧间距和状态码；不点击顶部整臂使能、清错、设置零点或更改行程限制。界面改变数值可能立即下发，不把它说成仅编辑未执行；不在Go2同时发命令。若控件语义/界面与描述不同，先回传实际状态，不尝试猜测按钮。用户尚未报告此次UI测试结果，助手未发送新硬件命令。

按既有SDK状态位，0x40为使能位、0x80为回零位；健康使能时可能0x40或0xC0，不能将任一完整码当唯一通过标准，更不能单凭使能位认定抓取成功。checkpoint=WAIT_USER_ARMROBOTUA_SMALL_OPEN_OBSERVATION。仅证据与文档更新，无产品代码修改/构建或抓取动作。


### 2026-11-13 用户报告ArmRobotUA夹爪使能点击无反应

用户报告“在夹爪状态点击使能没有反应”。已确认官方软件也出现用户可见的无响应，但尚缺点击后的按钮文字、右侧距离/状态码及10mm/0.200输入是否保留，不能直接断言使能位仍为0或电机离线。之前Go2复测有新鲜0x2a8全零反馈；已重新核对本地SDK0.6.2定义，使能与回零分别为bit6、bit7，不能以无动作或全零反馈确诊校零故障。下一步只需点击后夹爪页面截图，以及电机配置页面已有的夹爪/末端在线、版本或故障显示（若界面提供），不保存参数、不再次使能/校零/清错/刷固件、不通电插拔。供电、末端通信、软件/固件配置均仍是待验证假设。助手本轮未发送硬件命令、未SSH、未更改产品代码。checkpoint=WAIT_ARMROBOTUA_POST_ENABLE_FEEDBACK。


### 2026-11-13 用户授权Go2接口低速小角度关节验证；当前CAN未启用

用户确认机械臂连接Go2上位机，要求先确认能通过接口做低速小角度关节动作验证，作为当前测试授权，不扩展为抓取或校零。实际SSH复核standalone三个文件SHA256与此前一致；USB序列0046002F5246570520323934匹配can0，但can0为DOWN/STOPPED、RX/TX均0且未配置bitrate。原inspect入口4秒内仅WAIT_IO，command_attempted=false；监测正常退出，证据joint-small-motion-preflight01.jsonl（远端inspect-zqh5hilp.jsonl）。piper-boot.service为active/exited、ExecMainStatus0，进程检查仅见只读tail与本次检查，没有已知动作客户端；未重启服务。sudo -n -l返回需要密码，不是自动审批拒绝。

当前没有新鲜关节姿态，尚未选定关节目标或发送任何模式/使能/运动命令；不使用历史零姿态构造目标。需用户在Go2上位机本地终端以sudo将can0配置1Mbps并启用，完成后继续被动读取，再按本轮授权完成一次小幅低速关节目标验证。不要把CAN链路恢复当运动测试通过；当前无需重启设备、改代码或重新构建。checkpoint=WAIT_CAN_UP_FOR_AUTHORIZED_SMALL_JOINT_TEST。


### 2026-11-13 CAN配置busy后确认USB重新枚举，当前仍DOWN

用户执行配置1Mbps与up的组合命令得到RTNETLINK Device or resource busy，之后提供ip详情为ifindex10/can0/DOWN/STOPPED且无bitrate。助手同期只读SSH确认相同状态，原inspect-v08z1ik9.jsonl仅WAIT_IO，未发送动作。journalctl内核日志显示21:44:03 can0 link becomes ready，21:44:22 USB device5断开（gs_usb shutdown err=-19），21:44:24同序列0046002F5246570520323934以device6重新枚举；前轮ifindex9变为10。不能把断开直接归因于用户插拔或硬件异常。Linux v5.10 drivers/net/can/dev.c can_changelink源码在IFF_UP时拒绝设置bittiming并返回EBUSY，符合先前busy的一种解释；组合命令实际哪一步失败未确定。下一步让用户在Go2终端逐条执行配置bitrate、up、读取状态，每步无报错才继续；不要求盲目down/up或重启设备，不清错/校零。低速小角度关节验证授权保留，仍未取得新鲜关节反馈或发送目标。checkpoint=WAIT_CAN_CONFIGURATION_STEP_RESULT。无产品代码修改。


### 2026-11-13 CAN恢复UP/1Mbps但整臂无反馈，版本查询也未获得响应

用户提供can0 UP/LOWER_UP/ERROR-ACTIVE、bitrate1000000。助手运行原inspect约4秒，所有关节/位姿/状态/夹爪反馈均缺失，command_attempted=false，证据joint-small-motion-preflight02.jsonl（远端inspect-4apwaok6.jsonl）；不设过滤器独立SocketCAN接收3.017秒共0帧，USB序列匹配0046002F5246570520323934。用户进一步确认本体已上电、指示灯亮、CAN线已连接。

为区分无周期数据与查询不应答，使用现有SDK0.6.2 SearchPiperFirmwareVersion一次，源代码核对仅发送0x4AF/0100000000000000，不含运动/使能/零点/参数修改。ConnectPort关闭piper_init及接收线程，持既有USB控制锁，唯一O_EXCL记录/tmp/piper-joint-check-firmware-20261113-01.json，未重发。随后无过滤接收3秒仍0帧，GetPiperFirmwareVersion=-1199（SDK无版本字符串的哨兵值，不是机械臂故障码）。调用前后can0 RX/TX计数均0、仍ERROR-ACTIVE、未见SDK发送错误；不能将SDK调用返回当总线成功发送或设备收到，不能认定该状态证明电气链路正常。证据joint-check-firmware-query01.log。

当前尚无六关节起始角，未选择目标、未发送任何关节模式/目标/使能，也未测试夹爪。不据此诊断夹爪零点或改固件。待核对适配器到本体实际端口接线及此前USB断开是否人为；本轮用户低速小角度验证授权保留，实际执行待链路恢复。未改产品代码/构建。checkpoint=WAIT_CAN_LINK_PHYSICAL_DIAGNOSIS。


### 2026-11-13 重启后CAN恢复；低速J6目标一次实际尝试仍无角度变化

用户重启Go2重新连接后再次设置bitrate遇busy。实际can0 ifindex7已UP/ERROR-ACTIVE/1Mbps，RX488196、TX1；开机服务日志ENABLING→READY，六关节全部使能，服务active/exited，boot_id=d8cba4f7-4dc8-4d25-9377-54ec78eb7d21。该次busy符合已启用接口不允许修改bitrate，不需要再down/up。原standalone三个文件SHA256与前轮一致，没有已知并发动作进程。新鲜被动反馈恢复：ctrl_mode0、arm_status0、err_code0、六关节使能true，稳定角度[0,-0.115,0,0,0,0]度，夹爪0mm且未使能。

用户此前低速小角度测试授权延续，宣布J6+3度/5%且其余保持；先沿原入口以[0,-0.115,0,0,0,3]、容差0.3度启动12秒监测。日志joints-wx5t3f0g.jsonl共11 WAIT_RANGE（J2目标超限）和12 WAIT_LIMITS，无COMMAND_ATTEMPTED，未发送模式或关节目标；用户称未观察到、可能角度太小，已澄清实际原因是发送前校验等待。独立只查询一次设备限位，0x473原始结果见joint-small-limits01.json：J1[-150,150]，J2[0,180]，J3[-170,0]，J4[-100,100]，J5[-70,70]，J6[-180,180]度；六个max_speed均0.3rad/s。不将J2当前-0.115度当零点损坏证据，不改限位或零点。

向用户明确说明J2目标调整到0（仅回调0.115度），J6仍+3度/5%，其他目标不变，随后原入口发送合法目标[0,0,0,0,0,3]一次。唯一请求记录joint-small-test02-request.json及远端持久receipt ~/.local/state/piper-standalone/joint-small-test-20261113-02.json。日志joints-ii43xh4q.jsonl：1 COMMAND_ATTEMPTED，22 WAIT_TARGET；后续ctrl_mode由0变1，arm_status/err_code0、六关节使能true，关节反馈始终[0,-0.115,0,0,0,0]度，J6没有趋近3度。没有SDK错误或Traceback；12秒SIGINT结束监测，timeout124并有EXITED，不表示物理停止或取消保留目标。没有回程、自动重发、夹爪、校零或清错。通过现有piper_go2.sh diagnostics打开诊断窗口；gnome-terminal退出0，伴dconf只读告警。所有请求/执行/运行日志已留本地build证据。

用户问是否再次零位校准，已说明当前证据不足以支持重设零点；先查控制时序与目标执行。官方SDK0.6.2 piper_ctrl_joint.py周期5ms发送MotionCtrl_2与JointCtrl，现有入口模式和目标只紧邻发送一次；模式切换时目标是否被接受是待验证假设，不是已确认根因，未擅自改周期或跑官方全幅demo。当前CAN模式反馈1，但小角度运动仍未通过；第二次用户物理观察未收到，不能混用第一次观察。无产品源码修改/构建，不再发送动作。本轮低速单次实际运动目标尝试已结束，后续动作次数需按用户授权范围另行明确。checkpoint=JOINT_SMALL_TEST_NO_MOTION_SEQUENCE_DIAGNOSIS。


### 2026-11-13 关节不动作排查：当前固件、状态及两端实际SDK编码核对

用户要求“进行排查”。本轮只做本地代码检查、官方资料核对和一次固件查询，未发送新运动/模式/使能命令。查询得到S-V1.8-2（原始版本片段保留于joint-diagnosis-firmware-state01.json），3秒0x2a1约601帧、各关节反馈约600帧，六关节低速状态各120帧；CAN/MOVE J原始0x2a1=0100010000000000，teach_status0、arm_status0、err_code0，六关节使能true，角度仍[0,-0.115,0,0,0,0]。夹爪0x2a8全零。未见该接收窗口内其他控制帧，不能证明永远无其他控制器。

本机与Go2均使用实际SDK0.6.2，禁止socket并替换物理发送器，以捕获的真实状态/限位驱动原Command.step；断言通过。目标[0,0,0,0,0,3]、速度5编码严格为0x151/0101050000000000，0x155/0000000000000000，0x156/0000000000000000，0x157/0000000000000bb8。再次step不会重发；单位和编码正确，但离线结果不证明前次四帧均成功上总线或目标被固件接受。证据joint-diagnosis-encoding-local.json和joint-diagnosis-encoding-go2.json。

明确待验证差异：原Command.step紧邻调用MotionCtrl_2和JointCtrl，没有等新鲜模式反馈；官方0.6.2 piper_ctrl_joint.py为5ms周期模式+目标。模式切换时单发目标被忽略是候选原因，尚未确诊，不把官方周期示例当必须200Hz或自动连发授权。下一项最小对照拟在模式已稳定CAN/MOVE J时，以相同[0,0,0,0,0,3]/5%再次单次目标，同步记录原始0x151/155/156/157和0x2a1/2a5/2a6/2a7，观察约10秒，不加角度/速度、不校零/清错/回程；前次单次测试已结束，按AGENTS次数边界请求新对照授权。

资料补正：官方最新V2接口明确V1.5-8及以后max_joint_spd原始值单位0.01rad/s，之前SDK0.6.2旧注释按0.001换算导致joint-small-limits01.json中的max_speed_rad_s=0.3仅为旧解码结果。当前S-V1.8-2、raw=300应按新说明为3rad/s；没有更改该参数，运动入口只使用限位，不使用此换算派生速度。官方Q&A另提示未配置夹爪行程可导致全零反馈，但现有夹爪行程曾读100mm，不能据此确认其为本机原因或自动修改行程。来源：https://github.com/agilexrobotics/piper_sdk/blob/0.6.2/asserts/Q%26A.MD 、https://github.com/agilexrobotics/piper_sdk/blob/master/assets/V2/INTERFACE_V2.MD 。无产品代码修改/构建，校零必要性和运动故障根因仍未证实。checkpoint=WAIT_MODE_STABLE_SINGLE_TARGET_COMPARISON_AUTHORIZATION。


### 2026-11-13 用户重设六关节零点后，SDK低速J6小幅往返验证通过

用户明确报告按教程摆好机械零位后重新设置六关节零点，并要求再次通过Piper_interface低速运动。新鲜被动读取稳定角度[0,0,0,0,0,0]，CAN模式1、六关节使能true、无故障，can0 UP/ERROR-ACTIVE/1Mbps；当前boot_id仍d8cba4f7-4dc8-4d25-9377-54ec78eb7d21，但USB曾重新枚举，ifindex10。不是只做运动回零，不能沿用校准前关节目标或假定旧运动学对应已验证。

新授权test03：先被动确认CAN/MOVE J模式已稳定，随后原standalone入口单次[0,0,0,0,0,3]、5%、容差0.3度，12秒监测。底层SDK0.6.2 C_PiperInterface_V2，MotionCtrl_2(1,1,5,0)，JointCtrl(0,0,0,0,0,3000)。joints-xtx3ls4u.jsonl：1 COMMAND_ATTEMPTED、22 AT_TARGET；J6从0到2.931度，其他关节0，状态/故障码0，保持约10秒。独立原始CAN0x2a7记录42次值变化，峰值2.948、最后2.931度；0x2a1运动状态0→1→0。该次用户没有确认正向实物观察，而是要求“重新进行尝试”。

据用户新要求，先再次读取J6=2.931度，再明确宣布test04为5%速度回0度、幅度约3度。原入口单次JointCtrl(0,0,0,0,0,0)，同模式设置，不动夹爪、不改零点、不发送下一段。用户明确回复“看到了反向转动”。joints-3g8fandw.jsonl到位状态及最终角度记录在joint-small-test04-runtime.jsonl，独立raw在joint-small-test04-can.json；末次所有角度误差均小于0.3度，断言通过。test03/04均退出监测timeout124且原程序EXITED，不将进程退出当物理停止。每次唯一持久receipt ~/.local/state/piper-standalone/joint-small-test-20261113-03/04.json，local request/execution/runtime/can证据保留。

原始监听器未捕获本地主机的0x151/155/156/157回显（control_frames为空），因此不声称已完成实际TX逐帧核验；SDK调用记录、设备新鲜角度/运动状态变化和test04现场观察共同支持当前小幅关节控制通过。本次仍是原单次SDK入口，未改成周期发送；校准和起始模式等条件同时改变，不能单独归因于校零或模式时序，也不能据此声称必须周期发送。原夹爪仍0mm/未使能，夹爪开合和瓶子抓取未验收。

用户询问具体接口：Go2侧/home/unitree/piper-standalone-20261111/piper.sh→standalone/piper_control.py，C_PiperInterface_V2通过SocketCAN；反馈GetArmJointMsgs/GetArmStatus，由现有被动CAN解析路径更新。本次SDK目标单位0.001度，3度=3000；模式参数5是百分比，非5度/秒。无产品代码修改、无部署或构建。checkpoint=JOINT_SMALL_MOTION_VERIFIED_AFTER_USER_ZERO_CALIBRATION。


### 2026-11-13 六关节校准后夹爪复测实际张开并保持，10mm目标尚有反馈偏差

用户明确要求“那么再进行夹爪尝试”。先被动确认CAN模式1、六关节角度全0/使能true/无故障，夹爪0mm/未使能；保留原入口与独立诊断窗口，沿原standalone只发送一次10mm/0.2N·m、观察约10秒，不清错/校零/六关节动作。底层GripperCtrl(10000,200,0x01,0x00)。唯一receipt ~/.local/state/piper-standalone/gripper-small-test-20261113-03.json；原日志gripper-uq7fq9de.jsonl，本地open10-retest03-request/execution/runtime/can/preflight文件已保存。

用户现场确认“张开约1cm并保持”。原始0x2a8从0逐步增大，最终8.3mm，状态码0x40（使能），六关节始终全0。请求10mm与最终8.3mm差1.7mm，超过原入口±1mm判据，因此不能把这次写成精确10mm到位；最终仍WAIT_TARGET。实际开爪与空载保持已有用户观察和新鲜反馈双证据，夹爪已不再是此前全零不使能的状态。未据反馈力矩推断瓶子夹持力，未自动增加力矩、重发、闭合或测试全开口。

行程反馈0x47e=6464070000000000（原始记录保留，当前最大行程100mm）；相比早期助力等级字段变化，但发生在用户外部操作期间，不能归因或自行更改。监听仍无本机控制报文回显，不把control_frames空列表解释为控制未发送。监测12秒timeout124/EXITED，仅退出软件，不自动失能或取消目标；夹爪保持使能。前后校准及模式/设备操作同时变化，不将恢复动作单独归因于校零。

当前确认C_PiperInterface_V2既可执行局部J6往返，也可使夹爪张开并空载保持；闭合、瓶子所需开口、负载保持和抓取路径尚未验收。无产品代码修改/部署/构建。checkpoint=GRIPPER_OPEN_AND_HOLD_OBSERVED_TARGET_ERROR_REMAINS。


### 2026-11-13 夹爪空载闭合验证通过，用户确认合拢并保持

用户要求“验证闭合”。原inspect确认当前8.3mm/使能true、CAN模式1、六关节0度/使能true/无故障后，宣布空载0mm/0.2N·m单次目标并提醒手和物体离开夹指。沿原standalone入口GripperCtrl(0,200,0x01,0x00)，不校零/清错/加力/六关节动作，监测12秒（约10秒目标反馈）。独立receipt ~/.local/state/piper-standalone/gripper-close-test-20261113-01.json，原日志gripper-zhu4e2x5.jsonl，本地close00-test01-request/execution/runtime/can/preflight均已保存。

日志1 COMMAND_ATTEMPTED、22 AT_TARGET，反馈从8.3mm降至0.9mm并保持，末次反馈力矩-0.207N·m、使能true、状态码0x40，六关节始终0度，arm_status/err_code0。原始0x2a8宽度变化一致。用户明确回复“已合拢并保持”。因此空载闭合及短时保持有现场与反馈双证据；0.9mm满足现有±1mm到位容差，不把它写成精确零位或物理间隙0.9mm，也不据此自行再次校零。

命令只发一次、无自动再开爪/重发；timeout124且EXITED仅代表监测结束，夹爪仍使能并保留目标。无源码修改/部署/构建。当前小开口与空载闭合均有实物观察，但64mm瓶子所需开口、负载夹持、重新置零后的模型/TCP对应与抓取路径未验证。checkpoint=GRIPPER_EMPTY_OPEN_CLOSE_HOLD_OBSERVED。


### 2026-11-14 下一阶段瓶宽开口验证：CAN当前关闭，未发送目标

用户要求“进行下一部分验证”。承接空载小开口/闭合成功，明确说明下一步计划一次75mm/0.2N·m空载开爪，以核对能否容纳约64mm瓶身；不夹瓶或移动六关节，提醒空出夹指运动范围。本轮核对分支go2-cooperative-task-layer/HEAD323d5c5及现有改动，原standalone三文件hash未变。SSH发现新boot_id=1ae23188-abe9-4b8e-8b3c-f53995340f84；can0 ifindex10 DOWN/STOPPED、无bitrate、RX/TX0，现无已知Piper动作进程。原inspect约3秒仅WAIT_IO，command_attempted=false，证据open75-20261114-preflight01.jsonl（远端inspect-ikvonw_d.jsonl）。

piper-boot服务active/exited，当前boot日志曾READY，但该历史服务状态不能证明当前重连接口已UP。sudo -n -l提示需要密码，无法自主完成特权接口配置；不是自动审批拒绝。需用户Go2本地终端逐步设置can0 1Mbps、up并查看状态，如busy则先查看当前状态而非强制down。尚未取得当前姿态/开口或新鲜行程，未发送任何运动或夹爪命令，不沿用前日状态当实测。75mm目标还须由原入口核对设备实时行程，配置恢复后在现有授权范围继续该次测试。无代码修改/构建/部署，checkpoint=WAIT_CAN_UP_FOR_BOTTLE_WIDTH_OPEN_TEST。


### 2026-11-14 75mm空载开口未到位；按用户要求只读确认六关节均为零

用户恢复CAN后核实can0 UP/1Mbps/ERROR-ACTIVE，原inspect新鲜反馈六关节全0/使能true/无故障，夹爪0mm/使能true。沿原standalone入口只发送一次75mm/0.2N·m目标（GripperCtrl(75000,200,1,0)），通过原设备行程检查，不发六关节动作。原日志gripper-9lk8l2tz.jsonl，本地open75-20261114-test01-request/execution/runtime及preflight02已保存；remote持久receipt ~/.local/state/piper-standalone/gripper-open75-20261114-01.json。运行1 COMMAND_ATTEMPTED、22 WAIT_TARGET，监测内开口0→12.8mm，反馈扭矩约0.2N·m，六关节始终全0。用户反馈“张开了，但宽度不足或无法判断”。监测12秒timeout124/EXITED，仅结束读取，目标未取消；没有自动加力/重发/关闭。

后续仅查询夹爪参数并读取2秒（400帧0x2a8），得到开口27.2mm、状态0x40、故障低6位0、0x47e=6464070000000000即当前行程100mm。证据open75-20261114-diagnosis01.json。说明12.8mm是原观测窗口内结果，不能认定此后不再移动或把12秒退出当停止；力矩限制/阻力仅为假设，未确诊。用户目视确认没有异物，并要求先查看各关节是否回零。

按最新指示只运行原inspect约3秒，5个完整新鲜快照均J1..J6=[0,0,0,0,0,0]度，六关节使能true、ctrl_mode1、arm_status0、err_code0；当前夹爪27.2mm/使能true，末次反馈扭矩0.22N·m。证据joint-zero-read-20261114-01.jsonl（远端inspect-bnm8270e.jsonl）。这确认当前反馈相对于用户重设零点均为0，未发回零或再次校准，也不独立证明物理机械零位准确。75mm大开口仍未达到；不能据27.2mm宣称能容纳64mm瓶身，原目标可能继续执行。无产品源码修改/部署/构建，文档diff检查通过。checkpoint=JOINTS_ZERO_CONFIRMED_WIDE_OPEN_NOT_REACHED。


### 2026-11-14 用户要求75mm复测，但USB重连后CAN再次未启用

用户要求“重新尝试”，已明确沿用75mm/0.2N·m一次开爪，不加力。实际仅执行原inspect约3秒，全部WAIT_IO且command_attempted=false，证据open75-20261114-retest02-preflight.jsonl（远端inspect-vbf1kgdf.jsonl）。can0 ifindex由10变11，当前DOWN/STOPPED、无bitrate、RX/TX0；boot_id仍1ae23188-abe9-4b8e-8b3c-f53995340f84。内核记录11:32:02 USB device6断开、11:34:38同序列0046002F5246570520323934以device7重新连接。sudo -n -l需要密码。

未发送本次夹爪目标、使能或六关节动作；没有把之前27.2mm当作当前实测。已询问此次断开是否主动拔插/换接，以区分操作和异常掉线；恢复接口需用户本地sudo逐步配置1Mbps/up并检查状态，不能把恢复通信当大开口通过。本次单次复测授权尚未消耗，接口恢复后继续。无代码修改、构建或部署。checkpoint=WAIT_CAN_UP_FOR_AUTHORIZED_OPEN75_RETEST。


### 2026-11-14 恢复CAN后75mm/0.2N·m复测未达目标，随后SSH不可用

用户说“继续”，can0 ifindex11已UP/1Mbps/ERROR-ACTIVE；新鲜inspect六关节均0度/使能true、CAN模式1、无故障，夹爪变为0mm/未使能（与断线前27.2mm/使能true不同，外部操作细节未知）。按前轮未执行的复测授权，原standalone入口只发送一次75mm/0.2N·m目标，不改零点/力矩/六关节。唯一receipt ~/.local/state/piper-standalone/gripper-open75-20261114-02.json；原日志gripper-xo7exz5u.jsonl，本地open75-20261114-test02-request/execution/runtime及retest02-preflight-restored已保存。

日志1 COMMAND_ATTEMPTED、22 WAIT_TARGET；反馈0→11.7mm，整个约10秒观察窗口随后保持11.7mm，末次反馈扭矩0.208N·m、使能true，六关节仍0，arm_status/err_code0，无SDK错误。timeout124/EXITED只结束监测，目标未被取消，未自动重发。已询问本次实物开口观察，尚待回答；前次用户未见异物不能当当前完整验收。两次同参数仅开小段、反馈力矩接近请求限制，低力矩不足以克服阻力是待验证假设，不是已确诊根因，未据此自行加力。

随后只读inspect约3秒返回日志路径inspect-elmylmvz.jsonl；取回该文件时SSH exit255，一次只读重试仍报Connection timed out during banner exchange / UNKNOWN port65535。本地没有取得该后续文件内容，不能报告后续开口或是否仍保持；当前状态未知。没有重启服务、改网络、使能、校零或自动重放。本次复测命令已执行，重连后不能重复使用它的单次授权。先前USB断开是否主动拔插也未获回复。无代码修改/构建/部署，checkpoint=OPEN75_RETEST_NOT_REACHED_SSH_UNAVAILABLE。


### 2026-11-14 第三次75mm开爪仍不足；按用户转向要求J6测试反馈到位

用户要求“再次尝试”。SSH已恢复，can0 ifindex12 UP/1Mbps/ERROR-ACTIVE，无已知并发控制进程；原inspect新鲜角度全0、使能true、无故障，夹爪9.5mm/使能true。发送一次相同75mm/0.2N·m目标，唯一receipt ~/.local/state/piper-standalone/gripper-open75-20261114-03.json，日志gripper-i9ywm32s.jsonl。本地open75-20261114-test03-request/execution/runtime/preflight保留。1 COMMAND_ATTEMPTED、22 WAIT_TARGET；夹爪9.5→10.1mm、反馈扭矩约0.208N·m，六关节未动，无SDK报错。用户回复“没有动作，先尝试控制机械臂关节吧”，将此视为未观察到明显开爪，不抹掉反馈0.6mm变化。

按用户新要求先做关节验证，明确宣布J6+3度、速度5%、其余保持、不再发送夹爪命令。新inspect-fgi8qyaq.jsonl核对关节全0、夹爪10.1mm；随后原入口单次目标[0,0,0,0,0,3]、容差0.3度，唯一receipt ~/.local/state/piper-standalone/joint-small-test-20261114-01.json，日志joints-5jg224yk.jsonl，本地joint-small-20261114-test01-request/execution/runtime/preflight保留。1 COMMAND_ATTEMPTED、22 AT_TARGET；J6到3.032度（误差0.032度），其他五轴0，保持约10秒，六轴使能true、arm_status/err_code0，夹爪仍10.1mm。用户本次关节实物观察尚待回复，不沿用前日观察当此次证明。

两次命令均监测12秒退出（timeout124且EXITED），不自动回程、重发、校零/清错或加力；当前J6目标3度保留，夹爪先前75mm目标也未取消。关节新鲜反馈支持当前关节控制链路可执行，夹爪大开口仍未通过，不将所有问题归于CAN链路，也不未经证实认定机械卡阻/力矩/零点根因。无产品代码修改/构建/部署。checkpoint=JOINT_FEEDBACK_PASS_GRIPPER_WIDE_OPEN_FAILED_WAIT_VISUAL。


### 2026-11-14 用户再次要求关节测试，J6反向返回实物与反馈均确认

用户在J6+3度测试观察询问中回复“再次尝试”。明确说明新的单次动作是5%速度从约3.032度返回0度，其他关节不变、不发夹爪命令。先inspect-493thcmd确认当前J6=3.032度、其他五轴0、夹爪10.1mm、全部使能、无故障，再沿原入口单次全0目标。唯一receipt ~/.local/state/piper-standalone/joint-small-test-20261114-02.json，日志joints-q0y_4nov.jsonl，本地joint-small-20261114-test02-request/execution/runtime/preflight保留。

用户明确“看到了反向转动”。日志1 COMMAND_ATTEMPTED、22 AT_TARGET，最后六关节[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]度，状态和故障码0，夹爪反馈11.0mm。因此当前J6小幅关节控制有反馈与实物观察双证据；不能据此声称全六轴轨迹验收或夹爪故障解决。只发送本次一个关节目标，未自动重复/回程或改零点/力矩。原75mm夹爪目标未取消，软件监测退出不代表驱动停止。无产品代码修改/构建/部署。checkpoint=JOINT_SMALL_RETURN_VERIFIED_GRIPPER_WIDE_OPEN_UNRESOLVED。


### 2026-11-14 再次夹爪验证变为未使能全零，等待两轮间操作信息

用户明确要求“再次进行夹爪验证”。宣布仍75mm/0.2N·m单次目标、不加力/校零/六关节动作。原inspect-0ubzk6pd新鲜反馈六关节全0/使能true、CAN模式1、无故障，但夹爪已变为0mm/未使能，区别于上次J6回程末次11mm/使能true；未确认外部期间操作，不能猜测根因。沿原入口执行一次，唯一receipt ~/.local/state/piper-standalone/gripper-open75-20261114-04.json，日志gripper-1lo7i534.jsonl，本地open75-20261114-test04-request/execution/runtime/preflight保存。

日志1 COMMAND_ATTEMPTED、22 WAIT_TARGET，全部夹爪0mm、力矩0、使能false；反馈新鲜完整、六关节均0/使能true、arm_status/err_code0，无SDK异常。用户选择“没有动作或出现异常”，尚未区分完全不动或实际有声音/抖动。12秒监测正常EXITED/timeout124，不发送停止、清错或失能。没有自动加力、再校零或重发。

已说明当前优先排查使能未生效，不能把此前0.2N·m下开口不足的候选解释直接套用。已询问上次成功J6回程到本次间是否断电、拔插、校零或ArmRobotUA夹爪操作，并要求若有异响/抖动补充。本次单次授权已执行完，尚无大开口或抓取通过。无产品代码修改/构建/部署。checkpoint=GRIPPER_ENABLE_NOT_EFFECTIVE_WAIT_INTERVENING_ACTION_DETAILS。


### 2026-11-14 按用户要求保持当前开口使能，夹爪仍未使能；用户澄清只设置六轴零点

用户表示进行过零位校准，并要求先使能确保已经使能。已解释SDK夹爪使能必须携带位置/力矩，按新鲜当前0mm开口作为保持目标、0.2N·m，一次GripperCtrl(0,200,1,0)，不沿用75mm开爪目标、不重设零点、不发六关节动作。原inspect-xjoy5_qg确认0mm/未使能、六关节全0/已使能/无故障。原standalone入口日志gripper-qcv8aqfo.jsonl，唯一receipt ~/.local/state/piper-standalone/gripper-enable-hold-20261114-01.json，本地gripper-enable-hold-20261114-01-request/execution/runtime/preflight保存。

结果1 COMMAND_ATTEMPTED、22 WAIT_TARGET，所有后续新鲜反馈夹爪仍0mm/力矩0/使能false，六关节均0/使能true、arm_status/err_code0。没有SDK报错，不把命令返回当使能成功。随后只接收2秒，0x2a8共400帧、最新0000000000000000；0x2a1共401帧、最新0100010000000000，与SDK状态一致，不是单纯界面缓存。证据gripper-enable-hold-20261114-01-raw-check.json。未清错、加力、重复使能或开爪。

用户澄清本次校准仅“六个关节的设置零点”，不是夹爪设置零点；不能把本次问题直接归因于夹爪校零操作，也不建议继续重复六轴校零。当前六轴控制已曾现场验证，但夹爪使能尚未生效；全零主控反馈不独立证明夹爪驱动在线或供电正常，下一步应核对末端驱动状态/连接及配置，不能以默认全零无故障作为硬件健康结论。此次目标和监测结束均不作为夹爪通过，未继续抓取。无产品代码修改/构建/部署。checkpoint=GRIPPER_ENABLE_HOLD_FAILED_RAW_CONFIRMED。


### 2026-11-14 用户要求重新开合验证，当前CAN再次DOWN，尚未发送动作

用户明确要求“重新进行夹爪开合验证”。已宣布一轮小幅空载开合：10mm/0.2N·m单次开爪，确认响应后再0mm闭合；使能未生效时停在该步记录，不连发、不动六关节。原inspect-ka20u617仅WAIT_IO、command_attempted=false，证据gripper-cycle-20261114-01-preflight.jsonl。can0已是ifindex14（此前12），当前DOWN/STOPPED、无bitrate；boot_id仍1ae23188-abe9-4b8e-8b3c-f53995340f84，sudo -n -l需要密码。接口已经重新创建，但本轮未获取内核事件，不确定是主动拔插还是异常。没有发送任何夹爪或六关节动作、使能或校零。需用户Go2本地sudo恢复1Mbps/UP，保持连接后继续已授权开合，无需重复授权。无代码修改/构建/部署。checkpoint=WAIT_CAN_UP_FOR_AUTHORIZED_GRIPPER_SMALL_CYCLE。


### 2026-11-14 用户指定20mm后改70mm，两次0.2N·m均未到位；准备单变量力矩对照

用户中断仅含只读inspect的检查，明确通信已恢复无需单独验证、当前0mm、要求直接张开20mm；遵从该要求不再额外预检通信，保留原入口自带身份/健康/行程检查。单次20mm/0.2N·m命令，receipt gripper-open20-20261114-01.json、原日志gripper-3myqats5.jsonl；1 COMMAND_ATTEMPTED、22 WAIT_TARGET，最后10.0mm/使能true/力矩约0.212N·m，六关节均0，无故障。所有证据open20-20261114-test01-*保留。原10mm开后闭合计划由用户新指令替代，未执行闭合。

用户回复“张开到70mm”，先澄清是新目标还是实测，用户明确“要求下一步把目标改为70mm”。于是只发送一次70mm/0.2N·m，不加力/校零/六关节动作。receipt gripper-open70-20261114-01.json，日志gripper-d185e890.jsonl；1 COMMAND_ATTEMPTED、22 WAIT_TARGET，全窗口10.0mm，末次力矩0.202N·m/使能true，无故障，六关节全0。用户确认“仍只有约1cm或明显不足”。不能把70mm当用户已观察到的开口。证据open70-20261114-test01-*。两次监测均timeout124/EXITED，不停止或取消保留目标。

连续增大位置目标未改善，启用状态下力矩反馈接近0.2N·m，低力矩不足以克服阻力是候选原因，尚不能排除机械/驱动问题。下一项拟只将力矩限制改0.5N·m，目标保持70mm，空载一次、观察约10秒；不继续加力、重复、校零、清错或发六关节目标。实际本机SDK0.6.2禁止socket、替身发送离线核对GripperCtrl(70000,500,1,0)编码0x159/0001117001f40100通过，证据open70-effort05-proposal-offline.json；没有向硬件发送该候选指令。提高力矩超出前次单次0.2参数范围，需用户明确授权后才能执行。无产品代码修改/构建/部署。checkpoint=WAIT_AUTHORIZATION_OPEN70_EFFORT05_COMPARISON。


### 2026-11-14 用户授权70mm/0.5N·m单变量对照，大开口实际张开并保持

用户明确“允许”一次70mm/0.5N·m空载开爪对照。直接沿原standalone入口执行、保留内置健康/实时行程检查，不再额外通信预检。GripperCtrl(70000,500,1,0)，唯一receipt ~/.local/state/piper-standalone/gripper-open70-effort05-20261114-01.json；原日志gripper-g6t9hjt9.jsonl，本地open70-effort05-20261114-test01-request/execution/runtime保留。1 COMMAND_ATTEMPTED、22 WAIT_TARGET，开口10→68mm并保持约10秒，末次反馈力矩0.523N·m、使能true、六关节始终0、arm_status/err_code0。用户确认“张开到约7cm并保持”，未报告异常。

大幅开爪与保持有现场及反馈双证据，但70mm目标与68mm反馈差2mm，超过原±1mm判据，不能改写成精确到位或修改容差掩盖。与前次同70mm/0.2N·m停在10mm相比，结果支持当前0.2N·m限制不足以克服开爪阻力；这不是机构健康/零点精度完整验收，也不能归纳解释之前夹爪未使能全零的另一状态。仅把力矩限制这一变量从0.2改0.5，不再继续加力或自动重发/闭合。

观察到的68mm与约64mm瓶宽仅差4mm总余量，实际夹指净距、瓶身定位误差/接近路径仍需核实。此次0.5N·m是空载开爪测试，不当作矿泉水瓶闭合夹持参数；负载保持、抬升、TCP与六关节零点重设后的运动学对应仍未验证。timeout124/EXITED仅停止监测，夹爪仍使能保留目标，无自动失能/回程。无产品源码修改/构建/部署，文档diff检查通过。checkpoint=WIDE_GRIPPER_OPEN_OBSERVED_AT_EFFORT05_TARGET_ERROR_REMAINS。


### 2026-11-14 用户确认准备流程并切换蓝色积木，更新视觉及离线端点可达性

用户现场总结：ArmRobotUX中完成机械臂零位校准和使能、单独使能夹爪、设置合适力矩后可控制。将其记录为本次现场准备流程，不推广成每次启动均需重新设零，也不把此前所有异常归于单一根因。本轮未发送校零、使能、夹爪或六关节命令。原inspect-xvgfaccp及本轮相机配对被动反馈均显示六轴0度且已使能、夹爪68mm且已使能，arm_status/err_code0。

用户将目标改为实心蓝色长方体积木，约高10cm、宽6cm、厚4cm，先垫高10cm，再额外垫高8cm，累计约18cm；不是地面直接抓取验收。使用既有handeye.capture.Reader/paired_feedback被动接收CAN，RealSense435序列号151222077264、640x480/15fps RGB和深度，分别抓取3帧。证据build/piper-bottle-ground-20261113/blue-block-rgbd01与blue-block-rgbd02，各7个原始文件SHA256核对通过；首次scp目录点路径被拒后改显式文件名传输，未重新采样或发动作。初次积木下部裁切，额外垫高后的第二组完整看到积木轮廓和支撑物。

blue-block-rgbd02-geometry.json用手选正面四角、内部对齐深度平面、原manual04外参做候选计算：可见正面中心约基座[0.6034,0.0975,-0.0919]m，正面宽约4.24cm、高约11.72cm；三帧内部平面残差约0.36mm不等于绝对精度。宽度更接近用户4cm短边，隐藏深度暂保留4/6cm两个假设，未把任一中心作为执行目标；已询问正面尺寸方向和实际法兰端面至夹指接触中心沿伸出方向距离。原瓶子圆柱中心算法不再适用。

blue-block-rgbd02-candidate-ik.json仅离线候选：沿用官方候选URDF和未实测旧TCP135mm，夹爪横向沿可见正面宽度方向，向下30/45/60度，分别检查退后80mm预抓端点及抓取端点，两种隐藏深度共12项；30/45度的8项端点均找到限位内数值解，60度预抓2项有解、抓取2项位置误差7.78/15.27mm不满足1mm/1度数值判据。不表示直线/关节插补路径或自碰撞、Go2腿/护架/支撑物碰撞已验证；实际TCP、校零后物理FK及外参绝对精度仍需核实，不发布这些关节角到硬件。当前68mm开口对4cm夹取方向余量优于6cm，但实物净距、闭合力矩、带载保持和抬升仍未验证。0.5N·m仅有既往空载开爪成功证据。

本轮没有产品源码修改、构建、部署或真机运动；checkpoint更新为BLUE_BLOCK_SCENE_LOCALIZED_WAIT_TCP_AND_FACE_DIMENSIONS。


### 2026-11-14 用户确认积木正面6cm、前后4cm，尺寸差异待核实

用户明确正对相机宽约6cm、前后深约4cm，撤销之前根据视觉宽4.24cm猜测正面为4cm短边的解释。此前端点IK的夹指闭合方向沿可见正面宽度，按用户尺寸实际为6cm；不能将原IK称为夹4cm方向，换夹取方向需重算。原geometry及candidate-ik保留历史数值，新增execution_eligible=false和尺寸纠正说明。当前最近68mm反馈与60mm正面宽仅差8mm总余量，尚未测实物净开口。

本轮仅离线核对已有blue-block-rgbd02三帧：按原始深度内参反投影、depth_to_color列优先旋转/平移，再投影RGB内部区域，每帧约8060个点。原始点变换至彩色坐标Z中位约0.18663m，与SDK对齐深度中位0.186m差约0.67mm，不能解释4.24cm与6cm约29%的尺寸差。证据blue-block-rgbd02-depth-crosscheck.json；此一致性不证明近距离深度绝对准确，也不证明用户尺寸有误，未按比例强行缩放或更改外参。需尺量复核正面宽高，并补充实际法兰至夹指接触中心TCP。尚无新运动/硬件采样/参数修改，未构建或部署；文档diff检查通过。


### 2026-11-14 用户尺量修正积木正面4.2cm、厚2cm，更新中心候选

用户重新测量明确正面约4.2cm、厚约2cm，替代此前6/4cm数据。正面宽与已存三帧深度估计4.24cm相符，前轮尺寸矛盾按此次实测得到解释；这不独立验证外参绝对精度或全部深度范围。本轮未修改深度比例、相机内参或外参。

沿已拟合可见正面内法向偏移厚度一半10mm，得到候选积木中心基座坐标约[0.6133,0.0971,-0.0932]m，水平半径约0.6209m；三帧结果保存build/piper-bottle-ground-20261113/blue-block-rgbd02-corrected-dimensions.json。沿可见正面宽度方向闭合对应实际42mm；最近68mm开口反馈给出总名义余量26mm、居中每侧13mm，不等于实测净开口。此前按隐藏深度4/6cm算的IK只保留历史结果，不直接用作本次2cm厚度目标。

实际法兰端面至夹指接触中心TCP仍未收到，不能直接使用旧135mm工具长度。后续需按测量TCP重算接近点和整条路径，并核实物理标定、碰撞及带载夹持。当前高度仅有此前用户约10cm和视觉约11.7cm估计，未新测。本轮仅更新离线证据和交接，无硬件读取、运动或产品源码修改，无构建/部署；文档diff检查通过。checkpoint=BLUE_BLOCK_DIMENSIONS_REMEASURED_CENTER_UPDATED_WAIT_TCP。


### 2026-11-14 用户暂定TCP9cm，完成离线接近采样与零位展开诊断

用户表示法兰到夹指接触区域中心距离“暂且算作9cm”。接受为本轮显式暂定参数，不称为实测TCP，不改永久外参/产品配置，也不继续要求先精确测TCP才做离线工作。保持积木正面42mm、厚20mm及最近配对图像，采用局部+Z轴向90mm工具偏移。

证据blue-block-tcp90-candidate-path.json：沿可见正面宽度闭合，相对积木表面向下0/15/30/45/60度，分别从夹持中心沿接近轴后退80mm至中心，每10mm一个点，共45个IK采样；0/15/30/45度各9点均满足位置1mm/旋转1度数值判据，60度最终点约15.02mm/3.02度不通过。30度/45度最大相邻关节变化分别约7.57/5.31度；采样有解不证明连续轨迹、奇异性、动力学或碰撞通过。

blue-block-tcp90-zero-to-pregrasp-diagnostics.json针对30/45度，各201点假设同步关节线性插补，从最近记录全0角至预抓端点，仅检查FK轨迹范围。45度候选预抓TCP基座[0.564612,0.100087,-0.029761]m，法兰[0.509862,0.103434,0.041592]m；候选关节[13.232,145.777,-103.033,-33.055,18.762,43.054]度。J2需约146度展开，不能当作已验证J6±3度小动作的简单重复，也不直接一次下发。假设展开中TCP最高约0.294m、最低-0.0298m，腕部/法兰原点最低约0.0416m；原点不是连杆/夹爪包络，且此插补不保证等同SDK真实轨迹。30度候选法兰终点高度约0.0102m，45度较高但这不是安全通过判据。

还缺本机机械臂、Go2前腿/护架及目标支撑物的实际相对空间信息，需现场侧面视图/空间核对后准备分段非接触验证；未批准或执行本次完整路径。9cm误差会沿工具轴直接影响夹持位置，暂定值的第一步应为非接触几何核对而非直接闭合抓取。本轮无硬件采样或运动、无产品代码修改/构建/部署，文档diff检查通过。checkpoint=BLUE_BLOCK_TCP90_PROVISIONAL_IK_SAMPLED_WAIT_CLEARANCE_REVIEW。


### 2026-11-14 用户授权有人看管实际尝试，5段低速展开已到预抓位

用户明确“不要过于严谨和保守。如果有可用姿态就进行尝试，有人看管”，因此不再等待侧面照片，采用既有45度方案、暂定TCP90mm、5%速度现场分段尝试。新鲜原入口被动inspect-sq3g7w0j确认六轴全0、已使能、夹爪68mm已使能、故障0。按同一预抓关节向量比例，先J2=10度，再40/80/120度，最后完整预抓J2约145.777度；每段沿standalone piper.sh joints单次发送，入口保留实时设备限位/健康检查与锁，不重发、不自动使能、不校零/清错、不发夹爪命令。用户逐段明确前四段平稳且无干涉、可以继续。

唯一远端receipt ~/.local/state/piper-standalone/blue-block-approach-20261114-01.json至05.json；日志依次joints-_gkm_vap.jsonl、joints-geb36vq5.jsonl、joints-kh2kagwd.jsonl、joints-drwvxe9w.jsonl、joints-l5ojfqgv.jsonl。本地build/piper-bottle-ground-20261113/blue-block-approach01至05的request/execution/runtime完整保存，summary汇总。每段1 COMMAND_ATTEMPTED，末段28 AT_TARGET；五段末次最大关节误差均<0.3度。最后角度[13.232,145.713,-103.057,-33.089,18.774,43.127]度，SDK法兰[510.081,103.515,42.361]mm，夹爪68mm/已使能/力矩反馈约0.518N·m，arm_status/err_code0。第一二段URDF对SDK数值位置差0.076/0.082mm，只是数值一致性不代表物理标定通过。

预抓位置为沿接近轴距候选积木中心80mm，尚未执行最后接近、闭合或抬升。第五段实物对准与无接触观察待回复，正在额外抓取有限3帧配对RGBD核对。监测均timeout124并EXITED，只结束日志监测，不停止、取消目标或失能，当前保持预抓姿态。无产品代码修改/构建/部署，证据/交接按追加更新。


### 2026-11-14 预抓实际碰倒积木，已回退；用户最终指定更水平夹取

用户第五段实物观察：夹爪把积木碰倒，已重新立起，朝向不适合。更正上一节仅反馈到位：本次预抓物理验证失败，不能报告无接触成功。立即停止原后续接近；保持开爪，以5%发送一次上一停靠点关节目标[10.892,120,-84.814,-27.209,15.444,35.441]附近（精确目标见retreat01-request），无自动闭合/回零。blue-block-retreat01-request/execution/runtime保存；末次[10.859,119.963,-84.804,-27.23,15.477,35.448]度，SDK法兰[461.028,77.13,178.689]mm，夹爪68.4mm已使能，无故障，反馈到位。用户未明确单独回复回退观察，不将其写为实物确认。

预抓后与回退后各有限3帧RGBD分别blue-block-pregrasp-rgbd01、blue-block-after-retreat-rgbd01，各7源文件hash验证通过。用户重立后积木在画面位置明显改变，旧中心不再使用。9cm暂定TCP的预抓预测投影与可见夹指区域不符，证据blue-block-pregrasp-tcp-projection-diagnostic.json；黑色夹指部分深度缺失，不据此定量标定TCP或直接改外参。实际TCP、校零后外参对应、目标变化等根因尚未分离，不认为“有逆解/反馈到位”证明物理坐标准确。

用户先回答需更竖直，随后明确纠正为“更接近水平方向夹取”，最新纠正优先。竖直方案仅离线计算blue-block-upright-at-retreat-plan.json，未发送任何竖直调整；最终端点还有5.44mm/3.79度残差，保留为未采用证据。重新求水平姿态，目标法兰保持回退位置[0.460996,0.077075,0.178601]m，工具伸出轴落在基座XY平面、约向前偏左9.49度。候选关节[9.492,122.380,-72.276,0,-45.108,0]度，数值位置/朝向残差均小于判据。证据blue-block-horizontal-at-retreat-plan.json；对9/13.5/18/25cm假设轴向长度做关节插补尖端高度敏感性检查，不当作机构碰撞认证。按用户已授权有人看管试验，宣布分两半、5%调整方向，先执行horizontal01；不向积木下探、不发夹爪命令，等待本段反馈与用户观察。


### 2026-11-14 半程水平调整后按现场观察对正，等待前后偏差方向

horizontal01第一半段反馈到位，末次关节[10.181,121.156,-78.548,-13.628,-14.819,17.756]度、SDK法兰[466.24,89.309,173.338]mm、夹爪68.4mm已使能、故障0。用户说方向改善但位置偏左、容易碰倒，故原水平调整第二半段未执行。blue-block-half-horizontal-rgbd01三帧/7源文件hash核对通过，新图确认夹指空隙在积木左侧、画面右夹指靠近积木左上缘；不能直接下降。回退后目标重定位blue-block-after-retreat-geometry.json给出候选中心[0.6165,0.0336,-0.0936]m，宽约41.75mm，高约117.5mm，相比原候选横向变动约64mm；位置改变来自用户重立及碰撞后现场状态，不自动修正外参。

在已有看管试验授权内明确宣布固定朝向先抬20mm、再沿基座-Y（画面右）20mm，5%执行，两阶段分别单次原入口命令，第一阶段反馈到位且无故障才发送第二阶段；证据blue-block-lift-right-plan和lift-right01/02-request/execution/runtime。用户确认仍偏左但未接触，随后固定高度/朝向再次右移20mm（right-extra01-plan/request/execution/runtime），用户确认“基本对准，悬空未接触”。两次右移累计约40mm，尚不将视觉对正当作绝对标定校正。

随后明确宣布只下降20mm，保持前后和朝向、开爪；blue-block-lower01-request/execution/runtime，单次命令、21 AT_TARGET，最后关节[4.354,120.301,-76.970,-31.053,-17.188,32.186]度，SDK法兰[466.208,49.416,173.237]mm、RPY[166.144,66.064,178.799]度，夹爪68.4mm已使能，arm_status/err_code0。用户回答“左右对准，但前后位置没对准”，尚未明确本次是否发生接触，不能把该选项解读成无接触实证。现在保持位置/开口，已询问积木相对夹指更远需前伸，还是夹指伸过需回收，以及约差几厘米；不猜测方向继续推进。

本节所有动作原入口各只发一次，5%速度，未发夹爪闭合/增力/校零或Go2底盘动作。timeout124/EXITED只结束监测，不取消保持目标。无产品代码修改/构建/部署，文档diff检查通过。checkpoint=BLUE_BLOCK_LATERAL_ALIGNMENT_CONFIRMED_WAIT_FORE_AFT_DIRECTION。


### 2026-11-14 用户指导前伸和下调后允许轻夹，0.2N·m闭合仍不足

用户明确前后需要“向前伸”。新鲜inspect-9afm7dcl确认此前关节/68.4mm开口保持、无故障后，保持高度和朝向，沿基座+X10mm、5%单次forward01，反馈到位，SDK法兰约[476.005,49.782,173.260]mm。用户随后两次分别要求“向下一点”“还需要下降一点”，分别执行lower02与lower03每次-Z10mm，5%，保持前后/朝向/开爪，均单次且反馈到位。末次关节[4.296,124.232,-78.909,-28.320,-19.013,29.113]度，SDK法兰[476.249,49.884,153.501]mm，RPY[166.373,66.138,179.126]度。用户最后明确“位置合适，可以轻夹”。

按用户许可保持六关节，单次夹爪目标36mm、限制0.2N·m，观察约10秒；receipt blue-block-grasp-light-20261114-01.json，原日志及request/execution/runtime保存在blue-block-grasp-light01-*。1 COMMAND_ATTEMPTED、22 WAIT_TARGET；反馈68.4→67.9mm，末次扭矩-0.216N·m（闭合方向），夹爪使能true、六关节不变、arm_status/err_code0。尚未形成明显闭合，更不能当夹持成功；已询问现场两指是否贴住、未合上或推动积木，等待答复，不抬升、不自动加力。前伸及下调证据blue-block-forward01-preflight/request/execution/runtime、blue-block-lower02/03-request/execution/runtime完整保留。

所有原入口命令各只发送一次，监测timeout124/EXITED不取消保持目标；当前夹爪保留36mm/0.2N·m闭合目标。无产品代码修改/构建/部署，文档diff检查通过。checkpoint=BLUE_BLOCK_ALIGNED_LIGHT_CLOSE02_INSUFFICIENT_WAIT_VISUAL。


### 2026-11-14 有人看管、人工微调后的积木夹持及20mm抬升成功

用户确认0.2N·m下“没有合上”。在本轮已授权有人看管抓取、使用合适力矩范围内，明确宣布目标仍36mm、力矩改0.5N·m，仅一次闭合、不动六轴/不立即抬升。blue-block-grasp-effort05-01-request/execution/runtime保存；1 COMMAND_ATTEMPTED、22 WAIT_TARGET，反馈67.9→52.8mm、末次扭矩-0.523N·m、使能true、故障0。用户确认“两侧已贴住并保持，积木未偏移”。有物体时目标36mm未达到不独立等于夹持失败，仍保留原位置状态WAIT_TARGET，不修改判据或伪造AT_TARGET。

随后在抓取任务范围内宣布保持夹持/朝向，以5%速度沿基座+Z20mm，单次loaded-lift01，receipt blue-block-loaded-lift-20261114-01.json，request/execution/runtime完整保存；1 COMMAND_ATTEMPTED、21 AT_TARGET。SDK法兰由[476.249,49.884,153.501]mm到[476.309,49.859,173.087]mm，即[+0.060,-0.025,+19.586]mm；末次关节[4.296,122.039,-80.152,-34.053,-15.987,35.113]度，夹爪52.8mm保持、扭矩-0.525N·m、使能true、arm_status/err_code0。用户明确“已离开支撑面，悬空稳定保持”，因此本次有人看管、人工指导微调后的积木夹持和小幅抬升有实物与反馈双证据。不是仅用SDK位姿或夹爪误差判定成功。

结果汇总build/piper-bottle-ground-20261113/blue-block-supervised-grasp-result.json，checkpoint=SUPERVISED_BLUE_BLOCK_GRASP_AND_SMALL_LIFT_CONFIRMED。当前保持36mm/0.5N·m夹持目标及抬升后的六关节目标，未追加闭合、放回/松爪、回零或失能。限定结果：物体垫高约18cm，不是直接地面抓取；此前计算预抓碰倒积木，经过现场逐步修正才成功，不能称自主视觉抓取通过。90mmTCP仍暂定，物理外参/校零对应仍待验证；未测试放置、搬运或长时间负载保持。无产品代码修改/构建/部署。正在额外保存一次有限RGBD静止样本作为保持证据，采集不发动作。

保持样本blue-block-grasp-hold-rgbd01已完成：1帧RGB+深度+metadata共3源文件SHA256校验通过，配对被动反馈仍52.8mm开口、使能true、故障0；未发送动作。文档diff检查通过。


### 2026-11-14 按用户要求直接释放积木并返回初始六关节零位

用户明确“直接松开积木并回到原位，不用担心积木掉落”。解释并执行先开爪、再5%分段运动回本次抓取前六关节全0姿态，未重新设置任何零点。开爪单次70mm/0.5N·m，blue-block-release01-request/execution/runtime保存，反馈52.8→68.1mm，用户明确“积木已脱离夹指”；目标差1.9mm仍保留WAIT_TARGET，释放成功依据用户实物确认，未修改到位容差。

物体释放后，先向后上方收回到原展开J2=80度停靠点，再40度、10度和全0，共4段5%目标。blue-block-return-plan.json保存候选端点及简化FK高度检查，不声称完整碰撞认证；每段原standalone入口仅发一次，前段新鲜AT_TARGET/无故障后才发送后段。每段独立receipt blue-block-return-20261114-01.json至04.json，request/execution/runtime保存为blue-block-return01至04-*。最终新鲜反馈六关节[0,0,0,0,0,0]度、SDK法兰[56.127,0,213.266]mm、RPY[0,84.999,0]度，夹爪68.1mm仍张开且已使能，六轴均使能，motion_status/arm_status/err_code0。用户本轮要求已完成，不额外闭爪/失能/再抓取。

汇总blue-block-release-return-result.json，checkpoint=RELEASE_CONFIRMED_AND_INITIAL_JOINT_POSE_REACHED。此前成功的人工指导积木夹持/抬升记录继续保留，不因释放覆盖历史，也不宣称自主抓取或TCP标定通过。本轮无产品代码修改/构建/部署，文档diff检查通过。


### 2026-11-14 独立自主视觉抓取方案已交接，尚未实施

用户明确要求把视觉与控制接通，源码放VLM_Nav内、先作为独立功能，不接导航，在另一个对话实施；本轮仅交付[执行方案](../superpowers/plans/2026-11-14-piper-standalone-visual-grasp.md)。拟采用蓝色RGB-D检测与成功抓取参考的相对平移修正，固定成功朝向，不把暂定90mm或旧135mm当TCP。方案含真实证据来源、迁入仓库的参考/测试资产、4个功能模块、独立入口、6项实施任务、构建安装与现场验收步骤，以及可粘贴到执行对话的启动说明。未新增产品代码、未构建/部署/启动现场试验。

方案前只读核实机载 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`：Python3.8.10、NumPy1.17.4、SciPy1.3.3、OpenCV4.12.0、pyrealsense2 2.55.1.6486、piper-sdk0.6.2、python-can4.5.0；Rotation没有from_matrix/as_matrix，只有from_dcm/as_dcm，least_squares可用，已写入兼容要求。检查仅导入数学/图像库与读取包版本，未连接CAN或相机。首次shell引号错误在本机解析失败，改用SSH标准输入脚本后成功，无硬件命令。

最近真机状态仍以此前release/return结果为准：用户已确认积木释放，六轴反馈全0，夹爪68.1mm张开并使能；本轮未重新读真机反馈，不能当成当前持续状态保证。文档链接与diff检查完成，等待用户在另一个对话执行计划，不自动提交。

### 2026-11-14 独立蓝色积木视觉抓取已实现，等待最终安装核对

本轮用户明确授权按[独立视觉抓取计划](../superpowers/plans/2026-11-14-piper-standalone-visual-grasp.md)实施源码、测试、构建和运行文档，不自动提交或启动真机。分支仍为go2-cooperative-task-layer，HEAD323d5c52737477962a874ae012db117be5a93f34；保存任务开始时全部已有改动的SHA256，除本任务必要的CMake和文档追加外不覆盖既有工作。

新增 `tools/piper_vision_grasp/` 四模块、`scripts/piper_vision_grasp.sh`、四份配置与必要来源、真实RGB-D fixtures及定向测试。入口提供check/observe/preview/run/diagnostics；`preview --input`只读存档且不连接设备，真实run不接受离线输入。功能不接导航/ROS/DDS/旧Agent，不调用VLM/SAM。采用HSV连通域、完整可见面内部深度拟合及三帧中位数，固定成功夹取朝向做视觉相对平移；不用90/135mm TCP。原manual04外参不变，物理验证仍NOT_PERFORMED。

来源从build/piper-bottle-ground-20261113与候选URDF按hash提取，参考仅使用after-retreat闭合前图像，half-horizontal交叉核对；hold图像仅作抬升后不足证据测试。真实样本约3.48MB；自动检测参考宽44.75mm、高118.23mm，同位置中心差约0.11mm，均非绝对精度。H95..130/S≥160/V≥30由样本验证，原候选阈值纳入过多背景青色。参考关节与源记录逐项核对，FK与同期SDK位置做数值一致性检查。

离线路径发现从展开姿态直接进行朝向插值会选到受J4限位约束的另一分支，保留1mm/1°判据，改为从成功姿态向上建立局部分支，再连接空载关节过渡。规划记录端点、相邻采样和假设插补轨迹；未做物理碰撞认证。运行保留同USB控制锁、固件限位/反馈过滤、5%关节速度和0.5N·m夹爪限制，无自动六轴使能/校零/清错/加力/松爪。run/cycle/stage尝试持久化后才写SDK；重复ID/部分发送不重放。反馈中断或短故障会重新累计稳定时间，恢复后重新定位，目标变化进入WAIT_REPLAN，不继续旧目标。

运行手册：[piper_vision_grasp_runbook.md](piper_vision_grasp_runbook.md)；完整构建命令仅追加主[go2_runbook](go2_runbook.md)。独立构建输出 `build/piper-vision-grasp-20261114/overlay`，源码发现含根包和cooperative_ws/src。首次三包构建成功（27.1s），安装及最终用例数见下一条交付记录。导航默认overlay和launch无须改变，新入口通过相对路径使用自己的安装模块/配置。

开发机原系统Python缺硬件依赖，既有Piper venv缺视觉计算依赖；仅新建build/piper-vision-grasp-20261114/check-venv，复用系统数值库并安装piper-sdk0.6.2/python-can4.5.0/pyrealsense2 2.55.1.6486，用于离线check，不复制给Go2。机载候选解释器与部署目录本轮未SSH核实。未提交、部署、运行ROS/相机/CAN或发送任何运动；本轮不能报告现场自主抓取通过。

### 2026-11-14 独立视觉抓取离线交付完成

最终新增定向pytest **56项通过**，原Piper boot/gripper基线 **66项通过**。新输出内colcon定向ctest **3个包装均通过**，`colcon test-result`显示125条为122个实际pytest用例加3个包装，不是125个新增用例。三包首次完整构建与最终增量构建退出0；仅原有CMake变量/PCL发现提示，无构建错误。Python3.8语法检查、旧SciPy旋转API兼容用例、Bash语法、新文档链接/新增文件空白和git diff --check均通过。

最终安装路径 `build/piper-vision-grasp-20261114/overlay/install/vlm_nav/share/vlm_nav`；从这里运行check→CHECK_OK、离线preview→PREVIEW_READY，CAN发送0，事件记录的模块/config路径均为本次安装。23个运行源码/config/helper及来源文件逐字节与源码一致，证据 `build/piper-vision-grasp-20261114/evidence/installed-source-comparison.json`。安装入口独立查找相对路径，没有修改CoVLM_env、Cooperative_env或导航launch，未用外部Piper源码补依赖。

最小分发目录 `build/piper-vision-grasp-20261114/package/`，26个文件含SHA256SUMS、SOURCE_MANIFEST和使用说明；归档 `build/piper-vision-grasp-20261114/piper-vision-grasp-20261114.tar.gz`，SHA256=`3ade7db7b14a394ec5438fdc14faab89891df582896d5d1a7586b15af81b1073`，43777字节。分发目录独立check也通过，全部清单哈希一致；未分发到远端。归档不含测试图像、虚拟环境或导航栈；运行参考与必要原始来源已包含，真实样本仍在Git可跟踪的test/fixtures中。

本轮复核修复了短暂故障跨越稳定窗口、目录持久化不足、同目录第二次周期覆盖和采样配置未生效等问题，并添加回归。没有自动重发、松爪或回零；部分SDK发送进入COMMAND_UNCERTAIN。原始已修改文件的本轮变化仅CMakeLists与两份runbook/交接的必要追加，其余初始哈希保持。

未执行：全库ROS/硬件测试、机载Python3.8实际运行与依赖复核、SSH/部署、真实相机/CAN、真机自主抓取或多位置重复性。碰撞、真实SDK插补、夹指遮挡下可见性与物理标定仍待现场验证；不能把离线/fake通过写成真机通过。后续按独立runbook核实机载目录/环境、完成被动observe与preview，再在新的明确现场授权内run。checkpoint=PIPER_VISION_GRASP_OFFLINE_AND_INSTALL_VERIFIED_NO_DEPLOYMENT。

### 2026-11-14 用户机载observe的RealSense全局context类型错误已做离线修复

用户已按运行步骤自行部署至 `/home/unitree/piper-vision-grasp-20261114`，提供诊断确认check为CHECK_OK，机载解释器 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`，包版本与计划中的Python3.8环境一致。随后observe不退出，用户日志反复WAIT_VISION，reason=`global config-file/context: expecting an object; got ""`，command_attempted=false。这是用户提供的现场证据；开发端未SSH或连接设备。

核对本机代码与历史交接4696节发现：原手眼capture使用 `rs.context('{"inherit":false}')`，历史记录已确认机载 `.realsense-config.json` 的context为空字符串；新独立入口遗漏该适配，默认 `rs.pipeline()` 继承全局配置。已在capture_rgbd显式创建独立context并传给pipeline，保持其他参数/流程不变，不修改远端全局文件。当前远端文件本轮未重新读取，日志与历史已知问题一致；不能声称机载新取图已通过。

新增1项回归先复现同样错误，再修复通过；原2项相机fake同步适配context构造。视觉测试57项（上一版56+本轮新增1）、boot/gripper基线66项全部通过；三包新输出构建26.8秒成功。新输出 `build/piper-vision-context-20261114/overlay`，旧输出不改；定向ctest3包装通过，colcon126条为123个pytest+3包装。安装入口check→CHECK_OK、离线样本preview→PREVIEW_READY；模块/config加载路径在新安装下。Python3.8 AST、Bash语法、git diff --check及补丁本地应用的全清单哈希校验通过。

补丁 `build/piper-vision-context-20261114/piper-vision-context-fix-20261114.tar.gz` 仅包含vision.py及更新的SOURCE_MANIFEST.json/SHA256SUMS，适配用户刚部署的原版分发目录，不覆盖配置或原日志。归档SHA256=`e281000b3f7b692031b7909c8083e02944de7698e180ca6ac3d580bd722557af`；新vision.py SHA256=`481ef3037b0a549d3220cef7d490e6edc4b3c17f6a185a9e0c8301a85f099443`。证据在同目录evidence（regression-red、pytest、build、colcon-test、install-cli、patch-verification）。完整构建参数仍只维护主runbook，独立手册追加分终端更新/校验/check/observe步骤。

已告知用户先Ctrl+C退出旧observe再更新，终端B诊断可保持。尚未由开发端上传补丁、部署或启动任何现场程序，不自动提交；修复后observe与其他可能的等待原因仍须现场确认。checkpoint=PIPER_VISION_CONTEXT_FIX_OFFLINE_VERIFIED_WAIT_USER_UPDATE。

### 2026-11-14 observe重复帧处理修复，待机载复测

用户提供run_id=observe-20261114-163033-553009cf，WAIT_VISION，reason=`non-increasing camera frame numbers`，command_attempted=false。已进入帧序号检查，说明该次不再阻塞于全局context解析。旧日志未记录具体帧号，无法区分现场是单/双路重复还是倒退；本轮没有SSH或连接机载设备。

核对capture_rgbd及acquire调用链：原for循环每次要求两个流都递增，任一重复即异常，外层重启采集/暖机。RealSense官方pipeline接口说明允许同步帧组复用较慢流的上一帧（https://github.com/realsenseai/librealsense/blob/master/wrappers/python/pyrs_pipeline.cpp）。本轮先用fake复现重复/倒退/停滞，新增7个参数化场景在旧代码失败；最小修复为同一pipeline内跳过重复，仍只存三组双路严格递增帧，每组等待限3秒。帧号倒退立即失败，持续不更新超时，错误记录previous/current（color,depth）。不改相机FPS/曝光、检测阈值、运动参数或全局配置，不放宽acquire三帧和新鲜度检查。

视觉测试64项（上一版57+新增7）、boot/gripper基线66项全部通过；ctest3个包装通过，colcon133条=130实际pytest+3包装。三包构建26.6秒成功，输出build/piper-vision-frames-20261114/overlay，保留前两版输出。安装入口check=CHECK_OK、存档preview=PREVIEW_READY，模块/config均取本次install；23个运行/配置来源文件与源码一致。Python3.8语法、Bash语法、git diff --check通过。主runbook构建/检查输出及独立手册安装路径已更新，独立手册末节提供停旧进程、传输、备份、哈希校验、check、observe依赖步骤。

补丁build/piper-vision-frames-20261114/piper-vision-frames-fix-20261114.tar.gz只含vision.py、SOURCE_MANIFEST.json、SHA256SUMS，兼容首版及context版分发目录，包含context修复。归档SHA256=a74e02a711a159c3c640a7f6938362b0cec1827edae627ebec1afdd60bfff615；vision.py SHA256=b1b51e654319b62a96e514916c64facc1909fee647315185aed756f2970219a1。本地应用后全清单通过；evidence保存修改前文件、red/green、build/ctest、install-cli及patch-verification。没有覆盖配置、日志或用户其他改动。

未提交、上传、部署或启动真机。用户仍需更新后重跑observe；不能声称现场根因已排除、目标检测或抓取已通过。checkpoint=PIPER_VISION_DISTINCT_FRAMES_OFFLINE_VERIFIED_WAIT_USER_UPDATE。

### 2026-11-14 Piper语义抓取接入、部署及首轮现场识别

用户要求颜色与描述作参数、尺寸自动测量；读取外部 `/home/isee-pst/unitree_ros2/piper/agent` 后，用户选定描述→SAM→RGB-D→绝对抓取位姿→TCP换算方法，明确不增加门控/其他限制，随后明确授权“接入后启动真机，你直接调用抓取”。这更新了本任务此前不启动真机的限制；不授权自动重复抓取或扩大到其他物体/任务。实施计划见../superpowers/plans/2026-11-14-piper-semantic-grasp.md。未提交，保留原全部dirty工作和旧分发/安装。

新增semantic.py/semantic_server.py、semantic.json、test_piper_semantic.py、绿色盒子现场深度/mask fixtures；主入口增加serve、--color、--description，原蓝色config.json保持可用。语义模式不套蓝色HSV/尺寸/局部参考位移限制；使用实际内参及manual04完整外参，顶抓方向-Z，135mm工具偏移与合并-13mm基座Z偏移来自旧代码且明确未物理标定；预抓100mm，抬升20mm，闭爪目标=实测宽度减4mm，不小于0。保留现有5%/0.5Nm、反馈、IK与一次发送/等待恢复语义，没有新增审批开关。

开发机没有可用CUDA，Go2无torch/SAM；采用开发机CPU Qwen/SAM服务+SSH localhost反向隧道，机载不装重依赖。build/piper-vision-semantic-20261114/model-venv通过.pth复用co-nav-real与原check-venv，仅安装segment-anything1.0；SAM权重复用外部piper的sam_vit_b_01ec64.pth。凭证仅由原provider配置读取到进程环境，没有写入包/日志、没有导入会连接或使能机械臂的旧Agent。旧快照模型调用404；读取同provider /models只返回可用视觉模型qwen3-vl-plus及OCR版本，切换qwen3-vl-plus后真实存档调用成功（每帧约7.3–7.5秒）。说明链接 https://www.alibabacloud.com/en/notice/model_studio_update_notice_for_qwen3vlplus_and_qwenplus_models_669 。

实际推理发现SAM边缘深度背景污染最大/最小尺寸，增加深度IQR离群点处理并先红后绿测试，存档积木测得约18×47×119mm、夹取方向47mm。现场原始模型建议点同一盒子逐帧跳动最大约50mm，触发原WAIT_STABILITY；改为有效分割范围的像素中心+邻域深度，模型建议点仅作诊断保留。真实绿色盒子3帧深度/mask回归现为DETECTED，未放宽原3mm稳定判据。mask和标注图随输出保存，隐藏厚度不声称已测得；语义frame_max_age_s=120容纳CPU/网络推理且保留真实接收时间。

最终本轮12项新增语义测试通过，前版64项视觉和66项Piper基线均通过，总142 pytest；四个ctest包装通过，colcon146=142+4，不是146个实际用例。三包初次构建26.8秒、后续增量成功；当前输出build/piper-vision-semantic-20261114/overlay，26个运行/配置来源文件逐字节与源码一致，Python3.8语法/Bash语法/diff检查通过。安装legacy-check/semantic-check=CHECK_OK，legacy-preview=PREVIEW_READY；实际模型存档语义识别DETECTED，但该存档绝对顶抓规划WAIT_IK，不能声称语义规划/抓取全部通过。

分发包build/piper-vision-semantic-20261114/piper-vision-semantic-20261114.tar.gz，最新SHA256=a6f9b3c516e423d9a3f37d7cbdbca6ebc715c82da4d8a2afc7a16bec736ee81d。已获授权部署到Go2 `/home/unitree/piper-vision-semantic-20261114`，26文件清单校验通过；旧 `/home/unitree/piper-vision-grasp-20261114` 未覆盖。机载解释器仍 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`，semantic-check-20261114-01=CHECK_OK，加载新目录的模块/config。中心修复更新前校验原清单，备份semantic.py.before-centerfix，再更新清单，全部通过。

现场连接初次WAIT_IO: USB identity match count=0，用户主动重启Go2；重启后192.168.123.18无路由，但现有unitree-go别名的备用192.168.1.100可达。已确认ubuntu、can0 UP、USB-CAN序列号0046002F5246570520323934恢复，相机实际有限取图成功。不得把USB Hub序列号153223022502替代RealSense实际设备序列号151222077264。

semantic-observe-20261114-01采到绿色小垃圾桶形盒子，第一次因模型建议点跳动WAIT_STABILITY，command_attempted=false，已结束；完整证据回传evidence/semantic-observe-20261114-01。同组真实数据用修正后中心计算DETECTED，尺寸基座范围约18×84×107mm，夹取宽81–85mm；顶抓TCP约[665.8,35.4,-118.9]mm、135mm转换法兰约[665.8,35.4,16.1]mm，IK残差约98mm/21度，WAIT_IK。当前可见夹取宽度也大于硬件70mm开口。已告知用户向机械臂移近并旋转窄侧，等待摆放回复；这不是新增门控，不得用放宽IK或虚报物体尺寸强行发送。

当前模型服务service04（安装入口）与SSH127.0.0.1:18765反向隧道运行中；现场中心修复版semantic-observe-20261114-02已启动等待最新结果。当前尚未发送运动命令、尚未执行run抓取、尚未确认实物夹持。后续状态须追加真实证据，不把存档/测试/旧人工成功作为本次通过。evidence含red/depth-red/keypoint-red、pytest-final3、build-final3、colcon-test-final3、安装CLI、实际模型及现场图像和green-remeasured-plan-final。

### 2026-11-14 红色方块斜向接近与开爪力矩调整（现场进行中）

绿色盒子semantic-observe-20261114-02最终OBSERVED、CAN发送0；用户随后明确改抓红色方块。红色实测夹取宽约49.5mm，TCP约[664.6,35.1,-147.2]mm，90°顶抓WAIT_IK。新增语义--approach-pitch-deg，默认90°不变，现场用30°斜向接近：同组目标法兰约[547.7,34.2,-79.7]mm，规划PLANNED。135mm工具偏移随旋转换算，不改目标检测位置。新增1项角度回归后13项语义测试通过；143实际pytest与4个ctest包装通过，三包构建成功。该版归档SHA256=6f8dde97aee0d52660c15f0baa3c8322bfa40afdbd1aa2266abe181549d3565d已部署。

semantic-red-grasp-20261114-01在90°WAIT_PLAN，未发运动；已结束。semantic-red-grasp-20261114-02采用30°，已发一次70mm/0.5Nm开爪，持续WAIT_OPEN。反馈开口0mm、力矩约-0.008至0.026Nm、ctrl_mode=0、故障0、六轴和夹爪使能true；关节仍[0,0.107,0,0,0,0]，未发关节运动。Ctrl+C结束02且核实运行进程退出，原receipt保留；不能把进程退出当作物理急停。

用户明确“将力矩稍微调高才能打开夹爪”，后补充“一般跳到1-2Nm才可打开这个宽度”。先采用下限1.0Nm。增加可选gripper.open_effort_nm，仅开爪读取；semantic.json=1.0，闭爪effort_nm保持0.5；旧配置缺省仍0.5。IO按对应阶段配置校验，SDK目标与持久化receipt一致；原一次发送/不重放语义不变。候选0.6仅离线验证，未下发硬件。新增2项开/闭爪力矩回归，145实际pytest通过（66视觉、13语义、66boot/gripper基线），4个ctest包装通过，colcon149=145+4；三包构建成功、Python3.8语法/Bash语法/git diff检查通过。安装文件26项与源码一致，包SHA256=cc28951902086059c7a0c89f3c891284d4053cd26dcaf1ce46c73a97672ab69b，更新到同一语义目录前校验并备份旧文件。模型服务与反向隧道保留。当前尚未确认新力矩实物开爪或抓取，后续追加真实结果。

现场后续：semantic-red-grasp-20261114-03已发送70mm/1.0Nm，仍WAIT_OPEN，0mm/约0.02Nm/ctrl_mode0。停止该run后，单独持同一控制锁发送一次MotionCtrl_2(1,1,5,0)，不发送关节/夹爪新目标；唯一mode-attempt.json落盘后才调用。ctrl_mode切至1，原保留开爪目标随即执行，反馈由0→46.4→68.3mm，六关节没有变化、故障0。这确认本次待机模式阻止开爪，不能把先前不打开全部归因于力矩；也不能由此判断0.5Nm在CAN模式下是否足够。当前产品入口尚未自动完成开爪前的模式初始化，重启后的修复待后续接入；本次现场已完成一次初始化。

用户随后要求先35mm，暂停抓取，仅单次GripperCtrl(35000,1000,1,0)，semantic-open35-20261114-01使用现有PiperSession及durable receipt；12秒反馈最后35.8mm、ctrl_mode1、故障0、六关节仍[0,0.107,0,0,0,0]。用户明确“观察到夹爪正常开启，可以抓取”，继续授权本次红色目标，已启动semantic-red-grasp-20261114-04，70mm/1.0Nm开爪、0.5Nm闭爪、30°接近、5%速度。mode/open35脚本与远端证据已回传evidence；此处尚不能声称抓取完成。

semantic-red-grasp-20261114-04已OPEN_READY，重新三帧识别期间偶发深度中心漂移约6mm，自动重取后通过；第一次关节entry-1已实际移动。目标[0.329,10.024,-5.191,0.441,-2.406,-0.207]度，反馈长期[0.274,9.980,-5.197,0.401,-2.445,0]度，最大差0.207°，但motion_status=1持续超过60秒，原wait_snapshot因此一直不返回，未发送entry-2/闭爪。核对固定SDK0.6.2枚举：0=REACH_TARGET_POS_SUCCESSFULLY、1=REACH_TARGET_POS_FAILED；该字段不是速度。停止04，保留目标/receipt；没有回零或释放动作。

本轮最小修复main.wait_snapshot不再用固件到位位替代实际位置判断；wait_joints仍依原0.3°误差判断，拍照与规划仍核对前后关节变化0.3°，故障/使能/反馈新鲜度检查不变。离线preview相同处理，不能把motion_status1直接等同运动中。新增完整fake周期motion_status1回归及超差先WAIT_TARGET再位置到位的回归；147实际pytest（67视觉、14语义、66旧boot/gripper）通过，4个ctest包装，colcon151=147+4。三包构建成功、Python3.8语法/diff检查通过。包SHA256=97b1f7cc122a04f6166dad62fcf723186ef111fb7f798c10ab57ffdd17b67c4e，26安装来源一致；远端更新前全清单通过、备份main/manifest后更新且再次全清单通过。已发5秒提示音。semantic-red-grasp-20261114-05从实际当前关节重新识别/规划启动，未自动重放04的stage；新进度待追加。

semantic-red-grasp-20261114-05现场：从当前位置完成全部25段无载关节到位（entry/pregrasp/grasp），27个receipt为开爪+25关节+闭爪；没有lift receipt。最终抓取关节反馈[4.066,169.597,-116.982,10.161,-17.850,-7.954]度，SDK法兰[547.641,33.998,-79.818]mm，状态/故障0，motion_status0。按测得宽49.4657mm下发close45.4657mm/0.5Nm一次，实际反馈47.0mm、-0.5Nm。原接触判据要求反馈>目标+2mm，此次差约1.53mm，因此WAIT_CONTACT且尚未抬升；不能仅凭力矩或差值认定抓牢。已通过异步问题询问用户实物是否位于两指之间并夹紧，等待现场回复；此前“可以抓取”不是夹持后确认。

有限单帧只读图像red-contact-view-20261114-01已采集（无CAN写），本地evidence/red-contact-view.png显示相机视线被机械臂遮挡，无法从该图确认方块与夹指接触。05完整证据已回传evidence/semantic-red-grasp-20261114-05（该次快照含188条WAIT_CONTACT；后续日志仍增长）。模型服务与隧道仍运行；05执行进程仍在WAIT_CONTACT，保持目标、不重发、不抬升、不自动松开。后续必须根据用户实物反馈继续判断，不将该状态写为抓取通过。当前待办：现场接触确认；入口重启时CAN模式初始化仍未集成；手眼/TCP试验参数的物理准确性、遮挡下自动抬升验证尚未完成。

### 2026-11-14 用户确认抓空，抬升200mm后重新计算；当前等待目标移近

用户明确“机械臂未夹住方块，抬起机械臂”，并要求“然后重新计算夹取”。因此05不能记为成功；已Ctrl+C退出其WAIT_CONTACT并确认无该run控制进程。后续用户补充现场偏差“最终偏低”。未自动追加闭爪、松爪或重新抓取。

复用geometry.plan_motion/solve_ik、PiperSession持久化一次发送和main到位反馈，先由实时关节计算竖直100mm、保持末端朝向的10段路径，5%速度执行semantic-user-raise100-20261114-01。SDK法兰Z=-79.939→20.674mm，实测增量[0.134,0.013,100.613]mm，全部10关节receipt到位，无夹爪命令。当前视线仍被机械臂遮挡，semantic-red-replan-20261114-01反复requested object not found，已停止。向用户说明后，为完成抬起及重新取图再执行同方向100mm的10段路径semantic-user-raise100-20261114-02；最终关节[4.066,140.472,-105.197,86.524,-3.074,-84.799]度，SDK法兰[547.605,34.045,119.987]mm，故障0、motion_status0、开口46.3mm。累计法兰Z增加199.926mm，夹爪目标未改。抬升过程保持世界末端朝向；J4/J6在第二次抬升时有较大补偿旋转，均按既有限位和连续IK执行，不把逆解结果当碰撞证明。

上述两次抬升各在执行前做离线路径断言、执行后核对真实feedback/10个receipt；一轮脚本及现场证据见build/piper-vision-semantic-20261114/evidence/raise-arm.py、raise-arm-clear-view.py和两个semantic-user-raise100目录。是复用现有控制代码的一次现场操作，没有修改产品源码/配置，没有提交；未为文档及现场证据重跑全库构建。

第二次抬升后图像red-after-raise200.png已清楚看到直立红色方块及上方夹指。重新计算期间旧SSH反向隧道会话退出143，机载WAIT_VISION connection refused；本地模型服务仍监听18765，已恢复相同127.0.0.1:18765反向隧道。semantic-red-replan-20261114-02最终完成三帧DETECTED，但WAIT_PLAN absolute grasp IK unavailable，已停止只读循环；没有CAN运动写入。完整资料已回传evidence/semantic-red-replan-20261114-02。有效观测before-d365e7da：camera point=[0.011273864,0.073238667,0.318000015]m，可见基座范围约10.3×51.1×49.0mm，夹取宽51.05mm；仍不能声称测到隐藏厚度。旧参数抓取TCP=[740.779,24.544,-155.706]mm，比前次TCP的基座X约远76mm，原因未确定，不假定是用户移动或被夹爪推走。

重计算在开发机纯数值进行：原-13mm Z偏移与当前新观测，检查90/60/45/30/20/15/10/5/0/-10/-20度均WAIT_IK。根据用户“偏低”，候选方案先取消旧grasp_offset_base_m=[0,0,-0.013]下移项，改为[0,0,0]，使目标回到测得中心，上调13mm；这只是待验证候选，尚未改生产semantic.json或下发该目标，不声称修正了物理标定。随后模拟物体沿基座-X移近0..100mm，每10mm一档，扫描30/15/45/0/60/90度：40mm开始30度PLANNED；60mm时30/15/45度均PLANNED。证据red-replan-after-raise-comparison.json、red-height-and-reach-comparison.json；假设移动坐标不是现场实测坐标，不得直接发送。

已请求用户将红色方块向机械臂移近约6cm并保持直立，等待回复后重新识别计算，不能把时间流逝当已摆好。当前机械臂在抬起后的姿态保持，无抓取控制循环；模型服务及恢复的隧道运行。剩余事项：接收新摆放、核实实际目标位置、按确认的高度修正生成可审阅方案；本次抓取失败根因与真实TCP/外参误差尚未分离，重启CAN模式初始化仍为已知待办。

### 2026-11-14 六轴回零完成；按用户限制转为纯离线摆放空间计算

用户最新要求“现在将6个关节的位置全部回到零位，然后并不操作真机，计算给出可以抓取的空间范围，以便放置物体”。它替代此前等待移近目标并继续抓取的现场流程；回零完成后不再采相机/CAN、运动或现场验证。按实时关节[4.066,140.472,-105.197,86.524,-3.074,-84.799]，5%速度15段等比例关节插补到0，复用既有发送/反馈/限位/receipt；未重新校零、未发夹爪命令。semantic-user-return-zero-20261114-01最终ZERO_REACHED，关节[0,0.094,0,0,0,0]度，SDK法兰[56.276,0,213.173]mm，故障0、motion_status0、开口46.3mm。15条receipt及result已回传evidence；SDK与当前模型FK差约0.074mm仅说明数值一致性。真机操作到此结束。

离线复用当前geometry.py和本地模型/存档相机内外参，在build/piper-workspace-20261114保存脚本/JSON/CSV/PNG/PDF。扫描1200个Y=0平面姿态组合、420个局部三维组合，再对候选区按10mm网格细查216个位置。约50mm方块、135mm试验TCP、30°向下接近、闭合轴基座Y、预抓100mm/抬升20mm，候选夹持点X600–650/Y0–50/Z-125至-75mm的216样本全部完整规划通过且八角落在存档相机视野。示例中心[625,25,-100]mm。原点为Piper基座模型安装面中心，Y向机器人自身左侧，Z向上；不是相机或Go2基座。相机可见检查不含实际遮挡/深度质量，几何通过不等于夹持实测通过。

本轮候选计算取消旧-13mm基座Z下移，将目标设为计划夹持点；未修改生产semantic.json或部署此变更。TCP135mm仍未物理标定，手眼physical_validation仍NOT_PERFORMED，碰撞、载荷与上次偏低根因仍未验证。不改关节限位、不加运行门控；候选范围不是限制真实运动的配置。当前未允许再次抓取或启动真机验证。完整结果见docs/multi_agent/piper_grasp_workspace_20261114.md。没有产品源码修改或构建，保留全部dirty工作、不提交；只更新计算证据和文档。

### 2026-11-14 用户重新摆放后授权新一轮红色方块抓取

用户最新“现在移动了位置，尝试抓取”明确恢复本次现场识别/运动授权，更新上一轮回零后纯离线的限制。未沿用假设摆放坐标，启动新的相机识别与规划。核实没有遗留run/preview/回零控制进程，模型服务及127.0.0.1:18765反向隧道均监听正常。新建独立运行配置 `/home/unitree/piper-vision-semantic-20261114/config/piper_vision_grasp/semantic-height0-trial-20261114.json`，仅把基座Z偏移-0.013改为0，其他配置完全一致；不覆盖原semantic.json/安装包或用户源码。SHA256=e841523dfeaf6090f465459b26e92c9c4064555c682a3f8bb78a8dd1ff547905，副本已保存在evidence，差异断言通过。

通过同一安装入口先check，再启动semantic-red-grasp-20261114-06，红色/方块、30°接近、5%速度、70mm/1Nm开爪、测得宽减4mm/0.5Nm闭爪；本次运行配置与离线候选的取消下移条件一致。尚无本轮现场抓取结论，后续追加实际观察和命令receipt。无产品代码改动、未提交。

### 2026-11-14 取消下移后仍抓空；按现场修正后转为三维排障

semantic-red-grasp-20261114-06完成开爪与全部接近阶段，闭爪目标45.6028mm/0.5Nm，反馈46.6mm/约-0.39Nm，WAIT_CONTACT，未抬升。用户确认“没有夹住，夹的位置偏下”，本轮抓取失败，已退出等待进程。用户进一步明确“还需上抬13cm”，不是13mm。复用现有控制模块单次开70mm/1Nm，再以5%分13段向基座+Z移动130mm，semantic-red-align-up130-20261114-01反馈SDK法兰Z=-66.249→63.822mm，ΔZ130.071mm，未闭爪。用户确认“高度对了，需要水平调整”；根据有限图像与现场反馈向画面右侧/基座-Y平移40mm，semantic-red-align-right40-20261114-01共4段，SDK实际ΔY约-39.56mm，末次法兰[532.55,-8.409,63.337]mm、关节[-0.974,143.948,-97.649,-2.116,-11.17,1.921]度，开口68.3mm，故障0。未再次闭爪/抓取，也未把130mm/-40mm经验修正写入外参或生产配置。

用户随后要求将VLM语义分割标识的空间点、目标形状与机械臂模型放入一张3D图排障，因此保持该姿态转为可视化。有限采集red-diagnostic3d-20261114-01三帧RGB-D及被动前后反馈，CAN发送0；新反馈J4=-2.217°，前后变化<0.3°。用户没有确认右移后的最终夹持对准，不把该姿态记为抓取成功。

成果build/piper-diagnostic3d-20261114：可内联旋转图piper-spatial-diagnostic.html、含Plotly的离线独立diagnostic-standalone.html、静态PNG/PDF、scene/evidence/findings JSON与复算脚本。详述docs/multi_agent/piper_grasp_3d_diagnostic_20261114.md。VLM建议像素反投影点、SAM中心/可见点云/形状外包络/夹取轴、历史抓取前点云与目标/关节链、当前真实反馈关节驱动的URDF/STL模型、试验135mm TCP与坐标轴同图。URDF法兰与现有FK断言一致；外形显示简化，隐藏物体几何不补造。

新三帧中心最大距离1.029mm，夹取轴最大差79.113°，合并WAIT_ORIENTATION；当前模型TCP相对新SAM中心为[-58.56,-18.66,+135.05]mm，现场此前已确认上抬后高度对准，二者不一致仍待定位。VLM与SAM中心高度差<0.45mm，不能用建议点选择差异解释13cm。物体历史/当前X位置也不同，尚未确认是否物理移动。没有修改标定、TCP或识别算法，没有把可视化解释写成确诊结论。

坐标有限/网格索引/Plotly schema/JS语法/文档链接/diff检查通过，CDN3.7.0 HTTP200，静态图已查看；无可用浏览器，未验证真实交互渲染。无产品源码修改、无构建部署/提交。当前无运动控制进程，夹爪保持张开，后续先依据三维图排查；不得自动继续闭爪或抓取。

### 2026-11-14 用户再次要求六轴回零，最终反馈全0

用户明确“将机械臂所有关节回零”。确认无遗留运动进程、六关节使能/无故障，起始反馈[-0.974,143.948,-97.649,-2.217,-11.17,1.921]度。复用此前回零方法，以5%速度分15个等比例关节目标返回0；run_id=semantic-user-return-zero-20261114-02，每段单次持久receipt且不重放，不发夹爪指令、不重设零点。

执行结束首次ZERO_REACHED时J2=0.104°（其余0），在原0.3°容差内，motion_status1；随后独立只读5次反馈复查，末次六关节精确报告[0,0,0,0,0,0]度，motion_status0、arm_status0、err_code0、READY，SDK法兰[56.127,0,213.266]mm。夹爪保持68.3mm张开。15条receipt、5次被动postcheck及result全部回传build/piper-vision-semantic-20261114/evidence/semantic-user-return-zero-20261114-02，断言检查通过。

当前机械臂回零保持，无活动运动控制进程；3D排障图仍是先前右移后的历史姿态快照，不是实时机械臂显示。本轮未继续抓取或更新标定/产品代码，未构建或提交。

### 2026-11-14 用户指定相机外参基座Z增加130mm并授权重试

用户明确“直接调整标定的外参，将机械臂坐标下的相机位置向上提高13cm,旋转不变。然后重试夹取过程验证”。本轮仅修改独立视觉抓取配置的T_base_color_optical平移Z，由-0.12532006100896664m变为+0.004679938991033361m，精确增加0.130m；X=0.4159578390386031、Y=0.0529010550714366及全部旋转元素保持不变。原manual04-result.json未改；新增provenance/manual04-user-z130-20261114.json记录父来源/哈希、用户指定基座位移、派生正逆变换，extrinsics.json更新source_file/sha。原加载校验保留，没有绕过来源核验或新增门控。physical_validation仍NOT_PERFORMED，这是用户指定的人工外参修正，尚不能称为标定或抓取通过。

新增1项回归先失败后通过：核对旋转/X/Y精确保留、Z+0.130m，并验证多个相机点在基座下只发生同样的Z平移。定向148实际pytest通过（67视觉、15语义、66boot/gripper），4个ctest包装通过，colcon152=148+4。三包构建成功；安装入口semantic-check=CHECK_OK；27个运行/配置来源与源码逐字节一致。新包SHA256=e523d1bb800d6d8e9b503e7491ec2ce6767d0b45d97891b365e044370072c788。机载更新前旧清单通过，备份原外参/manifest到/tmp/piper-semantic-before-extrinsic-z130-20261114.tar.gz，再展开新包，27来源清单通过。源外参旧副本保存在本地evidence/extrinsics-before-z130.json。未改外部piper项目、导航、相机旋转、TCP或其他运动参数，保留全部dirty工作，不提交。

继续使用已有semantic-height0-trial-20261114.json（本轮未改其文件，仍取消旧-13mm抓取下移），共享extrinsics.json现加载新外参；CLI红色/方块/30°，5%速度，1Nm开爪、0.5Nm闭爪。新增semantic-extrinsic-z130-check-20261114-01后启动semantic-red-grasp-20261114-07，按新RGB-D重新定位/规划，不沿用旧物体坐标或人工横移40mm。当前尚无本轮抓取结论，后续补充真实feedback/receipt。旧3D图和工作空间图保留原外参历史含义，不能当作新外参下实时结果。证据pytest/colcon/build/deploy-extrinsic-z130与source派生文件已保存，已发5秒提示音。

semantic-red-grasp-20261114-07现场已执行到闭爪：按新外参重新识别，目标模型TCP=[620.063,32.888,-0.151]mm，目标法兰=[503.347,39.671,67.349]mm；测得夹取宽49.5936mm。全部23个关节阶段AT_TARGET，开爪/关节/闭爪共25个receipt，没有lift receipt。闭爪目标45.5936mm/0.5Nm，实际46.6mm、约-0.37Nm，原接触判据未满足，当前WAIT_CONTACT，尚未抬升或验证抓取成功。最终关节[5.981,138.901,-87.492,26.888,-17.980,-21.127]度，SDK法兰[503.393,39.967,67.686]mm，motion_status0、arm_status0、err_code0。已回传该时刻日志/计划/receipt至evidence/semantic-red-grasp-20261114-07；控制进程仍等待、不重复下发，后续日志可能继续增长。已询问用户高度是否对准、是否夹住及残余偏差，等待实物反馈，不将WAIT_CONTACT或到位反馈写成夹取通过。

### 2026-11-14 用户要求抬起后重新识别夹取，收集进一步外参修正证据

用户明确“将机械臂抬起并重新识别夹取，以进一步修正外参”。已退出07闭爪等待并核实无旧控制进程；07仍未获得现场夹持成功确认。复用既有上抬脚本，semantic-red-reobserve-raise100-20261114-01从实时关节规划，单次70mm/1Nm开爪后以5%速度分10段保持末端方向上抬100mm。实际SDK法兰Δ=[-0.070,+0.008,+100.069]mm，最终[503.323,39.975,167.755]mm，关节[5.981,127.151,-90.811,82.249,-8.090,-77.678]度、开口68.2mm、故障0。10段AT_TARGET及11个receipt（含开爪）已回传，规划方向/位移断言和执行结果检查通过；脚本evidence/red-reobserve-raise100.py。上抬是重新观察准备，不是新增外参补偿。

已启动semantic-red-grasp-20261114-08，继续semantic-height0-trial-20261114.json/红色方块/30°及现有基座Z+130mm外参，从新RGB-D重新定位规划。尚未更改外参其他分量、旋转或TCP；进一步修正需结合新识别与现场偏差，不能把模型预测自身当作真实标定误差。本轮无产品源码配置修改、无构建提交，当前运行结果待追加。

08多组识别中心稳定约基座[645.785,33.023,-1.787]mm，但旧框宽高启发式选择右侧近竖直边，投影水平后夹取轴约[-0.99,0.1,0]、宽15–21mm；反复WAIT_PLAN/WAIT_ORIENTATION。离线0/15/30/45/60/90度均WAIT_IK。退出08并核实无控制进程，未生成attempts、无新的接近闭爪动作。用户随后明确“先不修正，按照现有外参夹取，在此轮中观察”，因此本轮外参继续原+130mm、不增加横移/旋转补偿。

为完成此次现有外参下的夹取，仅修正semantic.measure方向选择：从相同有效RGB-D中估计框上边/右边的3D端点，取基座XY投影较长的边，不再按图像框宽高决定边；目标中心、外参、TCP、状态检查与接触阈值不变。原因：旧规则在153×138像素框上选了近竖直边，把几毫米深度变化归一化成前后夹取方向。新增2项回归覆盖宽框及相机滚转90°，旧实现一项失败、新实现全通过；150实际pytest=67视觉+17语义+66boot/gripper，四ctest包装通过，colcon154条目0错误0失败。三包构建成功，安装check=CHECK_OK、Python3.8语法/diff检查通过。08三组真实存档回放全部DETECTED/PLANNED，目标中心不变、夹取宽51.92–52.47mm；证据red-reobserve-axis-replay.json。仅测可见形状，不补造隐藏厚度，方向修正并不证明真实夹持。

新包SHA256=580a26235a9c6ef4e4dd86a12bdf85479f181c8afced25dd2fc6a9d5ec98717e，27个源码/安装来源一致。远端旧清单通过并备份semantic.py/manifest/README至/tmp/piper-before-axis-projection-20261114.tar.gz后更新；29文件校验全通过。外参SHA256=4440502eb5f0f9c4f54fba38f45917e771bbc25050e0cf21cf18aa0dc161c822，本轮外参未改。机载semantic-axis-check-20261114-01后启动semantic-red-grasp-20261114-09，使用同一height0试验配置、红色方块、30°、5%速度，重新现场取图；08存档目标仅用于离线验证。已发5秒提示音、未提交，09结果待追加。

09现场新图像已DETECTED并完成规划：模型TCP=[645.785,33.023,-1.787]mm，目标法兰=[534.336,68.346,65.713]mm，测得宽51.8827mm。15个接近/抓取关节阶段全部AT_TARGET，17个receipt=开爪+15关节+闭爪，没有lift。闭爪目标47.8827mm/0.5Nm，实际48.9mm、约-0.39Nm，因此当前WAIT_CONTACT，尚未抬升、不能确认夹持成功。末次关节[11.378,146.380,-103.673,80.725,-25.071,-64.407]度、SDK法兰[534.315,68.707,65.762]mm，motion_status/arm_status/err_code均0。日志/计划/receipt快照已回传evidence/semantic-red-grasp-20261114-09；该进程仍等待、不重复下发。已询问用户当前是否夹住及偏差方向/距离；本轮外参没有进一步修改，等待现场观察，不把闭爪到达或离线规划当成抓取验证通过。

### 2026-11-14 外参−Y110mm及光轴对齐，用户随后授权继续夹取

用户要求“相机外参再向piper y-移动11cm，令相机z+绕相机x+旋转与piper x+平行”。数值确认两条件不能同时精确满足：严格绕原相机X旋转−10.0463°仍有3.0856°残差；用户明确选择优先完全同向，允许相机X略变。已退出09等待并核实无旧控制进程。相机位置变为[0.4159578390386031,-0.0570989449285634,0.004679938991033361]m，在上一轮基础上仅Y减0.110m、保留Z+0.130m。围绕相机原点最小旋转10.5048148466°使R第三列=[1,0,0]（约1e-15误差），没有绕基座原点旋转平移向量。新来源provenance/manual04-user-z130-y110-alignx-20261114.json保留父来源hash、增量与逆矩阵；原manual04/z130记录未改，physical_validation仍NOT_PERFORMED。

新增1项回归先失败后通过，覆盖基座平移、光轴方向、正交/行列式、最小旋转及光轴点转换；原z130测试改为核实历史派生文件。151实际pytest=67视觉+18语义+66boot/gripper，四ctest包装通过，colcon155条目0错误0失败；三包构建与本地安装check通过。新分发包28源码/安装来源一致，SHA256=49fba4a903ff689ad2f7ceb88b436cc7e8a24f38d2858d6664085ff1ee5e3ee0。远端旧清单通过、备份/tmp/piper-before-extrinsics-y110-align-20261114.tar.gz后展开，30文件校验通过；semantic-extrinsics-y110-align-check-20261114-01=CHECK_OK且command_attempted=false。源/机载外参SHA256同为b662534aa1f222638b63641c9867b4666ed854602b57c0889e8bcc9c96e4d99d。旧外参备份和两候选比较保存在evidence/extrinsics-before-y110-rotation.json等；已发5秒提示音，未提交。

用户追加“完成调试参数后再尝试夹取”，授权按新外参重新现场识别/运动。当前09保持姿态直接上抬100/80mm纯离线规划无逆解（腕部限位附近），60/50mm可规划；未下发失败路径。选择6段60mm保持朝向上抬，先单次70mm/1Nm开爪，沿用5%速度和原反馈检查，semantic-red-reobserve-raise60-20261114-01已启动；脚本evidence/red-reobserve-raise60.py离线方向/位移断言通过。上抬后重新取图，不使用旧抓取坐标。后续实测结果待追加。

semantic-red-reobserve-raise60-20261114-01已完成：实际SDK Δ=[+0.036,-0.008,+60.234]mm，末次法兰[534.351,68.699,125.996]mm，关节[11.378,139.519,-104.722,97.701,-24.972,-83.112]度、开口68.3mm、状态/故障0。6段关节到位及7个receipt（含开爪）已回传并断言核实。随后启动semantic-red-grasp-20261114-10，使用现有height0配置、新Y−110mm/光轴对齐外参、红色方块/30°/5%，首轮识别规划及OPEN_READY已通过，正在开爪后再次取图，尚未记录本轮抓取结果。

semantic-red-grasp-20261114-10已按新外参完成本轮接近和闭爪。新RGB-D目标TCP=[640.958,-63.734,-40.975]mm，目标法兰=[526.747,-38.743,26.525]mm，测得夹取宽53.128mm；19个关节阶段AT_TARGET，21个receipt=开爪+19关节+闭爪，无lift。闭爪目标49.128mm/0.5Nm，反馈50.1mm、约−0.36Nm，当前WAIT_CONTACT且尚未抬升，不能确认夹持成功。末次关节[-2.749,148.405,-97.242,27.971,-17.782,-22.001]度，SDK法兰[526.777,-38.346,26.938]mm，motion_status/arm_status/err_code均0。完整当前日志/计划/receipt快照已回传evidence/semantic-red-grasp-20261114-10，并核对19到位/21receipt/无lift；控制进程仍只等待、不重发。已询问用户本轮是否夹住及剩余偏差方向/距离。现外参及位置修正的现场有效性仍待用户反馈，未自行再次调整或重复抓取。

### 2026-11-14 当前外参再沿基座+Z增加30mm并重试

用户明确“再将相机中心向piper z+方向移动3cm，然后测试夹取”。已退出10等待并核实无旧控制进程；10没有夹取成功确认。外参在z130-y110-alignx基础上仅T[2,3]增加0.030m，相机位置现为[0.4159578390386031,-0.0570989449285634,0.03467993899103336]m；X/Y与R逐元素保留，光轴仍对齐基座+X。新来源provenance/manual04-user-z160-y110-alignx-20261114.json保留父哈希/增量/逆变换；旧外参另存evidence/extrinsics-before-extra-z30.json。累计相对原始标定Z+160mm、Y−110mm，physical_validation仍NOT_PERFORMED。

更新现有外参回归先失败后通过，新增断言R与上一轮完全相同；本轮未增加测试数量，151实际pytest=67视觉+18语义+66boot/gripper全部通过，四ctest包装/colcon155条目0错误0失败。三包构建、安装check及diff检查通过；29源码/安装来源逐字节一致。新包SHA256=246682456e84f9216f6d808f09caac5397ebeb86311b3e446662087c21234773。机载旧清单通过并备份/tmp/piper-before-extrinsics-z30-20261114.tar.gz后更新，31文件校验通过，semantic-extrinsics-z30-check-20261114-01=CHECK_OK；源/机载外参SHA256一致为8b74d07ee417ff3403a1f51a82c920255d4be16b7860f8575c1e469f12987c94。已发5秒提示音，不提交。

准备重试：以10末次反馈为离线核对种子，保持朝向上抬100mm的10段路径PLANNED。复用现有脚本evidence/red-reobserve-raise100-03.py，run semantic-red-reobserve-raise100-20261114-03，运行时从新鲜反馈重新规划；先单次70mm/1Nm开爪，再以5%上抬，非新的外参补偿。上抬与随后的新图像抓取结果待追加。

semantic-red-reobserve-raise100-20261114-03完成，上抬SDK Δ=[−0.054,+0.084,+100.353]mm，末次法兰[526.723,−38.262,127.291]mm，关节[−2.749,135.659,−97.172,68.989,−8.771,−63.916]度、开口68.3mm、状态/故障0；11个receipt（含开爪）已回传并核实。

随后semantic-red-grasp-20261114-11（30°）重新识别方块，深度约0.287m，当前目标基座TCP约[702.958,−73.493,−29.413]mm，比10测得目标X更远约62mm；不能把此变化归因为单纯外参+Z或未经确认的物体运动。连续WAIT_PLAN，原因local entry branch unavailable/absolute grasp IK unavailable，command_attempted=false；没有接近/闭爪。根据本轮存档离线扫描接近角（evidence/red-z30-retry-pitch-scan.json），25/35/20/40/15/0°可完整规划，30/45/60/90°失败。保持外参不变，选择距原30°最近的25°，已向用户说明并退出11，启动semantic-red-grasp-20261114-12重新现场取图，非发送存档坐标；速度/力矩/检查均不变。12实际结果待追加。


## 2026-11-14 完成提示音乐调整

按用户要求，以 `build/piper-vision-grasp-20261114/evidence/completion.wav` 原5秒音频为周期，生成 `assets/audio/work_complete.wav`：原音高、+1半音、+2半音，各5秒，共15秒。使用本机既有librubberband保时长移调，未安装依赖；首周期PCM与原音频逐字节一致，三周期帧数检查通过。AGENTS.md已更新后续播放规则与paplay命令。仅修改音频资产和文档，不改机器人代码，不构建或启动ROS/硬件。

12（25°）现场完成16个关节阶段，18个receipt含开爪/闭爪，无lift。新目标TCP=[702.958,−73.493,−29.413]mm、目标法兰=[583.491,−47.083,27.641]mm、测得宽54.2615mm；闭爪目标50.2615mm/0.5Nm、实际51.3mm/约−0.37Nm，WAIT_CONTACT。末次关节[−3.290,160.737,−123.788,51.661,−10.564,−47.306]度，SDK法兰[583.389,−46.644,27.150]mm，状态/故障0；日志/计划/18receipt回传核实。用户未确认夹住，而继续要求“将z再上调2cm”；因此不将12记为成功。本轮外参+30mm阶段测试已完成到闭爪，继续按新指示追加+20mm并重试。

### 2026-11-14 按现场要求再上调20mm，累计基座Z+180mm

用户在12夹取后要求“将z再上调2cm”，作为当前调参/夹取测试的继续。已退出12等待并核实无控制进程。仅将当前相机平移Z再加0.020m，现T平移=[0.4159578390386031,−0.0570989449285634,0.05467993899103337]m；旋转/X/Y完全保留，累计相对原manual04 Z+180mm/Y−110mm。新来源provenance/manual04-user-z180-y110-alignx-20261114.json链接z160父来源与哈希/逆矩阵，旧值备份evidence/extrinsics-before-extra-z20.json。physical_validation仍NOT_PERFORMED。

现有外参测试更新为累计平移+50mm（相对z130文件），先失败后通过；151实际pytest（67视觉+18语义+66boot/gripper）通过，4ctest包装通过、colcon155条目0失败0错误。三包构建、安装check与diff检查通过；30源码/安装来源一致。新包SHA256=fa4306fa07ec123c070acf546740ebd404b521f819cb10ca09ed71e59d003690。机载旧清单通过并备份/tmp/piper-before-extrinsics-z20-20261114.tar.gz后更新，32文件校验通过；semantic-extrinsics-z20-check-20261114-01=CHECK_OK。源/机载外参SHA256一致为940d2583e3d3eea034e354f48bf3fc766ac40b8d83829b52996e5c952adb30c7。已发5秒提示音，不提交。

12当前姿态纯离线检查上抬100mm在腕部限位附近无逆解，未下发；60mm六段完整规划通过。复用evidence/red-reobserve-raise60-02.py启动semantic-red-reobserve-raise60-20261114-02，先单次开70mm/1Nm再以5%速度上抬，随后延续25°重新识别夹取；无新门控、无其他外参/算法改动。实际抬升和下一轮抓取结果待追加。

semantic-red-reobserve-raise60-20261114-02完成，SDK Δ=[+0.061,−0.098,+59.748]mm，末次法兰[583.450,−46.742,86.898]mm，关节[−3.290,153.034,−121.773,83.726,−8.378,−79.738]度，开口68.3mm、状态/故障0，7receipt已回传核实。随后启动semantic-red-grasp-20261114-13，延续height0配置/25°/5%并加载z180新外参，首轮现场识别规划及OPEN_READY已通过，正在重新定位执行；本轮结果待追加。

### 2026-11-14 第13轮闭爪后转为最新三维可视化

13现场完成20个关节阶段，22receipt（开爪+20关节+闭爪），无lift。目标TCP=[617.958,−82.790,16.423]mm、目标法兰=[496.315,−69.636,73.477]mm、测得宽51.4769mm；闭爪47.4769mm/0.5Nm，反馈48.7mm/约−0.41Nm，WAIT_CONTACT。最终关节[−8.391,137.762,−84.588,−5.011,−23.270,3.668]度，SDK法兰[496.147,−70.011,73.666]mm，motion_status/arm_status/err_code均0。用户没有确认夹住，改为“将现在的结果可视化”。已退出13等待且pgrep核实无活动抓取进程，没有新增移动/开闭爪/回零。

新成果build/piper-diagnostic3d-20261114-run13，含内联piper-current-grasp.html、离线diagnostic-standalone.html、整体/夹指局部PNG及PDF、scene/evidence/findings JSON、冻结config-resolved/extrinsics与反馈、复算脚本。详见docs/multi_agent/piper_grasp_3d_run13_20261114.md。全部离线使用13抓取前最后一组DETECTED三帧RGB-D与闭爪后最后反馈（相隔约85.1秒），未新增相机采样/CAN发送；不伪称闭爪后实时目标真值。旧3D图保留。

新外参位置[415.958,−57.099,54.680]mm、光轴+Z平行基座+X；目标表面、三帧VLM/SAM点、夹取轴、目标TCP/法兰/路径、实际反馈关节的URDF/STL机械臂和48.7mm夹爪同图。反馈模型TCP=[617.778,−83.256,16.552]mm，与程序目标差[−0.180,−0.466,+0.129]mm、距离0.516mm；这只核对软件与模型，不证明实物TCP/手眼或夹取成功。三帧SAM最大距离1.026mm，VLM-SAM Z差≤0.336mm。形状只表示可见深度，隐藏厚度未推断。

URDF/FK一致、坐标/网格索引/Plotly schema/JS语法检查通过，静态图已查看；内联约485KB，离线Plotly3.7.0内嵌约6.1MB，无外部脚本依赖。无浏览器，未实测交互。未改产品代码/配置，未构建提交。当前机械臂保持13闭爪姿态，无活动抓取控制进程；需要新指示再运动。

### 2026-11-14 相机沿基座+Y平移70mm并重新夹取

用户明确“将相机位置向piper y+方向移动7cm，再次夹取”，恢复当前真机重试授权。13可视化后无遗留控制进程。外参仅Y增加0.070m，当前相机平移=[0.4159578390386031,+0.012901055071436609,0.05467993899103337]m，X/Z/R保持不变，光轴仍对齐基座+X；相对原manual04累计Y−40mm/Z+180mm。新来源provenance/manual04-user-z180-y40-alignx-20261114.json链接z180-y110父来源/哈希/逆变换，旧外参备份evidence/extrinsics-before-yplus70.json。physical_validation仍NOT_PERFORMED；第13轮图保留历史含义。

更新现有外参回归先失败后通过；151实际pytest=67视觉+18语义+66boot/gripper通过，4ctest包装通过、colcon155条目0失败0错误。三包构建、安装check、diff检查通过；31源码/安装来源一致。新包SHA256=c1a7df0c6d81d68966317f50fe32a0d3a275a4aa2761c2910447f2f442d527b7。机载旧清单通过并备份/tmp/piper-before-extrinsics-yplus70-20261114.tar.gz后更新，33文件校验通过；semantic-extrinsics-yplus70-check-20261114-01=CHECK_OK。源/机载外参SHA256一致为ce7a8dca08b2049a2ed7e8e9ed9a56de8aadc9a905b0efd54c247220c1375533。提示音已发、不提交，保留其他dirty与新增assets。

发现旧模型服务及反向隧道均已退出，已复用内存取凭据的launcher恢复service-yplus70-20261114-01，MODEL_READY/cpu/127.0.0.1:18765；恢复同地址SSH反向转发，机载监听核实通过。没有打印或写入API密钥。准备重试：从13末次反馈离线规划保持朝向上抬100mm，10段PLANNED，脚本evidence/red-reobserve-raise100-05.py复用原控制模块；semantic-red-reobserve-raise100-20261114-05已启动，先单次开70mm/1Nm、5%速度上抬，运行时从实时反馈重新规划。随后延续25°、新RGB-D重新抓取，实际结果待追加。

semantic-red-reobserve-raise100-20261114-05完成：SDK Δ=[−0.108,−0.014,+99.790]mm，末次法兰[496.039,−70.025,173.456]mm，关节[−8.391,125.701,−87.780,−14.053,−8.094,13.057]度、开口68.2mm、状态/故障0，11receipt回传核实。随后启动semantic-red-grasp-20261114-14，height0配置/25°/5%、新Y+70外参，首组三帧WAIT_STABILITY、command_attempted=false，正在重采样。尚未产生本轮抓取结论。

14开爪后多组WAIT_STABILITY，存档中中心跳动约9mm，少数组约0.33mm；图像方块可见，部分帧中心附近有效深度点偏移。保持原判据重采样后通过，完成15个关节阶段/17receipt含开闭爪，无lift。目标TCP=[617.958,−13.453,16.261]mm、目标法兰=[496.947,4.610,73.315]mm，宽51.8455mm、闭爪47.8455mm/0.5Nm，反馈48.9mm/约−0.38Nm，WAIT_CONTACT。末次关节[2.334,137.392,−83.597,23.544,−25.206,−16.878]度、SDK[496.881,4.762,73.069]mm，状态/故障0。日志/计划/17receipt已回传。用户未确认夹住，继续要求“将相机再向y-移动2cm并重新抓取”；已退出14并核实无控制进程，未改变识别/稳定性逻辑。

### 2026-11-14 相机沿基座−Y再移动20mm并重试

当前外参仅Y从+0.012901055071436609减0.020m，变为−0.007098944928563392m；X=0.4159578390386031、Z=0.05467993899103337与R完全保留。累计原始基座Y−60mm/Z+180mm，光轴仍平行基座+X。来源provenance/manual04-user-z180-y60-alignx-20261114.json链接上一轮y40父来源/哈希/逆变换；旧值备份evidence/extrinsics-before-yminus20.json，physical_validation仍NOT_PERFORMED。

更新现有回归先失败后通过，151实际pytest（67视觉+18语义+66boot/gripper）/4ctest包装通过，colcon155条目0错误0失败。三包构建、安装check与diff检查通过，32源码/安装来源一致。包SHA256=77889b7394e043336e7ed8d5d3d4f29203916a392db18ec7b090998c366cb84e。机载旧清单通过、备份/tmp/piper-before-extrinsics-yminus20-20261114.tar.gz后更新，34文件校验通过；semantic-extrinsics-yminus20-check-20261114-01=CHECK_OK。源/机载外参SHA256一致fa0ecdd3281a4ef4a4b6a09280badf993b98aa217316574b2f78e7e51db4aae1。已发5秒提示音，无提交。

从14末次反馈纯离线核对上抬100mm十段PLANNED，evidence/red-reobserve-raise100-06.py复用既有控制；semantic-red-reobserve-raise100-20261114-06已启动，先单次开70mm/1Nm再5%上抬，从新鲜关节重新规划。随后沿用25°、height0配置和新外参重新现场识别夹取。实际结果待追加。

semantic-red-reobserve-raise100-20261114-06完成，SDK Δ=[+0.004,+0.110,+99.716]mm，末次法兰[496.885,4.872,172.785]mm，关节[2.334,125.236,−86.756,51.014,−12.551,−45.721]度，开口68.2mm、状态/故障0，11receipt已回传并断言核实。随后启动semantic-red-grasp-20261114-15，使用height0配置/25°/5%及新相机Y=−7.0989mm，现场首轮识别规划与OPEN_READY通过，正在开爪后再次取图；本轮结果待追加。


### 2026-11-14 第15轮结束；按用户要求六轴回零并终止

15完成21个关节阶段、23个receipt（开爪+21关节+闭爪），无lift。目标TCP=[643.958,−42.793,7.800]mm、法兰=[523.720,−65.436,64.853]mm；测得宽51.877mm，闭爪目标47.877mm/0.5Nm，反馈49.0mm/约−0.40Nm，WAIT_CONTACT，未确认夹住。末次关节[−10.341,143.969,−96.160,−50.721,−24.852,38.789]度，SDK法兰[523.755,−65.476,65.285]mm，状态/故障0。证据保存在evidence/semantic-red-grasp-20261114-15。用户观察怀疑旋转矩阵问题、需要重新标定；此为待验证假设，未将其写成已确认根因。已退出15等待。

最新用户明确“将机械臂所有关节都回零并终止，后续重新标定”。核实无抓取进程后复用return-zero02流程，仅更换run_id为semantic-user-return-zero-20261114-03；脚本evidence/return-zero03.py，从新鲜反馈以5%速度、每段不超过10度分15段回零，没有开闭爪或零点重标定命令。离线分段断言通过，现场15段AT_TARGET与15个joint receipt全部核实。程序退出后另开纯只读反馈会话，连续5组六轴反馈均为[0,0,0,0,0,0]度、motion_status=0、err_code=0，夹爪开口49.0mm；关节仍使能保持，不等同断电。证据含before/plan/events/result/attempts/postcheck，位于build/piper-vision-semantic-20261114/evidence/semantic-user-return-zero-20261114-03。

回零进程退出码0；本轮模型服务与18765反向SSH隧道已终止，机载核实无抓取/回零控制进程、18765无监听。当前外参保留y60/z180/alignx版本，未再次修改或执行重新标定；后续重新标定需另行继续。无产品代码修改、无构建/提交，保留全部既有修改和历史标定。


### 2026-11-17 开始新手眼标定，等待CAN恢复和板完整入画

用户要求重新手眼标定，并确认原ArUco ID25、黑方块横纵142mm、已平整刚性固定，可手动摆姿态。沿用已有手动逐张review/approve，计划独立recalib20261117批次10train+5validation；两端核实批次未用，不混入manual04/manual05及人工调参结果。当前外参和机器人控制代码保持不变，没有发送运动/模式/使能/夹爪命令。SSH到unitree-go成功；机载check日志run-HzX7JAHw.jsonl为CHECK_OK，相机151222077264、板配置一致。

通过原review.snapshot以自定义本地保存根目录build/piper-handeye-recalib-20261117/reviews拍摄pose01，远端reviews/recalib20261117/pose01、日志run-9aLTrhcG.jsonl。图片已回传并实看，板上半部超出画面、ID25检出0；can0为DOWN/STOPPED，无位姿配对，review_issues两项，尚无正式批准样本。CAN TX采样前后0→0，USB序列0046002F5246570520323934正确。sudo -n true/-l均要求密码，不能自主完成接口恢复；不是自动审批拒绝。请用户在Go2终端配置can0为1Mbps并UP，同时手动摆姿态让黑色四角及白边完整入画，完成后再用pose02重拍。当前无持续采集进程，不重启带使能的boot服务。checkpoint.json保留上述事实。未改产品代码/构建/提交；旋转矩阵是否为抓取偏差根因仍待实测验证。

用户确认CAN已设置后，只读核实can0为UP/ERROR-ACTIVE/1000000bit/s，TX0。原review.snapshot完成pose02，远端日志run-IxzZ8Lxh.jsonl，图片与元数据保存build/piper-handeye-recalib-20261117/reviews/pose02。图像前后反馈完整、平移/旋转/关节跨度均0，ctrl_mode=2/teach_status=2/arm_status=0/err_code=0。位姿关节[−3.988,126.735,−113.820,−4.741,38.721,86.922]度，SDK XYZ=[512.160,−40.427,247.518]mm；夹爪宽反馈−28.2mm原样保留，未作为有效物理开口解释或修改零点。图像实看黑方块上边和右上角仍超出画面，ID25检出0，故未批准，正式样本仍0。采样后TX0，无运动/模式/使能命令。已请用户保持安装关系不变，手动调整整臂使板移远、在图中下移左移并静止，再采pose03；采集已退出，无持续进程，checkpoint更新WAIT_BOARD_FRAMING。

用户确认已移动后完成pose03，日志run-6tX73TtY.jsonl，原图/元数据保存同批次reviews/pose03。相比pose02板占画面已变小，但黑色标记上边及两个上角仍超出顶边；ID25检出0，未收录，正式数量仍0。图像配对成功，平移/旋转/关节跨度均0、反馈完整、err_code=0，SDK位置[540.588,−39.160,236.500]mm。TX仍0→0；无控制命令，采集正常退出。需继续让整个黑色方块在图中下移、上边外侧白边也可见，下一张pose04；checkpoint保持WAIT_BOARD_FRAMING。

用户要求打开RViz实时查看相机。核对无相机占用后，复用机载ROS Foxy既有realsense2_camera（v4.55.1/链接librealsense2.53.1）发布仅RGB 640x480@15，serial151222077264，关闭depth/infra/imu/pointcloud/TF；无机器人命令，CAN TX仍0。机载eth0 DOWN，当前链路eth1=192.168.1.100；发布进程仅临时Domain0/CycloneDDS绑定eth1，未改系统或协同配置。相机SSH会话93674，日志rviz-camera.log，进程4929/4964。

本机RViz默认GLX初始化失败；LIBGL_ALWAYS_SOFTWARE=1单独不足，加入__GLX_VENDOR_LIBRARY_NAME=mesa后OpenGL4.5/llvmpipe成功。远端X转发试验窗口未形成可用预览且已退出（会话3949退出143），本机前两次失败显示进程均已退出。enp4s0上有192.168.123.222及192.168.1.222，按网卡名绑定时跨机数据接收未通过；当前进程明确绑定同一enp4s0上的192.168.1.222后收到递增图像，未切换wlo1、未改路由。运行RViz会话43362，本机Display :1，配置build/piper-handeye-recalib-20261117/camera.rviz只含Image显示，不含导航/发布点工具。临时环境ROS_DOMAIN_ID=0、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp、CYCLONEDDS_URI=<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.1.222</NetworkInterfaceAddress></General></Domain></CycloneDDS>，LIBGL_ALWAYS_SOFTWARE=1、__GLX_VENDOR_LIBRARY_NAME=mesa、QT_X11_NO_MITSHM=1、MESA_SHADER_CACHE_DIR=/tmp/piper-rviz-mesa-cache；RViz日志rviz-mesa-address.log。

机载有限订阅验证5帧递增，本机10秒有限检查收到4帧（原断言要求5而超时，不记成5帧PASS），已独立核实保存4帧时间戳严格递增及640x480。RViz窗口截图rviz-window.png已实际看到彩色标定板画面；图像位于左侧Piper calibration live RGB，右侧3D空白是仅配置Image显示的预期。实时预览保持运行，正式样本仍0；继续pose04前必须退出本轮相机预览释放USB，再用原采集器配对图像和位姿。无产品代码修改、无构建提交；旧手眼与抓取外参保持。

用户要求RViz改为0.5Hz。已说明RViz只订阅ROS图像，USB竞争来自RealSense驱动与标定器同时直接开设备；降频不能释放相机，正式采集前仍须关闭预览驱动。两端无topic_tools，复用现有ROS依赖编写仅本轮使用的build/piper-handeye-recalib-20261117/preview_half_hz.py：订阅原RGB，2秒定时发布最新收到帧到/piper/handeye/preview，消费后清空，无新帧则WAIT_IMAGE，不持续重发旧帧，不接CAN。RViz配置改订阅该话题，名称Piper RGB - 0.5 Hz；原相机仍640x480@15，采样与生产代码不改。Python3.8语法及配置检查通过，无ROS产品包修改，因此未重建overlay。

原预览会话已退出：相机93674退出0，RViz43362退出134。本轮第一次恢复相机78836时驱动退出−11、转发87819退出255，首次有限验证无图像；证据日志保留，未记通过。随后仅重启本轮相机与转发，用nohup独立于SSH保持，远端launcher9055、camera9081、转发9056；日志/tmp/piper-half-hz-camera-retry.log和/tmp/piper-half-hz-preview-retry.log，PID文件同名前缀.pid。转发脚本/tmp/piper-preview-half-hz-20261117.py。当前本机RViz会话82312，沿用Mesa及192.168.1.222绑定，日志rviz-half-hz.log。

恢复后本机收到5张时间戳递增图像，4个到达间隔约1.833、1.929、1.982、2.012秒，有限检查通过，half-hz-verification-retry.json保存明细；RViz截图rviz-half-hz-window.png已实看有标定板画面。发布端2秒定时，网络存在抖动，不把到达间隔称为硬实时保证；驱动长期稳定性尚未验证。远端最终状态保存half-hz-final-status.txt。checkpoint已更新LIVE_RVIZ_PREVIEW_HALF_HZ。继续标定时先核实上述PID身份，SIGINT退出launcher及转发并确认camera9081已结束释放USB，再采pose04；不重用已结束的旧SSH会话号。正式样本仍0，未发送机器人控制命令、未改外参或提交。

### 2026-11-17 继续标定：预览已停止，pose04成功待人工确认

用户要求继续标定。核实本轮PID身份后关闭RViz82312；相机launcher9055/转发9056对SIGINT未退出，随后对9055/9056/相机9081发送SIGTERM，ps核实全部退出，释放USB。首次pose04采集run-L4abGql5.jsonl持续WAIT_INPUT/No device connected。用户按请求重新插稳USB后仍失败，停止该等待进程11175；此时无图像/样本生成，未覆盖旧数据。进一步独立SDK枚举报RS4XX: RGB modules inconsistency - 2 found；内核重复uvcvideo Found UVC记录不单独证明物理断线，根因仍未确认。系统rs-enumerate-devices（librealsense2.53）能识别正确151222077264，标定Python2.55.1一度不能创建设备。

核对设备节点002/003为D435后执行一次usbreset 002/003成功；紧接一次枚举仍失败，等待重新枚举后用原隔离解释器-I确认151222077264、Stereo Module及RGB Camera各一个。未修改SDK/标定代码/外参，未重启机器人或CAN服务。随后原review.snapshot重试pose04成功，日志run-vo4YV69D.jsonl，review_issues=[]、command_sent=false；本地build/piper-handeye-recalib-20261117/reviews/pose04包含原图和配对元数据。四个黑角和周围白边完整，ID25、重投影RMS0.197106px，平移/旋转/关节跨度均0、反馈完整、故障0，SDK XYZ=[533.861,−20.174,202.019]mm，CAN TX0→0。尚未人工收录，正式训练0/10、验证0/5；已向用户展示原图并询问OK/NO，checkpoint=WAIT_POSE04_APPROVAL。采集正常退出，无持续预览/采集进程。后续批准必须针对同张pose04和保存的同期位姿，不重新拍照替代；下一次新姿态编号pose05。

用户对pose04明确回复OK后，先核对原图/元数据SHA256与待审checkpoint一致，再调用原review.approve和loop.decide离线正式收录同一帧及同期位姿，未重拍或连接硬件。正式数据路径build/piper-handeye-recalib-20261117/dataset/pose04（本批次采用仓库内独立dataset，不在外部piper/handeye-data），review目录保留decision.json=OK/train。human_approved/role/来源图hash/command_sent=false及计数核实通过，当前train1/10、validation0/5；checkpoint=WAIT_NEXT_POSE，下一张pose05，待用户手动改变末端位置与朝向、保持板固定并静止后采集。未求解/部署外参，无代码修改/构建/提交。

用户确认已换姿态后完成pose05，日志run-PK2CMNzr.jsonl；原图和元数据保存build/piper-handeye-recalib-20261117/reviews/pose05。图像实看标记上边和两个上角出画，ID25检出0，因此本张不符合收录条件，未批准、原图保留。位姿配对正常，平移/旋转/关节跨度0，SDK XYZ=[526.488,−32.076,238.269]mm，err_code=0，command_sent=false，CAN TX仍0。正式进度保持train1/10、validation0/5；checkpoint=WAIT_BOARD_FRAMING、下一张pose06。待用户保持安装关系不变，将板在图中下移或适当移远，使上方白边也完整可见并静止后再拍。无代码/外参修改、无运动命令。

用户确认摆好后完成pose06，日志run-yOFZo3QW.jsonl，原图/元数据保存build/piper-handeye-recalib-20261117/reviews/pose06。实看四个黑角及外侧白边完整可见，ID25识别，review_issues=[]、重投影RMS0.701646px、静止窗口三类跨度均0，反馈完整/err_code0；SDK XYZ=[569.133,−43.055,216.691]mm，command_sent=false，CAN TX仍0。尚未批准，当前train1/10、validation0/5，checkpoint=WAIT_POSE06_APPROVAL，源图与元数据hash已冻结；待用户OK/NO后收录同一帧和同期位姿，下一新姿态pose07。采集退出0，无持续相机占用、无代码或外参修改。

用户明确“OK，已切换姿势，继续”：核对pose06两源hash后用原review.approve/loop.decide收录同帧train，正式进度train2/10、validation0/5。随后新采pose07成功，日志run-A19ochjp.jsonl，原图/元数据保存在build/piper-handeye-recalib-20261117/reviews/pose07。四个黑角和近邻白边完整，右上角距图右边约35px；ID25、重投影RMS0.212149px、review_issues=[]、静止窗口三跨度0，SDK XYZ=[574.724,−45.508,210.322]mm、状态/故障0。command_sent=false、CAN TX仍0。pose07待用户OK/NO，不提前收录；checkpoint=WAIT_POSE07_APPROVAL，源hash已冻结，下一新姿态pose08。采集退出0，无相机预览进程，无运动/外参/代码修改。

用户对pose07回复“OK，继续”，核对源hash后使用原review.approve/loop.decide离线收录同帧及同期位姿为train，正式进度train3/10、validation0/5，human_approved/role/command_sent及计数验证通过。尚未确认已换下一姿态，因此没有重复拍当前姿态；checkpoint=WAIT_NEXT_POSE，待用户手动换姿态并静止后采pose08。未SSH/相机采集/运动，未改代码或外参。

用户明确后续每次回复OK或NO后都会更换姿态，按此持续约定处理：OK/NO用于上一张决定，并表示下一姿态已准备，不再额外询问是否换姿态；仍由原采集器检查静止及图像质量，不自动批准下一张。当前已换姿态后完成pose08，日志run-cOEbD9dT.jsonl，四黑角及外侧白边完整、ID25、重投影RMS0.771762px，review_issues=[]、静止窗口三跨度0、反馈完整/err_code0，SDK XYZ=[586.793,−12.931,192.300]mm。command_sent=false，CAN TX仍0。原图与元数据保存build/piper-handeye-recalib-20261117/reviews/pose08，源hash冻结，待OK/NO；正式仍train3/10、validation0/5，checkpoint=WAIT_POSE08_APPROVAL、下一张pose09。无持续采集、运动或代码/外参修改。

收到OK后按持续约定，先核对pose08源hash并使用原approve/decide收录train，进度train4/10、validation0/5，再直接采集新姿态pose09。日志run-0T9Zre55.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose09。实看四黑角与邻近白边完整、ID25识别，重投影RMS0.027821px，review_issues=[]、平移/关节跨度0、旋转跨度约1.21e−6度（数值精度量级），SDK XYZ=[568.001,−18.070,203.546]mm，反馈完整/故障0，command_sent=false，CAN TX仍0。pose09待人工确认，checkpoint=WAIT_POSE09_APPROVAL，源hash冻结，下一新姿态pose10。单帧低重投影误差不代表整轮标定精度；尚未求解或修改外参，无运动/代码修改。

收到OK后核对pose09源hash，原approve/decide收录train，正式train5/10、validation0/5；按持续约定采pose10。日志run-FfGwxfQ6.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose10，四黑角与白边完整、ID25、重投影RMS0.848737px、review_issues=[]、静止三跨度0，SDK XYZ=[584.955,27.060,190.808]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose10待人工确认，checkpoint=WAIT_POSE10_APPROVAL、源hash冻结、下一张pose11。为增加姿态变化，提示后续在四角可见且固定关系不变前提下加入不同方向的倾斜；不新增收录门槛或自动运动。尚未求解/修改外参，无代码修改。

收到OK后核对pose10源hash，原approve/decide收录train，进度train6/10、validation0/5；按约定采pose11，日志run-LtlRhnWn.jsonl。原图/元数据保存在build/piper-handeye-recalib-20261117/reviews/pose11，实看四黑角及白边完整，ID25、重投影RMS0.443617px、review_issues=[]、静止三跨度0，SDK XYZ=[580.554,−93.823,192.529]mm，反馈完整/故障0，command_sent=false，CAN TX仍0。pose11待用户OK/NO，checkpoint=WAIT_POSE11_APPROVAL、源hash冻结、下一张pose12。采集退出0，未求解外参、未改代码、无运动命令。

收到OK后核对pose11源hash，原approve/decide收录train，进度train7/10、validation0/5；按约定采pose12，日志run-Q7Cz1Esc.jsonl。原图/元数据保存在build/piper-handeye-recalib-20261117/reviews/pose12，实看四黑角及白边完整，ID25、重投影RMS0.461814px、review_issues=[]、静止三跨度0，SDK XYZ=[596.468,−79.806,164.426]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose12待用户OK/NO，checkpoint=WAIT_POSE12_APPROVAL、源hash冻结、下一张pose13。采集退出0，无运动/代码或外参修改，尚未求解。

用户继续并回复OK后核对pose12源hash，原approve/decide收录train，正式进度train8/10、validation0/5。按约定采pose13，日志run-jWfhvPM7.jsonl；原图/元数据保存在build/piper-handeye-recalib-20261117/reviews/pose13。实看黑色标记上部被裁切，右上部贴近画面右边，ID25检出0，review_issues非空，故不符合收录条件，原图保留但未批准。静止三跨度0、反馈完整/故障0，SDK XYZ=[552.942,−122.577,211.141]mm，command_sent=false、CAN TX仍0。checkpoint=WAIT_BOARD_FRAMING、下一张pose14；请用户使板在画面中下移、左移或适当移远，四角及白边完整后再采。持续OK/NO后换姿态约定有效，无代码/外参修改，尚未求解。

用户确认摆好后采pose14，日志run-IfncZLrI.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose14。实看四黑角与周围白边均在画面内，左上黑角y=17px较靠近上边但未裁切；ID25、重投影RMS0.766735px、review_issues=[]、静止三跨度0，SDK XYZ=[566.539,−89.717,208.000]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose14待人工确认，正式仍train8/10、validation0/5；checkpoint=WAIT_POSE14_APPROVAL、源hash冻结，下一新姿态pose15。采集退出0，无运动/代码或外参修改，尚未求解。

收到OK后核对pose14源hash，原approve/decide收录train，进度train9/10、validation0/5；按约定采pose15，日志run-wxjdUfIJ.jsonl。原图/元数据保存build/piper-handeye-recalib-20261117/reviews/pose15，实看四黑角与邻近白边完整，右上黑角x619px距右边较近但未裁切；ID25、重投影RMS0.443350px、review_issues=[]、静止三跨度0，SDK XYZ=[572.269,−84.296,175.936]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose15为待确认的第10个train，尚未收录；checkpoint=WAIT_POSE15_APPROVAL、源hash冻结、下一张pose16。若用户OK则训练达到10后，后续新样本开始标记validation（共5个，不参与拟合）；若NO继续补train。尚未求解/修改外参，无代码或运动命令。

收到OK后核对pose15源hash，原approve/decide收录第10个train，正式train10/10、validation0/5；角色按预先约定切换到独立validation，后续不参与拟合。采pose16成功，日志run-QTQsiy8R.jsonl，原图/元数据保存build/piper-handeye-recalib-20261117/reviews/pose16；四黑角及白边完整、ID25、重投影RMS0.377171px、review_issues=[]、静止三跨度0，SDK XYZ=[602.850,−64.828,146.128]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose16待用户确认后收录validation（第1个），checkpoint=WAIT_POSE16_APPROVAL/pending_role=validation、源hash冻结，下一张pose17。采集退出0，尚未求解/部署外参，无代码或运动命令。

收到OK后核对pose16源hash，原approve/decide收录validation第1个，正式train10/10、validation1/5。按约定采pose17，日志run-jCQtcWAJ.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose17。实看标记右上角及右边部分出画，ID25检出0，未收录、原图保留。静止三跨度0，SDK XYZ=[565.628,−89.530,185.485]mm，反馈完整/故障0，command_sent=false、CAN TX仍0。checkpoint=WAIT_BOARD_FRAMING/pending_role=validation，下一张pose18，待用户使板在画面中左移或移远、四角及白边完整后继续补第2个validation。无代码/外参修改或运动，尚未求解。

用户确认摆好后采pose18，日志run-4JQHyI5g.jsonl，原图/元数据保存build/piper-handeye-recalib-20261117/reviews/pose18。实看四黑角与邻近白边完整，ID25、重投影RMS0.184838px、review_issues=[]、静止三跨度0，SDK XYZ=[562.021,−94.047,183.154]mm、反馈完整/故障0，command_sent=false，CAN TX仍0。pose18待人工确认后作为第2个validation收录，当前正式train10/10、validation1/5；checkpoint=WAIT_POSE18_APPROVAL/pending_role=validation、源hash冻结，下一张pose19。采集退出0，无代码/外参修改或运动，尚未求解。

收到OK后核对pose18源hash，原approve/decide收录validation第2个，正式train10/10、validation2/5；按约定采pose19。日志run-p61wKweW.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose19，实看四黑角与邻近白边完整，ID25、重投影RMS0.621651px、review_issues=[]、平移/关节跨度0、旋转跨度约1.21e−6度（数值精度量级），SDK XYZ=[566.397,−50.740,185.593]mm，反馈完整/故障0，command_sent=false、CAN TX仍0。pose19待用户OK/NO后作为第3个validation，checkpoint=WAIT_POSE19_APPROVAL/pending_role=validation、源hash冻结，下一张pose20。采集退出0，尚未求解/部署外参，无代码或运动命令。

收到OK后核对pose19源hash，原approve/decide收录validation第3个，正式train10/10、validation3/5；按约定采pose20。日志run-OS1nn43I.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose20；四黑角及邻近白边完整，底边距画面下方约30px，ID25、重投影RMS0.625169px、review_issues=[]、平移/关节跨度0、旋转跨度约1.71e−6度（数值精度量级），SDK XYZ=[564.427,−36.974,186.674]mm，反馈完整/故障0，command_sent=false、CAN TX仍0。pose20待用户OK/NO后作为第4个validation，checkpoint=WAIT_POSE20_APPROVAL/pending_role=validation、源hash冻结，下一张pose21。尚未求解/部署外参，无代码或运动命令。

收到OK后核对pose20源hash，原approve/decide收录validation第4个，正式train10/10、validation4/5；按约定采pose21。日志run-0DGL0XxG.jsonl，原图/元数据位于build/piper-handeye-recalib-20261117/reviews/pose21；四黑角及白边完整、ID25、重投影RMS0.161712px、review_issues=[]、静止三跨度0，SDK XYZ=[572.503,−15.904,216.036]mm、反馈完整/故障0，command_sent=false、CAN TX仍0。pose21待确认后为最后第5个validation，checkpoint=WAIT_POSE21_APPROVAL/pending_role=validation、源hash冻结。用户OK则完成10+5收集后转离线求解与独立验证，不再继续拍新姿态；若NO才补pose22。尚未求解/部署外参，无代码或运动命令。

### 2026-11-17 重新标定10+5收齐并完成离线求解

用户OK后核对pose21源hash，用原approve/decide收录最后第5个validation；训练10、验证5角色保持分离，未继续采下一姿态。调用原handeye/core.py solve（隔离解释器）完成PARK eye-to-hand求解，原件build/piper-handeye-recalib-20261117/result.json，归档docs/multi_agent/piper_handeye_recalib_20261117_result.json，两文件SHA256相同fad58acb3ee96f3f3a57b8e0c27b7bbcac900b4116c3ee82267a1585f6523656。报告docs/multi_agent/piper_handeye_recalib_20261117.md，复算命令在go2_runbook末节。

训练平移RMS2.131518mm/max4.012517mm、旋转RMS0.565564°/max0.816345°；独立验证平移RMS3.314926mm/max5.107529mm、旋转RMS0.704034°/max0.991083°。相机基座坐标[358.486967,38.146424,−56.567116]mm，完整R/t/逆矩阵/T_end_marker见结果。逐样本来源、OK决定、冻结hash、同帧同期位姿、身份配置及10/5角色计数全部核实；15图像hash/采集时间不重复，正逆刚体变换及全部逐样本残差独立复算通过。旋转激励奇异值[0.569638,0.370346,0.117814]rad、最大相对机器人旋转28.05°；去掉任意一个训练样本重算，外参位置最大变化8.683683mm、旋转最大0.974393°，保留完整10训练最终解，不按验证结果删样本。证据audit.json/analysis.json/residuals.csv/SHA256.json保存在本轮build目录。

用户补充“因为相机重新固定过，旋转外参变化是正常的”。按本轮采样前重新固定理解，并已向用户说明这一理解；本轮内安装关系不变仍是采集假设。报告已修正：新旧外参属于不同安装状态，新R与旧manual04差15.341°、与手动修正抓取R差5.642°不作为旧矩阵有问题的证据；历史13.164mm/6.284°与本轮3.315mm/0.704°仅比较各批次内部一致性，不能单独归因算法或证明绝对精度提高。若后续发现本批次内也移动安装，须重新审视样本是否可合并。

checkpoint=CALIBRATION_SOLVED_OFFLINE；physical_validation=NOT_PERFORMED，未实物点位/抓取或TCP验证。当前抓取外参SHA256仍fa0ecdd3281a4ef4a4b6a09280badf993b98aa217316574b2f78e7e51db4aae1，原manual04哈希仍b8f843b4aee21b530e53c642d1ceaa16868c7680b16610504157135596433151；均未覆盖或部署。无产品代码变更/构建/提交，无新SSH/运动命令；本轮仅离线收录、求解和文档。

### 2026-11-17 新标定空间3D人工检查

按用户“将空间位置3D模型可视化，人工检验”，在build/piper-handeye-3d-20261117生成piper-handeye-spatial.html会话交互图、piper-handeye-browser.html内嵌Plotly3.7.0离线浏览器版，以及同名PNG/PDF静态双视角图。沿用旧诊断URDF网格处理，仅离线读取新结果和15个已批准样本；默认pose21，可切换15姿态。相机光学中心[358.486967,38.146424,-56.567116]mm；显示内参视锥、板观测/推算轮廓与法向，pose21中心差1.99mm/朝向差0.41°。负开度不可当作真实开口，省略link7/8可动夹指；未加旧135mmTCP或虚构安装支架。8mm网格聚类只用于显示。

scene.json/evidence.json/README.md记录数据、来源哈希和限制；15组变换与残差、142mm板边长、视锥投影、全部姿态交互切换和状态恢复、Plotly数据结构检查通过；URDF-FK与现有运动学一致，URDF与SDK最大差0.13060mm/0.004493°。PNG已实际查看，Firefox环境启动失败，WebGL未实际检查。渲染静态图使用/usr/bin/python3 -I，co-nav-real的matplotlib/mpl_toolkits混用会报错，未改环境依赖。无产品代码改动/ROS构建/提交/部署/SSH或真机命令；当前抓取外参保持原状，physical_validation仍NOT_PERFORMED，等待用户对照实物检查图像。

### 2026-11-17 用户授权应用外参并尝试夹取，已部署，等待退出示教

用户要求“给出外参的数值并应用，然后尝试夹取”，明确授权本次配置部署与一次真机抓取。已将完整新T_base_color_optical应用到源码和unitree-go:/home/unitree/piper-vision-semantic-20261114/config/piper_vision_grasp/extrinsics.json，不叠加旧人工偏移。新配置SHA256=10f4208f58454784d5e01113bd7797da3c8206731b6e7c37d189ad333d90b359，source_file=provenance/handeye-recalib-20261117-result.json，source_sha256=fad58acb3ee96f3f3a57b8e0c27b7bbcac900b4116c3ee82267a1585f6523656，与docs归档原件相同。平移[358.486967,38.146424,-56.567116]mm，R用新解完整精度。旧配置/来源/历史日志均保留，远端备份/tmp/piper-before-recalib-20261117.tar.gz已取回本地evidence。

新输出build/piper-handeye-applied-20261117/overlay，三包构建通过，安装入口/33个配置与来源文件逐字节一致；86项视觉/语义测试、66项原底层基线通过，4项ctest包装通过（汇总156含包装）。最初有1项写死旧人工参数的测试失败，保留其历史断言并改读历史来源，新增当前新解完整加载测试后通过。机载SOURCE_MANIFEST/SHA256SUMS只更新新外参与来源并核验通过；机载height0配置保持e841523dfeaf6090f465459b26e92c9c4064555c682a3f8bb78a8dd1ff547905。安装check及机载recalib-applied-check-20261117-01均CHECK_OK，机载实际加载证据remote-applied-check.json保存在本轮evidence。未提交。

现场SSH可达、can0 UP、无活动Piper/相机进程。只读反馈关节[-5.295,-0.128,-35.105,12.474,47.835,111.867]度，SDK[-14.679,15.992,328.870]mm，err_code0、motion_status0、全使能，但ctrl_mode2/teach_status1，WAIT_CONTROL（示教录制）；夹爪反馈-16.1mm不可当作真实开度。已分别发异步问题确认“末端标定板取下、红色方块就位”和“退出示教/静止”，目前待答。没有开爪、移动关节、切换控制模式或启动run，尚未进行本轮夹取。下一步收到现场确认后重新只读核对状态，恢复模型服务与反向隧道（目前18765无监听），继续已有height0配置/红色方块/25°/5%/开爪1Nm/闭爪0.5Nm重新取图规划一次抓取。模型启动助手已准备evidence/start-model.py，使用新安装路径、旧模型环境与凭证内存读取方式，尚未执行；不打印凭证。physical_validation仍NOT_PERFORMED，TCP135mm仍未经实测标定。

### 2026-11-17 开始红色方块尝试，控制已恢复，云端图像授权待确认

用户继续要求“现在开始尝试夹红色方块”。只读状态已从示教录制1变为STOP_RECORDING2，机械臂接近零位，但仍ctrl_mode2。开发机模型服务service-recalib-20261117-01和SSH反向18765隧道启动，recalib-red-scene-20261117-02三帧识别DETECTED；实看红方块在纸箱上，点color=[0.01461846,0.01846326,0.17900001]m、夹取宽51.99066mm。首次scene01预先创建目录与capture_rgbd的exist_ok=False冲突，空目录保留，scene02已修正调用方式；无产品代码改动。

现场初始化均先保存单次receipt：recalib-can-mode-20261117-01发送MotionCtrl_2(1,1,5,0)，随后观测为STANDBY/teach_status2、六轴和夹爪未使能，未据此判定变化根因。recalib-control-init-20261117-01依次发送MotionCtrl_1(0,0,0)、EnableArm(7)、MotionCtrl_2(1,1,5,0)，确认READY/ctrl_mode1/teach_status0/六轴与夹爪使能/故障0；未发送JointCtrl或GripperCtrl目标。末次关节[0.674,-0.155,-0.891,2.609,7.083,-6.086]度，SDK[55.502,1.164,207.457]mm，开度反馈0mm。

启动recalib-red-grasp-20261117-01，height0配置/红色方块/25°/5%/新外参，诊断窗口已打开。模型服务原进程退出143（原因未确认），抓取WAIT_VISION、Connection reset by peer、command_attempted=false；attempts为空，无开爪/抓取关节/闭爪/抬升。一次沙箱内脱离会话重启未存活、无日志；随后请求宿主环境启动模型服务被自动审批拒绝，理由是从既有代码读取API凭据并把现场图像发至外部DashScope缺少明确授权。已向用户说明自动审批拒绝，并发异步问题请求明确允许“读取既有凭据+现场图像发送DashScope qwen3-vl-plus”；没有绕过拒绝或换通道重启。

为避免无人等待期间持续采集，已向本次远端等待进程PID18470发送SIGTERM退出，仅停止监测/采集，不发送物理停止、释放或去使能命令。现场与模式切换日志已回传build/piper-handeye-applied-20261117/evidence；场景原图red-scene-before.png、完整scene02及run01证据保留。当前尚未夹取，新外参未再修改。用户允许外部图像处理后才可重试宿主模型服务；准备的start-model02.py仍指向未成功创建的service-recalib-20261117-02，执行前核实目录与端口，不能覆盖旧run。反向隧道工具session62804已启动，后续核实存活，不重复绑定。后续使用新的抓取run-id重新取图执行，不重用预览坐标；此次物理抓取授权尚未消费为完成动作。未改产品代码/构建/提交，physical_validation仍NOT_PERFORMED。

### 2026-11-17 授权云端识别后执行，碰撞相机，已急停终止

用户明确回复“允许”，授权本次读取既有模型凭据并将现场相机图像发送DashScope/Qwen。宿主模型service-recalib-20261117-02启动成功、MODEL_READY/device=cuda，既有反向18765隧道可达。重新核实控制READY/CAN1/teach0/故障0及新外参来源哈希一致。recalib-red-grasp-20261117-02重采识别成功，但起始J2=-0.155°超出模型下限0°，WAIT_PLAN/joint limit violation，零抓取receipt，随后退出等待。使用既有PiperSession/send_once单次recalib-entry-joint2-20261117-01把J2设为+1°，其余关节目标保持原反馈；实际到位0.903°、随后0.990°，无零点或限位修改。

recalib-red-grasp-20261117-03按height0/红色方块/25°/5%/开爪1Nm/闭爪0.5Nm和新外参重新采集规划。开爪稳定68.2–68.3mm，第二次定位宽51.7015mm，模型TCP=[533.458,18.448,-91.752]mm、目标法兰=[411.249,24.336,-34.699]mm。计划27段（unloaded14、approach11、lift2），最终未进入approach。13个entry子段AT_TARGET；第14个entry目标[7.802822,134.499686,-37.938546,10.432114,-66.538481,0.312901]度、法兰[320.723,28.697,7.563]mm时出现WAIT_FAULT：J5 motor_overheating/driver_error_status，随后collision_status。首故障反馈J5=-74.059°且J5使能false。该run共15receipt=开爪+14个entry关节目标，没有close/approach/lift receipt，没有夹取成功。

已SIGTERM退出run03防止故障恢复后自动续行，并在控制锁释放后单次MotionCtrl_1(1,0,0)急停，receipt与3秒反馈保存recalib-fault-stop-20261117-01；收到ctrl_mode0、arm_status1/EMERGENCY_STOP、J5未使能。后续4秒纯只读12个快照关节跨度全0，最终关节[7.915,146.666,-48.585,8.914,-96.153,0.396]度、SDK[353.942,35.058,-0.682]mm，夹爪68.4mm。motion_status1不能等价解释为仍运动或已停，以上只证明该4秒编码器无变化。J5低速反馈电压23V、驱动51℃、电机47℃、过热/碰撞/驱动错误标志true、使能false、电流0A；未凭标志单独确定热故障根因。没有清错、重新使能、松爪、抬升、回零或重试。

用户现场明确“碰到了相机”，确认相机干涉；不再归为单纯识别/外参偏差。相机安装可能被撞移，当前外参有效性待现场固定关系核查，不能据旧离线残差继续宣称有效；尚未确认相机是否实际移位。本轮尝试终止，下一步需检查相机/机械臂及重新规划避让，未经用户新指示不解除急停或动作。完整run02/entry-J2/run03/stop证据及fault-passive-after-stop.json位于build/piper-handeye-applied-20261117/evidence，当前外参未再变更，无产品代码改动/构建/提交。本轮一次物理抓取尝试已结束，未完成闭爪，physical_validation仍NOT_PERFORMED。

收尾已精确匹配本次service-recalib-20261117-02与18765专用反向隧道进程，SIGTERM关闭宿主PID25607和24870；未触及其他服务或发送新机械臂指令。后续恢复服务仍须使用新run-id并核对端口，已有本次图像外传授权不等于下一轮动作授权。

### 2026-11-17 用户授权保持外参、调整避让后重试；代码已部署，现场仍待结束示教

用户要求“假设相机没有移位，重启”，经范围澄清明确选择“调整避让路径后，重新尝试抓取”。按该假设保留当前新外参，不要求重新标定；该假设不是相机实际未移位的测量证据。本轮新重试已授权，仍沿用已获允许的既有模型凭据及DashScope图像识别方式，不需重复询问云端授权。

先读代码及旧run03证据，确认旧entry_clearance_m=0、沿工具反Z退100mm的关节入口穿过相机。离线9组高度、10组姿态扫描后，采用60°、基座Z预抓100mm、入口额外Z100mm（相对最终抓取法兰高200mm）。25°仅抬高入口仍在下降末段进入相机区域，未采用。audit_mesh.py以URDF完整STL、70mm开爪、每段<=0.5°采样，保守相机区域为光学坐标X[-100,100]、Y[-35,35]、Z[-40,25]mm；该区域基于官方D435 90×25×25mm并覆盖未知光学中心到外壳偏置，不是实测支架。旧路径234个link/pose重叠记录，25°竖直候选35个；60°356姿态无重叠，采样顶点最近65.5705mm（65°76.6677、70°87.6746mm）。这是离线采样和包围假设，不证明真实连续净空或未知支架/线缆安全。

产品最小修改tools/piper_vision_grasp/geometry.py的plan_absolute：两个可选semantic参数pregrasp_offset_base_m、entry_clearance_m；缺省仍原工具反Z/入口0，配置形状/有限数值校验，未增加运行审批或运动门控。独立新配置config/piper_vision_grasp/semantic-camera-avoid.json（红色方块、60°、height0、上述垂直参数），保留旧semantic/height0文件和所有外参。5项新增真实IK/非法配置测试先失败后通过；视觉/语义共91、底层基线66通过，三包构建成功、4项ctest包装通过（汇总161含包装），Python3.8语法及Bash入口检查通过。

新安装build/piper-camera-avoid-20261117/overlay/install/vlm_nav/share/vlm_nav，34文件/source匹配；机载仅更新geometry.py、新配置、manifest/SHA256SUMS，旧文件备份/tmp/piper-before-camera-avoid-20261117.tar.gz已回传。开发端install-check01及机载camera-avoid-check-20261117-01均CHECK_OK，remote-check-and-feedback.json确认机载模块/配置hash匹配安装产物且实际加载60°/0.1m/0.1m。主runbook及抓取手册已更新新配置入口；不得沿用旧CLI --approach-pitch-deg 25。模型启动助手build/piper-camera-avoid-20261117/evidence/start-model.py已准备但未启动；模型和隧道仍关闭。未提交。

证据/扫描/全STL审核/失败与通过测试/构建均在build/piper-camera-avoid-20261117/evidence，旧新路径图camera-avoid-paths.png已实看。图的起点是旧run03有效起点，目标是旧采样目标，不是当前姿态。实施计划docs/superpowers/plans/2026-11-17-piper-camera-avoidance.md，现场恢复/新鲜规划/执行两项未完成。

已异步请求用户结束示教并确认人员离开活动范围，尚待回复。本轮没有发送CAN控制、清错、使能、解除急停或新运动。最新只读（remote-check-and-feedback.json，time1794912844.2845917）：关节[1.303,182.648,-174.208,-1.781,54.731,-3.107]度、SDK[579.835,10.881,1.026]mm，ctrl_mode0、arm_status1急停、teach_status1录制、J5未使能，J2/J3超当前模型范围；早先只读J5仍过热/碰撞/驱动错误标志，驱动44℃/电机38℃，不能仅据温度清除故障解释。需要用户结束示教并整理合法起始姿态后重新读取；不能夹断实际角度或放宽限位假装可规划。恢复时已查SDK JointConfig(5,0,0,500,0xAE)可清关节错误但尚未调用，MotionCtrl_1恢复/EnableArm也未调用；严禁用SDK Reset（会失电下落）代替恢复。用户确认后按实际状态选择恢复步骤，重新采新RGB-D和真实关节进行preview及全路径检查，再执行本次一次抓取。提示音15秒已按项目要求播放。


### 2026-11-17 用户要求六关节回零，已完成并保持使能

用户将当前任务切换为“将机械臂回零”，随后明确确认“已结束示教，机械臂已可靠支撑”。首次只读实际仍为急停/teach1、J5未使能，关节[-2.055,-0.069,0.341,-7.716,1.134,2.965]度且3秒跨度0。纠正上一节恢复说明：SDK0.6.2的EmergencyStop(2)和ResetPiper实际都调用MotionCtrl_1(2,0,0)，后者明确注明可能立即失电下落，不能视为不同恢复机制。已向用户说明该风险并取得上述现场支撑确认后，才单次执行恢复；没有重设关节零点。

recalib-user-zero-recover-20261117-01持有控制锁并在发送前持久化receipt，单次MotionCtrl_1(2,0,0)。恢复中短暂出现通信错误63，随后故障全部清除、teach0/standby0/六轴未使能，位置仅小幅变化至[-2.001,-0.069,0.417,-7.621,1.207,2.961]度；未额外调用JointConfig清错。recalib-user-zero-enable-20261117-01单次EnableArm(7)，确认READY、六轴使能、故障0、位置稳定。

复用历史回零脚本，离线确认当前姿态仅需一个全零目标、最大变化7.621度，5%速度。recalib-user-return-zero-20261117-01发送一次后进入CAN1，但约20秒实际角度无变化/无新增故障；已结束该等待进程，保留receipt，未在同stage重放。控制模式稳定后新run recalib-user-return-zero-20261117-02单次发送全零目标，AT_TARGET、ZERO_REACHED，进程退出0。初次未动作可能与模式切换时序有关，根因未证实，不据此修改产品代码。

独立纯只读15快照再次确认六轴[0,0,0,0,0,0]度、跨度全0、READY/CAN1/teach0/arm_status0/err_code0、六轴使能。SDK XYZ=[56.127,0,213.266]mm，夹爪68.4mm保持，未发GripperCtrl、未重置零点或修改外参。此轮仅回零，未继续抓取；模型与图像服务未启动。全部四run的receipt、日志和verified.json已取回build/piper-camera-avoid-20261117/evidence。无产品代码改动、无需重建、未提交；未来抓取仍需新鲜目标与避让路径验证，不把本次回零视为抓取或相机位置有效性验证。


### 2026-11-17 红色积木抓取：40°避让已执行，未夹住，用户指示相对修正

用户要求“抓取前方红色积木”。开始反馈六轴0°、READY/CAN1/teach0/全使能/故障0；camera-avoid-red-check-20261117-02为CHECK_OK，机载外参/geometry/新避让配置SHA与源码相同。模型启动再次遭自动审批拒绝，理由是先前“允许”的具体上下文不足；未绕过，用户随后明确“允许读取既有凭据并向DashScope发送本次相机图像”，再启动service-camera-avoid-20261117-01（CUDA）及专用反向18765隧道。此轮云端授权明确有效。

camera-avoid-red-preview-20261117-01识别目标TCP约[658.059,5.718,-33.156]mm，比旧目标远约125mm；60°绝对IK失败，未发运动后退出该preview。离线扫描16个姿态，40°完整规划可达且全STL418姿态相机区域重叠0；只以CLI覆盖approach-pitch-deg=40，保留基座Z预抓100mm/入口再100mm，未改外参或默认配置。新鲜preview02为PREVIEW_READY，0 CAN控制发送；其路径重新审查418样本重叠0、采样顶点最小距离131.54mm（非实测/连续表面距离，支架线缆仍未建模）。

camera-avoid-red-grasp-20261117-01完成开爪68.4mm、重采定位、35个关节段到位和闭爪，37个receipt（open+35joint+close），无lift receipt。新宽52.3016mm、闭爪目标48.3016mm，实际49.5mm，保持WAIT_CONTACT；差1.1984mm未满足既有2mm非空接触条件。全程无WAIT_FAULT，SDK最终[554.823,-0.132,53.581]mm，关节[-0.466,149.866,-111.561,22.273,7.304,-24.505]度。用户明确“没有夹住，夹指位置有偏差”，故此轮未成功；已SIGTERM结束自动run，EXITED/previous_state WAIT_CONTACT，无抬升/松爪/去使能指令，独立12快照关节跨度0/READY/故障0。

close-inspection有限三帧已实看，夹指在目标前方的判断得到深度采样支持，但不作为精确修正量：红面ROI中位295–296mm、左指尖约227–228mm、右指尖242–243mm（右ROI有效率仅约33–43%，不据此校准TCP/外参）。完整run/preview/图像/深度/receipt已取回build/piper-camera-avoid-20261117/evidence。用户进一步指示“需要向上移动2cm左右，再向前3cm左右，不一定精确”。按Piper基座+Z20mm后+X30mm、朝向保持计划一次相对修正，先开爪再移动后同0.5Nm闭爪；未改外参。操作脚本red-relative-correction.py复用原IK/单次发送/到位与开闭爪检查，离线7段（含尚未执行的2段抬升候选）误差检查通过、61采样相机包围区域重叠0。已启动camera-avoid-red-correction-20261117-01，待记录结果；脚本只到接触候选，不自动抬升。无产品代码修改、无重建或提交。


相对修正01已执行开爪、5个1cm插值关节段及同目标闭爪，7个receipt、无lift；实测SDK增量[29.807,-0.269,19.512]mm，最终SDK[584.630,-0.401,73.093]mm、关节[-0.466,155.989,-129.617,8.452,19.018,-10.433]度，READY/故障0。闭爪仍49.5mm/WAIT_CONTACT，用户回复“仍有位置偏差”。已结束单次脚本PID25042，只退出监控、不松爪/去使能/抬升；脚本没有SIGTERM结果处理，SSH255不能当成功，使用operator-record.json明确记录人工未夹住及退出事实。close-inspection三帧已采集实看并取回，夹指仍在红块前侧但单视角不能据此确定精确修正。已异步询问剩余上/下、前/后、左右方向及厘米数，待答；未继续猜测移动或修改外参。模型服务和18765隧道目前仍运行、无持续相机采集，供本次未完成抓取后续使用。


用户补充“向上一点”，按小幅5mm基座+Z、其余位置/朝向及夹爪不变执行camera-avoid-red-up5-20261117-01。复用旧raise脚本，离线单段IK和位姿增量断言通过；这一小段在之前已检查的20mm抬升候选内。单个joint receipt，进程退出0；此处stage名lift是历史raise脚本命名，实际为用户要求的位置修正，不是夹住物体后的验证抬升。独立只读12帧复核最终SDK[584.668,-0.408,78.216]mm，相对上轮高度+5.123mm，关节[-0.466,155.475,-129.740,8.173,19.658,-10.171]度，READY/故障0/全使能，夹爪仍49.5mm。未发新的开闭爪、无外参或TCP改动。证据/verified.json已回传同evidence下run目录。已询问高度是否对准、物体是否在两指之间或仍需向前，待现场回复；抓取仍未证实成功，不自动追加修正。


### 2026-11-17 后续前移、移物重定向及外参候选记录（未应用）

用户确认高度已对准、仍需前移后，执行camera-avoid-red-forward10-20261117-01：开爪后分两段前移10mm，再闭爪；29采样相机区域重叠0，实测SDK[594.640,-0.519,78.721]mm、关节[-0.466,159.575,-138.575,6.612,24.357,-8.521]度，夹爪约49.6–49.8mm、仍WAIT_CONTACT、无故障。用户反馈“仍需向上抬1cm且向前移动约5cm”。离线当前朝向解：+Z10/+X10mm可达、+X20mm起失败；+X50mm位置残差34.086mm/朝向4.796°，未发送此修正，也未部分执行。结束forward10等待进程25969，退出143不是成功，operator-record.json保留人工未夹住事实。随后用户确认已将积木移近并放稳、手已离开。

camera-avoid-red-moved-scene-20261117-01纯只读新鲜三帧识别，与forward10闭爪检查帧比较，目标位移base=[-62.907911,+10.144599,+0.809884]mm。将用户剩余修正[+50,0,+10]mm与该位移相加，得到当前机械臂相对修正[-12.907911,+10.144599,+10.809884]mm，保持现有朝向；这是人工引导的局部重定向，不改相机外参。新宽50.3888mm、闭爪目标46.3888mm/0.5Nm；40采样相机区域重叠0。camera-avoid-red-relocated-20261117-01完成开爪、4段位置调整及闭爪，实际SDK[581.839,9.459,89.539]mm、关节[0.634,153.348,-127.879,5.642,19.694,-6.939]度，夹爪47.6–47.7mm/WAIT_CONTACT、故障0。用户明确“当前位置闭爪可以抓住。据此尝试记录修正外参的偏移量，先不应用”。该表述记录为人工认可当前可夹取位置，不扩写为已夹紧或验证抬升成功。结束等待进程27188，未抬升、未再发开闭爪或去使能。

camera-avoid-red-aligned-record-20261117-01完成当前位置有限三帧及前后机器人反馈，识别DETECTED、关节前后变化全0、READY/故障0/夹爪47.6mm；图像已实看。基于当前关节FK、现有135mm法兰TCP偏移、同期目标点及原R，计算delta_t = modeled_TCP_base - observed_point_base。候选平移增量base=[+77.816491,+5.641927,+33.324299]mm，原相机平移[358.486967,38.146424,-56.567116]mm，候选[436.303459,43.788351,-23.242816]mm；旋转逐元素保持不变。候选完整矩阵/公式/关节/图像路径和SHA/三帧一致性/局限已存docs/multi_agent/piper_handeye_translation_candidate_20261117.json，status=MANUAL_ALIGNMENT_TRANSLATION_CANDIDATE_NOT_APPLIED、applied=false。计算自检（矩阵合法、旋转不变、点补偿等式、配对反馈一致、来源hash）通过；三帧跨度0仅代表该采样一致，不代表标定精度。单点无法分离外参/TCP/接触点/运动学误差，135mm TCP未实测，候选不是新的手眼标定解。

运行外参源码及机载SHA256仍为10f4208f58454784d5e01113bd7797da3c8206731b6e7c37d189ad333d90b359，未改config/安装外参、未部署候选。现有40°仅本次CLI覆盖，默认新避让配置仍60°。全部证据位于build/piper-camera-avoid-20261117/evidence；记录脚本record-translation-candidate.py可离线重算。模型服务32393和专用18765隧道32474已精确关闭，无持续相机采集，机械臂保持末次姿态及夹爪目标。无产品代码更改/重建/提交；抓取与抬升成功仍未验证。后续若要应用候选须用户另行指示，不能把本次“先不应用”遗忘。


### 2026-11-17 用户授权应用平移候选并回零，均已完成，等待重新摆放

用户明确“应用，然后机械臂回零，我会调整方块位置，重新夹取”，撤销前轮仅记录不应用限制；本轮范围为应用+回零，重新抓取依赖用户摆好后告知。保留旧候选JSON的历史内容及原手眼解，新增config/piper_vision_grasp/provenance/handeye-manual-translation-20261117.json保存授权、完整候选、原矩阵/来源哈希及局限。extrinsics.json完整精度采用记录矩阵，平移增量base=[77.816491,5.641927,33.324299]mm，现相机平移[436.303459,43.788351,-23.242816]mm，旋转逐元素不变，physical_validation仍NOT_PERFORMED。新extrinsics SHA256=710f7781bcbab5f86844b1701adb912bb47a8cb81d16ba1ebf94d3aa0d4f379a，不重复叠加偏移。

新增配置回归先失败后通过，验证记录矩阵完整加载、只加一次平移、旋转不变和记录点补偿关系；原手眼完整矩阵/来源测试保留为历史验证。视觉/语义92项、底层原基线66项、4项ctest包装通过（colcon162含包装），三包构建成功，未运行无关全库测试。独立输出build/piper-translation-applied-20261117/overlay，35个安装运行/来源文件与源码匹配，安装check01为CHECK_OK。构建有原CMake开发/未使用参数警告，无构建或测试错误。主runbook与抓取手册已更新新输出及加载来源。

机载仅从成功安装产物更新extrinsics及新来源、SOURCE_MANIFEST/SHA256SUMS，部署根保持/home/unitree/piper-vision-semantic-20261114，解释器保持/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python。机载旧文件备份/tmp/piper-before-translation-20261117.tar.gz已回传，本地也有extrinsics-before.json。机载完整校验清单通过、translation-applied-check-20261117-01为CHECK_OK，最新manifest installed_root指向本轮产物。默认抓取角60°未改，旧height0配置未覆盖。

开始状态READY/全使能/teach0/故障0。回零前离线按当前真实关节[0.634,153.348,-127.879,5.642,19.694,-6.939]计算16个<=10°关节段，分别按原/新相机位置进行完整STL336姿态采样，均无包围区域重叠；采样不覆盖未知支架/连续表面运动。translation-applied-return-zero-20261117-01先单次open70mm/1Nm（避免带动物体）再5%速度16段回零，共17receipt，无复位零点/清错/再抓取。进程退出0/ZERO_REACHED；独立12快照最终六轴[0,0,0,0,0,0]度、跨度全0、READY/CAN1/teach0/故障0/全使能，SDK[56.127,0,213.266]mm，夹爪68.4mm。verified.json同时保存实际加载的新外参及SHA。机械臂保持零位，模型/隧道仍关闭，未启动相机或下一轮抓取。

全部测试/构建/备份/路径审查/机载check/回零与verified证据在build/piper-translation-applied-20261117/evidence。无提交。用户摆好红色积木并告知后，才重新采新图像/规划并检查路径；不得复用旧目标。新平移是单点人工对齐候选，仍含135mm TCP未实测等不确定性，后续新位置抓取尚未验证。完成提示音按项目15秒音频播放。


### 2026-11-17 新外参重新夹取失败：戳中弹走积木，偏左约3cm，已结束自动流程

用户“现在继续抓取”授权新摆放一次尝试。新模型service-translation-applied-20261117-01使用最新安装、CUDA和既有已明确允许的DashScope图像识别；专用18765隧道恢复。translation-red-preview-20261117-01使用新平移外参、40°CLI、原垂直预抓/抬高入口，PREVIEW_READY，TCP[634.770617,-25.694317,5.924650]mm、宽53.0969mm。按修正前及修正后相机位置各检查442姿态，全STL均0区域重叠；采样顶点最近分别152.67mm和82.43mm，不证明实物定位或物体接触安全。

translation-red-grasp-20261117-01开爪68.4mm后重采，目标点稳定、宽53.0569mm、闭爪目标49.0569mm/0.5Nm。完成35个joint阶段及open/close，共37receipt，无lift。程序WAIT_CONTACT，实际50.6mm、约-0.51Nm；无设备故障。用户明确“戳中了积木并将积木弹走了，夹取位置偏左了约3cm”，确认为碰到物体且抓取失败，不因READY/无驱动故障否认现场接触。

已SIGTERM结束run PID29038，result=EXITED/previous_state WAIT_CONTACT，未发新的运动、松爪、抬升、急停或去使能。独立12快照实际关节[-4.867,140.694,-102.057,47.427,9.959,-53.218]度、跨度全0，SDK[531.687,-33.647,92.926]mm，READY/CAN1/teach0/全使能/故障0，夹爪50.6mm。close-inspection有限三帧已采集实看，红块由原图右侧移到中部；该图像不再是旧目标。operator-record.json记录FAILED_OBJECT_DISPLACED、人工左偏0.03m，left_reference_frame=UNCONFIRMED，原新外参710f7781...保持，未直接叠加横向修正或宣称重新校准成功。

已异步询问“偏左”是相机画面还是观察者视角，以及积木是否重新放稳/手离开，待回复。未把未知左右直接映射成Piper Y正负。全部preview、run、图像、receipt、after-exit-passive证据在build/piper-translation-applied-20261117/evidence。模型服务及专用隧道暂保留供本次排障后续使用，无持续相机采集，机器人保持姿态。无产品代码/构建/部署/提交，本次应用候选在新摆放下未通过抓取验证。

### 2026-11-17 GraspNet 位姿后端接入：代码与构建完成，完整路径验收未通过

用户明确选择 GraspNet 决定位置、朝向和开度，其余流程不变；本轮范围是接入、真实离线测试、构建和文档，不部署、不启动真机。保留所有既有 dirty 修改，分支 go2-cooperative-task-layer、HEAD 323d5c52737477962a874ae012db117be5a93f34 未变、无提交。计划/裁决见 docs/superpowers/plans/2026-11-17-piper-graspnet.md；使用说明见 [GraspNet 手册](piper_graspnet_runbook.md)，构建命令在 go2_runbook 的同日 GraspNet 节。

新增 tools/piper_vision_grasp/graspnet.py、graspnet_server.py、graspnet_visualize.py；main 接入 prepare_plan/保存回放/serve-graspnet/visualize，geometry 仅新增后端配置检查，未改原规划与控制实现。新 semantic-graspnet.json 显式启用；旧默认入口和配置保持。18766 localhost 独立 GraspNet 服务，18765 Qwen/SAM 选物保持。三帧稳定后取末帧全场景米制点云，目标掩码沿用原 semantic.measure IQR 深度离群处理，采样20k、官方碰撞与NMS、目标关联、开口筛选、原完整路径IK。转换使用 GraspNet X→Piper工具Z、Y→Y，tip=t+depth*R[:,0]，现有外参及135mm TCP各应用一次。闭合目标恢复解码的1.2余量后减原4mm；速度/力矩/开口/接触/抬升等不改。无候选等待、不回退旧算法。

独立环境 build/piper-graspnet-20261117/model-venv，Python3.10/Torch2.7.1+cu118/NVCC11.8。baseline280c215129f759ed8649cb4e89fc5dfee55f4f80，APIeb57dd2092d8dbe05312a29c3d0c22f3226efbfc，原生扩展成功，无源码补丁。官方Drive下载超quota，采用两个公开项目同SHA的checkpoint-rs.tar镜像，SHA256=60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868；weights_only安全加载与网络strict匹配通过，未与官方原文件独立比对。config/piper_vision_grasp/provenance/graspnet-model-20261117.json记录来源限制，模型版本冻结 evidence/model-environment.txt。

最终新增22项+原有158项=180 pytest通过；三包构建成功，5个ctest包装通过（colcon185含包装），Python3.8语法、Bash、diff检查通过。新输出 build/piper-graspnet-20261117/overlay/install/vlm_nav/share/vlm_nav；39个运行/配置来源文件逐字节一致，installed-check-final=CHECK_OK。未切换旧导航或已部署机载产物。独立审阅两项Important均经RED→GREEN修复：缓存回放显式保存目标/拒绝CLI改色描述；规划冻结外参/TCP/模型配置供后续可视化，避免新标定误画旧计划。

真实离线证据在 build/piper-graspnet-20261117/evidence：replay-aligned-final 来自 camera-avoid-red-aligned-record-20261117-01/capture；replay-moved-final 来自 camera-avoid-red-moved-scene-20261117-01/moved-scene-701985fa；replay-failed-final 来自 translation-red-grasp-20261117-01/before-3e77a44a。三者原始候选216/151/121，非目标215/150/121，剩余目标候选前两组各1个超70mm开口，因此最终均0候选/WAIT_PLAN、退出1、CAN发送0；耗时0.145/0.124/0.337秒，峰值Torch显存662MiB。offline-summary.json明确INCOMPLETE，不能把推理跑通当完整路径或夹取通过。ROI30/100mm探索有少量候选但仍不可达，未纳入生产参数。没有为了验收修改外参/TCP/关节限位/开口。至少一组完整可行路径这一验收仍未满足，是后续模型与场景适配待办。

visualization-{aligned,moved,failed}-final/graspnet.html及graspnet.png均导出，已实看failed PNG有目标红色可见表面、相机轴、实际Piper URDF模型；无有效候选则不画假选中位姿/路径。只表示保存场景及当时规划参数，不表示当前硬件状态或全路径碰撞通过。外参文件SHA710f7781bcbab5f86844b1701adb912bb47a8cb81d16ba1ebf94d3aa0d4f379a保持，物理标定/TCP精度和相机支架避让仍未验证。

本轮模型服务 model-service-01 和 model-service-final 已结束，未停止既有 SAM37115；最终共存显存约SAM3218MiB、GraspNet1148MiB，证据gpu-coexistence.txt。曾因沙箱PID命名空间未找到host PID，转为host上下文严格校验run-id后结束本轮服务；final服务遇端口占用可等待并在旧服务退出后恢复MODEL_READY，未影响机器人。没有SSH、真机采集或CAN操作，机器人仍以上一现场记录为准。运行/检查日志全部保留，不清理build。
完成提示音使用既有15秒三音高资产 `paplay assets/audio/work_complete.wav` 播放；未修改音频。

### 2026-11-17 用户要求回零后抓取：回零完成，新现场 GraspNet 仍无可执行计划

用户新指令“给机械臂进行归零，之后再进行抓取任务”授权本轮现场操作及必要运行部署，覆盖前轮仅离线范围；没有授权改外参/TCP或切回旧抓取算法。读取新鲜反馈确认 READY/CAN1/teach0/全使能/故障0，初始关节[-4.867,140.694,-102.057,47.447,9.949,-53.218]°，15个只读快照位置跨度0，夹爪50.6mm。当前外参文件SHA710f7781bcbab5f86844b1701adb912bb47a8cb81d16ba1ebf94d3aa0d4f379a与本地一致。

复用既有return-zero规划/执行代码，仅更换本次run-id；按<=10°关节段生成15段，5%速度。现有完整STL/相机假设包围区域审查300姿态、重叠0、采样顶点最近82.349mm；不代表未知支架及连续运动完全无碰撞。graspnet-return-zero-20261117-01先open70mm/1Nm，再15段归零，共16receipt，ZERO_REACHED/退出0。独立15快照及本轮收尾12快照均六关节[0,0,0,0,0,0]°/跨度0、READY/err_code0/全使能/teach0，SDK[56.127,0,213.266]mm，夹爪68.4mm。没有重设关节零点、清错、复位或去使能。

将前轮成功构建的40个运行/配置/helper文件单独部署至机载新目录 /home/unitree/piper-vision-graspnet-20261117，旧 /home/unitree/piper-vision-semantic-20261114 保留。包SHA=e6b03a5897b11cae370ab89416fa4ddf108791104b7f1d9f81185c0637e40c9d；SOURCE_MANIFEST记录本地成功安装来源，SHA256SUMS机载40项通过。机载解释器仍/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python，graspnet-live-check-20261117-01=CHECK_OK，未装Torch到Go2。后续应显式从新目录加载semantic-graspnet.json，旧目录默认仍旧算法，不能混用。

开发机 model-graspnet-live-01=MODEL_READY，临时18766反向隧道建立；既有SAM/18765原样复用，两端HTTP可达。graspnet-red-observe-20261117-01重新采三帧，红色积木DETECTED、静止、约51.1×46.9mm可见面；相机点[-4.7087,-45.1406,211.0000]mm。图像已实看，红块在纸箱边沿前方，识别框与目标一致。observe退出0/OBSERVED，无抓取控制命令。

随后仅对本次刚保存数据经机载新入口做GraspNet preview，graspnet-red-plan-20261117-01=WAIT_PLAN/退出1/CAN发送0。真实模型输出136个候选，全136个不属于目标掩码，目标候选0；没有运行抓取控制流程或闭爪。为区分场景采样问题，用同一已保存点云做一次开发机离线局部workspace探针，完整场景碰撞点云仍保留：目标周围30mm采样得到5个候选，均被原完整规划的absolute grasp IK拒绝；60mm采样无可用候选。这只说明当前IK求解未通过，不能声称数学上绝对不可达。探针没有改生产代码/配置、外参、TCP、开口、关节限制，也未将探针结果发给机械臂。

本次所有证据位于 build/piper-graspnet-live-20261117/evidence：before-zero-passive.json、zero-plan.json、zero-mesh-audit.json、zero-verified.json、final-passive.json、task-result.json，以及对应zero/check/observe/plan运行目录；target-workspace-probe.json/log保存局部采样诊断。观察原图路径 graspnet-red-observe-20261117-01/observe-769a9d34/frame02-color.png，标注detection02.png。无抓取成功或抬升。

本轮GraspNet服务PID47876和18766隧道PID47970均按精确参数校验后SIGTERM结束，services-stopped.json保存；既有SAM/18765未动，无持续相机采集或自动抓取进程。机器人保持零位与68.4mm开爪，用户的抓取目标尚未完成，后续须解决目标候选采样与IK适配并重采新场景，不重放这轮文件下发旧目标。没有产品代码修改、没有重新构建或提交；部署来自前轮已验证构建产物。

### 2026-11-18 六轴抓取速度比例提高到30%，本地构建与离线检查完成

用户要求“将所有关节允许转动的速度调节到比较大的值，加快夹取速度”。本轮按夹取流程J1–J6统一速度比例处理：config/piper_vision_grasp/{config,semantic,semantic-camera-avoid,semantic-graspnet}.json的motion.speed_percent由5提高到30；不是修改固件逐轴最大转速。geometry.load_config在语义分支提前返回前统一检查0–100整数，PiperSession._attempt同步取消固定5的限制并保留发送边界校验，main.send_once仍直接传递配置到MotionCtrl_2(1,1,speed,0)。SDK范围依据[官方接口](https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/interface/piper_interface.py)。30%为此次实现选择，指令比例提高6倍不等于整次抓取耗时缩短6倍。夹爪力矩/开口、加速度、关节限位、标定、规划和等待恢复流程均未改，历史provenance中的5%保持原记录。

原180项基线与新增26项测试全部通过（206 pytest）；三包构建成功，5项ctest包装通过，colcon合计211包含包装，0失败。新增覆盖四配置默认值、普通/语义配置非法值拒绝、SDK速度/六轴目标及receipt一致性、非法速度零发送。RED阶段13失败/13通过；第一次send_once遇旧5%限制进入等待，已结束该测试进程，并给测试加等待即失败的断言后重新取得有界RED，未改产品等待逻辑。Python3.8语法、Bash语法及diff检查通过。构建仅有未使用COOPERATIVE_RUNTIME_PYTHON及PCL_ROOT CMake警告，不影响构建成功。

本次输出build/piper-speed-20261118/overlay；成功安装share/vlm_nav内40个运行/配置/helper文件与源码逐字节一致，四个配置均30%，四项安装入口check均CHECK_OK。入口scripts/piper_vision_grasp.sh按相对路径加载本次share内tools/config，无需改导航环境/ROS launch。完整构建环境/命令及入口位置见[runbook](go2_runbook.md#2026-11-18-piper-六轴抓取提速构建)。证据保留于build/piper-speed-20261118/evidence，包括pytest.log、pytest-red-bounded.log、build-and-test.log、installed-files.json及install-check/*。

本轮未SSH、未部署到Go2、未发送CAN或执行真机抓取；机载/home/unitree/piper-vision-graspnet-20261117仍是此前5%包，不把本地配置修改当作机载已生效。30%实际运动稳定性和提速效果尚未验证；上一轮目标候选/IK未通过的问题仍未解决，提速不改变WAIT_PLAN结论。未提交，保留原未提交修改和历史证据。

### 2026-11-18 已部署30%速度并恢复启发式语义默认入口

用户明确要求部署新的速度设置、取消使用GraspNet并恢复“大模型选物块标签，取物块上方为抓取点”的原启发式流程。本轮SSH及文件部署在此授权内，不执行抓取运动。核对机载原semantic/graspnet包均5%，没有活动piper_vision_grasp/main.py、GraspNet服务或相关控制进程；当前记录仅说明当次检查。

复用原semantic-camera-avoid.json：Qwen标签/框选物→SAM分割→RGB-D可见区域中心/夹取轴→absolute_target/plan_absolute，六轴统一30%，基座Z上方100mm预抓、入口额外抬高100mm、60°接近。已向用户说明该方法抓取位置是可见区域中心，不是新的顶面拟合。保留当前外参、135mm TCP、开闭爪力矩和等待恢复；没有恢复已废弃的相机干涉路径或-13mm偏移。唯一产品代码改动为main默认配置从旧HSV config.json切到semantic-camera-avoid.json，默认无GraspNet调用；历史GraspNet代码/配置及包保留，当前入口不选择它。旧HSV测试需显式选择config.json。

新增1项回归先RED后GREEN，核对实际CLI默认配置、30%、上方预抓参数、启发式make_plan且禁止GraspNet调用。原206项基线+新增1项=207 pytest通过；三包构建成功，5个ctest包装通过，colcon212含包装、0失败。Bash/Python3.8语法和diff检查通过。构建输出build/piper-heuristic-20261118/overlay；40个运行/helper/配置来源文件与安装share逐字节一致，开发机默认入口CHECK_OK。完整构建环境与入口见[runbook](go2_runbook.md#2026-11-18-恢复启发式默认入口并部署30速度)。

新机载包/home/unitree/piper-vision-heuristic-20261118，稳定链接/home/unitree/piper-vision-current指向它；脚本scripts/piper_vision_grasp.sh，解释器通过PIPER_VISION_GRASP_PYTHON指定/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python。部署包SHA256=c1944593224c0a889c23943966c76fa60a2a6af88d287b2c2a5cd08f1586853a；包哈希和40项SHA256SUMS机载通过。SOURCE_MANIFEST.json记录本次成功安装来源、当前默认配置/后端/速度；DEPLOYMENT_VERIFIED.json记录实际加载路径和检查结果。

机载heuristic30-deploy-check-20261118-01=CHECK_OK，实际模块及semantic-camera-avoid.json均在新包，Python3.8.10，command_attempted=false。先完成包核验/check，随后才建立current链接；原/home/unitree/piper-vision-graspnet-20261117及/home/unitree/piper-vision-semantic-20261114不覆盖，历史5%不改写。继续使用旧绝对路径或显式semantic-graspnet.json仍是旧/可选流程，后续使用当前入口及默认配置。

全部证据在build/piper-heuristic-20261118/evidence：pytest-red.log、pytest.log、build-and-test.log、package.json、install-check/default、deployment.log、remote-deployment.json。未发送CAN、未采图、未调用云端或启动模型/抓取进程；没有真机30%速度、抓取可达性和稳定性验收。此前GraspNet候选问题不作为当前启发式方法失败结论；新场景仍需重新取图规划，不能据部署成功声称夹取成功。未提交，保留用户未提交修改与全部历史证据。

### 2026-11-18 抓取过程可视化实现及一次红块尝试；用户手动整机重启，等待恢复取证

用户要求将中间结果集中保存，展示RGB/3D中物块与候选抓取位置、相机/机械臂坐标变换、IK与实际关节角，然后执行一次红色方块抓取。本轮授权含必要SSH、模型服务/云端识别、部署、有限采图及该单周期运动；后续用户先两次明确移近方块并继续同一次尝试，最后明确“现在重启机器狗”，澄清为用户手动整机断电重启。最后指令优先；重连后先只读核对旧进程和新状态，不自动恢复抓取。不能把这次未完成尝试或重启视为额外新实验授权。

新增tools/piper_vision_grasp/report.py和test/test_piper_report.py；main保存config_snapshot.json及每次before捕获内planning.json（含失败计划、初始反馈与冻结配置），成功计划绑定capture_dir；visualize默认走启发式报告，显式GraspNet保留旧报告。报告产生同图RGB/3D物块与抓取/预抓点、交互3D、相机↔基座矩阵/轴图、每个候选IK角与采图实际角柱图、已执行目标/反馈离散曲线及CSV/JSON。被拒绝的IK标明NOT SENT，未发送/未到达的实际角留空；相机后方/图外点标明不可投影；root配置和每次捕获配置冻结；搬回本地后不依赖旧机载绝对路径。独立审阅的预抓回退、缺少结果误标RUNNING、断流连线、状态掩盖、重合标记及坐标图比例问题已修复，真实PNG已实看并调整图层/布局。HTML图片/交互图延迟加载以容纳等待期间多个候选。

现场还确认standby下开爪分支没有选择CAN模式：本地piper_io._attempt现只对ctrl_mode=0的开爪/夹爪命令，在持久receipt后选择配置速度的CAN模式，再发夹爪目标；部分写入仍COMMAND_UNCERTAIN且不重试。新增2项RED→GREEN验证顺序、receipt和失败不重发。原207基线+7报告+2standby=216项，系统环境215通过/1绘图依赖跳过，既有model-venv中7项报告全部通过。最终三包构建及6个ctest包装通过，colcon222含包装/1跳过已另验证；日志pytest-complete.log、report-complete.log、build-and-test-complete.log。完整环境/构建见[runbook](go2_runbook.md#2026-11-18-抓取过程证据与可视化)，本地输出build/piper-evidence-20261118/overlay。

实际现场部署的是首次成功构建的41文件包，SHA83214bd89fdfce9d103f4a64f8307d61d058d5901a207c52a68778e61f03521c，机载/home/unitree/piper-vision-evidence-20261118，piper-vision-current从heuristic包切换到此目录；机载check=evidence-deploy-check-20261118-01、41哈希通过。其捕获与记录功能完整；后续离线报告改进及standby修复仅在本地最新安装，未热更新运行中的机载包，重启后需按新清单部署再运行。解释器保持/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python。

本次模型service-red-evidence-20261118-01复用原SAM环境/已授权DashScope凭据，只在内存中传递密钥，实际device=cpu，MODEL_READY。专用18765反向SSH隧道恢复。red-evidence-observe-20261118-01三帧OBSERVED/CAN发送0，实看红块在纸箱上，可见宽约50.24mm、相机深度301mm。唯一抓取run_id=red-evidence-grasp-20261118-01，原脚本无自动重启；GNOME诊断终端已打开（dconf只读告警，不影响tail）。30%、启发式60°/垂直预抓100mm/额外入口100mm、原外参/TCP及力矩保持。

最初TCP约[739.55,19.12,-6.18]mm，端点IK残差约75mm/13.6°，WAIT_PLAN且零命令。用户移近后相机深度196mm、TCP约[634.73,18.17,1.52]mm，端点可解但local entry branch unavailable，仍零命令。离线位置探针只模拟基座X位移（没有下发）：再-100mm时现有路径可解，用于向用户说明摆放参考，不改标定/路径门槛。用户再次明确移近继续；新相机点[0.01675177,0.02110731,0.15500001]m的完整规划通过，累计20次OBSERVED、19次WAIT_PLAN后发出唯一open receipt（70mm/1Nm）。六轴目标、闭合、抬升均尚未发送。

开爪最初读0mm且ctrl_mode0/全使能/无故障，停WAIT_OPEN。现场补发一次MotionCtrl_2(1,1,30,0)，不重复GripperCtrl、不发JointCtrl；原开爪目标随后响应。首次辅助脚本用系统Python在import piper_sdk失败，未连接SDK/CAN，can-mode-recovery.json仅准备记录；改用既有venv的can-mode-recovery02.json才实际发送模式命令。不要把两个准备记录算两次模式发送。后续CAN1/err_code0、开度6.7mm、反馈扭矩约1.04Nm、仍WAIT_OPEN。用户目视确认“实际已经大幅张开，反馈可能不对”，后补“没有置零，目前夹爪打开到最大”。未获得实测毫米开度，不改66mm判据、不乘10、未加力/校零/清错。

三秒原始CAN查询诊断确认602帧0x2A8，最新00001a2c04134000，位置原始6700、SDK6700，扭矩1043；固件S-V1.8-2，0x47E=6464030000000000，行程100mm、示教比例100、摩擦3。原始反馈与SDK一致，支持不是显示缓存；不能据“最大张开”直接认定比例或零位根因。只查询固件/参数，无新运动命令。gripper-raw-diagnostic.json和模式恢复日志均保存。

用户随后手动整机重启。已尝试对精确run-id发送SIGINT，但SSH在banner交换时已超时，此停止请求未送达；抓取SSH会话退出143，不当作程序优雅退出或物理停止证据。最后已观测状态是WAIT_OPEN/仅一个open receipt/无六轴命令；当前重启后硬件状态未知。最终取回scp同样SSH断开失败，最新accepted plan、完整开爪事件与result可能仍仅在机载持久目录。已有本地捕获不得清理，重连后优先取回/home/unitree/.local/state/piper-vision-grasp/red-evidence-grasp-20261118-01并检查旧进程，不能自动重跑。

本地统一结果目录build/piper-evidence-20261118/results/red-evidence-grasp-20261118-01，阶段性入口interim-visualization-02/index.html；该版对应断线前已取回的候选，不含尚未取回的全部末段记录。原始RGB/深度/mask、planning、配置快照与事件均在其下；另已保存operator-reboot.json标记用户重启、最后已知状态及取回缺口，并复制原始CAN/模式恢复诊断。辅助日志在同base/evidence。模型服务exec会话24417已Ctrl+C退出130，18765隧道会话35376已超时退出255；无本轮开发机模型服务或专用隧道继续运行。不能声称抓取成功、实际关节到位验证或开度问题已解决。未提交，保留历史与用户改动。

### 2026-11-18 重启后用户重新授权抓取：r2部署完成，新尝试等待CAN恢复

用户明确“重新抓取和收取中间数据，并可视化”，授权重启后新一次红块尝试。已核对boot_id=74ef7fa9-2a99-44b1-aefd-4d874f8977c6，无旧抓取进程。将上一条最终成功构建的41文件打包为piper-vision-evidence-20261118-r2.tar.gz，SHA256=0b52b061090a582d21f62f10c8a6a64094fb658a9965f8cc5d4a2e2ec8bca09c；机载41哈希及evidence-r2-check-20261118-01=CHECK_OK后，current切换至/home/unitree/piper-vision-evidence-20261118-r2。包含standby开爪修复和最终报告版本；默认仍启发式、30%。未修改产品代码或重建。部署证据build/piper-evidence-20261118/evidence/deployment-r2.log。

本轮run_id=red-evidence-grasp-20261118-02，机载PID3239；服务service-red-evidence-20261118-02（CPU/SAM/Qwen）与18765专用隧道已启动。程序仅WAIT_FEEDBACK、receipt为空，没有开爪或六轴指令。用户确认机械臂已上电且连接正常。can0按既有序列号0046002F5246570520323934匹配、1Mbps/ERROR-ACTIVE/UP，但多次3–4秒无过滤被动监听均0帧，内核RX计数固定74742。boot服务曾READY/全使能，不能据此当作当前反馈正常。用户执行can0 down/up后仍无帧；单次SearchPiperFirmwareVersion（0x4AF查询，无运动控制）无应答，GetPiperFirmwareVersion=-1199，TX计数也未增长。尚不能认定机械臂或适配器根因。远端sudo需密码，已请求用户重新绑定核对过的gs_usb设备1-2.1:1.0，再恢复1Mbps接口；等待操作结果。程序按原逻辑等待，反馈恢复后可能继续同一次抓取，已提醒用户保持活动范围外。

旧run01完整目录已取回，共20个planning捕获、37个计划关节阶段均NOT_EXECUTED。原events.jsonl尾部第2417行为2383字节全零，原文件SHA256=79c8671d5a094cd3e3ca3f4bf02b7b21905c5ffb6d736a95522859697808d256；另有0字节stop-for-reboot.json，均原样保留。仅派生recovered-input/events.jsonl采用前2416条可解析记录，recovery.json记录缺口/哈希，其余输入链接原始证据。没有result.json，报告准确显示NO_FINAL_RESULT、最后WAIT_OPEN。完整报告visualization-complete/index.html已生成并实看RGB/3D/IK图，页面明确日志缺口；没有补造实际到达角。最初visualization-recovered因损坏尾记录失败，其失败目录保留，不作为交付入口。

新run02诊断及等待态可视化已回传至同一results下的新目录；尚无新RGB/IK或运动反馈，不能把run01报告称为重启后新抓取。RealSense裸context查询被旧viewer全局配置拒绝，按现有vision.py的inherit=false方式可枚举D435/151222077264，未修改相机配置。最新CAN诊断见新run02的recovery-diagnostic.json、can-after-reenable.json、can-firmware-query.json。现场仍在继续，后续核对反馈并追加最终状态。

后续用户提供管理员凭据并明确要求直接执行。已核对USB序列号后重新绑定gs_usb的1-2.1:1.0，配置can0为1Mbps/UP，退出0；凭据仅输入sudo标准输入，未写入文件/日志。接口ifindex由7变9，3秒被动监听恢复全部关节/位姿/夹爪反馈（0x2A1等约600帧、每路低速120帧），RX开始增长。证据can-usb-rebind.json及run02/after-usb-rebind.json；支持重绑恢复通信，尚未证明底层卡住原因。现有程序自动重连、重新三帧采图；首捕获before-4e547de9红块相机深度190mm、基座TCP[623.046,26.264,-62.247]mm，上方入口IK失败（位置残差2.655mm、姿态0.786°），仍无运动receipt。纯离线探针基座X减30或50mm均PLANNED，未下发这些假设目标，已请求用户移近约4cm继续同一次尝试；等待现场回复。

用户随后确认移近继续；新RGB中红块底部被画面截断，before-1a5d60a6的mask内非零深度仅6916/71546=9.67%，测量中心落到约0.81m背景，连续WAIT_STABILITY，偶有稳定背景点则端点IK失败，仍无命令。原始RGB/深度/mask已取回，离线depth-diagnostic.png展示缺口，不把背景点当真物块坐标。用户确认退回并完整入镜后，before-3a46621d有效深度98.37%、实测259mm、宽49.26mm，端点IK仍失败。已明确请以相机镜头至红块正面距离约15–16cm摆放，避免把机械臂底座或夹爪作起点；等待同一次尝试继续。报告总入口build/piper-evidence-20261118/results/index.html，阶段性报告与原始数据分目录保存；未更改稳定性阈值、外参或路径参数。

用户再摆放后实测167mm，before-3b3bdbe6完整规划通过，单次open70mm后反馈68.4mm/OPEN_READY/CAN1/全使能/无故障；本轮standby修复正常生效，未再出现原6.7mm问题。开爪后重新采图的上方入口残差约1.0165mm，J5达到模型上限69.900851°，仍WAIT_PLAN。用户提出“之前的IK可用范围没有这么小，重新检查”；已SIGINT精确run02 PID3239，result=EXITED/previous_state WAIT_VISION（中断当时正在采图），唯一receipt=open，无六轴/闭爪/抬升。没有发送释放、去使能或物理急停。run02完整证据已回传，visualization-final/index.html包含14个已规划候选；全部实际到达角为空。

复查证据在build/piper-evidence-20261118/ik-review。AST逐函数比对2026-11-17已部署geometry：solve_ik/fk/plan_motion/plan_absolute/absolute_target均未改变，关节限位及1mm/1°判据未改，30%不参与几何求解。历史translation-red-preview-20261117-01真实CLI为40°，而本轮默认60°、基座Z预抓100mm加入口100mm。不能把固定姿态的全路径失败等同于机械臂整体范围缩小。旧当前样本TCP[600.835,53.905,-61.897]mm：60°上方分支卡J5上限，40°卡J4约99.98°上限，5组不同初值未得到另一可行完整分支，50°完整可行且J5最大43.2°；不同角度/入口高度/基座X采样均离线、未下发，见pitch-comparison.json、geometry-comparison.json、endpoint-seed-comparison.json。

复查期间新现场图像目标已为[648.160,25.442,3.010]mm（相机深度209mm），与旧样本Y/Z不同，不能沿用50°结果。该新目标30/45/50/60°当前求解均未找到完整路径，40°完整PLANNED/J5最大约48.3°；50°只读preview01已结束、CAN发送0。随后40°只读preview02=PREVIEW_READY/37阶段/CAN发送0，完整捕获及报告位于results/red-evidence-ik-review-preview-20261118-02。对其真实新计划做419个0.5°关节采样，当前及人工平移前相机假设下STL包围盒均0重叠，顶点最近约86.1/158.1mm；不证明连续运动、未知支架或实物标定精度。复查结论和图位于ik-review/index.html，已说明先前只归因为摆放过远不完整。

原循环已结束，按项目一次性实验授权边界确认具体新方案后，用户明确“按40°继续抓取并保存可视化”。新run03=red-evidence-grasp-20261118-03已启动，明确CLI --approach-pitch-deg 40，仍30%、预抓100mm/入口100mm、相同外参/TCP/力矩；每轮重新采图，不发送离线存档目标。源码和部署默认60°未擅自改写，此次仅使用既有CLI选项；没有产品代码修改或新构建。等待记录实际执行结果。

### 2026-11-18 40°红块抓取完成：现场确认悬空稳定，完整关节反馈与可视化已交付

上述run03已完成open、37段joint（包括grasp和2段lift）、close，共39份持久receipt；所有joint receipt均speed_percent=30，冻结配置approach_pitch_deg=40。37段均AT_TARGET，最大到位误差0.287981°（lift-1）；从第一joint指令到最终lift到位23.1946秒，不含图像识别、开爪及恢复等待。grasp实际[2.869,143.532,-107.322,-11.560,8.946,12.873]°；lift到位实际[2.869,141.470,-107.836,-8.991,11.438,10.474]°。开爪68.5mm，闭合后57.2mm、约-0.53Nm，CONTACT_CANDIDATE后完成20mm计划抬升。

after-lift-10cca56d/frame02-color.png实看红块在夹爪之间、高于箱面，顶部部分出画、夹爪遮挡。自动verification.json仍WAIT_VERIFY/face shape-orientation changed，未强改为通过。用户明确回复“已夹住，方块悬空且稳定”，独立operator-verification.json记录OPERATOR_CONFIRMED_GRASP，与程序自动结论区分。现场确认后SIGINT精确run03 PID7633，result=EXITED、previous_state WAIT_VISION（当时服务连接失败），自动grasp_confirmed仍false。未发送松爪、回零、去使能或急停，机器人保持夹持。

12次结束后只读快照全部READY、全使能、ctrl_mode1、teach0、arm_status/err_code0，六轴跨度逐轴0°；末次关节[2.869,141.410,-107.965,-8.895,11.601,10.305]°，SDK法兰[544.977,24.479,110.523]mm，夹爪57.2mm/-0.522Nm。此记录是关节反馈与现场确认，不等于外参/TCP实物精度已标定。证据after-grasp-passive.json、delivery-summary.json。

本轮模型服务曾中断：旧service-red-evidence-20261118-02 exec会话14503退出143，旧隧道12725退出0，具体外部终止原因未知；run03在WAIT_VISION原地等待，没有新建或重复运动周期。恢复service-red-evidence-20261118-03（CPU）及专用18765隧道后继续新三帧采集，完成全部动作。抬升后新模型再次退出143，后续WAIT_VISION连接失败；没有把它伪装成几何失败或自动验证通过。收尾确认模型会话8824已退出143，专用隧道10990已结束255，无需再重启模型，本次任务已获得所需证据。若后续排查模型生命周期，须另查退出来源，不猜测为云端或OOM。

完整原始运行目录已回传到build/piper-evidence-20261118/results/red-evidence-grasp-20261118-03。最终报告visualization-final/index.html含2组规划候选的同图RGB/3D、交互3D、冻结相机↔基座变换、37段IK/实测曲线及CSV、现场确认和抬升原图；执行反馈只绑定实际采用的开爪后捕获，未执行的候选不伪造到达角。根results/index.html为统一入口，也链接run01恢复报告、run02失败候选、40°预览与ik-review。全部来源JSON可解析，最终报告37条阶段均有实测值、仅1组捕获标记实际到达；链接全部存在，已实看RGB、3D、坐标和关节图。文档diff检查通过，未修改产品代码/重建/提交，保留历史证据。本次授权单周期已结束，后续新运动须有新授权；持久默认仍60°，此次成功运行明确使用40°CLI，不能省略参数声称复用同方案。

### 2026-11-18 用户授权松爪后再次抓取（进行中）

用户明确“松开夹爪，重新尝试”，授权一次释放和新一轮抓取，沿用前次40°/30%及中间数据记录。核对无旧run进程，机载current仍r2；新鲜反馈READY/无故障/全使能，关节及57.2mm夹持状态与前次结束一致。red-evidence-release-20261118-01复用现有PiperSession/send_once/wait_gripper，仅发送一次open70mm/1Nm，反馈68.4mm稳定，结果RELEASE_OPEN_READY；六轴未发送指令、前后反馈逐轴一致。完整释放记录已取回同一results目录。

恢复service-red-evidence-20261118-04（CPU）及18765专用隧道；凭据仍只在进程内存使用。red-evidence-retry-preview-20261118-01以40°只读重新采图，检查释放后物块和当前姿态的完整路径；不使用旧物块坐标下发。新运动周期预定red-evidence-grasp-20261118-04，尚待新预览结果。未修改产品代码、未重新构建或部署。

上述只读预览PREVIEW_READY，物块在箱面、夹爪已张开，目标基座约[658.754,12.197,2.535]mm，当前姿态出发30阶段。对新计划289姿态的相机区域采样，当前及人工平移前外参假设均0重叠，最近采样顶点约96.0/160.8mm；不作连续碰撞保证。随后授权范围内启动run04，实际新采图时进入WAIT_PLAN/local entry branch unavailable，未发送任何运动receipt。

### 2026-11-18 用户中断重试要求先归零，已回零并按最新指令保持

用户中途要求“先让机械臂归零”，已先SIGINT精确run04 PID10128；result=EXITED/previous_state WAIT_PLAN、command_attempted=false、attempts为空，没有执行新的抓取。10次只读反馈确认关节稳定在[2.869,141.410,-107.965,-8.895,11.601,10.305]°、夹爪68.5mm打开、READY/无故障/全使能。

复用既有分段回零helper，按当前真实关节生成15段<=10°目标，速度30%，夹爪不发新命令，不执行重新置零标定。300姿态相机区域采样在当前/人工平移前外参下均0重叠，最近采样顶点101.5/176.7mm。操作脚本build/piper-evidence-20261118/evidence/return-zero-20261118.py，机载/tmp/piper-return-zero-20261118.py；没有修改产品代码。run_id=red-evidence-return-zero-20261118-01，15份joint receipt均30%，结果ZERO_REACHED；到位初始J2=0.092°，其余0°，均满足原0.3°容差。

结束后独立12次只读反馈全部READY/全使能/ctrl_mode1/teach0/无故障；J2跨度0.023°、其余0，最终六轴[0,0,0,0,0,0]°，SDK法兰[56.127,0,213.266]mm，夹爪68.5mm、约0.394Nm。用户进一步明确“先保持零位”，不继续之前的抓取重试。模型服务会话29595已Ctrl+C退出130，专用隧道12769已结束255；未留下本轮自动抓取程序。

释放、预览、被中止run04和回零原始目录均回传build/piper-evidence-20261118/results；被中止run04报告visualization/index.html保留候选、不伪造执行角。统一results/index.html已更新最新状态为零位保持，并链接回零result.json、verified.json及历史报告。运行配置/15份receipt、独立反馈、JSON及文档diff检查通过；无产品源码改动、不重建、不部署、不提交。最新授权是保持零位，后续抓取须等待用户继续指令。

### 2026-11-18 目标移动后用户重新授权抓取及可视化（进行中）

用户明确“现在目标位置有移动，尝试夹取并提取中间数据可视化”，替代前条保持零位要求，授权新一次红块尝试。已核对current仍r2、无活动控制进程，新鲜反馈六轴全0°、夹爪68.5mm、READY/全使能/无故障。恢复service-red-evidence-20261118-05（CPU）与18765专用隧道，先用red-evidence-moved-preview-20261118-01重新三帧识别/40°规划；不复用旧目标位置。预定运动run05=red-evidence-grasp-20261118-05，仍30%速度及原外参/TCP/力矩。待记录新场景及实际执行结果。未修改产品代码或重建部署。

新预览PREVIEW_READY，before-5678c75e相机点[-57.803,-37.018,194.000]mm，基座TCP[634.741,94.133,-2.378]mm，可见夹取宽50.809mm；RGB实看红块位于箱面左侧，完整37阶段规划通过。451姿态、0.5°关节步长全STL相机区域采样在当前/人工平移前外参下均0重叠，最近采样顶点69.4/140.1mm；仍不证明连续运动或未知支架。预览数据和报告已保存。run05随后启动，重新取图后单次开爪68.5mm/OPEN_READY，正在开爪后重新采图；尚待最终实测结果。

### 2026-11-18 run05现场确认抓空，已结束该周期；追加离线点云展示

run05在40°/30%下完成open、35段joint至grasp及close，共37份receipt；35段均AT_TARGET，最大到位误差0.296997°。计划抓取宽50.792297mm、闭合目标46.792297mm，反馈47.5–47.6mm、约-0.29Nm，持续WAIT_CONTACT，2段lift从未发送。用户明确“抓空或位置偏了”；独立operator-verification.json记录失败，精确SIGINT该进程后result=EXITED/previous_state WAIT_CONTACT，未发送释放、回零、去使能或急停。关节跟踪准确不证明工具实际位置准确，偏差方向/幅度及根因尚未实测。

只读red-evidence-contact-observe-20261118-01起初因主程序持锁WAIT_IO（Errno11），主周期结束后完成采集并退出OBSERVED/CAN发送0；不是新CAN失联证据。闭爪后RGB可见红块在夹爪左侧、部分出画。目录全部回传results。先前after-exit-passive.json的锁冲突记录保留；观察器结束后的after-capture-passive.json中12次均READY/全使能/无故障/teach0、command_attempted=false，末次关节[11.913,143.336,-105.329,-52.211,12.617,60.007]°，法兰[531.279,96.038,83.931]mm，夹爪47.5mm/-0.283Nm。机载无piper_vision_grasp活动进程，模型05与专用隧道会话已终止。本周期已结束，当前保持抓取处姿态，没有抬升。

用户随后要求“添加点云的可视化，即相机深度所形成的空间点，将提供给产生候选夹取位置的那种点云也可视化”。正在本地离线实现：全场景彩色深度点云与SAM/IQR筛选后的真实启发式目标输入逐帧并列，保留完整PLY及抽样计数。没有新运动或部署授权需求；remote current仍r2。本次仅回放保存证据，外参/TCP物理精度继续标为未验证。新构建拟位于build/piper-pointcloud-20261118/overlay，完成情况后附。

追加点云功能已完成。tools/piper_vision_grasp/semantic.py把原mask+IQR+反投影块提为target_points，measure与离线报告共享；新cloud_report.py输出全场景/实际目标输入逐帧双面板、完整彩色二进制PLY和来源JSON。report.py接入已有visualize入口、展示独立operator-verification，未改检测/抓取参数。六帧真实数据旧安装measure与新measure逐字段完全一致，与机载保存量的差异仅浮点舍入（1e-12内），证据build/piper-pointcloud-20261118/evidence/geometry-regression.json。

完整三包构建成功，新安装build/piper-pointcloud-20261118/overlay/install/vlm_nav/share/vlm_nav；脚本及相关源码/配置6文件SHA一致。原216用例基础新增2用例（报告1/3帧覆盖及点云/PLY验证），系统216通过/2因缺Plotly跳过，colcon224条目含6包装、0错误/0失败；已有绘图环境9项报告测试全部通过（包括两项跳过），之前report+semantic合计35通过。只读审阅未发现阻断/重要问题，独立核对真实3帧6份PLY颜色完全一致、XYZ最大舍入3.55e-15m，24条HTML轨迹及三组切换正确；未做浏览器实际鼠标交互测试。

最终报告results/red-evidence-grasp-20261118-05/visualization-pointcloud-final/index.html，统一入口results/index.html已更新最新失败状态并保留历史。包含2组候选各3帧共12份完整PLY，candidate-01为实际采用的before-292dc93c；其frame02场景121320点（显示24264点）、启发式输入22632点（全部显示）。静态PNG实看已修正薄片坐标轴拥挤、中心重合标记，保留远距离深度点导致全场景目标较小的真实现象；可交互缩放，目标单独近景。源RGB、深度、mask、坐标变换、IK/实际角、失败反馈及闭爪后原图均保留；35到位/2抬升未执行与全部链接、PLY长度/有限性已核对。result为VISUALIZED/offline/CAN发送0。初版visualization-pointcloud保留，不覆盖历史。交付汇总run05/delivery-summary.json；构建、回归、导出核对脚本与日志在build/piper-pointcloud-20261118/evidence。远端仍r2，未部署本次离线报告模块，未启动新抓取、释放或回零。外参/TCP实物精度及抓空根因仍待验证。

### 2026-11-18 按用户最新指令再次关节回零，现保持零位

用户明确“将关节回零”。确认remote current仍r2、无其他控制进程，12次只读反馈稳定READY，起点[11.913,143.336,-105.329,-52.211,12.617,60.007]°、夹爪47.5mm。复用原分段回零helper，仅换独立run_id为red-evidence-return-zero-20261118-02；15段、30%速度、只发joint。315姿态离线相机区域采样在当前/人工平移前外参两假设下均0重叠，最近采样顶点约68.99/139.55mm，不代表连续碰撞保证。

执行结果ZERO_REACHED，15份joint receipt均30%。程序退出后独立12次只读反馈均READY/关节在0.3°内，最终六轴[0,0,0,0,0,0]°、夹爪47.5mm保持；未发送夹爪命令，不继续抓取。完整运行及verified.json已回传build/piper-evidence-20261118/results/red-evidence-return-zero-20261118-02，统一results/index.html更新当前为零位保持并保留抓空/点云报告。操作脚本、前后反馈和采样证据在build/piper-evidence-20261118/evidence/return-zero02*及before/after-return-zero02-passive.json。无产品代码改动、不重建、不部署、不提交。


### 2026-11-18 六关节回零正式脚本（仅本地实现，未部署/未运动）

用户要求“制作一个机械臂六关节回零脚本”。新增`scripts/piper_return_zero.sh`，调用既有`piper_vision_grasp/main.py`的`zero`动作；由当前新鲜关节反馈生成最大名义单轴角差10°的线性分段，默认30%速度、0.3°到位容差，逐段只发送joint，不发送夹爪、不修改零点标定、不自动使能/失能。复用现有控制锁、设备限位、持久receipt、诊断终端和异常等待；不启动相机、模型或ROS。诊断存储暂时故障时等待恢复，不重发或自动续发；持续存储故障下退出无法保证result落盘。分段插值不包含碰撞规划，不把旧两次现场回零作为新脚本验收。

只修改上述入口/main、现有test/test_piper_vision_grasp.py与相关runbook/本交接，保留原大量未提交工作。新增9项离线用例：分段/已零位、异常等待/不重发、存储恢复、未到位不继续、CLI无相机与重复run拒绝；基线161通过，新增后170通过。独立审阅发现诊断日志故障会退出的问题已修复，新增用例先失败后通过，最终审阅无阻断问题。

三包构建成功，输出`build/piper-zero-20261118/overlay`；本次6个Piper ctest包装内实际225通过、2项报告测试因系统环境缺Plotly跳过，colcon233条目含包装、0错误/0失败，不作233个独立用例宣称。完整命令/环境只在[runbook](go2_runbook.md#2026-11-18-六关节回零脚本构建)维护。安装路径`build/piper-zero-20261118/overlay/install/vlm_nav/share/vlm_nav`；脚本按自身目录加载tools/config，7个关键文件SHA与源码一致，实际安装入口--help通过，从/tmp导入安装模块fake跑15段至ZERO_REACHED，夹爪保持47.5mm、无相机调用。未改导航env/launch，因为不存在Piper旧安装硬编码；新安装自动包含独立入口。

证据`build/piper-zero-20261118/evidence/`包括baseline、red/green、storage-red、pytest-final、build-final、ctest/test-result、installed-sha256和installed-fake运行目录。使用顺序见[回零说明](piper_vision_grasp_runbook.md#六关节回零脚本2026-11-18)。本次没有SSH、部署、真机运动、ROS节点或新标定；机载current仍为先前记录的r2，本轮未连接核对；上次零位保持仅为历史状态，当前反馈待现场读取。未自动提交，现场碰撞路径和新脚本实际运行尚未验证。

### 2026-11-18 默认启发式改为俯仰＋水平朝向搜索（仅本地完成，Go2已关机）

用户要求只由大模型提供目标位置，不限制上方夹取，拒绝后更换下一个夹爪角度；明确选择“俯仰＋水平朝向”，随后说明Go2已关机。本轮没有SSH、设备查询、部署或运动；机器人当前反馈未知，不将上次归零状态当新读数。

semantic.py去掉Qwen提示中的上方抓取要求，仍以标签/框/点定位及SAM+深度得到三维可见中心；增加可见点的基座XY凸包，供转向后的宽度投影。geometry.search_orientations默认依次搜索水平偏角0、±30、±60、±90、±120、±150、180°，每个偏角搜索俯仰40、30、50、20、60、10、70、0、80、90°，共最多120候选；固定同一TCP位置，首个完整路径PLANNED即停止。预抓及入口沿该候选接近方向反向，默认semantic-camera-avoid各100mm；semantic.json没有额外入口时保持0。0°水平、90°向下，不枚举任意滚转或连续SO(3)。关节限位、1mm/1°IK判据、30%速度和夹爪力矩不变；全部拒绝保持WAIT_PLAN等待新观测。

默认两份semantic配置启用angle_search；显式--approach-pitch-deg保留原固定姿态语义并关闭自动搜索，帮助文本与启动事件已说明。GraspNet/旧参考模式/六轴回零保持原分支。main逐候选SEARCHING_IK事件、planning/plan中的orientation_candidates和selected_candidate_index可追溯角度/拒绝原因/目标；report增加搜索表和TCP坐标轴，并使用目标保存的真实预抓偏移，点云报告随之展示选中接近方向。代码改动保留此前未提交的正式zero入口。

独立只读审阅发现“旋转后可见宽120mm、IK通过但close116mm超过SDK100mm，搜索提前停止”的P1；已修复为复用既有open_mm/open_ready_mm/SDK100mm能力，宽度非正或超能力记WAIT_WIDTH并继续，不引入任意新容差。新增用例先失败后通过；审阅原真实IK复现已确认不再接受越界关闭候选。隐藏厚度和实物碰撞仍未知，不能据可见点投影保证真实可夹。

验证：前一版本六组Piper共227项基线；本轮新增13项（角度顺序/失败续试/成功停止/全拒绝、几何与宽度、侧向入口、手动覆盖、非法网格和两配置入口距离）。最终系统238通过/2项Plotly环境跳过，共240实际测试；colcon246条目含6包装、0错误/0失败。既有绘图环境semantic+report共48项全部通过，覆盖上述2项跳过。定向初始基线159通过/2跳过，新增前后区别不以colcon条目替代用例数。

新三包安装build/piper-angle-search-20261118/overlay/install/vlm_nav/share/vlm_nav；9个关键入口/模块/配置SHA与源码一致，从/tmp以隔离Python导入该安装模块重现4条离线路径，安装脚本--help确认固定角度覆盖说明。入口使用同目录tools/config，无导航env/launch旧Piper路径需切换。构建/测试完整环境命令仅在go2_runbook最新角度搜索章节维护；使用说明piper_vision_grasp_runbook最新节。未自动提交或部署；机载仍是此前r2的历史记录，关机后未复核。

离线证据build/piper-angle-search-20261118/results/offline-replay-final：重新使用历史before-3a46621d、before-8d9f2f8b、before-5ec10b45和before-292dc93c的RGB/depth/mask与冻结外参，三份原WAIT_IK和一份原PLANNED均在首候选40°/yaw0°完整通过（约0.3–0.4秒/份，开发机数据）。此处可达改善包含接近路径变化，不能归因于水平枚举或当现场抓取成功；拒绝后续试由新增测试及审阅真实IK复现验证。原证据未覆盖，派生输入replay-summary记录来源。visualization/index.html含4组候选、点云及角度表，已实看3D、检查链接、结果VISUALIZED/offline/CAN发送0；没有实际执行角度。evidence/中保存red/green、宽度修复、最终build/test、安装核对、回放和报告日志。Go2重启后尚需另行部署及新鲜场景/物理路径验证；不使用离线计划驱动硬件。


### 2026-11-19 回零脚本已部署，USB-CAN未枚举，真机回零等待连接恢复

用户明确“进行部署，成功后执行真机回零”，授权本轮部署、SSH检查和一次六轴回零，夹爪保持；尚未执行运动。实际SSH别名unitree-go可达192.168.123.18，主机ubuntu；没有活动抓取/回零程序，仅原piper-boot服务PID675运行。不要把昨天零位保持当新反馈。

使用本会话已构建验证的build/piper-zero-20261118/overlay/install/vlm_nav/share/vlm_nav制作44文件分发包，部署至新独立目录/home/unitree/piper-zero-20261119，保留旧目录及piper-vision-current→/home/unitree/piper-vision-evidence-20261118-r2。没有部署另一任务的角度搜索版本或覆盖当前源码改动。归档SHA256=f40094c0e12b08eb78aab63879c2f8d6f55f03c88ac1957efab4fbced51a85b4，机载归档与44清单全部通过；正式入口--help成功，zero-deploy-check-20261119-01=CHECK_OK。机载解释器/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python，Python3.8.10、numpy1.17.4/scipy1.3.3/OpenCV4.12/piper-sdk0.6.2/python-can4.5.0；未改依赖或服务。

现场阻塞证据：ip link无can0且无其他CAN接口；lsusb无1d50:606f；piper-boot active，但日志持续WAIT_CAN/USB identity match count=0 expected=1。新部署模块的12次readonly快照全部WAIT_IO/同一身份匹配错误，command_attempted=false、CAN发送0。已请用户核对机械臂供电与USB-CAN连接，尚待回复；没有启动潜伏等待后自动运动的回零进程，没有做USB重绑、重启服务或变更网络。恢复连接后继续本次授权，先读取新鲜关节/故障/使能状态，再调用新目录scripts/piper_return_zero.sh执行一次并独立复核。

证据与归档在build/piper-zero-deploy-20261119/，evidence包含remote-inventory、can-diagnostic、package、deploy-check、机载check完整目录和before-passive.json。运行说明已指向新部署目录。本轮仅分发已有验证产物和更新文档，未改产品代码、无需新构建；真机回零仍未完成。

用户随后明确选择“暂时无法连接，先完成部署”。本轮以部署完成结束，暂停真机回零。最后只读复查输出送达时，USB-CAN已重新枚举（1d50:606f，can0 UP），原boot服务10:26:15自行使能、10:26:16报告READY/六轴使能；这是原服务行为，不是本轮回零命令，未读取恢复后的关节角。按用户最新“先完成部署”限制，仍未启动任何回零等待程序，待用户告知继续后再读取实际反馈执行。部署44文件哈希、机载CHECK_OK和文档diff检查通过。


### 2026-11-19 已连接并执行一次回零目标，仍WAIT_TARGET，未到位

用户随后明确“已经完成连接”，恢复继续本次回零。新部署模块12次readonly全部READY：起点[0,0,-0.113,9.010,1.519,-9.898]°，夹爪0mm/使能，六轴使能true、ctrl_mode0、teach0、arm_status/err_code0；这是真实重启后反馈，与昨天姿态/夹爪不同。重新核对44哈希通过、无其他控制任务后，在/home/unitree/piper-zero-20261119调用正式scripts/piper_return_zero.sh --run-id joint-zero-20261119-01。

机载进程PID3733，exec会话74592仍运行。唯一zero-1 receipt：joint目标[0,0,0,0,0,0]，speed_percent30；只发送一次MotionCtrl_2及JointCtrl，未发夹爪、清错、校零、使能/失能或第二目标。模式由standby0变CAN1，日志COMMAND_ATTEMPTED后持续WAIT_TARGET，最大角误差9.898°，初始约99秒存档期间关节完全不变，尚无ZERO_REACHED/result.json，不能称回零通过。诊断tail会话41610已结束，仅结束查看器，未中断回零进程。

只读诊断：can0 ERROR-ACTIVE/1Mbps，TX11帧/88字节、错误0，RX持续增长。2秒原始CAN状态401帧、每组关节400帧、每路低速80帧；0x2a1始终0100010000000000，关节原始字节与SDK反馈一致。不是只靠SDK缓存判断。未捕获新控制帧符合此次只被动观察/没有重发，不据此倒推先前目标是否被固件接受。运行中另起readonly被控制锁拒绝（after-zero-passive.json为WAIT_IO/Errno11），该文件不是运动完成后的复核，也不是CAN失联。runtime.log未见SDK发送错误。SDK方法源码已保存，紧邻发送模式与目标、未等模式反馈；与历史2026-11-13冷启动无动作现象相似，但历史校零/模式多变量，不能据此确诊时序问题或再次校零。

已询问现场是否完全未动/存在阻挡，用户答“不确定”。目前保留WAIT_TARGET监测、不重发命令；没有结束后自动新建周期，也不将使能/无故障当运动完成。下一步需要现场确认实际动作与路径无遮挡，再决定同目标对照或进一步诊断；原目标是否已被控制器保留仍未知，退出监测不代表物理急停。用户未授权新次数实验，不自行增加角度/速度或清零标定。

证据build/piper-zero-deploy-20261119/evidence：before-connected-passive、before-run-integrity、zero-launch、running-snapshot（当时运行目录副本）、wait-target-diagnostic、sdk-methods、wait-raw、final-zero-state、delivery-status。最终状态仍待恢复，不覆盖先前部署CHECK_OK。没有新增产品代码/构建，本轮部署已完成、真机回零未完成。


### 2026-11-19 用户再次授权脚本回零，第二轮成功并独立确认六轴全0°

用户明确“再次进行脚本回零尝试”。先核对旧进程PID3733的cmdline与joint-zero-20261119-01匹配，再SIGINT结束旧监测；旧result=EXITED/previous_state WAIT_TARGET，未发送急停、失能或夹爪指令。新12次只读反馈均READY：关节仍[0,0,-0.113,9.010,1.519,-9.898]°，夹爪0mm，ctrl_mode1（此次起始已CAN）、六轴使能true、teach0/无故障；44清单哈希核对通过，未修改部署文件。

沿正式/home/unitree/piper-zero-20261119/scripts/piper_return_zero.sh，以新run_id=joint-zero-20261119-02执行一次；exec会话65277退出0。唯一zero-1 joint receipt为六轴0°、speed_percent30，目标发出至ZERO_REACHED约0.572秒，到位瞬时J3=0.121°、其余0，满足0.3°容差。没有发送夹爪、零点标定、使能/失能或其他动作。

程序退出后独立12次readonly快照全部READY、无故障/全使能；全部六轴均[0,0,0,0,0,0]°，逐轴跨度0°，夹爪0mm保持。真实运动反馈和独立复核支持本次脚本回零成功，不依赖ArmRobotUA或重新标定。此前从standby起始无动作、此次从稳定CAN模式起始成功，支持进一步调查模式切换时序，但尚不能单独确诊首次根因，不把这次成功写成已修复冷启动问题。

两轮完整目录已保存build/piper-zero-deploy-20261119/results/joint-zero-20261119-01及joint-zero-20261119-02，后者verified.json保存独立验证结论；retry02含before/after快照、运行/诊断日志、summary.json及final-processes.log。最后核对无这两轮回零控制进程；机器保持零位，本次一次性重试授权已完成，不继续抓取或新动作。仅更新文档与现场证据，无产品代码/构建/重新部署/提交；git diff --check通过。此前对现场实物观察为“不确定”，本次确认范围是新鲜CAN关节反馈，不宣称外参/TCP或物理零位标定精度已验证。

### 2026-11-19 角度搜索版本已部署并切换默认入口，仅部署无运动

用户明确“进行部署”，本次范围为角度搜索版本部署/校验；未延伸为新抓取或回零。SSH unitree-go已恢复可达，主机ubuntu，原current指向/home/unitree/piper-vision-evidence-20261118-r2；初始和切换前均核对无抓取/回零控制进程。此前关机及回零状态只作为历史，不读取/宣称本轮机械臂角度。

从已经构建验证的build/piper-angle-search-20261118/overlay/install/vlm_nav/share/vlm_nav分发，新目录/home/unitree/piper-angle-search-20261119。所有分发源文件均与当前源码一致，保留正式回零入口、点云报告和角度搜索；无新产品代码修改、无需重建。归档SHA256=79c268a15a195aa938945abaa0958b6d653838dc3cf17aa75139ab29e450d208，机载归档及44项SHA清单全部通过。未修改机载依赖、boot服务、CAN或网络。

新目录angle-deploy-check-20261119-01=CHECK_OK；机载Python3.8环境中numpy1.17.4/scipy1.3.3/OpenCV4.12及既有SDK依赖通过。纯离线脚本在机载从已保存四个场景观测重新计算，全部首候选40°/yaw0°PLANNED、目标TCP与开发机保存值一致、关闭宽度在既有开爪能力内；约4.75–5.86秒/场景，仅算法回放，没有连接相机/CAN、没有发送运动。完整44项再次校验及无控制进程核对后，原子切换/home/unitree/piper-vision-current至新目录；回退链接/home/unitree/piper-vision-current.before-angle-20261119保留旧r2目标，旧目录和独立piper-zero-20261119目录均保留。

通过current入口再运行angle-current-check-20261119-01=CHECK_OK，日志实际module/config路径均为新目录，解释器仍/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python。默认启发式、10俯仰×12水平偏角、速度30%；显式--approach-pitch-deg仍关闭搜索以保留固定角度语义。最终无抓取/回零控制进程，本次未启动模型服务、相机采集、ROS或真机运动，未新增部署后的硬件反馈验证。后续抓取需新鲜现场观测及实际路径验证，不能使用本轮回放直接运动。

部署包、清单、两次check完整目录、机载离线脚本/输入/结果、激活记录及final-status.json位于build/piper-angle-deploy-20261119。使用说明更新至piper_vision_grasp_runbook末节。文档diff检查通过，未提交。

### 2026-11-19 用户授权切换角度搜索版本并尝试一次红块夹取（进行中）

用户明确“切换到这个版本并尝试夹取”。已核对current为/home/unitree/piper-angle-search-20261119、44文件哈希正确、无旧控制任务。新12次readonly反馈全部READY/全使能/ctrl_mode1/teach0/无故障，六轴均0°，夹爪0mm。恢复本地新版本Qwen/SAM服务service-angle-grasp-20261119-01（CPU）与18765专用反向隧道；既有凭据仅在进程内存使用。先启动angle-red-preview-20261119-01采集新场景并默认角度搜索，不传固定pitch覆盖；本轮预定angle-red-grasp-20261119-01单周期，速度仍30%。证据根build/piper-angle-live-20261119；尚待预览及实际结果，不以昨日存档驱动运动。无产品代码改动。

angle-red-preview-20261119-01捕获before-7ae549f2：红块完整位于画面右侧，相机点[81.606,-42.299,318.000]mm，基座TCP[753.840,-49.175,-13.197]mm，可见宽51.177mm、有效深度比例0.838。机载正在默认角度枚举，截至136秒为candidate75，均端点IK失败，尚未发送运动。同步本地对同一新观测完整120候选复算全部WAIT_IK，最小位置残差5.705mm（俯仰10°/yaw0），不是穷举所有IK分支或机械臂全局不可达证明。仅假设基座X方向移近10/20/40mm、保持物体形状朝向的离线采样分别可在10°/20°/20°规划完整路径；这些假设点不用于运动。已询问用户将红块向机械臂移近约2–3cm后继续同一次尝试，或保持位置查看诊断/结束；只读预览保持，当前无run控制周期。证据evidence/local-preview-plan.json及position-sampling.json（明确hypothetical/not_for_execution）。

用户询问“位置误差”后，已解释是IK角度经FK计算的末端位置与目标位置的距离，不是相机误差或实际抓偏；用户明确“放宽这个通过阈值到8mm”。已SIGINT精确只读preview PID5873并取得EXITED，未有运动。仅修改geometry的可配置位置判据、两份启发式配置为.008及报告阈值标注；姿态1°、关节限位/速度保留，旧参考/GraspNet缺省仍.001，模型副本注入不改原运动学来源。新增8项测试，三包构建成功：246实际通过/2Plotly跳过，colcon254含6包装；绘图环境56通过补验。独立只读审阅无阻断问题。

机载新目录/home/unitree/piper-angle-search-20261119-ik8，归档SHA256=a1ba16dac7ff50e3d56ed9f898a947e104eb98351d08cfb86e189da35bcbd94f，44文件SHA与机载CHECK_OK通过后current已原子切换；旧角度版回退链接before-ik8-20261119保留。证据build/piper-ik8-20261119。对刚才新场景本地重算在candidate5（第6个、俯仰10°/yaw0）PLANNED，最大阶段位置残差5.705mm；仍为离线证据。现启动angle-red-preview-20261119-02重新采图，继续同一用户授权的红块尝试，未启动新的运动周期。该阈值是用户明确选择，不把通过解释为零误差或真实定位精度。

8mm机载新预览angle-red-preview-20261119-02完成PREVIEW_READY，before-1be05f2a目标TCP[753.840,-49.175,-13.197]mm，candidate5=俯仰10°/yaw0°、37阶段、最大位置残差5.69155mm。555姿态相机区域采样：当前外参下3个三角AABB宽相交，逐个三角-盒SAT进一步验证均不相交；人工平移前外参假设下0相交。当前包络顶点最近约0.710mm（该包络有保守扩张，不是实物距离）；旧假设约47.236mm。采样和未实测支架/标定不构成连续物理碰撞保证，原始结果mesh-audit.json保留。初步本地旧图样本同样3个AABB/0个SAT，未隐藏宽检测假阳性。

按本次既有授权开始唯一运动周期angle-red-grasp-20261119-01，current为ik8新目录，默认角度搜索、30%速度、原力矩；不传固定pitch，不发送旧预览目标。程序将重新采图并在开爪后再次定位。已提示离开机械臂活动范围，待记录实际执行和夹取结果。

### 2026-11-19 本次8mm/角度搜索真机尝试：现场报告夹取偏低、推走纸盒，已终止且未抬升

angle-red-grasp-20261119-01重新识别并开爪68.4mm，开爪后再次采图仍选candidate5=10°/yaw0°。实际完成open、35段joint至grasp及close，共37份receipt；最大关节到位误差0.298057°。闭爪后反馈48.3mm/约-0.22Nm，持续WAIT_CONTACT，未执行lift-1/lift。用户现场明确“夹的位置偏低，将下方纸盒推走了”；已立即SIGINT精确PID6892，result=EXITED/previous_state WAIT_CONTACT，独立operator-verification.json标记OPERATOR_REPORTED_LOW_GRASP_BOX_PUSHED，不宣称夹持成功。没有追加抬升、松爪、回零或失能。

退出后12次readonly全部READY/无故障/全使能，最后关节[-4.642,179.926,-156.774,-19.757,-8.605,19.002]°，SDK法兰[615.411,-45.359,11.688]mm，夹爪48.3mm保持。按到达grasp反馈做原URDF链正运动学（不对微小超出模型限位的测量值做规划限位裁剪），使用原135mm TCP，目标TCP[753.840,-49.175,-13.197]mm，模型反馈TCP[747.912,-49.454,-13.769]mm，差[-5.929,-0.279,-0.572]mm、距离5.963mm。不能把现场偏低直接等同于8mm阈值造成的8mm下偏；实物高度偏差、外参/TCP和支撑面间隙仍待测量。

补采angle-red-after-contact-20261119-01仅一次有限RGB-D、无运动，三帧RGB/原始及对齐深度与metadata保存完整；随后Qwen返回requested object not found，脚本退出1，未生成完整物块观测/result。capture-note.json如实注明RGBD_SAVED_DETECTION_FAILED，不伪造检测。实看frame02纸盒已移出原位置，夹爪前方原红块不再完整可见；不据图猜测其精确掉落/位置。记录这次真实接触失败；先前只检查相机包络，未检查纸盒支撑面的碰撞，不能把该检查当整场景安全保证。

模型服务会话51366和专用隧道4187均已结束，本轮没有待自动继续的抓取程序。当前current仍ik8版本，用户指定8mm未擅自回退，也未凭猜测增加抓取高度补偿。代码修改后的完成音频已播放；构建/测试结果见前文。已结束本次单周期授权，后续新运动需用户新指令。

完整证据与统一入口build/piper-angle-live-20261119/results/index.html；实际运行报告angle-red-grasp-20261119-01/visualization/index.html含2组候选、6帧点云/完整PLY、角度搜索、8mm阈值、RGB/3D、变换、35段实测关节及2段未执行抬升、现场失败反馈与闭爪后原图。contact-diagnostic.json保存模型TCP误差，clearance-diagnostic.json仅列模型夹爪及可见目标范围，不是实测支撑面或碰撞证明。已核对报告/链接/实际反馈缺省语义，保留1mm预览和8mm预览原始目录，不覆盖历史数据。


### 2026-11-19 用户要求回零及关节角改数字列表：已完成

本轮仅授权回零和修改报告，未再次抓取。回零前独立12次反馈READY，实测仍[-4.642,179.926,-156.774,-19.757,-8.605,19.002]°、夹爪48.3mm，无正在运行抓取程序。J2比URDF规划上限约多0.017°，现有正式run_zero对起始反馈调用严格FK会等待；本次复用既有一次性return-zero02操作脚本另存return-zero03.py（没有改模型限位或产品控制代码），记录真实起点、校验18个回零目标均在模型限位内、相邻单轴名义变化≤10°，通过现有send_once/wait_joints和控制锁执行，速度30%、到位容差0.3°。起点和整条路径378姿态相机包络采样0重叠、顶点距离约15.687mm；仅是已知相机假设检查，不是支撑物碰撞验收。

机载run joint-zero-20261119-03已结束ZERO_REACHED，18段均AT_TARGET。返回后另外12次只读全部READY，最后六轴[0,0,0,0,0,0]°，夹爪48.3mm；未发送夹爪、零点标定或使能切换命令。原始结果/回执/反馈/采样位于build/piper-joint-table-20261119/evidence，保持零位，无待自动继续的抓取任务。机载current仍/home/unitree/piper-angle-search-20261119-ik8；此次报告代码只在开发机更新，未切换运动部署。

report.py删除候选关节柱状图和六轴时间图，改为J1～J6数字表（3位小数、单位度）。候选表明确采图角不是到位角；执行表显示每阶段目标/最后有效实测/误差（实测−目标）和状态，未执行或无有效反馈保持缺省，不补零。CSV/JSON原始精度、RGB/3D点云/坐标变换图保留。修改覆盖报告模块、现有2个参数化报告用例及runbook，无新依赖。

三包构建通过；基线248实际测试（246通过、2绘图跳过，colcon254包含6包装）无失败。没有新增测试数量，扩展已有2例检查无关节PNG、正负误差数字、未执行缺省与候选标签；绘图环境报告9项全通过，补验2项跳过。安装report.py及入口与源码逐字节一致；从新安装入口离线生成visualization-numeric，CAN发送0。报告核验37阶段中35段有实测、lift-1/lift未执行；没有关节图文件/引用，链接全部存在。旧报告和原数据不覆盖，最新入口build/piper-angle-live-20261119/results/index.html，完整数字报告angle-red-grasp-20261119-01/visualization-numeric/index.html，包含原失败反馈/闭爪后图像及本轮回零状态链接。

新安装路径build/piper-joint-table-20261119/overlay/install/vlm_nav/share/vlm_nav；命令在go2_runbook最新“关节角数字列表报告”节。抓取偏低的实物根因和外参/TCP/支撑面间隙仍待验证，本轮未猜测性加高度补偿，也未重试抓取。正式run_zero对模型边界略外侧的实测值的处理仍未修改；此次复用操作脚本回零证据独立保存。


### 2026-11-19 空间图新增实测关节FK夹爪TCP：离线完成

用户要求在空间图标出实测关节角FK所得夹爪位置。本轮只修改离线报告，无SSH、CAN或运动，未改变机载current。report.link_points复用既有URDF链，末端加R_flange×tcp_offset；绘图不裁剪真实测量角，不修改规划限位。仅对执行计划关联捕获的最后有效grasp反馈显示橙色菱形，场景图连接候选TCP与反馈TCP；静态/交互三维场景、两个点云面板均含标记。summary/点云JSON记录反馈角、时间、状态、TCP与候选差值；网页列基座毫米坐标。无有效反馈为空，WAIT_TARGET仍按未到位标签解释，关节角继续使用数字表。没有把采图角、当前零位或SDK法兰位置冒充抓取TCP。

重生成历史angle-red-grasp-20261119-01到visualization-feedback-fk，旧报告保留。第一捕获无抓取反馈，不显示点；第二捕获before-cf07f145使用AT_TARGET/time1795057774.785795的关节[-4.642,179.964,-156.668,-19.757,-8.485,19.002]°（是抓取到位时记录，不是事后静止反馈），TCP基座坐标[747.911520,-49.453893,-13.768732]mm，相对候选[-5.928768,-0.278636,-0.571783]mm。模型结果与前轮独立诊断一致，不表示实物空间定位精度。报告保留人工失败反馈、闭爪后图像与后续回零说明；总入口build/piper-angle-live-20261119/results/index.html指向新报告。

三包构建成功，248项基线加2项FK用例，共250实际测试（248通过、2绘图跳过；colcon256含6包装），0失败；既有报告用例扩展验证实际标记/元数据及无反馈情况，绘图环境11项全部通过，补验2项跳过。新增FK测试覆盖旋转后的非轴向TCP偏移与略超模型限位的真实测量角。新安装build/piper-feedback-fk-20261119/overlay/install/vlm_nav/share/vlm_nav的report/cloud_report/入口与源码一致，入口解析同安装tools/config。由该安装入口生成报告，VISUALIZED/offline/CAN=0，静态空间/点云图目视确认橙色菱形可见，链接检查通过，未重新加入关节角图。构建命令及报告入口在go2_runbook新增节；证据build/piper-feedback-fk-20261119/evidence。实物TCP、外参精度和此前偏低根因仍未验证，本轮未重新抓取。


### 2026-11-19 四姿态机械臂轮廓点/完整模型叠加，抓取确认后闭爪回零：本地代码完成，未部署

用户明确先修改代码，并选择沿现有路径四个停靠点：pregrasp、现有grasp-N中间路点、grasp、lift；不是额外四个俯仰/yaw动作。两份启发式semantic配置默认motion.record_arm_outline=true、return_zero_after_grasp=true；旧参考和GraspNet默认关闭保持兼容。原8mm位置阈值、30%速度、夹爪目标/力矩、接触和抬升视觉确认均保留。每点三帧原始RGB/深度，逐帧前后机器人反馈；最后一帧由既有Qwen/SAM按机械臂描述独立分割，无FK/模型输入。每点arm-目录保存原始三帧、metadata配对反馈、arm-mask.png、arm-observation.json。采集跨反馈中断、主机配对窗口异常、角度/开口不稳定或分割错误会WAIT_ARM_CAPTURE等待重采，不重复发送已完成动作；不是硬件同步采样。

新arm_outline.py将分割轮廓内侧1像素有效深度反投影，再用冻结外参到基座系；不填补缺失深度，不做隐式配准。完整Piper STL表面按URDF与实测六轴+开口/2驱动，两侧夹指分别用joint7正/ joint8负位移。报告arm-overlay.html蓝色半透明模型/橙色实测轮廓，四姿态下拉切换，含RGB遮罩、轮廓点PLY/NPZ、每姿态数字关节表和深度覆盖率；arm-overlay.json保存模型hash和采图时间窗口。模型URDF原始SHA或仅统一LF/末尾换行后的SHA必须等于冻结运动学来源；实际外部URDF与provenance正是这一文本差异。并非放宽模型几何校验。原始SHA/规范SHA/全部STL SHA均保留。

完整流程仍先原抓取确认，通过后保持原闭爪命令，按实测起点≤10°步长分段回零，不发松爪命令。共享geometry.zero_stages校验全部目标点；不裁剪稍超模型限位的反馈读数，也不改限位。return-plan.json保存加载起点及阶段，根plan追加回零段供报告比较。完成GRASP_RETURNED_ZERO，grasp_confirmed_at=after_lift、holding_at_zero_verified=false，明确回零后是否仍夹住还需现场核验；失败继续等待，不自动重试抓取。该分段回零没有携物碰撞规划能力，下次现场执行前须按实际摆位核验。

执行计划docs/superpowers/plans/2026-11-19-piper-arm-outline-cycle.md。独立review_arm_outline_cycle发现跨WAIT_FEEDBACK恢复会接受长间隔同角反馈，已修复批次feedback_epoch变化重采，逐帧图像host时间必须处于前后读取窗口且跨度≤4s（原相机3s等待+1s反馈新鲜窗口）。新增moved/feedback_outage/long_pair回归，审查复核19项通过无新阻塞。

三包构建通过。原250实际基线上新增19项，共269实际（266通过、3绘图跳过；colcon275包含6包装），无失败；绘图报告14项全通过补验3项跳过，report/vision/semantic三套181项通过。完整真实STL离线渲染验证10部件183746三角面，27MB交互HTML，使用四姿态合成RGB/深度/关节输入，明确标记SYNTHETIC_TEST_ONLY，绝不可当现场标定或四姿态采集证据。测试报告build/piper-arm-outline-20261119/results/synthetic-full-model-final；早期因URDF换行哈希差异失败的synthetic-real-mesh-render保留。

新安装build/piper-arm-outline-20261119/overlay/install/vlm_nav/share/vlm_nav，主要模块/两份配置与源码一致，脚本使用同安装tools/config；完整命令和参数说明在go2_runbook最新“四停靠点机械臂轮廓与闭爪回零”节，证据build/piper-arm-outline-20261119/evidence。机载current仍历史ik8部署，本轮未SSH、未部署、未启动设备；用户下次要求执行时才切换并做现场核验，不能假定新代码已在机载生效。实际分割质量/深度覆盖、外参/夹指几何、主机配对质量与携物回零路径尚未现场验证。


### 2026-11-19 用户新授权执行一次：已部署四姿态版本，USB-CAN待现场重新插拔，尚无运动

用户在代码完成后明确“执行一次夹取”，授权本次单周期；已上传独立/home/unitree/piper-arm-outline-20261119，45项SHA通过，机载arm-outline-check-20261119-01=CHECK_OK（Python3.8/NumPy1.17.4/SciPy1.3.3/SDK0.6.2依赖兼容）。稳定current原子切换至该目录，旧链接/home/unitree/piper-vision-current.before-arm-outline-20261119指向ik8。归档SHA256=dccec8d6225f691ff7c9280c62808c37afffd27c5886b4a3a2412d5752cddd34，包与证据build/piper-arm-outline-20261119。

上位机约20:19时uptime17分钟（发生过新的重启）；boot日志曾正常READY，但此时CAN RX固定25904且12次只读均WAIT_FEEDBACK，六轴/夹爪未知，没有运行抓取控制程序。核对USB-CAN序列0046002F5246570520323934、gs_usb接口1-2.1:1.0和1Mbps。按已有管理员恢复授权定向重绑：解绑成功，bind报EPIPE(-32)，can0消失。进一步仅对已核实USB设备001/003执行USBDEVFS_RESET，内核重新枚举失败error -71/设备不响应地址，ioctl ENODEV。没有启停机械臂、使能切换或位置/夹爪目标发送。当前需现场仅重新插拔USB-CAN的USB端，已通过异步问题请求，等待用户回复；不能把CAN恢复当现场动作通过。

开发机新安装模型服务service-arm-outline-20261119-01已MODEL_READY（执行会话63532），专用SSH反向隧道18765会话9609运行。尚未启动本次preview/run，不存在反馈恢复即自动运动的抓取程序。先等USB重新枚举、恢复1Mbps和新鲜feedback，再只读新图规划/检查路径，之后才执行本次单周期。真实新关节/摆位均未知；不能使用旧零位或旧抓取目标。通信日志live-usb-recovery.log和live-before-passive.json保存；没有记录密码/API密钥。


现场继续：用户重新插拔USB-CAN后枚举为001/009，按原1Mbps启用can0，12次只读全READY。重启后的实际起点[-0.658,-0.106,0,-2.626,0,5.339]°、夹爪0mm、ctrl_mode0/teach0。准备回零01从待机切CAN但关节持续不动（收据已记录，不在该实例重放），已SIGINT结束；以新鲜CAN模式反馈重新规划准备回零02，完成ZERO_REACHED，最终到位时J3尚0.14°、其余0（随后只读预览初始六轴全0）。两次准备回零证据均在results/live，未将01记为成功。待机切CAN立即发送首个关节目标可能被固件忽略，目前仅为推测，代码未自动重发。

arm-red-preview-20261119-01=PREVIEW_READY，识别新红块；选30°/yaw0，TCP[623.085389,17.333657,-4.606591]mm，36阶段，位置残差近数值零。四点pregrasp/grasp-5/grasp/lift。相机包络去程358姿态0重叠、顶点最小37.669mm，预计回零294姿态0重叠、56.980mm；不包含持物/支撑面完整碰撞保证。预览图已目视红块在纸盒后上方，未用旧目标。用户问示教退出SDK及运行环境，已用机载SDK源码确认MotionCtrl_1(grag_teach_ctrl=0x02)再MotionCtrl_2(1,1,30,0)，说明本次已CAN不需另开控制进程。

已启动本次唯一抓取run arm-red-grasp-20261119-01，执行会话12032，使用新current和原30%/8mm默认参数；将重新采图和开爪后重规划，四点采集后原检查通过才闭爪回零。尚不能宣称抓取成功。专用观察脚本evidence/watch-grasp.py，结果路径/home/unitree/.local/state/piper-vision-grasp/arm-red-grasp-20261119-01；服务63532和隧道9609仍运行。


现场预抓暂停与同周期继续：arm-red-grasp-20261119-01完成open和21段joint到pregrasp，共22 receipts；预抓图中机械臂几乎全在视野外，顶部仅一小段，Qwen/SAM持续requested object not found导致WAIT_ARM_CAPTURE。已SIGINT精确PID5260结束监控，保持预抓，尚未合爪。此诊断不可见不应新增运动门槛：arm_outline.segment_capture现仅将模型明确not found记为visibility=NOT_DETECTED、空mask，保留图像/配对关节；网络/相机错误仍等待，报告明确无轮廓点、不代表对齐。新增缺失处理/非notfound错误传播测试，15报告测试通过；三包构建通过，colcon276含6包装/3绘图跳过，无失败（270实际，267通过/3跳过）。

r2新目录/home/unitree/piper-arm-outline-20261119-r2，45SHA+CHECK_OK通过；归档SHA3628390ca385a7164a3d78a8d916a64a0c54d0e2a799379c235433355858d5c2，current已切换，旧r1回退链接before-arm-outline-r2-20261119。API模型worker仍旧实例（分割接口未改）。一次性续行脚本evidence/resume-pregrasp.py复制机载r2根：核对22份先前回执精确匹配，run-id/目录不变，保存pregrasp-pause-result.json和resume-pregrasp.json；确认实测仍在pregrasp±0.3°，重新拍红块检查未移动，随后仅继续剩余阶段，不重复open或先前joint。离线FakeSession基于本次真实计划验证四处采集/只一次close/无先前阶段重复/闭爪回零通过（evidence/test-resume.py），这是操作续行脚本，不是通用重试产品入口。

续行当前执行会话15049（Python路径/home/unitree/piper-arm-outline-20261119-r2/resume-pregrasp.py），12032旧run已退出。若需中止须找resume-pregrasp.py精确PID，不能只扫描main.py。正在恢复观测，未宣称成功；result.json在续行最终覆盖前仍是旧EXITED，监控以最新events.jsonl及进程为准。四姿态报告仅收集成功写成arm-observation.json的目录，其他原始失败采图全部保留。


### 2026-11-19 本轮最终：抓空，用户要求补齐观测抬升；四姿态采集完成、六轴已归零

续行到grasp并close后WAIT_CONTACT，反馈约47.1mm；用户目视确认“抓空或位置偏了”。已SIGINT续行PID6283，没有将其当成功抓取。随后用户新指令“合爪抬升的图也采集，物块没有收集观测作用”，明确允许本次空夹爪抬升以完成机械臂观测。一次性evidence/observe-lift-return.py（机载r2同名）先核对已执行回执准确覆盖open/全部非lift/close，复核实测仍在grasp±0.3°，仅发送原计划尚未执行的lift阶段、记录lift组，然后return_zero_holding。没有再次开爪、没有重放已完成阶段。最终OBSERVATION_RETURNED_ZERO、grasp_confirmed=false、observation_only_lift=true。此脚本本次授权已结束；不是普通入口自动越过接触检查的开关。

实际49个关节阶段全部AT_TARGET且各一次，51份receipt（open、49 joint、close），到位最大角误差0.299071°。四处已采pregrasp/grasp-5/grasp/lift，每处3帧、逐帧前后反馈完整；最后帧主机配对窗口约0.242～0.244秒，不是硬件同步。四姿态关节（°）：pregrasp [5.347,120.320,-61.768,17.823,-24.329,-12.118]、开口68.7mm；grasp-5 [4.581,129.118,-72.728,17.464,-22.038,-12.494]、68.7mm；grasp [3.669,139.739,-88.079,19.573,-17.545,-15.462]、68.7mm；lift [3.669,137.013,-87.977,22.869,-14.976,-18.886]、47.0mm。回零后的12次独立只读全READY，最后J1～J6均0.000°、夹爪46.6mm、CAN1/teach0/无故障。final-passive.json保存。进程清单已无main.py/resume/observe续行；模型服务63532和专用隧道9609已Ctrl+C结束（exit130），没有待自动继续任务。

原始四次arm-observation均NOT_DETECTED。查看原图发现grasp的两夹指清晰可见，确定是原“整台Piper”提示词的局部识别失败，并非机械臂完全不入镜。已改为黑色/金属色的“可见机械夹爪或机械臂局部”，独立RGB测试得到夹爪mask，再对保存的四组图重新分割；未采新图、未动机器人，原始数据和mask不改。派生目录build/piper-arm-outline-20261119/results/arm-red-grasp-20261119-01-outline-reprocessed，包含original-arm-observation/mask及reprocessing.json。预抓仍NOT_DETECTED；grasp-5/grasp/lift分别DETECTED，分割边界有效深度点433/1618/931（总边界1628/4478/1444）；预抓0。已目视RGB叠加：grasp混入右侧线缆，lift混入椅背，明确记为分割未验证/背景污染，不能直接当准确机械臂点或外参标定结果。没有用FK裁剪观测点以掩盖不一致。

最终报告派生目录/visualization/index.html，包含四姿态完整STL与轮廓深度点同基座系交互叠加、RGBmask、全部点PLY/NPZ、数值关节表、49阶段反馈、原抓空结果和回零独立反馈链接。report结果VISUALIZED/offline/CAN=0/arm_poses=4，全部链接及12帧配对/49阶段验证通过；总入口build/piper-arm-outline-20261119/results/index.html。四姿态观测完成不代表成功夹取。本次仍未验证外参/TCP/夹指实物几何，不能把视觉偏差只归因于某一参数。

最新代码r3已构建/部署：新增NOT_DETECTED测试后270实际（267通过、3绘图跳过；colcon276含6包装），绘图报告15项全通过；局部提示词在真实保存RGB上验证能分割夹爪，未宣称无背景误检。机载/home/unitree/piper-arm-outline-20261119-r3，current已切换，45SHA+arm-outline-r3-check-20261119-01 CHECK_OK。归档SHA90af710adccc20a8e94619c2fa6532ca721f05d43cd4b2b675d76baddd20be1b，回退链接before-arm-outline-r3-20261119指r2，早期r1/ik8均保留。本地安装build/piper-arm-outline-20261119/overlay/install/vlm_nav/share/vlm_nav。现场缺失和局部提示词修正已同步runbook；未自动提交。

### 2026-11-19 按用户要求去除四姿态空间图橙色掩膜点

仅修改 arm_outline.render 删除橙色分割轮廓 Scatter3d 图层，并同步 report.py 图例说明及现有报告测试。灰色深度场景抽样点、蓝色完整FK模型、四姿态切换不变；原始RGB-D、RGB遮罩、轮廓导出、关节数字不删。其他抓取图的橙色实测FK位置标记不是掩膜点，保留。没有SSH、硬件采集、运动或部署。

在现有本地 arm-outline overlay 三包构建成功；相关基线267通过/3绘图跳过，colcon276含6包装，0错误/失败；绘图环境报告15项通过补验跳过项，无新增用例，仅更新既有显示断言。源/安装两个修改模块一致。使用安装模块从保存的四姿态数据生成 visualization-no-mask；原 visualization 保留，新页保留此前抓空、空夹爪观测、回零及背景误分的事实说明，并更新图层说明。统一入口 build/piper-arm-outline-20261119/results/index.html 指向新页。机载 current 仍前文 r3，未同步此次仅报告显示修改；现场标定仍未验证。

### 2026-11-19 四姿态空间图标注Piper基座原点并裁剪1米外点

用户要求原点设为Piper基座、剔除1米外点。确认原图已有基座系外参变换，本次未改标定或平移模型：增加黑色(0,0,0)原点标记，三轴范围[-1,1]m；在变换后按点到基座原点的三维欧氏距离≤1m过滤，再按原规则抽样显示。橙色掩膜点保持移除；原始RGB-D、轮廓文件和历史报告均保留，模型/数字关节表不变。新增显示元数据和每姿态有效/范围内/显示点数。

沿用arm-outline本地overlay完成三包构建；相关基线267通过/3绘图跳过，colcon276含6包装，0错误/失败；绘图环境报告15项通过。扩展现有四姿态测试，以非零外参平移验证基座距离、恰好1m边界保留和球形距离过滤。使用安装模块生成 visualization-base-1m，自动逐点核验四组实际HTML显示点均在1m内、原点标记齐全、无橙色掩膜层、链接存在；统一results/index.html已切换。没有SSH、部署、相机采集或机械臂运动；机载仍r3，物理标定未重新验证。

### 2026-11-19 抓取/抬升两帧夹爪配准与人工叠加页

用户要求剪裁夹爪模型并匹配点云提取矩阵，后指定基座系z>-0.03、x<0.66（保留此前半径≤1m）、只用抓取和抬升两帧拟合同一矩阵，最后要求仅显示拟合模型叠加、无橙点。已完成离线实验，无SSH/CAN/相机/运动/部署。产品代码及标定未改；一次性脚本和证据位于build/piper-gripper-registration-20261119。前一人工RGB区域实验及加入背景污染的接近中点实验保留作历史，最终只采用spatial-roi/registration.json中grasp/lift frame02，frame00/01只作验证。

从现有URDF/STL提取gripper_base/link7/link8（3个完整mesh），每姿态按实测开口构建夹爪局部模型；只两夹指表面参与匹配。原深度经原T_base_color_optical转换后严格空间截取；不使用RGB掩膜/人工像素区域、不按FK距离裁剪、不补深度。抓取frame02完整22619点、抬升15262点；拟合2mm体素抽样后1097/787点，各夹指面积加权采样18000模型表面点。使用近FK多初值、85%截断对应、SVD刚体ICP，按所有选中观测点20mm封顶距离均方目标在候选中择优，不保证全局最优。保留候选和失败证据，零5mm匹配支持的接近中点没有当有效结果。预抓/接近中点主要截到红块，未用于最终矩阵；本轮用户明确选择grasp/lift。

最终D（列向量p_observed_base=D@p_fk_base，平移m）为[[0.952435565,-0.140101611,0.270625263,0.063948694],[0.111352691,0.986643900,0.118888150,-0.040116564],[-0.283667187,-0.083098451,0.955315432,0.154574720],[0,0,0,1]]。D不是相机外参；JSON另存C=inv(D)、原外参、假设候选C@T_base_camera（未应用），每姿态D@T_base_gripper及inv(T_base_camera)@D@T_base_gripper。抓取/抬升观测→模型中位距离21.41/24.73mm降至1.543/1.525mm，5mm内比例86.6%/85.1%，但全体RMSE仍8.28/10.07mm。RGB投影目视仍有明显轮廓差，近似可见模型→观测中位距离7.18/8.09mm、5mm覆盖仅36.6%/34.2%；已标PARTIAL_ALIGNMENT_ONLY，不能以低单向距离认定标定正确。四个留出帧仍是同传感器静止帧，不是独立物理验收。

已执行合成刚体恢复/部分重叠ICP/矩阵逆向检查；逐点核验最终grasp/lift两姿态六份完整空间截取PLY满足严格阈值，验证D刚性/正交/行列式/逆矩阵、相机变换方向及报告链接。verification.json为最终两姿态核验。最终inspection.html仅两个实测点云trace和两个共同拟合mesh trace（绿色/灰色，无橙标注），完整PLY不抽样显示，基座系不变，只放大视角。统一arm-outline/results/index.html追加最新链接；原报告/数据不覆盖。无产品代码修改，未运行colcon或部署；脚本运行与自检通过不表示实物标定通过。

### 2026-11-19 根据用户尺度进行夹指可见下侧表面三项加权

用户观察点云在模型内部，要求旋转角差/平移差/贴合分别赋权；确认使用“相机看到的下侧外表面”，差值相对原FK。随后指定1°、3mm、3mm为相近影响。最终三个归一化尺度严格固定为1deg/3mm/3mm，默认权重1:1:1，并保留1:1:10、1:1:100贴合优先对照；先前10deg/30mm的fit-priority日志只为历史，未进入最终报告。仍仅抓取/抬升frame02、同一D，原z/x/1m筛选，其他帧验证，无RGB掩膜，未改源数据。

一次性 weighted_fit.py 使用Open3D首交查询+SciPy优化。对每个候选把观测及相机一起逆变换到原FK模型系；对应点取相机沿测量射线首次碰到的夹指表面，未命中时用最近点方向首交引导。几何对象仅link7/link8，完整夹爪模型含gripper_base用于显示。原指STL winding一致但非watertight，故不用signed distance判“实体内部”；“behind_first_surface_over_2mm”只是沿射线落在首交面后的比例。贴合损失按观测点数/姿态数平均，表面后超过1mm加3倍单边惩罚，10mm以上Huber。旋转为SO(3)角差、平移为夹爪原点变化，不把D的平移列当夹爪位移。等权多初值后追加差分进化/Powell检查局部解，仍是有限搜索候选。

最终证据 `build/piper-weighted-surface-fit-20261119/index.html`，统一arm-outline/results/index追加链接；显示仅绿色拟合模型与灰色全部截取点（无橙色点/原模型层）。等权结果equal-scales-refined.json：角1.1695°，grasp/lift位移7.4196/7.2781mm，外表面中位距离22.8026/24.2900mm，射线命中17.87%/0.13%；仍未贴合。fit-weight-10.json：角3.571°、位移26.08/25.60mm、距离13.12/15.21mm。fit-weight-100.json：角10.274°、位移62.13/65.15mm、距离5.692/6.654mm，命中85.45%/65.44%，首交面后>2mm30.36%/3.05%，不宣称完全消除穿入。各JSON含D、C、每姿态D@FK、假设候选外参（applied=false）、目标分项、holdout及搜索记录，TXT矩阵均为列向量模型→观测、平移米。

验证：合成立方体首交/表面后惩罚、刚性及逆向、三权重独立性；1°旋转/3mm平移/3mm前侧贴合各自归一化贡献为1；数值Jacobian有限。三个实际输出刚性/逆矩阵检查通过，目标分项重算与优化cost一致，截取阈值、HTML四trace只模型+点云与链接检查通过。只新增离线证据脚本和文档，未改产品代码/构建/部署/物理标定；未连接或移动机械臂。默认保留等权结果，贴合优先对照不是用户重新选择的控制参数。

### 2026-11-19 以原始FK实际数值为起点的六滑杆人工拟合页

用户要求RPY/XYZ滑杆供人工拟合，并明确“以初始的rpy和xyz值作为起点”。最终使用原始FK（非自动拟合/非零偏移）作为输入。新页build/piper-manual-fit-20261119/index.html，独立自包含Plotly+两帧完整ROI点云+完整gripper_base/link7/link8模型，无橙色点。统一arm-outline/results及weighted报告追加最新入口。没有更改源RGB-D、外参、产品代码、机械臂参数或机载部署，无运动。

抓取原始RPY[-179.857326847,60.094263007,177.142556570]°、XYZ[506.388854291,23.287183357,63.303155483]mm；抬升原始RPY[-179.892669560,59.974708507,177.131365508]°、XYZ[506.208365111,23.331986291,83.188817494]mm。加载时保留完整精度原矩阵/单位D，不因滑杆步长提前舍入模型。RPY=RzRyRx主动基座系，XYZ绝对位置，输入当前帧T后D=T*inv(F_current)，同一D作用另帧；切换帧更新输入、保留D。恢复初始按钮两帧D归单位阵。逐帧原开口不变。矩阵显示当前T及共同D，JSON导出两帧T、D、inverse(D)、单位/来源和applied_to_robot=false。无自动持久化，刷新恢复原FK；页面说明先导出调整。

实现仅新增证据目录generate.py/manual_fit.js/template.html和测试；原产品模块无修改，不运行colcon。Node测试验证RPY与两个真实FK矩阵一致、±90°万向锁等价、mm/m、XYZ/RPY输入、固定点云、两帧联动、快速连续输入最后状态、非法值、重置及JSON下载事件。真实Firefox155.0.1临时WebDriver会话验证页面/WebGL、初始单位D、XYZ和RPY操作后真实Plotly mesh顶点与计算吻合、Base坐标轴米单位、换帧D不变、重置和导出对象；browser-check.json及browser-preview.png保存。沙箱内Snap启动因DBus transient scope受限，自动审批允许的本机临时Firefox测试成功，不使用用户现有浏览器会话；测试结束会话及driver清理。手动拟合仍需用户目视判断，未做物理标定或控制应用。

### 2026-11-20 Piper 手眼重新采样入口

开发机独立工具 `../piper/handeye/loop.py` 的训练数量选项增加 20。沿用原验证数量 5 和同帧人工确认。控制电脑进入 `/home/isee-pst/unitree_ros2/piper`，在交互式 Bash 执行 `bash piper_handeye_loop.sh --session recalib20_20261120 --train-count 20 --board-attached`；仅在标记板实际固定后启动，从同一命令恢复同一安装批次。原图回传后在本机查看。独立 Python 工具直接加载源码。项目手眼环境 58 项原有测试通过，额外临时离线检查覆盖拒收后完整 20+5 分组与自动结束。未连接设备，本次新样本和当前精度均待现场验证。

同日按用户要求补充同流实时预览。控制电脑手眼循环支持 --live-preview，本机入口与机载运行库路径、操作见 go2_runbook 同日章节。本机安装位于 ../piper/handeye，测试在 ../piper/tests/test_live_preview.py。原有 58 项离线手眼用例与新增 4 项均通过，覆盖单次开关相机、慢编码期间采原图、配对、重传、实际子进程管道和 20+5 分组。尚未连接现场设备，CPU/网络负载和 USB 丢帧率未测。

### 2026-11-20 Piper 手眼 20+5 离线求解

开发机数据 ../piper/handeye-data/recalib20_20261120。核对 25 组 OK、角色及原图/元数据来源哈希，按既有 core.py solve 使用全部 20 组 train 拟合 T_base_color_optical 与 T_end_marker，pose27–31 仅验证。训练闭环平移/旋转 RMS 为 20.638 mm / 3.466°，独立验证为 22.655 mm / 3.663°，最大 33.408 mm / 5.073°。相机原点在 Piper 基座中为 [371.690,46.685,-150.677] mm。全精度候选见[结果 JSON](piper_handeye_recalib_20261120_result.json)，[逐组闭环、像素与 FK/SDK 差值](../../build/piper-handeye-recalib-20261120/residuals.csv)。25 组 URDF link6 与 SDK 末端最大差 0.134 mm / 0.0051°，属于软件一致性检查。pose27→28 的 SDK 末端与视觉标记相对旋转角分别为 20.31°、24.76°；刚性安装理想条件下两角应相等，差异来源待查。此次仅离线求解，未更改活动外参，未连接设备或执行实物验证。

### 2026-11-20 Piper 新批次的逐张与每五组质量反馈

控制电脑 ../piper/handeye/loop.py 调用 quality.py；core.py 提取原 PARK 拟合和残差计算供阶段诊断复用。已安装，原有 62 项与新增 2 项离线测试通过。回放上一批：pose22 单帧 0.52 px、与 15 个已收录姿态比较的转角差中位 0.65°，最大 8.09°（pose12）；前 5/10/15 组分别对接下来 5 个训练姿态预测的平移 RMS 为 20.94/34.71/36.82 mm、旋转 RMS 为 2.29/3.50/6.92°。独立验证留到五组齐全，不用于训练。未修改现用外参。已打开新批次 recalib20_20261120_r2 的操作窗口，初始等待现场板固定确认；窗口创建本身不证明已取图。操作及诊断字段见 go2_runbook 同日章节。

### 2026-11-20 Piper 重采 r2 离线求解

recalib20_20261120_r2 已 COLLECTION_COMPLETE。核对全部 25 个 OK、角色、原图与来源元数据 SHA256，从原图复算，20 组拟合、pose28–32 留出验证。训练平移/旋转闭环 RMS 19.966 mm / 0.704°，验证 15.696 mm / 0.699°；验证最大平移 24.708 mm（pose28），最大旋转 0.944°（pose31）。相机原点在 Piper 基座中为 [340.711,27.310,-113.359] mm。正逆矩阵与逐组闭环 RMS 复核通过。20 个已收录训练姿态中最大的两两相对转角差为 1.458°（pose04/26）；已有近似左右对称观测没有控制图像位置与参照，无法据此确认固定侧偏差。候选见[完整结果](piper_handeye_recalib_20261120_r2_result.json)，[逐姿态误差](../../build/piper-handeye-recalib-20261120-r2/residuals.csv)。尚未实物验证，未应用或部署。

### r2 续采准备（30+5）

独立扩采批次 ../piper/handeye-data/recalib30_20261120_r2_ext 已复制原 r2 全部 25 个批准样本；复核原图、审核来源哈希、原角色和派生元数据，图片逐字节一致。原数据集为 20+5，扩采起点为 25 个 train、0 个 validation，接下来 pose33。旧 5 个 validation 仅在扩采批次改为 train，seed_manifest.json 留存来源，新结论须用另采的 5 个 validation。控制机采样入口支持 --train-count 30；64 项原有离线测试通过，另模拟拒收、25→30 训练及 5 个新验证通过。桌面扩采窗口设现场确认后才连接设备。准备工作没有生成 30+5 结果；实际安装连续性及新增进度以现场和采样记录为准。

### 2026-11-20 扩采 30+5 正式复算

recalib30_20261120_r2_ext 已 COLLECTION_COMPLETE。逐一核对 25 个种子样本来源和原始角色、10 个新增样本的 OK 记录、35 张图像及审核元数据哈希。原图正式复算：新增训练 pose33–37，独立验证 pose38、39、40、42、43，pose41 人工 NO。训练平移/旋转闭环 RMS 21.369 mm / 2.540°；验证 39.552 mm / 5.942°，最大 43.956 mm（pose43）/ 6.966°（pose42）。同一 5 个新验证上，原 r2 的 20 组解为 40.028 mm / 6.999°，仅用 25 个种子重新拟合为 41.024 mm / 7.109°。均用各自训练集估计的固定末端到板变换。前 25 个姿态内部的相对转角差最大 1.458°，新增 10 个内部最大 1.322°，跨两段最大 7.484°（pose24/40）；具体物理来源未确定。候选相机原点在基座中为 [340.994,20.672,-109.638] mm。已复核正逆矩阵和训练/验证统计。仅保存[正式候选](piper_handeye_recalib30_20261120_r2_ext_result.json)和[逐姿态对照](../../build/piper-r2-extend-30plus5/ext30-residuals.csv)，未应用、部署或执行实物验证。

扩采 35 姿态逐对复核：以 pose24 和 pose40 分别为参照，各与其余 34 个姿态计算相机和末端的相对转角及其绝对差。pose24 对原 r2 其余 24 组的差值中位数 0.373°、对新增 10 组为 2.661°；pose40 对原 r2 的 25 组为 3.932°、对新增其余 9 组为 0.811°。两侧最大均为 pose24↔40 的 7.484°；成对差异不能单独定位问题姿态。[逐对转角 CSV](../../build/piper-r2-extend-30plus5/pose24-pose40-angle-pairs.csv)

### 2026-11-20 用户选定第二轮原始外参

后续 Piper 手眼标定和实物验证统一引用第二轮 [r2 原始 20+5 结果](piper_handeye_recalib_20261120_r2_result.json)，批次为 `recalib20_20261120_r2`。已核对结果文件 SHA256 与扩采前记录一致。30+5 扩采数据和求解结果仅留作历史排障资料。本次只记录选用决定；仓库抓取配置仍引用 2026-11-17 的外参，r2 的实物精度待验证。

### 2026-11-20 r3 独立采样 20+8

新批次 recalib20v8_20261120_r3 初始为训练 0/20、验证 0/8。控制机 loop.py 已支持 --validation-count 8，原命令默认 5。原 64 项和新增 1 项离线测试均通过；新增回归覆盖 NO、20+5 时暂停、恢复后补齐最后 3 个验证姿态。已打开现场操作窗口，当前批次进度须按实际采样记录核对。r2 原始选定外参仍独立保存。

### r3 前五组图像与平移尺度诊断

只读查看 pose01–05 的完整原图。五张目标黑框完整，pose04 最斜、像素面积最小（约 1.94 万 px²）。按记录 142 mm 重算的逐姿态闭环为 4.95、20.73、8.25、32.62、21.66 mm，RMS 20.28 mm / 0.81°。替换亚像素/轮廓角点后平移 RMS 仍为 19.97/20.19 mm。相对运动沿各自转轴的位移（理想刚体共轭下相等），pose02→04 末端/视觉 102.9/152.8 mm，pose03→04 为 −68.2/−101.6 mm。整体尺度拟合约 0.686，对应有效边长约 97.4 mm，仅为五组数据推算。用同一套 PARK/PnP 在内存假设 100 mm 黑框重算，五组 RMS 4.37 mm / 0.81°，四拟合一留出汇总平移 RMS 从 35.14 降至 8.58 mm。此对照有待实测黑框横竖边长、核对平面度和度量来源；未改采样记录、尺寸配置或现用外参。不能将假设值用作标定真值。

### r3 新板实测 100 mm，前五组已重算

用户确认 100×100 mm 为 r3 前五张标记的黑框外沿尺寸，量尺时板相对末端及相机相对基座均未动。已保留原始审核记录和 142 mm 数据，新批次 ../piper/handeye-data/recalib20v8_20261120_r3_100mm 带独立 board.json；前五张原 PNG 字节和来源哈希核对，新样本沿用原 CAN 反馈/静止窗口，从原图重算 ID25 的 PnP。平移闭环 RMS 20.276→4.374 mm，旋转 0.813°，最大平移 6.280 mm（pose05）。控制机实时入口将批次板配置显式传给单个机载相机进程，solve 默认优先使用同批 board.json，旧数据集保留原默认配置。已打开新窗口，起点为 pose06、训练 5/20、验证 0/8。65 项原有测试（含扩展的相机管道模拟）及 1 项新增自动选批次板的求解测试通过。新窗口需要旧窗口先退出；现场新帧与独立验证待采，不替换 r2 外参。

### r3 完整 20+8 按实测黑框 100 mm 求解

原采样批次 recalib20v8_20261120_r3 已 COLLECTION_COMPLETE，28 张均保存为旧的 142 mm 元数据，另外 100 mm 续采目录只有同源前五张。仅从完整批次原图和同期反馈按 100 mm 重新检测，核实 28 个唯一图像哈希、审核来源、OK 角色及静止窗口，派生数据保存在 build/piper-r3-board100/r3_100mm_20plus8。全部 20 组训练，pose23–30 八组独立验证。训练闭环 RMS 4.312 mm / 0.819°；验证 RMS 3.711 mm / 0.883°，最大 5.550 mm（pose23）/ 1.609°（pose26）。同一验证集按旧 142 mm 为 22.220 mm / 0.883°。相机原点在 Piper 基座中为 [354.844,33.748,-65.293] mm。已复核正逆矩阵和逐样本残差。[候选矩阵与检测结果](piper_handeye_r3_100mm_20plus8_result.json)、[逐组闭环 CSV](../../build/piper-r3-board100/residuals_100mm_20plus8.csv)。未部署，未做实物点位验证，未替换用户此前选定的 r2 外参。

### 2026-11-20 用户选定 r3 并应用于本地 Piper 抓取配置

用户最新选择为 [r3 100 mm 黑框 20+8](piper_handeye_r3_100mm_20plus8_result.json)，取代此前选用 r2 的决定。共享 config/piper_vision_grasp/extrinsics.json 采用该结果的完整 T_base_color_optical，来源文件 SHA256 为 6227be2823bb01f13ecc8dd022dca7062ccdf42be9df15bedd31dc67fcc1b3de，外参配置 SHA256 为 36d057b5d3e91602a3b5b0bdda8449ded28bcd97d98ce49325545ec415a48667。开发机新安装 build/piper-r3-applied/overlay/install/vlm_nav/share/vlm_nav；默认入口离线 CHECK_OK，四种配置与归档矩阵逐元素一致。三包构建成功，Piper 相关原有 270 个用例（含转为历史来源的人工平移断言）加 1 个 r3 回归，268 通过、3 个绘图用例跳过，绘图环境 15 个报告用例全部通过。构建、检查快照与测试日志在 build/piper-r3-applied/evidence。机载未部署，未执行运动或实物验证。
