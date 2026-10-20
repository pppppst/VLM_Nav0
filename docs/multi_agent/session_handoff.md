# Go2 控制电脑会话交接（2026-10-08）

> 最新行为以文末2026-10-20独立C/取消保护区章节为准；此前资源许可与顺序退出记录属于历史版本。


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
