# Go2 cooperative tasks

Repository: `/home/isee-pst/unitree_ros2/VLM_Nav`.
ROS: Humble. New sources: `cooperative_ws/src` (root `vlm_nav` remains a separate package).
Current state, installation location and unresolved work: [session_handoff.md](session_handoff.md).
This runbook lists commands, not authorization to start hardware or a new experiment.
Task semantics: Go2=A, Ranger=B; both fresh READY before C; same C observation;
Go2 deposit **and whole-body exit** before Ranger. No physical arm.

Field preparation and independent Go2 checks: [go2_field_checklist.md](go2_field_checklist.md).
The independent entry defaults to read-only and does not unlock cooperative tasks.
Optional internal furniture selection and offline catalog checks: [go2_semantic_search.md](go2_semantic_search.md).
Dynamic costmap final overlay, isolated stationary acceptance and Go2 map/scene
collection: [dynamic_costmap_stage.md](dynamic_costmap_stage.md). The default AMCL
profile remains unchanged; select the cooperative override explicitly.

## Go2 独立人工点连续探索

`multi_agent_go2_check --action explore`复用协同Go2 adapter，只执行Go2自己的点位列表；
不启动coordinator、Ranger、bridge、硬件或ARM，不修改双机READY/资源协议。
2026-10-09用户再次明确替换旧点，以四次map下点击按中心→方向配成两个点，目标为灭火器。
当前探索顺序如下；表中显示6位小数，源scene.yaml保留计算精度：

| 顺序 / ID | x（米） | y（米） | yaw（弧度） |
|---|---|---|---|
| 1 / go2_view_01 | -0.374534 | 1.998225 | 1.911663 |
| 2 / go2_view_02 | 0.936335 | 2.581040 | 0.229311 |

源`scene.yaml`已记录A和这两个点；yaw=atan2(方向y−中心y,方向x−中心x)，
约109.530228°和13.138574°。点击z不用于二维目标；点位替换时enabled及验收标记未修改。
本入口不要求B/C目标或Ranger标定；完整协同live仍关闭。

执行顺序：选点→全局costmap停车footprint检查→ComputePath→资源路径检查→
再次检查全局停车footprint→NavigateToPose→
确认动作终态及物理停止→四方向单目标扫描→下一点。
规划失败/停车受阻/保护区域无许可时记录并跳过；导航失败只有确认停止后才换点。
定位、实时costmap、执行路径、独占运动或停止能力失效，以及扫描API/结果缺失时停止整轮。
找到目标停止搜索，不发送接近或拾取目标；DETECTED但无有效投影单独记录。
列表耗尽报告EXHAUSTED，仅表示本次配置列表搜索结束，不证明区域不存在目标。
点位在启动时读取，不热更新，不自动生成新点。
目标预检使用goal_costmap_topic=/global_costmap/costmap_raw的完整Nav2 Costmap；
仅explore进程订阅该必需输入，不增加RGB或点云诊断。局部costmap保持3×3米，
继续用于原有健康检查、近处接近候选及Nav2运行避障，不用它否决远处探索点。
全局输入缺失、过期、payload无效或costmap时刻TF不可用时拒绝/停止本轮；
完整footprint出全局边界、碰到253/254或255未知格仍跳过该点，不只检查中心。
全局目标预检单独使用`goal_costmap_max_age: 3.0`，同时检查原header年龄及单调接收年龄；
这是针对实测1 Hz及一次2秒接收间隔的本轮实验初值，不是安全推导的时限。
读取缓存不续期。局部运行健康仍使用`costmap_max_age: 1.0`，路径接收仍为5秒。

构建使用Go2控制电脑新终端、仓库根目录，独立输出，不覆盖此前现场安装或退出证据。以下固定输出已在本轮构建成功，
启动实验无需重复构建；仅源码再次修改后在同一输出重建。
下面构建只更新multi_agent_nav；vlm_nav和任务消息复用已验证依赖，两个源码根均参与发现。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-camera-tf/install/local_setup.bash"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export GO2_EXPLORE_OUTPUT="$PWD/build/go2-goal-costmap-20261009"
colcon --log-base "$GO2_EXPLORE_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --packages-select multi_agent_nav \
  --build-base "$GO2_EXPLORE_OUTPUT/build" --install-base "$GO2_EXPLORE_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

构建退出0后，局部测试使用同一输出，并优先导入当前源码。运动测试使用fake/stub；
2项既有observer检查短时创建localhost/Domain31节点并退出，不需要完整验收环境，
不发送真机目标；不重跑全量项目测试。

```bash
source "$GO2_EXPLORE_OUTPUT/install/local_setup.bash"
export PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:${PYTHONPATH:-}"
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_exploration.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_scan_evidence.py \
  cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py \
  cooperative_ws/src/multi_agent_nav/tests/test_region_guard.py \
  cooperative_ws/src/multi_agent_nav/tests/test_agent.py \
  cooperative_ws/src/multi_agent_nav/tests/test_coordinator.py \
  cooperative_ws/src/multi_agent_nav/tests/test_safety_regressions.py \
  cooperative_ws/src/multi_agent_nav/tests/test_live_gates.py -q
```

实机执行前须有真实Domain0感知/定位/Nav2/安全监督、站稳的Go2及现场急停操作者。
起立时序：相机、配置/环境和API配置检查可在趴姿完成；在执行
scripts/run_go2_preflight.sh之前，用既有手动方式让Go2站稳。
从30秒传感器预检、临时驱动退出、go2_system的FAST-LIO初始化，到AMCL初始化/Nav2
就绪检查均保持站立静止，满足运动门槛后才ARM并启动探索。脚本不自动起立，
ARM仅解锁Sport速度转换；预检程序检查静止数据，不自动证明站姿。
若预检采集或定位初始化期间改变姿态，应停止该轮并重新取得稳定状态及新鲜证据。
保留活动地图、相机TF/深度、实时costmap、真实路径输入及停止余量要求；
入口在整轮实验开始时收集实际输入并检查既有运动门槛，不要求另做逐项真机测试。
用户已确认本次无共享资源，独立入口允许明确的resources: {}，已有保护区域仍须实测，
完整协同仍要求篮子。不把验收false直接改true或填演示多边形。
原manualnav2/VLMNav脚本含SSH/录包/运动流程，不能作为本节无录包启动器。
本入口不会自动补齐底层节点。配置缺项会在ROS初始化和运动锁之前退出1并写REJECTED证据。

终端1在已具备上述底层输入的环境下设置当前构建路径（实际位置见交接），加载环境。
终端2用于RViz，终端3/4用于Sport转换和探索；四个终端均 source 同一文件。不要沿用地图标注的Domain31；不最后source旧overlay。

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
```

公共环境由根目录 `CoVLM_env.bash` 维护：加载Humble、XT16、Go2、任务、当前探索、相机TF overlay，
最后沿用 `scripts/go2_network_env.sh` 核对enp4s0/192.168.123.x并设置Domain0/CycloneDDS。
脚本切换到仓库目录，设置REPO、各GO2输出路径、GO2_EXPLORE_CONFIG、
VLM_NAV_EXTRA_SETUP_FILES和ROS_LOG_DIR；source失败返回非零，停在本步排障。
只加载环境，不启动节点；API密钥仍由终端4按已有方式配置，不写入公共脚本。

启动探索的终端须核对实际导入路径；重启底层节点不会切换旧终端的Python overlay。
以下只读检查应输出当前`go2-goal-costmap-20261009/install/multi_agent_nav`前缀及
`CURRENT_GLOBAL_GOAL_CODE=OK`及`CURRENT_VLM_RECORDING_CODE=OK`，退出0后才继续。不通过时重新加载上面的环境块，
不要仅依据`ros2 pkg prefix`判断Python模块已切换。

```bash
ros2 pkg prefix multi_agent_nav
/home/isee-pst/venv/co-nav-real/bin/python - <<'PY'
from pathlib import Path
import multi_agent_nav.go2_check as check
import multi_agent_nav.go2_exploration as exploration
import multi_agent_nav.scan_evidence as evidence
import multi_agent_nav.adapters.go2 as adapter
expected = Path('build/go2-goal-costmap-20261009/install/multi_agent_nav').resolve()
for module in (check, exploration, evidence, adapter):
    actual = Path(module.__file__).resolve()
    print(actual)
    assert expected in actual.parents, '旧Python overlay仍覆盖当前探索包'
print('CURRENT_GLOBAL_GOAL_CODE=OK')
print('CURRENT_VLM_RECORDING_CODE=OK')
PY
```

终端2先启动并保持RViz，Map状态应OK。该配置显示/map、local/global costmap、
绿色实时车体轮廓和黄色导航障碍点云/vlm_nav/obstacle_cloud，保留MarkerArray及人工采点工具。
点云是Nav2既有过滤输入，只显示最新帧，Best Effort/Volatile；没有额外点云处理节点或RGB订阅。
两张costmap为Reliable/Transient Local的OccupancyGrid显示，不把costmap_raw接到Map插件。
该配置不提供Nav Goal或定位重置工具。固定map、俯视视角用于停车中心/朝向采集。

```bash
rviz2 -d "$PWD/config/go2_exploration.rviz"
```

已有RViz时，在该窗口用File→Open Config选择上述源文件，或仅退出旧RViz后在其终端
重新执行命令；底层保持运行，无需构建。确认Global Costmap、Local Costmap、Robot footprint
和Navigation obstacle cloud均Enabled、Status=OK后再采点；若显示重叠，临时切换Displays的
勾选或Alpha比较局部/全局，不修改Nav2参数。Robot footprint是当前车体轮廓，约0.76×0.36米，
不是候选点预览；可用Measure辅助留出完整车体范围。障碍点云只展示Nav2过滤后的障碍，
不是完整环境点云。可见空白不代替全局costmap及完整footprint停车检查。

若当前相机、雷达/IMU/FAST-LIO、AMCL及Nav2仍在运行且已设初始位姿，保持它们运行，
不重启、不重跑传感器预检。只需使用上述环境启动新实验进程；停止其他上层目标owner，
不要在RViz发Nav Goal，不启动coordinator/任务agent或原单机VLM。

### 底层已运行时，两个新点的完整实验顺序

2026-10-09四条map下点击已替换源scene.yaml及当前安装配置；按本节两个点的顺序运行。
已核对有限数值、中心与方向不重合、yaw计算、列表正好两项及其余配置保持，
offline配置检查通过；没有另跑新点真机停车/规划验收，实际判定在完整探索中观察。

终端A为已有相机、雷达/IMU/FAST-LIO、AMCL/Nav2底层，保持运行，不重新设初始pose。
终端B用于RViz、终端C用于Sport转换、终端D用于探索；B/C/D均先执行前文公共环境块。

1. 终端B复用已有RViz并Open Config加载当前源go2_exploration.rviz；没有窗口才执行
   前文rviz2命令并保持运行。确认map及四个新显示Status=OK，采点工具不发导航目标。
2. 终端C复用已有唯一twist_to_go2_sport_bridge；若没有，执行前文真实Sport转换命令，
   初始DISARMED并保持终端运行。本次只读查询未发现该节点，不能假定底层launch启动了它。
3. 终端D执行前文实际Python路径检查，通过后按下面只读配置命令核对两点。
   命令退出0只表示配置可进入本轮单机实验；完整live_gates仍有未验证项。

```bash
/home/isee-pst/venv/co-nav-real/bin/python - <<'PY'
from multi_agent_nav.config import load_config
from multi_agent_nav.go2_exploration import measured_viewpoints
config = load_config('cooperative_ws/src/multi_agent_nav/config')
points = measured_viewpoints(config)
assert len(points) == 2, '本轮必须使用两个新点'
for point in points:
    print(point['id'], point['pose'])
PY
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG"
```

4. 终端D按下文桥接状态分支处理：FAULT须先reset至DISARMED；DISARMED才ARM；
   已ARMED不重复ARM。现场机器人站稳、急停可用、外部Sport控制请求静默；
   ARM返回success=true/ARMED后才启动。视觉API环境按既有方式配置，不打印密钥。
5. 终端D执行下文300秒完整探索命令，用新证据目录；A/B/C保持运行。
   观察两点SELECTED→PLANNED→NAVIGATING、执行路径接收/更新、导航终态和物理停止，
   随后SCANNING/SCAN_RESULT。停车受阻或规划失败按实际事件处理，不改标记解锁。
   找到目标可提前结束；未找到则依次尝试两点，EXHAUSTED不等于到达或完成扫描。
6. 探索结束后先确认机器人实际停止，再在D执行下文DISARM；若停止未确认先按急停流程处理。
   失败后查看本轮result.json/events.jsonl，不重跑所有底层或逐项故障注入。

若底层已关闭，按以下分支冷启动。每个终端先执行上述环境块，均在仓库根目录。
先按既有手动流程起立并站稳，现场保留急停操作者，Sport转换保持DISARMED。
终端3启动相机并保持运行（已有相机时跳过）：

```bash
export VLM_NAV_EXTRA_SETUP_FILES="$GO2_EXPLORE_OUTPUT/install/local_setup.bash:$GO2_CAMERA_OUTPUT/install/local_setup.bash"
bash scripts/01_camera.sh
```

终端4在雷达/IMU驱动尚未运行时进行既有传感器预检；它不是路径故障注入。
预期PASS、临时驱动已退出、正式报告healthy=true；失败先修复传感器，不继续下一步。

```bash
export VLM_NAV_EXTRA_SETUP_FILES="$GO2_EXPLORE_OUTPUT/install/local_setup.bash:$GO2_CAMERA_OUTPUT/install/local_setup.bash"
export GO2_PREFLIGHT_REPORT=/tmp/go2_costmap_preflight.json
bash scripts/run_go2_preflight.sh
```

上一步成功后，同一终端4启动底层并保持运行；保持站立静止直到FAST-LIO初始化完成，
再在RViz工具栏添加SetInitialPose设置实际初始位姿。已有正常初始位姿时不重新设置。
预期地图显示、AMCL定位成立并进入Nav2阶段；未就绪不ARM。

```bash
ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:="$GO2_PREFLIGHT_REPORT" \
  sensor_preflight_config:="$REPO/config/go2_preflight_xt16.yaml" \
  calibration_file:="$REPO/config/go2_calibration_xt16.yaml" \
  spark_config_file:="$REPO/config/spark_fast_lio_go2_xt16.yaml" \
  target_stage:=nav2 localization_mode:=amcl \
  map:="$REPO/maps/lab_rear120_20261001_IwQjMX/lab.yaml" \
  amcl_scan_topic:=/scan start_bridge:=false \
  costmap_overrides_file:="$GO2_EXPLORE_CONFIG/nav2_go2_cooperative_costmap.yaml"
```

`start_bridge:=false`不会启动Sport速度转换。终端5执行环境块后，在没有既有同类节点时
启动一个真实Sport转换并保持运行。它是底层速度接口，不是Domain40协同DDS桥。
预期初始DISARMED；不得启动第二个转换或其他Sport控制源。

```bash
ros2 run vlm_nav twist_to_go2_sport_bridge --ros-args \
  --params-file "$REPO/config/go2_bridge.yaml" -p dry_run:=false
```

底层就绪、Go2站稳、急停可用且视觉API环境已按现有方式配置后，终端1解除Sport转换锁。
不打印或写入API密钥。预期success=true、消息ARMED；失败不继续探索，按返回原因排障。

```bash
timeout 10 ros2 service call /twist_to_go2_sport_bridge/arm std_srvs/srv/Trigger "{}"
```

ARM后遥控器或其他客户端发出外部Sport请求会触发FAULT并关闭CONTROL_ARMED；
重启探索进程不会解除此控制锁。若本轮报告`control_armed_gate=false`，保持底层运行，
在上述环境下先只读查询，预期恢复后为ARMED：

```bash
timeout 10 ros2 topic echo /vlm_nav/go2_bridge_state std_msgs/msg/String \
  --once --qos-reliability reliable --qos-durability transient_local --no-daemon
```

FAULT时，由现场操作者先停止外部控制请求、确认机器人静止且急停可用，再调用：

```bash
timeout 10 ros2 service call /twist_to_go2_sport_bridge/reset_fault std_srvs/srv/Trigger "{}"
```

须返回`success=true`及`bridge is DISARMED`后才继续。已是DISARMED时跳过reset；
随后按上面的arm命令恢复，只有`success=true`、ARMED才启动探索。外部请求须静默至少
既有配置2秒；ARM失败按返回原因处理，不关闭foreign Sport保护。

2026-10-09针对失败的只读global grid采样显示：此前三个旧点都在全局边界内，
但完整车体覆盖253/254格；仅中心格空闲不够。证据见交接第27节。
此前三个点已被本节的新两个点替换；旧采样不证明新点可用。
若新点被FOOTPRINT_BLOCKED，不重复ARM/重启来绕过；操作者依据真实全局costmap
重新测量完整车体可停驻的中心/朝向，再按“追加人工探索点”的采点流程更新列表。
保留障碍/未知格否决，不自动移动点位或把该采样当作稳定性验收。

门槛和现场状态确认后，终端1明确启动一次最长300秒的探索，输出目录必须不存在。
先停止其他上层目标owner；不得启动原单机VLM或任务coordinator并行发目标。

```bash
export GO2_EXPLORE_EVIDENCE="$REPO/build/go2-field/explore.$(date +%Y%m%d_%H%M%S)"
ros2 run multi_agent_nav multi_agent_go2_check \
  --config-dir "$GO2_EXPLORE_CONFIG" --action explore --allow-motion \
  --target 灭火器 --timeout 300 --output-dir "$GO2_EXPLORE_EVIDENCE"
```

目录后缀使用本机时间，如 `explore.20261011_143025`。这里只设置路径，入口独占创建目录；
同名目录已存在会拒绝，不提前mkdir、不覆盖；重新取当前时间后再执行。
终端最终JSON省略footprint/path坐标数组，以footprint_count/path_count表示数量；
故障原因、健康门槛、结果、停止确认和证据状态继续输出。完整报告仍保存在result.json，
逐事件证据仍在events.jsonl，停车footprint和路径安全检查照常执行。
启动先收集2秒输入；若RGB-D仍未就绪，在本次--timeout总预算内继续被动等待，
记录WAITING_INPUTS，不发送运动目标。输入成立后还检查全局目标costmap及原定位/静止/控制等门槛；
等待耗尽则REJECTED（startup input wait timed out），不绕过RGB-D或图像时间TF要求。
`events.jsonl`逐事件flush/fsync，记录点ID、map位姿、plan_reachable、状态和扫描结果；
同时记录MOTION_REQUESTED/ACCEPTED的目标及native UUID、PATH_RECEIVED的原header、
单调接收时间和receive_interval_s、plan_stamp_changed、PATH_RECEIVE_TIMEOUT、PATH_CLEARED、
CANCEL_REQUESTED、MOTION_TERMINAL的原生终态及STOP_COMMAND_SENT/STOP_OBSERVATION。
同几何路径仍更新缓存；header变化支持追踪计划更新，但单凭变化与否不能判定是否重规划。
每个扫描方向复用当前冻结RGB，在既有视觉worker中写入
`vlm/<随机ID>_view_00/`（编号为view_index，目录完整路径见VLM_VIEW_RESULT事件）：

- `input.jpg`：与API实际提交的JPEG字节一致，请求前保存，不是另外拍摄的画面。
- `response.txt`：模型返回的原始文字（配置密钥若出现在文字中则脱敏），包含无效JSON；
  传输/API错误没有模型响应时不会伪造这个文件。
- `result.png`：本地按模型返回像素坐标标注输入图，附状态与置信度；负结果不画目标标记。
  当前API返回文字/坐标，此图是本地生成的结果图，不是服务端生成图。
- `metadata.json`：点位/扫描序号、图像stamp/frame/sequence、提交时实际位姿、
  导航目标和任务身份、模型/响应及request ID（SDK提供时）、单调耗时、解析结果与错误类别。

`api_returned`表示SDK取得响应，`validation_success`表示结果结构校验通过；
两者都不证明语义检测正确，不编造成功HTTP状态。逐帧`VLM_VIEW_RESULT`记录
APPLIED/ERROR/TIMEOUT/CANCELED与对应observations，不等整组SCAN_RESULT；
后续旋转失败仍保留已有图像/结果。API/结构异常仍是API_ERROR，不转成NOT_DETECTED。
正常返回时完成本方向写盘后才进入下一次Spin；超时后也等待旧worker结束再转向，
等待仍受整组扫描超时和健康检查限制。取消先发实际取消/zero，再记录本方向取消。
取消/超时后HTTP工作线程可能仍在返回，报告`vlm_evidence_pending=true`表示退出时
尚未完成记录，文件可能晚更新；晚响应不作为已应用检测，以VLM_VIEW_RESULT为准。
写盘失败设置故障并沿用停止处理；pending或evidence_write_failed不作为证据完整通过。
文件保留在本轮独立证据目录，不自动覆盖/清理旧图，也不新增RGB订阅。
本次不改变四方向扫描顺序、检测判据或到点朝向；到点首方向已可见目标时，优先对照
第一个view的input.jpg和response.txt判断实际送入画面及模型判断，再针对事实排障。
`result.json`记录总体结果与停止确认，以及goal_costmap_topic/goal_costmap_valid；
逐事件记录全局目标检查来源，FOOTPRINT_BLOCKED的reason标明全局覆盖/障碍/未知格否决。PLAN_FAILED表示本次规划请求未成功，不等同于
永久不可达；规划超时/健康错误不记作NOT_DETECTED。
RViz的`/vlm_nav/go2_exploration/current_point`实时显示箭头和ID/状态/规划可达性，
0.5秒刷新，RELIABLE+TRANSIENT_LOCAL供运行期间后加入的RViz接收。
探索退出后新启动的订阅者不能依赖已退出发布者的缓存，历史以JSONL为准。
成功判据包括导航SUCCEEDED、完整目标扫描和stop_confirmed=true；EXHAUSTED不代表找到目标。
用户Ctrl+C后仍需确认取消终态/物理停止；STOP_UNCONFIRMED或残留运动锁时不继续启动，
不自动清理锁。日志写失败不打断取消、停止观察或ROS清理，报告evidence_write_failed；
证据不完整不记作通过。操作者在整轮实验中观察目标切换、路径更新、避障与停止；
出现具体问题后针对性分析日志，不预先逐项注入故障。实验结束并确认静止后终端1执行：

```bash
timeout 10 ros2 service call /twist_to_go2_sport_bridge/disarm std_srvs/srv/Trigger "{}"
```

预期success=true、DISARMED；若未停止先按现场急停流程处理，不用退出界面代替停车。
以上不生成bag，不修改验收标记、不部署或提交。

## 追加人工探索点

用户2026-10-09确认本次无共享资源并需标注多个点。保留scene.resources: {}，不填篮子；
独立入口仍检查定位、真实路径、costmap、footprint、相机、停止余量及动作终态。
完整协同live的篮子/READY/退出协议保持原要求。运行中的探索不热读配置，先完成采点
和列表审查，再启动下一轮；不通过clicked_point发导航目标。

以下优先使用已运行的真实Go2地图。终端2为RViz，终端4接收点击；都沿用前文的
Domain0/CycloneDDS/enp4s0环境，目录为仓库根目录，保持Sport转换DISARMED。
终端4先确认有地图数据；无输出时停止本路径，改用下方隔离标注步骤。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
timeout 10 ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
  --once --field info --qos-reliability reliable \
  --qos-durability transient_local --no-daemon
ros2 topic info /clicked_point -v --no-daemon
```

地图width/height须大于0，RViz Map状态OK、Fixed Frame=map；点击话题应有RViz发布者，
发现未知的运动转换订阅者时先查来源。终端4开始最多5分钟的点击接收，保持运行：

```bash
timeout 300 ros2 topic echo /clicked_point geometry_msgs/msg/PointStamped \
  --qos-reliability reliable --no-daemon
```

终端2先按上述配置确认costmap/车体/点云状态OK，再在RViz中每个探索点按以下顺序操作：
1. 选择Publish Point，点击Go2计划停驻的中心；避开墙、障碍和未知区域，留出整机间隙。
2. 再选择Publish Point，点击期望朝向上的另一点；两点不能重合，该点仅用于计算yaw。
3. 按相同顺序继续点2、点3等。配置Single click=true，每次点击后需重新选择工具。

每两个PointStamped为一组，header.frame_id必须为map；二维坐标忽略点击z，
yaw=atan2(y_direction-y_center,x_direction-x_center)。按顺序提供点击输出后，以唯一ID
后续未占用的go2_view编号追加到scene.viewpoints.go2；用户明确要求替换列表时按其顺序替换，
不把方向点单独加入探索列表，不生成未测坐标。列表顺序就是探索顺序，未发现则前往
下一人工点；VLM只识别目标，不负责生成这些点。

当前explore停车预检使用全局costmap，目标不要求在3×3米局部窗口内；
实际全局边界、障碍、未知格及完整footprint检查仍成立，最后还须ComputePath。
点位按真实可通行路线审查，不把全局检查可停车当成已到达或路线已验收。

### 已有地图未运行时的隔离标注

无需为采点启动相机、雷达、定位或Nav2。使用三个全新终端A/B/C，仅加载Humble，
每个都先执行同一环境块；不source go2_network_env.sh（会切回Domain0）。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_LOG_DIR="$PWD/build/go2-field/evidence/annotation-ros-logs"
mkdir -p "$ROS_LOG_DIR"
```

终端A启动保存地图并保持运行；若已有同名节点，不重复启动，先查来源。

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __ns:=/go2_point_annotation -r __node:=map_server \
  -p use_sim_time:=false \
  -p yaml_filename:="$PWD/maps/lab_rear120_20261001_IwQjMX/lab.yaml"
```

终端B依次配置、激活地图节点，前一步成功才继续；若已处于active，不重复转换。

```bash
ros2 lifecycle get /go2_point_annotation/map_server
ros2 lifecycle set /go2_point_annotation/map_server configure
ros2 lifecycle set /go2_point_annotation/map_server activate
timeout 10 ros2 topic echo /go2_point_annotation/map nav_msgs/msg/OccupancyGrid \
  --once --field info --qos-reliability reliable \
  --qos-durability transient_local --no-daemon
```

收到非零地图尺寸后，终端C启动RViz并保持运行：

```bash
rviz2 -d "$PWD/config/go2_exploration.rviz"
```

在RViz把Go2 map显示的Topic改为/go2_point_annotation/map，Fixed Frame保持map，
Publish Point仍用/clicked_point。终端B执行上面的300秒点击接收命令，然后按中心→方向
采点。只有保存地图坐标，没有真实定位/避障或可达性验收；采点完成关闭A/C。
实机探索仍使用Domain0环境，不能把隔离地图当成已启动导航的证据。

## 实机启动缺项核对（出现问题时使用）

本节保留此前排障方法，不是本轮单机实验逐项执行的前置清单；正常启动按前文
完整实验顺序进行，不要求另跑inspect、静止图像API或路径故障注入。

最新结果见[交接第9节](session_handoff.md#9-实机启动缺项核对与停车检查接口修复)。
本节针对已有运行栈；不ARM或发送目标。Ranger/对齐/B/C等完整live缺项不作为单机
explore的通过条件，实际静态门槛用go2_exploration.exploration_gates判断。

终端4沿用上节Domain0环境，在仓库根目录最后prepend源Python以使用停车接口修复；
检查进程是有限运行，底层/相机/Sport转换/RViz须继续运行且保持DISARMED。

```bash
export PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:${PYTHONPATH:-}"
ros2 run multi_agent_nav multi_agent_go2_check \
  --config-dir "$GO2_EXPLORE_CONFIG" --action inspect --require-ready --timeout 15 \
  --goal -0.5513970851898193 1.5923351049423218 -0.184007117515727
```

local_observation_ready=true不包含RGB-D/ARM/路径执行验收；还需rgbd_ready=true及真实运动
门槛。TypeError应消失并输出costmap_probe；footprint_free=false可能是障碍、未知或点超出
局部窗口，不直接认定规划不可达；本节inspect的probe仍为局部检查，
不能拿它作为远处探索目标的否决依据。explore改用全局停车检查。

终端4只读核对相机TF、活动costmap窗口/动态层和真实路径候选，每条完成后检查输出：

```bash
timeout 5 ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
timeout 5 ros2 run tf2_ros tf2_echo base_link camera_link
timeout 5 ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
ros2 param get /local_costmap/local_costmap width
ros2 param get /local_costmap/local_costmap height
ros2 param get /local_costmap/local_costmap resolution
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 topic list -t --no-daemon | rg 'nav_msgs/msg/Path'
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

tf2_echo有持续变换输出才算连通，timeout退出124本身不代表失败；最终仍需inspect核对
图像时间TF及RGB-D同步。参数服务超时或底层节点退出时先恢复前置输入，不ARM。
默认3×3米local窗口不覆盖任意全局目标，先核对活动参数，不猜测性放大窗口。
动态层存在/消息更新不替代实物marking/clearing；/plan是候选，须核实真实发布者、frame、
stamp、BT输入与执行/重规划关联，不能据话题存在设置path_source_verified=true。

### 执行路径来源核查（2026-10-09）

本次结果及授权见[交接第13节](session_handoff.md#13-执行路径来源核查与缓存有效期修复)。
第13节当时的Domain0有界核查未发现Nav2或Path发布者，保留了null/false。
后续用户设置AMCL初始位姿后，确认`/received_global_plan`为nav_msgs/msg/Path，
唯一发布者是`controller_server`，QoS为Reliable/Volatile、Keep Last(1)。
源配置现填写`path_topic: /received_global_plan`，`path_source_verified: false`仍保持；
这只确认订阅接口，不把话题存在或Domain31合成结果填成现场验收。

后续使用上文终端4的Domain0/CycloneDDS/enp4s0环境、仓库根目录；
本地雷达/IMU/FAST-LIO/定位/Nav2必须已由原终端启动并保持运行。
以下各条为只读检查，执行后退出，不ARM或发规划/导航/Spin/速度命令。
先发现Path话题；`grep`替代未安装的`rg`，无输出时先核实底层和ROS环境：

```bash
ros2 topic list -t --no-daemon | grep -F 'nav_msgs/msg/Path'
```

只有实际发现该话题后，才检查下列接口；存在命名空间时使用实查的完整话题/节点名：

```bash
ros2 topic info /received_global_plan --verbose --no-daemon
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.publish_global_plan
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

预期Path来自当前`controller_server`、插件为DWB、发布开关为true且QoS兼容。
参数超时、零发布者或插件不同则停止本接入流程；静止时没有新路径不证明接口故障。
空BT路径可能表示使用安装默认树，须核对实际树，不能解释为没有重规划。

本机Nav2 1.1.20的DWB在`setPlan()`及`prune_plan=true`时的控制循环中发布
`received_global_plan`；循环重发保留原计划header时间戳。
Domain31已验证初始/替换路径、取消CANCELED及正常退出，但这只是FollowPath源接口验证，
尚未验证真实NavigateToPose→BT→FollowPath目标关联与现场重规划。
2026-10-09本轮单机实验按用户决定改为独立接收断流监护：

- 删除计划header年龄拒收和由其触发的导航取消，原时间戳只用于日志和前一目标屏障。
- `path_receive_timeout: 5.0`仅用本机单调时钟计算距最后有效消息接收的间隔；
  读取缓存不续期，非法/其他任务消息不续期，不改写header。5秒是本轮实验初始值，
  没有安全推导依据，不保证5秒内物理停车。
- 仅自身已接受、未取消且未终止的NavigateToPose接收路径；保留map frame、非空、
  有限坐标以及早于本次导航请求的stamp拒收，取消/成功/失败/任务切换清缓存。
- 每条有效新接收消息更新路径，即使几何和stamp相同；几何没变不能据此判定没有重规划。
- 无有效路径保持/请求停止。`path_start_timeout: 1.0`只允许首路径等待时仍静止；
  一旦移动不享有等待。路径失效走真实取消、零速及动作终态+静止确认，不只设置WAIT。

`path_source_verified`仍false：本轮不伪造现场验收结论；单机navigate/explore不再以它
阻塞启动，完整协同live仍要求该标记及其他原有门槛。标准Path没有目标UUID，当前目标
关联由自有accepted handle、请求stamp屏障和独占目标owner支持，仍须整轮实验观察。
只读inspect不拥有导航，不把其他客户端路径当作自有有效路径。

本轮相关代码检查、局部测试及独立构建通过后即可进入一次完整单机真机实验；
不要求先补完整Domain31环境，不把逐项真机断流/错误frame/取消注入设为前置条件。
实验中记录当前目标、路径接收间隔、计划更新接入、取消原生终态和停止现象；
未触发/未观察事项保持待验证，出现问题再针对性排障，不关闭真实雷达/IMU制造故障。
保留定位、避障、控制门槛和急停要求；当前resources:{}，通道互斥尚未启用。

历史证据：早先局部87项及Domain31直接FollowPath接口检查保存于
`build/go2-field/evidence/go2-path-source.bsSuvM/`，不算完整BT或真机通过。
旧path_max_age=1.0同时限制header年龄及接收间隔；提交505e970/d5c9680/2575a54
使用的1秒是实现默认值，没有通道反应/制动预算推导。本轮已移除该参数及双年龄规则，
上文5秒接收规则取代旧新鲜度/首次来源验收的启动阻塞。

旧路径与通道边界：RegionGuard同时检查真实当前位置的外接圆+stop_margin包络和
执行路径包络。已配置区域、无许可且当前位置包络碰到区域时，即使缓存路径完全
绕开通道也返回STOP，AgentRunner随后取消自有导航并发停止命令；无效/过期路径
则在几何判断前返回STOP。旧路径可能漏掉提前的资源请求，当前位置检查是后备，
不能替代执行/重规划路径。它不是底层按许可过滤速度的独立联锁，依赖上层持续运行、
新鲜定位、正确区域/对齐、取消执行和实测停止余量，不能单凭逻辑保证物理车体不越界。
当前源scene.resources={}没有受控通道；仅go2_system的Nav2与健康supervisor不会
自动执行协同资源许可监护，外部RViz目标也不自动进入AgentMachine的自有任务监护。

相机现按用户明确授权沿用上一版外参并启用相应标记；只接相机TF或复检时，使用
[单源相机TF步骤](#go2-相机外参沿用与单源tf接入2026-10-09)。不再手动启动硬编码
static_transform_publisher；go2_system、go2_vlm和该独立入口复用同一所有权检查。
停止余量须实测；用户已确认本次无共享资源，使用明确的resources: {}即可，不填篮子。

终端4可先无API调用地检查当前终端是否具备视觉客户端配置；不打印密钥。
本检查通过仅表示客户端构造成功，尚须静止真实图像API验证才能确认目标识别可用。

```bash
/home/isee-pst/venv/co-nav-real/bin/python - <<'PY'
import os
from multi_agent_nav.config import load_config
from multi_agent_nav.adapters.go2 import ABCClient
try:
    config = load_config(os.environ['GO2_EXPLORE_CONFIG'])['robot_go2']
    client = ABCClient(dict(config, vision_mode='single'))
    client.client.close()
except Exception as error:
    print('Vision client configuration failed:', type(error).__name__)
    raise SystemExit(1)
print('PASS: vision client configured; API request not tested')
PY
```

## Build and offline validation

以下步骤仅供后续获相应任务授权时使用，本轮不执行。使用 Go2 控制电脑的终端 1，
每块命令完成并核对成功判据后再进入下一块，不需保持构建进程运行。
先加载环境并列出包，预期 vlm_nav、multi_agent_task_msgs、multi_agent_nav 各出现一次；
环境加载失败、缺包或重复时先排查，不继续构建。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_COOP_OUTPUT="${GO2_COOP_OUTPUT:-$PWD/build/go2-coop}"
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon list --base-paths "$PWD" "$PWD/cooperative_ws/src"
```

包列表正确后，在同一终端构建。成功判据为 colcon 退出 0 且所需包构建成功。

```bash
colcon --log-base "$GO2_COOP_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base "$GO2_COOP_OUTPUT/build" --install-base "$GO2_COOP_OUTPUT/install" \
  --packages-up-to multi_agent_nav \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

Only after the build exits 0, source that same output. Run the full suite only
when the change requires it; documentation changes need no build or ROS test:

```bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
```

仅当需要全量回归时，在同一终端单独执行：

```bash
bash scripts/test_cooperative_no_hardware.sh
```

The script defaults to `/tmp/go2-coop` if GO2_COOP_OUTPUT is unset; keep the
export above so build, tests and result files use the same directory. It records
baseline Python, baseline C++, cooperative tests and colcon results separately.
Success means the command exits 0 and the relevant groups report no failures.
On build/test failure stop that sequence and inspect its log before proceeding.

System Python3.10 generates/installs messages. Task executables and their ament
pytest use the existing CPython3.10 `co-nav-real` environment (httpx/vision SDK).
This avoids its setuptools/packaging build incompatibility without changing it.
The built executable shebang must point at the configured runtime interpreter.
Prepend repository PYTHONPATH without removing the ROS-generated entries.
Do not source the legacy `unitree_ros2/install` after the cooperative overlay:
its older Sport binary lacks the response subscription expected by the baseline.
Build, test, source and result all use GO2_COOP_OUTPUT. Existing logs/evidence
must be preserved; do not remove a directory to force a rebuild. No system upgrade is needed.

## Costmap 退出修复的独立构建与局部验证

用户于2026-10-08明确允许本任务必要的构建和ROS启动；本节限定本机Domain31、localhost、
合成map/TF/点云，无硬件、桥接、录包或运动。最新结果见
[退出排查补充](costmap_shutdown_handoff_20261008.md#新增授权后的隔离复现与修复)。
不执行生产costmap launch的完整入口，它会覆盖为Domain0；测试只复用其参数构造函数。

复现时使用Go2控制电脑终端1、仓库根目录。先加载已验证依赖并创建新的独立输出；
不覆盖现场安装。预期包列表中multi_agent_nav、multi_agent_task_msgs、vlm_nav各一处，
且旧依赖安装目录存在；缺失或重复时停止，不继续构建。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
export GO2_SHUTDOWN_OUTPUT="$(mktemp -d "$PWD/build/go2-costmap-shutdown.XXXXXX")"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon list --base-paths "$PWD" "$PWD/cooperative_ws/src"
```

同一终端仅构建multi_agent_nav；根包和消息依赖复用旧安装，因此colcon提示这两包未在
本输出构建是预期信息。成功判据为退出0、multi_agent_nav构建成功；失败先查本输出log。
本步骤不运行全量测试，不需保持构建进程运行。

```bash
colcon --log-base "$GO2_SHUTDOWN_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base "$GO2_SHUTDOWN_OUTPUT/build" --install-base "$GO2_SHUTDOWN_OUTPUT/install" \
  --packages-select multi_agent_nav --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
```

构建成功后，在同一终端最后加载新输出，设置隔离环境并检查Domain31图。
这是ROS参与者检查，预期无其他节点；有其他实例时先查来源，不并发运行合成costmap。
不加载go2_network_env.sh，不启动ROS daemon或桥接。

```bash
source "$GO2_SHUTDOWN_OUTPUT/install/local_setup.bash"
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$GO2_SHUTDOWN_OUTPUT/ros"
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
ros2 node list --no-daemon
```

图无冲突后，同一终端执行此文件的7项局部检查。成功判据为退出0、7 passed；
两项合成测试须local/global标记、清除成立，直接SIGINT及先SHUTDOWN/FINALIZED两种
退出方式的host/manager都为0，且未kill兜底。完成后测试进程结束，无需保持运行；
失败保留日志和JUnit，不重复使用同一basetemp覆盖证据。该结果不替代现场验收。

```bash
/usr/bin/python3 -m pytest \
  cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py -q \
  --basetemp="$GO2_SHUTDOWN_OUTPUT/pytest-tmp" \
  --junitxml="$GO2_SHUTDOWN_OUTPUT/junit.xml"
```

## Independent configuration inspection

After sourcing an existing installation, this command reads configuration and
map files only; it does not call rclpy.init or contact runtime ROS publishers:

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
ros2 run multi_agent_nav multi_agent_go2_check --offline \
  --config-dir "$PWD/cooperative_ws/src/multi_agent_nav/config"
```

Exit 0 without `--require-ready` means the report was produced, not that live
gates passed. `--require-ready` makes missing single-robot experiment configuration gates exit 1;
neither mode verifies live inputs, TF, positioning accuracy or physical stop.
Default `inspect` without `--offline` creates a ROS participant and RGB-D/other
subscriptions even though it does not send motion goals. Inspect its load/scope
before using it on hardware. Navigate/scan additionally require an explicit
motion action and `--allow-motion`; this option does not replace field authorization.
The no-hardware test suite does initialize isolated Domain31 ROS nodes: it is
not equivalent to an inspection that initializes no ROS at all.

预期报告 mode=configuration_only、motion_sent=false；缺项看 go2_local_gates 和
live_gates（完整协同），single_robot_experiment_gates（本轮单机）。
若需用退出码检查单机配置门槛，在同一终端另行执行：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready \
  --config-dir "$PWD/cooperative_ws/src/multi_agent_nav/config"
```

single_robot_experiment_gates非空时退出1；当前配置预期退出0，
path_source_verified仍false且live_gates仍列完整协同缺项。退出0只检查配置，
不证明真实输入、停止或识别已经通过，不为得到0改验收标记。

## Diagnostic full-cloud monitoring

`scripts/run_ab.py` defaults both raw and converted full-cloud monitors off,
retaining odometry/native LowState monitoring. `--full-cloud-monitoring` enables
only those additional monitors; it does not control essential converter, LIO,
obstacle or costmap subscriptions. Evidence and unresolved mechanism are in
[session_handoff.md](session_handoff.md). Keep the monitors off unless the task
explicitly needs the intrusive check within its authorized scope; prefer offline analysis.
`--self-check` initializes no ROS and starts no hardware. A hardware A/B run
requires session/reference/new-output arguments and records a bag in B; it is
not the unrecorded full static-chain entry.

## Go2 本地里程计与旧状态转发前置核对

本节是后续获现场核对授权后的操作顺序，本轮不执行。用户报告网页联调时疑有
两个订阅者、无 `/odometry` 发布者，尚无当时端点记录；先定位本地输入，不能先判
桥接或QoS故障。Domain0只走enp4s0，Domain40只走wlo1；实际地址运行前另核实。

既有本地输入顺序如下。条目列已有入口及参数来源，不要求重新启动已运行的进程；
无发布者或异常时停止推进，避免重复启动。各输入进程须保持运行至本地转发核对结束。

| 顺序 | 输入 → 输出 | 真实入口及参数来源 |
|---|---|---|
| 1 | 机身已有 `/lowstate` → `/body_imu` | `build/go2-coop/install/vlm_nav/lib/vlm_nav/lowstate_imu_adapter`；[源码](../../src/lowstate_imu_adapter.cpp)；没有LowState时先核对机身连接，不能用雷达IMU代替 |
| 2 | XT16 → `/lidar_points` | `/home/isee-pst/Documents/liang/hesai_xt16_ws/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node`；参数为历史轮次的 `driver-params.yaml` |
| 3 | `/lidar_points` → `/lidar_points_fastlio` | `/home/isee-pst/unitree_ros2/go2_ws/install/hesai_fastlio_converter/lib/hesai_fastlio_converter/hesai_fastlio_converter_node`；历史轮次 `converter-params.yaml` |
| 4 | 真实传感器外参 → 静态TF | `/opt/ros/humble/lib/tf2_ros/static_transform_publisher`；实测 `base_lidar_extrinsic`、`base_imu_extrinsic` 来自证据根的 `go2_calibration_xt16.yaml`，不能以Ranger/演示值替代 |
| 5 | 转换点云 + 机身IMU + TF → `/odometry` | `/home/isee-pst/unitree_ros2/go2_ws/install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping`；证据根 `spark_fast_lio_go2_xt16.yaml`，remap `lidar:=/lidar_points_fastlio`、`imu:=/body_imu` |

参数和历史启动命令逐项见
[run_static.py](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/run_static.py)
及其目录下的 [driver-params.yaml](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/driver-params.yaml)、
[converter-params.yaml](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/converter-params.yaml)。
该一次性脚本会启动完整静止链和costmap，已有runtime，不能直接重跑或作为网页前置入口。
长期可复用的静止入口仍待costmap退出闭环后整理；不替换为会录包/ARM的manualnav2/VLMNav脚本。

历史 [graph-running.json](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/graph-running.json)
中，driver点云、转换器输出、body_imu和lio_mapping的odometry发布者都是RELIABLE。
converter输入为BEST_EFFORT，LIO点云输入为RELIABLE、IMU输入为BEST_EFFORT。
这是历史实测；FAST-LIO源码的odometry可靠性使用SystemDefaults，当前端点须重新核实，
不能把默认值或历史图当成本次端点。

获本地核对授权后，在Go2控制电脑终端1、仓库根目录加载已有安装环境。
每条source成功才继续；网络脚本失败时先检查报出的网卡/rmem问题，不改域绕过。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_COOP_OUTPUT="$PWD/build/go2-coop"
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
export GO2_NET_IFACE=enp4s0
source scripts/go2_network_env.sh
```

先在同一终端查 `/odometry` 的真实发布者及订阅QoS。`--no-daemon`避免使用daemon缓存，
仍会创建ROS参与者，不属于文件离线检查。预期有实际lio_mapping发布者；0个时按上表
从LowState、IMU、原点云、转换点云和LIO进程逐段核实，仅查端点，不另加完整点云订阅。
若有多个非预期发布者先排查重复进程，不继续启动agent。

```bash
ros2 topic info /odometry -v --no-daemon
```

仅在确认发布者后，做5秒有界的小消息核对。以下使用BEST_EFFORT接收，证明消息存在；
还须将实际发布者QoS与旧agent所请求的RELIABLE/VOLATILE比较，不能只凭echo有数据
断言agent匹配。预期多条header时间戳推进，并核对新鲜度及frame；无输出或陈旧则先排障。
timeout到期通常返回124，按消息和退出原因判断，不当成程序正常退出0。

```bash
timeout --signal=INT --kill-after=2 5 ros2 topic echo /odometry nav_msgs/msg/Odometry \
  --qos-reliability best_effort --qos-durability volatile --field header --no-daemon
```

本地输入与QoS核对成立后，才在单独终端2启动旧agent并保持运行：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_NET_IFACE=enp4s0
bash scripts/start_go2_agent.sh
```

这个旧脚本只启动agent，且加载旧workspace/install；不要在已有协同/底层终端中source
它的旧overlay。静态确认旧安装的`multi-agent-comm.egg-link`指向
`/home/isee-pst/unitree_ros2/build/multi_agent_comm`，其中模块软链接解析到下述源码；
实际进程加载的模块及环境仍须另核实。源码位置为
`/home/isee-pst/unitree_ros2/multi_agent_comm/multi_agent_comm/go2_agent_node.py`。
源码请求 `/odometry` RELIABLE/VOLATILE depth1，并发布旧state BEST_EFFORT/VOLATILE depth1。
state沿用里程计stamp，frame_id固定为go2/odom，healthy固定为true、nav_state固定为IDLE；
这些字段不证明定位健康、READY或新任务状态。

终端1再依次查旧state端点、短时接收header，预期来源为go2_agent_node且时间戳随新odom推进。
失败先检查agent回调、安装版本及QoS；本地state成立后，才按跨机授权另查定向bridge和对端接收。

```bash
ros2 topic info /multi_agent/agents/go2/state -v --no-daemon
```

```bash
timeout --signal=INT --kill-after=2 5 ros2 topic echo /multi_agent/agents/go2/state \
  multi_agent_msgs/msg/AgentState --qos-reliability best_effort --qos-durability volatile \
  --field header --no-daemon
```

旧路由见 [config/multi_agent_bridge.yaml](../../config/multi_agent_bridge.yaml)：Go2旧state
本机0→40，Ranger旧state反向导入；bridge必须用
[按域绑定网卡的XML](../../config/cyclonedds_go2_multi_agent_bridge.xml)。
该旧YAML不显式固定QoS，本次未查运行端点或确认实际部署，不能推断桥接已经匹配。
本节不提供启桥、Ranger操作或新任务start命令；旧state联通也不开放完整协同live。

## Local dual-fake ROS example

终端 1 使用已加载的实际安装目录，保持 launch 运行直到终端 2 完成任务/暂停。
若选择了非默认 GO2_COOP_OUTPUT，两个终端须显式设置同一绝对路径。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_COOP_OUTPUT="${GO2_COOP_OUTPUT:-$PWD/build/go2-coop}"
source /opt/ros/humble/setup.bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
export ROS_LOG_DIR="$GO2_COOP_OUTPUT/ros-logs"
ros2 launch multi_agent_nav cooperative_mock.launch.py
```

The launch fixes Domain31, localhost only and a unique prefix
`/multi_agent/mock/run_<id>`, printed in node/service names and logs. It starts
three task nodes, no sensor/action clients, velocities, bridge or real adapters.
No motion before explicit start. In a second terminal use that actual prefix:

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_COOP_OUTPUT="${GO2_COOP_OUTPUT:-$PWD/build/go2-coop}"
source /opt/ros/humble/setup.bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
export ROS_LOG_DIR="$GO2_COOP_OUTPUT/ros-logs"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
: "${MOCK_PREFIX:?Set the actual prefix printed by the running mock launch}"
ros2 run multi_agent_nav multi_agent_start_task start --scenario synthetic --prefix "$MOCK_PREFIX"
```

终端 2 先将 MOCK_PREFIX 设为终端 1 日志的实际值。start 预期 accepted=true；
拒绝时核对原因，不盲目重复。需要暂停时在同一终端另行执行，预期 success=true：

```bash
ros2 run multi_agent_nav multi_agent_start_task pause --prefix "$MOCK_PREFIX"
```

The automated ROS smoke test discovers its own prefix, starts the task and
asserts DONE and both final states; it needs no manual commands or ROS daemon.

## Live preparation only — not executed this round

Keep the existing Domain0 sensor/localization/Nav2 stack and its validated
startup scripts. `bash scripts/manualnav2.sh localization_mode:=amcl map:="$GO2_MAP_YAML"` is
the existing manual stack; `VLMNav-go.sh` additionally starts the autonomous
single-target loop and must not run concurrently with cooperative task ownership.
Do not run either script as an offline test: they contain hardware probes/ARM.
GO2_MAP_YAML 须先设为已核实的真实地图 YAML 路径，不能照搬示例或猜测坐标。

The task launch below starts only coordinator + Go2 task agent, no Ranger,
fake peer, bridge, low-level stack or ARM. It defaults to IDLE; incomplete
configuration constructs no live adapter and StartTask rejects with reasons.

仅在已有现场授权范围及门槛满足时使用。终端 1 将 LIVE_CONFIG_DIR 设为已审查的
真实配置目录并保持 launch；缺项/启动失败时先排查，不进入 start。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_COOP_OUTPUT="${GO2_COOP_OUTPUT:-$PWD/build/go2-coop}"
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$GO2_COOP_OUTPUT/install/setup.bash"
export ROS_LOG_DIR="$GO2_COOP_OUTPUT/ros-logs"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_NET_IFACE=enp4s0
export CYCLONEDDS_URI="file://$GO2_COOP_OUTPUT/install/vlm_nav/share/vlm_nav/config/cyclonedds_go2.xml"
: "${LIVE_CONFIG_DIR:?Set the reviewed live configuration directory}"
ros2 launch multi_agent_nav go2_cooperative.launch.py config_dir:="$LIVE_CONFIG_DIR"
```

In a second terminal, enter the same repository directory and repeat the same
source/export commands above, using the same output directory. Field
authorization, open gates and both real peers are required before invoking:

```bash
ros2 run multi_agent_nav multi_agent_start_task start --scenario lab
```

start 预期 accepted=true；否则核对原因，不推进任务。需要暂停时，在同一终端
另行执行，核实 success=true、双方 action 终态及物理停止；状态不明不继续发 start。

```bash
ros2 run multi_agent_nav multi_agent_start_task pause
```

The launch fixes Domain0/CycloneDDS/enp4s0 using the existing XML; its DDS
configuration does not move any validated low-level process to Domain31.

Required live data/acceptance:

- Actual independent map identities/versions and map paths; lab_ref equals Go2
  map, Ranger transform from measured correspondences and an independent check.
  alignment.report must retain source/common/held-out points, thresholds and
  identities; startup recomputes the fit and checks the configured transform.
  map_digest binds each PC's YAML/image content; only local files are read on each PC.
- Measured scene viewpoints, wait/exit points, doorway and C protection polygons,
  and doorway transit exits for each robot. Regions must cover selected approach
  and exit paths. Live templates have no invented coordinates.
- Go2 configured fresh executing-path topic and verified association with actual
  NavigateToPose replans. A ComputePathToPose precheck or guessed `/plan` topic is
  insufficient. This interface is currently an explicit live gate.
- Measured stopping margin, verified camera geometry/depth units, ABC inference
  samples, full-body dynamic obstacle detection and actual motion-direction coverage.
  Configure path_source_verified/camera_calibrated/vision_validated/
  peer_obstacle_verified only with field evidence. Existing AMCL input is unchanged.
  costmap_source_verified additionally requires valid fresh full raw payload, TF and
  active dynamic layers. The old AMCL static-map merge removes dynamic layers;
  explicitly select the final cooperative costmap override documented in
  [dynamic_costmap_stage.md](dynamic_costmap_stage.md). Its existence or a qualitative
  stationary observation does not set field acceptance flags automatically.
- Sole task goal ownership, existing NAV_READY/CONTROL_ARMED gates, both real agents
  fresh/located/stopped, identical protocol and alignment version. No auto ARM.

`ros2 run multi_agent_nav multi_agent_calibration references.yaml` prints a candidate report only; it
does not publish TF, write active configuration or hot-swap alignment.

The first executing-path publication may arrive after NavigateToPose starts.
`robot_go2.path_start_timeout` (default 1 s) bounds the wait, allowed only while
fresh odometry confirms physical stop and the stationary stopping envelope is
clear/permitted. Movement before the first valid path, timeout, or any later
path loss cancels and pauses. Preflight paths never satisfy this monitor.

Pause invalidates pending motion results immediately; state must then confirm
action termination and physical stop. A C EXIT keeps the original deposit permit
identity/token; loss/failure cannot release it. New tasks reject held permits.
An uncertain occupied region requires field clearance and stopped peer reconciliation;
do not clear occupancy just because a process restarted or a timeout elapsed.
Once a new task is accepted, SYNC for every retired task is rejected; a delayed
old SYNC cannot reset command history and revive canceled execution.

For an authorized live shutdown, pause first and verify stopped agent states.
The existing `scripts/stopgo.sh` provides the established single-robot cancellation
and DISARM procedure; do not call it during mock tests. SIGINT cleanup attempts
cancellation/stopping; it is not proof of hardware stop. After stopping task nodes,
the unchanged single-robot stack is the rollback path.

## 2026-10-09 用户指定配置与停止余量

用户本轮明确要求源robot_go2.yaml的costmap_source_verified=true，已照此设置；
这是用户指定的验收配置状态，本轮没有新增动态costmap现场实验。
启动须显式使用仓库源config-dir，既有安装副本及运行中进程不会因此更新。
相机数值已为base_link→camera_link的xyz=[0.32715,-0.00003,0.04297]、rpy=[0,0,0]；
用户后续明确授权沿用上一版外参，三个对应标记已为true，依据和启动接入见下节；
这不是本轮重新实测标定或投影验收。不能把该零旋转直接连接到camera_depth_optical_frame。

stop_margin单位为米，是RegionGuard在整机外接圆之外追加的停止保护距离。
当前footprint外接圆半径约0.4205米；有共享区域时用0.4205+stop_margin检查当前位置
及执行路径。它不改变Nav2 inflation、目标容差、物体standoff或物理制动性能。
resources:{}时没有资源边界计算，但入口仍要求实测、有限且正的stop_margin。

建议按“触发停止至确实停稳的最大继续位移+定位/对齐误差+预留”定值。
估算可用m≥v×T+v²/(2a)+e，其中T为端到端反应延迟，a为实测有效减速度，
e为误差及预留；原理见[Nav2官方停止距离说明](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)。
当前源速度上限0.40m/s、软件减速度0.50m/s²；这些不是已实测运动响应。
若暂以T=1.5s、a=0.50m/s²、e=0.10m作明确假设，估算0.86m，可将1.0m作为
待验证候选。这些假设尚无现场上界证据，1.0m不是已验收值或安全保证。
用户后续明确要求暂设1.0m，源stop_margin已设为1.0；记录为用户指定临时值，未实测。
尤其不能仅凭软件减速度或超时参数宣称实机停止距离已知。

## Go2 相机外参沿用与单源TF接入（2026-10-09）

依据：**用户确认沿用上一版 Go2 相机外参**。旧go2_calibration.yaml已有calibrated=true，
旧manualnav2.sh另发同一安装TF，旧go2_vlm覆盖视觉参数为true；本轮是明确沿用，
不记录为重新完成实测标定或投影验收。当前三个生效标记：
config/go2_calibration_xt16.yaml的camera_extrinsic.calibrated=true；
config/robot_go2.yaml的vlm_nav.camera_extrinsic_calibrated=true；
cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml的camera_calibrated=true。
协同make_observer先读实际vlm_nav share的robot_go2.yaml，再用显式协同config-dir中的
camera_calibrated覆盖camera_extrinsic_calibrated；已用真实只读观察节点验证最终true。

外参由所选calibration_file集中提供：base_link→camera_link，
xyz=[0.32715,-0.00003,0.04297]、rpy=[0,0,0]。
go2_system、go2_vlm及独立go2_camera_tf.launch.py现均默认XT16标定文件；命令行
calibration_file覆盖该默认值。manualnav2显式给两条launch传同一个配置，移除硬编码TF。
go2_vlm仍以最后的节点参数字典覆盖camera_extrinsic_calibrated=true，enabled默认false。
新独立root overlay必须最后source；既有安装未覆盖，源Python优先不能替代旧launch和
可执行文件的更新。前述环境块已增加新root overlay，传感器预检子脚本还应设置：

```bash
export VLM_NAV_EXTRA_SETUP_FILES="$PWD/build/go2-coop/install/local_setup.bash:$GO2_EXPLORE_OUTPUT/install/local_setup.bash:$GO2_CAMERA_OUTPUT/install/local_setup.bash"
```

新增go2_camera_tf节点只订阅/tf和/tf_static。按实际publisher GID检查camera_link的来源、
父frame和数值，正确已有来源复用；缺失时发布配置值。同机同Domain的多个入口用
进程锁保证只有一个自有发布者；有多个既有来源或不一致时退出1，集成launch关闭。
manualnav2在common.sh重载环境后检查当前包的相机所有权程序，旧overlay缺失时在
网络/预检/录包/硬件启动前退出；纠正VLM_NAV_EXTRA_SETUP_FILES后才能继续原流程。
不改驱动camera_link→RGB/depth optical TF。外部未更新的手工发布者若后起，无法在其
发送前阻止短暂重叠；节点发现后撤回自有来源或报错。正式运行不要另启手工安装TF。
投影使用真实图像header：raw_depth_mode=true取depth光学frame的图像时间map变换，
再用驱动depth→color变换计算color投影；camera_frame仍是camera_color_optical_frame。

本轮独立构建位于build/go2-camera-tf/install，复现构建用新终端、仓库根目录：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
colcon --log-base "$PWD/build/go2-camera-tf/colcon-log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" --packages-select vlm_nav \
  --allow-overriding vlm_nav --build-base "$PWD/build/go2-camera-tf/colcon-build" \
  --install-base "$PWD/build/go2-camera-tf/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
```

仅核对相机TF时，终端A、B都先执行下列环境块。相机驱动保持运行，无需起立、ARM、
启动雷达/FAST-LIO/Nav2或目标寻找；本步骤不启动自主运动。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-camera-tf/install/local_setup.bash"
export GO2_NET_IFACE=enp4s0
source "$PWD/scripts/go2_network_env.sh"
export ROS_LOG_DIR="$PWD/build/go2-camera-tf/ros-field-log"
ros2 pkg prefix --share vlm_nav
```

share必须指向build/go2-camera-tf/install/vlm_nav/share/vlm_nav，否则先纠正overlay顺序。
终端A启动仅相机TF入口并保持运行；底层已运行时该入口应复用已有来源：

```bash
ros2 launch vlm_nav go2_camera_tf.launch.py \
  calibration_file:="$PWD/config/go2_calibration_xt16.yaml"
```

预期Publishing configured ... single local owner或Reusing existing ...；出现duplicate/
inconsistent报错先处理已有来源，不能继续。终端B按顺序逐条检查：

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_link camera_link
timeout 5 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
timeout 5 ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 topic info /tf_static --verbose --no-daemon
```

第一条平移应为给定值、旋转为单位四元数；后两条保留驱动optical旋转，不能要求零旋转。
timeout的124不代表TF失败，以期间变换输出为准；失败时停止后续导航步骤。
/tf_static还有相机内部/其他安装TF发布者，不能以总publisher数认定安装边重复；
专用节点按camera_link和GID核查，终端A不应有冲突错误。若底层定位已运行，再检查：

```bash
timeout 5 ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

没有map/odom时完整地图链不成立，不为本相机检查补启底层。检查完成Ctrl+C关闭A；
正式go2_system/独立TF入口须保持一个活动所有者。TF退出不代表机器人停止。

本轮局部回归复现命令（新终端、仓库根目录；仅Domain31隔离ROS，无真机输入）：

```bash
source /opt/ros/humble/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-camera-tf/install/local_setup.bash"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$PWD/build/go2-camera-tf/ros-log"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/cooperative_ws/src/multi_agent_nav:${PYTHONPATH:-}"
export GO2_CAMERA_TF_EXECUTABLE="$PWD/build/go2-camera-tf/install/vlm_nav/lib/vlm_nav/go2_camera_tf"
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  test/test_go2_camera_tf.py test/test_go2_profile.py test/test_go2_localization.py \
  cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py -q --tb=short \
  --junitxml=build/go2-camera-tf/final.xml
```

## 2026-10-11：家具提示搜索并接近灭火器

用户已确认新观察点且授权本轮完整寻找/接近，现场有人急停；底层尚需冷启动。
先关闭Domain31标注流程，按现有冷启动顺序完成相机、预检、底层、Domain0 RViz实际初始
位姿和唯一Sport转换。只读检查通过且机器人站稳后才ARM；失败停在对应步骤排障。
四终端公共环境仍为source CoVLM_env.bash。新目录的table_01_view_01/table_02_view_01已启用，
旧固定策略点未修改；这次必须显式选择semantic_furniture。

终端4，仓库目录，先完成配置和被动现场检查（自动退出，不保持运行）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --strategy semantic_furniture
```

配置退出0后，等待底层Nav2就绪，执行被动检查。预期active_map_matches、localization_valid、
health.stopped、costmap_valid、rgbd_ready均为true；DISARMED时nav_ready可false，不因此提前ARM。

```bash
ros2 run multi_agent_nav multi_agent_go2_check --action inspect --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --timeout 10
```

确认相机/定位/costmap有效后，按前文读取Sport状态；DISARMED才调用arm，success=true/ARMED
才继续。FAULT按原因处理，不能靠重新启动探索解锁。A/B/C保持运行，终端4执行一次最长300秒：

```bash
export GO2_EXPLORE_EVIDENCE="$REPO/build/go2-field/explore.$(date +%Y%m%d_%H%M%S)"
ros2 run multi_agent_nav multi_agent_go2_check \
  --config-dir "$GO2_EXPLORE_CONFIG" --action explore --allow-motion \
  --strategy semantic_furniture --target 灭火器 \
  --search-hint '灭火器在 table_02 附近' --approach-target \
  --timeout 300 --output-dir "$GO2_EXPLORE_EVIDENCE"
```

不要预创建输出目录。模型自己在可达候选中选择家具；提示不是硬编码移动指令。
事件应依次出现FURNITURE_SELECTED、VIEWPOINT_NAV、SCANNING、有效TARGET_FOUND，随后
APPROACH_PLANNING/APPROACH_SELECTED、PICKUP_NAV、TARGET_APPROACHED。接近点由深度/TF目标
位置及既有0.9米standoff规划计算，不使用家具中心或模型坐标。
成功须result=TARGET_APPROACHED且stop_confirmed=true；未定位、API错误、候选不可达或接近失败
都不算完成。原vlm/保留实际相机帧和结果，选择事件另记录search_hint。
Ctrl+C会进入原取消和停止确认。正常结束也先确认实际停止，再按既有流程DISARM；
停止未确认先现场急停，不开始下一轮。不得把本地接近成功当双机READY/C验收。

### 更新：复用原 VLM_Nav 的到达距离闭环

2026-10-11 后续源码已复用原配置 `config/robot.yaml` 再覆盖 `config/robot_go2.yaml` 的
arrival contract：首次0.89m请求停止，停稳后距离≤0.81m才成功，最多重试1次，重试停止半径0.81m。
参数通过任务robot_go2.yaml的arrival_config_files关联，不另复制阈值。单机原VLMNavigator与
Go2探索共用vlm_nav.arrival数学判据，停止终态与静止确认仍使用各自已有安全逻辑。
当前0.9m候选上限会被成功半径约束；首次未达标时按实际停止偏差缩小候选半径重规划，
新候选仍需完整车体/局部和全局costmap/路径/资源检查，不可达或重试后超距返回失败。
不得仅凭Nav2 SUCCEEDED或计划目标距离判成功。位置仍来源于定位与目标深度投影，非实尺保证。

本轮仅离线实现和隔离构建，未追加运动，也未切换CoVLM_env.bash默认overlay。
新安装为build/go2-arrival-contract-20261011/install，同时包含vlm_nav和multi_agent_nav；
原外部overlay未覆盖。后续获准实验前，在执行终端先加载公共环境，再最后加载此新安装：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
source "$REPO/build/go2-arrival-contract-20261011/install/local_setup.bash"
export VLM_NAV_EXTRA_SETUP_FILES="$REPO/build/go2-arrival-contract-20261011/install/local_setup.bash"
```

之后不再source旧overlay。原探索命令的--approach-target接口不变。
新结果事件包含ARRIVAL_CONTRACT、APPROACH_DISTANCE_STOP、ARRIVAL_VERIFIED及实测估计距离；
未满足距离要求为APPROACH_DISTANCE_FAILED，进程非零。上一轮1.13m按此新判据不能算成功。

需要重建本版本时，复用已安装依赖，使用独立输出：

```bash
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-camera-tf/install/local_setup.bash"
export GO2_ARRIVAL_OUTPUT="$PWD/build/go2-arrival-contract-20261011"
colcon --log-base "$GO2_ARRIVAL_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --packages-select vlm_nav multi_agent_nav \
  --build-base "$GO2_ARRIVAL_OUTPUT/build" --install-base "$GO2_ARRIVAL_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

成功须两包完成且退出0，然后从同一输出加载验证；构建本身不启动机器人。

### 强度 A/B 隔离构建（2026-10-12）

设计与现场门槛见 [intensity_ab_20261012.md](intensity_ab_20261012.md)。仓库根目录，终端仅构建，不启动ROS节点：

```bash
source CoVLM_env.bash
colcon --log-base build/go2-intensity-ab-20261012/log build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" --packages-select vlm_nav \
  --allow-overriding vlm_nav --executor sequential \
  --build-base build/go2-intensity-ab-20261012/build \
  --install-base build/go2-intensity-ab-20261012/install \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
```

成功退出后最后source该输出 `install/local_setup.bash`。本轮只改根包，不重建协同包；保留两处搜索路径。单元检查运行 `build/go2-intensity-ab-20261012/build/vlm_nav/test_obstacle_cloud_filter`。既有已验证overlay不覆盖，不更改CoVLM默认路径。

### Go2默认启用≤3强度过滤（2026-10-12用户指定）

`config/robot_go2.yaml`及`go2_system.launch.py`默认启用体素后删除intensity≤3。
Ranger与裸节点默认不变。`CoVLM_env.bash`新增GO2_FILTER_OUTPUT并最后加载
`build/go2-intensity-default-20261012/install/local_setup.bash`，额外setup链同样最后加载它。
重新打开/使用终端先source CoVLM_env.bash，再沿用原Go2底层命令；仅source不会改变正在运行的节点。
已启动底层需正常退出后再启动，不能叠加第二个过滤节点。下一次启动预期参数
intensity_filter_enabled=true、intensity_threshold=3.0，障碍输出仍只有一个发布者。
显式回退在go2_system原启动命令追加 `intensity_filter_enabled:=false`。
该参数是启动只读参数，不能用ros2 param set热切换。过滤同时影响/scan及其AMCL消费者。

新默认输出的构建命令（仓库根目录，不启动节点）：

```bash
source CoVLM_env.bash
colcon --log-base "$GO2_FILTER_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" --packages-select vlm_nav \
  --allow-overriding vlm_nav --executor sequential \
  --build-base "$GO2_FILTER_OUTPUT/build" --install-base "$GO2_FILTER_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
```

新默认在独立目录完成构建，没有覆盖之前的实验/现场overlay。本次只验证构建、6项过滤单测、31项Go2配置/定位测试及安装路径/默认值，不重启真机。既有全场景低反射障碍验收限制仍有效。


## 2026-10-13：生成候选双图VLM选点

策略及证据语义见 [go2_semantic_search.md](go2_semantic_search.md#2026-10-13generated_candidates_vlm)。
本轮只构建及离线/fake验证，以下真机命令未执行，不授权新的运动。旧固定点策略保持默认。
用户本轮指定 `scene.yaml` 的 B=水杯、C=篮子，A沿用灭火器；只填写描述，没有新增目标坐标。
若未来改回占位描述，新策略离线require-ready退出1，报告
`generated scans require measured A/B/C target descriptions`。描述不代表现场已检测到目标。

### 隔离构建

新 Bash 构建终端，仓库根目录；自动退出，不启动 ROS、桥接、API 或运动。
复用已安装依赖；不要省略 go2_ws 环境，否则根包可能不构建 Sport/IMU 可执行文件。
不原地覆盖旧已验证 overlay；两处源码搜索路径保留。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-camera-tf/install/local_setup.bash"
export REPO="$PWD"
export GO2_GENERATED_OUTPUT="$REPO/build/go2-generated-candidates-20261013"
colcon --log-base "$GO2_GENERATED_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --packages-select vlm_nav multi_agent_nav \
  --build-base "$GO2_GENERATED_OUTPUT/build" --install-base "$GO2_GENERATED_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

成功判据：退出0、两包完成，安装中含原 Sport bridge、IMU adapter 和新增 Python 模块。
失败不切换运行环境，先处理构建日志。本轮 root 包一同构建以带入已存在的 arrival 共享实现、
相机所有权与用户指定过滤默认；没有修改这些底层参数。PCL开发警告及复用既有接口包提示不等于失败。

### 相关离线/fake测试

仍在构建终端，上一构建成功后才执行；测试自动退出，Domain31/localhost隔离。
新 overlay 最后 source，使用同一个输出；切到 /tmp 避免仓库同名包遮蔽安装模块，不运行无关全量基线。

```bash
source "$GO2_GENERATED_OUTPUT/install/local_setup.bash"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_LOG_DIR="$GO2_GENERATED_OUTPUT/ros-logs"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd /tmp
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_generated_candidates.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_semantic_search.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_exploration.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py" \
  -q --tb=short --junitxml="$GO2_GENERATED_OUTPUT/related.xml"
```

成功判据：无失败；验证双图标记位置/颜色一致、静态与动态整盘筛选、全局远点检查、12点上限、
ID解析/非法/取消/迟到拒绝、fake选点→观察→下一点、API失败不算完成和原目标接近。
测试失败先修复，不进入现场流程。API调用用注入的离线响应，不访问真实模型。

### 后续现场启动（需新的明确运动授权）

新 Bash 终端1–4，各在仓库根目录加载下面环境。不要沿用刚才测试的Domain31环境。
CoVLM_env 默认输出不改；每个真机终端显式最后加载新 overlay，子脚本也必须最后加载它。
这组命令只加载环境，不启动节点，执行后核对模块路径。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source CoVLM_env.bash
export GO2_GENERATED_OUTPUT="$REPO/build/go2-generated-candidates-20261013"
source "$GO2_GENERATED_OUTPUT/install/local_setup.bash"
export GO2_EXPLORE_OUTPUT="$GO2_GENERATED_OUTPUT"
export VLM_NAV_EXTRA_SETUP_FILES="$GO2_CAMERA_OUTPUT/install/local_setup.bash:$GO2_FILTER_OUTPUT/install/local_setup.bash:$GO2_GENERATED_OUTPUT/install/local_setup.bash"
export ROS_LOG_DIR="$GO2_GENERATED_OUTPUT/ros-logs"
(cd /tmp && /home/isee-pst/venv/co-nav-real/bin/python -c 'import multi_agent_nav.generated_candidates as g; import vlm_nav.arrival as a; print(g.__file__); print(a.__file__)')
```

预期两个模块均来自 `build/go2-generated-candidates-20261013/install/`；不是旧观察或arrival安装。
不匹配时停止，修正source顺序。不要随后再source旧overlay。ROS_DOMAIN_ID应为0，本地域仍走enp4s0。
终端1–3复用原相机、预检/底层Nav2、RViz和唯一Sport桥入口与初始位姿步骤，保持运行；
有已运行输入时复用并核实唯一真实发布者、新鲜RGB-D/odom/map/TF/两costmap与QoS，不重复启动。
相机外参、单位报告、API环境变量沿用原要求，不打印密钥。

终端4先检查配置（自动退出，不初始化ROS）：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --strategy generated_candidates_vlm
```

退出0、`single_robot_experiment_gates=[]` 后才继续。该报告中位姿仅为本地配置诊断，
不发送给VLM；静态候选数不表示实时可达。若B/C仍占位则退出1，在scene.yaml填写真实描述后重查。
随后被动现场检查（自动退出）：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --action inspect --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --strategy generated_candidates_vlm --timeout 10
```

预期active_map_matches、localization_valid、health.stopped、costmap_valid、rgbd_ready为true；
DISARMED时nav_ready可false。失败先排障，不开始探索。确认输入/定位及人员急停前置后，
沿用前文Sport状态检查/ARM顺序；仅明确授权且arm响应success=true/ARMED时运行一次探索。
不将本小节当作现场运动授权。下面最长300秒，保持终端4运行至正常退出或取消：

```bash
export GO2_EXPLORE_EVIDENCE="$REPO/build/go2-field/generated-explore.$(date +%Y%m%d_%H%M%S)"
ros2 run multi_agent_nav multi_agent_go2_check \
  --config-dir "$GO2_EXPLORE_CONFIG" --action explore --allow-motion \
  --strategy generated_candidates_vlm --target 灭火器 --approach-target \
  --timeout 300 --output-dir "$GO2_EXPLORE_EVIDENCE"
```

不要预创建证据目录。事件应为CANDIDATE_REQUEST/SELECTED→VIEWPOINT_NAV→ARRIVED/停止确认→
SCANNING→有效ABC→OBSERVATION_RECORD→下一决策；检测A后进入既有PICKUP_NAV/距离闭环。
完整目标接近成功须TARGET_APPROACHED且stop_confirmed=true；仅到点不算观察完成，EXHAUSTED只表示
当前视角完成，WAITING_VIEWPOINTS表示候选暂不可达或观察失败，不是目标不存在。
Ctrl+C沿用取消/停止；结束先确认停止，再按前文DISARM，最后关闭本轮界面/底层。
停止未确认则现场急停，不启动下一轮。无新的bridge/coordinator或诊断订阅步骤。


## 2026-10-13：RViz生成点、选点、RGB与10次实验记录

本节更新上一节的环境/记录入口；CoVLM_env.bash已默认最后加载
`build/go2-exploration-display-20261013/install`（协同包），其vlm_nav依赖来自
`build/go2-generated-candidates-20261013/install`。不要再按上一节最后source旧generated
协同安装，否则会覆盖此次标记和记录逻辑。source不自动SSH、启动ROS或运动。

### 相机与RViz

Go2控制电脑，新Bash终端1，目录 `/home/isee-pst/unitree_ros2/VLM_Nav`。
先加载Domain0/enp4s0环境，再检查已有相机真实发布者，不重复开启驱动：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 topic info /camera/camera/color/image_raw --verbose
ros2 topic info /camera/camera/depth/image_rect_raw --verbose
```

存在唯一发布者只是前提，不证明图像新鲜；配合下方RViz持续刷新及原RGB-D输入门槛确认。
已有正常相机则复用并跳过启动。没有相机且确认旧驱动/USB已释放后，在终端1启动专用SSH入口：

```bash
bash "$REPO/scripts/go2_camera_ssh.sh"
```

该脚本仅连接 `unitree@192.168.123.18`，远端加载
`/home/unitree/unitree_ros2/setup.sh`、`/home/unitree/ros2_ws/install/setup.bash`，执行：

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_sync:=true \
  align_depth.enable:=false
```

终端1保持运行；预期 `RealSense Node Is Up`。SSH失败、USB占用、重复发布者或原RGB-D门槛
不通过则停止后续步骤，按[原相机指引](../VLM_Nav初步启动指引.md#1-go-终端启动-d435-raw-rgb-d)排障，
不批量杀进程或盲目重复启动。此脚本不执行相机probe，也不在控制电脑启动本地USB相机。

Go2控制电脑终端2（Bash），以下两行就是用户指定的RViz入口，保持运行：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
rviz2 -d "$REPO/config/go2_exploration.rviz"
```

预期 Go2 live RGB 状态OK、画面连续刷新。直接订阅现有
`/camera/camera/color/image_raw`（raw、best_effort、volatile、depth1），不增加转发/诊断节点。
地图仍需已有map/定位/Nav2输入，沿用原底层前置和初始位姿步骤；RViz不负责启动它们。
Generated exploration candidates 显示程序生成点：灰色待检查/已观察/暂不可用，橙色本轮
已筛选可选点和P编号；VLM selected observation point 以绿色箭头标示最终选中观察点。
每轮请求清除上一轮选择；下一实验启动清除旧标记。接近目标时绿色箭头仍表示选中的
观察点，不移动到目标接近位姿。旧 `/vlm_nav/go2_exploration/current_point` 不在此配置中显示。
固定点模式继续可执行，但不会在这两个新显示项中冒充自动生成点。
候选只在新策略程序运行、筛选/选点事件发生后出现；打开RViz本身不计算点或调用模型。
若新项无数据，先核对启动策略和实际multi_agent_nav安装来源，不为显示点启动未授权运动。

### 统一实验目录及滚动清理

新策略省略 `--output-dir` 时，自动建立
`$REPO/build/go2-vlm-experiments/experiment.<时间>.<唯一ID>/`，完整记录一轮实验：

- `events.jsonl`、`result.json`：任务/决策、内部与展示映射、执行/停止结果。
- `selection/<decision_id>/`：发送双图、`request.json`（实际messages、模型、tokens、请求schema）、
  `response.txt`（返回原文，已脱敏）和`metadata.json`（解析结果/错误）。
- `vlm/<uuid>_view_<方向>/`：实际上传`input.jpg`、含提示词和图像内容的`request.json`、
  `response.txt`、`result.png`及元数据。API无响应则记录错误类型，不能伪造返回或未检出。

记录不包含HTTP认证密钥。目录是实验资料包，不新增ROS bag录制。
根目录最多保留10个由此流程标记的实验；新包建立时删除最早的非活跃包。
活跃实验和仍在写盘的迟到API worker持文件锁，清理不会删除；若10个包都活跃，则拒绝新建，
不越过数量上限。只清理有管理标记的直接子目录，跳过符号链接；旧build/go2-field日志/bag
不进入清理范围。`--output-dir`仍保留兼容用途：根目录直接子目录参与清理，根目录外明确
指定的输出不受自动删除管理；要使用10次限制，采用本节默认命令，不沿用旧外部output-dir。
写盘失败会中断请求/执行，不通过丢失证据继续导航；超时/非法响应仍不触发运动。

终端4先做不初始化ROS的配置检查（自动退出）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 run multi_agent_nav multi_agent_go2_check --offline --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --strategy generated_candidates_vlm
```

退出0、`single_robot_experiment_gates=[]` 才进入原现场输入检查、停止确认及Sport ARM顺序。
未来获得明确运动授权、现有底层/相机/RViz正常且ARMED后，终端4用默认统一目录运行一次：

```bash
ros2 run multi_agent_nav multi_agent_go2_check \
  --config-dir "$GO2_EXPLORE_CONFIG" --action explore --allow-motion \
  --strategy generated_candidates_vlm --target 灭火器 --approach-target --timeout 300
```

保持至正常退出或Ctrl+C停止；事件、图片和结果位于最新experiment目录，不预创建包目录。
完整接近成功仍须TARGET_APPROACHED和stop_confirmed=true；结束先确认停稳再DISARM。
图像可见/选点可见不代替导航或安全验收。以上SSH/RViz/运动命令本轮未实际运行。

### 本轮隔离构建与相关验证

构建Bash终端，仓库目录；不启动硬件，自动退出：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-generated-candidates-20261013/install/local_setup.bash"
export GO2_DISPLAY_OUTPUT="$PWD/build/go2-exploration-display-20261013"
colcon --log-base "$GO2_DISPLAY_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" --packages-select multi_agent_nav \
  --build-base "$GO2_DISPLAY_OUTPUT/build" --install-base "$GO2_DISPLAY_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

成功判据：退出0、multi_agent_nav完成；vlm_nav及消息包复用此前安装，不重建底层包。
上一命令成功后加载同一输出，在/tmp验证安装模块，Domain31/localhost隔离：

```bash
export REPO="$PWD"
source "$GO2_DISPLAY_OUTPUT/install/local_setup.bash"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_LOG_DIR="$GO2_DISPLAY_OUTPUT/ros-logs" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd /tmp
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_generated_candidates.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_semantic_search.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_exploration.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_scan_evidence.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_exploration_display_records.py" \
  -q --tb=short --junitxml="$GO2_DISPLAY_OUTPUT/related.xml"
```

测试退出0且206项通过；新增本轮显示/记录11项，既有相关195项。没有真实API或SSH连接。
环境测试单独在仓库目录执行，mock网络探测但真实加载overlay：

```bash
cd "$REPO"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  test/test_covlm_env.py -q --tb=short --junitxml="$GO2_DISPLAY_OUTPUT/environment.xml"
```

预期3项通过；总209项相关检查，不是全量基线。失败停止推进，先处理报告。

## 2026-10-13：健康分项与ABC返回协议修复

最新安装输出为 `build/go2-health-abc-20261013`，覆盖两个受影响包；`CoVLM_env.bash`
已最后加载此输出。上述display输出保留为历史记录，当前不要在新环境后source旧输出。
ABC提示统一要求A/B/C各含status+grounding，保留严格解析；不补用旧非法返回。
失败时 `events.jsonl` 的 `HEALTH_FAILED.failed_gates` 和 `health_details` 保存实际失效项。
不调整传感器、定位、Nav2或任何安全时限；不改协同协议。

### 构建和离线回归

构建终端（Bash），仓库目录，自动退出，不启动现场输入：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source "$PWD/build/go2-coop/install/local_setup.bash"
source "$PWD/build/go2-generated-candidates-20261013/install/local_setup.bash"
export GO2_HEALTH_OUTPUT="$PWD/build/go2-health-abc-20261013"
colcon --log-base "$GO2_HEALTH_OUTPUT/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" --packages-select vlm_nav multi_agent_nav \
  --build-base "$GO2_HEALTH_OUTPUT/build" --install-base "$GO2_HEALTH_OUTPUT/install" \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
```

预期退出0、两个包完成；消息包复用go2-coop安装。构建失败先处理，不覆盖旧验证输出。
仅构建成功后，同终端加载本次安装，在/tmp测试实际安装代码，自动退出：

```bash
export REPO="$PWD"
source "$GO2_HEALTH_OUTPUT/install/local_setup.bash"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_LOG_DIR="$GO2_HEALTH_OUTPUT/ros-logs" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd /tmp
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_generated_candidates.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_semantic_search.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_exploration.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_scan_evidence.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_exploration_display_records.py" \
  "$REPO/cooperative_ws/src/multi_agent_nav/tests/test_go2_health_diagnostics.py" \
  -q --tb=short --junitxml="$GO2_HEALTH_OUTPUT/related.xml"
```

预期225项通过；成功后，同终端回仓库验证客户端及入口环境，自动退出：

```bash
cd "$REPO"
/home/isee-pst/venv/co-nav-real/bin/python -m pytest \
  test/test_vlm_client.py test/test_covlm_env.py -q --tb=short \
  --junitxml="$GO2_HEALTH_OUTPUT/client-environment.xml"
```

预期35项通过；合计260项，新增诊断/入口19项、既有相关241项，没有重跑无关全量基线。
单目标提示的6组历史字节比对另已通过；真实模型是否遵循新ABC格式仍待现场验证。

### 重启后先只读检查

终端4，Bash；前置：本地指引的相机/底层/定位/Nav2保持运行、实测初始位姿已设置。
本轮检查时用户已停止这些输入，退出1不能诊断20:12的瞬时失效。先按原指引恢复输入；
本节命令不启动底层、不ARM、不发运动、不请求API，结束自动退出：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 pkg prefix multi_agent_nav
```

预期 `$REPO/build/go2-health-abc-20261013/install/multi_agent_nav`；不匹配先处理环境。
确认安装来源后执行一次：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --action inspect --require-ready \
  --config-dir "$GO2_EXPLORE_CONFIG" --strategy generated_candidates_vlm --timeout 10
```

必须退出0且 `active_map_matches`、`health.localization_valid`、`health.stopped`、
`costmap_valid`、`rgbd_ready`、`health_window.non_motion_inputs_stable` 均true，再继续原ARM步骤。
稳定摘要要求定位/局部costmap/RGB-D/深度单位/odom首次全部就绪后观察至少2秒且无失效，
记录采样数/时长。ARM、Nav2服务等仍在完整诊断中，摘要不认证运动就绪。
DISARMED时 `control_armed_gate` / `health.nav_ready` 为false可以预期；不要为只读检查ARM。
输入不稳定或退出1则停止推进，依据 `health_window.transitions` / `health_details` 排障。
后续探索失败取 `HEALTH_FAILED.failed_gates`，不因通用退出1盲目重跑。
