# Ranger PC handoff (no Ranger connection performed)

> 当前公共到达修复见文末2026-10-22节；独立C/取消保护区规则保留，此前资源许可与顺序退出记录属于历史版本。


当前私有 C 模式以本文末尾 **2026-10-20 本轮最小接入说明** 为准；前文保留为旧共享模式参考。

Transfer the commits on `go2-cooperative-task-layer` through the final commit
listed in the delivery/test record. Keep `ranger_go2_codex_plan.md` and the
implementation plan beside the code. Use identical `multi_agent_task_msgs` v1
and shared `multi_agent_nav`; keep old `multi_agent_msgs` and
`/multi_agent/agents/{go2,ranger}/state` consumers unchanged.

## Public protocol

Five reliable, volatile topics:

| Topic | Type | depth |
|---|---|---|
| /multi_agent/go2/state, /multi_agent/ranger/state | multi_agent_task_msgs/msg/AgentState | 10 |
| /multi_agent/go2/observation, /multi_agent/ranger/observation | multi_agent_task_msgs/msg/Observation | 50 |
| /multi_agent/go2/command | multi_agent_task_msgs/msg/Command | 10 |

Full fields are in generated interfaces and `models.py`; all poses on these
topics are common lab_ref, with local_map_id/map_version/alignment_id explicitly
identifying the source. Image/stamp fields are trace-only; heartbeat expiry
uses receiver monotonic time. `simulation=false` is mandatory for live peers.

AgentState persists READY/deposited, current command identity and terminal reply,
resource request/occupancy/exit. Coordinator SYNC contains ack_sequence and phase.
Command actions: EXECUTE, SYNC, PAUSE, CANCEL, GRANT_RESOURCE. EXECUTE sequence
is per robot/task and idempotent. CANCEL targets cancel_command_id. Resource
GRANT binds parent_command_id; kind=RELEASE acknowledges a validated exit.
EXIT retains the **deposit command** parent identity and token, although it is
a new EXECUTE command. Resource grant must never replace the active main task.

Observation emits A/B/C individually per view, with DETECTED, NOT_DETECTED,
UNCERTAIN or API_ERROR; invalid depth retains DETECTED but position_valid=false.
Object common positions are not transferable parking points: select a local
approach using Ranger map/costmap and ComputePath before navigating.

## Ranger-specific work

Implement `adapters/ranger.py` against `adapters/base.py` and wire only the Ranger
live branch of AgentNode on that PC. This round supplies no guessed Ranger SDK,
camera topics, footprint, map path or hardware adapter.

Adapter operations: get_health; asynchronous compute_path and select_approach;
start_navigation returning goal_id; cancel_navigation; start_scan; stop_motion.
Use the same Future/result-queue contract as Go2/Fake and AgentRunner. Results
must bind task_id/command_id/goal_id/goal_kind. Health reports local map pose,
fresh localization/path/footprint, stationary confirmation and action terminal.
Scan checks all ABC, serializes Spin and navigation, and keeps inference off
the ROS heartbeat thread. Do not copy Go2 sensor-height/footprint assumptions.

Deploy Ranger agent on Ranger **Domain0**; no coordinator on Ranger. Build/source
Ranger's existing ROS overlay, generated interfaces and the common task package
using a locally verified CPython ABI/runtime. Go2's absolute venv path is not a
Ranger deployment requirement; configure COOPERATIVE_RUNTIME_PYTHON locally.
The common package's vlm_nav dependency presently carries Go2's vision utilities;
keep the existing shared vlm_nav package available, without starting its Go2 nodes.

Both PCs use separate occupancy maps. Fit common_from_ranger from ≥3 separated
non-collinear fixed references plus held-out references; set actual map versions
and the same alignment_id. Rebuilding a map invalidates the previous alignment.
No task-time calibration changes; missing/invalid configuration blocks live start.
Each robot configuration now includes `map_digest`: SHA256 of concatenated YAML
and referenced image SHA256 hex strings. Use `map_file_info` or the offline Go2
check entry to calculate it; verify each PC's own actual files and active map locally.
The Go2 PC does not need a copied Ranger map file. Remote map identity and digest
must still be filled consistently; unknown values remain unset.

## Pending communication setup

Use `config/bridge_ranger_pending.yaml` on Ranger and
`config/bridge_go2_pending.yaml` on Go2. Ranger exports only its state/observation
0→40 and imports Go2 state/observation/command 40→0. Go2 does the complementary
half. Both local domains are 0; robot identity comes from PC/topic source.
Keep Go2 bridge XML Domain0=enp4s0, Domain40=wlo1; supply Ranger's actual local NIC
isolation separately. Never enable bidirectional task routes or bridge Domain31.

Before activation, install/source custom interfaces in both bridge environments.
No connection, SSH, packets, bridge startup or Ranger code changes were performed
from the Go2 implementation session.

## Subsequent field acceptance

Validate Ranger local projection, approach/cancel/scan and executing-path monitor;
verify map alignment in both rooms/door; verify new task message routes/QoS,
Domain0 isolation and no recirculation; verify peer-body marking/clearing and
stopping; test door FIFO and whole-body exit, then the same-C full ordered mission.
These are separate from synthetic tests and prior single-robot acceptance.

## 2026-10-20 本轮最小接入说明（覆盖旧共享 C 要求）

本节是当前实施范围；上文的共同 lab_ref、跨地图标定、同一个 C 绑定只适用于旧共享模式。
本轮 `share_c_coordinates=false`，不共享 C 或 A/B 坐标，不需要任务坐标对齐。
**这里只完成 Go2 和公共任务层、fake Ranger；Ranger live adapter 仍不存在，未连接/部署/验收 Ranger。**
本轮没有自动提交；不得按上文“transfer commits”整体覆盖 Ranger 分支，应逐项合并公共改动并保留其硬件实现。

需同步的公共代码均在 `cooperative_ws/src/multi_agent_nav/multi_agent_nav/`：
`go2_exploration.py`、`generated_candidates.py`（公共逐步核心和本机候选）；
`models.py`、`planner.py`、`coordinator.py`、`agent.py`、`runner.py`、`frame_adapter.py`、`config.py`
（目标任务、私有坐标边界、严格验收、许可链）；`adapters/base.py`/`fake.py`、`mock.py` 及相关测试。
`agent_node.py` 的 Runner/身份/状态接线也需合并到 Ranger 自己的节点。
Go2 的 `adapters/go2.py` 只供参考接口和 ABC 请求，不替代 Ranger adapter。
`multi_agent_task_msgs` 字段、schema v1、action/kind 字符串、五话题和 bridge YAML **没有增加**；
字段指纹相同不证明下面的新命令语义已同步。旧 SYNC 模式不能与 SYNC_LOCAL_C 混跑。

### Ranger 本地执行接口

在 Ranger 控制电脑实现/接入自己的 adapter，交给
`AgentRunner(machine, adapter, descriptions, on_observation, config=config, success_radius=1.0)`。
原单机 `explore` 同步推进公共 `explore_steps`，仍读取0.81 m；协同逐 tick 推进并强制当前接近导航成功。
不要调用 Go2 启动脚本或复制完整探索循环。机器人参数读 `robot_ranger`，候选来源校验 `source_robot=ranger`。

- `get_health`：本机 map 下 base_link 位置、真实停稳、motion_terminal、定位/Nav2健康、当前执行路径及完整 footprint。
- `compute_path`、`select_approach(local_object, excluded, *, standoff_distance, path_accept)`、
  `read_approach_parameters` 返回 Future；后者返回当前 `xy_goal_tolerance` 和 `planner_tolerance`。
  必须检查**实际路径终点**预算，不能用请求 goal 替代。预算和最后停稳验收均用1.00 m；保留原安全余量。
- `costmap_transform(global_goal=False)`、`costmap_candidate_free(...,global_goal=False)`、
  `observation_candidate_free`、`approach_candidate_free` 使用 Ranger 自己的地图、TF和几何。
  远端候选全局完整 footprint/旋转空间/路径必须有效；不得仅因 local 窗口不足否决。
  local 已覆盖的碰撞/未知和缺图/陈旧/缺TF仍失败；运动继续本机局部避障/RegionGuard。
- `client.choose_candidate(snapshot)` 与 `vision.submit(callable)`（后台 Future）供本地 VLM 选本轮 P ID；
  `start_scan(descriptions, scan_id, *, stop_on_target)` 返回含 ABC 的 Observation 列表 Future。
  保留最多8次顺时针45°、停稳后新鲜RGB-D、图像时刻TF、ABC同图以及自身目标提前结束扫描。
- `start_navigation` 返回独立 goal_id，`results` 队列结果绑定 task/主command/goal/kind；
  `cancel_navigation`、`stop_motion` 和可选 `tick` 沿同样异步契约，不阻塞心跳。
  遵守 Runner 注入的 `motion_guard`；提供 `simulation`、`client`、`vision`，
  Runner 会设置 `selection_evidence_dir`、`scan_evidence_dir` 保存本机证据。
  原有 `selection_epoch` 若用于取消，需随取消变化并丢弃迟到结果。

Ranger 保留自己的相机、adapter、地图、TF、footprint、Nav2和配置路径。
各端沿用现有 `scene.resources`、`transit_exits`、`exit_points`，**在各自本机地图中**配置；
相同资源 ID 必须现场确认对应同一实物。无需交换多边形，也不将 Go2 配置复制到 Ranger。
本地地图身份/摘要、候选资产、定位、停止和资源门槛仍有效；本轮只取消跨机变换依赖。

### 命令和状态语义

| 既有消息组合 | 本轮含义 |
|---|---|
| SYNC_LOCAL_C | 建立私有坐标模式、task身份、阶段、心跳和ack；common_frame_id/alignment_id为空 |
| EXECUTE APPROACH_PICKUP，Go2=A / Ranger=B | 本机生成候选、VLM选ID、导航扫描并接近；验收后READY |
| EXECUTE SEARCH_VIEWPOINT，target_id=C | 双方READY后，本机继续找有效C；只完成定位，不执行放入 |
| EXECUTE APPROACH_DEPOSIT，target_id=C，observation_id=本机引用 | 复用本机C，接近/放入仍经过资源许可 |
| EXECUTE EXIT | 使用本机退出点；保留C接近命令的parent_command_id/resource_id/token |
| PAUSE / CANCEL / GRANT_RESOURCE | 继续原取消身份、许可、整机退出与RELEASE规则 |

私有 EXECUTE 不接受位置：position_valid=false、common_goal/object_position均为零；
coordinator 不设置 viewpoint_id，不选探索点。禁止收到坐标式pickup后退回旧验收分支。
AgentState 不发共同位置（pose_valid=false，common_pose零），但必须如实回报 localization_valid、nav_ready、
stopped、task/command/sequence/reply、ready/deposited/exited 及资源申请/占用/退出字段。
Observation 所有网络位置均无效且置零；仅本机可用 C 回报 `status=CACHED` 和自己的 observation_id，
原始目标坐标仅保留在本机缓存。task切换清空缓存，旧scan/goal/取消迟到结果不推进状态。
地图身份版本仍非空并固定在任务内；变化暂停。接收计时用本机 monotonic。

Go2、Ranger 完成各自A/B严格验收才READY；双方READY才进入C阶段。
CACHED可提前回报，但不能提前接近C。Go2 C放入并整机退出释放后才放行Ranger。
C许可贯穿接近/放入/EXIT；EXIT不能换token或parent。搜索和A/B接近不申请basket，避免READY屏障死锁；
门通行仍允许申请并在整机退出后释放。导航成功而未停稳、超过1m或身份过期均不得完成。

### 桥接和后续验收

沿用原五条 Reliable/Volatile 定向路由，无新增路由：

| 话题 | Go2 bridge | Ranger bridge | depth |
|---|---|---|---|
| /multi_agent/go2/state | 0→40 | 40→0 | 10 |
| /multi_agent/go2/observation | 0→40 | 40→0 | 50 |
| /multi_agent/ranger/state | 40→0 | 0→40 | 10 |
| /multi_agent/ranger/observation | 40→0 | 0→40 | 50 |
| /multi_agent/go2/command | 0→40 | 40→0 | 10 |

两端本地Domain0、共享Domain40；Go2 Domain0=enp4s0、Domain40=wlo1及原 bridge XML不变，
Ranger按自身真实网卡隔离。不得桥接TF/costmap/图像/action/速度或启用双向回流。
先在 Ranger 本机接入公共接口并完成本地验证，再按新增现场授权测试真实跨机任务、握手失联停车、
资源ID一致、门整机退出及Go2→Ranger顺序；本轮fake通过不替代这些检查。
构建和无节点复现统一见 [runbook](go2_runbook.md#2026-10-20-公共本地执行核心本轮入口差异与离线复现)。

## 2026-10-20 空地点生成补充

当前用户选择不录入家具标签。公共`generated_candidates_vlm`现以已知自由地图网格为基础，
有合法家具标注时保留原家具观察点并合并；无标注路径/缺文件/合法空家具可正常生成点，已有坏标注仍报错。
本补充覆盖前文把家具标注列为必需资产的描述，不取消本地资源/退出点或真实验证门槛。

Ranger需同步`generated_candidates.py`、`go2_exploration.py`、`go2_check.py`及候选VLM提示词变化，
保留自己的adapter和参数；没有新增消息/话题/配置项。网格仍由自身footprint/旋转半径、地图分辨率和原点生成。
候选按当前位置由近到远检查，每轮最多12个；空地点的内部furniture_id为空、稳定ID以free/开头。
VLM双图接口保持，无标签时家具层为空，空地点同样可选。不得在Ranger端继续将无家具作为启动失败条件。
0.81/1.00m入口差异、ABC扫描、C本机缓存和门/篮子许可链未变。
独立安装与无节点回归命令见[runbook新节](go2_runbook.md#2026-10-20-空地候选与可选家具标注)。
本轮依旧只有fake Ranger验证，不表示其硬件接入或真实跨机验收通过。

## 2026-10-20 独立C与取消保护区（最新覆盖）

用户已明确取消门/篮子保护区、资源许可和C先后顺序。本节覆盖前文同日仍保留许可/顺序退出的说明。
双方当前A/B成功且真实停稳后READY；双方READY后各自使用有效本机C缓存，否则本机继续探索。
一方找到C即可执行APPROACH_DEPOSIT，不等待对端找到C；双方可同时接近。
双方deposited且stopped、当前命令按task/command/sequence匹配结束后DONE。完成停在接近位置；不下发EXIT，不伪造exited。
这里deposited仍为接近验收/模拟放入，不包含实体机械臂动作。

**公共包同步范围：**在原公共执行核心/空地候选版本之上，合并models.py、agent.py、runner.py、coordinator.py、
coordinator_node.py、planner.py、config.py、mock.py、go2_exploration.py、go2_check.py及对应测试。
基本路径/完整footprint检查统一为models.motion_path_error；独立navigate/scan也不再调用RegionGuard。
ResourceManager/RegionGuard旧工具类仅保留兼容导入与原几何工具复用，活动任务不调用。
Ranger必须在自身控制电脑接入同一公共接口，保留自己的adapter/地图/TF/相机/footprint/Nav2参数；不得整体覆盖其分支。

**命令和状态：**SYNC_LOCAL_C、EXECUTE APPROACH_PICKUP/SEARCH_VIEWPOINT/APPROACH_DEPOSIT、PAUSE/CANCEL语义保留。
GRANT_RESOURCE与EXECUTE EXIT明确拒绝；不再申请资源、转入DOOR_TRANSIT或要求parent/token链。
resource_id/token/parent_command_id和状态资源字段保留schema兼容，正常新任务为空；exited保留false，不能作为完成门槛。
READY/deposited必须来自当前目标的成功导航、真实停稳、动作终态和≤1.00m距离；单机仍≤0.81m。
不要混用旧版还等待EXIT/许可的Ranger，即使消息字段指纹一致。

**配置和路由：**不增加字段/话题，不修改桥接YAML；沿用前文五条Reliable/Volatile定向路由及Domain0/40隔离。
本轮不传地图坐标，无地图对齐要求；CACHED仍只传本机引用，无双缓存屏障。
不再要求scene.resources、transit_exits、exit_points或区域stop_margin；旧值即使存在也不拦截。
各端地图身份/摘要、全局候选完整footprint和规划路径、局部运动避障、心跳/失联停车/取消与迟到保护继续有效。
没有任务层C互斥和固定退让，不能据此承诺C处两机先后通行；现场仍需验证各自局部避障。

Go2独立安装为build/go2-independent-c-20261020/install/multi_agent_nav，默认环境尚未切换。
完整构建与离线回归统一见[runbook最新节](go2_runbook.md#2026-10-20-取消保护区与独立c执行最新)。
本轮仅fake Ranger验证，不能称为Ranger真实接入或跨机验收；后续须在其电脑完成接口后再按现场授权验收。

## 2026-10-22 双机验收前公共到达修复

用户已说明双方完成迁移；本节不再用旧“Ranger未接入”描述断言当前远端状态。本轮没有访问Ranger实际代码，以下须由其本机核对。
schema v1、五话题及SYNC_LOCAL_C/EXECUTE三种任务/PAUSE/CANCEL不变，资源许可与EXIT继续拒绝。
当前完整检查、消息摘要、Ranger核对清单及验收顺序见[runbook本轮节](go2_runbook.md#2026-10-22-双机联动前必要检查)。

仅需同步公共agent.py的本轮修复及对应测试，不覆盖Ranger自己的adapter/地图/硬件参数：

- 私有模式必须有`AgentRunner(..., config=本机配置, success_radius=1.0)`启用本地执行器；缺失时拒绝EXECUTE，不能回退到网络零坐标导航。
- 旧共享执行分支收到当前目标导航SUCCEEDED后进入内部ARRIVAL_STOP，等待真实stopped和motion_terminal，再验导航种类及本机距离≤1m；不再当场置READY/deposited。
  Ranger若有自定义旧回调直接置标志，也必须移除相同旁路；ARRIVAL_STOP不能被自定义状态映射当作完成。
- 正常私有核心的严格导航成功/真实停稳/动作终态/距离判据保持；双方READY屏障、独立C引用、另一方继续搜C以及当前命令终态后DONE保持。
- 实际安装必须包含修复，不仅检查源码或消息字段。Go2旧默认安装仍缺此修复，尚未自动更新；两端上线前各查实际导入与preflight。

本轮Go2源码新增覆盖35项通过（29项新文件、6项Ranger故障参数化），未启动ROS或真机联动；
完整相关集合266通过/5项改前同样失败，失败依赖旧关闭gates假设，未通过改配置消除。真实Ranger行为及双机现场结果仍待核对。

## 2026-10-22 异步启动公共语义（待Ranger核对安装）

本轮没有访问或覆盖Ranger实际源码/安装。共同AgentMachine已实现PREPARING → WAITING_PEER/STANDBY → SYNCING/SYNCED；不能沿用“先等对方IDLE，本机才准备”的门槛。双方可先后启动，当前任务执行前的单端重启可重新接受SYNC_LOCAL_C、立即发布本轮身份并握手；执行后重启/失联仍严格暂停，恢复消息不恢复运动。

待Ranger维护端核对：同步公共models/agent与真实peer接收回调；真实health来自本机；SYNC接受立即发布；完整保留新增run_state；本机motion_terminal/stopped判据与本地执行器配置保留；不得将SYNCED或WAITING_PEER映射成ready=true。Go2 coordinator同版本等待双方本轮SYNCED后才派工。
schema v1及五话题不变，但旧版IDLE不能冒充新版启动握手；仅消息接口一致不足以验收。Ranger自定义adapter/地图/硬件参数不在本次覆盖范围。
逐项待确认和离线证据见[修改说明](async_startup_changes_20261022.md)；两端实际部署、DDS调度和现场四类异步场景均未验证。

## 2026-10-22 最终契约的 Go2 实施与安装答复

用户提供[最终契约](ans_async_startup.md)，并确认Ranger已依其修改；本轮仅Go2本机实现与离线核对，没有连接Ranger验证其实际安装。
Go2已补两种SYNC编码、仅SYNCING新建task、run_state/reply双字段SYNCED、提交/乱序/首条丢失保护、固定peer地图身份、干净私有报文及旧worker等待。已发ACK记录不因peer先移动丢失；phase提交后首个EXECUTE未到也执行严格失联/重启监护。暂停后显式新任务无需交替重启对端，旧PAUSED缓存不取消新握手。

Go2默认安装已改为build/go2-async-contract-20261022/install，旧overlay未覆盖，原进程未热更新；27个Python模块与源码一致，三个schema摘要均匹配契约。参数/双方地图身份原已一致，未覆盖Ranger adapter、硬件、地图或本机许可。实际安装离线551通过/5同基线失败，新增86项通过，默认preflight通过；现场静止通信和运动未验收。
详见[本轮修改说明](go2_async_contract_changes_20261022.md)，以前“reply未明确”“未构建”“Ranger接口待决定”由最终契约及本节覆盖。
