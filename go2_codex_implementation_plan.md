# Go2 电脑协同导航代码修改 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task when available. Steps use checkbox (`- [ ]`) syntax for tracking. 本轮采用 Go2 电脑上的 Codex 顺序执行；不要求启动并行 agent。CLI 若未安装这些技能，直接遵循本文步骤，不因技能缺失停止工作。

**Goal:** 在 Go2 控制电脑实现公共协同任务层、Go2 本机适配和可复现的双模拟 agent 验证，为随后 Ranger 电脑独立接入提供相同协议与明确交接。

**Architecture:** 两端各有 robot_agent，各自使用独立 occupancy map 和 Nav2。Go2 电脑运行 coordinator 与 RulePlanner；以 lab_ref 交换目标和状态，各端 FrameAdapter 转换坐标、本机判断可达性。第一版完成 Go2 找 A、Ranger 找 B、READY 屏障、同一 C 顺序放入与退出。

**Tech Stack:** 沿用实际仓库的 ROS 2 发行版、Python、rclpy、Nav2、现有 VLM API 与 Domain bridge。纯逻辑 pytest；消息使用 ROS 2 interfaces；不升级系统、ROS、Nav2 或模型 SDK。

**Spec:** 已审查的 `ranger_go2_codex_plan.md`。执行时将该文件与本计划放在同一目录；两者随代码进入仓库 `docs/superpowers/plans/`。架构尚未实现，本计划不是部署或测试通过记录。

**部署修订（2026-10-07）：** 按用户最新确认，两端真实本地ROS均为Domain0，各PC通过自己的domain_bridge连接共享Domain40。Go2本机Domain0/CycloneDDS=enp4s0 ONLY，Domain40/CycloneDDS=wlo1 ONLY；不迁移已成功的底层域，不使用30/31作live部署。Domain31仅可作为不桥接的本机mock测试域。旧agent里程计话题和旧multi_agent_msgs保持；新协议包为multi_agent_task_msgs。

## Global Constraints

- 当前操作的是 **Go2 控制电脑**；Ranger 使用另一台电脑。本轮不连接 Ranger、不 SSH、不尝试跨电脑/跨 Domain 通信、不启动旧桥接链路作测试。
- 部署位置：coordinator和Go2任务agent在Go2 PC Domain0；Ranger任务agent在Ranger PC Domain0。桥接链路为Ranger PC 0↔共享40↔Go2 PC 0，保持双方独立地图。Domain参数配置化；live=0，mock可用31，不自动回退/切域。
- Go2有线enp4s0=192.168.123.222，连接Go2上位机192.168.123.18；Go2无线wlo1=192.168.1.118，Ranger无线wlo1=192.168.1.108。Go2 Domain0的DDS只绑定enp4s0，Domain40只绑定wlo1。复用现有按域/参与者选网卡的实际配置，不用单网卡全局XML覆盖bridge两侧，不改变硬件SDK自身通信参数。
- 固定 Go2→A、Ranger→B。每张观察图同时检查 A/B/C；当前有效自身 A/B 接近主任务 Nav2 SUCCEEDED 才置 READY。
- 双方 READY 且状态新鲜后才进入 C 阶段；早期 C 线索允许保存。Go2 先放入并整机退出，Ranger 后放入并退出，二者完成才 DONE。
- lab_ref 数值与当前 Go2 地图一致。Ranger→lab_ref 为经验证 SE(2)，不编造标定、场景坐标、真实话题或地图路径。两端 local_map_id 不要求相同。
- 第一版只用 RulePlanner；保留 `decide(snapshot) -> planned_actions`。不新增 global VLM 调用、自动覆盖搜索、动态帮搜、第三张 occupancy map 或跨域 TF。
- 保持已验证 AMCL/传感器/底层 Nav2；联合任务只允许一个本机上层导航目标控制者。扫描速度与导航速度必须串行仲裁。
- 初始心跳 2 Hz、失联阈值 3 s、任务超时 600 s，均配置化；新鲜度按接收端单调时钟判断。
- 门/C 资源只在确认整机 footprint 离开后释放；失联、定位无效、退出失败不能自动释放。同一机器人最多持有一个资源。
- VLM API 不阻塞心跳、取消和区域监护；未检出、不确定、API 失败分开记录。API key 继续从环境读取，日志不得输出密钥。
- 默认启动不运动；显式启动任务。mock 模式只接 fake adapters，禁止混入真机 Nav2 或用 mock Ranger 自动推动真实 Go2 完成联合任务。
- 不覆盖用户未提交修改，不重置仓库、不迁移已有成功脚本。常规接口适配和失败修复自主完成；缺现场数值时继续完成代码与模拟测试，保留真机启动门槛。

## Review Focus

1. 取消后晚到成功、旧 task/command/goal 或内部子目标成功：不能置 READY/deposited；由任务 3 的 goal 身份测试覆盖。
2. 地图重建、标定版本错配、无效位置或两端各自不可达：拒绝相应坐标任务，不能误用对方地图；由任务 1、2、3 覆盖。
3. READY、命令和资源授予重复/乱序，coordinator 重启：不重复取放、不过早进入 C、不自动续跑；由任务 3、4、5 覆盖。
4. 持资源时失联、出口导航成功但机身仍在区域、重规划绕过门：停止推进并保留占用；由任务 5 覆盖。
5. VLM 超时或反馈异常、clock 不一致、mock/真机混用：心跳继续，不能把失败记成没看到或真机成功；由任务 3、4、6、7 覆盖。

## 0. 执行位置与第一轮读取

历史仓库线索是 `/home/isee-pst/unitree_ros2/VLM_Nav`，历史分支是 `VLM_nav_go`；二者必须本机核实，不能直接当作当前事实。当前对实际仓库尚未完成审查。

据用户转述，本机Codex已核实上述仓库/分支、ROS2 Humble、未发现适用AGENTS.md、旧协议有既有消费者，并在co-nav-real环境中通过52项基线测试。可复用已有审查记录，只补本次部署修订所需内容；这些基线不是新任务层测试。本会话没有直接访问Go2仓库，不能把转述写成云端已验证。

- [ ] 在 Go2 本机读取 `pwd`、`git status --short`、`git branch --show-current`、仓库内 `AGENTS.md`；使用 `rg --files` 找 `scripts/`、package.xml、launch、Nav2/视觉/桥接配置。避免递归扫描用户整个 home。
- [ ] 确定真实仓库根目录 `REPO`、colcon source 根目录 `ROS_SRC`、实际 ROS 发行版与 build/source 顺序；读取关键脚本调用链，不启动机器人。
- [ ] 记录当前图像/深度/TF 投影、三目标识别、旋转扫描、目标接近、Nav2 action、取消/停止、定位状态、路径反馈、costmap/footprint 的真实接口；判定哪些可直接复用。
- [ ] 静态读取既有桥接配置，记录桥接软件/类型/QoS/域方向。现有测试消息通过仅是背景，不能记作新自定义消息已通过。
- [ ] 记录两端本地域均0和Go2双网卡隔离；核对Go2 bridge的两个DDS参与者分别选择enp4s0/wlo1。现场发现隔离列待验。本轮不启动ROS图探测/桥接/网络探测，不由“同域0”擅自撤掉bridge。
- [ ] 在 `REPO/docs/multi_agent/go2_integration_notes.md` 建立“真实入口→适配调用→输出/异常→是否需要现场验证”表，记录原有成功启动方式与回退方式。
- [ ] 按本机开发规范创建工作分支；仅在不会破坏现有脚本绝对路径和运行方式时使用独立 worktree。保持工作区已有修改。后续提交只暂存本轮文件。
- [ ] 将本计划、架构文件存入 `REPO/docs/superpowers/plans/2026-10-07-go2-cooperative-navigation.md` 与同目录 `ranger_go2_codex_plan.md`。把下文 `ROS_SRC`、`REPO` 转为实际绝对路径写入集成记录。

本步骤输出是接口记录与确定的文件路径。仅缺 Ranger 配置/标定/场景坐标不阻塞后续模拟实现；Go2 接口缺口明确记录，不能造一个同名话题假装已接入。

## 1. 文件边界与公共协议

以下是**拟新增**路径，不表示已有文件存在。若存在同职责包，按既有结构整合，在集成记录中给出一对一映射，避免维护第二套接口。

采用本机Codex已提出的目录：`ROS_SRC=/home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src`。旧根包保持，显式传入两个base-paths并先用colcon list确认新包各出现一次；新任务接口不替换旧multi_agent_msgs。build/install/log输出独立于旧工作区。

| 拟新增路径 | 职责 |
|---|---|
| `ROS_SRC/multi_agent_task_msgs/msg/AgentState.msg` | 心跳、任务状态、当前/终态回执、资源反馈 |
| `ROS_SRC/multi_agent_task_msgs/msg/Observation.msg` | 逐目标检查结果与可选 common 物体位置 |
| `ROS_SRC/multi_agent_task_msgs/msg/Command.msg` | 主任务与控制事件 |
| `ROS_SRC/multi_agent_nav/multi_agent_nav/models.py` | 无 ROS 依赖的 DTO/枚举与协议校验 |
| `…/frame_adapter.py`、`…/calibration.py` | SE(2) 正反转换、对应点拟合/验证 |
| `…/agent.py`、`…/adapters/base.py`、`…/adapters/fake.py` | agent 任务状态机、能力接口、模拟实现 |
| `…/coordinator.py`、`…/planner.py` | 全局状态/命令回执、固定规则决策 |
| `…/resource_manager.py`、`…/region_guard.py` | FIFO 资源许可、路径/footprint 边界判断 |
| `…/adapters/go2.py` | 现有 Go2 视觉/扫描/Nav2 接口复用 |
| `…/ros_codec.py`、`…/agent_node.py`、`…/coordinator_node.py` | DTO↔ROS、话题与计时调度 |
| `…/start_task.py` | 本机显式开始/暂停任务客户端 |
| `ROS_SRC/multi_agent_task_msgs/srv/StartTask.srv` | 本地启动服务；不跨桥接 |
| `ROS_SRC/multi_agent_nav/config/`、`…/launch/`、`…/tests/` | 四类配置、启动入口、必要测试 |
| `REPO/docs/multi_agent/` | 集成记录、运行说明、Ranger 交接和测试结果 |

省略号均为 `ROS_SRC/multi_agent_nav/multi_agent_nav`；建包的 package.xml、构建文件、资源索引和入口点按本机 ROS 2 约定。StartTask.srv 由 interfaces 包生成，Python 节点包只使用该接口。

### Task 1：DTO、消息与启动校验

**Files:** 上表三个 msg、实际 `multi_agent_task_msgs/srv/StartTask.srv`、`models.py`、`ros_codec.py`；`config/{alignment,scene,robot_go2,robot_ranger}.yaml`；`tests/test_protocol.py`。

**Interfaces:** 定义 `Pose2D(x: float, y: float, yaw: float)`、`Command`、`AgentState`、`Observation`、`Snapshot`、`PlannedAction`、`Effect`、`CommandReply`、`GoalResult`、`Health`、`ResourceFeedback`、`ResourceRequest`、`ResourceDecision` DTO。`validate_command(command: Command, context: AgentContext) -> CommandReply`；`to_ros(dto)` / `from_ros(message)` 仅在 codec 中依赖生成接口。`AgentContext` 含当前任务、地图/标定身份、robot_id 和运行模式；`Snapshot` 含双方状态及接收时间、观察记录、资源、命令记录、阶段与 now 单调时间。

最低字段约定：

- 共用 `schema_version`、task_id、robot_id/recipient_id、common_frame_id、alignment_id；状态/观察附来源 local_map_id 和 map_version。时间戳用于追溯，不用于跨电脑计算失联。
- AgentState 含 simulation、common_pose/pose_valid、定位与 Nav2 健康、运行状态、ready/deposited、主 command_id、导航 goal_id、命令 ACK/终态/原因、资源请求/占用/退出反馈。资源字段可以是包内辅助 msg；不必另建话题。保留单调命令序号与最新终态回执直至 coordinator 确认。
- Observation 每条对应一个 target_id=A/B/C，含 observation_id、viewpoint_id、scan_id/视角、识别状态、confidence（不可把模型自报值作为唯一导航门槛）、common_position/position_valid、图像时间和来源。状态用 DETECTED / NOT_DETECTED / UNCERTAIN / API_ERROR。识别但深度无效可以 DETECTED、position_valid=false，不能导航。
- Command 含 `command_id`、递增 `sequence`、EXECUTE/SYNC/PAUSE/CANCEL/GRANT_RESOURCE、phase、kind、目标/观察点/observation_id、可选 common_goal 或 object_position、parent_command_id、resource_id/token。CANCEL 指明所取消主命令；GRANT_RESOURCE 是控制事件，不替换主任务。ACK/结果由状态持续回报。
- SYNC 发给 `recipient_id="*"`，含 phase 与最近命令/状态确认；其 ID 不占用执行序号。只有此类广播和指定给自己的消息可接受，错误 recipient 必须忽略。
- StartTask 请求仅选择已配置 scenario_id，不接受任意运动坐标；响应 accepted/task_id/reason。task_id 由 coordinator 产生。暂停客户端调用同一节点的实际本地暂停接口（可用标准 Trigger），不新增跨端服务。
- 已有接口包需先查消费者；若兼容修改无法保证，新增版本化类型并同步所有新配置，不破坏旧测试节点。
- 本机审查已确认旧multi_agent_msgs/AgentState被里程计节点消费，故采用独立multi_agent_task_msgs；旧 /multi_agent/agents/{go2,ranger}/state 保持原类型。新 /multi_agent/{go2,ranger}/state 使用任务版AgentState，不混用话题或把旧位姿消息当READY。新任务节点用不冲突的名称（例如multi_agent_go2_agent），不与旧go2_agent_node重复。

配置模板：Go2 identity 已定义，**地图身份仍由现场确认**；Ranger alignment 默认 `validated: false`，不填零值假装标定。scene 数值缺失保持 disabled；mock 使用 tests/fixtures 的专用虚拟场景与标定，不能让 live 加载它。模型名/endpoint/timeout/API key 环境变量名参数化，不硬编码临时 qwen 名称；planner 模型字段可留空且不调用。

- [ ] 在 `test_protocol.py` 写测试：A/B/C 结果独立编码；无位置无法成为导航线索；recipient/schema/task/frame/alignment 不匹配拒绝；local_map_id 不同但各自匹配标定可接受；重复 EXECUTE 序号不新增效果。
- [ ] 运行 `python3 -m pytest tests/test_protocol.py -q`，确认新行为尚未实现而失败。
- [ ] 实现 DTO、协议校验、消息生成声明与配置加载；缺现场标定/场景时返回可读启动门槛，mock 配置与 live 配置显式区分。
- [ ] 编译 interfaces 后增加真实 DTO↔ROS round-trip 测试，验证枚举、空位置和资源控制字段不会丢失；不把纯结构镜像测试作为状态正确性的证据。
- [ ] 跑该组测试并提交本任务文件；在接口记录中固定最终 schema、QoS 与文件映射。

## 2. 不同地图的坐标适配

### Task 2：FrameAdapter 与离线标定

**Files:** `frame_adapter.py`、`calibration.py`、`tests/test_frames.py`；离线入口 `multi_agent_calibrate`。

**Interfaces:** `FrameAdapter.to_common(pose: Pose2D) -> Pose2D`、`to_local(pose: Pose2D) -> Pose2D`、`polygon_to_local(vertices) -> vertices`、`validate_ids(local_map_id, map_version, common_frame_id, alignment_id) -> None`；不匹配抛专用异常，由 agent 转成拒绝原因。`fit_alignment(source_points, common_points, held_out_pairs, thresholds) -> CalibrationReport`。Report 含方向、tx/ty/yaw、拟合误差、独立验证误差、几何条件/是否达标与地图身份。

- [ ] 写 `test_frames.py`：非零旋转平移的正反变换、yaw wrap、多边形每个顶点转换；Go2 identity；地图版本变化拒绝；≥3 分散拟合点+独立点恢复已知合成变换；重复/近共线点、反射、缺独立点、验证超阈值不得 validated。
- [ ] 运行该文件，确认行为测试失败。
- [ ] 实现 SE(2)：local→common 使用 R·p+t，反变换 Rᵀ·(p−t)，yaw 同步旋转。最小二乘刚体拟合禁止引入缩放/反射；近共线判据与误差门槛作为明确参数、记录实际数值。
- [ ] 离线工具读取用户提供的对应点 YAML/CSV，只输出候选 alignment 与验证报告，不发布 TF、不修改正在运行的标定。缺真实点可完成合成验证；现场采点单列。
- [ ] 跑该组测试并提交。记录跨图 motion 的硬门槛：真实两端地图身份、共同 alignment_id、现场独立验证达标；不能通过修改 `validated=true` 绕过报告。

## 3. 本地任务机及取消语义

### Task 3：robot_agent 的纯逻辑与 fake adapter

**Files:** `agent.py`、`adapters/base.py`、`adapters/fake.py`、`tests/test_agent.py`。

**Interfaces:** `AgentMachine.accept(command: Command, now: float) -> CommandReply`；`on_goal_result(result: GoalResult, now: float) -> list[Effect]`；`on_observation(observation: Observation, now: float) -> list[Effect]`；`tick(now: float, health: Health) -> list[Effect]`；`state() -> AgentState`。

`RobotAdapter` 提供 `get_health() -> Health`、`compute_path(local_goal: Pose2D) -> PathResult`、`select_approach(local_object: Pose2D, excluded_candidates) -> ApproachResult`、`start_navigation(local_goal: Pose2D, task_id, command_id, goal_kind) -> goal_id`、`cancel_navigation(goal_id) -> cancellation_handle`、`start_scan(target_descriptions, scan_id) -> scan_handle`、`stop_motion(reason) -> stop_handle`。长操作通过异步句柄/回调完成；定义在 base，fake/Go2 实现保持相同签名。区域监护所需位姿、footprint、当前路径由 Health/PathResult 提供，附有效性。Effect 是执行这些调用或发布状态/观察的结构化请求，纯任务机不直接 import rclpy。

GoalResult 绑定 task_id/command_id/goal_id 与 goal_kind。主任务 kind：SEARCH_VIEWPOINT、APPROACH_PICKUP、WAIT、APPROACH_DEPOSIT、EXIT；内部 kind：VIEWPOINT_NAV、DOOR_TRANSIT、WAIT_NAV、PICKUP_NAV、DEPOSIT_NAV、EXIT_NAV。只有当前 APPROACH_PICKUP 下 PICKUP_NAV 成功置 READY，只有当前 APPROACH_DEPOSIT 下 DEPOSIT_NAV 成功置 deposited。

- [ ] 写 `test_agent.py`：自身 A/B 接近成功置 READY；扫描/等待/过门不置 READY；C 接近成功只置 deposited；其他机器人 READY 不修改本机标志；拒绝不匹配地图身份/无效位置/不可达候选。
- [ ] 添加取消测试：CANCEL/PAUSE/失联后立即使旧结果失效；晚到 SUCCEEDED 无效；取消未确认前不启动新运动；重复命令只重复回执；资源控制不重启主任务。
- [ ] 添加停止确认测试：取消回复已接受但action未终态、action已终态但机身仍运动、里程计陈旧均不得启动替换运动。低速阈值和持续时长配置化；监督NavigateToPose与Spin各自句柄，失败/超时保持暂停。
- [ ] 运行测试，确认未实现行为失败。
- [ ] 实现主任务串行执行、内部子目标身份、单次重试、回执保持与取消状态。主任务替换需旧运动已停止/取消确认；状态标志只随合法结果改变。先 READY 去等待点，保留 ready。
- [ ] 扫描输出 A/B/C 三条明确结果，记录实际视角/检查目标；API_ERROR 不记一次有效未检出扫描。每点每台每阶段至多完成一轮扫描；失败重试一次，耗尽报告本轮未找到，不宣布不存在。
- [ ] fake adapter 支持延迟结果、取消、路径不可达、不同机器人不同停车点、API 超时与失联注入；全部不连接真实传感器/action/server。
- [ ] 跑该组测试并提交。先定义任务机状态，不把完整旧单机自主任务循环直接塞进 adapter。

## 4. coordinator 与固定策略

### Task 4：READY 屏障与同一 C 的任务调度

**Files:** `coordinator.py`、`planner.py`、`tests/test_coordinator.py`。

**Interfaces:** `Coordinator.start(scenario_id: str, now: float) -> StartReply`、`ingest_state(state: AgentState, received_at: float) -> None`、`ingest_observation(observation: Observation, received_at: float) -> None`、`tick(now: float) -> list[Command]`、`pause(reason: str) -> list[Command]`；`RulePlanner.decide(snapshot: Snapshot) -> list[PlannedAction]`。Coordinator 校验 planned actions、分配 command_id/sequence、去重/跟踪回执；planner 不发送 ROS 消息。

- [ ] 写 `test_coordinator.py`：固定分工、不同观察点预约；一方 READY 不进 C；双方当前 task READY、新鲜/定位有效才进 C；早期 C 只保存；两端使用同一 observation_id，各选自己的停车点合法。
- [ ] 添加顺序测试：Go2 deposited 但未退出，Ranger 不得接近 C；Go2 整机退出后 Ranger 可放入；两台 deposited+退出才 DONE；等待点未结束/取消确认前不发替换运动。
- [ ] 添加异常测试：旧任务 READY、重复 observation、乱序命令回执不推进；receiver 单调时间超过 3 s 暂停；发送方 stamp 超前/落后不影响新鲜度；600 s 超时暂停/失败；重启 IDLE/PAUSED、不自动续跑；simulation 属性混用拒绝启动。
- [ ] 运行测试，确认未实现行为失败。
- [ ] 实现规则策略、目标线索表、机器人特定失败线索记录、扫描记录与观察点预约。某机器人不可达只影响其候选，不把线索全局判废；失败接近尝试一次其他可达停车点后恢复搜索。
- [ ] coordinator 失联/定位失效触发 PAUSE/CANCEL，agent 独立监测 SYNC/peer 状态；状态恢复不自动续跑。SYNC 不产生新目标。两个来源状态各自接收计时。
- [ ] C 接近区与退出点必须有有效配置。C 选择固定、可追溯；任务内不因另一个 C 线索悄悄换篮子。若地区配置不足或线索无有效位置，暂停并指出缺项。
- [ ] 跑该组测试并提交。ModelPlanner 仅保留接口，不新增 API 请求或模型能力推测。

## 5. 门/C 互斥与路径监护

### Task 5：资源请求、整机退出与重规划检查

**Files:** `resource_manager.py`、`region_guard.py`，连接 `agent.py`/`coordinator.py`；`tests/test_resources.py`、`tests/test_region_guard.py`。

**Interfaces:** `ResourceManager.request(request: ResourceRequest, now: float) -> ResourceDecision`、`update(feedback: ResourceFeedback, now: float) -> None`、`freeze(robot_id: str, reason: str) -> None`；`RegionGuard.check(pose, footprint, path, permits, health) -> GuardDecision`。request/token 绑定 task_id、robot_id、parent_command_id、resource_id；GuardDecision 表明允许、区域外等待、需要资源或停止，不直接发速度。

- [ ] 写资源测试：同时申请按 FIFO 唯一授权；同 token 重复授予幂等；不同任务/机器人/主命令 token 拒绝；一次只能持一个资源；先完成门再申请 C；取消尚未入区的排队请求可移除，已有占用不能猜测释放；刚授予时持有者在区外不能被误认作已通行退出。
- [ ] 写区域测试：中心已在区域外但 footprint 相交不释放；过门子目标 SUCCEEDED 不释放；有效定位下 footprint 完全外且反馈对应正确 token 才释放；持有者失联/定位无效/退出失败保留占用。
- [ ] 添加重规划测试：初始路径不经门，新路径跨门且无 token，则在进入前暂停/取消；无法取得有效路径、边界余量或定位时禁止继续跨受控区域；C deposited 不释放。
- [ ] 运行测试，确认未实现行为失败。
- [ ] 实现 common 区域→local 转换、本机路径与 swept footprint/保护边界检查。区域要覆盖整段不能会车空间；停止余量根据速度、停止能力、footprint 与标定误差配置，不能凭空给“安全厘米数”。区分 RESERVED/ENTERED/EXIT_CONFIRMED；正常释放须曾进入后确认整机退出，若取消尚未入区的已授予任务，则须确认取消/停止且整机在区外后撤销许可。
- [ ] 无许可先在区域外等待，授予控制事件恢复当前主任务；过门为内部动作。每次路径更新及连续位姿反馈都检查许可，不能只检查最初规划。定位/路径/停止监护缺接口时标记 live gate，不静默降级。
- [ ] 跑该组测试并提交。现场停止和 peer 身体避障仍需真实验证，算法测试不等价于碰撞保证。

## 6. 接 Go2 已有能力

### Task 6：Go2 adapter 与三目标感知

**Files:** `adapters/go2.py`、`config/robot_go2.yaml`、任务 0 找到的必要现有入口（实际路径写入记录）；`tests/test_go2_adapter.py`。

**Interfaces:** 完整实现 Task 3 的 RobotAdapter。复用现有图像→目标像素→深度/TF→本机 map 投影，FrameAdapter 再转 common；不要用 VLM 提供的文字距离或像素直接作为导航地图坐标。若已有能力仅导航至观察者位姿、没有可验证的物体 map 投影，报告此缺口；有识别结果也必须 position_valid=false，不能用观察者停车点冒充物体位置。

- [ ] 针对真实 API/SDK 接口用 stub 写测试：合法 ABC 结构解析、部分未检出、无效深度、API超时、异常输出；无有效 common_position 不发可导航线索。
- [ ] 写非阻塞与停止测试：延迟推理时 state/SYNC 计时与暂停处理继续；Nav2 SUCCEEDED/ABORTED/CANCELED 正确映射和 goal 身份保留；扫描运动停止后才能导航；取消/停止异常不继续发新运动。
- [ ] 运行测试，确认新适配未实现而失败。
- [ ] 实现有限队列/异步推理、超时、已取消 scan 的结果丢弃；同图优先单次 ABC 请求，若现场样例表明不可靠，允许同图分别询问 A/B/C 的配置模式，输出统一 Observation。模型通过配置切换，不在本任务擅自迁移服务。
- [ ] Go2 本机 ComputePath/选接近点/导航/取消接真实接口；接近点在 Go2 map/costmap 上选取，保留合适 footprint/停车距离。任务机拥有 goal 发起权，旧脚本保留但联合入口不得启动其自主发 goal 循环。
- [ ] 区分地图位姿与里程计：读取 /odometry 的frame/child_frame，不直接当map位姿；使用有效map→base_link位姿或正确组合map→odom→base_link后转lab_ref。定位健康/跳变监护仍有效，里程计低速仅用于停止确认。
- [ ] 明确实际导航重规划路径的接口/时间戳，订阅真实正在执行的路径更新并连接RegionGuard；ComputePathToPose的一次预检不等于当前NavigateToPose路径。无法监护当前路径/保护边界时关闭相关live通行，不以假订阅代替。
- [ ] 复用co-nav-real前记录其绝对解释器路径；验证rclpy、httpx和生成消息导入，核对安装后ROS entrypoint实际解释器。纯pytest通过不能代替ROS节点运行环境检查，不把任意conda解释器接到系统ROS二进制。
- [ ] 静态审查 Go2 动态 costmap 的 marking/clearing、实时 scan/点云来源、运动方向感知覆盖与 peer 身体高度；保持已验证 AMCL scan。仅对有明确证据的缺口做最小配置改动，现场“能看到 Ranger 身体并停止”列待验，不凭纯软件测试宣称解决。
- [ ] 跑该组测试并提交。若现有扫描/视觉代码无法 import，抽取小函数/包裹薄接口，保持原启动脚本可用；不重写整套单机系统。

## 7. ROS 节点、本机验证与桥接准备

### Task 7：本机 mock 联调和 live 启动入口

**Files:** `agent_node.py`、`coordinator_node.py`、`start_task.py`、入口/构建声明；`launch/{cooperative_mock,go2_cooperative}.launch.py`；`tests/test_mission_mock.py`、`tests/test_ros_roundtrip.py`；`config/bridge_routes.md` 与实际 bridge 的**待启用**配置片段。

**Interfaces:** node仅负责ROS收发、时间、异步调度和效果执行；纯机和adapter接口不更名。`ros2 launch multi_agent_nav cooperative_mock.launch.py`只启coordinator+fake Go2+fake Ranger，使用配置的mock域（可为31），本机限制且不桥接；`ros2 launch multi_agent_nav go2_cooperative.launch.py`在Domain0只启coordinator+Go2任务agent，不隐式启动Ranger/mock/bridges/原自主导航。底层已有Domain0栈保持验证过的启动方式，不迁到31。live缺真实Ranger/标定/场景时保持无运动，仍可完成编译及mock联调。

- [ ] 写完整 fake 流程：Go2 A、Ranger B，状态互见 READY，早期 C 延迟使用，共同 C、Go2 放入退出→Ranger 放入退出→DONE；两张虚拟地图非零变换；包含 Ranger 发现 A 后 Go2 利用共享线索。
- [ ] 写故障注入流程：门竞争、held resource 失联、暂停晚到成功、同一目标各自不可达、VLM超时、重复消息；每项断言阶段/动作/资源结果，不只检查节点活着。
- [ ] 执行 pytest，确认新增联调预期失败；实现节点、显式 StartTask/暂停接口与启动门槛。live 缺 peer/标定/场景时保持无运动并给出原因，不因单机缺 Ranger 阻塞编译或 fake 验证。
- [ ] 新话题沿用已审查五条：`/multi_agent/{ranger,go2}/{state,observation}` 与 `/multi_agent/go2/command`。两个 agent 各自订阅对方 state/observation，state 变化立即发布并 2 Hz 持续发布；coordinator 周期 SYNC，命令重发/回执幂等。
- [ ] 显式固定 QoS：state/command 使用 RELIABLE+VOLATILE，depth=10；observation 使用 RELIABLE+VOLATILE，depth=50。启动 SYNC/重复状态与应用回执提供恢复，避免历史 motion 命令被订阅者自动执行。若 bridge 不支持此配置，调整两端与桥接为同一经文档核实的组合并记录理由。
- [ ] 本机ROS smoke：只在mock环境发布合成state/observation/control，不发布真实运动目标。mock域可为31且不启bridge；ROS_LOCALHOST_ONLY或同类本机限制不等于隔离本机所有进程，同时使用明确测试topic/service remap或拒绝冲突实例。绝对话题必须实际remap，不能只设置namespace后假设改变。fake不发 /cmd_vel、不订阅真实Nav2/action/传感器/TF；simulation混用硬拒绝。
- [ ] 静态准备按来源PC定向routes：Ranger state/observation由Ranger桥0→40、Go2桥40→0；Go2 state/observation/command由Go2桥0→40、Ranger桥40→0。每话题仅在源PC导出、接收PC导入，不在两桥都启bidirectional；不新增scan/map/odom/TF/Nav2 actions。保持Go2桥Domain0=enp4s0、Domain40=wlo1的实际配置方式；消息类型使用multi_agent_task_msgs。配置格式来自已安装domain_bridge与本机现有文件，不造通用YAML或未经核实的DDS语法。
- [ ] 只做配置语法/类型检查，不启动 bridge、不 ping/SSH/probe Ranger、不跨 Domain 发包。自定义类型在 Ranger 电脑和两侧桥运行环境均需安装/source 同版本。
- [ ] 静态确认本机Domain0导航/相机接口的配置可复用；不额外建立0↔31底层桥、不新增待迁移31↔40生产配置。后续现场验证包含DDS发现隔离与回灌检查，当前仅准备。
- [ ] 跑本机测试并提交。mock 和真实启动说明分开；不得将 fake Ranger 作为 live 完整任务的 READY 来源。

## 8. 最终检查与 Ranger 交接

### Task 8：构建、必要复核与可复现交付

**Files:** `docs/multi_agent/{go2_runbook,ranger_handoff,test_results}.md`；补齐本轮包声明与配置安装规则。

- [ ] 按下方统一输出路径执行colcon list/build/test/test-result，source实际install；显式pytest使用已核实解释器。注册ament测试并核对实际测试数量，新任务层结果与52项旧基线分开；不能把“零测试通过”当通过。
- [ ] 从 git diff 核对 Global Constraints/Review Focus：旧定位入口仍可用、无硬编码真实假坐标/密钥、无新 global planner API、无模拟结果参与 live 屏障、取消结果身份正确、资源失联不自动释放。
- [ ] 在 go2_runbook 写绝对仓库路径、实际 ROS/source 顺序、原成功脚本、mock 与 live 启动/显式 start/暂停命令、必要配置与当前 live gates、回退/停机方式。不要复制不存在的 action/topic 名。
- [ ] 在ranger_handoff写：需检出的commit、multi_agent_task_msgs schema、相同公共包、Ranger端需新增adapters/ranger.py实现Task3接口、独立地图身份、相同alignment_id、Ranger本地Domain0、Ranger桥0↔40的定向半程、build/source顺序与旧agent话题保留。本轮不实现或验证Ranger真实硬件接口。
- [ ] test_results 逐项填写 **实际命令、实际结果、测试模式、未完成原因**；区分纯逻辑、ROS 本机模拟、Go2 真机、跨端通信、双机真机。后面三类未执行就明确未执行，不复用用户既有单机验收充数。
- [ ] 汇总修改/构建/测试结果与剩余现场门槛。完成本轮代码并交付，不以“以后可做”代替公共逻辑/Go2适配/模拟验证；缺真实标定与 Ranger 不应造成这些部分半途结束。

构建和测试均指定同一输出位置，不让测试回到旧工作区默认build。先source实际Humble/既有Go2依赖overlay，并按Task6记录的解释器约定执行；下列为本机命令约定，不表示已在本会话运行：

```bash
colcon list --base-paths /home/isee-pst/unitree_ros2/VLM_Nav /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src
colcon --log-base /tmp/go2-coop/log build \
  --base-paths /home/isee-pst/unitree_ros2/VLM_Nav /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src \
  --build-base /tmp/go2-coop/build --install-base /tmp/go2-coop/install \
  --packages-up-to multi_agent_nav
source /tmp/go2-coop/install/setup.bash
colcon --log-base /tmp/go2-coop/log test \
  --base-paths /home/isee-pst/unitree_ros2/VLM_Nav /home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src \
  --build-base /tmp/go2-coop/build --install-base /tmp/go2-coop/install \
  --packages-select multi_agent_task_msgs multi_agent_nav --return-code-on-test-failure
colcon test-result --test-result-base /tmp/go2-coop/build --all --verbose
```

/tmp输出用于本轮独立构建，清理/重启后可能丢失；运行手册给出重新构建步骤。若部署改为持久输出，build/test/source/result四处一并修改。

## 本轮验收界限

**本轮应完成：** 公共消息和规则 coordinator、FrameAdapter/标定工具、agent 状态机与取消监督、资源监护、Go2真实接口代码适配、双 fake完整流程、ROS本机收发、桥接配置准备与两端交接文档。所有声称通过的项须有实际测试输出。

**现场数据到位后再做：** 对应点采集/真实标定；Go2实际接近与停机验证；Ranger电脑接本机adapter；新自定义消息经Ranger PC 0↔共享40↔Go2 PC 0转发及发现隔离/无回灌检查；两机互为动态障碍；窄门竞争/机身退出；共同C完整模拟取放。跨图导航与联合运动在必要门槛满足前不能开放。

没有实体机械臂；READY/放入保持 Nav2 成功的模拟语义。共同地图转换不改变本机可通行空间；coordinator不得替Ranger用Go2地图判断路线。

## 直接贴给 Go2 电脑 Codex 的启动指令

```text
当前你运行在 Go2 控制电脑。请读取随附 ranger_go2_codex_plan.md 和
go2_codex_implementation_plan.md，按已审查架构开始修改本机代码，顺序完成任务0—8。

先核实真实仓库/分支并读取 VLM_Nav/scripts、AGENTS.md 和现有 Nav2/视觉/桥接接口。
历史路径 /home/isee-pst/unitree_ros2/VLM_Nav、分支 VLM_nav_go 只是线索。
本轮实现公共任务层、Go2 adapter和双fake测试；live coordinator/Go2任务agent在Domain0。
两端本地域均0，桥接为Ranger PC0↔共享40↔Go2 PC0；Go2 Domain0仅enp4s0、Domain40仅wlo1。
保留旧multi_agent_msgs和/multi_agent/agents/*/state；新任务消息包multi_agent_task_msgs，使用独立任务话题。
Domain31仅可作不桥接的mock测试域，不迁移已验证底层ROS；bridge按PC来源/话题单向路由。
两机器人用不同地图，lab_ref 数值等于Go2 map；Ranger变换未标定不得填虚假值。
固定Go2找A、Ranger找B，互发READY，屏障后共同C，Go2先放入退出再Ranger。

不连接或操作Ranger电脑，不尝试跨端通信，不启动桥接链路做测试，不启动真机运动。
准备定向桥接配置与Ranger交接文档即可。保留已有单机成功栈和用户修改。
缺真实标定/场景坐标时继续完成代码与合成测试，live保持启动门槛。
无需为常规实现选择再次询问；若真实接口不符，先给出证据、完成可独立部分并说明缺口。
最终提交实际修改、构建/测试证据、运行命令与尚未执行的现场验收项。
```
