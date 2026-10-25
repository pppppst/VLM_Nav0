# Go2 协作接近流程按单机基线恢复：Codex Implementation Plan

> **For agentic workers:** 使用 superpowers:executing-plans 按任务顺序实施；用户要求是复用指定单机代码，不是重新设计导航。若执行环境没有该技能，按本文顺序执行，不因技能缺失重建方案。不要自动提交、推送、部署或启动真机。
>
> **执行指令：** 请执行本计划限定的代码修改和离线验证。先读取指定 Git 对象及当前 AGENTS.md，再实施。凡涉及运动决策的改变，必须给出单机源函数或本文明确授权的协作接口适配依据。没有依据的新增导航规则不允许进入补丁。

**Goal:** 将 Go2 协作框架内的目标接近、停止、验收及相关执行路径监护，恢复为指定单机版本的行为；保留当前协同任务协议和动态避障配置。

**Architecture:** 从冻结单机版本提取目标接近状态与回调逻辑，通过最小接口适配接入现有 Go2 本地执行器。协作层继续负责任务身份、心跳、取消、READY 屏障和结果汇报；同一时刻只能有一个执行器拥有 Nav2 运动目标。

**Tech Stack:** Python、ROS 2 Humble、Nav2、现有 multi_agent_nav / vlm_nav；不引入新依赖。

**Spec:** 本文第 1—4 节即本计划的完整范围与行为约定。不要把先前讨论过但已被用户否定的切线碰撞算法、末端退让或恢复状态机当作需求。

## 1. 固定 Git 依据

仓库：https://github.com/pppppst/VLM_Nav0

| 用途 | 固定提交 | 分支仅作定位说明 |
| --- | --- | --- |
| 唯一单机行为基线 | `32eab6244ec4b483807f8ed68e8667f5495874bb`，2026-10-06 | `VLM_nav_go` |
| 本轮已审查的协作代码 | `df1a519df222c40f32e5222801d42fe20d8c4c44`，2026-10-25 | `go2-cooperative-task-layer` |

单机源码：
https://github.com/pppppst/VLM_Nav0/blob/32eab6244ec4b483807f8ed68e8667f5495874bb/vlm_nav/vlm_navigator.py

单机参数：
https://github.com/pppppst/VLM_Nav0/blob/32eab6244ec4b483807f8ed68e8667f5495874bb/config/robot.yaml
https://github.com/pppppst/VLM_Nav0/blob/32eab6244ec4b483807f8ed68e8667f5495874bb/config/robot_go2.yaml

约束：
- 用完整 SHA 的 `git show SHA:path` 读取依据，不能用变化中的分支最新文件代替。
- 不能把 10 月 2 日、10 月 20 日或其他版本中的片段任意拼成“单机实现”。
- 当前 HEAD 若已前进，先核对本计划涉及文件相对协作 SHA 的差异。无关改动保留；若同一流程已有实质修改，先说明冲突，不强行覆盖或整分支回退。
- 用户未提交改动、bag、日志、build 输出均保留。本计划不授权 reset、clean、覆盖既有安装或连接另一台机器人。
- 本轮只针对 Go2 协作的本地独立任务模式（`share_c_coordinates=false`）；不改变 Ranger、mock、旧共享坐标模式的行为。

## 2. 全局约束：严格复用，不新增导航规则

### 2.1 允许的改动

1. 将指定单机函数及必需状态迁移为 Go2 协作内部可调用的接近执行逻辑。
2. 删除或绕开与该单机接近流程冲突的协作新增前置否决、终点替换和重复规划。
3. 进行必要的接口适配：协作目标转入本机 map；异步 future/action 回调；task/command/goal 身份；取消所有权；结果汇报。
4. 接通已有日志并补充原流程的判定事实；日志不增加运动条件。
5. 添加验证单机行为等价性和协作接口正确性的测试。

### 2.2 禁止的改动

- 不新增路径切线 yaw 估计、转弯扫掠预检、重新定义 253/254 阈值或 footprint 算法。
- 不新增停车候选、沿路径退让、圆周停靠采样、替代目标选点。
- 不新增 Nav2 恢复状态机、恢复阶段宽限、动态超时或无依据的重试。
- 不调整 NavFn、DWB、速度平滑器、机器人 footprint、膨胀半径、规划容差、到点容差。
- 不修改动态障碍层、三周期确认、点云过滤、传感器、雷达外参、AMCL、地图或 Keepout。
- 不修改探索候选生成/VLM 选点、扫描角度与次数、视觉识别或深度恢复。
- 不重构整个单机节点、双机架构或仓库；不让两个节点同时拥有运动目标。
- 不通过把所有 `path_valid` 永久设为 true、所有 CANCELED 改成 SUCCEEDED 等手段绕过检查。
- 不把单机没有的“Nav2 ABORTED 自动重试”偷偷保留在新接近路径内；按单机对应分支处理。
- 不将本轮离线通过表述为虚假障碍已消除或双机避碰已验收。

### 2.3 保留的协作约束

保留当前任务身份和迟到结果隔离、串行运动所有权、双 READY 屏障、本机 C、心跳失联停止、PAUSE/CANCEL、定位与现有健康门槛。
保留协作外层既有的停稳、motion_terminal、定位有效与距目标 ≤1.00 m 检查，不新增数值。
这些是明确保留的协作约束；交付时不得声称整个系统与单机完全相同。

## 3. 必须恢复的单机行为及出处

以下均来自单机 SHA 下 `vlm_nav/vlm_navigator.py`，以函数名定位，行号不作跨版本依据。

| 行为 | 单机依据 | 协作接入要求 |
| --- | --- | --- |
| 请求目标坐标规划 | `navigation_tick` 的 APPROACHING 分支；`plan_then_navigate`、`request_next_plan` | 使用目标 x/y；yaw 沿用该分支取机器人当前 yaw 的方式；use_start=False。目标已经进入停止触发范围时，沿用原直接停止验收分支 |
| 规划结果处理 | `on_plan_response`、`on_plan_result` 的 kind=="approach" 分支 | action 成功且路径非空后，向原请求目标导航；保留原分支的失败处理和 token 隔离，不以完整路径矩形检查替代 |
| 发送导航 | `send_navigation_goal`、`on_goal_response` | 不把 ComputePath 返回终点替换成导航目标；适配现有 action 所有权，不重复发送第二套动作 |
| 途中测距 | `safety_tick`、`navigation_tick`、`target_reference_distance`、`approach_stop_radius` | 沿用原比较符、首次/重试阈值和相关调度语义，不根据路径终点距离预判成功 |
| 取消与停稳 | `begin_approach_stop`、`request_approach_cancel`、`on_approach_cancel_response`、`advance_approach_stop`、`on_arrival_odom` | 停止指令、动作终态、里程计新鲜度、连续静止样本、超时分别按源码迁移；取消请求被接受不等于动作已终止 |
| 结果与有限重试 | `on_navigation_result`、`advance_approach_stop`、`finish_approach_success`、`reset_approach_stop` | 只复用原有触发原因和次数；动作异常、距离未达标、外部取消不能混为一类 |
| 执行路径消息 | 单机接近执行没有协作版 /received_global_plan 的首条/5秒接收门槛 | 在本轮 Go2 本地执行模式内，去除这些额外停止触发，保留路径显示与记录；不增加恢复识别机制 |

必须记录“单机函数/分支 → 新位置 → 必需接口变化”的对应表。迁移后每一条决定发送、停止、成功或重试的条件，都能从上表源码或 2.3 节找到出处。

### 3.1 本轮采用的单机参数

接近执行器读取并验证下列冻结基线有效值；优先复用现有参数加载，不散落硬编码。若现场有效值不同，报告差异，不静默改成猜测值。

| 参数 | 基线值 |
| --- | --- |
| target_success_radius | 0.81 m |
| approach_cancel_radius | 0.89 m；重试时的停止触发按 approach_stop_radius 原分支取 0.81 m |
| arrival_retry_limit | 1 |
| arrival_linear_speed_tolerance | 0.03 m/s |
| arrival_angular_speed_tolerance | 0.05 rad/s |
| arrival_stationary_samples | 3 |
| arrival_odom_max_age | 0.5 s |
| arrival_cancel_timeout | 3.0 s |
| arrival_stop_timeout | 5.0 s |
| 到达里程计话题 | 合并 Go2 profile 后为 /odometry；不要误抄 robot.yaml 的通用 /fastlio/odom |

严格复制 `on_arrival_odom` 中的速度分量、计数重置与时间判定方式。协作原有 health.stopped 与该单机内部停稳判定分开，不能用布尔量互相冒充；外层既有条件未满足时不得置 READY。

协作外层 ≤1.00 m 不覆盖单机内部 0.81 m，不再使用“路径终点距离 + 0.25 + 0.03”的新增接近前置预算。
既有 Nav2 xy_goal_tolerance=0.25 等参数保持不变。

## 4. 文件范围与接口约束

生产文件允许范围（均相对仓库根目录）：
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_exploration.py`：仅替换 Go2 协作目标接近分支；探索、扫描分支保持。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/go2.py`：复用动作调用、补齐单机停稳所需输入、记录动作身份；移除本轮模式的接近预检调用，不修改被探索复用的几何函数和阈值。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/runner.py`：最小接线、执行器生命周期、事件记录、避免接近重复规划及错误重试。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py`：本轮 Go2 模式的成功证据和路径接收监护适配；其余机器人及模式保持。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent_node.py`：仅必要模式接线或日志接线。
- 可新增一个内部模块 `cooperative_ws/src/multi_agent_nav/multi_agent_nav/single_robot_approach.py`，只承载从冻结单机提取的接近状态和回调，不新增算法。

测试：
- 新增 `cooperative_ws/src/multi_agent_nav/tests/test_single_robot_approach_parity.py`。
- 按实际受影响分支更新现有 `test_target_nav2_endpoint.py`、`test_local_tasks.py`、`test_local_c.py`、`test_go2_executing_path.py`、`test_runner.py`，其余按调用关系选择。
- 原有端点替换/强制 Nav2 SUCCEEDED 测试应按明确改变的合同更新；不能直接删除失败测试或改成无条件通过。

文档：
- `docs/multi_agent/session_handoff.md`：本轮事实与未验证项。
- `docs/multi_agent/go2_runbook.md`：实际离线命令和环境。
- 本计划与最终来源对应表、差异说明。

保护文件：原单机 `vlm_nav/vlm_navigator.py`、所有导航/障碍/地图/传感器 YAML、coordinator、消息定义、Ranger 代码、默认安装环境入口，本轮不修改。
若必需接口缺口涉及白名单外文件，先证明具体阻塞并列出最小变更，不借此扩大功能。

接口要求：
- 继续使用当前任务身份、`Pose2D`、`PathResult` 和 native action 结果；不修改跨机协议。
- 规划目标和导航目标的 x/y/yaw 必须与单机同输入轨迹一致。
- 单机执行器内部成功经当前身份校验后，转换成已有 `TARGET_APPROACHED` 结果。
- `local_approach_succeeded` 只可由经核验的单机接近完成证据置真，不再仅由一次 Nav2 SUCCEEDED 置真，也不能因任意 CANCELED 置真。
- 距离触发的正常停止仅取消本执行器拥有的 goal；不能调用会把整个协作任务永久 PAUSED 的外部取消路径。
- 协作 PAUSE、失联、任务替换、旧 goal 回调，永远优先阻断成功汇报。

## 5. 顺序执行任务

### Task 1：冻结现场代码状态，建立行为映射

- [ ] 在实际 Go2 仓库执行只读核对；不默认硬编码的路径一定存在。

```bash
pwd
git remote -v
git branch --show-current
git rev-parse HEAD
git status --short
git cat-file -e 32eab6244ec4b483807f8ed68e8667f5495874bb^{commit}
git cat-file -e df1a519df222c40f32e5222801d42fe20d8c4c44^{commit}
git merge-base --is-ancestor 32eab6244ec4b483807f8ed68e8667f5495874bb df1a519df222c40f32e5222801d42fe20d8c4c44
```

目的：确认对象、来源、当前工作区及继承关系。成功判据：对象存在且 ancestry 检查退出 0；当前 HEAD/未提交状态完整记录。
对象不存在时，只从核实过的该仓库 remote 获取必要对象；不要自动切换或重置现有分支。

- [ ] 阅读当前 AGENTS.md 和交接；用 `git show` 阅读第 3 节源函数、Go2 参数覆盖及相关测试，不靠本文摘要重写代码。
- [ ] 对照当前调用链，定位重复规划、runner ABORTED 重试、局部执行结束与 READY 的入口；生成来源对应表。
- [ ] 在不影响现有进程和工作区的独立工作副本实施；若存在相关未提交改动，明确保留和纳入方式。
- [ ] 按 runbook 的实际 Python/ROS 环境先运行相关原测试，记录已有失败。计划不规定未知虚拟环境路径，不初始化 Domain0。

### Task 2：先建立单机行为差分测试，再提取执行逻辑

**输入：** 固定 SHA 单机源码、同一组机器人位姿/里程计/action 事件和可控时钟。
**输出：** 可对比的规划请求、导航请求、取消、停止、成功/失败及重试序列。

- [ ] 在 test_single_robot_approach_parity.py 中，以冻结源码的真实方法/分支构造参考执行器；可 mock ROS 消息、action 和时钟，不能手写另一套“期望单机算法”充当参考。
- [ ] 覆盖表中案例，确认当前协作实现在哪些语义上不一致；保存基线输出。
- [ ] 提取原单机必要状态与方法，可机械调整容器、回调和上下文接口。禁止改变条件先后、比较符、计数、重试原因或阈值。
- [ ] 在相同输入序列下比较参考与提取实现的决策序列，要求一致。若只能做子流程差分，明确覆盖范围，不能冒充完整等价测试。

| 案例 | 必须验证 |
| --- | --- |
| Nav2 返回终点偏离请求目标 | 导航仍使用原目标，未替换成返回终点 |
| 规划失败、拒绝、空路径 | 与单机失败分支一致，不制造停车候选 |
| 当前距离已进入停止触发范围 | 与单机直接进入停止验收分支一致，不强行要求完成一次新导航 |
| 首次距离 0.89 m 边界两侧 | 原 <= 比较与停止时机一致 |
| cancel 已应答但 action 未终止 | 不宣布停稳或成功；按原终态等待超时处理 |
| 3 个静止样本，中途有运动样本 | 原计数/重置逻辑一致 |
| 里程计过期 | 不通过单机停稳验收 |
| 停稳距离 0.81 m 边界两侧 | 成功/重试分支与单机一致 |
| 第一次停稳超距、再次超距 | 最多一次原有距离重试，随后失败 |
| Nav2 正常 SUCCEEDED | 仍进入原停止和距离验收，不立即 READY |
| Nav2 ABORTED 或停止中异常终态 | 与单机失败分支一致，不叠加 runner 的额外接近重试 |

### Task 3：接入 Go2 协作，移除冲突的额外接近规则

- [ ] 先写集成失败测试，覆盖下面四个要求。
- [ ] 在仅 Go2 本地协作目标接近分支调用提取逻辑；A 和本机 C 共用同一流程。
- [ ] 删除/绕开该分支的完整路径 footprint 否决、返回终点替换、终点距离预算、圆周 select_approach 和重复规划；共享函数若探索仍使用，不全局删除。
- [ ] 适配 runner 的动作回调，保证同一 goal 只有一个结果消费者；取消与 action 迟到结果遵循原有身份隔离。
- [ ] 接近完成由单机证据驱动，外层既有健康/停稳/≤1m门槛仍需满足；尚未满足时不能提前完成或误用旧证据。
- [ ] 更新相关测试并执行，验收：
  1. 默认零 yaw 或路径覆盖 253 不再单独触发新增接近拒绝；只是消除额外否决，不宣称该路径物理安全。
  2. Nav2 自身失败仍正确失败；动态 costmap、Keepout 保持加载。
  3. 正常距离触发取消后，原单机验收通过可以完成本机接近。
  4. 外部取消、失联、任务替换、旧结果、错误目标均不能置 READY/deposited；另一台未 READY 时屏障不释放。
- [ ] 确认未将单机执行器与旧运动状态机同时启用。

### Task 4：去除本轮 Go2 模式的额外执行路径消息停止门槛

本项按已讨论范围恢复单机行为：作用于 Go2 本地协作执行期间的 Nav2 导航，包括走向观察点及接近目标；不改变观察点生成或扫描。其他机器人/模式不变。

- [ ] 先添加模式隔离测试：本轮模式、Ranger、旧共享模式分别验证。
- [ ] 排除首条执行路径接收门槛，以及已接收路径 5 秒无更新导致停止的额外条件；同时检查 agent._monitor_path 与 explore_steps.health_check，避免只改一层仍被另一层拦截。
- [ ] 保留定位/健康/心跳/任务超时、action 失败和取消逻辑。不能通过伪造 path_valid 放行所有模式。
- [ ] 不读取恢复阶段来创造新条件；正常 Nav2 action 生命周期按单机处理。
- [ ] 测试：当前 action 正常、心跳和健康新鲜，仅 /received_global_plan 超过 5 秒未刷新，不再因该项自动暂停；相同场景中定位失效、失联或外部 CANCEL 仍停止。
- [ ] 路径订阅、显示、接收时间诊断可保留；不改 path_receive_timeout 为超大值掩盖原逻辑。

### Task 5：接通被动日志与失败归因

- [ ] 将 adapter.record_event 接到当前任务记录目录；执行结束或替换时处理记录器生命周期，旧事件保留原 task/command/goal 身份。
- [ ] 记录原始目标、实际规划终点（仅诊断）、实际发出的导航目标、原生 action status、距离取消原因、终态等待、静止样本数、里程计年龄、最终距离、重试次数与失败原因。
- [ ] 日志可以记录路径接收超时，但不能因此再引入停止规则。
- [ ] 不打印密钥，不新增常驻相机/点云订阅，不周期性落盘整幅 costmap。
- [ ] 验证日志开/关时，同一输入序列的运动决策一致。接线不得把新增诊断写入失败变成新的导航失败条件；日志异常要保留可见告警并保证取消/停止调用不受阻。
- [ ] 确认失败来自原 Nav2、原单机停止验收还是保留的协作门槛，而不是统一写成“不可达”。

### Task 6：回归、自审与交付

- [ ] 运行 Task 1 已记录的相关基线测试及新增差分/集成测试。按原 runbook 使用真实解释器、ROS underlay、PYTHONPATH 和同一套构建输出。
- [ ] 典型测试调用形式（在实施副本根目录，以已核实环境执行；不得把占位环境当成已存在）：

```bash
python -m pytest   cooperative_ws/src/multi_agent_nav/tests/test_single_robot_approach_parity.py   cooperative_ws/src/multi_agent_nav/tests/test_target_nav2_endpoint.py   cooperative_ws/src/multi_agent_nav/tests/test_local_tasks.py   cooperative_ws/src/multi_agent_nav/tests/test_local_c.py   cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py   cooperative_ws/src/multi_agent_nav/tests/test_runner.py
git diff --check
git diff --name-only
git diff --stat
```

目的：验证行为等价、集成及文件边界。成功判据：新增/修改测试通过，旧失败逐项独立报告，不用随意 deselect 制造全绿；diff 无空白错误且只涉及授权范围。

- [ ] 若需要构建验证，按 runbook 使用全新隔离输出；不要覆盖已有安装，不修改默认 COOP_SETUP_FILE、不切换运行中的组件。Domain31 无硬件验证必须确认隔离及没有硬件启动入口。
- [ ] 对照受保护文件与实施前工作区快照，确认本轮没有修改动态层、Nav2 参数、地图、单机节点、Ranger 和 coordinator。不能把用户原有未提交 diff 算作本轮变更。
- [ ] 自审每条运动决策：能否在固定单机源码或保留的协作约束中找到依据？找不到就删除或明确提出范围冲突，不能以“更安全/更合理”自行保留。
- [ ] 不自动提交/推送，不 ARM、真机运动、录包、SSH/Ranger、重启或停止现有现场组件。

## 6. Review Focus 与最终验收

重点复核：
1. 受控距离取消与外部取消混淆：只有前者经完整原单机验收才可能成功（Task 3）。
2. 单机 0.81m 被协作 1.00m 无意覆盖：内部严格沿用单机；外层检查独立（Task 2/3）。
3. 动作终态不等于停稳、取消应答不等于终态：等待顺序、超时与计数一致（Task 2）。
4. agent、runner、explore 多层消费同一结果或残留门槛：无重复目标、无额外 ABORTED 重试、无隐藏路径超时停止（Task 3/4）。
5. 模式串扰：Go2 本地接近改动不得改变 Ranger、扫描、候选合法性或其他模式（Task 3/4）。

交付必须包括：
- 实际起始 HEAD、固定单机 SHA、工作区保护情况。
- 每项生产改动对应的“单机函数/分支 → 新位置”，并单列纯接口适配。
- 明确删除的协作新增规则。
- 明确保留的协作差异，不宣称整个系统完全等价。
- 基线测试、新增测试、失败项、构建/隔离测试的实际命令与结果。
- 尚未部署、尚未真机验收的事实，以及本轮未处理的动态障碍问题。
- 不得将“移除误拒绝”表述成“虚假障碍已修复”或“机器人一定能到达”。

## 7. 交给 Codex 的最后约束

请严格执行以上固定版本迁移。优先搬运和复用单机已存在的状态与分支，最小适配协作接口；不要根据聊天摘要重新发明接近算法。遇到源代码与本文描述冲突，以核实过的固定单机源码为行为依据，列出冲突后仅暂停相关歧义项，继续独立的只读核查。任何原单机没有、本文也未明确保留的运动规则，都不属于本轮授权。

