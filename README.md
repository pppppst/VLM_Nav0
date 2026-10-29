# VLM_Nav

基于 ROS 2 Humble 的视觉语言导航与 Go2 / Ranger 双机器人协同任务系统。
每台机器人在自己的地图上完成感知、定位、探索与 Nav2 导航；Go2 控制电脑上的
规则协调器负责任务分配、双方就绪屏障和执行监护。VLM 负责识别目标和选择本地
探索候选，路径规划、避障与运动执行由本机导航栈完成，VLM 不直接产生速度命令。

仓库同时保留原 Ranger 单机默认 profile、Go2 单机入口和协同任务层。Livox / Hesai、
FAST-LIO、RealSense 和底盘驱动依赖各机器人已有工作空间，不在协同层重新实现。

> 本文按 **2026-10-29 当前工作树**整理。当前任务采用独立 C 模式；早期方案中的
> 共享坐标、门/C 许可和顺序退出不属于当前执行流程。代码实现、安装版本和现场
> 验收是不同状态，最新证据与待办见[会话交接](docs/multi_agent/session_handoff.md)。

## 从哪里开始

| 需要做什么 | 入口文档 |
|---|---|
| 了解当前状态、安装位置和待解决问题 | [会话交接](docs/multi_agent/session_handoff.md)，优先读最新日期章节 |
| 启动双机任务、开始下一轮、暂停和关闭 | [双机协同启动与验收](docs/双机协同启动与验收.md) |
| 构建、选择环境、离线检查、mock 和 Go2 单机实验 | [Go2 runbook](docs/multi_agent/go2_runbook.md) |
| 核对双方准备与启动握手 | [异步启动契约](docs/multi_agent/ans_async_startup.md) |
| 在 Ranger 电脑接入公共任务层 | [Ranger 交接](docs/multi_agent/ranger_handoff.md) |
| 查看原 Go2 单机流程及移植背景 | [单机启动指引](docs/VLM_Nav初步启动指引.md)、[Go2 移植说明](docs/GO2_PORTING.md) |
| 追溯最初架构及实现计划 | [架构方案](docs/superpowers/plans/ranger_go2_codex_plan.md)、[执行方案](docs/superpowers/plans/2026-10-07-go2-cooperative-navigation.md)，其中旧任务语义须结合最新交接阅读 |

## 当前协同架构

```text
Go2 控制电脑 · 本地 Domain 0              Ranger 控制电脑 · 本地 Domain 0
┌──────────────────────────┐           ┌──────────────────────────┐
│ Coordinator / RulePlanner│           │ Ranger task agent        │
│ Go2 task agent           │           │ Ranger 本机执行适配       │
│ 本地 VLM / RGB-D / TF    │           │ 本地 VLM / RGB-D / TF    │
│ 独立地图 / 定位 / Nav2    │           │ 独立地图 / 定位 / Nav2    │
│ Go2 Sport 速度转换与门控   │           │ Ranger 底盘驱动           │
└────────────┬─────────────┘           └────────────┬─────────────┘
             │ 定向 domain_bridge                  │ 定向 domain_bridge
             └────────── 共享 Domain 40 ───────────┘
                         state / observation / command
```

- **任务协调：**`Coordinator` 与 `RulePlanner` 部署在 Go2 控制电脑的本地域，
  根据任务身份、状态回执和 READY 屏障派工，不调用全局 VLM planner。
- **本地执行：**`AgentMachine` 管理任务状态，`AgentRunner` 调用机器人 adapter，
  完成本机搜索、接近、取消和停稳检查。Go2 单机与协同复用探索及接近执行器。
- **独立地图：**两端不共享 occupancy、TF、costmap 或导航 action；目标坐标和
  可达性在本机处理。当前配置 `share_c_coordinates: false`，跨机状态/观察不携带
  可用公共坐标，也不以 `lab_ref` 对齐作为当前模式的执行前提。
- **硬件边界：**本仓库提供 Go2 live adapter 和双 fake 逻辑验证；Ranger live adapter
  在 Ranger 电脑独立接入。公共包或本机构建成功不代表远端已同步实现。

### 任务流程与完成判据

当前场景见 [scene.yaml](cooperative_ws/src/multi_agent_nav/config/scene.yaml)：
Go2 找 A、Ranger 找 B；当前 A/B 均配置为“灭火器”，C 为“白色的桶”。场内约定一个 C，
两端分别定位并保存自己的坐标。

```text
各自 PREPARING → WAITING_PEER / STANDBY
    → StartTask 受理，建立本轮 task_id
    → SYNCING → 双方 run_state=SYNCED 且 reply=SYNCED
    → SEARCH_AB：Go2 搜索/接近 A，Ranger 搜索/接近 B
    → 双方 READY，且当前命令完成
    → SEARCH_C / DEPOSIT：各自搜索或复用本机 C，再接近并模拟放入
    → 双方 deposited + stopped，当前命令确认结束 → DONE
```

READY 只表示自身 A/B 主任务通过到达验收，不能由观察点到达、扫描成功或通信连通替代。
任务层还检查动作终态、定位/导航健康、停稳和目标距离（上限 1.00 m）；Go2 本地接近
执行器使用更严格的 0.81 m 停稳后距离验收。取消动作必须等待原生终态并确认停稳。

C 阶段允许一方接近、另一方继续搜索，也允许两端同时接近。当前流程不申请门/C 资源，
不发放许可，不要求 Go2 先行或执行 EXIT；完成后停在接近位置。
`deposited` 表示模拟放入，不代表机械抓取或实际投放。互相避障依赖各自感知和 Nav2，
当前没有任务层两机互斥，现场效果须独立验收。

当前配置为 2 Hz 心跳、3 秒失联门槛、从 start 受理起 600 秒整轮预算；消息新鲜度按
接收端单调时钟判断。执行期失联或健康失效先触发停车。最新现场交接记录的版本会锁存
PAUSED；整理时工作树已出现导航/心跳异常的可恢复暂停逻辑，恢复需满足同任务、新鲜
同步、双方健康停稳等条件，人工暂停、定位失效等仍锁存。该新增逻辑的安装状态、
双端兼容性及现场效果本次未验证，不能凭心跳恢复自行认定任务已续跑。

Go2 收到本轮 `SYNC(DONE)` 且本机 C 已完成、停稳和动作终态成立后，保留
`IDLE/SUCCEEDED` 及 `ready/deposited`；后续健康异常另行报告，不改写已完成结果。
DONE 不自动关闭进程或 DISARM，远端完成锁定行为仍须核对其实际版本。

## 本地感知与导航

Go2 当前硬件链为 Hesai XT16 + 机身 IMU → 点云转换 / FAST-LIO → 障碍过滤与 LaserScan
→ 独立地图 / AMCL / Nav2；D435 RGB-D 经相机模型、实测外参和图像时刻 TF 提供目标位置。
本地 Nav2 速度通过独立的 Sport bridge 门控后送往 Go2；它与跨机任务 domain_bridge 是两个组件。

当前 Go2 搜索使用 `generated_candidates_vlm`：

1. 停稳后先识别当前朝向的新鲜图像，同时检查 A/B/C；当前目标位置有效即可接近。
2. 未确认目标时，从本机地图已知空地生成候选，家具语义标注可选。VLM 根据占据图和
   语义图选择候选编号，当前选点请求暂不附带相机 RGB；Nav2 保留可达性否决权。
3. 远处候选按全局地图和规划判断；到达并停稳后，才用局部 costmap 检查整圈旋转空间。
   覆盖不足或未知区域等待更新，确认障碍则跳过该点，数据健康故障仍停车处理。
4. 环扫保留 8 个方向，同任务奇数轮顺时针、偶数轮逆时针；检出主目标可提前结束。
   仅完整环扫后，才排除实际停靠位置 2.5 m 内无遮挡的重复候选，不将墙后区域算作已观察。
5. 接近后取消/停稳并复核距离；动作已结束且停稳、一次 retry 后仍未达到 0.81 m 时，
   放弃该次目标结果并继续观察，不置 READY 或 deposited。取消失败和真实健康故障不按此恢复。

相机临时缺帧、RGB-D 同步或图像时刻相机 TF 未就绪时，在观察位置停稳等待；新鲜有效
图像恢复后继续，不用旧图像推进任务。相机等待仍计入整轮 600 秒预算。
具体参数以 [Go2 任务配置](cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml)
及其加载的根包配置为准。

## 通信协议与网络

协同任务使用独立的 [multi_agent_task_msgs](cooperative_ws/src/multi_agent_task_msgs/)：

| 接口 | 类型与用途 |
|---|---|
| `/multi_agent/{go2,ranger}/state` | `AgentState`：任务身份、执行回执、READY/deposited、健康与停稳 |
| `/multi_agent/{go2,ranger}/observation` | `Observation`：识别结果与本机缓存引用；当前模式不发布有效公共位置 |
| `/multi_agent/go2/command` | `Command`：协调器统一命令流，按 `recipient_id` 分发给两端 |
| `/multi_agent/coordinator/status` | `std_msgs/String`：Go2 本地协调阶段和原因 |
| `/multi_agent/coordinator/start` | `StartTask`：受理场景，返回 `accepted/task_id/reason` |
| `/multi_agent/coordinator/pause` | `std_srvs/Trigger`：请求暂停 |

协议校验 schema、地图身份/版本、`task_id`、`command_id`、sequence 和 goal 身份，
隔离重复命令及过期回调。当前模式保留旧公共坐标/资源字段以兼容消息定义，但清空或置无效。

两端本地 ROS 均使用 Domain 0，经各自定向 bridge 接共享 Domain 40。Go2 的本地感知、
定位、Nav2 和 agent 仅走 `enp4s0`，Domain 40 仅走 `wlo1`。桥接五个任务话题，
QoS 为 Reliable / Volatile，不转发 TF、图像、点云、costmap、速度或 action。
Domain 31 仅用于隔离 mock/回放，不接现场 bridge。

路由配置：[Go2](cooperative_ws/src/multi_agent_nav/config/bridge_go2_pending.yaml)、
[Ranger](cooperative_ws/src/multi_agent_nav/config/bridge_ranger_pending.yaml)；Go2 bridge 必须使用
[按域绑定网卡的 DDS 配置](config/cyclonedds_go2_multi_agent_bridge.xml)，不能替换成本地域 XML。
运行前核实实际网卡/IP 和双向新鲜消息；bridge 进程存在不证明 DDS 数据已转发。

旧 `multi_agent_msgs` 和 `/multi_agent/agents/{go2,ranger}/state` 保留给原里程计消费者。
旧 `go2_agent_node` 只转发已有 `/odometry`，不启动传感器/FAST-LIO，其 state 不代表
任务 READY 或定位验收。话题存在或只有订阅者也不证明输入连通。

## 环境、构建与启动

根目录是 `vlm_nav` 包，协同源码位于 `cooperative_ws/src`，构建必须覆盖两处。
完整依赖、Python/ROS 环境、构建和测试命令统一维护在
[runbook](docs/multi_agent/go2_runbook.md)，使用同一输出完成 build/test/source/result，
不要最后 source 旧 overlay 覆盖当前包。

截至本次整理，`Cooperative_env.bash` 和 `CoVLM_env.bash` 默认选择
`build/go2-local-scan-20261029/overlay/install/local_setup.bash`；显式 `COOP_SETUP_FILE`
优先，`COOP_CONFIG` / `GO2_CONFIG` 指向本次配置。这是本机已有安装位置，不是新克隆仓库
自带产物。换电脑须按 runbook 构建并配置本机路径；已运行进程不会随源码或环境更新。

API 凭据仅通过环境提供。当前入口检查 `DASHSCOPE_API_KEY`、端点
（`DASHSCOPE_BASE_URL` 或 `DASHSCOPE_WORKSPACE_ID`）与 `DASHSCOPE_MODEL`，
兼容 `OPENAI_API_KEY` / `OPENAI_BASE_URL` / `OPENAI_MODEL` 回退，不将密钥写入仓库。

以下是入口索引，具体命令、终端、前置条件和成功判据按
[正式启动文档](docs/双机协同启动与验收.md)逐步执行，完整流程与分步流程不要叠加：

| 使用场景 | 入口及行为 |
|---|---|
| 双机完整启动 | [start_go2_cooperative.sh](scripts/start_go2_cooperative.sh)：本机预检/硬件准备、任务通信、等待对端、协调器和一次 start；**会自动 ARM 并发起真实任务** |
| 仅准备 Go2 硬件 | [start_go2_cooperative_hardware.sh](scripts/start_go2_cooperative_hardware.sh)：保留人工确认，成功后 Sport 仍 DISARMED |
| 分步准备、开始下一轮、暂停/解除使能 | [start_go2_cooperative_task.sh](scripts/start_go2_cooperative_task.sh)：`prepare`、`start`、`restart`、`pause`、`disarm` 为不同操作 |
| Go2 独立 VLM 导航 | [start_go2_vlm.sh](scripts/start_go2_vlm.sh)：不等待 Ranger，复用本地执行器，保留人工确认；先退出协同任务组件并确认停稳/DISARM |
| 离线预检和任务组件入口 | [cooperative.sh](scripts/cooperative.sh)：`preflight`、`bridge`、`agent`、`coordinator`、`state`、`peer`、`watch` 等 |

完整入口要求 Go2 控制电脑图形桌面、交互式 Bash 和 `gnome-terminal`，组件窗口须保持运行。
Go2 上位机相机、雷达/IMU、定位/Nav2 与 Sport 是本地执行前提；Ranger 由其操作者在本机
准备，Go2 入口不会远程启动 Ranger。AMCL 初始位姿须来自现场，不能填示例坐标。

`accepted=true` 仅表示 start 已受理，随后还须核对本轮双方 SYNCED、READY 和 DONE。
关闭 watch、主入口或显示窗口不会暂停任务。停止时按正式文档依次请求 pause、确认实体
停稳和原生动作终态、正常结束任务组件，再 DISARM 和关闭硬件；不以消息中的 stopped
替代现场确认。

### 保留的 Ranger 默认 profile 与原单机流程

根包默认仍使用 [config/robot.yaml](config/robot.yaml) 和
[vlm_navigation.launch.py](launch/vlm_navigation.launch.py)，保留 Ranger 的
RealSense 对齐 RGB-D、Livox / FAST-LIO、障碍过滤与 SLAM / Nav2 流程。
原入口包括 [start_hardware.sh](scripts/start_hardware.sh)、
[start_navigation_validation.sh](scripts/start_navigation_validation.sh)、
[check_system.sh](scripts/check_system.sh)、[arm.sh](scripts/arm.sh) 和 [stop.sh](scripts/stop.sh)。
这套入口的外参、传感器和使能流程不直接套用 Go2 协同配置。

Go2 旧 [VLMNav-go.sh](scripts/VLMNav-go.sh) 与 [manualnav2.sh](scripts/manualnav2.sh)
保留原成功栈，其预检、SSH、录包和运动流程不能当作纯只读诊断。原单机细节见
[单机启动指引](docs/VLM_Nav初步启动指引.md)，当前协同及共用执行器的操作以正式启动文档为准。

## 代码、记录与验证边界

| 路径 | 内容 |
|---|---|
| [src/](src/)、[vlm_nav/](vlm_nav/) | 根包 C++/Python：传感器适配、障碍处理、VLM/RGB-D、Go2 监督与底层集成 |
| [launch/](launch/)、[config/](config/) | 单机及 Go2 系统入口、Nav2/相机/标定/DDS 配置 |
| [multi_agent_nav](cooperative_ws/src/multi_agent_nav/) | 协调器、规则规划、agent/runner、Go2 adapter、探索/接近、配置与测试 |
| [multi_agent_task_msgs](cooperative_ws/src/multi_agent_task_msgs/) | 三类任务消息及 StartTask 服务 |
| [scripts/](scripts/) | 构建检查、启动编排、现场检查及离线分析工具 |
| [maps/](maps/) | 地图、Keepout 与可选语义标注；地图身份和路径须与本机配置一致 |
| [test/](test/) | 根包测试；协同测试在 `cooperative_ws/src/multi_agent_nav/tests/` |

Go2 RViz 配置为 [go2_exploration.rviz](config/go2_exploration.rviz)，显示候选点、VLM 选点、
已定位 A/B/C 和导航路径；“Go2 当前状态”窗口显示 agent/coordinator 状态、完整原因及消息新鲜度。

启动日志位于 `build/go2-field/cooperative-task.<随机标识>/`；本地 VLM 实验记录默认位于
`build/go2-vlm-experiments/`，包含事件记录和 `views/<序号>/input.jpg`、`result.png`、
`response.txt`。这些 build 目录不受 Git 保护；受管理实验目录默认滚动保留最近 10 次，
重要证据应另行归档，详细保留规则见 runbook。
不默认增加完整点云/RGB 诊断订阅或录包。

验证分为源码/配置检查、离线单元测试、Domain 31 fake/ROS 验证、本机真机和跨端现场验收。
`--offline` 配置检查不初始化 ROS，默认退出 0 仅表示报告输出；默认 inspect 和无硬件 ROS
测试可能初始化 ROS，不能混同为纯离线。使用条件和退出判据见 runbook。

本机已有构建与离线验证记录，详见交接和 runbook；它们不证明当前版本双机现场任务通过。
最新交接仍有无线改址后 DDS 绑定失配、对端校时引发健康短暂失效及恢复策略待办。
真实现场须核对版本、双向任务消息、目标接近、互相避障、失联停车和完整 DONE，不能仅凭
旧 PASS、TF 新鲜或 `localization_valid=true` 宣布验收完成。
