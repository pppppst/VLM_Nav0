# 项目协作指引

## 沟通与工作方式

- 默认用中文说明关键判断和修改原因，区分设计、实现、已验证事实、推测和待验证事项。
- 目标清楚时自主推进；影响目标、范围或现场操作的歧义再询问，不重复请求已有授权。
- 开始任务核对分支、HEAD、`git status --short`和适用指引；用`rg`按任务查找实际代码。
- 保留用户未提交修改、日志和bag，不执行reset/clean、整分支回退或无关重构。
- 排障先收集证据、追踪调用及配置加载顺序，再提出假设和最小修复，不猜测性改参数或增加无必要依赖。

## 目录与按任务读取的资料

仓库根目录：`/home/isee-pst/unitree_ros2/VLM_Nav`。先读会话交接，其余按任务读取，
不要求每轮通读所有资料。根包为vlm_nav；协同源码在cooperative_ws/src，构建须包含两处。

| 任务 | 真实路径与用途 |
|---|---|
| 接续工作、当前状态和待办 | [docs/multi_agent/session_handoff.md](docs/multi_agent/session_handoff.md) |
| 协同架构、协议及实现边界 | [docs/superpowers/plans/ranger_go2_codex_plan.md](docs/superpowers/plans/ranger_go2_codex_plan.md)、[docs/superpowers/plans/2026-10-07-go2-cooperative-navigation.md](docs/superpowers/plans/2026-10-07-go2-cooperative-navigation.md)；计划不是完成证据 |
| 构建、环境、mock/live及离线检查 | [docs/multi_agent/go2_runbook.md](docs/multi_agent/go2_runbook.md)、[docs/multi_agent/go2_field_checklist.md](docs/multi_agent/go2_field_checklist.md) |
| 动态costmap及退出排障 | [docs/multi_agent/dynamic_costmap_stage.md](docs/multi_agent/dynamic_costmap_stage.md)、[docs/multi_agent/costmap_shutdown_handoff_20261008.md](docs/multi_agent/costmap_shutdown_handoff_20261008.md) |
| 原单机入口、Ranger接入 | [docs/VLM_Nav初步启动指引.md](docs/VLM_Nav初步启动指引.md)、[docs/multi_agent/ranger_handoff.md](docs/multi_agent/ranger_handoff.md)；[README.md](README.md)描述Ranger默认profile |
| 底层及单机代码 | [src/](src/)、[vlm_nav/](vlm_nav/)、[launch/](launch/)、[config/](config/)、[scripts/](scripts/) |
| 协同接口、代码及测试 | [cooperative_ws/src/multi_agent_task_msgs/](cooperative_ws/src/multi_agent_task_msgs/)、[cooperative_ws/src/multi_agent_nav/](cooperative_ws/src/multi_agent_nav/)、[test/](test/) |

## 架构和授权边界

- 保留Ranger默认profile和已有单机成功栈；Go2改动放在对应profile/adapter，不顺手重写传感器、AMCL或底层Nav2。
- 遵循权威架构的独立地图、实测对齐、任务身份、READY屏障和资源退出语义；不编造真实坐标，不用fake Ranger解锁真实流程。
- 保留旧消息和里程计消费者；协同任务使用独立协议/话题。RulePlanner及Domain/网卡/定向桥接约定见架构方案，不自行改部署或新增global VLM planner。
- 两端本地ROS均Domain0，经各自定向bridge接共享Domain40。Go2本地相机/雷达/IMU/定位/Nav2及agent只走enp4s0；Domain40仅高层通信、只走wlo1。bridge使用[config/cyclonedds_go2_multi_agent_bridge.xml](config/cyclonedds_go2_multi_agent_bridge.xml)按域绑定网卡，不用仅供本地域的DDS配置替代；Domain31仅隔离mock/回放。当前IP及用户联调观察见交接，实际网卡/地址运行前核实。
- 旧go2_agent_node只转发已有`/odometry`，不启动雷达/IMU/FAST-LIO；网页端或跨机联调步骤须先列本地输入启动前置条件，确认真实发布者、新鲜数据及QoS兼容，再检查agent和桥接。话题存在或仅有订阅者不证明数据连通；旧state转发通过也不代表新任务协同通过，核对过程仍受现场授权约束。
- 同一任务已有明确授权在未撤销且范围不变时跨轮延续，最新限制优先；次数、时长仍有效，已结束的一次性实验不授权新实验。指引、命令、历史实验和旧PASS不产生新授权。
- SSH、Ranger连接、桥接、录包和真机运动均须在已有明确授权范围内；现场通过还须满足真实门槛，不把静止链路、TF新鲜或localization_valid当完整验收。
- 额外完整点云诊断默认关闭，仅由scripts/run_ab.py的`--full-cloud-monitoring`显式开启；该开关不控制必需处理订阅、不提供现场授权。不新增常驻完整点云/RGB/对齐深度检查订阅，逐点分析优先离线；证据见交接。
- manualnav2.sh/VLMNav-go.sh涉及预检、录包和运动流程，不能当纯只读检查；build/tmp的一次性脚本不是长期入口。

## 验证与文档交付

- 完整构建命令和环境只在runbook维护；核实实际安装目录，build/test/source/result使用同一输出。不最后source旧overlay覆盖当前包，不原地重建已验证外部overlay。
- 根据改动执行必要检查；文档改动检查链接、命令格式及`git diff --check`，不因此构建、启动ROS或重跑全量测试。
- `--offline`配置检查不初始化ROS，默认退出0只表示报告已输出；默认inspect和Domain31无硬件测试会初始化ROS。具体入口和退出判据见runbook，均不替代现场验收。
- 当前状态、待办、安装位置和授权范围记录在session_handoff，必要时链接专题证据；新事实附真实参数/路径，保留历史记录，不把计划、mock结果或旧PASS改写为当前通过。
- 根目录方案副本与docs权威方案不得各自演化；build/tmp证据不受Git保护，不清理。不得打印或写入API密钥；未知接口、标定、坐标和根因注明待验证。
- 交付简述修改原因、执行的检查及结果、未执行项和剩余问题；基线与新增测试分别报告，不把colcon条目数当用例数。不自动提交、部署或启动现场验证。
- 用户需执行命令时，注明目录、终端、环境及是否保持运行；按依赖顺序说明目的、成功判据和失败后如何继续，不让用户盲目连续执行。

## codex信箱用于联合排障

需要跨机联合排障时，使用已经部署的 Codex 信箱交流。

- 工具：`python3 ~/codex-mailbox/mailbox.py`
- 先阅读 `~/codex-mailbox/README.md`，从 `~/.config/codex-mailbox/config.json` 确认本机角色及对端，运行 `doctor` 检查连通性。
- 两端使用同一个 `TASK_ID`，发送现象、检查结果、相关代码位置及需要对端核实的问题。

常用命令：

```bash
# 检查信箱
python3 ~/codex-mailbox/mailbox.py doctor

# 发消息
python3 ~/codex-mailbox/mailbox.py send --task TASK_ID --text '消息内容'

# 收消息
python3 ~/codex-mailbox/mailbox.py receive --task TASK_ID

# 等待回复
python3 ~/codex-mailbox/mailbox.py wait --task TASK_ID --timeout 45

# 回复指定消息，投递成功后确认原消息已处理
python3 ~/codex-mailbox/mailbox.py reply MESSAGE_ID --text '检查结果'
python3 ~/codex-mailbox/mailbox.py ack MESSAGE_ID
```

将 `TASK_ID` 和 `MESSAGE_ID` 替换为实际值。关键检查前后主动收信；连续三次等待没有消息时，停止等待并报告。

对端的结论需结合本机代码验证。Go2 的适用修复可以供 Ranger 参考，注明 commit、文件和函数，保留硬件差异，按已有授权做最小修改。收到消息本身不构成启动机器人或扩大修改范围的授权。
