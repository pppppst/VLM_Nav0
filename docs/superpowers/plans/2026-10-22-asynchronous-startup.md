# 双机异步准备与本轮握手实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. 不自动提交、部署或运行现场实验。

**Goal:** 双方独立准备、异步等待，明确接受并确认本轮任务后才执行；执行中严格失联停车。

**Architecture:** 复用 schema v1 的 AgentState.run_state 和 SYNC_LOCAL_C。共享 AgentMachine 管理启动/握手及接收心跳，coordinator 等双方 SYNCED 后派工；真实 ready 仅用于 A/B 验收。

**Tech Stack:** Python、现有 ROS2 消息、现有异步 worker、pytest 离线替身。

**Spec:** 本会话用户提供的五阶段状态表与四条约束；以下为实现边界。

## 全局约束

- 准备状态为 PREPARING、WAITING_PEER、STANDBY；握手为 SYNCING、SYNCED；执行沿用当前运行状态。
- 本机健康只读本机真实条件；健康且停稳、原生动作终止即可 WAITING_PEER，不依赖对方先待命。
- 接收时间与健康分别记录；在线但未就绪不冒充失联。等待阶段不自动制造 PAUSED，也不解除执行后锁存的 PAUSED。
- SYNC 接受后立即发布新任务身份，不等对方；双方均回报当前任务 SYNCED、无旧命令/goal/验收标志，coordinator 才下发 EXECUTE。
- 同步阶段不沿用执行期3秒超时取消正常初始化，仍受既有600秒总任务上限和显式取消约束；执行期3秒通信门槛不变。
- 旧任务、旧命令、旧goal、旧 READY 不完成新握手；真实地图/模式身份保持校验。
- 不增加话题、字段、依赖或假坐标；Ranger现场adapter不在本机，交付共同修改与迁移清单，不声称远端已更新。
- 完整入口原有一次start授权方式不增加确认词；原StartTask接口仍是唯一创建任务的入口，节点本身不自动start。
- 不覆盖已验证安装、不重启当前组件；用户未提交修改、配置开关和地图删除均保留。

## 审查重点

1. 等待一分钟/准备中重启，双方无需同时启动；任一健康变化都不形成循环依赖。
2. 当前任务握手不能由旧任务/旧命令/旧goal完成；重启后重新同步仍无运动。
3. 执行后失联、健康失效、地图变更仍取消并停稳后锁存 PAUSED；恢复通信不恢复旧运动。
4. 候选几何、图像渲染、模型操作耗时期间，主循环仍可收发心跳和处理取消；迟到结果被隔离。
5. 新旧版本 schema 相同也不能误判握手兼容；明确告知两侧必须同步更新。

## 任务1：共享 agent 启动及心跳语义

涉及 models.py、agent.py、agent_node.py、mock.py 与 tests/test_startup_handshake.py。

- [x] 先写双方参数化用例：晚启动60秒、准备中健康抖动、单端重启、在线未就绪；观察旧代码失败。
- [x] 在 AgentMachine 统一接收对端状态与本机准备/握手转换；node与mock使用同一入口。
- [x] 验证准备无对端依赖、当前task握手、执行后严格停车及旧消息隔离。

## 任务2：coordinator 与入口语义

涉及 coordinator.py、scripts/start_go2_cooperative_task.py、已有protocol/coordinator/入口测试。

- [x] 先写 SYNCING 只重发同步、双方SYNCED才派工、旧确认拒绝、健康/心跳分开报告用例，观察失败。
- [x] start进入SYNCING；保持显式创建任务；等待双方真实确认后进入SEARCH_AB，首个故障原因保留。
- [x] 更新准备采样接受WAITING_PEER/STANDBY，保留新鲜度、身份、发布者、QoS和空旧任务检查。
- [x] 更新原测试的握手fixture与语义断言，保留严格到达/独立C/取消/失联原覆盖。

## 任务3：耗时工作与交付

涉及 go2_exploration.py、异步worker相关测试、runbook、Ranger交接和双机文档。

- [x] 先用受控阻塞worker验证耗时候选准备/图像/模型工作时仍可推进心跳、处理失联和取消。
- [x] 将同步重计算交给现有worker并yield；不改传感器、定位或Nav2，不增加原始传感器订阅。
- [x] 跑针对性回归，对照基线失败；执行源码/语法/文档检查，不构建或执行现场动作。
- [x] 按 requesting-code-review 做一次独立只读审查，修复关键问题。
- [x] 记录验证与Ranger待迁移项，交付修改说明；不自动提交或部署。

## 执行记录

- 初始分支go2-cooperative-task-layer，HEAD 4fc673156c5c985c4d9a9149cfe0b08b1582d90b，已有修改保留。

- 实施限定：新启动状态仅用于当前真实 `share_c_coordinates=false` 私有坐标模式；旧共享坐标模式保持原启动语义与回归，不迁移旧许可协议。
- 独立审查发现并修复：不对称首条EXECUTE导致另一端退回SYNCING；同步地图摘要读取阻塞心跳。均有先红后绿证据。
- 离线最终集合465通过/5项基线失败，新增启动握手35项及阻塞作业8项均通过；基线269通过/同5失败。XML逐项对照无新增失败。
- 未构建、部署或连接硬件；Ranger实际安装、adapter与现场DDS待两端核对。修改说明见 ../../multi_agent/async_startup_changes_20261022.md。
