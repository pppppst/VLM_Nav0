# Go2 最终异步启动契约实施计划

**Goal:** 按 [双方最终契约](../../multi_agent/ans_async_startup.md) 补齐 Go2 接收、回执、阶段提交与隔离安装，不改硬件启动顺序。

**Architecture:** 复用 schema v1、AgentMachine/Coordinator、现有单 worker 和入口。保留旧共享坐标模式；只在当前私有模式执行新规则。

**Tech Stack:** 现有 Python/pytest/colcon，不新增依赖。

**Spec:** docs/multi_agent/ans_async_startup.md，覆盖旧异步说明的冲突部分。用户称 Ranger 已依契约修改，本轮不访问或覆盖其源码。

## 全局约束与审查重点

- 2Hz、lost_timeout=3.0、task_timeout=600.0；时间按本机单调时钟。配置和真实地图身份按契约保持，不修改真实验证标志。
- 同步接受两种正式编码；新任务只由干净 phase=SYNCING 同步建立。PAUSE 始终有效，CANCEL 必须绑定当前命令。
- SYNCING/空reply 可以推动 agent 发 ACK；只有 SYNCED/SYNCED 可以让 coordinator 提交。
- 保留已发 ACK 记录，移动状态先于阶段同步也不死锁；提交后即使首条 EXECUTE 尚未到达，仍严格监控执行期故障。
- 旧task消息不刷新当前任务时间；未知地图/schema/模式不确认且报告，不能自动采纳新身份。
- 后台旧任务未结束时新工作yield等待，不阻塞或自动重启；结果到达后保持原运动检查。
- 不覆盖旧安装、未提交修改或地图删除；不自动提交、部署、SSH、ROS现场采样或运动。

## 任务与验证

1. 协议与状态：models.py、agent.py、agent_node.py、mock.py、coordinator.py。
   先补 test_async_contract.py：两种SYNC编码；非法字段与旧阶段拒绝；双字段ACK；阶段乱序与首条丢失；地图/schema/模式拒绝；600秒整轮预算及显式暂停/命令级取消。观察旧实现失败后补最小实现，更新旧fixture。
2. 入口与worker：scripts/start_go2_cooperative_task.py、scripts/cooperative.sh、go2_exploration.py。
   先验证脏准备状态不能通过入口/StartTask、旧worker仍阻塞时新任务能发心跳并取消；保留无限等待和一次start。
3. 对比本轮基线，跑相关离线集合和消息摘要检查；独立只读审查并修复重要问题。
4. 按runbook构建新隔离目录，build/test/source/result统一该输出；核对实际安装与源码、消息字段摘要、离线预检。旧overlay不覆盖，不切换运行中的进程。
5. 记录修改说明、准确验证数与剩余现场事项，更新runbook/交接/启动文档的安装引用。

## 执行记录

- 开始：go2-cooperative-task-layer，HEAD 4fc673156c5c985c4d9a9149cfe0b08b1582d90b，全部已有改动保留；快照/日志在build/async-contract-20261022。

- 已完成：协议/状态/诊断/worker/入口默认安装与help文案；2Hz/3秒/600秒与地图身份值本来一致，未猜测改参数。
- 基线465通过/5旧失败；新增契约72项、入口脏状态10项、worker旧任务占用2项、默认/显式overlay2项通过。实际安装最终551通过/同5失败。
- 独立审查发现旧PAUSED缓存误取消新任务，先红后绿修复；旧共享坐标初始宽限回归保留。
- 已构建build/go2-async-contract-20261022并验证27模块一致、契约消息摘要一致、默认preflight通过；没有启动ROS/硬件或切换运行中进程。
