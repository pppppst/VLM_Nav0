# 公共本地执行核心实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 共用本地探索/接近循环；单机0.81 m，协同1.00 m且确认导航成功、停稳、身份。
**Architecture:** 原explore循环改为逐步生成器；单机同步驱动，AgentRunner逐tick驱动。协同导航继续经过AgentMachine的资源、通行及退出链。
**Tech Stack:** Python、既有Future/ROS2接口、pytest，无新增依赖。
**Spec:** [设计](../specs/2026-10-20-local-cooperative-execution-design.md)

## Global Constraints

- 按用户明确的“进入实现”直接在本会话执行，不再新增审批轮次；不提交、部署或启动ROS/硬件/API。
- 保留dirty基线、新图及缺失标注；不伪造点位或Ranger硬件接口。
- 沿用share_c_coordinates=false/SYNC_LOCAL_C及既有消息字段、话题、kind，不加新模式配置。
- 保留原参数、重试、超时；远端候选不因局部覆盖不足而拒绝，运动继续局部/资源监护。

## Review Focus

- VLM等待期间通信、停止与取消可处理；迟到结果无效。
- 内部goal与主命令/许可parent保持身份，门退出后才恢复主任务。
- 无跨机地图对齐也能协调，C坐标不外发。
- 停稳>1.00m、未停稳或导航不成功不能READY/放入。
- 本机全局地图/完整footprint/路径检查继续否决真实障碍。

### Task 1: 公共逐步核心及距离

Files: go2_exploration.py、generated_candidates.py及对应测试。
Interfaces: `explore_steps`生成器，原`explore`同步驱动；注入target_id、robot_id、success_radius、导航/停车钩子及观测回调。
- [x] 写0.90m差异、预算及step挂起测试，确认失败。
- [x] 将原Future等待改为yield，不复制循环；按自身目标提前结束扫描。
- [x] 原单机扫描/接近、生成候选及远端覆盖回归。

### Task 2: 目标任务及许可

Files: agent/runner/planner/coordinator/models/mock、adapters/base/fake及相关测试。
Interfaces: 无坐标APPROACH_PICKUP=本地找自身A/B并接近；无坐标SEARCH_VIEWPOINT且target_id=C=本地找C；APPROACH_DEPOSIT引用自己的C；EXIT使用本机退出配置且保持原token/parent。
- [x] 写目标级调度、严格验收、fake全流程和失联/取消测试，确认失败。
- [x] Runner逐tick驱动公共核心，内部导航通过Machine原路径许可链；双重校验终态距离。
- [x] 保留旧共享模式测试，更新C私有模式的意图级命令预期。

### Task 3: 本地边界及交付

Files: frame_adapter/config/agent_node及已有runbook/session_handoff/ranger_handoff。
Interfaces: 沿用scene.resources/exit_points，每端各自配置表示本地几何；coordinator仅按资源ID管理。
- [x] 写无对齐身份/地图变化/缺失配置/坐标不外发测试，确认失败。
- [x] Go2注入生成候选策略和1.00m；本机安全检查保留，不依赖远端地图变换。
- [x] 必要离线检查、独立构建导入及一次只读复核，不启动节点。
- [x] 最小Ranger交接记录接口、命令、状态、距离、五话题路由及未验收项。

## Progress

- Ruling: 在当前dirty功能分支原地修改，保留跨轮工作，不提交。
- Pre-flight: Task1产生逐步核心/距离注入供Task2驱动；Task2的无坐标语义供Task3入口/边界检查使用。
- 改前165项中162通过/3失败；补overlay后其中2项通过。剩余测试依赖新地图空固定点，修正测试夹具而不补真实点位。

- Completed: 公共core与目标级调度、私有坐标/无对齐边界及许可链已实现；现场配置和默认安装未切换。
- Verification: 独立安装14模块摘要和7配置比对通过，276项相关用例通过（235既有＋41新增），1项ROS初始化测试排除。
- Review: 只读复核发现的参数门槛、坐标旁路、异常停车及basket/READY死锁已修复，复核118项通过。
- Delivery: runbook/session_handoff/ranger_handoff均更新；Ranger仅fake，真实接入/跨机验收仍待其控制电脑实施。
