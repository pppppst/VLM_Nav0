# Go2 目标坐标规划与实际终点导航

> 使用 writing-plans / executing-plans 在当前会话实施；用户已确认下列三步设计。保留现有工作区改动，不提交、不部署、不启动现场实验。

**Goal / Spec:** 用户确认：不设最低安全距离；识别目标地图坐标交给 Nav2 规划，不枚举圆周 8 点；检查返回路径、终点机身碰撞与禁入区及距离；将安全的实际终点交给导航，最终要求 Nav2 成功、停稳、距目标 ≤1m。

**Architecture:** 在现有 `go2_exploration.py` 的 Go2 协同分支复用 compute_path、costmap footprint 检查、导航及到达验收。保留 Ranger 与原单机成功栈。目标物体坐标是规划请求，不要求物体所在位置可供机身停车；必须检查规划实际返回的路径和终点。沿用控制容差与上限距离余量，不把它们解释成最低安全距离。不改 Nav2 参数、协议或依赖。

**Tech Stack:** Python、ROS 2 Humble、Nav2，pytest 离线假适配器。

## 范围与验证重点

- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_exploration.py`：取消协同 Go2 的 select_approach 调用，按返回终点导航，保留取消/健康/超时及终态门槛。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/runner.py`：二次规划及重试复用路径验收，拒绝或输入异常进入停止流程，取消时清理回调。
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/go2.py`：保留原碰撞语义，仅将每点全图运算限制到机身包围盒网格，避免整路径检查阻塞回调。
- `cooperative_ws/src/multi_agent_nav/tests/test_target_nav2_endpoint.py`：目标占据但返回终点可用、空/失败/非有限路径、路径中间碰撞/禁入、距离上限、无最小距离、取消与晚返回、最终失败/未停稳/超距。
- 原接近、初始相机、local tasks/C、双机验收测试：保证接线与旧流程兼容。
- `docs/multi_agent/session_handoff.md`、runbook：记录实际检查及源码/安装状态，不把离线结果写成现场通过。

## Task 1：目标坐标规划

- [x] 保存当前文件并运行原相关测试基线：208通过，3项既有失败。
- [x] 新增回归测试：21项旧流程预期失败；4项真实runner测试另复现二次规划绕过校验，再修复。
- [x] 修改目标规划分支，验证路径与实际终点；保持有限重试，不再缩小圆周半径。
- [x] 最终306通过、明确排除3项基线失败；含29项新增。独立审查发现的二次规划和全图运算阻塞问题已修复并复审通过。
- [x] 更新交接与runbook；不自动更新安装或启动真机。

**执行判断：** 当前功能依赖大量用户未提交代码，因此在原工作区保存改前副本后小范围修改，避免从 HEAD 建工作树遗漏实际基线；设计已获用户明确确认，不再次请求相同授权。

**审查后的范围调整：** 复用原检查回调覆盖runner二次规划是完整执行三步设计的必要接线；ROI是从终点检查扩展为整路径检查所需的性能修复。未扩展到全局导航重构。证据在 `build/go2-target-plan-20261025/evidence/`，最终验证环境与命令见runbook。
