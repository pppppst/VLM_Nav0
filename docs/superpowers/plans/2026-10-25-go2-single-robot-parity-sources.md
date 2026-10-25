# Go2 单机接近迁移：来源、边界与验证

## 冻结依据与范围

起始分支 `go2-cooperative-task-layer`，HEAD `df1a519df222c40f32e5222801d42fe20d8c4c44`，工作区干净；与[附件计划](2026-10-25-go2-single-robot-parity.md)协作依据完全相同。两 Git 对象存在，单机提交是协作提交祖先。以 `git show 32eab6244ec4b483807f8ed68e8667f5495874bb:vlm_nav/vlm_navigator.py` 和该 SHA 的 `config/robot.yaml`、`config/robot_go2.yaml` 为唯一单机来源；没有拼接其他单机版本。

在 `/tmp/go2-single-parity-20261025` 本地共享克隆中实施，原工作区、现有安装、日志和 bag 保留。没有提交或切换分支。

模式限定为真实 Go2、`share_c_coordinates=false`。A 和本机 C 接近共用内部执行器。Ranger、mock、旧共享模式保持原分支；因此旧 mock 端点测试仍有意义，不将它们改写为新真实模式的合同。

用户明确补充：单机 `safety_tick` 的运动半径与动作超时分支也迁移。读取 `max_travel_radius + 0.50` 和 `goal_timeout`，不将基线计算结果写成运动条件常量。参数验证仅检查所需参数类型、有效值及冻结基线一致性，不增加距离关系、余量或准入条件。

## 单机函数 → 新位置 → 接口适配

下列源函数均属于完整 SHA `32eab6244ec4b483807f8ed68e8667f5495874bb` 的 `vlm_nav/vlm_navigator.py`。新模块位于 `cooperative_ws/src/multi_agent_nav/multi_agent_nav/single_robot_approach.py`；除表中注明的适配外，保留接近分支的比较符、条件顺序、状态更新、次数与超时起点。

| 源函数/分支 | 新位置 | 必需适配 |
| --- | --- | --- |
| `navigation_tick` APPROACHING | `SingleRobotApproach.navigation_tick`；`go2_exploration.explore_steps.approach_target` 选择新入口 | 协作目标经 `frame.to_local` 转 map；x/y 用目标，yaw 用当前机器人；无活动动作/规划时才请求，近距离直接停止验收 |
| `plan_then_navigate`、`request_next_plan` | 新模块同名方法 | 复用 observer 的 ComputePathToPose 客户端，`use_start=False`；保留 token、候选与 `goal_started` 的规划更新时机 |
| `on_plan_response`、`on_plan_result` 的 approach 分支、`reject_current_plan`、`goal_key` | 新模块同名方法 | 原生规划状态/非空路径决定导航；返回终点只诊断，原请求目标传给导航；任务取消后的迟到规划接受额外取消其句柄，防止孤立请求 |
| `send_navigation_goal`、`on_goal_response` | 新模块同名方法；`Go2Adapter.start_approach_navigation` / `_motion` | 沿用现有动作所有权与 task/command/goal 身份；adapter 持有传输和终态清理，单机执行器唯一消费接近结果，不再进入普通 runner 结果队列 |
| `safety_tick` 接近/停止/半径/超时分支、`target_reference_distance`、`approach_stop_radius` | 新模块同名方法 | 可控 monotonic 时钟；保留 0.2 s 安全检查与配置导航周期调度，外层 runner 仍先检查协作健康 |
| `begin_approach_stop`、`request_approach_cancel`、`on_approach_cancel_response` | 新模块同名方法 | 仅取消执行器自己的句柄；停止命令复用 observer 原 `publish_stop`，不调用任务 PAUSE；取消应答不代表动作终态 |
| `advance_approach_stop`、`on_arrival_odom`、`elapsed_since` | 新模块同名方法；adapter 原里程计订阅转交消息 | 保留 `hypot(linear.x, linear.y)`、`abs(angular.z)`、运动样本清零、接收时间及连续样本规则；不冒用协作 `health.stopped` |
| `on_navigation_result` | 新模块同名方法；adapter 原生 action 回调 | SUCCEEDED 进入停止验收；仅停止状态中的 CANCELED 可继续验收；ABORTED 按源分支失败，不走协作额外重试 |
| `finish_approach_success`、`reset_approach_stop` | 新模块同名方法；`AgentRunner.local_approach` 与 `AgentMachine.complete_local_execution` | 当前身份内的内部成功产生既有 `TARGET_APPROACHED` 结果；外层健康、停稳、终态、≤1 m 独立检查，未满足则等待，不重新导航/重试 |
| `cancel_motion`、`fail_safe` | 新模块 `cancel_motion` / `close` / `fail_safe` | 原清理字段保留，包括 `active_motion_origin`、pending 标志和 stop 状态；外部生命周期关闭幂等，并通过 token 阻断迟到回调 |

`active_motion_origin` 仍在接近规划成功时取 `plan_window_origin`，在导航终态、成功完成与取消清理中清空。`goal_started` 仍在规划开始和导航请求前分别更新，不在结果回调或取消应答时重置。

冻结合并参数：成功半径 0.81、首次取消半径 0.89、重试停止半径按原函数取成功半径、距离重试上限 1；线速度 0.03、角速度 0.05、连续样本 3、里程计最大年龄 0.5、取消终态超时 3、终态后停稳超时 5；里程计 `/odometry`。迁移所需导航周期、运动半径、动作超时和 blocked-goal 时长也从同一合并配置读取并比对。基线表用于报告不一致，不用于替代实际参数。未查询现场运行节点的参数。

## 移除的额外规则与保留差异

真实 Go2 本地接近绕过原 `approach_attempt` 的整条路径/终点 footprint 否决、实际终点替换、终点距离加容差/余量预算、圆周选点和二次规划。探索使用的几何函数、253/254/255 阈值、Keepout 和动态 costmap 配置未改。

agent 和 explore 两层在本模式导航期间均不再以首条执行路径或 5 秒无路径更新触发停止，包括去观察点；adapter 的真实 `path_valid`、路径订阅、显示和超时诊断保留，没有永久伪造 path_valid，也没扩大接收超时。位置/footprint 格式检查与其他健康检查保留。

协作外层仍保留身份、PAUSE/CANCEL、心跳、定位、Nav2/传感器/地图健康、串行运动所有权、任务超时、停稳、动作终态、≤1 m 与双 READY 屏障。内部 0.81 m 不被外层覆盖。内部成功后外层未放行期间只等待已有门槛，不重新调用执行器，也不消耗或增加重试。因此不能声称整个系统与单机完全等价。

被动日志记录原目标、路径终点、实际导航目标、原生状态、停止原因、取消应答、终态、里程计年龄、样本数、最终距离、次数与失败原因。运动日志捕获原任务记录器，迟到事件不写入新任务目录；新增诊断写入失败告警，不设置 motion_fault 或阻断停止。

## 验证边界

差分参考由测试运行时用完整 SHA 读取、AST 提取真实方法；没有手写替代参考算法。两端使用同一输入事件和可控时钟，比对规划、导航、取消、停止、状态和失败序列。测试隔离了传感器健康、ROS 传输、可视化和 fail_safe 的外围依赖，属于接近子流程差分，不是完整单机节点等价证明。另用真实 adapter、runner、agent 与受控 action future 验证集成及失败清理。

实际命令和最终计数在 [runbook](../../multi_agent/go2_runbook.md#2026-10-25-冻结单机接近行为迁移离线源码验证) 维护；本轮状态见 [session_handoff](../../multi_agent/session_handoff.md)。基线三个旧门槛失败不修改、不隐藏。

未部署、未启动 ROS 节点或真机、未验证实体停稳、两机避碰和现场目标可达性。动态障碍连续三周期确认与此前疑似残留障碍均不属于本轮修复，移除额外拒绝不证明虚假障碍已消除。
