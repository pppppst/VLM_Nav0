# Go2 最终异步启动契约补齐说明（2026-10-22）

依据 [Ranger/Go2 最终契约](ans_async_startup.md) 完成 Go2 代码、入口和隔离安装。用户确认 Ranger 已按契约修改；本轮未访问其源码、安装或机器人，不能以本机结果证明远端已生效。

## 本轮修改

| 契约要求 | Go2 落实 |
|---|---|
| 两种正式同步编码 | 支持 `action=SYNC, kind=SYNC_LOCAL_C` 及 `action=SYNC_LOCAL_C, kind=WAIT/空/SYNC_LOCAL_C`；发送端沿用后者 |
| 仅同步阶段建立身份 | 私有模式新 task_id 只接受合法、干净 `phase=SYNCING` 同步；旧 SEARCH_AB/SEARCH_C/DEPOSIT/PAUSED/DONE 等不能使重启端采纳旧任务 |
| 双字段确认 | agent 接受同步立即发 SYNCING/空 reply；收到本轮干净 SYNCING/空 reply 或 SYNCED/SYNCED 后可发 SYNCED/SYNCED。coordinator 只接受后一种规范 ACK |
| 阶段提交与乱序 | 双方 ACK 后先同步 SEARCH_AB 再 EXECUTE。曾发有效 ACK 的记录不因对端先移动或临时撤回当前就绪状态丢失；重复 SYNC 不清身份、peer 缓存、历史 ACK 或总预算 |
| 提交后的严格保护 | SEARCH_AB 提交后即启用执行期监护，即使首条 EXECUTE 尚未到达；对端重启导致 task_id 为空、身份变化、健康失败或3秒超时均停止并锁存 PAUSED |
| 干净准备与握手 | 检查 sequence/reply、命令/goal/验收标志、私有资源/令牌及零公共位姿；旧 IDLE 不作为准备/握手确认 |
| 固定地图身份 | agent 从本机配置读取预期 peer 地图；未知地图/schema/模式不更新有效心跳或推进握手，报告 mismatch，执行期立即暂停。coordinator 不自动采纳未知地图 |
| 显式新任务恢复 | 停稳 PAUSED 可参与显式新任务，agent 仍本地检查原生动作终态并清理旧命令/标志；旧任务 PAUSED 缓存不能取消新 T 首次 SYNC。CANCEL 仍必须绑定当前命令，握手期用 PAUSE |
| 后台任务占用 | 旧后台任务未退出时，新候选工作 yield 等待 worker 可用；不阻塞心跳、不排队启动重计算，取消后迟到结果不驱动运动。导航前复核保留 |
| 安装与入口 | 扩大预检的一致性检查至 models/agent/agent_node/coordinator/coordinator_node/mock/runner 等10个关键模块，且两种同步编码都必须通过；Go2 默认环境选择新隔离安装，显式 COOP_SETUP_FILE 覆盖保留 |

准备仍无限等待、每5秒报告原因；StartTask 只发一次，单独 agent/coordinator 不建任务。相机、底层、RViz、Sport、bridge、agent、coordinator 顺序不改；Ranger 自定义 adapter、底盘、地图、footprint 和硬件许可不覆盖。旧共享坐标模式保留原启动及初始宽限逻辑。

## 参数与安装核对

下列参数此前已一致，未修改真实值、验证标志、网卡绑定或桥接路由：

| 参数 | 实际值 |
|---|---|
| share_c_coordinates | false |
| heartbeat_hz / lost_timeout / task_timeout | 2.0 / 3.0 / 600.0，600秒含握手与执行 |
| Go2 地图 | lab_rescan_20261018_155656_0pa6gm；sha256:6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279 |
| Ranger 预期身份 | lab_ranger_20261018161152；sha256:8b1a3f0587afd56e20431668647cf2624d83897a587090ef808424eb6049fc30 |
| 新安装 | build/go2-async-contract-20261022/install/local_setup.bash |

构建复用原消息/root包，仅重建修改的 multi_agent_nav；完整环境和命令只在 [runbook](go2_runbook.md#2026-10-22-最终异步契约隔离安装已验证) 维护。未覆盖旧 overlay，未 source 到正在运行的进程。

实际导入的27个 Python 模块与源码逐字节一致，3个消息字段摘要均匹配契约。显式新安装及不传 COOP_SETUP_FILE 的默认预检都退出0：两种 SYNC 编码 true、live_gates=[]、API环境齐全、候选518、motion_sent=false。这是离线配置核对，不是现场健康或通信通过。

## 验证结果与限制

- 修改前相关集合：465通过、5失败。
- 实际新安装最终相关集合：551通过、同5失败，XML逐项比较无新增失败。
- 新增86项：契约72、准备采样10、旧worker占用2、默认/覆盖安装选择2，均通过。涉及晚启动、重启、乱序、600秒整轮预算的原用例一并回归。
- 同5项旧失败依赖 gates 尚关闭的过时假设：`test_protocol::test_broadcast_only_sync_and_live_templates_block`、`test_independent_c::test_no_gate_requires_resource_geometry_or_stopping_envelope_margin`、`test_local_tasks::test_live_local_gates_require_local_assets_but_not_remote_alignment` 及两项 `test_malformed_local_geometry_is_reported_as_gate_failure`。未改真实配置使其转绿。
- 独立只读审查发现旧 PAUSED 缓存误取消新任务，已补先红后绿回归；收尾未留已确认的重要问题。保留原共享模式回归。
- shell语法、Python AST、入口帮助输出、修改文档本地链接及 `git diff --check` 均通过；构建后再次比对27模块仍与源码一致。

证据：`build/async-contract-20261022/` 保存本轮前源码、基线、契约/worker/入口红绿日志、构建日志、安装逐模块核对与预检；`build/go2-async-contract-20261022/evidence/installed-tests.xml` 是实际安装测试结果。build 证据不受 Git 保护，不清理。

未执行真实 ROS 节点、SSH、静止通信、跨机重启、ARM/start、录包或物理运动。两端应先按原流程暂停旧任务、确认停稳和原生动作终态、正常退出旧任务节点，再用最终安装验证；原进程不会热加载修改。本契约的现场静止通信、实际调度和运动验收仍未完成，不自动启动或续跑旧任务。
