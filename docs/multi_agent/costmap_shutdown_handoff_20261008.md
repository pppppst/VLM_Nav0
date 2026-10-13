# Go2 costmap 退出排查交接补充

日期：2026-10-08。接续 [session_handoff.md](session_handoff.md)，已读取其中的
[权威架构](../superpowers/plans/ranger_go2_codex_plan.md)和
[权威执行方案](../superpowers/plans/2026-10-07-go2-cooperative-navigation.md)。
最初只新增本文件、做静态核对；新增授权后的复现及修复见
[末节](#新增授权后的隔离复现与修复)。此前静态分析保留为历史；完整协同live继续关闭。

最初静态结论：现有停止链存在重复 SIGINT 投递路径，且父 launch 返回 0 掩盖子进程异常。
应先隔离核实并修正该停止路径，不改 costmap 参数。旧轮次的 SIGSEGV 具体崩溃点
仍未定位；最新生命周期 shutdown 后的 SIGINT 退出不能证明段错误已经修复。

## 工作区和有效解释

仓库 `/home/isee-pst/unitree_ros2/VLM_Nav`；实查分支 `go2-cooperative-task-layer`，
HEAD `ce2999a4ace8409a4244e4aac02080e9ad93f479`。检查仓库祖先目录及仓库内
（含隐藏目录、build/install）未发现适用 `AGENTS.md`。开始时工作区如下，全部保留：

```text
 M docs/multi_agent/dynamic_costmap_stage.md
?? docs/multi_agent/session_handoff.md
?? go2_codex_implementation_plan.md
?? ranger_go2_codex_plan.md
?? scripts/run_ab.py
?? test/check_go2_diagnostic_monitor.py
```

以下证据链接均指本机已有文件，不能清理或覆写。证据根目录为
`build/go2-field/evidence/fastlio-retest.1_h_1z2p/`。

- [clearing-review.json](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/clearing-review.json)：
  操作者“原位置消失，新位置出现”支持所观察 costmap 的定性标记/清除；未区分
  local/global，原全图 0/38、1/26 不是该实物的 ROI 清除率。
- [posture-clarification.json](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/posture-clarification.json)：
  启动约 0.34 m 位移包含趴姿到站姿的真实运动；没有转换起止时间及外部真值，不能
  当静止漂移或减去假设站高。基线后统计不变，也未独立确认基线前站姿转换结束。

静止感知、定位、地图与 costmap 链路贯通仍不等于完整自主导航或双机协同通过。
本次未改任何验收标记、配置或上述原始证据。

## 两次退出的证据

| 轮次 | 生命周期及信号日志 | 子进程结果 | 父 launch 结果 |
|---|---|---|---|
| 150 秒 `static-chain-fixed.0id75ch3` | SIGINT → global/local preshutdown、deactivate、cleanup → local `Destroying`；costmap 有两条 signal_handler 日志 | costmap 16882 为 −11（SIGSEGV）；manager 16884 为 0 | manifest 中 costmaps 16736 为 0 |
| 182.39 秒 `static-chain-repeat.skw8amvz` | command=4 → global/local deactivate、cleanup、shutdown → manager 完成日志 → SIGINT → preshutdown、local `Destroying` | costmap 17959 和 manager 17961 均为 −2（SIGINT） | manifest 中 costmaps 17814 为 0 |

证据位置：

- 旧轮次 [costmaps.log](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-fixed.0id75ch3/runtime/costmaps.log)
  第 162—181 行；[manifest.json](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-fixed.0id75ch3/manifest.json)。
- 最新轮次 [costmaps.log](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/runtime/costmaps.log)
  第 118—147 行；[costmap-shutdown.log](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/costmap-shutdown.log)
  保存 command=4、success=true；[manifest.json](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/manifest.json)。

旧轮次最后一条正常日志只定位到 local 节点析构附近，不是崩溃栈。最新 −2 说明
最终被信号 2 终止，不应写为“正常退出”，也不是另一条 SIGSEGV 证据。
本次在现有 evidence 文件名中未找到 costmap core/backtrace；未查询系统 core 存储。

## 信号和生命周期调用链

[run_static.py](../../build/go2-field/evidence/fastlio-retest.1_h_1z2p/static-chain-repeat.skw8amvz/run_static.py)
第 28、88 行通过 `ros2 launch` 启动 costmaps，finally 调用
[scripts/run_ab.py](../../scripts/run_ab.py) 的 `stop_owned`。`launch_owned` 使用
`start_new_session=True`；`stop_owned` 第 95—111 行先向所有存活进程组发 SIGINT，
再逐个等待。逆序发送不等于逐个完成后再停止上游。记录的 returncode 是 Popen
直接管理的父进程，未检查 launch 内的 costmap/manager 结果。

本机 launch 的 `ExecuteLocal.__on_shutdown` 在 noninteractive 模式下转发 SIGINT；
`ros2 launch` 默认由 stdin 是否 TTY 判断该模式。现有日志明确记录了向两个子进程
转发 SIGINT。本机 osrf_pycommon 的 subprocess 创建代码没有另设子进程 session
或进程组。因此静态链路为：

```text
stop_owned: killpg(launch_pid, SIGINT)
  → launch 及其同组 costmap、manager 收到信号
  → launch 再向 costmap、manager 各转发 SIGINT
```

这证明代码允许重复投递；旧 costmap/manager 各两条 handler 日志与之相符。
没有逐次信号跟踪，尚不能给最新 −2 确定第二次投递的精确时刻。
本机静态依据：

- `/opt/ros/humble/lib/python3.10/site-packages/launch/actions/execute_local.py`：249—280、426—431 行。
- `/opt/ros/humble/lib/python3.10/site-packages/ros2launch/command/launch.py`：77、175 行。
- `/opt/ros/humble/lib/python3.10/site-packages/osrf_pycommon/process_utils/async_execute_process_asyncio/impl.py`：39—47、72—80 行。

本机安装版本：Nav2 costmap/util/lifecycle_manager 为 1.1.20；rclcpp 及
rclcpp_lifecycle 为 16.0.19；launch 为 1.0.14，launch_ros 为 0.19.14。
以下上游源码按这些版本核对，属于源码顺序依据，不是现场线程栈或实际加载库证明。

Nav2 manager 的普通进程 preshutdown 只停止管理线程、清理客户端等，并不等价于
`manage_nodes SHUTDOWN`。后者按节点逆序执行 deactivate → cleanup → unconfigured
shutdown；1.1.20 的 `shutdownAllNodes()` 没有向外传递转换失败返回值，`shutdown()`
最终返回 true。因此验证必须查两个节点的实际终态，不能只看 success=true。
[Nav2 1.1.20 生命周期管理器源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_lifecycle_manager/src/lifecycle_manager.cpp#L269-L305)。

## 宿主和析构顺序

[costmap_acceptance.cpp](../../cooperative_ws/src/multi_agent_nav/src/costmap_acceptance.cpp)
第 10—24 行的当前顺序为：创建外层 executor → 创建 local、global 节点并加入
executor → spin → `rclcpp::shutdown()` → 局部 nodes 容器销毁 → executor 销毁。
最后 `Destroying` 是 Nav2 LifecycleNode 基类日志，不代表所有派生成员、ROS
生命周期对象和 executor 已销毁完毕；容器释放次序也不能替代实际析构日志。

默认 rclcpp 信号处理线程调用 `Context::shutdown()`：先执行 preshutdown 回调，
再使 context 失效并唤醒 executor。Nav2 LifecycleNode 的回调会对 ACTIVE 执行
deactivate，对 INACTIVE 执行 cleanup；基类析构也会尝试状态清理。
所以宿主并非“没有生命周期清理”，简单在 spin 后再次添加无条件 cleanup 不成立。
[rclcpp Context 源码](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/context.cpp#L300-L334)，
[Nav2 LifecycleNode 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_util/src/lifecycle_node.cpp#L43-L108)。

原生 costmap 的 deactivate 停止并 join 地图更新线程；cleanup 释放 costmap、TF、
发布和服务对象，最后停止内部 callback executor 线程；on_shutdown 本身不代替这两步。
两个现场日志都有 cleanup，现有证据不足以把某个线程、插件或 TF 释放确定为 −11 根因。
[Costmap2DROS 1.1.20 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp#L307-L354)。

`rclcpp::shutdown()` 还会卸载全局信号处理器、恢复旧处理器，而当前宿主节点在其后
析构。结合重复 SIGINT 路径，存在析构时被后续信号终止的窗口；这是最新 −2 的候选
机制，尚未动态确认。只挪动 shutdown 行也不能保证 context 在析构时仍有效，因为
默认信号线程可能已经关闭 context。
[rclcpp shutdown 源码](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/utilities.cpp#L170-L182)，
[信号处理器恢复和异步处理源码](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/signal_handler.cpp#L167-L195)。

源码与安装 launch 的 SHA256 相同。安装宿主是独立文件，build/install 二进制摘要
不同；安装处理可改变二进制，不能据此断言版本错配。后续隔离构建须记录实际执行文件、
依赖库解析路径和版本，避免把测试新宿主等同于现场旧二进制。此次未执行任何宿主。

## 最小修复范围

以下为待隔离验证的具体修改方向，本轮没有实施，也不宣称 −11 已解决。

1. **先消除重复投递。** 停止 costmaps 的 noninteractive launch 时，只向其父 PID
   调用一次 `p.send_signal(SIGINT)`，交由 launch 转发；启动该父进程时显式 `-n`，
   避免依赖终端环境。其余直接启动的进程仍按现有进程组方式停止，不全局替换 killpg。
   保留有界等待及失败后的组级清理兜底；进入兜底必须记录失败，不能计为正常退出。
2. **有序完成生命周期。** 在 costmap、manager、map/TF 输入仍可用时先请求该 manager
   的 SHUTDOWN，记录响应，分别确认 local/global 已 FINALIZED，再结束 costmaps
   launch 并等待两个子进程退出；之后再停止上游输入。响应失败、超时或任一终态不符
   都留作退出失败。不能直接对 ACTIVE costmap 只发 shutdown 转换并假设它完成 cleanup。
3. **补齐退出判据。** 独立验证直接收集 costmap 和 manager 的退出码；launch 路径
   同时记录子进程 ProcessExited 结果，不能只用父 returncode=0。复用既有测试的
   map/TF/点云发布流程，增加终态和退出码断言，不重跑全部协同测试。

最小改动可落在以后获授权整理的静止入口及已有
[test_dynamic_costmap_config.py](../../cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py)。
当前 synthetic 测试 finally 发 SIGINT、等待后可能 kill，但未断言退出码，因此历史
marking/clearing 测试通过不证明退出正常。共享 `run_ab.py` 的 A/B 调用和两份一次性
`run_static.py` 都依赖 `stop_owned`；不要为 costmaps 擅自改变所有调用者的停止策略，
也不要覆写证据目录中的脚本。

如单次投递、有序 shutdown 后仍有 −11，先取得崩溃栈，再决定是否修改宿主或上游
清理顺序。不能用重复清理、吞异常、忽略信号或换插件掩盖失败。参数、QoS、障碍高度、
标记/清除配置和已验证定位栈均无此次退出故障的因果证据，不在修复范围。

## 隔离验证方案

仅在后续另行安排后执行，本轮未实验、未构建、未运行测试。

环境限定为 Domain31、localhost、rmw_fastrtps_cpp，unset CYCLONEDDS_URI；不桥接。
**不能直接执行当前 `go2_costmap_acceptance.launch.py` 的完整 generate_launch_description：
它硬设 Domain0、CycloneDDS、enp4s0。** 沿用既有 synthetic 测试只调用其 `build(context)`
生成参数、再直接启动两个可执行文件的方式；如验证 launch 父进程转发，使用临时隔离
launch，仅包含这两个 Node，不带生产环境覆盖。节点命名部分由宿主硬编码，须确认
Domain31 中无同名实例和冲突的 map/TF/点云发布者，不能假设外层 namespace 改掉了它。

只用合成 map、map→odom→base_link TF 和合成障碍点云；不加载机身驱动、FAST-LIO、
真实相机、Nav2 运动 action、任务 agent 或 coordinator。构建输出置于新的独立目录，
不覆盖 `build/go2-coop/install`，不覆盖历史 runtime/bag；无录包。

| 隔离项 | 固定输入及退出操作 | 解决的问题 |
|---|---|---|
| A 原停止方式 | 激活两个 costmap；同组 killpg，保留 launch 转发 | 尝试复现重复信号和历史退出；未复现不算修复 |
| B 单次投递 | 与 A 相同，只把 SIGINT 发给 noninteractive launch 父 PID | 单独检验重复信号的影响 |
| C 有序生命周期 | 与 B 相同；先 SHUTDOWN、检查两节点 FINALIZED，再给父 PID 一次 SIGINT | 检验生命周期收尾对退出结果的影响 |
| D 直接宿主对照 | 同样合成输入，直接管理 host/manager 子进程；有序 shutdown 后各一次 SIGINT | 区分 launch 转发与宿主/库析构问题 |

每项记录实际命令、环境、执行文件和库版本、生命周期状态、带时间日志、每个子进程
退出码、是否兜底、是否遗留子进程。C/D 的合格判据是两节点终态正确、host/manager
都退出 0、无强制终止；单纯日志没有 ERROR 或父退出 0 不足以通过。为 timing 问题做
少量重复；有限轮次正常退出只证明这些轮次，不能宣称消除所有竞态。

若仍为 −11，在同一隔离输入下保留 core 或取得 `thread apply all bt full`，结合
实际依赖库定位崩溃线程和对象；这是后续实验的一部分，本轮未启动调试器或抓取 core。
若表现为 −2，可在隔离环境跟踪信号接收及处理器恢复时序，再确认是否落在卸载处理器
后的窗口。A—D 本身没有真实运动，不能为 live 验收开门。

## 本轮边界及后续入口

本轮只新增本交接补充。未改功能代码、安装产物、测试、参数、验收标记和历史证据；
未启动实验、ROS 图查询、录包、SSH、Ranger 连接、桥接或运动目标；未跑全量测试，
未 stage、commit。文件核对仅检查链接、既有文件摘要和 Git 状态，不计为运行时验证。

下轮先读取本文件与 `session_handoff.md`，重新核实工作区；先安排上述隔离验证，
取得证据后再落地最小退出修复。−11 根因未闭环，完整协同 live 保持关闭。

## 新增授权后的隔离复现与修复

2026-10-08用户明确允许此任务必要的构建和ROS启动；本节为最新结论，前文“本轮未构建/
未实验”指此前只读阶段。只在Domain31、localhost、rmw_fastrtps_cpp使用合成map/TF/点云，
生产launch的Domain0覆盖未执行。初始图仅诊断probe，无其他节点；不接硬件或bridge。
证据全部保留在`build/go2-field/evidence/costmap-shutdown.o2f53dag/`：

| 对照 | 原宿主，各2轮 | 保持插件库生命周期的修复，各2轮 |
|---|---|---|
| A：killpg并由launch转发 | host=-11，manager=0，父launch=0 | host/manager/父launch均0 |
| B：仅父launch单次SIGINT | host=-11，manager=0，父launch=0 | host/manager/父launch均0 |
| C：两节点FINALIZED后仅父launch单次SIGINT | host=-11，manager=0，父launch=0 | 两节点FINALIZED；host/manager/父launch均0 |
| D：两节点FINALIZED后直接宿主/manager各一次SIGINT | host=-11，manager=0 | 两节点FINALIZED；host/manager均0 |

每轮两个costmap的合成障碍ROI均从254清为0；没有强制终止或遗留子进程。
原始结果：[matrix-results.json](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/matrix-results.json)；
修复结果：[fixed-matrix/matrix-results.json](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/fixed-matrix/matrix-results.json)。
另1轮[RESET→STARTUP→SHUTDOWN](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/lifecycle-restart/matrix-results.json)
确认重配后两个costmap重新标记/清除及正常退出；结束图仅probe，随后probe自身销毁。
运行二进制摘要、实际库路径和环境见两个矩阵目录的runtime-manifest.json及*-ldd.txt。

**已确认崩溃机制：**
[完整线程栈](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/backtrace/D-1/process-0.log)的主线程为
`Costmap2DROS::~Costmap2DROS → CallbackGroup::~CallbackGroup`。
[寄存器及映射](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/backtrace-mappings/D-1/process-0.log)
中故障指令为`CallbackGroup`析构的弱引用控制块虚调用，rax=`0x7ffff07bc1e0`；
该地址在cleanup前属于`liblayers.so`的`0x7ffff07ba000–0x7ffff07c4000`只读映射，
故障时GDB读该地址返回Cannot access memory。本机header中plugin_loader_声明晚于
callback_group_，逆序析构导致库先卸载。不能再将重复SIGINT写为已确认SIGSEGV根因。

只提前释放内部executor/callback group的中间尝试仍失败；
[其栈](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/first-fix-backtrace/D-1/process-0.log)
转到`NodeBase::~NodeBase → CallbackGroup::~CallbackGroup`，说明默认group也受此库生命周期影响。
最终[costmap_acceptance.cpp](../../cooperative_ws/src/multi_agent_nav/src/costmap_acceptance.cpp)
在原生cleanup前把已加载库交由已有`class_loader::MultiLibraryClassLoader`保留；
该所有者声明在executor/nodes之前，后于它们销毁，库可在弱引用释放后正常卸载。
保留原生生命周期、所有参数及context处理；不修改系统Nav2、共享run_ab.stop_owned或历史脚本。
上游顺序依据：[Nav2 1.1.20 cleanup](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp#L307-L324)、
[rclcpp 16.0.19 callback group](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/callback_group.cpp#L36-L39)；崩溃结论依据为上述本机动态证据。

**验证与产物：**新增SIGINT/生命周期shutdown退出断言在旧安装和独立重建原代码上均
2 failed（host=-11、manager=0）；最终局部测试文件7 passed（5项配置/静态检查、2项原生
合成输入与退出检查），见[JUnit](../../build/go2-field/evidence/costmap-shutdown.o2f53dag/green-retained/junit.xml)。
只构建multi_agent_nav，vlm_nav和任务消息复用旧安装；新产物在此证据目录isolated-build/install，
未覆盖现场安装。构建和局部测试复现命令统一在[runbook](go2_runbook.md#costmap-退出修复的独立构建与局部验证)。
最初默认ROS日志目录只读、第一次pytest缺父目录的准备错误也已保存；后续改用独立可写目录，
不把这两项准备错误当作退出回归失败。

**剩余限制：**重复信号路径仍存在，未来静止入口仍应使用单次父launch信号、有序SHUTDOWN、
实际FINALIZED和子进程退出判据。上述有限隔离轮次不保证所有信号竞态消失；历史−2的精确
信号时序尚未确认。现场旧二进制未更新，Domain0/CycloneDDS及真实静止退出未复验；
未重跑全量测试、SSH、Ranger、bridge、录包、运动或提交，完整协同live继续关闭。
