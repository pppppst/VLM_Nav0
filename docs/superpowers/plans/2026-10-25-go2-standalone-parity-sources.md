# Go2 独立入口对齐：来源、实施与验证

> 以下保留10月25日迁移时的历史来源与验证。10月27日已按[五项最小修改](../../multi_agent/go2_runbook.md#2026-10-27-五项最小可靠性修改)有意改变停稳、感知等待和扫描退出语义；不再宣称这三项与冻结旧版完全一致。

本轮规格来自用户附件 `/home/isee-pst/Downloads/2026-10-25-go2-standalone-parity.md`。
原迁移来源记录见 [单机接近来源表](2026-10-25-go2-single-robot-parity-sources.md)。
实施目录 `/home/isee-pst/.codex/worktrees/a897/VLM_Nav` 为已有隔离 worktree，detached HEAD；
原仓库 `/home/isee-pst/unitree_ros2/VLM_Nav` 的 `go2-cooperative-task-layer` 与本轮起点均为
`1acae1d56e4e43dac29514f2244d439b760520bf`。起始工作区干净，结束 HEAD 未变，修改未提交。
未同步、覆盖原工作区或旧安装，未切换运行中终端。

## 固定来源与接口变化

| 固定来源 | 本轮位置 | 仅接口/生命周期适配 |
|---|---|---|
| `1acae1d56e4e43dac29514f2244d439b760520bf` `runner.py::local_approach` | `local_approach_execution.py::LocalApproachExecution.run`、`runner.py::local_approach` | 包装原执行器 tick、结果和 close；原始 odom 仍通过 adapter.approach 转交。协同任务身份/暂停/peer 检查留在 runner/machine |
| 同 SHA `agent.py::complete_local_execution` | `local_target_accepted` 及原函数调用点 | 原 nav_ready、localization_valid、stopped、motion_terminal、目标距离≤1m 的本地门槛共用；原内层0.81m成功证明仍必需 |
| 同 SHA `runner.py::start_local_execution`、`go2_exploration.py::explore_steps` | `go2_check.py::standalone_explore_steps` | initial_camera_check=True、current_view_only 分支原样复用；描述由独立 target 覆盖 scene A；独立无 peer/READY/coordinator，不进入 C |
| 同 SHA `runner.py::tick/motion_ready`、`agent.py::_monitor_path` 和 explore 的 health_check | 独立包装 check/motion_ready | 同一本地健康、所有者、停稳与动作终态检查；发送前和每轮轮询检查截止时间；扫描允许属于当前 scan 的空 task_id；结束后 guard 失效 |
| 同 SHA `go2_exploration.py::health_check(navigating=True, approaching!=None)` | 独立注入 approaching，健康分支未改 | 观察点导航也沿用限定路径接收监护语义；Health.path_valid 不被修改，真实路径继续显示/诊断 |
| 同 SHA `adapters/go2.py::_motion/on_odometry` | adapter 原文件未改 | 接近原生回调直接交给执行器，不进入旧 results 队列；扫描/观察点导航仍使用原结果消费。取消迟到回调保持 token fencing |
| 同 SHA `runner.py` 与 adapter 的被动诊断告警策略 | 独立 record 与 single_robot_mode 接线 | 仅任务/动作/display 被动诊断异常告警；VLM evidence、result.json 写入等既有故障语义保留 |
| `32eab6244ec4b483807f8ed68e8667f5495874bb` `vlm_nav/vlm_navigator.py` 冻结方法 | 已有 `single_robot_approach.py`，本轮 diff 为零 | 原目标x/y和当前yaw、use_start=False、原目标导航；0.89/0.81、一次距离重试、3样本/0.5s、3s/5s、3.5m/120s及条件顺序不改 |

`TARGET_APPROACHED` 被动事件延后到外层验收通过后，防止内层成功、外层仍未停稳时误报。
执行器结束但外层未通过，只等待原门槛，不追加导航/重试。协同仍由 complete_local_execution
决定 READY/DEPOSITED；暂停、替换身份、缺 peer 都不能借独立包装解除门槛。

独立仍调用现有探索引擎。目标接近固定注入共享包装，旧 approach_attempt/standoff/预算/
候选否决/重复规划分支不可达；其他直接调用探索引擎的模式保留原分支。原配置格式校验保留。
没有增加第二份接近算法、规划器、硬件参数或订阅。

## 附件任务记录

- [x] Task 0：读取实际指引、交接及构建环境，确认两个固定 Git 对象、已有 worktree 和干净起点；保存基线。
- [x] Task 1：共享接近生命周期、外层验收函数、唯一结果消费者、进入/退出 odom 关联、取消迟到和等待测试。
- [x] Task 2：独立实际 explore_steps 初始图像分支、目标描述、轮询、扫描所有权、接近与完整退出接线。
- [x] Task 3：成对安装统一选择、显式覆盖、父子终端检查、实际模块及配置来源；help/prepare-only 回归。
- [x] Task 4：从新安装执行离线回归，隔离构建、受保护文件检查、语法与链接检查；交付文档。

生命周期测试使用真实 Go2Adapter/SingleRobotApproach，ROS transport、时钟和输入受控，不创建节点。
两包装在统一610s截止时间下，正常、范围内起步、空路径、规划失败、ABORTED、停止超距场景的
规范化规划参数、执行器事件、停车调用数、导航次数及最终成功值逐项相等。
另外覆盖目标未接受取消、规划迟到、外层等待取消、唯一结果消费者和执行结束后 guard 失效。
冻结差分直接从完整 Git SHA 提取原函数，补充默认3.50m/120s及0.03m/s、0.05rad/s边界。

## 实际验证

证据目录 `build/go2-standalone-parity-20261025/evidence/` 不受 Git 保护，须保留。
完整执行命令、加载与用户启动顺序见 [runbook 本轮章节](../../multi_agent/go2_runbook.md#2026-10-25-go2-独立入口对齐隔离构建)。

| 检查 | 本轮实际结果 |
|---|---|
| 改前18文件基线 | 553通过、4失败，exit 1，baseline.log |
| 扩大5文件原源码基线 | 137通过、2失败，exit 1，baseline-extra.log；在未修改的原仓库执行 |
| 从/tmp加载新安装，24文件最终回归 | 742通过、同6项旧失败，11条第三方弃用告警，exit 1；新增52项全部通过，final-installed.log/xml |
| 独立/协同与冻结源差分两文件 | 99通过，exit 0，differential.log |
| 成对隔离构建 | vlm_nav、multi_agent_nav成功，23.1秒，exit 0；build.log |
| 新输出 CTest | 2条目，对应原空间过滤6+时间过滤8=14个C++用例，全部通过，exit 0 |
| 安装来源 | 仓库外55个Python模块位于新前缀且逐字节匹配；根config及arrival合并顺序匹配，exit 0 |

六项旧失败完整名称与原因列在 runbook，没有 deselect，没有修改旧门槛来使测试变绿。
本轮不能表述为全库通过；未执行完整仓库/Domain31节点/现场预检/真实模型或真机试验。
离线一致性验证不能证明真机接近成功、动态障碍消失或双机避碰已验收。

独立审查发现fixed_viewpoints/semantic_furniture后续扫描只有A、ABC客户端要求ABC的接口错配，
已用真实infer_abc（仅替换网络响应）先复现两项KeyError，再将approaching回调模式的描述集合
与原协同navigation回调模式一致；旧无回调模式不变。修复后新增真实客户端回归通过，最终全相关回归
仍仅六项原基线失败。没有未处理的重要审查项。新增父子窗口测试实际执行Bash重载旧环境，验证组件不会启动。


## 2026-10-27 同步后续协同更改

上文是2026-10-25冻结基线对齐的历史记录。用户随后要求单机与协同同步：现已将原仓库后来确认的移除接近3.5m半径终止、
避免低信息死胡同提示与19点Keepout合入共享代码，两个入口同时采用；超时/停稳/取消及外层验收保持。
因此当前版本有意不再与冻结源码的半径终止分支相等，对应测试改为证明长距离可继续、到达会停止、超时仍失败。
正式源码和新成对安装在原仓库，启动/验证详见[runbook](../../multi_agent/go2_runbook.md#2026-10-27-单机与协同同步安装)。
