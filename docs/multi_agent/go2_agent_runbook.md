# Go2 RoboRSI / Codex CLI 可选入口

2026-11-03 接入实现及离线验证。原日常入口仍为 `scripts/start_go2_vlm.sh`，只有
`scripts/start_go2_agent_task.sh`（等价 `--agent-framework`）启用新后端。
留在 `go2-cooperative-task-layer` 原目录，不切分支、不合并、不提交。
本次不运行真实模型、ROS 节点、ARM 或运动；下述现场阶段尚未验收。

## 实现与版本

宿主继续负责原硬件准备、定位确认、owner 检查、人工 ARM、DISARM 及读回。
`Launcher.check()` 在搜索派发处生成同一份 argv；legacy 直接运行，Agent 通过私密 stdin
管道把 argv 和原环境交给独立框架进程。管理器仅追加原有 `--output-dir`，不重写算法或参数。
ROS 命令仍是原 `ros2 run multi_agent_nav multi_agent_go2_check`，安装入口 shebang 为
`/home/isee-pst/venv/co-nav-real/bin/python`（3.10.12）。目标空格、引号、中文原样传递，`shell=False`。
搜索 300 秒、NAV_READY 240 秒、ABC、内部重试、地图、Keepout、靠近及停止标准保持。

框架仓库 `/home/isee-pst/unitree_ros2/SenseNova-RoboRSI`，HEAD
`0208cd72ec7c885a7219040cd2be459d127e374e`，使用其 `.venv/bin/python`（3.10.12，pytest 9.2.0）。
预检拒绝框架 HEAD/工作树/导入来源变化。未升级或安装依赖。
Codex 配置入口 `/home/isee-pst/.local/bin/codex`，精确版本 `codex-cli 0.166.0`；解析为
`/home/isee-pst/.codex/packages/standalone/releases/0.166.0-x86_64-unknown-linux-musl/bin/codex`，
本次 SHA-256 `b8a11fedf24b2dfb59a70c9e52b030f9a508dcd49cd2dd0d41956f4c785156b8`。
每次预检记录真实路径和摘要，不回退其他 CLI，也不把历史摘要永久钉住导航算法。

`tools/go2_agent/config.yaml` 固定 `gpt-6-astra` / `high`、`max_turns: null`、
`max_tokens: null`、`planner_timeout_s: 1200`、`action_timeout_s: 600`。
固定版本的 `LocalToolSpec.handler` 不经过 action timeout；取消由任务管理器实现。
使用 `EmbodiedAgent.run(skills=[.../SKILL.md])` 静态加载一个搜索技能。

| 工具 | 唯一参数 | 行为 |
|---|---|---|
| `start_find_target` | `target: str` | 必须与宿主原文相同；持久化消费后至多启动一次 |
| `get_task_status` | `task_id: str` | 串行查询，服务端等待约 1 秒，读取原文件 |
| `cancel_task` | `task_id: str` | 幂等请求 SIGINT，不能据此宣称已停车 |

另有框架 `finish` 工具，模型自报不决定任务成功。工具 schema 不暴露 argv、环境或阈值；
handler 也拒绝额外字段。相同目标重复 start 返回原任务，不同目标拒绝；完成、失败、取消、
启动失败及重启读取已消费会话均不重放。启动前取消也消费本轮机会。

框架以 `-I` 启动，清除 ROS/Python/动态库搜索路径并设置框架 PATH；搜索收到原环境副本。
私密环境仅经管道和进程环境传递，不写 spec 文件；manifest 仅记非敏感环境白名单。
不改用户 `CODEX_HOME`、认证或全局 `OPENAI_BASE_URL`。CLI 工作目录在独立 `cli-work`，
使用 `--ignore-rules` 避免读取父仓库开发指引，sandbox 为 read-only、approval 为 never。
这些设置及提示词**不证明只能调用声明工具或禁止网络/机器人访问**；未验证原生工具完全禁用，
须在后续假机器人真实 CLI 阶段检查。至多一次由管理器硬约束，不靠提示词。

## 证据和成功判据

每轮 `build/go2-agent-runs/session-*/`，保留：

- `preflight.json`：框架/CLI 身份，不含认证值。
- `manifest.json`：原子持久化消费、PID/PGID/创建身份、状态、时间顺序、原事件 ID、原结果和错误。
- `search/`：原执行器创建的 `events.jsonl`、`result.json` 和视觉证据；不修改原报告。
- `search.log`、`adapter.log`、`framework/`：原输出及框架证据。
- `agent-result.json`：框架收尾时的搜索结果，`host_cleanup=PENDING`。
- `host-result.json`：宿主接收后写入；只有原 DISARM 及读回成功后为 `DISARMED`。

该目录不在旧十份清理器的直接子目录中，且没有其 owner 标记，不受其清理。
原始 report 不包含外层 session ID；来源通过宿主独占新目录、启动时间、同一原 run_id 事件链、
目标、SESSION_END 与 report 的 result/stop 字段、视觉根路径关联，不伪造原 task/run/command ID。
未知值为 null。只在进程退出后采纳完整报告，并再读取一遍最终事件以覆盖退出竞态。

搜索成功要求退出码 0、原 `TARGET_APPROACHED`、`stop_confirmed is True`、
`evidence_write_failed is False`、`vlm_evidence_pending is False` 全部成立。
原动作 `CANCELED` 不单独否定成功；缺失/半写/旧报告和迟到视觉证据不能变成成功。
取消先受理可归 CANCELED，但仍保存原始结果；已完成结果不被晚到 cancel 覆盖。
整轮还要求无框架错误、宿主 DISARM 并读回 DISARMED；模型 finish 不能代替这些事实。
未启动就 finish 为 `AGENT_NO_EXECUTION`；提前 finish 或框架异常会取消仍在运行的搜索。

## 取消与保留的限制

在运行宿主的终端按一次 Ctrl+C。宿主只向自己创建的适配进程发一次 SIGINT；适配进程
核对搜索 Popen/PID/PGID/创建身份后向搜索独立进程组发一次 SIGINT。SIGTERM 受控退出也
进入同一取消流程，清理期间重复信号不会中断等待。搜索调用原 SIGINT 清理，最多等待 30 秒；
这不是原内部最多 5 秒的停止判断。宿主给适配收尾最多 65 秒，不套用 `Launcher.command()`
的 SIGTERM/SIGKILL 升级策略。搜索是 Codex 的同级子进程，不是 CLI 后代，框架只清理 CLI 树。

`STOP_UNCONFIRMED`、身份不符、未退出、缺少可信最终报告或原 stop 不为 true 时，
保留原锁和证据，不强杀、不自动 DISARM、不重新 ARM/重试，也不换入口再启动。
操作者按原现场急停和排障流程处置。日志 I/O 异常不阻止向已核对身份的搜索发送 SIGINT；
启动前无法持久化消费则禁止 spawn。宿主收尾失败时 `host-result.json` 保持 PENDING，不能当整轮成功。

仍未修复的原有边界：adapter 的 ABORTED/本地记录移除不总是原生终态；原 stop 判断不是独立物理证明；
SIGKILL、父进程崩溃、系统故障不保证 finally；原报告非原子且视觉证据有迟到限制；
现有 flock/owner 检查不是所有运动客户端的统一锁。没有新增 watchdog 或更改这些语义。
本轮 /proc 仅能看到隔离沙箱，未确认宿主当前是否有实验运行，也未触碰现有运行进程。

## 可复制的离线检查

在控制电脑新 Bash 终端执行，目录均为 `/home/isee-pst/unitree_ros2/VLM_Nav`；无需保持终端。
以下不加载现场入口，不调用真实模型、ROS 节点或运动。每一步成功后再继续；失败查看输出，停止后续现场步骤。

1. 新适配测试：清除 ROS 的 Python 路径并关闭自动 ROS pytest 插件，使用框架 venv。
   成功判据为退出 0、所有用例通过；不往 co-nav-real 安装框架依赖。

   ```bash
   cd /home/isee-pst/unitree_ros2/VLM_Nav
   env -u PYTHONPATH -u PYTHONHOME PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
     /home/isee-pst/unitree_ros2/SenseNova-RoboRSI/.venv/bin/python -m pytest \
     test/test_go2_agent_session.py test/test_go2_agent_lifecycle.py test/test_go2_agent_runtime.py -q
   ```

2. 原入口及新增入口回归使用已有视觉依赖的 co-nav-real。成功判据为退出 0；框架 venv 缺 cv2，
   不应拿它运行这组旧测试。

   ```bash
   env -u PYTHONPATH -u PYTHONHOME PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
     /home/isee-pst/venv/co-nav-real/bin/python -m pytest \
     test/test_start_go2_vlm.py test/test_start_go2_cooperative_task.py -q
   git diff --check
   bash -n scripts/start_go2_agent_task.sh
   ```

3. 可选本地版本预检只调用 `codex --version`，不登录、不请求模型。成功判据为退出 0 且生成
   `preflight.json`；版本或框架身份不符时先排查，不换二进制、不自动升级。

   ```bash
   GO2_AGENT_CHECK_DIR="$(mktemp -d "$PWD/build/go2-agent-check.XXXXXX")"
   env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
     /home/isee-pst/unitree_ros2/SenseNova-RoboRSI/.venv/bin/python -I \
     tools/go2_agent/runtime.py --preflight "$GO2_AGENT_CHECK_DIR"
   ```

原独立执行与接近回归需原 ROS Python 模块可导入，但测试使用受控 transport、不初始化节点。
本轮在已有 ROS 环境终端执行下列命令，106 项通过；离线检查终端不需要保持运行：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:$PYTHONPATH" \
  /home/isee-pst/venv/co-nav-real/bin/python -m pytest --noconftest \
  cooperative_ws/src/multi_agent_nav/tests/test_standalone_go2_parity.py \
  cooperative_ws/src/multi_agent_nav/tests/test_single_robot_approach_parity.py -q
```

## 后续现场阶段（本次未授权执行）

顺序固定：先获得用户授权，以纯假机器人后端验证真实 Codex 工具调用、权限与取消；通过后再获得
单机现场授权，按[原 runbook](go2_runbook.md)完成硬件准备、急停人员、占用检查和人工 ARM。
仅“CLI 版本预检通过”不允许跳过假后端验证。

在控制电脑桌面交互 Bash 中，工作目录为仓库根目录，保留 RViz 显示环境。
由下面入口自动加载原 `CoVLM_env.bash`；确认活动 overlay 和上一轮停车/DISARM 后，**二选一**：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
# 原日常入口
scripts/start_go2_vlm.sh --target '灭火器'
```

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
# 完成假后端验收并获得单机授权后，显式选择 Agent
scripts/start_go2_agent_task.sh --target '灭火器'
```

启动器所在终端须保持运行直到收尾；相机和冷启动预检不再要求 CAMERA/START 确认；模型不能代答 READY/CURRENT/ARM 等后续人工确认。
`--prepare-only` 仍只准备，不触发框架预检、ARM 或搜索。两个入口不能同时下发运动。
成功判据为宿主打印 `TARGET_APPROACHED；停车已确认，Sport 已 DISARMED。` 且 Agent 的
`host-result.json` 对应同轮并为 DISARMED；随后才按原顺序关闭本次终端，复用进程由原终端管理。
失败或停止未知先按原流程处置，不盲目重跑。最后再比较两入口同目标、同配置的行为。

## 回退与以后修改共同探索代码

日常切回原入口：上一轮已结束并按原流程确认停车后，直接用 `start_go2_vlm.sh`。
框架未安装/预检失败不影响该默认路径，无需切分支、合并或删除 Agent 文件。
切入口不会恢复共享启动器的历史版本。若共享启动器回归，应比较本轮前完整快照，
仅撤销接入对应差异并保留原 dirty 和后续有效改动；本轮不自动执行恢复、不用 reset/clean/stash。

完整接入前快照位于
`/home/isee-pst/unitree_ros2/go2-agent-baseline-20261103-205731/`（权限 0700，含 staged/unstaged 二进制 diff、
未跟踪清单、dirty 文件副本及摘要；可能含本机配置，不上传）。
核对证据位于 `build/go2-agent-adaptation-20261103-20261103-205731/`。

以后有意修改探索源码，只维护当前 `vlm_nav` 与 `cooperative_ws/src/multi_agent_nav`。
按[原 runbook 的“可恢复暂停 overlay 构建”章节](go2_runbook.md#2026-10-29-可恢复暂停-overlay-构建)
在新输出目录共同构建两包（现安装为普通副本，改源码不会热更新）；构建成功后使用原环境机制
统一选择新的 `COOP_SETUP_FILE` 和配置，先完成安装来源/源码一致性及两入口离线回归。
停止旧任务并确认停车和 DISARM，再重启依赖该安装的相关进程；两个入口必须解析到同一安装。
失败时不切换到不完整产物。新版本建立新的共同摘要基线，不把 Agent 留在旧算法或旧 overlay。
本轮未改 ROS 包，因此依方案未重建或同步改写受保护安装副本。

## 本轮交付验证记录

新增适配 44 项、入口追加 12 项，共 56 项新增通过；原启动器 129 项和原独立执行/接近
106 项，共 235 项既有通过，总计 291 个不同用例。原入口组有 11 条既有第三方弃用告警。
初次把旧入口组放框架 venv 跑时有 8 项缺 cv2；在原 co-nav-real 重跑全部通过。
Python 编译、Bash 语法与 `git diff --check` 通过。
549 个受保护源码/配置/安装文件摘要未变，55 个 Python 源/安装配对及配置/launch 完全一致；
暂存区二进制 diff 未变，未修改其余已有 dirty 文件。本轮独立审查发现的一项退出事件竞态已修复并有回归。
离线通过不等于模型/真机通过；未跑全库、C++或真实模型/ROS/运动，也未消除上述停止证明缺口。
