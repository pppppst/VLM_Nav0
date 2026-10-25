# Go2 双机协同启动审查与修改说明

审查日期：2026-10-21。依据：[双机协同启动与验收](../双机协同启动与验收.md)。
审查对象为 Go2 侧入口、环境、实际安装、配置及其调用代码。Go2 已关机断连；本轮完全离线，网络不可用不判为脚本错误。

结论：此前缺失 `go2` 键的问题已修正。本轮没有发现需要再修改启动脚本的明确错误，只纠正一条过时的 Go2 配置注释。
原文档、启动顺序、命令、公共脚本、消息、DDS 路由和 Ranger 配置均未改动。
本结论只覆盖下述离线范围；实际双机协同仍受 [待确认清单](go2_startup_pending_20261021.md) 和真实验证项约束。

## 1. 基线与范围

- 分支：`go2-cooperative-task-layer`；HEAD：`4fc673156c5c985c4d9a9149cfe0b08b1582d90b`。
- 进入本轮前，`alignment.yaml`、`session_handoff.md` 已有上一轮修改；`maps/lab/lab.pgm`、`maps/lab/lab.yaml` 已删除。
- `Cooperative_env.bash`、`scripts/cooperative.sh`、双机文档原为未跟踪文件。本轮保留全部上述状态，不提交、不回退、不恢复旧地图。
- 预期使用方式为每台电脑由操作者逐终端启动，每个组件唯一；没有引入并行自动启动、后台管理或跨机部署。

核对的调用链：

`Cooperative_env.bash → CoVLM_env.bash → 各已安装 overlay → cooperative.sh → 安装版任务节点/检查器`。

硬件链另核对 `go2_camera_ssh.sh`、`run_go2_preflight.sh`、`common.sh`、Go2 system/navigation launch、
XT16/标定/AMCL/Nav2/costmap/Sport 配置；只读代码和资源，不执行硬件入口。

## 2. 修改说明

### M1 上一轮配置修复，本轮复核有效

位置：[alignment.yaml](../../cooperative_ws/src/multi_agent_nav/config/alignment.yaml)。

原有两个 `ranger:` 且没有 `go2:`；YAML 读取时后一个空条目覆盖前一个 Ranger 身份，
造成 G1/G2 的 `GeneratedCandidates` 和 G0 的 `live_gates` 都抛出 `KeyError('go2')`。

上一轮按实际 Go2 地图文件、`robot_go2.yaml` 和 HEAD 恢复本机身份，
并把用户填写的 Ranger ID/version 保留到唯一 Ranger 条目，`transform` 保持 `null`。
本轮没有再改该文件。Ranger 身份是否与远端当前配置一致仍待用户确认。

### M2 本轮唯一新增的配置文件修改

位置：[robot_go2.yaml](../../cooperative_ws/src/multi_agent_nav/config/robot_go2.yaml)，`generated_candidates` 下的注释。

- 原注释声称必须先采集新地图家具标注。
- 现改为：家具标注可选，没有该文件时仍能使用已知空地候选。
- 原因：当前 `GeneratedCandidates` 已实现可选家具标注，本机没有该文件也实际生成 518 个候选；旧注释与实现不符。
- 影响：只改一行注释，YAML 解析后的配置值不变；不改变策略、门控、地图、参数、话题或启动流程，无需构建。

新增交付文件为本文及 [待确认清单](go2_startup_pending_20261021.md)，并在会话交接追加索引。

## 3. 已确认的离线事实

| 环节 | 已核实结果 |
|---|---|
| 环境与 overlay | 真实 source 链最后选择 `build/go2-independent-c-20261020/install/multi_agent_nav`；底层包来自 `build/go2-manual-takeover-20261015/install/vlm_nav`。仅替换网络探测命令，没有改变环境文件 |
| 源码与安装 | 当前协同安装 27 个 Python 文件、底层安装 24 个 Python 文件均与源码逐字节相同；相关 8 个安装 launch/配置资源也相同 |
| 运行解释器 | agent 安装入口使用 `/home/isee-pst/venv/co-nav-real/bin/python`；ROS、NumPy、Pillow、OpenAI SDK、httpx、OpenCV 均可导入，未调用模型 |
| 配置选择 | 默认使用 `cooperative_ws/src/multi_agent_nav/config`；显式配置目录经过 source 后继续传给 agent/check；未误读安装目录中的模板配置 |
| 本地 DDS | 普通 Go2 节点 Domain0，`enp4s0`，CycloneDDS；桥的 Domain0/40 分别绑定 `enp4s0`/`wlo1` |
| 桥路由 | Go2 state/observation/command 为 0→40，Ranger state/observation 为 40→0；Ranger 文件方向相反；均 Reliable/Volatile；没有增加图像、TF、costmap、速度或 action 路由 |
| 地图身份 | 本机为 `lab_rescan_20261018_155656_0pa6gm`，实际地图摘要为 `6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279`；robot 配置与 alignment 版本一致 |
| 候选输入 | 家具文件可缺失；当前实际生成 518 个静态候选；不代表现场动态可达 |
| 策略选择 | YAML 的 `exploration_strategy: fixed_viewpoints` 仍供原单机默认入口使用；协同 `AgentRunner.start_local_execution` 会在本机配置副本中选择 `generated_candidates_vlm`，不是漏改配置 |
| 私有 C | `share_c_coordinates=false`；实际 Go2 接受 `action=SYNC_LOCAL_C, kind=WAIT`，不接受旧 `action=SYNC, kind=SYNC_LOCAL_C`；后者不是当前 Go2 coordinator 的发送格式，不据此改协议 |
| agent/coordinator | 显式 `simulation=false`、`topic_prefix=/multi_agent` 和配置路径传入正确；真实 gates 非空时 agent 不创建运动 adapter；coordinator 启动不自动 start |
| 重复实例 | bridge 的带后缀节点、agent、coordinator 重复启动均拒绝；已有 `vlm_nav` 或独立探索 motion owner 时 agent 入口拒绝；不结束已有进程 |
| start/pause | 当前 gates 阻止 start；pause 分支不调用 live 预检。但所有动作仍先加载基础网络环境，网络环境失败时软件 pause 入口也可能提前退出，仍按原文实体急停兜底 |
| Nav2 加载顺序 | 默认参数→Go2 参数→AMCL 静态层覆盖→协同 costmap 最终覆盖；最终全局含 obstacle_layer，局部含 voxel_layer，动态层没有被 AMCL 覆盖掉 |
| 有效 Nav2 参数 | 全局 frame=map、局部 frame=odom、base_link；两层膨胀半径均 0.45m；控制到达容差 0.25m、规划容差 0.5m；速度输出经 `/cmd_vel_bridge` |
| 到达/退出语义 | 协同 runner 使用 1.00m 并要求当前导航成功、终态和停稳；单机仍使用原合同；独立 C 及无 EXIT 已有相关回归通过 |
| Sport | 底层 launch 使用 `start_bridge:=false`，由 G-H4 单独启动；配置默认 DISARMED，命令只覆盖 `dry_run:=false`，不会自动 ARM |
| 预检解释 | 环境变量完整性检查本轮为 true，仅表示变量齐全；4 个真实 gates 保持关闭；断连时基础环境拒绝加载符合现有前置条件，不做绕过 |

当前 4 个阻塞项：`vision_validated`、`scene.enabled`、`path_source_verified`、`peer_obstacle_verified`。
本轮未调整这些标志，也未因为离线检查通过宣称可以开始真实任务。

## 4. 验证结果与证据

证据目录：[build/go2-startup-review-20261021](../../build/go2-startup-review-20261021/)。该目录不受 Git 保护，不清理。

| 检查 | 本轮结果与界限 |
|---|---|
| 入口分支检查 | 22 项 assert 检查通过。真实环境/overlay 和预检 Python 体执行；`ip`/`sysctl` 是替身，`ros2` 是只记录参数的替身，check 分支记录调用而不采样 |
| YAML 检查 | 17 个相关配置无重复键；Go2 地图版本/摘要一致；XT16 正式预检配置可加载 |
| 安装核对 | 协同 27、底层 24 个模块及 8 个资源与源码一致；未初始化 ROS |
| 实际桥配置解析器 | 已有 `check_bridge_routes` 二进制成功解析 Go2、Ranger 两个配置的五条定向路由；不创建 bridge 节点 |
| 既有回归 | **187 passed、1 failed**，共 188 个既有 pytest 用例；未新增 pytest 用例，22 项入口断言另计 |
| 失败来源 | `test/test_obstacle_chain_config.py::test_every_python_test_is_registered_with_ament_cmake_pytest` 的固定白名单漏列已注册的 `test_check_multi_agent_comm.py`、`test_go2_camera_tf.py`。CMakeLists.txt 和该测试均与 HEAD 相同，是既有测试断言过时；不删除注册项，也不扩展本轮修改范围 |
| 静态检查 | 8 个相关 Shell 文件逐个 `bash -n`；配置注释修改前后 YAML 数据一致；共享环境、启动脚本、双机文档及 alignment 摘要相对本轮基线不变；`git diff --check` 通过 |

回归文件范围：`test_covlm_env`、`test_check_multi_agent_comm`、`test_go2_profile`、`test_go2_bridge_config`、
`test_obstacle_chain_config`，以及协同包的 `test_live_gates`、`test_generated_candidates`、`test_local_tasks`、`test_local_c`、`test_coordinator`。
测试入口禁止真实 `rclpy.init`；通信测试内的 ROS 对象为替身。

主要机器证据：

- [entrypoint-audit.json](../../build/go2-startup-review-20261021/entrypoint-audit.json)：22 项入口检查及捕获的参数。
- [configuration-audit.json](../../build/go2-startup-review-20261021/configuration-audit.json)：安装、地图、最终 costmap、依赖及缺失文档链接。
- [preflight.json](../../build/go2-startup-review-20261021/preflight.json)：实际预检输出，只有 API 环境是否完整，不含密钥。
- [source-tests.xml](../../build/go2-startup-review-20261021/source-tests.xml)：187 通过、1 既有失败。

## 5. 交付边界

保留原文档流程继续使用；本轮没有需要分发到 Ranger 的启动脚本补丁，也无需重建或部署。
先审核待确认清单，尤其是两端目标、地图身份和真实验证项，不能把本轮离线审查当成现场验收。

未执行机器人/相机 SSH、Ranger 连接、ROS 节点启动、真实通信采样、传感器预检、录包、模型请求、ARM、导航或运动。
未重跑全仓测试、未重建 C++ 程序，未验证当前真实传感器、活动地图、costmap/TF 时效、避障或完整双机任务。
