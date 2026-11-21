# Piper 独立蓝色积木视觉抓取

## 六关节回零脚本（2026-11-18）

正式入口为 [scripts/piper_return_zero.sh](../../scripts/piper_return_zero.sh)。它调用现有控制程序的 `zero` 动作，
读取真实关节反馈，从当前位置按最大单轴名义角差10°分段回到 `[0,0,0,0,0,0]°`；默认配置速度30%、
到位容差0.3°，每段六轴均到位才继续。只发送关节控制命令，夹爪保持，不改变零点标定，不自动使能或失能。
复用USB身份、控制锁、新鲜反馈、设备限位与持久命令记录；不需要相机、视觉模型服务或ROS节点。
分段关节插值不是碰撞规划，运行前须确认当前姿态到零位的路径和所夹物体有足够空间。

本地安装为 `build/piper-zero-20261118/overlay/install/vlm_nav/share/vlm_nav`，入口按自身目录加载本次tools/config。
2026-11-19已按用户授权部署到Go2独立目录 `/home/unitree/piper-zero-20261119`，44个清单文件哈希通过，机载离线检查 `CHECK_OK`。
原 `piper-vision-current` 仍指向历史r2，未覆盖；本次仅部署已验证的回零版本，未部署后续角度搜索改动。
部署初检USB-CAN缺失，连接恢复后第一轮 `joint-zero-20261119-01` 停在 `WAIT_TARGET`；用户再次授权后结束旧监测，第二轮 `joint-zero-20261119-02` 已 `ZERO_REACHED`/退出0。退出后12次独立反馈六轴全0°、READY，夹爪0mm保持。此次起始已处于CAN模式，首次待机切换后无动作的根因仍待验证；最新状态见session_handoff。
构建步骤见[主runbook](go2_runbook.md#2026-11-18-六关节回零脚本构建)。

下面是获准运动且USB-CAN连接恢复后的执行顺序。

1. Go2上位机Bash终端A，进入已核对包含新入口的实际部署目录。机械臂应已上电、CAN可用、由原boot服务使能，且其他控制程序已结束。
   已知机载解释器为 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`，依赖沿用现有Piper环境，
   不要使用开发机虚拟环境。先只读检查入口，自动退出、不连接CAN：

   ```bash
   cd /home/unitree/piper-zero-20261119
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_return_zero.sh --help
   ```

   成功判据：退出0且动作列表包含 `zero`。若脚本不存在或依赖缺失，先处理部署，不执行下一步。

2. 同一终端A、同一目录和环境，启动一次回零（此命令会运动）。保持终端直到完成或人工退出：

   ```bash
   bash scripts/piper_return_zero.sh
   ```

   入口只显示日志/结果路径；有桌面时自动打开诊断终端。无桌面时，在上位机第二个Bash终端B、同一部署目录执行
   `bash scripts/piper_return_zero.sh diagnostics`，保持运行并查看 `state`、`reason_class`、`reason`、`stage` 和 `measured`。
   成功判据为结果 `ZERO_REACHED`、六轴反馈均在0.3°内且终端A退出0；终端B可随后Ctrl+C结束。
   `WAIT_*` 表示等待恢复/到位，不要并行重启另一个回零程序；`COMMAND_UNCERTAIN` 保留现场等待人工处理，不自动重发。
   Ctrl+C只退出程序，不撤销驱动中已有目标，也不是物理急停。每次运行自动创建独立目录；显式重复 `--run-id` 会拒绝启动。
   诊断存储暂时故障时保持等待，恢复后继续记录 `WAIT_RUNTIME`，不重启运动序列；若存储持续故障时退出，结果文件可能无法落盘。

> 当前选择（2026-11-18）：按用户要求恢复启发式语义抓取，默认配置为 `semantic-camera-avoid.json`，六轴30%。当前包部署及入口见 [runbook](go2_runbook.md#2026-11-18-恢复启发式默认入口并部署30速度)。GraspNet旧包/记录保留，不用于当前默认流程。

> 2026-11-18：源码及本次安装包的四个抓取配置统一改为六轴30%速度比例（原5%）；当前启发式包已部署30%，旧包与历史现场记录中的5%不变。最新构建/入口见 [go2_runbook](go2_runbook.md#2026-11-18-piper-六轴抓取提速构建)。30%尚未真机验证，比例提高6倍不等于总耗时缩短6倍。

本功能在**连接 RealSense 与 Piper USB-CAN 的 Go2 上位机**运行，源码与安装产物均来自 VLM_Nav。它不启动导航、ROS节点、桥接或旧视觉Agent，不调用VLM/SAM，不加载外部Piper工程源码。

首版交付是离线实现与验证；开发端没有SSH、部署、连接设备或真机试验。后续用户已自行部署到 `/home/unitree/piper-vision-grasp-20261114`，提供的机载日志确认 `CHECK_OK`；`observe`遇到全局RealSense context类型错误，修复和更新步骤见文末。历史成功的是有人看管、人工方向微调后的夹取，不能当作本功能自主验收。现场实际安装目录和解释器仍须在部署对话核实。

## 算法与适用范围

流程为有限RGB-D采集 → HSV蓝色分割与连通域 → 完整可见面的边界及内部深度平面 → 三帧中心中位数 → 相对参考位移 → 固定朝向的局部路径 → 夹持候选 → 抬升20mm → 物体视觉验证。

参考是重新立起积木后、闭合前的 `blue-block-after-retreat-rgbd01`，同位置 `blue-block-half-horizontal-rgbd01` 用于交叉检查；参考法兰由成功夹持前六关节与6轴URDF FK计算。二者来源、时间和哈希在 `config/piper_vision_grasp/reference.json` 与 `provenance/`。测试RGB-D在 `test/fixtures/piper_vision_grasp/`，原build证据保留。

`delta_base = R_base_color_optical × (point_now - point_reference)`，只将这个差分加到成功法兰位置，朝向保持成功姿态。没有叠加90mm/135mm TCP。外参平移在差分中抵消，外参旋转误差仍影响结果；`manual04`的物理验证状态仍为 `NOT_PERFORMED`。

首版仅适用同一个蓝色长方体、同一支撑面和朝向、相同相机/机械臂安装与零点、静止Go2。配置范围为基座X/Y各±30mm、Z±10mm，建议首次试摆约20mm；超出是本轮参考范围不足，不是Piper绝对不可达。安装、站姿、零点或物体朝向改变，需要重新建立参考；不要把重复校零当启动步骤。

检测参数由真实样本确定：OpenCV H95..130、S≥160、V≥30，非固定像素ROI。初始H90..140/S≥60包含背景青色区域，因此收紧颜色阈值。参考估计可见正面约44.7×118.2mm，两组同位置参考的中心约相差0.11mm；这不是绝对标定精度。三帧稳定性3mm、朝向变化5°是本轮算法参数。多目标、裁切、遮挡、深度不足或非零畸变不生成抓取点。

规划包含历史空载展开停靠点、参考预抓上方60mm的入口、局部预抓、约10mm间隔接近及抬升。局部IK分支从成功姿态向上建立，再用空载关节过渡连接，避免从展开姿态选到受限的另一腕部分支。每一段记录端点误差、关节变化和假设关节插补的法兰采样。位置误差≤1mm、旋转误差≤1°只是数值判据，**未验证连杆/夹指/Go2护架/支撑物碰撞，也不保证SDK按笛卡尔直线运动**。

## 入口与产物

唯一入口是 `scripts/piper_vision_grasp.sh`。源码与安装share都保留同样的 `scripts/`、`tools/`、`config/` 相对布局。解释器由 `PIPER_VISION_GRASP_PYTHON` 指定，未指定时使用 `/usr/bin/python3`；不source导航环境。

| 命令 | 行为 | CAN发送 |
|---|---|---|
| `check` | 离线校验配置、来源和依赖版本，结束 | 无；不连接设备 |
| `observe` | 被动读取反馈、暖机、有限采3帧并保存，结束 | 0 |
| `preview` | 新鲜有限采集、被动反馈、纯规划，结束 | 0 |
| `preview --input CAPTURE_DIR` | 仅读存档图像、元数据与其中的关节状态，离线规划 | 无；不连接设备 |
| `run` | 一个周期：开爪、接近、闭合、抬升、视觉核验，结束后保持目标 | 仅本周期限位查询及明确目标 |
| `diagnostics` | 跟踪当前事件日志，保持运行 | 无 |

真实 `run` 不接受 `--input`，不会使用离线样本控制硬件；不接受一个旧preview当作执行输入。开爪后会重新采集和规划，避免开爪等待导致图像过期。断流/故障恢复后、后续运动前重新定位；若目标已移动，则 `WAIT_REPLAN` 保持，待恢复现场或退出并核对已执行阶段后重新开始，不自动回放旧路径。

默认产物目录为 `~/.local/state/piper-vision-grasp/<run-id>/`。可用 `--output-root` 指定独立目录，所有命令和诊断终端应使用同一个根目录。未指定 `--run-id` 时自动生成唯一值；重复ID在连接硬件前被拒绝，不能覆盖旧结果。

每次保留 `events.jsonl`、`runtime.log`、`result.json`，视觉阶段的PNG、原始及对齐深度NPZ、每帧内外参/尺度/时间域/frame number、前后静止反馈、`observation.json`和检测标注；规划含 `plan.json`；抓取验证含 `verification.json`。命令的 `attempts/<stage>.json` 在SDK写入前持久化。同一USB全会话持有既有锁 `~/.local/state/go2-piper-gripper/control-<serial>.lock`，不与独立手动夹爪程序争用控制。

主入口终端只显示日志/结果路径；诊断字段有 `state, reason_class, reason, stage, target, measured, command_attempted, frame_age, reference_id, run_id`。SDK等第三方输出进 `runtime.log`。有可用图形环境时自动打开GNOME诊断终端；无图形环境会给出第二终端命令。

## 开发机离线使用

**开发机 Bash，目录 `/home/isee-pst/unitree_ros2/VLM_Nav`**。构建、定向测试、Python环境和安装入口检查的完整命令只在 [主runbook](go2_runbook.md#2026-11-14-独立-piper-蓝色积木视觉抓取构建与离线验证) 维护，按那里顺序执行；不需要保持终端运行。

首版独立安装为 `build/piper-vision-grasp-20261114/overlay/install/vlm_nav/share/vlm_nav/`；context修复版安装为 `build/piper-vision-frames-20261114/overlay/install/vlm_nav/share/vlm_nav/`，首版产物保留。开发机检查解释器位于 `build/piper-vision-grasp-20261114/check-venv/bin/python`，仅供离线检查，不能复制给机载ARM系统。

最小分发包准备在 `build/piper-vision-grasp-20261114/package/` 及同级 `piper-vision-grasp-20261114.tar.gz`，从成功安装产物提取，仅含本功能模块、独立入口、复用的boot只读helper、配置及必要来源、使用说明与SHA256清单。不含虚拟环境、导航栈、外部Piper源码和模型权重；本轮不部署。

## 后续现场操作顺序

下列步骤是后续获准部署/现场试验后的操作说明，本轮未执行。运行机器始终是**Go2上位机**，不是当前开发机；不要将开发机绝对路径直接照搬到Go2。

1. **终端A，Go2上位机Bash，进入实际部署根目录。** 部署时记录目录并设置 `PIPER_VISION_ROOT`；它应包含本手册所述三个相对目录。已知候选解释器为 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`，先只读核实实际存在和版本，缺依赖时停止并按实际环境补齐，不升级导航环境。

   ```bash
   : "${PIPER_VISION_ROOT:?请先设置机载实际部署根目录}"
   cd "$PIPER_VISION_ROOT"
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   "$PIPER_VISION_GRASP_PYTHON" --version
   sha256sum -c SHA256SUMS
   bash scripts/piper_vision_grasp.sh check
   ```

   目的：核对安装完整性及离线依赖。预期文件全部OK、命令退出0、诊断为 `CHECK_OK`。计划时机载环境为Python3.8.10/NumPy1.17.4/SciPy1.3.3/OpenCV4.12/pyrealsense2 2.55.1.6486/Piper SDK0.6.2/python-can4.5.0；本轮未远程复核。`check`不证明设备就绪。此步骤完成后终端A可继续操作。

2. **终端B，同一机器、根目录和解释器环境，保持运行。** 若没有自动诊断窗口：

   ```bash
   cd "$PIPER_VISION_ROOT"
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_vision_grasp.sh diagnostics
   ```

   目的：查看后续所有阶段和等待原因。诊断默认跟踪 `current.log`，新命令会切换到新的运行日志。若用了自定义 `--output-root`，这里也要带相同选项。退出这个终端只结束日志查看。

3. **终端A，完成物理准备后采集。** 使用RealSense `151222077264`、USB-CAN `0046002F5246570520323934`；CAN须已UP/1Mbps/ERROR-ACTIVE，机械臂处于正常非示教状态且六轴已使能、无故障。本入口不会调整CAN或使能六轴。没有其他客户端发送命令；Go2静止，蓝色积木重新立在同一支撑面、同一面朝向相机，无需精确还原位置。

   ```bash
   bash scripts/piper_vision_grasp.sh observe
   ```

   目的：取得本次真实目标与被动反馈。预期保存3个不同帧，目标完整、深度有效、状态 `OBSERVED`、CAN发送0，随后退出。若等待，按 `reason` 恢复输入，程序重新采集；不要把历史参考照片当新现场证据。手应离开夹指活动区。

4. **仅在步骤3正常后，终端A执行预览。**

   ```bash
   bash scripts/piper_vision_grasp.sh preview
   ```

   预期 `PREVIEW_READY`、CAN发送0，产物含目标三维点、相对位移、法兰目标、每段关节目标/残差及未验证碰撞项。超范围、姿态变化或无解按原因处理，不直接执行失败计划。命令正常后结束，不需保持运行。

5. **只有后续对话明确现场试验范围后，终端A执行一个周期。** 现场看管者先检查新过渡段与周围物体；本功能不要求逐段回答移动方向。

   ```bash
   bash scripts/piper_vision_grasp.sh run
   ```

   程序重新获取目标，以30%关节速度、70mm/0.5N·m开爪、36mm/0.5N·m闭合，形成夹持候选后抬升20mm，再看物体实际视觉位移。正常完成输出 `GRASP_CONFIRMED`，保持关节和夹爪目标，不自动释放、回零、失能或再抓。等待状态时保持终端A，诊断终端B持续显示原因；接管/退出用Ctrl+C，**只结束程序，不是物理急停，不撤销驱动中的既有目标**。

6. **重复性验证另按现场授权进行。** 每次重新立好目标，小范围改变位置，使用新run-id；不依赖人工方向修正。记录2～3个摆位的成功、等待原因和任何人工干预。一次通过只报告该次，多位置结果才支持局部重复性；放置、携物导航和任意物体/朝向不在本轮范围内。

## 状态与失败分支

| 状态 | 含义与继续方式 |
|---|---|
| `WAIT_IO` / `WAIT_FEEDBACK` | 接口或数据缺失/陈旧，修复连接后等待新反馈；不自动重配CAN或重发运动 |
| `DISABLED` / `WAIT_CONTROL` / `WAIT_FAULT` / `WAIT_GRIPPER` | 现有使能/控制状态或故障未恢复；查看原因并由现场操作者处理，不校零/清错/加力 |
| `WAIT_TARGET` / `WAIT_OPEN` | 目标尚未满足，持续监测；超时不当成功、不重发 |
| `WAIT_TARGET`（视觉）/ `WAIT_AMBIGUOUS` / `WAIT_VISIBILITY` / `WAIT_DEPTH` | 无可靠单一完整目标，恢复可见性/深度后重新定位；不沿用旧图像 |
| `WAIT_PLAN` / `WAIT_IK` | 本轮局部范围/姿态/数值求解不成立；查看原因，不擅自扩大范围 |
| `OPEN_READY` | 新鲜、使能且稳定开口≥66mm；底层±1mm到位判据仍如实记录，68mm不会被改写为70mm到位 |
| `CONTACT_CANDIDATE` | 稳定开口已明显变小且高于空闭目标，无故障并使能；仅为接触候选 |
| `WAIT_CONTACT` | 可能夹空、未闭合或尚不稳定，不抬升、不自动再夹 |
| `WAIT_VERIFY` | 物体未观察到稳定上升10..30mm，或形状/可见性不足；保持并继续有限采集，不靠SDK上升宣布成功 |
| `WAIT_REPLAN` | 运动后的恢复采集中目标已变化；保持，恢复现场或退出后核对已执行阶段，不回放旧轨迹 |
| `COMMAND_UNCERTAIN` | 部分写入或历史尝试记录导致结果不明；保持监测且不继续后续阶段。先核实现场，禁止通过重用run-id重放 |
| `WAIT_RUNTIME` | 未预料的运行错误，保留阶段及日志，保持监测，不盲目重启整个周期 |
| `GRASP_CONFIRMED` | 自动视觉证据支持同一完整蓝色目标抬升并稳定；与操作者补充观察分别记录 |

历史 `blue-block-grasp-hold-rgbd01` 只有一帧且夹指遮挡顶部，自动验证不能用它写成功。新的真实试验也可能因夹指遮挡进入等待，不能把这类等待改成“机械臂抬高即成功”。

## 2026-11-14 observe 全局RealSense context解析修复

用户机载诊断已确认CHECK_OK，随后observe反复WAIT_VISION，原因 `global config-file/context: expecting an object; got ""`，command_attempted=false。历史交接已记载机载 `.realsense-config.json` 的context字段为空字符串，手眼采集用inherit=false规避；独立视觉入口原先默认构造pipeline，遗漏了这条已验证适配。修复只在 `capture_rgbd` 内显式创建 `rs.context('{"inherit":false}')` 并传给pipeline；不修改全局Viewer配置、CAN、抓取参数或既有日志。

开发机已准备补丁 `build/piper-vision-context-20261114/piper-vision-context-fix-20261114.tar.gz`，仅含安装产物中的vision.py、更新后的SOURCE_MANIFEST.json和SHA256SUMS。它适用于此前原版分发目录；若远端另有自定义修改，先核对再应用。开发端未远程部署或启动设备。

1. **Go2终端A**先Ctrl+C退出旧observe；终端B诊断可以保持运行。旧进程不会热加载代码。
2. **控制电脑另一个Bash终端，仓库根目录**传补丁（仅传输，不运行设备）：

   ```bash
   cd /home/isee-pst/unitree_ros2/VLM_Nav
   scp build/piper-vision-context-20261114/piper-vision-context-fix-20261114.tar.gz unitree-go:~/
   ```

   scp退出0后继续。
3. **Go2终端A，原部署目录**备份模块并应用补丁，无需ROS：

   ```bash
   cd ~/piper-vision-grasp-20261114
   cp -pn tools/piper_vision_grasp/vision.py tools/piper_vision_grasp/vision.py.before-context
   tar -xzf ~/piper-vision-context-fix-20261114.tar.gz
   sha256sum -c SHA256SUMS
   ```

   校验全部OK后执行check；失败先核对，不启动observe。

   ```bash
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_vision_grasp.sh check
   ```

   check退出0且诊断CHECK_OK后，再执行：

   ```bash
   bash scripts/piper_vision_grasp.sh observe
   ```

   预期不再出现该context解析错误，继续采集/定位；出现OBSERVED且命令结束才进入preview。其他WAIT原因需分别排查，不能把本次离线修复当作机载取图或检测已通过。

### observe：帧序号未递增

用户在context修复后报告 `WAIT_VISION: non-increasing camera frame numbers`，command_attempted=false。该旧日志没有前后帧号，不能区分重复和倒退。原代码每遇到任一路重复就退出整次采集、重新暖机；[RealSense接口说明](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/pyrs_pipeline.cpp)允许同步帧组复用较慢流的帧。现改为在同一pipeline内跳过重复帧组，只保存两路均严格递增的三组数据；每组等待最多3秒，倒退立即失败，错误包含previous/current（顺序为color/depth）。外层仍可等待恢复，不放宽三帧独立性或新鲜度要求。

本补丁已包含上一版context修复，仅更新vision.py及来源清单，不覆盖配置和日志。机载是否只是重复帧、修复后能否完成取图仍待复测。

1. **Go2终端A**先Ctrl+C退出旧observe，终端B诊断保持。旧进程不会热加载修复。
2. **开发机Bash终端**传输，退出0后继续：

   ```bash
   cd /home/isee-pst/unitree_ros2/VLM_Nav
   scp build/piper-vision-frames-20261114/piper-vision-frames-fix-20261114.tar.gz unitree-go:~/
   ```

3. **Go2终端A**，无需ROS，备份并应用；校验全部OK才继续，失败贴出失败项：

   ```bash
   cd ~/piper-vision-grasp-20261114
   cp -pn tools/piper_vision_grasp/vision.py tools/piper_vision_grasp/vision.py.before-frames
   tar -xzf ~/piper-vision-frames-fix-20261114.tar.gz
   sha256sum -c SHA256SUMS
   ```

4. **同一Go2终端A**，明确解释器，check结束且诊断CHECK_OK才继续observe：

   ```bash
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_vision_grasp.sh check
   ```

   ```bash
   bash scripts/piper_vision_grasp.sh observe
   ```

   保持运行观察终端B；OBSERVED且命令结束后才继续只读preview。若仍WAIT_VISION，提供新的完整reason，尤其previous/current；若变为WAIT_TARGET等检测状态，按新原因处理，不能据此声称抓取通过。

## 颜色、描述与自动尺寸：语义模式

用户2026-11-14选择接入旧Piper描述/SAM/关键点方法，明确“不增加门控和其他限制”，并授权本轮由开发端启动真机调用抓取。语义模式不套原蓝色HSV、物体尺寸范围、参考位置/朝向范围，不新增审批开关。保留现有反馈、帧独立性、IK、单次发送、故障等待和视觉验证。普通config.json仍为原蓝色模式；新入口使用 `--config config/piper_vision_grasp/semantic.json --color 绿色 --description 小盒子`。

描述和颜色共同进入模型请求，模型返回目标框和建议关键点，SAM提取轮廓；抓取位置使用有效掩码外接范围的中心及邻域深度，模型建议点仅留作诊断，避免其逐帧跳动；利用实际对齐深度内参计算点云，输出dimensions_base_m及grasp_width_m。尺寸是可见点云在基座各轴的范围，不保证完整隐藏长宽高；不要求用户填写物品尺寸。抓取沿基座-Z顶抓，根据框边缘的实际三维方向确定夹爪朝向，使用当前manual04外参的完整旋转/平移。法兰位置=抓取TCP位置−旋转后的工具偏移。

semantic.json中tcp_offset_m=[0,0,0.135]、grasp_offset_base_m=[0,0,-0.013]来自旧代码试验参数（-18mm与+5mm合并），不是当前已标定结果。预抓沿工具负Z退100mm，保持原20mm抬升/5%速度。用户现场反馈开爪需1–2Nm，语义配置现采用open_effort_nm=1.0（70mm开爪），effort_nm=0.5（闭爪）；未设置open_effort_nm的旧配置仍使用effort_nm开爪。闭爪目标为测得的夹取宽度减4mm且不小于0，不继续写死36mm；现有驱动宽度检查仍按真实设备处理。语义模式帧年龄配置120秒以容纳CPU/网络推理，实际接收时间保留，诊断frame_age包括推理耗时。模型未找到、掩码/深度不可用、接口异常均进入现有等待；不会执行模型生成的代码。

### 启动顺序

先完成[主runbook](go2_runbook.md#2026-11-14-piper语义识别及绝对抓取接入)的新构建和模型环境，下面每步达到成功判据后再继续。模型服务在开发机，控制程序在Go2；无需ROS。新分发目录以实际部署为准，本轮使用 `~/piper-vision-semantic-20261114`，旧目录和日志保留。

1. **开发机终端C**设置模型环境并启动服务，保持运行。已有服务MODEL_READY时不重复启动。凭证在终端静默输入，仅放环境变量：

   ```bash
   cd /home/isee-pst/unitree_ros2/VLM_Nav
   export PIPER_VISION_GRASP_PYTHON="$PWD/build/piper-vision-semantic-20261114/model-venv/bin/python"
   export PIPER_SAM_CHECKPOINT=/home/isee-pst/unitree_ros2/piper/agent/sam/sam_vit_b_01ec64.pth
   read -rs -p '模型API密钥: ' PIPER_VLM_API_KEY
   export PIPER_VLM_API_KEY
   bash build/piper-vision-semantic-20261114/overlay/install/vlm_nav/share/vlm_nav/scripts/piper_vision_grasp.sh serve \
     --config build/piper-vision-semantic-20261114/overlay/install/vlm_nav/share/vlm_nav/config/piper_vision_grasp/semantic.json
   ```

   诊断MODEL_READY说明SAM已加载，未证明云端识别成功；若WAIT_CONFIG/MODEL_LOADING不前进，按日志检查环境/权重，勿反复启动占同一端口。

2. **开发机终端D**建立SSH反向隧道并保持：

   ```bash
   ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=2 \
     -R 127.0.0.1:18765:127.0.0.1:18765 unitree-go
   ```

   命令保持运行且没有转发失败即继续。Go2重启会断开，需要重建；终止该终端只关闭模型访问隧道。

3. **Go2终端B**运行诊断并保持：

   ```bash
   cd ~/piper-vision-semantic-20261114
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_vision_grasp.sh diagnostics
   ```

4. **Go2终端A**指定解释器和新配置，离线check退出0且B显示CHECK_OK后再observe：

   ```bash
   cd ~/piper-vision-semantic-20261114
   export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
   bash scripts/piper_vision_grasp.sh check --config config/piper_vision_grasp/semantic.json --color 绿色 --description 小盒子
   ```

   ```bash
   bash scripts/piper_vision_grasp.sh observe --config config/piper_vision_grasp/semantic.json --color 绿色 --description 小盒子
   ```

   observe仅采集与被动反馈，OBSERVED且退出后才继续preview；若WAIT_IO先按reason恢复连接，WAIT_VISION看模型服务和深度原因。

5. **同一Go2终端A**只读规划；完成PREVIEW_READY后，在该次现场运动授权内run一次：

   ```bash
   bash scripts/piper_vision_grasp.sh preview --config config/piper_vision_grasp/semantic.json --color 绿色 --description 小盒子
   ```

   ```bash
   bash scripts/piper_vision_grasp.sh run --config config/piper_vision_grasp/semantic.json --color 绿色 --description 小盒子
   ```

   run重新采集，不重放preview文件；执行结束保持，不自动松爪、回零或再次抓取。等待态保留进程并读B原因。Ctrl+C结束当前程序不会撤销已发出的机械臂目标。

离线调用可加 `preview --input <capture目录>`，使用其配对机器人反馈，不连接相机/CAN；但语义识别仍会向上述模型服务提交该存档图像。离线图像通过不等于现场通过。

度量会按目标掩码内深度的四分位范围剔除稀疏远背景，防止SAM边缘把米级背景并入盒子尺寸；这是深度计算步骤，不是新增物体尺寸/运动门槛。输出保存maskNN.png和detectionNN.png以便现场核对。存档积木实测可见范围约18×47×119mm，夹取方向宽约47mm；18mm并不代表完整物体厚度。

### 接近方向现场调整

`--approach-pitch-deg`用于语义模式设置接近方向相对基座水平面的向下夹角，默认90°（垂直顶抓）；颜色/描述和度量位置不随角度改变，工具偏移按实际旋转重新换算法兰，预抓仍沿工具负Z退100mm。2026-11-14用户将目标改成红色方块；宽约49mm的目标TCP约[664.6,35.1,-147.2]mm，90°顶抓不可达，离线对同一现场数据计算30°斜向接近可通过现有IK和采样。该结果不是碰撞或实物夹持验证。

同一Go2终端A、同一新部署目录、解释器与模型隧道已就绪时，本轮经用户明确授权的调用为：

```bash
bash scripts/piper_vision_grasp.sh run --config config/piper_vision_grasp/semantic.json \
  --color 红色 --description 方块 --approach-pitch-deg 30
```

这是实际运动命令，会重新取图规划；不要与已有run同时启动。完整现场结果见交接，不从此文档推导未来运动授权。

现场到位判据：SDK `motion_status` 表示固件是否判定到达目标，不是关节速度。2026-11-14现场出现六关节最大误差0.207°、位置持续不变，而该字段长期为1；入口按新鲜六关节反馈与原0.3°容差判断到位，保留该字段供诊断。拍照前后仍检查关节变化不超过0.3°，闭爪仍检查稳定开口和接触候选，故障/失联仍等待恢复。`AT_TARGET`不代表物体已经抓住。

重启后若 `ctrl_mode=0`，夹爪可能保留目标而不执行。本轮单次切换CAN/MOVE J模式后70mm目标才执行到68.3mm。当前自动入口尚未集成该初始化，不能把增大开爪力矩当作待机模式问题的修复；勿反复重发或直接运行SDK示例。

### 2026-11-14 用户指定外参基座Z上移130mm试验

用户明确要求相机在机械臂坐标下向上提高13cm、旋转不变，并重试抓取。`extrinsics.json`的平移现为 `[0.4159578390386031, 0.0529010550714366, 0.004679938991033361]` m；原Z为-0.12532006100896664m，精确增加0.130m。旋转及X/Y逐值保留。原`provenance/manual04-result.json`未改写；新增`provenance/manual04-user-z130-20261114.json`记录原文件哈希、基座平移增量、派生正逆变换，主外参指向派生文件及其哈希，加载来源核验保留。该变更是用户指定的人工修正，physical_validation仍为NOT_PERFORMED，不能称为重新标定通过。

本次现场继续使用机载已有`config/piper_vision_grasp/semantic-height0-trial-20261114.json`（仅取消旧-13mm抓取下移，SHA256=e841523dfeaf6090f465459b26e92c9c4064555c682a3f8bb78a8dd1ff547905），它通过共享extrinsics.json加载新外参；30°接近、5%关节速度、1Nm开爪及0.5Nm闭爪参数不变。该试验配置是在现场另存的文件，不属于默认分发包。完整新包包含派生来源文件；机载更新须确认27个运行/配置来源文件清单通过，再用上述配置check/run。

此前工作空间图与3D排障图是旧外参下的历史结果；新外参会把同一相机点在Piper基座下的Z整体提高130mm，不能把旧图当作当前坐标。此次未修改外部piper项目或导航标定。

### 2026-11-14 夹取方向的水平投影修正

上抬后红色方块图像框略宽于高，旧语义测量依据框宽高选中了近竖直边，再投影到基座水平面，得到前后方向的小投影与15–21mm夹取宽，导致WAIT_PLAN/WAIT_ORIENTATION。现改为比较同一深度点云中框上边、右边的基座XY投影长度，采用较长投影确定夹取轴，并沿该轴重新测量宽度；不改变目标中心、相机外参、TCP及既有检查。它测量的仍是可见几何，不推断隐藏厚度。

两项回归覆盖宽图像框及相机旋转90°，相关150项pytest通过；本轮三组现场存档回放均DETECTED且PLANNED，宽约52mm。实际新运行必须重新取图，不能发送存档目标。当前外参继续保持基座Z+130mm、旋转不变；用户明确本轮先不继续修正外参，观察夹取结果。构建、机载部署与实测结果以session_handoff最新记录为准。

### 2026-11-14 相机沿基座−Y平移110mm并将光轴对齐基座+X

用户在两种旋转约束中选择“相机Z+与Piper X+完全同向”，允许相机X轴略变。当前外参在上轮Z+130mm的基础上，再将基座平移Y减去0.110m；相机位置为[0.4159578390386031,-0.0570989449285634,0.004679938991033361]m。围绕相机自身原点施加10.5048148466°最小旋转，使旋转矩阵第三列为[1,0,0]（浮点误差内），不绕基座原点旋转相机位置。这不是严格只绕原相机X轴；严格单轴方案仍有3.0856°残差，用户已明确选择完全对齐。

来源为provenance/manual04-user-z130-y110-alignx-20261114.json，保留原始manual04及上一轮z130派生文件与哈希链。physical_validation仍NOT_PERFORMED。旋转会改变所有相机点到基座的转换，不能仅把旧抓取目标Y减110mm；再次执行时必须重新采集并规划。旧3D排障图/工作空间图仍是历史快照。09抓取等待已退出，更新外参本身不发送运动指令。

### 2026-11-14 对齐后再沿基座+Z增加30mm

用户要求在上述外参基础上再将相机中心沿Piper +Z移动3cm并重试。当前平移为[0.4159578390386031,-0.0570989449285634,0.03467993899103336]m，旋转矩阵逐元素保留，相机Z+仍与Piper X+同向。相对于原manual04累计基座Z+160mm、Y−110mm。新来源为provenance/manual04-user-z160-y110-alignx-20261114.json，上一轮来源和原始标定均保留；仍属人工修正，物理验证状态未改为通过。重试使用新RGB-D与当前反馈重新规划，现场结果见交接末尾。

### 2026-11-14 继续沿基座+Z增加20mm

用户在+30mm试验后要求再加20mm。当前相机平移为[0.4159578390386031,-0.0570989449285634,0.05467993899103337]m；R、X、Y保持不变，累计原始基座Z+180mm、Y−110mm。来源provenance/manual04-user-z180-y110-alignx-20261114.json连接上一轮z160来源，保留历史记录。现场重试延续25°接近，仍使用新RGB-D与实际关节重新规划，试验不等于外参物理标定通过。

### 2026-11-14 沿基座+Y增加70mm

用户要求相机位置沿Piper +Y移动7cm并再次夹取。在z180-y110-alignx基础上，当前平移为[0.4159578390386031,0.012901055071436609,0.05467993899103337]m，X/Z/旋转保持不变；累计相对原manual04为基座Y−40mm、Z+180mm。来源provenance/manual04-user-z180-y40-alignx-20261114.json保留父哈希与平移增量。第13轮3D图保留其历史外参，不随本次修改更新。重试延续25°接近并重新采集定位，现场结果见交接。

### 2026-11-14 沿基座−Y再移动20mm

用户要求在+Y70mm后再向−Y移动2cm并重新抓取。当前相机平移为[0.4159578390386031,-0.007098944928563392,0.05467993899103337]m，X/Z/R保持不变；累计相对原manual04为基座Y−60mm、Z+180mm。来源provenance/manual04-user-z180-y60-alignx-20261114.json保留父来源与增量。重试仍使用新RGB-D、25°接近和原运动参数，实测结果见交接。

### 2026-11-17 应用重新标定外参

用户明确要求应用新外参并尝试夹取。源码及Go2部署目录`/home/unitree/piper-vision-semantic-20261114`的共享extrinsics.json现改为本轮完整标定矩阵，来源`provenance/handeye-recalib-20261117-result.json`，不叠加11月14日人工偏移。平移[358.486967,38.146424,-56.567116]mm，完整旋转见[结果](piper_handeye_recalib_20261117_result.json)。新外参文件SHA256为10f4208f58454784d5e01113bd7797da3c8206731b6e7c37d189ad333d90b359，来源SHA256为fad58acb3ee96f3f3a57b8e0c27b7bbcac900b4116c3ee82267a1585f6523656。

最新开发端安装为`build/piper-handeye-applied-20261117/overlay/install/vlm_nav/share/vlm_nav`；本次33个安装/来源文件一致，机载清单已更新并验证。保留原机载height0试验配置及其SHA256 e841523dfeaf6090f465459b26e92c9c4064555c682a3f8bb78a8dd1ff547905，后续调用仍明确`--color 红色 --description 方块 --approach-pitch-deg 25`，速度5%、开爪1Nm、闭爪0.5Nm，TCP仍是未经实测标定的135mm。相机重新安装后，原蓝色参考样本不是新安装下的现场参考；本次使用语义入口。

机载离线CHECK_OK，但只读反馈为WAIT_CONTROL：teach_status=1、ctrl_mode=2，仍在示教录制。已请用户结束示教并确认取下末端标定板、摆好红色方块；本次尚未启动运动/抓取。下一步先核实这两项现场状态，再恢复模型服务/隧道、以新RGB-D进行规划和一次抓取，不重用存档物体坐标。备份在Go2 `/tmp/piper-before-recalib-20261117.tar.gz`，开发机证据同名副本位于`build/piper-handeye-applied-20261117/evidence`。

后续现场更新：用户授权云端图像处理后，recalib-red-grasp-20261117-03已实际开爪并运行entry路径，末段出现J5碰撞/驱动故障且失去使能。用户确认“碰到了相机”。已退出抓取进程并单次发送急停，收到EMERGENCY_STOP；没有闭爪或抬升，不算夹取通过。当前不应直接重复上述run命令或解除急停，应先核查相机安装是否被撞移及路径干涉；如果安装变化，新外参需重标定。详细参数、反馈和证据路径见[交接](session_handoff.md#2026-11-17-授权云端识别后执行碰撞相机已急停终止)。本轮模型服务及专用反向隧道已关闭，外参文件未再修改。

### 2026-11-17 相机避让重试配置（尚未完成现场重试）

用户指定假设相机未移位，并选择“调整避让路径后，重新尝试抓取”。新入口配置`config/piper_vision_grasp/semantic-camera-avoid.json`已部署到原Go2目录，使用60°、基座Z方向预抓100mm、入口额外抬高100mm，5%速度、原开闭爪力矩及新外参不变。只加两个可选几何参数，默认旧配置路径保持原行为；没有新增运行审批开关。

离线全STL采样复现旧entry的相机干涉；仅抬高25°入口仍在下降末段重叠，不采用。60°新候选356组0.5°内关节采样无相机区域重叠，采样顶点最小距离65.57mm。相机区域是保守假设、未含未知支架/线缆，不是现场无碰撞保证。参考[路径对比图](../../build/piper-camera-avoid-20261117/camera-avoid-paths.png)和[实施计划/验证边界](../superpowers/plans/2026-11-17-piper-camera-avoidance.md)。

本次最新开发安装位于`build/piper-camera-avoid-20261117/overlay/install/vlm_nav/share/vlm_nav`，机载仍`/home/unitree/piper-vision-semantic-20261114`。远端旧geometry/manifest/checksums备份`/tmp/piper-before-camera-avoid-20261117.tar.gz`已取回本地evidence；机载新配置检查camera-avoid-check-20261117-01为CHECK_OK。

后续操作依赖现场结束示教、故障恢复且实际姿态可规划；模型服务/反向隧道需恢复，用户此前已允许本次使用既有凭据与云端识别。Go2终端A、原部署目录、解释器`/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`下，先执行新配置`check`（退出0且CHECK_OK），再用相同配置`preview`获取新目标及真实起点，核对避让路径；最后才用新run-id执行一次`run`。三个动作都通过原`scripts/piper_vision_grasp.sh`，统一传`--config config/piper_vision_grasp/semantic-camera-avoid.json`；不要复用旧25°CLI参数或发送存档候选。诊断终端B继续原diagnostics并保持运行，A的check/preview成功后退出，run若等待则保持并按具体state排障。


### 2026-11-17 已授权应用人工对齐平移候选

当前外参的平移已按[候选记录](piper_handeye_translation_candidate_20261117.json)调整：Piper base +X77.816491mm、+Y5.641927mm、+Z33.324299mm，旋转保持新手眼标定值。候选记录是应用前档案，来源包`config/piper_vision_grasp/provenance/handeye-manual-translation-20261117.json`另存本次应用依据与原矩阵，不覆盖原手眼结果。当前相机平移为[436.303459,43.788351,-23.242816]mm，physical_validation仍NOT_PERFORMED。

最新安装输出为`build/piper-translation-applied-20261117/overlay/install/vlm_nav/share/vlm_nav`；机载目录、Bash入口和虚拟环境解释器不变，启动加载共享extrinsics.json的新来源。构建/检查见[主runbook](go2_runbook.md)。本次用户要求应用后回零，待其调整积木再重新抓取，因此不要因配置check通过自动启动run。上轮40°只是针对旧摆放的CLI参数，新摆放须重新计算。

### 2026-11-17 可选 GraspNet 抓取位姿后端

新配置 `semantic-graspnet.json` 让 GraspNet 决定位置、朝向和宽度，其余控制流程保持。开发机独立模型服务使用 18766 端口，Qwen/SAM 仍使用 18765。详见 [GraspNet 离线使用与来源说明](piper_graspnet_runbook.md)，完整构建命令仍在 Go2 runbook。当前仅完成开发机接入与离线验证，未部署；保存场景尚未满足完整路径可行验收，不能按旧现场步骤直接视为已验证的抓取算法。

## 俯仰与水平朝向枚举（2026-11-18，本地实现）

用户选择俯仰＋水平朝向搜索；Go2已关机，本次只构建和离线回放，未连接/部署/运动。
默认`semantic-camera-avoid.json`及`semantic.json`启用`semantic.angle_search`。
Qwen仍只输出目标标签、二维框/点；SAM和深度给出可见中心及几何。程序固定该中心位置，枚举夹爪姿态，首个完整路径`PLANNED`即采用。

- 外层水平偏角依次为`0,30,-30,60,-60,90,-90,120,-120,150,-150,180`度，相对深度几何估计的水平夹爪方向，覆盖一周。
- 每个水平偏角依次尝试俯仰`40,30,50,20,60,10,70,0,80,90`度。0度为水平接近，90度为竖直向下；不是连续姿态或全部滚转角穷举。
- 预抓沿工具接近轴反向`pregrasp_distance_m`，入口再沿同方向反向`entry_clearance_m`。默认入口配置两者均100mm；`semantic.json`没有额外入口距离时保持0。抬升仍为基座Z方向20mm。
- 水平旋转后，宽度按本次目标有效深度在XY平面的凸包投影重新计算；没有完整三维模型，不估造隐藏厚度。若可见宽度非正或超过既有open_ready_mm/open_mm与SDK100mm能力的最小值，该候选记为WAIT_WIDTH并继续搜索，避免IK通过但关闭指令不可执行。
- 每个候选使用原关节限位、1mm/1° IK判据和完整入口/预抓/抓取/抬升路径检查。失败继续下一个；120个都失败则`WAIT_PLAN`等待新观测。不是自动放宽误差或发送失败解。
- `SEARCHING_IK`事件保存候选角度、状态、失败原因/残差/目标；`planning.json`和`plan.json`含`orientation_candidates`及`selected_candidate_index`。报告显示拒绝列表、选中方案、实际接近方向和TCP坐标轴。

既有`--approach-pitch-deg`显式参数继续表示手动固定姿态，会关闭本次自动搜索；使用默认搜索时不要传此参数。GraspNet入口和旧参考模式不使用此搜索，六轴回零动作也不涉及搜索。

本地新安装为`build/piper-angle-search-20261118/overlay/install/vlm_nav/share/vlm_nav`，完整构建与检查见[主runbook](go2_runbook.md#2026-11-18-夹爪角度搜索构建)。历史数据离线重算不是新现场图像，绝不可作为`run`的运动输入。规划没有新增实物碰撞检查；侧向路径与标定/TCP、可见宽度的物理适用性仍需现场验证。

### 角度搜索版本已部署（2026-11-19）

本次用户明确授权部署后，`/home/unitree/piper-vision-current`已切换至`/home/unitree/piper-angle-search-20261119`。44项文件哈希、机载新目录及current入口两次`CHECK_OK`、四份历史观测的机载离线IK回放均通过。没有启动真机运动；本节覆盖前文“未部署”的历史状态。

当前入口（Go2上位机Bash，目录不限，以下仅离线配置/依赖检查，成功即退出，无需保持终端）：

```bash
export PIPER_VISION_GRASP_PYTHON=/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python
bash /home/unitree/piper-vision-current/scripts/piper_vision_grasp.sh check
```

成功判据为结果文件`CHECK_OK`且STARTING日志module/config指向`piper-angle-search-20261119`；失败先查看打印的诊断路径，不继续运动。默认启用10俯仰×12水平朝向搜索，30%速度，模型仅定位、启发式生成姿态。不要附加`--approach-pitch-deg`，除非有意恢复固定角度。真实preview/run仍需模型服务/隧道、新鲜相机和反馈以及对应现场授权，部署检查不能代替这些条件。

旧r2目录保留，回退目标记录在`/home/unitree/piper-vision-current.before-angle-20261119`；独立回零部署`/home/unitree/piper-zero-20261119`没有被覆盖。部署证据在开发机`build/piper-angle-deploy-20261119/`。未在机载安装绘图依赖，点云/报告模块随包提供，完整可视化继续使用开发机既有绘图环境。

### 2026-11-20 本地抓取配置采用 r3

开发机当前共享 extrinsics.json 使用 [r3 的 100 mm 黑框、20+8 标定结果](piper_handeye_r3_100mm_20plus8_result.json)。平移为 [354.844,33.748,−65.293] mm，方向为光学系到 Piper 基座。四个配置共用这份完整旋转和平移。启动路径见[主 runbook](go2_runbook.md#2026-11-20-采用-r3-实测-100-mm-手眼外参本地)。现场抓取精度仍待验证；本次更新范围为本地源码与构建安装。
