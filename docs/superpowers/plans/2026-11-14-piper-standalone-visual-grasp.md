# Piper 独立自主视觉抓取 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户要求在另一个对话执行；本对话只交付方案，不启动实施、部署或真机动作。默认单代理执行，不自动提交。

**Goal:** 在 `VLM_Nav` 内实现一个可独立启动的蓝色积木视觉抓取功能：自动定位、按成功参考姿态修正目标、低速接近、夹持、抬升并验证结果，不需要操作者逐步指示移动方向。

**Architecture:** RGB-D 蓝色目标检测 + 固定抓取朝向的局部参考位移修正 + Piper SDK 单次目标执行。全部源码和必要参考资产纳入本仓库；在连接 RealSense/CAN 的 Go2 上位机运行。第一版不接导航、ROS节点、DDS、VLM任务规划或旧的 `agent/main4.py`。

**Tech Stack:** Python 3.8兼容代码、NumPy、OpenCV、SciPy、pyrealsense2、piper-sdk 0.6.2、python-can 4.5.0；复用现有可用环境，不引入模型权重、PyTorch、SAM、GroundingDINO或新服务。

方案编写时已在机载候选环境只读核实：Python 3.8.10、NumPy 1.17.4、SciPy 1.3.3、OpenCV 4.12.0、pyrealsense2 2.55.1.6486、piper-sdk 0.6.2、python-can 4.5.0。SciPy有 `least_squares`，但Rotation仅有 `from_dcm/as_dcm`，没有 `from_matrix/as_matrix`。实现需兼容该API，不为此升级已有环境；执行时仍复核实际环境。

**Spec:** 本文“设计约定”是本次独立试验的实现规格。更大的抓放/搬运设计仅作为背景：[既有抓放搬运设计](../specs/2026-11-10-go2-piper-pick-place-carry-design.md)，不把其全部范围带入本任务。

## Global Constraints

- 源码根目录 `/home/isee-pst/unitree_ros2/VLM_Nav`；独立功能，不修改现有导航流程、Ranger默认配置、Go2运动桥接或协同协议。
- 先读根 `AGENTS.md` 和 [session_handoff](../../multi_agent/session_handoff.md)，核对分支/HEAD/未提交改动。方案编写时分支为 `go2-cooperative-task-layer`，HEAD为 `323d5c52737477962a874ae012db117be5a93f34`，工作区有大量既有修改；不得 reset/clean 或覆盖这些修改。
- 外部 Piper 项目只作为可追溯来源，不作为新功能运行时的源码依赖。禁止导入会初始化/使能机械臂的旧抓取主程序。
- 不自动校零、清错、重启服务、改CAN参数、增加力矩、释放持物或操作Go2底盘。ArmRobotUX中的准备流程由现场操作者完成；“每次都重新设置零点”不是程序启动步骤。
- 运行故障进入带原因的等待状态；未发送的动作可待输入恢复，已经尝试发送的同一阶段不能因超时、异常或重连自动重发。
- 沿用现有USB身份、反馈新鲜度、设备限位、使能/故障检查和同一控制锁；不另外堆叠未讨论的授权门槛。有人看管的一次完整 `run` 不要求逐个关节阶段重复确认。
- 主入口终端只给必要日志/产物路径；诊断终端显示状态、原因及进度。关闭监测不等于物理急停，不宣称进程退出会取消已发送目标。
- 代码修改后按 [go2_runbook](../../multi_agent/go2_runbook.md#build-and-offline-validation) 完成必要构建并核对安装入口；完整构建命令只在该runbook维护。文档-only改动不构建、不启动ROS。
- 不自动提交、部署、启动现场试验；当前请求授权编写方案。执行对话若明确要求实施则完成代码/测试/构建；现场动作沿用该对话明确的试验范围，不能重用历史单次命令的授权。
- 完成代码修改后按项目约定播放持续5秒提示音乐；本轮只写方案不播放。

## Review Focus

以下五类问题必须分别进入对应任务的测试，不能只在最终说明中列风险：

1. **参考配错事件：** 误用碰倒前画面或抬升后画面绑定成功夹取角度，导致数厘米的系统偏差（任务1、2）。
2. **图像不完整：** 多蓝色物体、裁切、遮挡、缺失深度或非零畸变模型，不能输出一个看似有效的抓取点（任务2）。
3. **单位/变换错误：** 米/毫米、角度/弧度、深度到彩色旋转存储顺序错误；局部修正不能又叠加9cm/13.5cm TCP（任务3）。
4. **断流和部分发送：** SDK返回不等于执行成功，重启/重连不能重放已尝试的动作（任务4、5）。
5. **夹空或卡住被当成功：** 36mm位置目标未达到也可能夹住，达到目标也可能夹空；不能单凭夹爪角度或机械臂抬高判定物体被抓起（任务5）。

## 设计约定

### 1. 已确认事实和本轮边界

2026-11-14已完成有人看管、人工逐步修正的蓝色积木抓取和约2cm抬升，之后按用户要求释放并回到六关节零位。已验证的是硬件控制及一次具体夹取，不是自主视觉抓取。

| 项目 | 本次实现采用的信息 |
|---|---|
| 目标 | 蓝色实心长方体，正面约42mm、前后厚20mm；用户原估高约100mm，深度几何约117mm，不把原估高度用作绝对抓取高度 |
| 场景 | 同一支撑面，垫高约180mm；Go2静止、相机与机械臂安装不变，积木保持同一面朝向相机 |
| 硬件身份 | RealSense `151222077264`；USB-CAN `0046002F5246570520323934`，1Mbps；按USB身份发现接口，不把can0名称当永久身份 |
| 首轮位置变化 | 参考位置附近的小范围平移；建议首轮水平各方向约20mm内，不扩展到任意朝向/高度/物体 |
| 关节控制 | SDK关节位置模式，5%速度，既有到位判据0.3°且反馈motion_status=0 |
| 空载张开 | 70mm目标、0.5N·m；实测约68.1mm，不能把它改写为70mm精确到位 |
| 闭合 | 36mm目标、0.5N·m；成功夹持反馈约52.8mm，力矩反馈约−0.525N·m |
| 抬升 | 基座+Z20mm；成功记录实际SDK法兰上升19.586mm |
| 最近回位 | 六关节全0°、夹爪约68.1mm张开，均使能、无故障；这是历史状态，运行前重新读取 |

初次按45°朝向和暂定90mm TCP计算的预抓曾碰倒积木。成功过程后来改变了朝向，并做了横向、前后和高度修正。因此禁止重放那次失败预抓；禁止把90mm或旧代码135mm写成已标定TCP。

### 2. 选择局部参考方法

第一版保留成功夹取的实际朝向，通过新旧目标的视觉位置差修正法兰目标：

```text
delta_B = R_base_color_optical @ (p_color_now - p_color_reference)
T_base_flange_target.R = T_base_flange_reference.R
T_base_flange_target.t = T_base_flange_reference.t + delta_B
```

- `p_color`使用**同一个自动检测算法得到的可见正面中心**；不混用人工圈选中心、物体背面中心或抬升后的中心。
- `T_base_flange_reference`从成功夹持前关节反馈和同一URDF FK计算，并与同期SDK位姿核对。
- 固定工具朝向、同一物体面和高度时，工具偏移及外参平移在差分中抵消；**外参旋转误差并不会抵消**。该方法只支持本轮局部试验，不等于完成通用手眼/TCP标定。
- 不在公式后再添加90mm/135mm工具补偿，不用单点修正去覆盖正式外参。
- 若相机/机械臂安装、Go2站姿或机械零点改变，旧参考不能直接沿用；记录原因并等待重新建立参考，不自动校零。
- 备选方案是先重做完整TCP/物理手眼校验后进行绝对坐标抓取；范围更大，本轮不实施。原角度序列直接回放无法证明视觉自主性，也不作为本轮方案。

### 3. 参考证据与来源

下列路径均相对于仓库，除特别注明的外部来源。`build/`证据不受Git保护，实施任务1必须提取必要小样本及摘要到受版本管理的目录，运行时不能依赖这些旧build路径。

| 用途 | 真实来源 |
|---|---|
| 参考物体RGB-D | `build/piper-bottle-ground-20261113/blue-block-after-retreat-rgbd01/`：积木被重新立起后的完整可见目标，3帧 |
| 同位置交叉检查 | `build/piper-bottle-ground-20261113/blue-block-half-horizontal-rgbd01/`：其后、闭合前，同一物体位置；检查自动算法对轻微遮挡的稳定性 |
| 成功夹持前关节姿态 | `build/piper-bottle-ground-20261113/blue-block-lower03-runtime.jsonl`，并核对 `blue-block-grasp-effort05-01-runtime.jsonl` |
| 成功的局部预抓位置 | `build/piper-bottle-ground-20261113/blue-block-right-extra01-runtime.jsonl`：用户确认基本对准、悬空未接触 |
| 闭合/抬升结果 | `blue-block-supervised-grasp-result.json`、`blue-block-loaded-lift01-runtime.jsonl`（均位于上述证据根目录） |
| 抬升后视觉证据 | `blue-block-grasp-hold-rgbd01/`，仅用于结果验证测试，**不能当夹持前参考物体位置** |
| 空载展开/回位 | `blue-block-approach01-runtime.jsonl`、`blue-block-approach02-runtime.jsonl`、`blue-block-approach03-runtime.jsonl`；`blue-block-return01-runtime.jsonl`至`blue-block-return04-runtime.jsonl`（均位于共同证据根目录） |
| 候选运动学模型 | `build/piper-reachability-20261106/piper_description.urdf`；只提取6关节的origin/axis/limit到配置，不把mesh可视化当碰撞检查 |
| 外参 | `/home/isee-pst/unitree_ros2/piper/handeye-data/manual04-result.json`；SHA256=`b8f843b4aee21b530e53c642d1ceaa16868c7680b16610504157135596433151` |
| 已验证SDK反馈/控制逻辑 | `/home/isee-pst/unitree_ros2/piper/standalone/piper_control.py`：`Feedback`、`Command.step`、反馈过滤与一次发送语义 |
| 本仓库设备与健康逻辑 | `scripts/go2_piper_boot.py`：`discover`、`prepare_can(readonly=True)`、`assess`；绝不调用其使能流程 |
| 已验证有限相机采集 | 外部 `/home/isee-pst/unitree_ros2/piper/handeye/capture.py`：显式序列号、暖机、元数据和静止前后配对思想；只迁入本任务所需部分，不引入ArUco标定流程 |

成功夹取参考关节为 `[4.296,124.232,-78.909,-28.320,-19.013,29.113]`°，同期SDK法兰 `[476.249,49.884,153.501]`mm，RPY `[166.373,66.138,179.126]`°。这些值只用于核对提取是否正确，实际参考文件必须附源文件hash、采样时间和算法版本。

局部预抓法兰约为 `[466.287,49.451,193.153]`mm，即相对成功夹取约后退10mm、抬高40mm。使用其记录建立参考局部路径；不要再使用失败的45°/80mm预抓点 `blue-block-approach05`。

### 4. 文件布局与接口

以下均为**拟新增**，不得把方案中的命令当成已经存在：

| 路径 | 责任 |
|---|---|
| `tools/piper_vision_grasp/__init__.py` | 无硬件副作用的模块入口 |
| `tools/piper_vision_grasp/vision.py` | 有限RGB-D采集、蓝色目标检测、深度/形状检查、物体抬升证据 |
| `tools/piper_vision_grasp/geometry.py` | 配置校验、FK/IK、局部位移修正与路径采样；纯计算，无CAN/相机 |
| `tools/piper_vision_grasp/piper_io.py` | 迁入已验证的反馈与单次目标逻辑，复用仓库内设备/健康函数；无自动初始化运动 |
| `tools/piper_vision_grasp/main.py` | CLI、阶段状态、事件日志、结果文件、诊断输出；不引入通用任务框架 |
| `scripts/piper_vision_grasp.sh` | 唯一独立入口，解析源码/安装根目录、解释器和诊断窗口，不source导航环境 |
| `config/piper_vision_grasp/config.json` | 相机/USB身份、参数、路径、局部试验范围 |
| `config/piper_vision_grasp/reference.json` | 受版本管理的历史参考、局部预抓/抓取姿态、来源hash；不是新TCP标定 |
| `config/piper_vision_grasp/extrinsics.json` | 原外参及来源标识，不覆盖原手眼结果 |
| `config/piper_vision_grasp/kinematics.json` | 从已核对URDF导出的6轴origin/axis/limit及源hash，无mesh依赖 |
| `test/fixtures/piper_vision_grasp/` | 少量必要RGB-D真实样本与manifest，不复制整个build目录 |
| `test/test_piper_vision_grasp.py` | 纯计算、反馈/序列fake及导入无副作用测试 |
| `docs/multi_agent/piper_vision_grasp_runbook.md` | 独立功能使用顺序、运行机器/解释器、诊断与结果含义；构建命令仍只链接主runbook |

需要修改 `CMakeLists.txt`：安装新tools目录，注册新测试；现有config和scripts目录安装规则可复用。需要同步更新主runbook、session_handoff。**不需要新launch文件或修改导航环境脚本**；安装布局须保持 `scripts/`、`tools/`、`config/` 的相对位置，避免源码能跑而安装产物找不到模块。

使用简单dict作为跨模块数据，不另造插件/总线/调度器：

- `vision.capture_rgbd(config: dict, out_dir: Path, count: int = 3) -> dict`：返回文件、时间、frame number、尺度、原始/对齐深度及对应内外参。仅调用时连接相机；采完释放，不常驻完整点云/RGB订阅。
- `vision.detect_blue_block(rgb: ndarray, depth: ndarray, camera: dict, config: dict) -> dict`：`status`、`reason`、`point_color_m`、目标mask/边界、有效深度比例、可见宽高；无法可靠定位时点为null。
- `vision.verify_lift(before: dict, after: list, expected_delta_base_m: list, extrinsics: dict) -> dict`：返回物体位移、可见性/一致性证据与 `VERIFIED/WAIT_VERIFY`，输入为同一检测器的前后观测，不能读取SDK位姿充当物体位移。
- `geometry.retarget(reference: dict, observation: dict, extrinsics: dict) -> dict`：返回 `delta_base_m` 与固定朝向的目标法兰变换。
- `geometry.plan_motion(current_deg: list, reference: dict, target: dict, model: dict) -> dict`：返回每阶段关节目标、采样残差、适用范围和未验证项。
- `piper_io.PiperSession(config: dict)`：上下文管理器，`snapshot()`、`attempt_joint(stage_id, angles_deg, speed_percent)`、`attempt_gripper(stage_id, width_mm, effort_nm)`、`poll()`；构造/导入不发送命令，持有全会话同一USB控制锁。
- `main.run_once(config: dict, session, camera, out_dir: Path) -> dict`：注入session/camera以便无硬件测试；一次运行最多一个抓取周期，结束保持，不自动掉落、回零或再抓。

### 5. 运行流程与结果语义

拟定CLI：`check`（离线配置/依赖）、`observe`（有限采集+被动反馈）、`preview`（新鲜视觉+只读反馈+纯规划）、`run`（一次真实周期）、`diagnostics`（跟踪日志）。`preview --input <capture_dir>`为离线样本模式，只使用文件及其中记录的关节状态，不连接相机/CAN；真实run不接受该离线输入。`check`不触碰硬件，`observe/preview`不得发送任何CAN帧；只有`run`可查询设备限位并发送目标。

一次 `run`：读取当前状态 → 暖机并采3帧 → 自动定位 → 计算相对位移 → 生成路径 → 打开夹爪 → 经空载过渡点到局部预抓 → 分段接近 → 闭合 → 判断夹持候选 → 抬升20mm → 视觉验证 → 保持。

- 蓝色检测第一版用HSV+连通域+形状/深度检查，不调用VLM。起始HSV候选为OpenCV H=90..140、S≥60、V≥30，实施时用真实样本验证并把最终值写入配置；不能用固定像素ROI替代检测。
- 参考和新目标使用完全相同的检测/深度算法。对齐深度与彩色内参匹配，零深度不能作为零距离；原始深度复核时遵循RealSense外参列优先旋转。遇到未支持的非零畸变必须报出，不能静默当针孔相机。
- 三帧位置采用中位数；初始稳定性诊断阈值3mm、物体朝向变化诊断阈值5°，作为本轮算法配置而非通用硬件安全保证。裁切、明显遮挡、多候选/无候选进入相应WAIT原因，恢复后重新定位，不能沿用旧目标。
- 首轮局部参考范围配置建议基座水平各±30mm、竖直±10mm，覆盖约20mm试摆；超出说明“超出本轮局部参考适用范围”，不能写成Piper不可达或擅自外推。范围调整应有新的现场证据。
- 空载过渡可复用此前J2=10/40/80°的整体关节向量；局部路径从参考预抓上方约60mm的过渡点进入预抓，再按记录的后退/抬高偏移接近。每个目标用上一解作seed，检查原有模型/设备限位、IK残差和相邻采样；不保证SDK关节插补就是笛卡尔直线。首轮现场重点观察新过渡段，不重用失败45°预抓。
- 开爪70mm实际约68mm：控制底层继续如实报告位置误差；流程可按本次已验证的开口定义 `OPEN_READY`（使能、新鲜、稳定且反馈≥66mm），不能篡改底层±1mm的 `AT_TARGET`。
- 夹持候选要求：闭合后宽度稳定、比开爪明显减小、仍高于空闭目标、无故障且使能；这只能得到 `CONTACT_CANDIDATE`。36mm空闭到位不证明抓住，52.8mm也不能被硬编码为每次成功值。
- 抬升结果必须看**物体**：前后RGB-D中同一蓝色目标向上移动约20mm，初版可接受10..30mm且末段连续观测稳定。物体被遮住/深度不可信则 `WAIT_VERIFY`，不能仅依据SDK上升写 `GRASP_CONFIRMED`。操作者可追加实物观察记录，但与自动判定分开。
- 完整正常周期不询问“向左还是向右”。人的角色是现场看管和异常接管；成功验收要明确有没有人工方向修正。
- 诊断至少包括 `state, reason_class, reason, stage, target, measured, command_attempted, frame_age, reference_id, run_id`。异常不触发自动松爪/失能；`COMMAND_UNCERTAIN`保持不重发，恢复需先核实当前状态和已执行阶段。

## 实施任务

### Task 1：固化可追溯参考、模型和测试样本

**Files:** 新建上述config四文件、fixtures/manifest；测试 `test/test_piper_vision_grasp.py`；只读来源表中的历史证据。

**Interfaces:** 产出schema版本1的配置、参考与模型dict，供 `retarget`、`plan_motion` 使用；参考最少含 `point_color_m, T_base_flange, pregrasp_offset_base_m, source_files, source_sha256, timestamps, camera_serial, usb_serial, detector_version`。

- [ ] 先写 `test_reference_provenance_and_phase`：成功关节等于记录值；参考采集发生于闭合前；抬升后数据不能被标注为夹持前参考；缺失文件/hash不符必须失败而非补默认坐标。
- [ ] 用保存源的hash验证后，仅复制必要RGB/深度/元数据到fixtures；从URDF生成kinematics配置，保留弧度限位及米制origin。
- [ ] 参考 `point_color_m`暂不手填：待任务2实现后由同一检测器从参考fixture生成；在此之前reference标记 `incomplete`，不得允许真实run。
- [ ] 保存外参原值及 `physical_validation=NOT_PERFORMED`，不得因为本次人工夹取成功改为已标定。
- [ ] 核对来源路径/时间和fixture大小，记录新增资产清单；不提交整批日志/虚拟环境/模型权重。

### Task 2：有限RGB-D采集与自动蓝色积木定位

**Files:** `vision.py`、fixtures、reference.json、配置、同一测试文件。

**Interfaces:** 实现 `capture_rgbd`、`detect_blue_block`；返回的点始终为彩色光学坐标系米制，失败时无有效点。

- [ ] 先写检测测试：完整目标可检出；全零深度返回 `WAIT_DEPTH`；两目标不任意选一个；边界裁切/严重遮挡不生成抓取点；颜色图/深度尺寸或内参不匹配报明确原因。
- [ ] 用颜色mask内部的有效深度拟合可见面，结合自动提取的面边界计算中心；不把背景深度或夹指深度混入。宽约42mm的真实样本估计应与实测合理一致，不能用尺寸强行反算深度覆盖测量。
- [ ] 实现显式设备序列号 `151222077264`、640×480@15fps、RGB/depth、暖机和3帧存档；保存每帧内参、尺度、时间域及frame number，结束释放相机。
- [ ] 用 `blue-block-after-retreat-rgbd01`生成自动参考点；用half-horizontal样本核对同位置检测一致性。若历史参考配对不足以成立，明确标为参考未就绪，另行现场录制一次参考，不硬拼旧数据。
- [ ] 实现 `verify_lift` 并测试“手臂动、物体没动”“遮挡导致mask中心变化”“物体真实上升20mm”；前两类不能报抓取成功。

### Task 3：参考位移修正、FK/IK和局部路径

**Files:** `geometry.py`、kinematics/reference配置、同一测试文件。

**Interfaces:** 实现 `retarget` 和 `plan_motion`；planner输出阶段列表，不导入SDK，不发送命令。

- [ ] 先写 `test_delta_zero_reproduces_reference`、`test_camera_delta_rotates_into_base`、`test_extrinsic_translation_cancels`：零位移返回同一参考法兰；改变外参平移不改变差分结果；相机+X通过真实R变为正确基座方向，单位保持米。
- [ ] 写 `test_no_second_tcp_offset`：结果不得额外平移0.09m或0.135m；固定朝向必须等于参考R。
- [ ] 实现6轴FK和带限位的SciPy IK，复用本次离线方法，但兼容机载SciPy 1.3.3的 `from_dcm/as_dcm`；本机新API与机载旧API路径都要覆盖。对零位和成功姿态核对SDK位置差≤1mm（仅数值一致性）。位置残差≤1mm、旋转残差≤1°才作为可用数值解。
- [ ] 构造空载过渡、局部预抓、接近与20mm抬升阶段；夹取附近以约10mm位置间隔采样并记录关节变化。测试无解、超限、改变初始关节状态需要重规划，不忽略当前姿态重放旧路径。
- [ ] 离线改变目标位置，断言生成的末端目标随视觉变化；证明不是仅回放固定关节角。保存preview包含基准/新目标/修正量/各阶段与未验证碰撞项。

### Task 4：仓库内独立SDK执行与反馈

**Files:** `piper_io.py`、同一测试文件；复用但不修改现有boot/gripper入口行为。

**Interfaces:** 实现 `PiperSession`。只在显式进入硬件会话时创建SDK/socket，传入 `ConnectPort(can_init=True,piper_init=False,start_thread=False)`；借鉴已有Feedback解析逻辑。

- [ ] 先写无硬件测试：import/check/observe/preview不得打开CAN发送或调用使能；使用fake SDK/CAN统计每一种调用。
- [ ] 将已验证的反馈过滤、单位换算、限位查询和一次发送逻辑迁入本仓库模块，标注源文件hash。设备身份与健康复用 `go2_piper_boot`，仅调用只读检查，不能调用 `step/session/main`使能路径。
- [ ] 同一USB会话持有既有 `~/.local/state/go2-piper-gripper/control-<serial>.lock`；SDK解析与会话接收避免多线程无锁共享。SDK/相机依赖延迟导入，离线几何测试不依赖设备。
- [ ] 写 `test_attempt_recorded_before_partial_send`、`test_reconnect_never_replays_stage`、`test_stale_feedback_cannot_advance`、`test_gripper_fault_waits`。持久化run/stage尝试记录必须先于SDK写；重复run-id不可覆盖或重放。
- [ ] 关节执行仍用 `MotionCtrl_2(1,1,5,0)` + `JointCtrl`；夹爪用相应 `GripperCtrl(width*1000,effort*1000,1,0)`。禁止自动追加校零/清错/使能六轴/力矩升级。

### Task 5：单次自主流程、诊断和结果

**Files:** `main.py`、`scripts/piper_vision_grasp.sh`、配置、同一测试文件。

**Interfaces:** CLI调用同一 `run_once`，通过注入session/camera做完整fake测试；产物统一位于一次运行独立目录，包含RGB-D、计划、events.jsonl、result.json。

- [ ] 先写fake端到端测试：检测到移动后的积木→目标相应改变→开爪→接近→夹持候选→抬升→物体视觉上升→完成；断言全程无需人工方向输入，且每阶段只发送一次。
- [ ] 写失败流程测试：无目标/深度中断可等待恢复；命令已尝试后断流不重复发送；夹空不抬升；物体不动/不可见不报成功；Ctrl+C不自动松爪/失能、不发送后续阶段。
- [ ] 独立入口自动打开可用诊断终端；无图形界面给出第二终端 `diagnostics` 用法。入口stdout只打印日志/结果路径，不能倾倒配置/日志内容。
- [ ] `run`默认只执行一次抓取并保持；不加入自动落物、放回、导航、语音/VLM、重复抓取或常驻后台服务。稳定性/状态等待可轮询，但禁止无限循环重复运动。
- [ ] 将目前算法的局部范围写明 `shortcut: ...` 注释：固定物体朝向/高度、成功姿态附近；改变安装/零点或扩展任意姿态时升级为重新建立参考或完整标定。

### Task 6：构建、安装验证、运行手册与现场交付

**Files:** `CMakeLists.txt`、主go2_runbook、独立runbook、session_handoff、入口脚本。

- [ ] 定向执行新测试及现有 `test_piper_boot.py`、`test_piper_gripper.py`，分别报告新增与基线结果；所有运行都使用fake，不连接硬件。补Python3.8语法兼容和 `bash -n` 检查。
- [ ] 按主runbook建立独立build/install/log输出，源码发现包含根目录与 `cooperative_ws/src`；修改后的根包完成必要构建。仅在主runbook补充本功能构建/测试精确命令，不能把完整构建命令复制到两个文档。
- [ ] 从安装目录执行离线check及 `preview --input <fixture目录>`，核对实际模块/config/解释器路径来自同一安装产物，不能靠外部 `/home/isee-pst/unitree_ros2/piper` 的源码补洞；fixture目录仅为只读数据输入。
- [ ] 运行位置是Go2上位机。既有候选解释器 `/home/unitree/piper-handeye-20261112/.handeye-venv/bin/python`；先只读检查依赖版本。不得复制控制电脑的x86虚拟环境到机载端，缺依赖先核实再决定最小补齐方式，不升级导航环境。
- [ ] 准备最小运行产物：本功能tools、独立入口、4个配置、被复用的 `scripts/go2_piper_boot.py`、来源清单。部署路径在执行对话核实，不能假设机载端存在控制电脑绝对路径；部署须在该对话授权范围内进行。
- [ ] 独立runbook写明下面的操作顺序、每步预期和失败分支；`git diff --check`、链接检查、构建/安装路径核对后更新交接。不要自动提交。

## 提议的运行顺序与验收

以下命令是**实施完成后必须提供的接口约定，当前尚未实现**。执行者在独立runbook中替换为机载端实际安装目录与已核实解释器，不能让用户直接照抄控制电脑路径到Go2。

1. **开发机离线终端，仓库根目录，非ROS运行环境。** 运行定向pytest、离线check、保存样本preview；成功判据是测试通过、没有硬件连接、参考来源有效。失败先修复，不进入现场run。详细测试/构建命令落在主runbook。
2. **Go2上位机终端A，实际独立安装根目录。** 明确 `PIPER_VISION_GRASP_PYTHON` 指向核实过的环境；无需source导航环境。`bash scripts/piper_vision_grasp.sh check`：应退出0并报告配置/依赖/来源可用，不初始化相机或CAN。
3. **Go2上位机终端B，同一根目录和环境，保持运行。** `bash scripts/piper_vision_grasp.sh diagnostics`：跟踪本功能日志，显示当前阶段和原因。终端A继续后续命令。
4. **先重新立好积木。** 同一支撑面、同一面朝向相机，无需精确复原位置；Go2保持静止。`observe`采有限3帧及被动反馈后结束，确认完整目标和当前状态。
5. **终端A执行 `preview`。** 应输出自动检测图、三维位置差、目标法兰和路径文件，CAN发送计数为0。错误按原因等待/修复，不将参考数据当新现场图像。
6. **在执行对话明确现场试验后，终端A执行一次 `run`。** 正常流程不逐步询问方向；操作者看管、可接管。run需重新采集新鲜目标，不能盲用上一步preview的旧现场数据。成功后保持，不自动释放或回零。
7. **重复2～3个小范围不同摆位。** 每次单独run-id、同样的识别/参数，无人工方向微调；记录每次是否自主抓起、误差、等待原因及人工介入。一次成功只能报告该次通过，多位置结果用于评价本轮局部自主性。

最终交付应明确区分：离线测试通过、安装入口通过、单次现场自主抓取通过、多位置重复性；不把mock、SDK位姿上升、历史人工微调成功写成自主验收。真实地面抓取、任意物体/朝向、放置和搬运均不属于本轮。

## 另一个对话的启动说明

可将下面这段原样发给执行对话：

> 请读取 `AGENTS.md`、`docs/multi_agent/session_handoff.md` 和 `docs/superpowers/plans/2026-11-14-piper-standalone-visual-grasp.md`，按计划实现 VLM_Nav 内独立的蓝色积木自主视觉抓取功能。先核对分支和未提交改动，保留现有工作。采用成功抓取姿态加RGB-D相对位移修正，固定成功朝向，不使用暂定9cm或旧13.5cm做绝对TCP；不接导航框架、不导入旧抓取主程序。完成源码、真实样本测试、fake控制测试、构建、安装入口核对和运行文档，不自动提交或启动真机。交付具体可运行的单次observe/preview/run入口和诊断终端，再按本对话的现场授权开展验证。
