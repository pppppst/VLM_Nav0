# Piper GraspNet 接入与离线使用

> 当前选择（2026-11-18）：按用户要求恢复启发式语义抓取，默认配置为 `semantic-camera-avoid.json`，六轴30%。当前包部署及入口见 [runbook](go2_runbook.md#2026-11-18-恢复启发式默认入口并部署30速度)。GraspNet旧包/记录保留，不用于当前默认流程。

> 2026-11-18：源码及本次安装包的四个抓取配置统一改为六轴30%速度比例（原5%）；当前启发式包已部署30%，旧包与历史现场记录中的5%不变。最新构建/入口见 [go2_runbook](go2_runbook.md#2026-11-18-piper-六轴抓取提速构建)。30%尚未真机验证，比例提高6倍不等于总耗时缩短6倍。

GraspNet 取代语义几何模块对抓取位置、朝向、宽度的决定。Qwen/SAM 继续选物，物体稳定性、现有外参、135 mm TCP、30% 速度、70 mm 开爪、力矩、抬高路径、接触确认和抬升验证保持原流程。旧配置仍走旧算法，新配置必须显式选择 `semantic-graspnet.json`，没有自动回退。

本轮仅开发机离线验证，未部署、未启动真机。三组现场保存数据能完成真实推理，但尚无完整可行路径，不能作为实机抓取通过。当前机器人姿态沿用交接记录，本轮没有读取或改变。

## 数据与坐标约定

三帧物体观测稳定后，仅取最后一帧深度和 SAM 掩码调用一次模型。掩码采用现有语义模块相同的深度 IQR 离群点处理；场景点云保留背景，用于官方局部碰撞过滤。采样 20,000 点，固定随机种子 0；按目标关联、70 mm 开口、官方碰撞过滤、NMS 和分数筛选最多 50 个候选，再逐个尝试原有完整路径规划。无候选/不可达为 `WAIT_PLAN`，服务不可用也可等待重试，不发送该规划的运动命令。

缓存回放明确使用保存时的目标：结果分别记录 `target_spec`（实际缓存目标）和 `requested_target_spec`（当前配置默认值），诊断原因也包含缓存目标。`preview --input` 不接受 `--color/--description` 重新选物；要改目标需重新分割，不能把旧掩码当成新物体。规划证据冻结外参、TCP、基座偏移、模型身份等配置；可视化使用该快照，后续修改标定不会改写旧计划含义。早期没有快照的临时结果须重新运行离线预览。

服务仅监听 `127.0.0.1:18766`。`POST /grasps` 输入 JSON：`frame=color_optical`、`units=m`、`cloud_npz`（base64 编码的压缩 NPZ，`points` 为 N×3 float32 米制点云，`target` 为 N 个 bool）、`max_width_m`。返回候选 `score/rotation/translation/width/height/depth`、筛选计数、耗时、显存和模型来源。NPZ 禁止 pickle。推理不读取 API 凭据，不访问 DashScope。

GraspNet X 为接近轴，Y 为闭合轴；Piper 工具 Z 为接近轴，Y 为闭合轴。工具旋转为 `R_grasp @ [[0,0,1],[0,1,0],[-1,0,0]]`，指尖为 `t_grasp + depth * R_grasp[:,0]`，然后应用现有外参及基座偏移，减去工具坐标 `[0,0,0.135] m` 得到法兰目标。深度和 TCP 各处理一次。

官方解码宽度含 1.2 倍开口余量：开口筛选用解码宽度，闭合命令为 `max(0, width / 1.2 * 1000 - 4) mm`。它是网络估计而非物体宽度实测；真实夹紧仍由原反馈判据确认。GraspNet 模式拒绝 `--approach-pitch-deg` 覆盖，避免固定角度覆盖网络朝向。

## 模型环境与来源

开发机独立环境：`build/piper-graspnet-20261117/model-venv`，Python 3.10、Torch 2.7.1+cu118、CUDA Toolkit 11.8、RTX 4060 Laptop。不将 Torch/CUDA 安装到 Go2，不改现有 SAM 环境。完整冻结记录在 `build/piper-graspnet-20261117/evidence/model-environment.txt`。

官方源码 baseline 固定 `280c215129f759ed8649cb4e89fc5dfee55f4f80`，API 固定 `eb57dd2092d8dbe05312a29c3d0c22f3226efbfc`。两项扩展无需源码补丁；`build_ext --inplace` 因上游包目录布局无法复制产物，改用标准 pip 安装成功。

官方 [Google Drive](https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view) 本轮下载额度耗尽。实际使用 [GeneralVLA 提供的镜像](https://huggingface.co/AIGeeksGroup/GeneralVLA/tree/main/checkpoints/v1)，其 LFS SHA256 与 [A2 项目镜像](https://huggingface.co/dgrachev/a2_pretrained/tree/main) 一致，下载后验证相同哈希：`60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`。`torch.load(weights_only=True)` 和网络严格参数匹配通过。**尚未与官方链接原文件独立比对**；不能将镜像一致性描述成官方真实性认证。[来源记录](../../config/piper_vision_grasp/provenance/graspnet-model-20261117.json)保留实际信息。

在新的开发机环境复现时，在仓库根目录 Bash 终端执行以下安装步骤；当前环境已安装，无需重复。每段成功后继续，终端不必保持：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GN_ROOT="$PWD/build/piper-graspnet-20261117"
python3 -m venv --system-site-packages "$GN_ROOT/model-venv"
git clone https://github.com/graspnet/graspnet-baseline.git "$GN_ROOT/graspnet-baseline"
git -C "$GN_ROOT/graspnet-baseline" checkout 280c215129f759ed8649cb4e89fc5dfee55f4f80
git clone https://github.com/graspnet/graspnetAPI.git "$GN_ROOT/graspnetAPI"
git -C "$GN_ROOT/graspnetAPI" checkout eb57dd2092d8dbe05312a29c3d0c22f3226efbfc
"$GN_ROOT/model-venv/bin/pip" install torch==2.7.1 --index-url https://download.pytorch.org/whl/cu118
"$GN_ROOT/model-venv/bin/pip" install 'numpy<2' 'open3d==0.19.0' 'Cython<3' ninja wheel gdown
"$GN_ROOT/model-venv/bin/pip" install --no-build-isolation "$GN_ROOT/graspnetAPI" 'numpy<2' 'opencv-python<4.12'
export CUDA_HOME=/usr/local/cuda-11.8
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=8.9 MAX_JOBS=2
(cd "$GN_ROOT/graspnet-baseline/pointnet2" && "$GN_ROOT/model-venv/bin/pip" install --no-build-isolation --no-deps .)
(cd "$GN_ROOT/graspnet-baseline/knn" && "$GN_ROOT/model-venv/bin/pip" install --no-build-isolation --no-deps .)
```

CUDA 构建必须能看见 GPU；KNN 的上游安装脚本依靠 `torch.cuda.is_available()` 选择 CUDA 分支。不要在 GPU 不可见的容器里误装 CPU 扩展。权重保存到 `$GN_ROOT/checkpoint-rs.tar` 后核对上述 SHA256；可先尝试官方 gdown，失败时使用文中镜像并如实保留来源。权重无需解包。

## 已构建入口的离线使用

三包完整构建命令统一维护在 [Go2 运行手册](go2_runbook.md#2026-11-17-graspnet-接入构建与离线验收)。以下均在开发机执行，不 source ROS，不连接真机。终端 A、B、C 分别先进入仓库根目录并设置：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GN_SHARE="$PWD/build/piper-graspnet-20261117/overlay/install/vlm_nav/share/vlm_nav"
export GN_EVIDENCE="$PWD/build/piper-graspnet-20261117/evidence"
```

**终端 A：模型服务，保持运行。** 启动脚本为 `serve-graspnet` 自动选择独立模型环境；可用 `PIPER_GRASPNET_PYTHON` 覆盖解释器，`PIPER_GRASPNET_SOURCE`/`PIPER_GRASPNET_CHECKPOINT` 覆盖源码和权重路径。

```bash
bash "$GN_SHARE/scripts/piper_vision_grasp.sh" serve-graspnet \
  --config "$GN_SHARE/config/piper_vision_grasp/semantic-graspnet.json" \
  --output-root "$GN_EVIDENCE/services" --run-id model-local-01
```

**终端 B：诊断，保持运行。** 看到 `MODEL_READY` 后才执行预览；`WAIT_VISION` 时根据 reason 修复环境、权重或端口占用，服务保留等待，不视为模型已就绪。

```bash
bash "$GN_SHARE/scripts/piper_vision_grasp.sh" diagnostics --output-root "$GN_EVIDENCE/services"
```

**终端 C：保存数据预览，完成后自动退出。** 不重新采集、不重新访问 Qwen；更换 `--input` 为包含 metadata、observation、三帧深度和 mask00–02 的目录。每次使用新的 run-id，防止覆盖证据。

```bash
bash "$GN_SHARE/scripts/piper_vision_grasp.sh" preview \
  --config "$GN_SHARE/config/piper_vision_grasp/semantic-graspnet.json" \
  --input "$PWD/build/piper-translation-applied-20261117/evidence/translation-red-grasp-20261117-01/before-3e77a44a" \
  --output-root "$GN_EVIDENCE/previews" --run-id saved-red-01
```

退出 0 且结果 `PREVIEW_READY` 才表示保存数据得到完整计划；退出 1、`WAIT_PLAN` 表示没有可行候选，应查看 `plan.json` 中筛选计数和拒绝原因，不继续执行真机流程。`CHECK_OK` 仅为配置/依赖检查，不包含模型推理成功或抓取成功。

**终端 C：导出该预览的 3D 图，完成后自动退出。** `WAIT_PLAN` 的预览也可导出，便于检查目标与机械臂关系。

```bash
bash "$GN_SHARE/scripts/piper_vision_grasp.sh" visualize \
  --config "$GN_SHARE/config/piper_vision_grasp/semantic-graspnet.json" \
  --input "$GN_EVIDENCE/previews/saved-red-01" \
  --output-root "$GN_EVIDENCE/visualizations" --run-id saved-red-01
```

成功判据为 `VISUALIZED`，产物在该运行目录的 `graspnet.html`、`graspnet.png`。HTML 自带绘图库，可离线打开。默认 URDF 来自开发机既有 Piper 资料，可用 `--urdf` 指定同布局的描述文件；缺少模型时报错，不用虚构机械臂替代。目标凸包仅表示可见表面，不表示隐藏形状。图中的路径是规划路点，不是已执行轨迹或完整扫掠碰撞证明。

使用结束 Ctrl+C 停止终端 A/B。本手册未安排部署、真机采集或 `run`；现场恢复须依据届时最新状态及授权。

## 本轮验证结果与文件

源码测试为新增 22 项 + 原有 158 项，共 180 项通过；五项 ctest 包装通过，colcon 的 185 汇总包含包装，不是 185 个 pytest 用例。三包构建、Python 3.8 语法、Bash 语法、安装 `CHECK_OK` 通过，39 个运行/配置来源文件逐字节一致。未运行全仓库测试或真机验证。

最终安装入口真实推理结果如下，三组均没有通过目标与开口筛选的候选，故均为 `WAIT_PLAN`，**“至少一组完整可行路径”的验收尚未满足**。

| 保存场景 | 原始候选 | 非目标候选 | 目标候选超 70 mm | 最终候选 | 推理耗时 |
|---|---:|---:|---:|---:|---:|
| 人工对准记录 | 216 | 215 | 1 | 0 | 0.145 s |
| 物体移位记录 | 151 | 150 | 1 | 0 | 0.124 s |
| 当前外参失败抓取记录 | 121 | 121 | 0 | 0 | 0.337 s |

峰值 Torch 分配约 662 MiB；实际进程驻留约 1148 MiB，与已有 SAM 的约 3218 MiB 同时存在，没有本轮 OOM。ROI 采样探索增加少量候选，但仍不可达，因此没有纳入生产配置。没有放宽开口/关节限制、修改外参/TCP，或返回旧算法假装成功。

证据根目录为 `build/piper-graspnet-20261117/evidence/`：

- `offline-summary.json`：三组实际推理、筛选、耗时、模型 hash 和零 CAN 发送记录。
- `replay-{aligned,moved,failed}-final/`：最终安装入口的观察、计划、结果及参数快照。
- `visualization-{aligned,moved,failed}-final/graspnet.html`、`graspnet.png`：目标可见形状、相机坐标轴、Piper 模型；因为本轮无有效候选，图中没有虚构选中夹爪或运动路径。
- `pytest-final.log`、`build-final.log`、`ctest-final.log`、`test-result-final.log`、`installed-manifest.json`：代码与安装验证。
- `gpu-coexistence.txt`、`model-service-stop.json`：并存显存及本轮服务结束记录。原 SAM 服务未停止。

外参文件 SHA256 保持 `710f7781bcbab5f86844b1701adb912bb47a8cb81d16ba1ebf94d3aa0d4f379a`，未重新标定。官方局部碰撞过滤只涉及简化夹爪与沿候选接近轴的短路径，不能替代真实 Piper 全路径、相机支架及 TCP/外参精度的现场验证。

### 现场接入状态：2026-11-17 回零后首次尝试

用户随后明确授权回零后抓取，机载新目录现为 `/home/unitree/piper-vision-graspnet-20261117`，使用原 Python3.8 控制环境；40文件清单和机载 CHECK_OK 已通过。旧语义目录保留，不自动切换旧入口。六轴已按5%速度回零，当前最后反馈均0°、故障0、开爪68.4mm。

现场新采红色积木成功，但 GraspNet 全场景136个候选均不在目标掩码内，WAIT_PLAN且未发送抓取动作；目标附近离线采样得到5个候选，原IK均未通过，未应用该采样策略。证据在 `build/piper-graspnet-live-20261117/evidence/`，详见 session_handoff 最新条目。该次现场结果仍不是抓取验收成功。此次18766服务/隧道已结束，SAM/18765保留；后续现场步骤须重启对应服务、重采数据，不能直接执行保存计划。
