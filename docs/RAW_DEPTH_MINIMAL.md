# Go2 raw-depth 最小闭环

独立工具：`scripts/raw_depth_probe.py`。不发布导航目标或运动命令，不改导航状态机。
几何调用使用本机现有 `/usr/local/lib/librealsense2.so.2.53.1` 的官方 C API，
通过 Python 标准库 ctypes 传入 ROS 图像数组；没有安装新 SDK 或实现自定义对齐算法。

## 相机与帧率

在 Go 原 ROS 环境启动（先停止已有相机进程）：

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_sync:=true align_depth.enable:=false
```

Go 可运行复制到 `/tmp/raw_depth_probe.py` 的相同工具。两端分别开两个终端，
同一窗口各测一路 60 秒：

```bash
python3 scripts/raw_depth_probe.py rate --topic /camera/camera/color/image_raw
python3 scripts/raw_depth_probe.py rate --topic /camera/camera/depth/image_rect_raw
```

`unique_hz` 用唯一帧数除以整个采样时长，防止只用有帧的短区间计算出虚假的 15 Hz。
通过条件：14–16 Hz、duplicate <1%、rollback=0。保留失败窗口证据。

## 笔记本环境与 snapshot

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source scripts/common.sh
source /home/isee-pst/venv/co-nav-real/bin/activate
source scripts/go2_network_env.sh
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
export ROS_LOG_DIR=/tmp/raw15_probe_roslog
```

捕获一份含真实 map TF 的 snapshot，文件必须是新路径：

```bash
python scripts/raw_depth_probe.py capture --output /tmp/raw_snapshot.npz
```

自动订阅 color/depth Image 和各自 CameraInfo；仅接受 stamp 差 ≤50 ms 的一组。
保存图像副本、两套 CameraInfo、两路 stamp、双向 optical 外参以及 depth stamp 时刻的 map TF。
同时保存同名 PNG 供人工选原始 color pixel。map TF 缺失会失败，不替代成最新 TF。

单位检查阶段可用 `capture --camera-only --output /tmp/raw_units.npz`。
此文件明确保存 `T_map_depth_optical=null`，只能验证相机坐标，不能声称 map 闭环通过。

## 单位实测

机器人静止，中心平面距 depth 光心约 1 m，记录沿光轴实测距离。
2026-09-15 读取设备 depth scale 为 `0.0010000000474974513`，它本身不是 ROS 单位验收结果。
已有未校验快照 `/tmp/raw15_unit_pending.npz` 中心 raw=1392，不能据此认定目标距离。

以下是填写真实测量值后的命令格式（尖括号内容必须替换，勿原样执行）：

```text
python scripts/raw_depth_probe.py units /tmp/raw_units.npz \
  --pixel 320 240 --distance-m <沿光轴实测距离> \
  --device-scale <设备实读scale> --meters-per-unit <待验证ROS转换系数> \
  --fps 15 \
  --output /tmp/raw_units_verified.json
```

工具输出原值、候选换算值、设备 scale 换算值与实测距离；误差 ≤max(5 cm,5%) 才标记通过。
不通过实测报告，project 不运行。该单位报告只用于这次固定相机/profile 的实验。

## 人工单点与 VLM

先在原始 PNG 上选择中心、左侧和右侧目标各一个点，对每个点运行：

```bash
python scripts/raw_depth_probe.py project /tmp/raw_snapshot.npz \
  --units-report /tmp/raw_units_verified.json --pixel 320 240 --delay 5
```

输出对应 depth pixel、raw value、米制 depth、depth optical 3D 和 map point。
坐标链调用官方 color→depth pixel 函数，再按 depth 内参反投影，最后乘缓存 TF。
只有单点最近邻读取，无 bbox 多点采样或持续 full-frame alignment。
当前实测两套 CameraInfo 畸变系数全零，可按无畸变模型使用；非零时要求显式提供经确认的 SDK 模型枚举。

`--delay 5` 在载入快照后等待，再对同一点重复计算，检查快照哈希及结果未变。
这是离线快照绑定验证；实际 map TF 存在后还需运行一次真实采集快照验收。
比较方向与同一坐标口径的实测距离，误差暂定 ≤max(5 cm,5%)。

人工三点通过后，用既有 VLM 环境变量执行一次真实请求：

```bash
python scripts/raw_depth_probe.py project /tmp/raw_snapshot.npz \
  --units-report /tmp/raw_units_verified.json --target 'red chair'
```

复用现有 VLM client 的坐标转换和 target_pixel。上传及投影绑定同一个 snapshot，
不读取响应返回时的新图像或 TF。VLM 可见性/像素无效则明确失败。

## 软件检查

```bash
python -m pytest -q test/test_raw_depth_probe.py
```

使用官方库验证已知 1 m 平面的中心、左右映射及已知 map 变换，同时检查快照独立副本及 5 秒延迟 CLI。
这些是合成几何检查，不能替代实物距离、双机稳定性和真实 map TF 验收。

## 2026-09-15 实施结果（硬件闭环未通过）

相机实际恢复到 640×480×15、sync=true、alignment=false。
Go 单独复测曾达到 RGB 14.89 Hz、depth 15.01 Hz，duplicate/rollback 均为 0。
随后双机同测仍失败；最终 60 秒使用整个窗口计数，结果如下：

| 位置 | 数据流 | unique Hz | duplicate | rollback |
|---|---|---:|---:|---:|
| Go | RGB | 12.29 | 0 | 14 |
| Go | raw depth | 1.65 | 0 | 15 |
| 笔记本 | RGB | 12.11 | 0 | 0 |
| 笔记本 | raw depth | 1.52 | 0 | 0 |

两个启动日志均出现 `uvc streamer watchdog triggered`（endpoint 130/132）。
数据不足以区分 USB 采集故障、发布阻塞或负载问题，不能仅据该日志认定 USB 硬件损坏。
Go 端回退是探针收到的 header 顺序结果，不等于已经证明主机时钟回退。

证据：`/tmp/raw_depth_minimal_evidence/`，笔记本报告为 `/tmp/raw15_laptop_final_{rgb,depth}.json`。
保存的诊断 snapshot stamp 差 0 ms，两套畸变系数均为 0，中心 raw=1392；实物距离未提供。
设备 scale 已读取，但尚无通过实测的 ROS 单位报告，因此未执行真实图像的米制投影或 VLM 请求。
实际 capture 检查确认 `map` frame 不存在；相机坐标 snapshot 的 map TF 为 null，不伪造 map 验收。

## 2026-09-16/17 双机最终拓扑复测

### Raw RGB + raw depth

测试拓扑只保留 Go 上的 D435 publisher；Go 不运行本地图像 probe/subscriber，
笔记本同时以 sensor-data QoS 订阅 RGB 和 raw depth。相机参数为
640×480×15、`enable_sync=true`、`align_depth.enable=false`。

60 秒结果：

| 数据流 | unique Hz | unique frames | duplicate | rollback |
|---|---:|---:|---:|---:|
| RGB | 14.981 | 899 | 0 | 0 |
| raw depth | 14.997 | 900 | 0 | 0 |

Go `eth0` TX 约 190.13 Mbps；两端 interface error/drop、`UdpRcvbufErrors` 和
`UdpSndbufErrors` 增量均为 0。该最终数据拓扑达到 15 Hz 目标，2026-09-15
双机多 probe 场景中的降频和 rollback 未复现。证据位于
`/tmp/go2_final_topology_run.XEAmfK/`。

### RGB + aligned depth 对照

相机改为相同 profile、`enable_sync=true`、`align_depth.enable=true`；Go 仍只运行
D435 publisher，笔记本只订阅 RGB 和 aligned depth。标准 multicast discovery 下，
新启动的 Go 相机节点会在 RealSense 初始化前稳定 `SIGSEGV`。GDB backtrace 位于
CycloneDDS `libddsc.so.0` 的 `builtins_dqueue_handler -> ddsi_plist_init_frommsg`；
相同相机参数只绑定 loopback 时可启动到 `RealSense Node Is Up`，因此该启动崩溃
属于 DDS discovery 路径，不能归因于 D435、alignment 或 USB。

为完成数据测试，Go 临时保持 `eth0` 数据接口，但关闭 multicast discovery，并只把
笔记本 `192.168.123.222` 配为静态 peer。该设置仅隔离 discovery，未改变 RGB/depth
数据路径；它是运行时配置，尚未写入仓库正式 CycloneDDS 配置。

两轮 60 秒结果均复现降频：

| 窗口 | RGB unique Hz | aligned depth unique Hz | duplicate | rollback | 最近邻绝对 stamp 差 median / P95 / max |
|---|---:|---:|---:|---:|---:|
| 第 1 轮 | 13.712 | 6.806 | 0 | 0 | 67.18 / 335.96 / 403.12 ms |
| 第 2 轮 | 12.989 | 6.395 | 0 | 0 | 67.18 / 335.96 / 470.27 ms |

第 2 轮使用严格同窗计数：RealSense 进程 CPU 为 86.25%，Go `eth0` TX 为
131.40 Mbps；两端 interface error/drop 和 UDP buffer error 增量仍全部为 0。
相机在正式窗口前启动时执行过一次 hardware reset 并自动重新枚举，两个测试窗口内
没有新增 reset/disconnect。现有证据支持 alignment/发布处理路径负载导致输出不足，
不支持物理网络丢包或 UDP socket buffer 丢包假设。证据位于
`/tmp/go2_aligned_final.Y6Xeys/` 和 `/tmp/go2_aligned_repeat.4N3dqJ/`。

### 当前恢复状态

测试结束后相机已恢复为 640×480×15、`enable_sync=true`、
`align_depth.enable=false`，且没有图像 probe/subscriber。启动阶段发生一次 hardware
reset，自动重新枚举后再次达到 `RealSense Node Is Up`。Go 当前仍使用上述 Laptop
静态 peer 的临时 DDS discovery 配置，以避免已复现的 multicast discovery 崩溃。

已完成 raw 最终拓扑复测。

  - RGB：14.998 Hz，900 帧，PASS
  - raw depth：15.013 Hz，901 帧，PASS
  - duplicate/rollback：均为 0
  - Go TX：约 190.37 Mbps；接口及 UDP buffer errors/drops 均为 0
  - 软件检查：3 passed
  - 相机已恢复并保持 raw 配置运行

## 正式 VLM raw-depth 迁移

Go2 profile 已改为订阅：

- `/camera/camera/color/image_raw`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/camera_info`

正式节点与本工具共用 `vlm_nav.geometry.RealSenseGeometry`，VLM 彩色像素必须经
librealsense 官方 color→depth pixel 映射后读取 raw depth。每个请求冻结 RGB、
raw depth、双 CameraInfo、首次取得并缓存的静态 color↔depth 外参，以及 depth
stamp 时刻的 `map → camera_depth_optical_frame`；VLM 返回后不会读取新 depth 或
最新 TF。

输入 readiness 使用最近 30 帧窗口，至少 10 帧且 90% 能在 50 ms 内一一配对。
节点分别发布 `/vlm_nav/input_ready` 和 `/vlm_nav/autonomy_ready`。后者还要求上述
单位报告通过、profile 为 640×480×15、sync=true、alignment=false、topic/device
scale/误差元数据匹配，并且 VLM API ready。单位报告默认路径为
`~/.config/vlm_nav/raw_depth_units_verified.json`；报告缺失时允许只读诊断，但
`enabled=true` 会被拒绝。

## 认为VLM raw-depth 迁移完成

实测 1.06 m；raw=1054，换算 1.054 m，误差 6 mm，verified=true / passed=true。报告：/home/isee-pst/.config/vlm_nav/    raw_depth_units_verified.json   - 官方映射验证：color (320,240) → depth (311,234) → 1.059 m，并成功投影至 map；快照：/tmp/    raw_snapshot_106cm_verified_retry_20260917.npz   - 当前 input_ready=true、depth_units=verified、VLM 禁用且未 ARM；autonomy_ready=false 仅因尚无真实 API heartbeat。Sport 请求    为 0，构建及 6 项测试通过。

VLM 节点独占发布 `/vlm_nav/vlm_api_ready`。首个 snapshot 会在 `enabled=false`
状态下执行一次真实 Qwen 启动检查；其后每个真实 target/frontier 请求的有效响应
置 `true`，timeout、异常、空响应或无效结构立即置 `false` 并触发 fail-closed。

VLM_Nav/vlm_nav/vlm_navigator.py 现在独占发布 /vlm_nav/vlm_api_ready。首个 immutable snapshot 在允许 enable 前自动执行真实
    Qwen 检查；后续每次真实响应刷新状态，超时、异常、空响应或无效结构立即置 false 并禁用 VLM。

  - 真机启动检查使用 qwen3-vl-flash，耗时 2.453 s，当前 vlm_api_ready=true、input_ready=true、autonomy_ready=true；仍保持
    enabled=false、control_armed=false，Sport 流量为 0。

问题2也已解决。
