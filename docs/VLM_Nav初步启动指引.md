# VLM_Nav 初步启动指引

本流程只写当前已经验证可执行的路径。任一检查失败时停止在当前步骤，不继续叠加系统，
也不通过手工发布假的 ready 信号绕过 gate。

## 推荐入口：一键启动正式 VLM 导航

确认 Go2、XT16、Go 上位机和 D435 已供电，且第 1 节的 D435 进程已经启动后，笔记本只需
运行：

```bash
cd /home/isee-pst/unitree_ros2
./VLM_Nav/scripts/VLMNav-go.sh "红色灭火器"
```

将目标描述替换为现场唯一目标并保留引号。脚本依次完成网络和 NTP 检查、D435 单
publisher 与 640×480×15/sync/alignment 参数检查、单位报告/API 环境检查、30 秒
XT16/IMU preflight、Nav2/RViz/rosbag、相机外参、raw-depth VLM 和真实 Qwen API gate。
正常启动不再运行相机帧率探针或 `go2_camera_preflight`。脚本始终保持
`easy_case_mode=false`，不会使用未验收的 Easy Case。

启动时会额外打开一个监控终端，其中两个标签页分别持续显示送入机器狗 bridge 的
`/cmd_vel_bridge` 和 `/vlm_nav/diagnostics`。该窗口只订阅，不发送控制消息。

按现场状态依次输入：

此时**先让狗趴着**

START_NAV：RViz 地图和 footprint 正常，已用遥控器移动到起点并停稳；

**站起来**之后调好起点位置ARM

ARM：实体急停可用、场地清空且没有其他 Sport 控制器；

ENABLE_VLM ：1 Hz 相机预览、地图、路径区域和目标描述均正确。

任何 gate 失败时脚本都会停止，不得手工发布 ready 或绕过。以下分节保留 D435
恢复方法、诊断细节和验收标准；正常启动不要再逐节重复启动同一套节点。

终止/结束测试：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/stopgo.sh
```

先确认机器人已经收到 DISARMED，然后清理真正持锁的进程：

```bash  
fuser -v /tmp/manualnav2.lock
fuser -k -TERM /tmp/manualnav2.lock

while ! flock -n /tmp/manualnav2.lock -c true; do
	sleep 0.2
done

echo "manualnav2 lock released"

# 清理每次失败重启产生的监视终端进程：

pkill -TERM -f '[r]os2 topic echo /cmd_vel_bridge' || true
pkill -TERM -f '[r]os2 topic echo /vlm_nav/diagnostics' || true

# 确认后重启：

flock -n /tmp/manualnav2.lock -c 'echo LOCK_FREE'
```

## 0. 启动前检查

- Go2、禾赛 XT16 和 Go 上位机均已供电，笔记本已连接 `enp4s0`。
- 场地封闭空旷，操作者持遥控器并能随时急停。
- 关闭 Go 上位机上的 `realsense-viewer`。
- 健康且参数正确的 D435 进程直接复用；只有检查失败时才停止并重启。旧的
  `go2_system.launch.py`、bridge 或 `manualnav2.sh` 必须在原终端正常停止，
  不要启动第二套相同节点。
- 系统运行期间禁止突然校时。先确认 Go 的 `NTPSynchronized=yes`；若不是，先修复 NTP，
  再重新启动相机、FAST-LIO、Nav2 和 VLM。

笔记本检查网络：

```bash
ip -brief address
ping -I enp4s0 -c 3 192.168.123.18
```

应看到 `enp4s0` 具有 `192.168.123.x` 地址，且 Go 上位机 `192.168.123.18` 可达。

Go 时钟检查：

```bash
ssh unitree@192.168.123.18 'timedatectl show -p NTPSynchronized --value'
```

必须输出 `yes`。

## 1. Go 终端：启动 D435 raw RGB-D

D435 连接在 Go 上位机，不能在笔记本运行 `VLM_Nav/scripts/01_camera.sh`。

### 1.1 笔记本先判定是否复用，若不重复启动跳到2.，若停止重启跳到1.2

每次启动前先在笔记本执行：

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source VLM_Nav/scripts/go2_network_env.sh
export ROS_LOG_DIR=/tmp/d435_check_roslog
mkdir -p "$ROS_LOG_DIR"

camera_publishers() {
  timeout 5 ros2 topic info "$1" 2>/dev/null \
    | awk '/Publisher count:/ {print $3; exit}'
}
camera_param_is() {
  timeout 5 ros2 param get /camera/camera "$1" 2>/dev/null \
    | grep -Fqx "$2"
}

if [[ "$(camera_publishers /camera/camera/color/image_raw)" == 1 &&
      "$(camera_publishers /camera/camera/depth/image_rect_raw)" == 1 ]] &&
   camera_param_is depth_module.depth_profile 'String value is: 640x480x15' &&
   camera_param_is rgb_camera.color_profile 'String value is: 640x480x15' &&
   camera_param_is enable_sync 'Boolean value is: True' &&
   camera_param_is align_depth.enable 'Boolean value is: False'; then
  echo 'D435进程已启动，不重复启动'
else
  echo 'D435未按要求运行，进入停止并重启流程'
fi
```

**若输出 `D435进程已启动，不重复启动`，不要再运行相机 launch**，直接运行文首的
`VLMNav-go.sh`。该判定同时要求 RGB/raw depth 各只有一个 publisher，且四个关键参数
完全匹配；只看到进程名或 topic 名不算通过。

### 1.2 检查异常进程的来源并停止

**只有上一步未通过时**才登录 Go：

```bash
ssh -tt unitree@192.168.123.18

camera_processes() {
  ps -eo pid=,comm=,args= | awk '
    (($2 == "python3" || $2 == "ros2") &&
     index($0, "/opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py")) ||
    ($2 == "realsense2_came" && index($0, "/realsense2_camera_node"))
  '
}

camera_processes

mapfile -t camera_launch_pids < <(
  camera_processes | awk '$2 == "python3" || $2 == "ros2" {print $1}'
)
if (( ${#camera_launch_pids[@]} != 1 )); then
  printf 'ERROR: 找到 %d 个 D435 launch，不能自动选择；请回各自原终端 Ctrl+C。\n' \
    "${#camera_launch_pids[@]}"
  printf 'PID: %s\n' "${camera_launch_pids[*]:-none}"
else
  camera_launch_pid="${camera_launch_pids[0]}"
  ps -o pid,ppid,lstart,stat,args -p "$camera_launch_pid"
  systemctl status "$camera_launch_pid" --no-pager 2>/dev/null | sed -n '1,16p' || true
fi
```

**找到0个则直接跳转1.3**

优先回启动它的原终端按 `Ctrl+C`。若 `systemctl status` 显示它属于明确的
`.service`，应使用输出中的真实单元名停止或重启，不能再手工启动第二套相机：

```text
# 系统服务（把名称替换为 systemctl status 显示的真实单元）
sudo systemctl stop <实际单元.service>
sudo systemctl start <实际单元.service>

# 用户服务
systemctl --user stop <实际单元.service>
systemctl --user start <实际单元.service>
```

若 service 的 `ExecStart` 参数不符合 640×480×15、sync=true、alignment=false，先修正
service 配置再重启；不要用第二个手工 launch 掩盖错误配置。只有确认进程不是 service
管理、且上面恰好找到一个 launch PID 时，才在同一个 Go 终端精确停止：

```bash
kill -INT "$camera_launch_pid"

for _ in {1..20}; do
  if [[ -z "$(camera_processes)" ]]; then
    break
  fi
  sleep 0.5
done

remaining_camera_processes="$(camera_processes)"
if [[ -n "$remaining_camera_processes" ]]; then
  printf '%s\n' "$remaining_camera_processes"
  echo 'ERROR: D435 进程未完全退出，不要重启；继续检查上述 PID 或所属 service。'
else
  usb_path="$(lsusb -d 8086:0b07 | awk '{gsub(":", "", $4); print "/dev/bus/usb/" $2 "/" $4}')"
  if [[ -n "$usb_path" ]]; then
    fuser -v "$usb_path" 2>&1 || true
  fi
  echo 'D435 进程已停止；fuser 无占用输出后才可重启。'
fi
```

禁止使用 `pkill`、`killall` 或模糊匹配批量结束 ROS 进程。若 `SIGINT` 后仍未退出，
不要叠加启动；先查清仍存活的精确 PID 或自动拉起它的 service。

### 1.3 确认释放后重启

只有 1.1 检查失败且 1.2 已确认进程和 USB 占用全部释放时，才在 **Go 终端执行：**

```bash
source /home/unitree/unitree_ros2/setup.sh
source /home/unitree/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/realsense_roslog
mkdir -p "$ROS_LOG_DIR"

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_sync:=true \
  align_depth.enable:=false
```

**保持该终端运行**，并确认出现 `RealSense Node Is Up`。

当前禁止启用 `align_depth`：实测 aligned depth 只有约 6.4 Hz，RGB 约 13 Hz，
RGB-depth 时间差中位数约 67 ms，不能满足 50 ms 门槛。Go 端只运行 D435 publisher，
不要在 Go 上运行 RGB/depth probe；否则额外本地订阅会拖慢最终数据拓扑。

若标准 multicast discovery 导致 Go 上的 CycloneDDS 在相机初始化前崩溃，停止本流程并
按已记录的静态 peer 方案排障。该方案尚未写入仓库正式配置，不在这里临时拼接配置。

## 2. raw RGB-D 异常时的诊断（正常启动跳过）

两个终端都先执行：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source scripts/common.sh
source /home/isee-pst/venv/co-nav-real/bin/activate
source scripts/go2_network_env.sh
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
export ROS_LOG_DIR=/tmp/raw15_probe_roslog

# 先在原终端停止 VLM launch、RViz Camera、Qwen probe 和其他图像探针
# 若 VLM 曾使能，先执行：
# ros2 param set /vlm_nav enabled false
```

```bash
# 先进行
ros2 topic info /camera/camera/color/image_raw
ros2 topic info /camera/camera/depth/image_rect_raw
```

终端 1：

```bash
python scripts/raw_depth_probe.py rate \
  --topic /camera/camera/color/image_raw --duration 30
```

终端 2：

```bash
python scripts/raw_depth_probe.py rate \
  --topic /camera/camera/depth/image_rect_raw --duration 30
```

两条命令必须在同一时间窗口运行。两路均须满足：

- `unique_hz` 为 14–16 Hz；
- `duplicate < 1%`；
- `rollback = 0`；
- `passed = true`。

若失败，保留输出并停止；不要继续启动 Nav2 或 VLM。最终正确拓扑是 Go 只发布、笔记本
订阅 RGB 和 raw depth。

上述 30 秒探针只用于相机检查失败、图像明显卡顿或 snapshot 长期无法形成时的排障，
不属于正式启动 gate。不要再运行 `go2_camera_preflight`。

## 3. 手动排障参考：发布相机外参

当前 XT16 标定文件不发布相机外参，因此单独发布已记录的
`base_link -> camera_link`：

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source VLM_Nav/scripts/go2_network_env.sh

ros2 run tf2_ros static_transform_publisher \
  --x 0.32715 --y -0.00003 --z 0.04297 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id camera_link
```

**保持运行。**

可在另一终端验证：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 4. 手动排障参考：启动唯一 Nav2/bridge 栈

使用已有安全启动器；它会生成正式 preflight、启动 XT16/FAST-LIO/SLAM/Nav2/RViz，
并在人工确认后才启动和 ARM bridge：

```bash
cd /home/isee-pst/unitree_ros2
./VLM_Nav/scripts/manualnav2.sh
```

执行顺序：

1. Go2 在 30 秒 preflight 期间保持静止。若是一直站很可能摔倒，所以趴着测。
2. Nav2 和 RViz 就绪后，用遥控器把 Go2 移到起点。
3. 松开摇杆，确认机器人静止且遥控器不再发送 Sport 指令。
4. **输入 `START_NAV`**。
5. 脚本启动 bridge 并确认 `SYSTEM_READY`、`NAV_READY` 后会提示输入 `ARM`；
   此时先保持等待，**不要 ARM**，继续执行下面的 Qwen 和状态检查。

保持该终端停在 `ARM` 提示处，直到第 8 节明确要求 ARM。ARM 后输入
`CANCEL` 可取消当前 Nav2 目标，全部测试结束后输入 `DISARM`。不要同时启动
另一套 `go2_system.launch.py` 或 bridge。

### 4.1 单位校准报告（已一次验收）

当前 D435 的 raw depth 到米制距离已完成两次实物对照，误差约 1 cm 以内；
启动流程不再要求现场重新测量沿光轴距离。仅确认已安装的一次性校准报告存在：

```bash
test -r ~/.config/vlm_nav/raw_depth_units_verified.json && \
  echo 'raw-depth unit calibration report: present'
```

VLM 启动时仍会 fail-closed 校验报告中的 640×480×15、sync=true、
alignment=false、raw topic 和 depth scale。报告缺失或不匹配时不得手工绕过；
只有更换相机、修改 profile/depth scale，或怀疑深度值异常时才重做实物单位校准。

## 5. 笔记本终端 5：可选的独立 Qwen 图片诊断

此步骤只上传一帧 D435 RGB 图片并验证响应，不会控制机器人。它用于独立诊断，
不负责发布 `/vlm_nav/vlm_api_ready`；正式启动检查由第 7 节的 VLM 节点自动完成。

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source /home/isee-pst/venv/co-nav-real/bin/activate
source VLM_Nav/scripts/go2_network_env.sh

python VLM_Nav/scripts/qwen_latency_probe.py \
  --topic /camera/camera/color/image_raw \
  --target "red chair" \
  --mode target \
  --samples 1 \
  --timeout 8 \
  --max-latency 5
```

API key、workspace 和模型继续使用终端从 `~/.bashrc` 加载的现有环境变量。

## 6. 状态检查

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source VLM_Nav/scripts/go2_network_env.sh

ros2 topic echo /vlm_nav/system_ready --once --field data
ros2 topic echo /vlm_nav/nav_ready --once --field data
ros2 topic echo /vlm_nav/go2_bridge_state --once --field data
ros2 topic echo /vlm_nav/control_armed --once --field data
```

尚未输入 `ARM` 时应依次看到：

```text
True
True
DISARMED
False
```

如果明确进行 RViz 手动 Nav2 测试并输入了 `ARM`，后两项才应变为 `ARMED`、`True`。

## 7. 手动排障参考：启动 raw-depth VLM 输入诊断，但不自动 enable

正式 Go2 VLM 已迁移到 raw depth，可以以禁用状态启动：

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source VLM_Nav/install/setup.bash
source /home/isee-pst/venv/co-nav-real/bin/activate
source VLM_Nav/scripts/go2_network_env.sh

ros2 launch vlm_nav go2_vlm.launch.py enabled:=false
```

VLM 节点会复用它已有的 RGB 订阅，在笔记本本地发布约 1 Hz 的
`/vlm_nav/camera_preview`。`manualnav2.sh` 启动的 RViz 中
`Camera RGB (1 Hz preview)` 面板会显示该画面。禁止把 RViz 直接指向
`/camera/camera/color/image_raw`，否则会增加一路 15 Hz 跨机订阅并可能再次拉低帧率。

可选检查预览频率：

```bash
timeout 10 ros2 topic hz /vlm_nav/camera_preview
```

新终端检查分层 readiness：

```bash
ros2 topic echo /vlm_nav/input_ready --once --field data
ros2 topic echo /vlm_nav/autonomy_ready --once --field data
ros2 topic echo /vlm_nav/diagnostics --once
```

`input_ready=true` 仅表示 raw RGB-D、双 CameraInfo、90%/50 ms 同步窗口、
color↔depth 静态外参和 depth stamp map TF 可用。`autonomy_ready=true` 还要求
通过上述正式单位报告，并且 `/vlm_nav/vlm_api_ready=true`。VLM 节点取得首个
immutable snapshot 后会自动执行一次真实 Qwen 请求并独占发布该 topic；运行中每次
真实 target/frontier 响应都会刷新状态，timeout、异常、空响应或无效结构立即置
`false`。

图像时刻比当前 TF 缓存新数十毫秒属于正常等待；节点会保留该帧并按原时间戳
重试，不使用最新 TF 替代，也不将明确的 `extrapolation into the future`
计为 TF failure 或反复输出 WARN。若 30 秒后 `input_ready` 仍为 `false`，停止 VLM，
重新检查两机 NTP 和 `map -> camera_depth_optical_frame`；不得强行 enable。

任一项未通过时，不要执行：

```text
ros2 param set /vlm_nav enabled true
```

禁止手工发布假 `true` 绕过 gate。单位报告、自动 Qwen 启动检查和真实 map 三点投影
验收均通过后，才允许显式 enable 执行短 VLM 自动任务。

## 8. VLM 正式导航任务验收

本流程只使用正式八方向扫描、目标落地和前沿探索链路，不使用未验收的
Easy Case。首次可选择唯一、静止、易描述且能在扫描过程中看到的目标，
以缩短运动链路；目标与任务路径必须位于 3 m 滚动半径内。通道与停车区无人、
无楼梯、坑洞、玻璃或移动障碍。操作者始终持遥控器，不站在机器人与目标之间。

### 8.1 DISARMED 状态下设置任务

`VLMNav-go.sh` 使用命令行中的目标描述启动 VLM，保持 `enabled=false`，并等待
`input_ready`、`vlm_api_ready`、`autonomy_ready` 全部为 `True`；同时强制检查
`easy_case_mode=False`。任一项失败都会在 ARM 前退出。任务运行中禁止修改目标描述
或任务模式。

### 8.2 ARM 后最后一次门控检查

确认实体急停可用、代价地图与 footprint 正常、现场清空后，在启动脚本提示处输入
`ARM`。脚本确认 bridge 为 `ARMED`、`control_armed=True` 后，再次检查 system、Nav2、
API、输入与 autonomy gate；任一项不符合就退出并 DISARM，不启用 VLM。全部通过后，
核对 RViz 并输入 `ENABLE_VLM`。

### 8.3 执行与监视

两个新终端分别持续观察：

```bash
ros2 topic echo /vlm_nav/state --field data
```

```bash
ros2 topic echo /vlm_nav/diagnostics
```

保持 RViz 中的 RGB、VLM 标注图、目标 marker、全局/局部代价地图和路径可见。
`ENABLE_VLM` 确认后由启动脚本只执行一次 `enabled=true`，不要在其他终端重复设置。
正常状态从 `SCANNING` 开始；看到目标后进入
`TARGET_CONFIRMING`，未看到时可经 `FRONTIER_SELECTING` / `EXPLORING` 后重新扫描。
落地后经 `TARGET_ALIGNING` / `APPROACHING` /
`APPROACH_STOPPING` 到 `SUCCEEDED`；深度重观测时可短暂出现
`TARGET_REOBSERVING`。不要以 Nav2 动作单独返回成功作为验收结论。

出现以下任一情况，本次立即记为 FAIL 并按 8.5 停止：

- VLM 标注或 marker 指向错误物体，或目标投影明显不合理；
- `/vlm_nav/vlm_api_ready` 或其他 gate 变为 `False`，或 `vlm_enabled` 在成功前自动变为
  `False`；
- 状态进入 `SENSOR_WAITING`、`API_ERROR` 或 `FAILED`；
- 路径进入禁区，机器人偏离安全区，产生持续振荡，或需要人工接管。

### 8.4 PASS 判定与证据

仅当以下条件同时满足时记为 PASS：

- VLM 标注的是预期物体，且使用该次不可变 RGB-D snapshot 完成目标落地；
- Nav2 路径与实际运动均在自由空间内，bridge 全程未进入 `FAULT`；
- `/vlm_nav/state` 最终为 `SUCCEEDED`；该状态只会在 Nav2 动作已进入终态、里程计
  连续 3 帧判定静止，且复核距离通过后进入；
- 最终 diagnostics 中 `approach_status` 显示 `distance_m<=0.810`；进入成功状态后
  `action_terminal` 和 `stationary` 计数会被清零，不将它们的最终显示值作为失败依据；
- 机器人实际已停稳，`vlm_api_ready` 仍为 `True`，`last_failure_reason=none`。

`manualnav2.sh` 已自动录制 Nav2/bridge 证据，路径会显示在其终端。VLM 证据位于：

```text
~/.ros/vlm_nav/arm_records/arm_YYYYmmdd_HHMMSS_ffffff/
```

验收时保留该目录中的 `events.jsonl` 和标注图，并记录当次 rosbag 路径、目标描述、
PASS/FAIL 与失败原因。

Go2 正式配置会把每次导航测试的标记图和事件保存到：

```text
~/unitree_ros2/log/VLM_feedback/arm_YYYYmmdd_HHMMSS_ffffff/
```

只保留最近 5 次导航测试的完整文件夹；第 6 次启用 VLM 时自动删除最旧的一次。

### 8.5 正常结束或取消

任务 PASS 或中途取消时都运行第 9 节的 `stopgo.sh`。它先禁用 VLM，再取消 Nav2
目标，最后 DISARM 并验证状态；不要只取消 Nav2 而保持 VLM enabled，否则节点可能
再次规划。

## 9. 正常停止与紧急停止

正常结束或中途取消时，在另一终端运行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/stopgo.sh
```

该脚本严格按 `VLM enabled=false`、取消 Nav2 目标、bridge DISARM 的顺序执行，并确认
`go2_bridge_state=DISARMED`、`control_armed=False`。启动脚本随后会关闭本次启动的所有
子进程并保留日志和 rosbag。

出现人身、碰撞或失控风险时，先使用手中遥控器实体急停，然后在任意笔记本终端执行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/stopgo.sh
```

该命令不代替实体急停。保留 rosbag 和 ARM 记录排障；未查明原因前不得重新 ARM。
