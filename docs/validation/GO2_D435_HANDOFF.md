# Go2 D435 硬件问题交接

此问题目前阻塞 D435 正式导航输入，但不影响已通过的 LiDAR/IMU DDS 基础检查。
没有刷固件、修改 Go2 文件、修改内核参数或启用运动控制。

## 设备与软件实测

- Go2：Ubuntu 20.04、kernel `5.10.104-tegra`、aarch64。
- D435：USB ID `8086:0b07`，USB 3.2 / 5 Gb/s，serial `151222077264`。
- D435 firmware：`05.12.07.150`。
- RealSense ROS：`4.55.1`，git commit `8a86cb88a428bdefa204759c899b84adc81606ae`。
- librealsense：build/runtime `2.53.1`，唯一库位于 `/usr/local/lib`。
- Go2 的 wrapper checkout 有一处本地修改：把官方 4.55.1 CMake 请求的
  librealsense `2.55.1` 改为 `2.53.1`。
- USB power control：`auto`，autosuspend 2 秒；`usbfs_memory_mb=16`。
- Go2 处于 `nvpmodel` MAXN，RealSense 节点约占单核 80%，系统内存充足。

## 可复现现象

默认 profile：depth `848x480x30`、color `640x480x30`，sync/alignment=true。

- RGB 约 30 Hz；aligned depth 跨机单订阅约 20.9 Hz。
- Go2 本机订阅 aligned depth 同样只有约 10→15.6 Hz，排除跨机 DDS 为主因。
- 15 秒收到 299 个 aligned-depth header；平均间隔 47.68 ms，最大 300.31 ms，
  16 次间隔超过 40 ms，无 timestamp backward/duplicate。
- 节点报告 `USB SCP overflow`、`Hardware Error` 和大量
  `control_transfer ... Resource temporarily unavailable`。
- 内核报告 `uvcvideo: Failed to set UVC probe control: -32`。

低带宽对照：depth/color 均为设备明确支持的 `640x480x15`。

- 5 秒 RGB 74、aligned depth 72、CameraInfo 74。
- message-age jitter 约 4–5 ms；RGB/depth 最大 header 配对差 67.19 ms。
- libusb warning 仍存在，但该次日志未出现 `USB SCP overflow`。
- 因仍超过 50 ms 且系统时钟未同步，该 profile 仅用于诊断，未成为正式配置。

NTP 同步后的 60 秒扩展结果：

- Go2 本机单独订阅 aligned depth：average 14.23 Hz，window 821，约为配置值的
  94.9%。
- 最短到达间隔约 37 ms，最长约 368 ms，仍存在偶发连续缺帧。
- 测试稳态未再报告 `USB SCP overflow`，但启动期 libusb control-transfer
  warning 仍存在。
- 跨机 Python RGB-D preflight 的 offset median 为 0、maximum absolute
  135.16 ms；超过 100 ms 的 pair 必须拒绝。
- 因输出率高于 Ranger 已运行的约 4 Hz RGB 基线，`640x480x15` 可作为候选
  VLM profile，但不能据此认定 USB/版本根因已经修复。

## 希望 Go2/Unitree 同事确认

1. Unitree 当前镜像是否有意使用 `realsense-ros 4.55.1 + librealsense 2.53.1`？
2. D435 firmware `5.12.07.150` 是否是该镜像验证过的版本？若不是，推荐组合及
   firmware/SDK 的备份、升级和回退流程是什么？
3. `5.10.104-tegra` 是否包含当前 D435/native UVC backend 所需的 RealSense
   kernel patches？
4. 此平台上 `usbfs_memory_mb=16`、USB autosuspend=`auto` 是否为预期值？若建议
   修改，请给出 Unitree 验证过的数值、持久化位置和回退方式。
5. `tegra-xudc ... failed to get usbphy-0: -517` 是否为该上位机的已知无害日志，
   还是说明 USB role/PHY 配置异常？
6. 该 D435 的供电、线缆和接口是否为 Unitree 推荐连接方式？是否有已验证的
   30 FPS RGB-D profile？

在上述问题确认前，不执行 firmware 刷写、SDK/wrapper 替换、kernel patch、
autosuspend 或 usbfs 参数持久化修改，也不把 D435 接入 VLM 自动导航。
