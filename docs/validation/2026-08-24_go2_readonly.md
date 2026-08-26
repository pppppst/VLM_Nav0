# Go2 只读验证记录（2026-08-24—25）

本工具未发布 `/api/sport/request`，未修改 Go2 文件或服务，也未启动 FAST-LIO、
SLAM、Nav2 或运动控制。2026-08-25 的相机最初由操作者在另一终端手动启动；完成
一次限时重启诊断后已通过 SIGINT 干净退出，当前相机节点未运行。

## 阶段状态

- 第 1 级：PASS（本机构建）。固定 commit 已导入独立 `go2_ws/src`，两个仓库
  HEAD 已核对，SPARK-FAST-LIO、`unitree_api` 和 `vlm_nav` 已在独立
  `go2_ws/{build,install,log}` 构建；`twist_to_go2_sport_bridge` 可执行文件已链接。
  安装全部 ROS runtime 依赖后的完整回归为 177 passed，bridge state-machine
  原生 gtest 为 11 passed。构建仅有 PCL_ROOT/CMP0074 developer warning。
- 第 2 级：PASS。`enp4s0` 是唯一 `192.168.123.x` 接口；CycloneDDS 环境通过。
  `/utlidar/cloud` 与 `/utlidar/imu` publisher 均为 RELIABLE、KEEP_LAST(1)、
  VOLATILE，兼容 subscriber 已分别连续收到三帧。D435 的 RGB 和 color
  CameraInfo 已实测为 RELIABLE/VOLATILE 并分别收到 3/3 帧。操作者授权启用相机
  sync/alignment 后，aligned depth 也已实测为 RELIABLE、KEEP_LAST(1)、VOLATILE；
  完整五话题 gate 已重跑，每个话题都以兼容 subscriber 连续收到 3/3 帧。
- 第 3 级：PARTIAL PASS / BLOCKED。point-level time 语义和三轮 60 秒静止数值
  对齐已通过；全点重建的最近 IMU 绝对差 P99 为 1.884 ms，无 header backward
  或明显漂移。静止数据不能证明动态物理相位，故 measurement-alignment gate 仍
  fail-closed，详见 `2026-08-25_go2_point_time.md`。
- 第 4 级：PASS（系统 NTP 同步）。IMU header 相对 callback `system_now` 的稳定
  正偏移降级为已知时间戳现象，只作观测，不参与 FAST-LIO `healthy`、
  `SYSTEM_READY` 或 ARM gate。

阶段 4 的只读系统状态进一步确认：本机 `systemd-timesyncd` 为 active、
`NTP=yes`、`NTPSynchronized=yes`；Go2 上 `systemd-timesyncd` 虽为 active 且
`NTP=yes`，但 `NTPSynchronized=no`，并且 RTC 仍显示 1970。Go2 未安装 chrony。
Go2 只有 `192.168.123.0/24` 直连路由，没有默认路由；公网 NTP 名称无法解析，
`timedatectl show-timesync` 显示选中 `1.pool.ntp.org` 但
`ServerAddress=(null)`。因此不能把当前设备时间认定为同步，也没有在运行中对
Go2 执行校时。随后已由操作者启动只绑定 `192.168.123.222:123` 的临时 LAN
chronyd（`-x`，不调整本机时钟），并在 Go2 配置
`/etc/systemd/timesyncd.conf.d/go2-lab.conf` 指向该地址。Go2 后续状态为
`System clock synchronized: yes`、ServerName/Address=`192.168.123.222`；第三个
NTP 样本 jitter 为 1.113 ms，四个 NTP timestamp 一致，RTC 也已校正。该本机
chronyd 从 `/tmp` 运行，不是重启后持久服务。

同步后的 5 秒原始数据复测：LiDAR 77 帧（15.369 Hz），message age median
`+14.376 ms`、minimum `+13.628 ms`；IMU 1254 帧，message age median
`−52.137 ms`、minimum `−52.337 ms`。两路均无 timestamp backward；最近
LiDAR/IMU header 差 median `−1.192 ms`、minimum `−3.256 ms`、maximum
`+3.493 ms`。系统 NTP 已正常；绝对 message age 仅保留为诊断统计，不据此判定
FAST-LIO BLOCKED，也不加入固定 offset。结果在
`/tmp/go2_raw_diagnostics_synced.json`。

第 4 个 NTP 样本 jitter 为 1.087 ms 后再次复测：LiDAR message age median
`+13.464 ms`，IMU median `−52.978 ms`、minimum `−53.210 ms`，结论不变。
随后直接在 Go2 本机运行 `ros2 topic delay`：`/utlidar/imu` 稳定约 `−55 ms`
（std dev 约 0.05–0.09 ms），`/utlidar/cloud` 约 `+11 ms`。因此 IMU 未来时间戳
来自 Unitree publisher/设备时间映射，不是跨机 DDS 或本机 preflight；该现象
本身不再阻塞 FAST-LIO。正式 blocker 是尚未确认 point-level time 语义及实际
LiDAR–IMU measurement alignment。重复结果在
`/tmp/go2_raw_diagnostics_synced_repeat.json`。

## 第二次 5 秒共同窗口统计

- 点云：78 帧，15.424 Hz，无解析错误，无 header timestamp backward。
- PointCloud2：x/y/z finite；`time` 存在且变化；帧内跨度 minimum 45.93 ms、
  median 62.76 ms、maximum 82.21 ms。
- ring：整帧单值现象为 true，仅作诊断，不作 blocker。
- IMU：1244 帧，无 NaN/Inf，无 header timestamp backward。
- 静止 IMU 原始量：加速度范数 9.992–10.673 m/s²（median 10.295）；gyro 范数
  0.00126–0.02558 rad/s（median 0.01069）；最大加速度相邻跳变 1.533 m/s²。
- 共同覆盖窗口内 LiDAR-header 减最近 IMU-header：minimum −1.960 ms、median
  −1.061 ms、maximum 3.391 ms，范围 5.351 ms。
- `local_receive - header.stamp`：LiDAR median −74.47 ms、minimum −91.45 ms；
  IMU median −140.12 ms、minimum −140.28 ms。负值表示消息头时间在本机接收时钟
  的未来；该统计只描述 header-to-wall 现象，不参与正式 timing gate，且不能用
  单次 `date` 结论替代 NTP 状态。

后续 Go2 本机两轮 10 秒 A/B/C 联合测试见 `GO2_TIME_IMU_HANDOFF.md`：最近邻
`imu.header.stamp - lidar.header.stamp` median 分别约 `+1.59 ms` 和 `+1.17 ms`，
两路均无 timestamp backward，暂未发现明显相对时间轴失配。随后已完成三轮
60 秒 point-level 检查；语义与静止数值对齐通过，但动态物理相位仍待验证，详见
`2026-08-25_go2_point_time.md`。

原始 JSON 临时保存在 `/tmp/go2_raw_diagnostics.json`。正式结论必须来自保存的
静止 rosbag、阈值来源记录和重复测量。

## 2026-08-25 D435 只读验证

- `/camera/camera/color/image_raw`：约 30 Hz，5 秒收到 150 帧，图像元数据健康。
- `/camera/camera/color/camera_info`：5 秒收到 150 帧，尺寸、frame_id、内参健康。
- `/camera/camera/depth/image_rect_raw`：约 30 Hz，但这是未对齐 raw depth，不接入
  VLM projection。
- 首次检查时 `/camera/camera/aligned_depth_to_color/image_raw` 不存在，且
  `align_depth.enable=false`、`enable_sync=false`。相机前台进程占用了操作者终端，
  因此随后键入的 param 命令当时没有由 shell 执行。
- 经操作者确认后已执行运行时参数修改；当前 `align_depth.enable=true`、
  `enable_sync=true`，aligned depth topic 已出现。该设置会在相机节点重启后丢失，
  后续应以显式 launch 参数启动。
- aligned depth publisher 实测为 RELIABLE、KEEP_LAST(1)、VOLATILE。第二次 5 秒
  preflight 收到 RGB 149 帧、aligned depth 79 帧、CameraInfo 149 帧；三路元数据、
  时间单调性、RGB/depth 尺寸和 CameraInfo 内参检查通过。
- 第二次统计的 `local_receive - header.stamp` median：RGB −18.751 秒、aligned
  depth −18.745 秒、CameraInfo −18.751 秒，远超 50 ms 门槛。Go2
  `NTPSynchronized=no`，因此相机 gate 仍正确返回失败。
- RGB 与最近 aligned-depth header stamp 的 median 差为 0，但 maximum absolute
  为 166.8 ms；同时 aligned depth 帧数明显低于 RGB。结合终端中的 libusb
  resource-unavailable warning，需继续只读确认帧间隔与丢帧，不能直接放宽同步阈值。
  原始结果分别在 `/tmp/go2_camera_preflight.json` 和
  `/tmp/go2_camera_preflight_aligned.json`。
- 单独跨机订阅 aligned depth 约 20.9 Hz。15 秒 header 统计收到 299 帧，平均
  header delta 47.68 ms、最小 33.33 ms、最大 300.31 ms，16 次间隔超过 40 ms；
  无 timestamp backward 或重复。
- Go2 本机订阅也只有约 10→15.6 Hz，而本机 RGB 稳定约 30 Hz，证明问题位于
  RealSense/Go2 处理链，不是跨机 CycloneDDS 或本机网卡。
- 已验证真实 stream profile：depth `848x480x30`、color `640x480x30`、
  aligned queue size 16。以 `enable_sync:=true`、`align_depth.enable:=true` 从启动
  时限时重启，5 秒仍只有 RGB 147 帧、aligned depth 76 帧；“仅因运行时打开
  filter”假设被否定。结果在 `/tmp/go2_camera_preflight_launch_args.json`。
- 该次启动明确报告 `USB SCP overflow` / `Hardware Error`，并连续出现 libusb
  `Resource temporarily unavailable`；内核还记录
  `uvcvideo: Failed to set UVC probe control: -32`。设备虽工作在 USB 3.2 / 5 Gb/s，
  但 power control 为 `auto`、autosuspend 2 秒，`usbfs_memory_mb=16`。这些只作为
  根因证据，尚未修改内核/USB 参数。
- 版本实测：D435 firmware `05.12.07.150`、RealSense ROS `4.55.1`、librealsense
  build/runtime `2.53.1`。远端 `realsense-ros` checkout 是 tag `4.55.1`，但
  `realsense2_camera/CMakeLists.txt` 有本地修改，把官方 `find_package(realsense2
  2.55.1)` 改成了 `2.53.1`；系统仅有 `/usr/local/lib/librealsense2.so.2.53.1`。
  librealsense v2.53.1 官方 release 推荐 D400 firmware `5.14.0.0` 或以上。升级
  firmware、wrapper 或 SDK 都必须作为独立、可回退的硬件变更评审，当前未执行。
- 低带宽对照使用设备枚举明确支持的 depth/color `640x480x15`，仍从启动时启用
  sync/alignment。5 秒收到 RGB 74、aligned depth 72、CameraInfo 74 帧；三路
  message-age jitter 降至约 4–5 ms，RGB/depth maximum absolute header offset
  降至 67.19 ms。测试仍出现 libusb control-transfer warning，但本次日志未再出现
  `USB SCP overflow`。这证明降带宽显著改善链路，却仍未达到 50 ms gate，不能写成
  正式导航默认值。结果在 `/tmp/go2_camera_preflight_640x480x15.json`；测试结束后
  相机已干净退出。
- NTP 同步后完成 60 秒低带宽复测。跨机三话题 Python preflight 收到 RGB 524、
  aligned depth 510、CameraInfo 526，三路时间单调、尺寸和内参健康；RGB/depth
  offset median 为 0，但 maximum absolute 为 135.16 ms。该进程同时反序列化
  两路完整图像，不能用其计数单独反推相机原生 publisher Hz。
- 随后在 Go2 本机只订阅 aligned-depth 单话题 60 秒，最终 average rate
  14.23 Hz、window 821，达到 15 FPS profile 的约 94.9%；最短间隔约 37 ms、
  最长约 368 ms。稳态窗口没有再次出现 `USB SCP overflow`，节点按 timeout
  SIGINT 干净退出；启动阶段仍有 libusb control-transfer warning。
- 因有效输出频率高于 Ranger 已用约 4 Hz RGB 基线，`640x480x15` 可作为 Go2
  VLM 候选 profile；仍需丢弃超过 runtime 100 ms slop 的 pair，不能因降低配置
  频率而忽略偶发长间隔。60 秒跨机报告位于
  `/tmp/go2_camera_preflight_640x480x15_60s.json`。

## 已解决的构建依赖

操作者已执行：

```bash
sudo apt-get install -y ros-humble-rosidl-generator-dds-idl \
  ros-humble-diagnostic-updater ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan ros-humble-realsense2-camera
```

随后 `scripts/setup_go2_workspace.sh` 已成功完成独立构建；这没有复制 Ranger 的
build/install/log，也没有启动任何运动节点。
