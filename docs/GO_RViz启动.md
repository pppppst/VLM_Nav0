# Go2 从开机到 RViz 代价地图

本流程只启动 XT16、Go2 body IMU、FAST-LIO、SLAM Toolbox、Nav2 和
RViz。RViz 直接订阅 Nav2 原生代价地图，使用黑灰白 `map` 色板，
不需要任何颜色转换节点。全程保持 bridge dry-run，不 ARM、不发送
Nav2 goal、不启动 VLM。

## 1. 开机和网络准备

1. 启动 Go2 和禾赛 XT16。
2. 连接网线。
3. 确认电脑存在 `192.168.123.x` 地址：

```bash
ip -brief address
```

`go2_network_env.sh` 会自动选择对应网卡。正常时后续终端会输出类似：

```text
Go2 DDS interface: enp4s0
```

## 2. 每个新终端的环境初始化

正式系统和 RViz 使用独立终端。每个新终端先执行：

```bash
deactivate 2>/dev/null || true
set +u
unset PYTHONHOME PYTHONPATH
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/common.sh
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh
export PYTHONPATH="/home/isee-pst/unitree_ros2/VLM_Nav:${PYTHONPATH:-}"
```

不要激活 `/home/isee-pst/venv/co-nav-real`；该虚拟环境可能使 Python 找不到
ROS 2 `rclpy`。

## 3. 一键生成 30 秒正式 preflight 报告

保持 Go2 静止，在一个完成第 2 节初始化的终端执行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/run_go2_preflight.sh
```

脚本会自动启动临时 XT16 driver 和 body-IMU adapter，等待两个传感器话题，
采集 30 秒正式报告，最后停止两个临时进程。成功时必须看到
`healthy: True` 和 `PASS`。如果失败，停止本流程并查看报告：

```bash
/usr/bin/python3 -m json.tool /tmp/go2_costmap_preflight.json
```

脚本拒绝在 `/lidar_points` 或 `/body_imu` 已有 publisher 时运行，以免产生
重复 driver。临时日志路径会在脚本结束时打印。

## 4. 主终端：启动正式 Nav2 链

在 preflight 报告生成后 600 秒内，在仍保留第 2 节环境的终端执行：

```bash
ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:=/tmp/go2_costmap_preflight.json \
  target_stage:=nav2 \
  bridge_dry_run:=true
```

保持该终端运行。正式 launch 会依次启动：

```text
Hesai + body IMU → FAST-LIO → obstacle cloud → LaserScan
                 → SLAM Toolbox → Nav2
```

如果出现 `formal preflight is stale`，返回第 3 节重新采集；不要放宽
600 秒 freshness gate。

## 5. 检查终端：确认 Nav2 和代价地图

等待 staged launch 完成，然后在另一个完成第 2 节初始化的终端执行：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic info /local_costmap/costmap
ros2 topic info /global_costmap/costmap
```

必须看到：

```text
active [3]
Publisher count: 1
```

如果 costmap 的 `Publisher count` 为 `0`，不要启动额外转换节点；检查
主终端的 staged launch 失败信息。

## 6. RViz 终端：启动 RViz

在另一个完成第 2 节初始化的图形终端执行：

```bash
/home/isee-pst/unitree_ros2/VLM_Nav/scripts/start_rviz.sh \
  /home/isee-pst/unitree_ros2/VLM_Nav/config/go2_costmaps.rviz
```

RViz 配置：

- Fixed Frame：`map`
- `SLAM Map`：默认开启，半透明背景
- `Local Costmap`：默认开启
- `Global Costmap`：默认关闭，需要时在 Displays 中勾选
- 代价地图直接使用 RViz `Map` 显示和黑灰白 `map` 色板
- `Scan`、`Robot footprint` 和 `Robot +X direction` 均使用灰白色系

RViz 输出的 `Stereo is NOT SUPPORTED` 只表示不支持立体显示，不是错误，
不影响地图和代价地图。

## 7. 停止顺序

1. 关闭 RViz，或在 RViz 终端按 `Ctrl+C`。
2. 在主终端按 `Ctrl+C` 停止正式 `go2_system.launch.py`。

地图观察全程保持 `bridge_dry_run:=true` 和 DISARMED。
