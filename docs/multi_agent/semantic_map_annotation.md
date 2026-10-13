# Go2 固定地图家具语义标注

仅离线采集家具地面投影角点。地图身份是 Go2 本机的固定地图和文件摘要，坐标为
`map` frame、米；Domain31 是通信环境，不是地图身份，不代表 Ranger 坐标对齐。
不加载定位、底盘、导航、探索、桥接或实时传感器；不修改原地图或导航配置。

地图 YAML：`maps/lab_rear120_20261001_IwQjMX/lab.yaml`，关联 `lab.pgm`。
实查尺寸 222×312，resolution=0.05，origin=[-6.52,-5.34,0]，mode=trinary，
negate=0，occupied_thresh=0.65，free_thresh=0.25。

## 离线启动顺序

以下是独立复用流程。若本次 A/B/C 已由助手打开，不再重复启动。
三个终端均在仓库目录执行公共环境：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
unset PYTHONHOME PYTHONPATH CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$PWD/build/go2-field/evidence/annotation-ros-logs"
mkdir -p "$ROS_LOG_DIR"
```

不加载 `go2_network_env.sh`。如继承了 `FASTRTPS_DEFAULT_PROFILES_FILE`、
`FASTDDS_DEFAULT_PROFILES_FILE`、`ROS_DISCOVERY_SERVER` 等发现配置，先核对来源，
在本次离线终端取消影响本地发现的覆盖。不要改动用户其他运行终端。

终端 A 加载地图，保持运行，最后 Ctrl+C：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __ns:=/go2_point_annotation -r __node:=map_server \
  -p use_sim_time:=false \
  -p yaml_filename:="$PWD/maps/lab_rear120_20261001_IwQjMX/lab.yaml"
```

终端 B 配置生命周期。必须看到 `Transitioning successful` 才执行下一条；
失败时检查 A 的加载日志、三个终端环境及发现，不进入选点：

```bash
ros2 lifecycle set /go2_point_annotation/map_server configure --no-daemon --spin-time 4
```

前一步成功后执行：

```bash
ros2 lifecycle set /go2_point_annotation/map_server activate --no-daemon --spin-time 4
```

激活成功后检查地图实际消息。Humble 本机帮助已确认支持 `--no-daemon`；
其他环境应先查 `ros2 topic echo --help`。成功判据是收到 width=222、height=312、
resolution≈0.05、origin 平移=(-6.52,-5.34)，超时先排障。
CLI 输出可能带末尾 `---`；程序解析应使用 `yaml.safe_load_all` 并忽略空文档。

```bash
timeout 10 ros2 topic echo /go2_point_annotation/map \
  nav_msgs/msg/OccupancyGrid --once --field info \
  --qos-reliability reliable --qos-durability transient_local --no-daemon
```

终端 C 启动专用 RViz，保持运行：

```bash
rviz2 -d "$PWD/config/go2_semantic_annotation.rviz"
```

该配置仅显示指定固定地图，Fixed Frame=map、Transient Local、俯视视角；
Publish Point 发布 `/clicked_point`。确认地图出现且 Status=OK，再进入录点。
不修改或使用含实时显示的原探索配置。

终端 B 启动录点，保持运行：

```bash
python3 scripts/record_semantic_points.py \
  --topic /clicked_point --frame-id map \
  --output-dir "$PWD/maps/lab_rear120_20261001_IwQjMX"
```

看到“录点已就绪”和实际记录路径后，选择 RViz 工具 Publish Point，按顺/逆时针
依次点击每件家具四个角，每点击一次先确认 B 收到编号。不是中心＋方向取点。
无效 frame/非有限 x/y/z 仍记录、编号并标记无效；不进行 TF 转换。
JSONL 保存 session、index、原始 sec/nanosec、frame_id、x/y/z、有效性及原因，
每条追加后 flush；非有限值保存为 `nan`/`inf` 字符串，使文件保持标准 JSON。
每次启动独立文件、独立编号，重选时同时提供 session 和点号；不要删除误点记录。

结束后在 B Ctrl+C，确认保存总点数和路径，再提供点号→ID/类别/可选名称或房间的分组。
收到分组前不生成家具 YAML 或推测对应关系。

## 根据明确分组生成语义层

`scripts/render_semantic_map.py` 的 `build_semantic(map_yaml, record_path, groups)`
供助手或 Python 调用。每个 group 必须包含 `id`、`category`、四个 `point_indices`，
可附 `name`、`room_id`。只使用明确提供的分组，保留原编号和顺序。

函数核对编号、有效 frame/坐标、唯一 ID、重复角点、退化、自交和边界；错误报告家具
ID 和原始点号，不移动角点或重排。越界须人工核对，不裁剪。原始未使用点保留。
生成的数据包括 schema_version=1、source_robot=go2、frame_id=map、units=meters，
`source_map` 的 YAML/图片路径、两份 SHA256、resolution/origin/尺寸/加载参数，
`point_records` 的 session/原始文件/SHA256 和 `furniture` 的 polygon/session/point_indices。
polygon 是本机 map 的米制 x/y。多个 session 可分别构建后合并 point_records/furniture，
保持共同 source_map 和全局唯一 ID；渲染器会重新核对每个 session。

保存 `semantic_map.yaml` 前先检查旧文件；若存在，调用 `backup_existing(path)` 保留
独立 `.bak`，再写入新版本。更正对应关系时仍保留角点和记录，只改明确要求的字段或分组。

YAML 完成后在仓库目录、已加载上述环境的终端 B 执行：

```bash
python3 scripts/render_semantic_map.py \
  --semantic-yaml "$PWD/maps/lab_rear120_20261001_IwQjMX/semantic_map.yaml" \
  --output "$PWD/maps/lab_rear120_20261001_IwQjMX/annotated_map.png"
```

成功判据：报告实际路径/家具数量；源摘要、地图关联、polygon 与原始编号一致。
渲染以原 occupancy 图作底图，用 6 倍最近邻放大保留像素；对 origin 平移/yaw
做逆变换并翻转图像纵轴，叠加轮廓、轻透明填充与 ID。ID 重叠时移到空白处并引线；
无法清晰放置则明确失败。已有 PNG 先备份，再原子替换，禁止覆盖源文件。
展示 PNG 供用户确认对应关系；未获用户核对不宣称家具标注验收通过。

结束顺序：B Ctrl+C 关闭录点并确认保存 → 关闭 C 的 RViz → A Ctrl+C 关闭离线 map_server。
助手打开的 A/B xterm 在程序退出后保留输出，可以再关闭窗口。

## 工具验证

只运行本功能针对性检查，不需要构建 ROS 包或启动真机：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest test/test_semantic_map.py -q
git diff --check
```

采点环境中的真实消息自检使用独立 `/semantic_annotation_selfcheck` 话题和 evidence
目录，不向 `/clicked_point` 发送合成家具点，也不把自检文件当真实家具交付。
