# Go2 动态 costmap 与静止现场采集（基准 2575a54）

本轮已修复**显式选择的协同配置**的最终覆盖；没有改变旧 AMCL 默认栈。
原生 Nav2 动态层已在 Domain31/localhost 合成输入下验证，不能当作真机场景通过。
实际现场采集被 enp4s0 的 NO-CARRIER/无 Go2 子网地址阻止；未初始化 Domain0
参与者，未启动 bridge、运动目标或 Ranger。Ranger 标定仍为空、完整 live 关闭。

## 实际加载顺序与独立配置

`scripts/manualnav2.sh` → `launch/go2_system.launch.py` →
`launch/go2_navigation.launch.py` → `vlm_nav/go2_nav2_config.py`：

1. 已安装 `nav2_bringup/params/nav2_params.yaml`（本机 Nav2 1.1.20）。
2. `config/nav2_go2.yaml`：全局 obstacle、局部 voxel，输入 `/vlm_nav/obstacle_cloud`。
3. 仅 AMCL：`config/nav2_go2_static_map.yaml`，plugins 数组整体替换为 static/inflation。
4. **仅显式提供** `costmap_overrides_file` 时：再合并
   `cooperative_ws/src/multi_agent_nav/config/nav2_go2_cooperative_costmap.yaml`。
5. 现有 wall-time 强制值保持；写临时最终参数文件，传给各 Nav2 server。

最终覆盖只允许 costmap 节点，拒绝 AMCL/map_server 等键；默认参数为空，因此旧
脚本继续得到旧 static-map 配置。go2_system 已透传可选参数，旧脚本/AMCL YAML未改。

| 项目 | 静态配置已核实 | 真机运行时已验证 |
|---|---|---|
| 全局 plugins | static_layer, obstacle_layer, inflation_layer | 否：本机现场网络预检失败 |
| 局部 plugins | static_layer, voxel_layer, inflation_layer | 否 |
| 动态 source | go2_cloud / PointCloud2 / `/vlm_nav/obstacle_cloud`；marking/clearing=true | 否：需实际订阅、输入与变化证据 |
| raw payload | always_send_full_costmap=true | 合成 Nav2通过；真机未验证 |
| AMCL | `/scan_amcl`、OmniMotionModel、alpha与定位输入配置保持 | 未取得本轮参数快照 |
| footprint/frame/voxel/range | 继承原值，不缩 footprint、不改高度/range/定位参数 | 原单机历史验收不能替代本轮动态障碍验收 |
| 地图 | 用户指定的 YAML/image已核实；Go2 ID/version/digest已填写 | 当前活动文件尚未通过本轮ROS查询确认 |
| 本机场景 | lab_ref 数值仍等于 Go2 map；字段与有限数值检查已有 | 语义点位尚未提供或采集 |
| 路径/停止能力 | 收集命令已准备；来源/余量仍待填 | 未发送动作，未验证 |

独立静止入口：`go2_costmap_acceptance.launch.py`，仅运行隔离的两张原生 costmap
及 lifecycle manager；不运行 AMCL、地图服务器、传感器、action server、速度发布器
或 bridge。使用与完整 Nav2 相同的四层参数合并，只把 costmap 名空间隔离至
`/go2_costmap_acceptance`，TF与地图仍读取本机已有输入。

Humble的原生 Costmap2DROS 没有注册可加载 component，因此用一个小 C++ host持有
两个 NodeOptions 构造的原生节点。[上游1.1.20实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp)
也不建立 costmap lifecycle bond，独立入口的 bond_timeout=0；这是只读地图生产进程，
消费者仍必须以 TF、stamp、接收龄检查失效。完整 Nav2 的 lifecycle/bond参数没有改。

## 现场网络恢复后的静止验证命令（本轮未执行）

Go2保持静止，使用**已经运行且禁止运动的传感器/AMCL底层栈**。不使用
manualnav2.sh/VLMNav-go.sh启动采集，因为它们包含 ARM/硬件流程。
没有现有定位与传感器输入时，不以假TF、假地图或合成cloud做现场证明。

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
export GO2_NET_IFACE=enp4s0
ip -br link show dev enp4s0
ip -br -4 address show dev enp4s0
source scripts/go2_network_env.sh || { echo '网络预检失败，停止本次采集'; exit 1; }
# 要求本机Go2子网地址；失败必须停止，不能转wlo1。
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /tmp/go2-coop/install/setup.bash
export ROS_LOG_DIR=/tmp/go2-field/evidence/ros
mkdir -p /tmp/go2-field/evidence
ros2 param dump /amcl > /tmp/go2-field/evidence/amcl-before.yaml
ros2 param get /map_server yaml_filename
ros2 launch multi_agent_nav go2_costmap_acceptance.launch.py
```

第二终端重复 source/env，查询最终加载值，而不是只看源 YAML：

```bash
for side in local global; do
  node="/go2_costmap_acceptance/${side}_costmap/${side}_costmap"
  ros2 lifecycle get "$node"                # 必须active
  ros2 param dump "$node" > "/tmp/go2-field/evidence/${side}-loaded.yaml"
  ros2 param get "$node" plugins
  ros2 param get "$node" always_send_full_costmap
done
ros2 param get /go2_costmap_acceptance/local_costmap/local_costmap voxel_layer.enabled
ros2 param get /go2_costmap_acceptance/local_costmap/local_costmap voxel_layer.go2_cloud.marking
ros2 param get /go2_costmap_acceptance/local_costmap/local_costmap voxel_layer.go2_cloud.clearing
ros2 param get /go2_costmap_acceptance/global_costmap/global_costmap obstacle_layer.enabled
ros2 param get /go2_costmap_acceptance/global_costmap/global_costmap obstacle_layer.go2_cloud.marking
ros2 param get /go2_costmap_acceptance/global_costmap/global_costmap obstacle_layer.go2_cloud.clearing
ros2 topic info /vlm_nav/obstacle_cloud -v --no-daemon
ros2 param dump /amcl > /tmp/go2-field/evidence/amcl-after.yaml
diff -u /tmp/go2-field/evidence/amcl-before.yaml /tmp/go2-field/evidence/amcl-after.yaml
```

`plugins/enabled/marking/clearing` 参数值证明配置加载；还不证明真实障碍检测。
ROI选真实视野内、静态地图原本空闲的位置，记下map坐标和物体尺寸/高度；Go2整机
不动，依次采空场、放置可控障碍物、移除后恢复背景回波。保持ROI在有效射线和测距
范围内；**空cloud不等于有效清除射线**，移除后必须有真实背景回波穿过原障碍位置。

配置已准备在 `/tmp/go2-field/config`，只填了用户指定的Go2地图身份。先复制独立只读配置，保留
已有填写值；只改验收 costmap 话题，不改任何 verified 标记：

```bash
mkdir -p /tmp/go2-field/config-costmap-acceptance
cp -n /tmp/go2-field/config/*.yaml /tmp/go2-field/config-costmap-acceptance/
python3 - <<'PY'
from pathlib import Path
import yaml
p=Path('/tmp/go2-field/config-costmap-acceptance/robot_go2.yaml')
d=yaml.safe_load(p.read_text())
d['costmap_topic']='/go2_costmap_acceptance/local_costmap/costmap_raw'
p.write_text(yaml.safe_dump(d,sort_keys=False,allow_unicode=True))
PY
export GO2_FIELD_CONFIG=/tmp/go2-field/config-costmap-acceptance
bash scripts/collect_go2_cooperative_static.sh --config-dir "$GO2_FIELD_CONFIG" \
  --output-dir /tmp/go2-field/evidence/baseline --phase baseline --duration 15
# 人工放入障碍物，Go2继续静止
bash scripts/collect_go2_cooperative_static.sh --config-dir "$GO2_FIELD_CONFIG" \
  --output-dir /tmp/go2-field/evidence/mark --phase mark --duration 15
# 人工移除障碍物，恢复真实背景射线
bash scripts/collect_go2_cooperative_static.sh --config-dir "$GO2_FIELD_CONFIG" \
  --output-dir /tmp/go2-field/evidence/clear --phase clear --duration 15
ros2 bag info /tmp/go2-field/evidence/baseline/baseline
ros2 bag info /tmp/go2-field/evidence/mark/mark
ros2 bag info /tmp/go2-field/evidence/clear/clear
```

采集脚本只查询参数/图和被动adapter，可选择录bag；不会设置参数、取消/发送动作、
ARM或启动底层。status.tsv记录每条查询返回码；COLLECTION_ONLY不是验收通过。
输出目录必须不存在，重复采集请用新目录，避免旧报告和新失败混在一起。
bag的限时SIGINT返回124是正常截止信号，要另查bag是否可读、消息是否齐全。
验证整个阶段odometry时间与接收龄、速度和TF，而不只看开始瞬间静止。

可直接在同一**实测局部map位姿**观察 footprint 占用，无规划、无运动目标：

```bash
: "${GO2_ROI_X:?实测map x}" "${GO2_ROI_Y:?实测map y}" "${GO2_ROI_YAW:?实测yaw}"
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --action inspect --goal "$GO2_ROI_X" "$GO2_ROI_Y" "$GO2_ROI_YAW" --timeout 15
```

检查costmap_valid、时间/TF、costmap_probe.footprint_free；null表示没有有效检查条件，
不能当空闲。有效ROI的原始格应体现空闲→253/254阻挡→恢复，与cloud及两个costmap
时间对齐。footprint_free还可能因为地图边界或未知格失败，需看raw数据排除这些原因。
实际真机证据齐全后另行审查 costmap_source_verified；peer_obstacle_verified仍要
真实机身/方向覆盖验收，不能用普通障碍物试验替代。

## Go2地图身份与本机场景填写

用户指定活动地图为
`/home/isee-pst/unitree_ros2/VLM_Nav/maps/lab_rear120_20261001_IwQjMX/lab.yaml`。
它也与2026-10-06的历史map_server快照一致，但历史记录不是当前ROS证明。
本轮核实YAML引用 `lab.pgm`、图像可解码为222×312 uint8、分辨率0.05m、origin
`[-6.52,-5.34,0]`。这些是文件已有元数据，未新增演示坐标。
已填写源配置及 `/tmp/go2-field/config`：

- `alignment.yaml:robots.go2.local_map_id=lab_rear120_20261001_IwQjMX`。
- `robot_go2.yaml:map_digest=c9f354a26471032e7ff10b5898db627b65fce7526c28124891833b6a3a164c02`。
- `alignment.yaml:robots.go2.map_version=sha256:<上述完整digest>`，地图内容变化须更新版本。
- `robot_go2.yaml:map_path` 为用户给定绝对路径；lab_ref/Go2 transform仍为 `[0,0,0]`。

文件核查证据：[go2-map-identity.json](dynamic_costmap_evidence/go2-map-identity.json)，
runtime_active_map_verified=false。Ranger、alignment_id/report/validated均未填通过。

现场恢复后立即采新的只读报告，核对已经填写的活动路径、文件摘要、定位与持续静止。
不匹配时停止检查，不自动把另一张地图改成指定地图：

```bash
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" \
  --timeout 15 > /tmp/go2-field/evidence/current-readonly.json
timeout 5 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --field info --qos-durability transient_local
# 与上述文件尺寸/resolution/origin比较，不仅检查文件名。
export PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:${PYTHONPATH:-}"
/home/isee-pst/venv/co-nav-real/bin/python - <<'PY'
import json,os,yaml
from pathlib import Path
from multi_agent_nav.config import map_file_info
r=json.load(open('/tmp/go2-field/evidence/current-readonly.json'))
assert r['health']['localization_valid'] and r['health']['stopped'] and r['active_map_yaml']
info=map_file_info(r['active_map_yaml']);root=Path(os.environ['GO2_FIELD_CONFIG'])
p=root/'robot_go2.yaml';d=yaml.safe_load(p.read_text())
assert Path(d['map_path']).resolve()==Path(info['yaml_path'])
assert d['map_digest']==info['map_digest'] and r['active_map_matches']
p=root/'alignment.yaml';a=yaml.safe_load(p.read_text())
assert a['robots']['go2']['local_map_id']==Path(info['yaml_path']).parent.name
assert a['robots']['go2']['map_version']=='sha256:'+info['map_digest']
print(info)  # 只核对；不改地图、Ranger、对齐、场景或验证开关。
PY
ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$GO2_FIELD_CONFIG" --require-ready
```

仅在同一已确认活动地图下采点。`current-readonly.json:health.pose` 是当前Go2
map/lab_ref位姿 `[x,y,yaw弧度]`；不要把它自动命名为A观察点或退出点。
由现场语义确认后填写 `scene.yaml:viewpoints.go2[{id,pose}]`、`wait_points.go2`、
`exit_points.go2`。门/C多边形应取实测区域边界（≥3点），并检查整机停止包络及出口
清空；RViz Publish Point仅用于提供map下的人工边界点，不发送导航目标。
未知A/B/C描述、视点朝向、区域边界与Ranger点位保持空，scene.enabled=false。

RViz具体采点方法（现场网络恢复、已有定位稳定后执行）：

1. Fixed Frame设为`map`，仅添加Map(`/map`)、TF和PointStamped(`/clicked_point`)
   显示；选择工具栏 **Publish Point**。本轮不使用2D Nav Goal；也不用2D Pose
   Estimate采点，因为后者会改变AMCL初始位姿。
2. 另一终端先运行下面的echo或只读录包，再点击需要的地面点。确认每条消息的
   header.frame_id是map，记录x/y与标签；z不是场景2D字段。按点击顺序记录区域
   顶点，至少3点、非退化，不能把物体中心替代整个门/C保护区域。
3. Publish Point不提供朝向。对于一个view/wait/exit pose，第一点取机器人中心，
   第二点取期望朝向上的另一实测点，计算 `yaw=atan2(y2-y1,x2-x1)`（弧度）；
   两点不能重合。或在机器人已处于所需朝向且静止时，直接读取map→base_link的
   yaw与只读报告pose，确认定位新鲜后记录。此方法不会发送运动目标。

```bash
ros2 topic info /clicked_point -v --no-daemon
# 先确认没有把clicked_point转成运动的未知订阅者；仓库代码无此运动订阅。
timeout 60 ros2 topic echo /clicked_point geometry_msgs/msg/PointStamped
timeout 5 ros2 run tf2_ros tf2_echo map base_link
# 或只读保存全部点击、TF与odom，供逐点标注和时间核对：
timeout --signal=INT --kill-after=5 60 ros2 bag record \
  --output /tmp/go2-field/evidence/scene-points /clicked_point /tf /tf_static /odometry /map
```

现场配置中的 `scene.yaml:viewpoints.go2` 每项是 `{id: <实际标签>, pose: [x,y,yaw]}`；
wait_points.go2/exit_points.go2同样是三元pose。没有采得的位置不写入列表；本轮仍为空。
标注完成后用 `multi_agent_go2_check --offline --config-dir "$GO2_FIELD_CONFIG" --require-ready`
检查剩余配置缺项；完整门槛因Ranger/场景/路径/停止等尚缺而继续退出1。

## 执行路径与停止能力：准备命令，尚未动作验收

下面只发现/录制接口，不请求动作。重复同一Go2/enp4s0环境；不要启bridge。

```bash
ros2 topic list -t --no-daemon | rg 'nav_msgs/msg/Path'
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 action info /navigate_to_pose
ros2 action info /follow_path
ros2 action info /spin
: "${GO2_PATH_CANDIDATE:?填实际发现的路径话题，不能猜测}"
ros2 topic info "$GO2_PATH_CANDIDATE" -v --no-daemon
timeout 5 ros2 topic echo "$GO2_PATH_CANDIDATE" nav_msgs/msg/Path --once --field header
# 为随后单机运动验收准备记录；本轮不发送目标、不取消目标、不ARM。
timeout --signal=INT --kill-after=5 45 ros2 bag record \
  --output /tmp/go2-field/evidence/path-stop-capture \
  "$GO2_PATH_CANDIDATE" /odometry /tf /tf_static /goal_pose \
  /navigate_to_pose/_action/status /follow_path/_action/status /spin/_action/status \
  /cmd_vel_nav /cmd_vel_bridge /api/sport/response \
  /vlm_nav/nav_ready /vlm_nav/control_armed
```

静止期间无path/status消息不构成接口通过或失败。后续需绑定真实native UUID、
FollowPath正在执行及重规划过程；ComputePath预检、planner输出和距目标反馈不能
单独证明执行路径。保留path_topic=null/path_source_verified=false，来源不明就不填写。
停止能力后续记录取消请求/接受/终态时刻及新鲜odom持续低速，实测延迟、停止距离、
方向和速度包络后才填stop_margin。静止录包不能验证制动/取消能力。

本轮证据与全量结果见本页对应 evidence；所有真机标记仍为false。完整双机协议、
Ranger对齐、桥接、互为障碍和任务顺序验收继续保留原门槛。

## 本轮实际执行与结果

重新构建安装产物，再执行 `bash scripts/test_cooperative_no_hardware.sh`，与前两轮
证据目录分开保存至 `dynamic_costmap_evidence`：

| 检查 | 结果 | 实际证据 |
|---|---|---|
| 构建 | 3包成功 | [build.log](dynamic_costmap_evidence/build.log) |
| 原单机Python基线 | 273 passed，8.77s | [baseline-final.log](dynamic_costmap_evidence/baseline-final.log) |
| 原C++基线 | 3 CTest / 21 gtest，全过 | [baseline-cpp.log](dynamic_costmap_evidence/baseline-cpp.log) |
| 协同/接入测试 | 84 passed，16.09s（原78＋本轮6） | [cooperative.log](dynamic_costmap_evidence/cooperative.log) |
| 合成原生costmap | plugins/full payload/marking/clearing/无速度发布均过；仅Domain31 | [config-native.log](dynamic_costmap_evidence/config-native.log) |
| 配置覆盖回归 | 先复现最终仍是static/inflation，再修复 | [config-red.log](dynamic_costmap_evidence/config-red.log) |
| 原生桥接解析 | 1 passed，仅解析不启bridge | [test-result.log](dynamic_costmap_evidence/test-result.log) |
| colcon汇总 | 108 entries，0失败/错误/跳过；84＋1 parser＋2 wrapper＋21 gtest | [colcon-test.log](dynamic_costmap_evidence/colcon-test.log) |
| 已安装入口/离线地图 | show-args/help成功；Go2文件门槛为空，完整live20个缺项且退出1 | [entry-exit-status.log](dynamic_costmap_evidence/entry-exit-status.log)、[config-gates.json](dynamic_costmap_evidence/config-gates.json) |
| 现场只读尝试 | enp4s0预检退出1；未初始化Domain0 ROS | [field-interface.log](dynamic_costmap_evidence/field-interface.log)、[field-network.log](dynamic_costmap_evidence/field-network.log) |
| 独立审查 | 未发现新增Critical/Important | [review.log](dynamic_costmap_evidence/review.log) |

原AMCL YAML、Nav2基本/static-map配置、manual/VLM脚本及前两轮证据与2575a54相比
没有修改。仅新增显式选择的costmap最终覆盖及入口，旧默认合并结果保持。
所有自动ROS验证只在localhost Domain31；真机场景测量、动态层marking/clearing、
当前map_server路径/内容核对、执行路径关联与物理停止验收均未完成，不能据上述
合成测试填现场通过标记。
