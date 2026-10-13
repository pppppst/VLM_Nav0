# Go2 强度阈值 A/B：实验实现与现场验收

## 已实现与结论边界

用户指定本轮允许全输出阈值对照，不拆支架，不改变雷达/FAST-LIO输入。
现有 `obstacle_cloud_filter` 新增启动参数：

- `intensity_filter_enabled=false`：默认关闭，A组。
- `intensity_filter_enabled=true`、`intensity_threshold=3.0`：B组。
- 位置固定在高度过滤、自裁剪和体素化**之后**。仅保留强度>阈值的完整点记录，其他字段/时间戳保持；阈值包含等号。
- 参数只读，切组须重启节点；不允许 `ros2 param set` 看似成功却无实际效果。
- 阈值限有限[0,255]；缺少float32 intensity或出现非有限体素强度时拒绝该帧并报错，不伪装为空场景。
- 此版本是全输出阈值，不是此前设想的空间ROI过滤。禁止把它作为已验证生产默认值。

`go2_system.launch.py` 提供同名launch参数，默认仍false。未更改现有CoVLM环境默认overlay、运行节点、雷达原生Interstitial开关或任何导航参数。

链路已核对：`/cloud_registered_base → obstacle_cloud_filter → /vlm_nav/obstacle_cloud → go2_cloud_to_scan → /scan`。因此凡使用 `/scan` 的AMCL都会受影响；launch默认AMCL配置为 `/scan_amcl`，实际命令可覆盖。现场须查询真实 `scan_topic` 与发布者，不能推断。实施时实时查询AMCL返回Node not found，当前未完成现场门槛。

## 已有数据离线对照

使用新脚本 `scripts/compare_obstacle_intensity.py`，不初始化ROS、不发布消息，读取此前保存的**体素后CDR**，执行与C++末段相同的 `intensity > 3` 判定；没有对保存点云重复体素化。

|采样|A全部点|B剩余点|疑似ROI A→B|
|---|---:|---:|---:|
|20:51|91635|63566|144→0|
|21:25|87903|59100|200→0|

20:51地图参考组3517点中1204点（34.23%）会被删除。这里≤3，与先前报告的<3不是同一判据。减少点数不等于障碍消失，但必须检验细腿/黑色临时障碍是否仍逐帧可见。未重建costmap，不宣称虚假致命格减少、清除正常或定位通过。

离线复算：仓库根目录，`source /opt/ros/humble/setup.bash`，执行：

```bash
python3 scripts/compare_obstacle_intensity.py \
  build/go2-field/evidence/cloud-clearance.20261012_212555 \
  --threshold 3 --output /tmp/intensity-ab-recheck.json
```

输出文件必须不存在（防止覆盖）。正常输出四项总点数并退出0，无需保持运行。已有结果在两次采样目录的 `ab_le3.json`。

## 现场顺序（未执行）

先保持静止，由现场指定低反射物体及摆放位置。使用当前误报位置、普通家具位置，以及前方未在地图中的低反射物体“放入—保持—移走”。记录物体材质、尺寸、实测距离和可见范围，不只依赖/map。每一场景先A后B，姿态、物体位置、观察时长及既有旋转流程完全相同；不要通过再定位或修改其他参数补救其中一组。旋转不与人工放入物体同时进行。

1. 终端A在仓库根目录 source `CoVLM_env.bash`，再**最后** source `build/go2-intensity-ab-20261012/install/local_setup.bash`。复用既有底层启动命令及其全部标定/地图参数；A追加 `intensity_filter_enabled:=false intensity_threshold:=3.0`，B只将false改true。必须先结束旧实例并确认退出，不能启动两个同名过滤发布者。底层保持运行，Ctrl+C结束。此处不提供另一个自动旋转/运动入口。
2. 终端B同环境，只读执行 `ros2 param get /obstacle_cloud_filter intensity_filter_enabled`、`ros2 param get /obstacle_cloud_filter intensity_threshold`、`ros2 param get /amcl scan_topic`。核对 `/vlm_nav/obstacle_cloud` 只有一个发布者、costmap有效更新、完整TF存在、AMCL已定位。任何缺项先排障，不能进入旋转。
3. 终端C同环境，每组建立时间命名证据目录，保存完整启动命令和参数dump，限时录包。建议固定每段30秒，保持/移除时刻由现场标记；启动成功后再执行该段操作。仅记录 `/vlm_nav/obstacle_cloud /scan /tf /tf_static /odometry /amcl_pose /map /local_costmap/costmap_raw /global_costmap/costmap_raw /diagnostics`；AMCL若实际订阅其他scan则一并记录实际话题。不要增加原始雷达/完整FAST-LIO云订阅。

```bash
AB_RUN="$PWD/build/go2-field/intensity-ab.$(date +%Y%m%d_%H%M%S)"
mkdir -p "$AB_RUN"
# 先确认上面的门槛；每个目录只对应一个组和一个场景。
ros2 param dump /obstacle_cloud_filter > "$AB_RUN/filter.yaml"
ros2 param dump /amcl > "$AB_RUN/amcl.yaml"
timeout --signal=INT --kill-after=5s 30s ros2 bag record -o "$AB_RUN/bag" \
  /vlm_nav/obstacle_cloud /scan /tf /tf_static /odometry /amcl_pose /map \
  /local_costmap/costmap_raw /global_costmap/costmap_raw /diagnostics
```

该命令录制而不控制运动。预期bag正常关闭并生成metadata；timeout退出124本身不是数据通过，须检查各必需话题计数、时间覆盖和实际内容。每个姿态/物体阶段都应有独立记录，不混成一个未知场景。

4. 使用相同坐标区域与时间段比较：疑似点数/出现帧率、致命格面积和持续时间；真实物体各帧保留与costmap标记；移除后清除时间；map→odom平移/旋转的时间序列、跳变量及其与AMCL更新时刻关系。定位连续性既看TF，也看旋转期间观测/odom和停止确认。没有预先约定的定位跳变容限时报告数值，不擅自给PASS。
5. 真实临时低反射障碍若出现漏标，即使/map背景仍有固定障碍也不能通过。≤3删多或定位变差才降到≤2；若拉丝仍在，降低阈值不会更有效，改为分析剩余来源/区域/安装。全部通过只能记“当前场景阈值验证通过”，不能外推双机动态避障。

当前未执行真实物体放置、旋转、costmap标记清除或定位连续性验收；现场物体信息仍待提供。停止时先结束旋转/任务并确认停止，再结束bag，最后退出本轮底层；恢复默认关闭配置。
