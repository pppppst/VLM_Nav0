# 2026-10-12 雷达强度离线审计

结论：intensity 有来源且传递正常，但障碍输出中的值已经经过体素平均。当前样本的疑似点全为零，真实障碍对照也含零强度点，不能仅凭 intensity 判别。未新增软件过滤、未修改任何雷达/导航参数、未新增 ROS 订阅或采集。本轮只进行了已有数据离线计算与雷达 Web 的只读 GET。

## 数据与标注依据

证据目录：`build/go2-field/evidence/intensity-audit-20261012/`，包含复算脚本 `analyze_intensity.py`、`statistics.json`、逐点数据 `point_samples.npz`、分布图 `intensity_comparison.png`、原生设备页面/查询结果及体素验证程序。

场景数据：`build/go2-field/evidence/cloud-clearance.20261012_205145/cloud_*.cdr`，31 帧、接收跨度 2.9976 秒、91635 点。原有 XYZ-only PCD/NPY 不含强度，本次直接解码完整 CDR。用户确认真实障碍未移动，实测位置与 /map 障碍位置偏差在 5 cm 内。用同期 map 和保存的 map←base_link TF 对齐，计算点到占据栅格**方形范围**的最短距离（不是只计算到格子中心）。

疑似 ROI：base_link 中 0.45<x<0.60、|y|<0.18 m，共144点，全部 intensity=0。该 ROI 只来自先前近裁剪边缘现象，不是已证实的相机干扰空间。37点同时进入地图障碍5cm容差区，保留为歧义点，不强行归类。

对照范围：0.45<x<1.20、|y|<0.40 m，距地图占据格≤0.05 m；排除与疑似 ROI 重叠点。地图标签只支持位置对照，不能识别材质，也不能证明某个点一定是桌腿。

|类别|点数/出现帧数|intensity min / P25 / median / P75 / max|雷达距离估计范围|
|---|---|---|---|
|无歧义疑似点|107 / 31|0 / 0 / 0 / 0 / 0|0.316–0.460 m|
|地图障碍对照|3517 / 31|0 / 2.25 / 12.25 / 30 / 96|0.482–1.113 m|

疑似点 XYZ 范围：[0.4529,0.0023,0.0139] 到 [0.5759,0.1107,0.0698] m。
对照点 XYZ 范围：[0.6168,-0.1118,-0.1562] 到 [1.1987,0.3930,0.2000] m。
距离由 base_link 点减标定文件 `config/go2_calibration_xt16.yaml` 的雷达原点 [0.14543,0,0.13312] 后计算；是去畸变、体素化后的几何估计，不是原始 UDP 测距。只保存了场景 TF 快照，未逐帧运动补偿，存在定位与5cm栅格离散误差。两组距离不匹配，不能排除距离相关效应。

真实对照中176点为零，占5.00%。若只按强度阈值全局删除，阈值<1、<3、<5分别会删除对照点的7.68%、27.01%、39.04%，同时都删除全部疑似点。这里是诊断性分布统计，不是建议部署阈值。空间限定规则本帧能避开大部分对照，是区域分离带来的结果；不能证明低反射黑色障碍进入该区域后仍安全。停止调阈值，暂不实现软件过滤。

## 字段传递核对

1. Hesai SDK `udp6_1_parser.h` 将 `GetReflectivity()` 写入 intensity；协议 `udp_protocol_v6_1.h` 返回 uint8，范围0–255。ROS 用 float32承载该反射率码。它不是未处理的模拟回波幅度。
2. `hesai_fastlio_converter/src/converter_core.cpp` 读取 float intensity 并 memcpy 到输出，ring为uint16，时间字段转为 FAST-LIO 格式。旧 bag `fastlio-retest.1_h_1z2p/recording-ab.9ws2548r/B/bag` 中27组相同时间戳的原始/转换帧逐数组核对，XYZ、intensity、ring全部一致；原始与转换强度均0–255、ring0–15。
3. 当前 FAST-LIO lidar_type=2，velodyne handler复制 intensity；去畸变排序并改XYZ，不改intensity。内部 PointXYZINormal 不含ring，输出已丢失通道。`publishFrame` 的 `/cloud_registered_base` 与 `/cloud_registered_lidar` 使用 `cloud_undistort_`，不是用于定位的体素云；不能误把 world输出受dense开关影响的逻辑套在base输出上。旧bag两种输出均14帧、828118点、强度0–255，无ring。此bag与当前场景不同，仅用于传递核对。
4. `src/obstacle_cloud_filter.cpp` 做TF转换；`src/obstacle_cloud_filter_pipeline.cpp` 依次高度筛选、自身裁剪、5cm体素化。前两项筛点，最后使用 PCLPointCloud2 VoxelGrid，`setDownsampleAllData(true)`，包括intensity在内的字段参与平均。离线编译同类型最小验证：同体素强度2和10输出6，断言通过。当前对照数据出现2.25、12.25等值符合平均行为，不能当单个原始反射码。此阶段已无ring可供恢复。

## 雷达原生选项（只读）

向配置中的设备192.168.123.20读取首页、JS、setting.html，以及 `pandar.cgi?action=get&object=device_info`、`action=get&object=lidar_config`，未调用任何set接口。

- 实际产品 PandarXT-16，SW_Ver 1.0.15、FW_Ver 1.2.8、Up_Fpga_Ver 1.5.10。
- InterstitialPoints=0；当前设备自身UI明确定义0=OFF、1=ON，且支持该型号显示此选项。因此当前为关闭。
- ReflectivityMapping=0；当前UI定义为Linear Mapping。NoiseFiltering=0。
- 这些是当前查询状态，不可倒推历史bag采集时设置。原生开关的抑制效果尚未实验验证，不能据此确认近处疑似点就是其针对的间隙点。

## 验证和后续边界

离线脚本成功完成；27帧字段一致性断言通过；PCL体素平均验证通过。无需ROS初始化即可复算：仓库根目录清理PYTHONHOME/PYTHONPATH，source `/opt/ros/humble/setup.bash`，运行证据目录 `analyze_intensity.py`。C++验证使用系统PCL头文件/库直接编译（本机pkg-config未找到pcl_filters，已改为显式链接）。未运行全项目构建或导航测试。

尚缺：同场景原始点的通道关联、黑色/细桌腿独立标签、与疑似区相同距离的真实障碍对照、相机干扰因果验证。软件ROI/阈值尚无足够实测依据，不填写、不启用。若之后做原生过滤对照，只切换Interstitial一个变量，保持软件过滤不变，并同时检查近障碍保留与静止costmap标记/清除；本次未执行该现场实验。
