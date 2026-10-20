# RGB-D 健康门槛：成功/失败实验对比与修复

## 实验与安装证据

- 最近完整成功：`build/go2-field/explore-approach.20261013_183747`，
  `fixed_viewpoints`，导航、四向扫描、识别和接近后 `TARGET_APPROACHED`、`stop_confirmed=true`。
- 当前失败：`build/go2-vlm-experiments/experiment.20261013_205504.8f852ffd`，
  `generated_candidates_vlm`，选点请求后8.989ms因RGB-D同步健康false退出，尚未发送运动。
  ARM/NAV/定位/costmap/action服务正常；VLM晚到合法P1未应用。
- 两者入口均为 `multi_agent_nav/multi_agent_go2_check --action explore`，内部共用
  `go2_exploration.explore`、`Go2Adapter`、`make_observer`和根包`VLMNavigator`。
- 旧启动指引备份 `build/tmp/start-guide-20261013/original.md` 明确要求协同模块来自
  `build/go2-goal-costmap-20261009/install/multi_agent_nav`；成功事件未出现新版
  `ARRIVAL_CONTRACT`，与该旧实现相符。成功实验未保存进程的`__file__`清单，不能倒推
  每个模块的绝对加载路径为已实测。另核对arrival-contract安装，其相关安全语义相同。
- 失败版本为 `build/go2-health-abc-20261013/install`，事件包含该版新增的健康分项。
  比较camera-tf、intensity-default、generated-candidates及health-abc四份根包安装：
  `record_rgbd_stamp`、`on_rgbd`、`resolve_pending_rgbd`、`vlm_input_ready`的AST摘要完全一致。
  因而旧根包来源的不确定性不影响本次相关函数比较结论。

证据和函数摘录：`build/go2-field/evidence/rgbd-pair-comparison-20261013/`。

## 四项核对

| 项目 | 旧成功版本与当前失败版本 | 结论 |
|---|---|---|
| 配对方式 | 均由`ApproximateTimeSynchronizer`等待完整RGB-D对，`on_rgbd`校验、缓存，`resolve_pending_rgbd`等待图像时刻TF；但健康配对率在每个单路消息回调中提前更新 | 配对机制未替换；健康读取可能落在两路消息之间 |
| 取消条件 | 旧`get_health`同样将`vlm_input_ready`计入nav_ready；`explore.health_check`和扫描`tick`持续检查，单次false即停止 | 旧版并非只在启动前检查；新策略新增早期VLM选点等待，但未新建这项取消规则 |
| 统计初始化 | 每次创建observer时初始化30帧窗口；启动收集/等待后进入任务；选点、`AsyncVision.submit`及取图均不清窗口 | 未发现VLM请求清空统计；原单机`reset_task`/`enter_sensor_wait`可清snapshot，但协同选点没有调用它们 |
| 执行方式 | 都使用`SingleThreadedExecutor.spin_once`和后台`AsyncVision(ThreadPoolExecutor)`；worker不改RGB-D缓存 | API不会同步等待并阻塞主回调；主循环会在单路回调完成后读到尚未配对的窗口状态。不是并发读到半个dict |

新策略双图渲染在主线程，会推迟该段时间内的回调处理；网络API在worker中。
现有日志不能证明渲染造成现场丢帧或延迟，不将它当已确认根因。

相关配置在上述安装中一致：raw depth；RGB `/camera/camera/color/image_raw`、深度
`/camera/camera/depth/image_rect_raw`；slop=0.05s、window=30、minimum_samples=10、
minimum_rate=0.90；完整帧期限2s、相机TF期限3s、图像TF队列等待5s/容量16。

## 最小修复

复用原matcher、统计函数、缓存和图像TF等待：单路回调只积累时间戳，
在既有`on_rgbd`完整配对回调中更新`rgbd_sync_status`。没有新增同步模块、计时器或订阅。
没有改取消条件、配对率阈值、图像/TF期限、Nav2、驱动或定位参数。

完整对到来时仍用两个30帧窗口的**全部样本**计算配对率，持续低配对率仍false。
单路或双路停流不能更新完整帧缓存，原2秒帧期限仍将输入关闭；TF失效仍受原3秒期限约束。
不会使用单路消息延长有效图像寿命。初次不足10个样本仍不可用。

## 离线验证及边界

复用此前人工到达序列：27/30匹配 → 单路到来时26/30 → 另一流到来恢复27/30。
测试真实`SimpleFilter`、原`ApproximateTimeSynchronizer`、`on_rgbd`和图像TF缓存，
以2×2内存图像与固定TF替代硬件；未初始化ROS。RGB先到、深度先到两例先在旧代码上失败，
修复后通过。另覆盖启动不足样本、持续低配对率及恢复、RGB单流/深度单流/停流超时、TF过期。
此序列为已有离线人工例，失败实验没有原始时间戳窗口；不能宣称重放了20:55真实消息序列。

受影响包vlm_nav隔离构建到 `build/go2-rgbd-pair-health-20261013`；相关检查36项通过：
runtime13、健康/取消/worker19、环境3、原图像时刻TF重试1；新增7项，既有29项。
根包测试以`--noconftest`从/tmp导入新安装；安装模块与源码逐字节一致。
只读复核无阻断问题。未跑无关全量基线、真实API或真机，不自动提交。

`CoVLM_env.bash`最后加载新根包，协同包仍来自health-abc安装；旧安装保留，已有节点不切换。
构建、复核及原入口命令见[runbook新增小节](go2_runbook.md#2026-10-13rgb-d完整配对边界健康修复)。
