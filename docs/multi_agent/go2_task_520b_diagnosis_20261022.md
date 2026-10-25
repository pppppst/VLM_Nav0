# Go2 task 520b：转向但未识别目标的排查

范围：仅任务 `520b4506478b4f6d926269244e3a9529`。用户观察到长时间探索、转向，眼前目标未触发停车。本次保留生产代码、参数及启动顺序，不恢复任务。

## 已确认事实

1. 地图选点请求成功：模型约 4.109 秒返回 P3，目标位姿为 `(7.435, -15.275, 0)`。这次输入为地图，不能当作相机目标识别通过。
2. Nav2 于 15:48:25.720 开始导航，15:49:45.740 取消，持续约 80.021 秒。期间发生 4 次障碍轨迹拒绝，15:49:02 有一次全局规划失败；没有导航成功记录。不能仅凭这些日志确定持续不到点的全部原因。
3. 本轮无 `ARRIVED`、`SCANNING`、相机识别结果或 `views/` 文件；behavior_server 日志也无执行扫描 Spin 的记录。用户观察到转向属实，但现有证据不能将该转向归为视觉扫描，更不能据此判模型漏检。
4. 实际协同路径在观察点导航成功、停稳及运动条件复核后才调用 `start_scan()`；扫描转向结束、帧就绪后才调用相机 ABC 识别。代码见 [go2_exploration.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/go2_exploration.py) 及 [Go2 adapter](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/go2.py)。导航途中没有目标识别触发路径，因而沿途目标出现不会触发目标停车。当前任务确实没有进入相机目标识别阶段。
5. 离线受控复现将导航 Future 保持 80 秒，实际生成器只调用导航；返回 SUCCEEDED 后才调用 scan。未初始化 ROS、调用真实模型或运动。相关六个源码模块与实际安装一致。
6. 15:52 的历史采样中，原生 `/vlm_nav/nav_ready=true`，但 agent 为 `PAUSED`、`nav_ready=false`、原因 `local navigation not ready`。agent 的 nav_ready 汇总导航/规划/转向 action、外部门槛、新鲜度、RGB-D、costmap 和内部运动故障等条件，不等于原生话题。

## 未确定事项

- 首次触发暂停的具体健康子项未保存。`last_health_details` 存在于 adapter 内存，但当前 runner 未绑定 adapter 的事件落盘回调，不能从总括原因反推出 RGB-D、costmap 或 Nav2 中哪项失败。
- 后续两次 CameraInfo 采样中，深度正常到达，彩色 CameraInfo 未收到；这是彩色分支待核实线索。未订阅原始图像，不能据此断言 RGB 图像停止，更不能回溯证明 15:49 的首因。
- 后续独立订阅的 costmap/TF 检查通过，只能证明采样窗口的上游状态，不能证明 agent 内部缓存及首次暂停时的状态。
- 15:55:48 后 Sport 日志出现外部控制接管并 DISARM，晚于 15:49:45 导航取消，不能解释先前暂停。该日志不证明当前实体停车或使能状态。

## 待用户审核的修改方向（尚未实施）

| 项目 | 建议及边界 |
| --- | --- |
| 识别触发时机 | 若要求眼前目标优先处理，先增加本机已停稳时的首次识别。若还要求沿途识别，须明确停下观察的策略；取消/终态/停稳确认后才能取新帧及识别，继续保留严格接近验收与 READY 条件。不能直接将检测结果当作到达或 READY。 |
| 首因可观测性 | 首次健康失败时保存已有 `last_health_details`，带任务身份与时间；不改变门槛或自动恢复策略。 |
| 长时间不到观察点 | 核对对应轨迹、障碍和进度检查配置后再决定处理方式，不根据本轮汇总日志直接改超时或障碍阈值。 |
| 彩色输入 | 在后续获授权的静止检查中核实实际 RGB 新鲜度、RGB-D 同步及相机 TF；当前不重启相机、SSH 或追加订阅。 |

## 证据与验证

- [历史日志核对报告](../../build/go2-field/task-520b-vision-recheck.o7uaxr5q/report.json)：含导航时序、日志副本、事件及源码/安装一致性。
- [离线触发验证](../../build/go2-field/task-520b-analysis.75gaybmd/recognition-trigger-offline.json)：80 秒未到点时不调用 scan，到点后才调用。
- [历史状态](../../build/go2-field/task-520b-analysis.75gaybmd/live-status.json)、[costmap 检查](../../build/go2-field/task-520b-analysis.75gaybmd/costmap-live-check.json)、[第二次 CameraInfo 采样](../../build/go2-field/task-520b-analysis.75gaybmd/camera-info-confirm.json)。
- 原始任务证据目录：`build/go2-vlm-experiments/520b4506478b4f6d926269244e3a9529_go2_1/`。不混用上午其他任务的扫描失败或模型连接错误。

本次只整理和核对证据；未修改运行逻辑、识别提示词、门槛或启动流程，未恢复任务、重启组件或追加模型调用。当前实体状态与完整现场验收尚未验证。
