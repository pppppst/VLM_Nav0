# 2026-10-15 RViz RGB与扫描取消

## 已确认的失败经过

实验包：`build/go2-vlm-experiments/experiment.20261015_112422.0a4e4b4e`。
11:24:31选择P3（table_02/edge_4），11:24:35导航成功并停稳；随后6次Spin均成功。
11:25:36.066679记录`STOP_COMMAND_SENT: scan health lost`，view_index=5识别取消，
最终stop_confirmed=true。未发现该时刻Nav2动作失败或桥接FAULT记录。

旧adapter在tick中检查nav_ready/localization_valid失败后直接取消scan future，外层
exploration因此未走到保存HEALTH_FAILED分项的分支。result.json的health是探索前快照，
不能当作取消时健康。历史日志不能还原该瞬间具体哪个门槛下降，也不记录RViz勾选时刻。

## 无运动对照证据

目录：`build/go2-field/evidence/rviz-rgb-audit-20261015/`。仅使用现有read-only inspect，
另加自动退出的临时best-effort原始Image订阅；无运动、API、SSH或驱动重启。
用户确认终端保持运行；检查时RGB原话题publisher=1、subscriber=0，不能据此认定RViz仍勾选。
初次10秒单观察器检查就绪后55个样本/5.86秒无输入健康失败。

随后38秒inspect：先单观察器，约12秒加入第二进程RGB订阅12秒，再撤除该订阅。

| 对照 | 观察结果 |
| --- | --- |
| 原始RGB双订阅 | 13.149秒同步率22/30=0.733；16.085秒可用RGB-D帧年龄2.101秒，超过2秒门槛；25.830秒恢复 |
| 同期其他输入 | RGB-D异常转换时定位、local costmap、action服务、外部门槛接收时效均正常；桥接一直DISARMED，本次未ARM |
| 深度接收 | 恢复后的深度时间戳窗口含10.152秒断档；第二RGB订阅仍收到120帧、约10 Hz |
| 已有预览订阅 | 同样38秒inspect、额外订阅12秒，收到12帧约0.992 Hz；相机原始RGB仍仅1个订阅者；就绪后313个样本/33.642秒无输入健康失败 |

原始对照就绪后有97次rgbd_ready失败；预览对照为0。原始对照中同步率曾恢复但帧已过期，
说明不能仅放宽配对率消除该问题。此证据支持额外原始RGB订阅触发RGB-D消费异常，并能解释
用户报告的取消路径；尚不能证明历史取消瞬间的门槛，或区分远端驱动/主机负载/DDS传输原因。
临时订阅未运行RViz渲染线程，故不是完整GUI负载复现。

两次报告分别为`inspect_subscription_ab.json`、`preview_ab/inspect_subscription_ab.json`；
`extra_rgb_subscriber.json`及同名preview文件保存接收时间戳、端点和计数，timing文件保存阶段时间。
脚本与clockwise_offline.json一并保留。早先3秒链路快照226.6 Mb/s RX、无网卡drop/error增量，
仅反映恢复后状态，不能排除失败时或远端过载。inspect退出0本身不代表窗口无失败，须读报告。

## 最小修改及验证

- Go2 scan_angles改为-π/4至-2π共8项：顺时针45°×8，每步停稳后识别，原流程/时限不变。
- 复用现有`/vlm_nav/camera_preview`（1 Hz）给RViz，不增加相机订阅或预览节点。
  预览只在inspect/探索观察器运行时更新；入口退出后显示可能保留旧帧，不能当作当前图像。
- 扫描健康失败保存同一次检查的`SCAN_HEALTH_FAILED`：失败门槛、scan phase/index、
  RGB-D窗口时间戳/匹配率/帧年龄、待TF帧数、定位/costmap/外部门槛时效。先发送停止，再写日志。
  保留原健康门槛与取消条件；无健康计数清空或新同步实现。
- 仅构建multi_agent_nav到`build/go2-scan-health-20261015`；CoVLM_env同步安装路径与日志目录。
  根包继续复用manual-takeover版本，未改底层传感器、AMCL或Nav2参数。

相关测试共84项通过（新增3项取消快照测试、既有81项）；新增测试先在旧代码失败后通过。
配置测试先验证原始RGB话题不符合预览要求，再修改配置通过。安装adapter与源码一致；
另用实际安装adapter和stub运动对初始yaw=0、3.1、-3.1验证各发8次-π/4并回到初始方向。
此检查不启动ROS或API。无运动对照与离线测试均不代替真实顺时针/完整RViz现场验收。

启动和构建见[runbook](go2_runbook.md)末尾，本地Notes指引已同步。运行中的RViz不会自动读取新配置，
需仅重开终端2的RViz；终端4重新source环境后再按原流程操作。相机和终端1无需为本改动重启。
