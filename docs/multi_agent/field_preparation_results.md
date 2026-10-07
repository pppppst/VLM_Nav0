# Go2 现场接入准备记录（2026-10-07）

实际基准为 `go2-cooperative-task-layer` 的
`1ba25891794a5aab69072524353f632160878c84`，在原分支上修改。现场逐项清单、实际
文件/字段、填写方法与命令见 [go2_field_checklist.md](go2_field_checklist.md)。
本轮未连接 Ranger，未启动 bridge、Domain0 真实参与者、底层栈或真机运动。

## 实际修改

- 新增 `multi_agent_go2_check` 独立入口，默认 inspect，只读且无速度发布器；
  `--offline` 不初始化 ROS。plan/approach 只调用 ComputePath；navigate/scan
  必须显式选择动作并给出 `--allow-motion`，仍检查单机现场门槛和现有运行许可。
  不发布任务消息，不生成 fake Ranger READY，不授予门/C资源许可。
- 修复 Humble 参数查询接口为 `/map_server/get_parameters` / GetParameters；
  核对实际活动地图路径。新增 map_digest 绑定真实 YAML/image 文件，Go2/Ranger
  各自校验本机文件。配置目录拼错明确失败，不回退到默认模板。
- Go2 接近点现在使用新鲜 raw Costmap 的真实 payload、时刻 TF、origin yaw
  和实际未吸附网格中心的 footprint。未知/253/254格拒绝；ComputePath 返回后
  再检查 costmap。preflight 路径不能作为正在执行路径，执行路径拒绝旧目标前的
  stamp、过期数据和非有限坐标。
- 停止用新鲜 odometry 的独立三维线/角速度范数与持续静止，并同时要求本进程
  action 终态。动作发送前二次核对健康、许可、costmap、静止、终态和控制者。
  独立运动入口用 fcntl 本机互斥；未确认停止、停止接口异常或异常退出保留阻断标记。
- 完整 live 增加地图文件、场景有限点位/非退化区域、真实描述和实时 costmap 验证
  门槛；未知地图、坐标、标定、停止余量、路径来源仍为空，验证标记仍为 false。

旧 scripts/config/launch/multi_agent_comm 与 `1ba2589` 完全一致；旧 VLMNavigator
仅增加可选独立节点名和禁止创建停止发布器的被动观察选项，默认行为由全量基线验证。
旧消息包、状态话题、桥接配置和现有工作空间安装产物未改。

## 已执行验证与证据

构建命令见 [go2_runbook.md](go2_runbook.md)：system Python3.10 生成、既有
co-nav-real 运行，独立输出 `/tmp/go2-coop`；[build.log](field_evidence/build.log)
显示 3 包成功。重新执行：

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
bash scripts/test_cooperative_no_hardware.sh
```

| 分组 | 实际结果 | 证据 |
|---|---|---|
| 原单机 Python 基线 | 273 passed，8.83s | [baseline-final.log](field_evidence/baseline-final.log) |
| 原单机 C++ 基线 | 3 CTest 全过，含 21 gtest cases | [baseline-cpp.log](field_evidence/baseline-cpp.log) 与对应 XML |
| 协同/接入准备 pytest | 78 passed，11.06s；原49 + 本轮29 | [cooperative.log](field_evidence/cooperative.log) |
| 原生桥接配置解析 | 1 passed，仅解析，不启动 bridge | [test-result.log](field_evidence/test-result.log) |
| colcon 汇总 | 102 entries，0错误/失败/跳过 | [colcon-test.log](field_evidence/colcon-test.log)、[test-result.log](field_evidence/test-result.log) |

102 = 78 pytest + 1 parser + 2 CTest wrappers + 21 基线 gtest；它不包含另行记录的
273 Python 基线，也不是102个新增测试。ROS测试仅使用不桥接的Domain31/localhost；
独立入口运行分派回归把 rclpy.init 替换为 stub，因此没有 Domain0 participant。
双fake ROS测试保持独立 mock 前缀，未与真实协同连通。

独立审查的 footprint中心偏移、规划后许可变化、运动进程互斥问题均补回归并修复。
停止传输接口异常也先复现失败，再修复为未确认停止；针对性35项通过。
[review-regressions.log](field_evidence/review-regressions.log) 记录 RED/GREEN摘要。

已对重建后的安装入口执行 `--help`、`--offline`、
`--offline --require-ready`、候选地图摘要和不存在的 config-dir 检查。
[entry-help.log](field_evidence/entry-help.log)、[config-gates.json](field_evidence/config-gates.json)
显示24个完整协同缺项且 motion_sent=false；require-ready预期退出1，目录拼错也退出1。
[candidate-map.json](field_evidence/candidate-map.json) 仅证明仓库一份候选文件可读及其摘要，
不声称它就是活动地图。历史 tasks0—8 的 evidence 目录与记录保持原样。

## 未执行的现场验收

地图身份/活动文件确认；真实 Ranger 对齐；A/B/C描述、视点/等待/退出与门/C多边形；
执行/重规划路径来源关联；单机 Navigate/Spin、接受前后取消及真实停止余量；
RGB-D外参/深度单位/ABC识别；实时 costmap 动态 marking/clearing、断输入/失效TF；
Ranger adapter、本地域隔离、桥接路由、双机互为障碍和完整顺序任务。

尤其 AMCL 最终 static-map 参数覆盖删除动态层，且没有已验证的执行路径 exporter。
它们在 [现场清单](go2_field_checklist.md) 中保留明确缺口。现场未知值继续待填，
完整协同门槛保持关闭；单机验收入口不能替代它。
