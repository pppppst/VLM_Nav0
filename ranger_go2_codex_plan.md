# Go2 + Ranger 不同地图下的最小协同导航：最终架构方案

日期：2026-10-07。本文是架构设计与后续 Codex 交接约定，尚未编写协同架构代码，也未审查实际仓库；不代表部署或联合验收已经完成。本版本替代此前“双方加载同一 occupancy map”的方案。

部署修订：按用户于2026-10-07再次确认的实际结构，两端本地 ROS 均为 Domain0，各自 bridge 接入共享 Domain40；此前30↔40↔31描述不用于本次部署。Go2控制电脑 Domain0/CycloneDDS 仅绑定 enp4s0，Domain40仅绑定 wlo1。以下为用户确认背景，不是本会话新做的网络验收。

## 1. 已确定背景与任务

- 两台机器人各使用独立控制电脑。Go2 使用自己的地图；Ranger 使用独立建图得到的另一张地图。
- 用户已验收 Ranger 定位、已知地图中旋转寻找物体和导航；Go2 沿用其已验证的本机栈。不能把双方单机成功等同于跨地图目标共享已经验证。
- 固定 Go2 找 A、Ranger 找 B；每次视觉观察同时检查 A/B/C。
- 当前有效自身 A/B 接近任务的 Nav2 返回 SUCCEEDED，即模拟取物成功并置 ready=true。观察点、等待点、过门的内部导航成功不能置 READY。
- 双方 READY 后寻找/使用同一个篮子 C，依次接近模拟放入，再退出。无机械臂，两台都完成放入和退出才算成功。
- 场景两个房间及窄门/走廊，手工配置观察点、房间、通道和等待点。
- 第一版使用固定规则 planner，不接模型 global planner、不自动生成覆盖点、不增加先 READY 者帮搜。
- 当前讨论架构，不执行产品代码或真机通信测试。后续 Codex 优先在 Go2 控制电脑实现公共任务层和 Go2 适配，Ranger 电脑单独接入。
- 既有启动脚本位于 VLM_Nav/scripts，后续先读取实际入口；若真实接口、硬件或启动背景不符合本设计，明确报告并适配，不猜文件名或话题。

## 2. 总体架构与模块部署

采用“两个本地 robot_agent + Go2 电脑上的轻量 coordinator”。各自 occupancy map / AMCL / Nav2 / 传感器处理继续独立，协同层以共同参考坐标交换信息。

| 模块 | 部署与 Domain | 职责 |
|---|---|---|
| Go2 robot_agent | Go2 PC，Domain0 | 执行本机导航/扫描/接近；检查 ABC；判断本机可达性；交换共享坐标与状态 |
| Ranger robot_agent | Ranger PC，Domain0 | 同上，使用 Ranger 自己的地图和导航配置 |
| FrameAdapter | 各 agent 内部功能 | 本机地图与共同参考坐标的正反变换，检查坐标标识/标定版本 |
| coordinator | Go2 PC，Domain0 | 阶段、状态、目标线索、命令回执、观察记录与资源互斥 |
| RulePlanner | coordinator 内 | 固定取物归属、人工观察点顺序、双方 READY 后 C 阶段安排 |
| 既有 bridges | 两台 PC，各自Domain0↔共享Domain40 | 按PC来源和话题定向转发上层任务消息，保持已有通信架构 |

FrameAdapter 是数学变换与检查，不要求独立 ROS 节点。RulePlanner、目标记录、资源管理可以同处 coordinator 程序，分别保持清晰代码边界，无需额外服务器。

每个 agent 是本机任务层唯一的 Nav2 目标控制者。底层启动入口可以保留；联合运行时停用会同时发送导航目标的旧自动任务入口。

## 3. 两张地图与共同参考坐标

### 3.1 三种坐标身份

| 身份 | 定义 |
|---|---|
| Go2 本机地图 | Go2 定位/Nav2 使用的地图，独立 local_map_id |
| Ranger 本机地图 | Ranger 定位/Nav2 使用的地图，独立 local_map_id |
| common_frame = lab_ref | 协同参考坐标；第一版数值上与当前 Go2 地图坐标一致 |

Go2 的 local→common 为恒等变换；Ranger 的 local→common 是经标定的二维刚体变换 T_common_from_ranger，由旋转 yaw 和平移 tx/ty 构成。导航目标统一使用地面平面中的位置/朝向，米和弧度。共同坐标本身不是第三张 occupancy map。

两个底层栈仍可各自在原生 Domain 使用名为 map 的 TF frame；通信中的 local_map_id / common_frame_id 必须显式区分来源。相同 frame 名、YAML origin 或地图分辨率不是实际坐标对齐的证据。

### 3.2 标定与有效性

- 第一版用至少 3 个分散、避免近似共线的对应固定参照点，拟合 Ranger→Go2 的二维旋转和平移；优先墙角/门框，避开移动家具。
- 至少一个未参与拟合的参照点验证误差，检查两个房间/通道附近能否同时对齐。
- alignment.yaml 记录两个 local_map_id / 地图版本、common_frame_id、alignment_id、变换方向、tx/ty/yaw、拟合与独立验证误差。
- 误差上限根据窄门余量和目标接近要求由现场设定，报告实际误差；没有通过验证不能启用跨地图坐标导航。
- 若单一刚体变换无法使两房间同时对齐，检查参照点与地图几何畸变；本轮不增加非刚性地图配准，先解决对齐条件。
- 重建任一地图或更改其几何坐标后，旧 alignment_id 失效，需重新验证。任务期间不热更新标定。
- 一张地图图片可以用于人工标点，不需要合并 occupancy、共享原始点云或把整套 TF 跨域发送。

### 3.3 FrameAdapter 的转换边界

| 数据 | 处理 |
|---|---|
| 本机位姿 | agent 获取本机地图位姿，转 common 后发送 AgentState |
| 本机识别目标 | 图像对应深度/TF 投影到本机地图，再转 common 后发送 Observation |
| 收到目标/观察点指令 | common→本机地图后，生成本机 Nav2 目标 |
| 门口/等待区/资源区域 | common 中定义，转到本机进行路径/边界监护 |
| 本机路径与整机 footprint | 本机判断并按需转 common，供资源请求或退出确认 |

位姿朝向也要旋转；区域多边形要变换所有顶点。目标物体位置不直接作为通用停车点，接收方在自己地图/costmap 中选择有效接近点。转换失败、定位无效、深度无效或标定版本不匹配，不执行该坐标指令。

## 4. 本地 agent 与 coordinator 的职责修改

### 本地 robot_agent

1. 接收/去重命令，将共享坐标转为本机坐标。
2. 用本机 Nav2 与 costmap 检查可达性、计算路径和选择接近点；Go2 的自由格/路径不能作为 Ranger 可达性的证据。
3. 串行执行导航、扫描、目标接近；将结果绑定到当前 task / command / Nav2 goal。
4. 依据本机路径判断是否经过受控通道，申请许可；无许可在区域外等待。
5. 用转换后的资源区域监护通行、重规划和整机退出，报告资源状态。
6. 发布自身状态、ABC 识别结果和 common 坐标；订阅对方状态与线索。

### coordinator

1. 汇总 common 中的双方位姿、物体线索、房间/通道标注和观察记录。
2. 唯一维护任务阶段、C 绑定、观察点预约及资源许可。
3. 用 RulePlanner 提出下一步任务，并基于本机 agent 的接受/失败/资源反馈调整。
4. 不在 Go2 occupancy map 上替 Ranger 规划路径，不把共享物体位置转换成两台通用停车点。
5. 第一版候选任务发给对应 agent 后，若本机判不可达则拒绝/失败并尝试下一个候选；未来 planner 需要比较路径代价时，再增加按候选 ID 查询并回报本机路径代价的能力。
6. 不发送速度命令，不代替本机实时避障，不因 READY 或 Nav2 内部子目标成功错误释放通道。

## 5. 任务状态与最小搜索策略

全局阶段：IDLE → SEARCH_AB → SEARCH_C → DEPOSIT → DONE，另有 PAUSED / FAILED。

ready / deposited 是独立任务标志，运行状态可为 IDLE / NAVIGATING / SCANNING / WAITING_RESOURCE / WAITING / PAUSED / FAILED。ready 后去等待点时标志保持。

### SEARCH_AB

- 新任务显式启动并生成 task_id；双方真实在线、各自定位/Nav2 可用、共用标定已验证、地图身份对应正确。
- scene.yaml 中各机器人配置 pickup_target、人工观察点顺序。点以共同物理位置的 viewpoint_id 和 common 坐标描述，允许机器人专属点；能否到达分别判断。
- 初始优先不同房间。coordinator 避免同一点并发预约，但不把整间房永久归属某台。
- 去观察点、停止、多方向扫描；每张图同时检查 ABC。有自身目标有效线索则转接近，无线索继续列表。
- Go2 接近 A、Ranger 接近 B 的当前有效任务 Nav2 SUCCEEDED 后置 ready，并互发状态。
- 先完成者去可达且不堵通道的等待点，不自主帮搜；等待点任务须完成/确认取消后才能替换任务。
- READY 只证明当前导航层模拟取物，不加入独立距离或到达后视觉复核。
- 另一台看到本机目标时可共享 common 物体位置；收到线索不能直接置 READY。

### SEARCH_C / DEPOSIT

- 双方当前任务 ready 且状态新鲜/定位有效后，统一进入 C 阶段。
- C 可以在此前被识别保存；屏障前禁止为了 C 执行搜索或接近。
- 已有有效 C 就绑定共同 observation_id；否则分配不同观察点继续检查 C，保留之前的观察记录。
- 多次没看到不证明区域无目标。记录 robot / viewpoint / 扫描方向 / 实际检查目标与结果；不同高度/视角的记录不能合成“完整覆盖”。
- 每点每台每阶段最多完成一轮扫描；失败重试一次。列表耗尽报告本轮未找到，不宣布目标不存在。
- 绑定同一个 C 后，Go2 先接近，Ranger 后接近。两台停车点可以不同，必须各自可达且指向同一物体。
- 本机 APPROACH_DEPOSIT 主任务的 Nav2 SUCCEEDED 置 deposited；随后退出受控接近区，确认整机离开后放行下一台。
- 两台都放入并退出才 DONE。接近区/退出点缺乏有效配置时暂停，不生成未经验证的坐标。

### 通用失败处理

- 导航默认重试一次；目标接近可改选另一个有效接近点。仍失败则对该 robot / observation_id 标记不可用，恢复搜索，不循环重用失败线索。
- 心跳初始 2 Hz，失联阈值初始 3 秒，任务超时初始 600 秒；全部配置化。
- 按接收端单调时钟判断新鲜度。失联/定位无效时取消活动导航并确认本机停止，禁止推进阶段。
- 取消/暂停后的晚到成功回调不得置 READY 或 deposited；旧 task / command / goal 结果不影响当前任务。
- coordinator 重启默认暂停；第一版核对并停止双方后新建任务，不做自动断点恢复。

## 6. 通信与消息

### 6.1 保留已有跨域链路

用户已报告双向、跨电脑、跨 Wi-Fi 通信成功；最新确认的部署结构为：

Ranger PC本地Domain0 ↔ Ranger bridge ↔ 共享Domain40 ↔ Go2 bridge ↔ Go2 PC本地Domain0。

已知旧测试话题 /multi_agent/test/ranger_to_go2，反向也曾验证；最新既有agent状态话题为 /multi_agent/agents/ranger/state 与 /multi_agent/agents/go2/state，分别从 /odom 和 /odometry 获取本机信息。实际消息类型与配置由Go2本机Codex静态确认，不将里程计状态直接当作协同任务READY。

- coordinator与Go2任务agent在Go2 PC Domain0；Ranger任务agent在Ranger PC Domain0。robot_id和来源话题区分机器人，Domain数字不是机器人身份。
- Go2有线侧 enp4s0=192.168.123.222，连接Go2上位机192.168.123.18；Domain0的ROS侧相机接口、Hesai XT16、body IMU、FAST-LIO、定位/SLAM、Nav2和agent沿用其网卡配置。实际相机驱动部署位置保持现有入口。
- Go2无线侧 wlo1=192.168.1.118；Ranger PC无线侧 wlo1=192.168.1.108；Domain40仅承载高层任务通信。控制电脑运行任务层与协调，不在Go2上位机上部署coordinator。
- 两端本地域同为0可用，依赖实际DDS网卡/发现隔离；Go2 bridge的Domain0参与者也必须沿用enp4s0限制，不能因Domain40使用Wi-Fi而把两侧都绑定wlo1。后续现场复验确认Ranger Domain0没有绕过bridge直接发现Go2 Domain0；本轮仅静态审查。
- 桥接只承载上层任务消息，不增加 /tf /odom /scan /map 或 Nav2 action 的跨端转发。
- 现有测试消息成功不等于新增自定义任务消息已通过，新增话题/类型还要配置并后续验收。
- 当前仅讨论设计；此前“不尝试跨端通信”的执行限制仍保留，直到现场明确安排复验。Ranger 单机已恢复可用不自动意味着可以远程操作其电脑。

### 6.2 三类消息，坐标含义修改

| 消息 | 主要内容 |
|---|---|
| AgentState | task_id / robot_id / stamp，local_map_id，common_frame_id / alignment_id，common_pose，定位/转换有效性，运行状态，ready / deposited，当前命令与接受/执行/终态回执，失败原因，资源请求/占用/退出反馈 |
| Observation | task_id / robot_id / observation_id / viewpoint_id / target_id / stamp，source_map_id，common_frame_id / alignment_id，识别结果，common_position / position_valid |
| Command | task_id / command_id / recipient_id，phase / kind / action，目标或观察点 ID，common_frame_id / alignment_id，common_goal或共享物体位置，资源 ID / token，取消所指向的命令 ID |

- 上层坐标都明确为 common；local_map_id 用于来源追溯，不要求两台相等。
- 任务绑定 common_frame_id 和 alignment_id。Go2 与 Ranger 配置必须引用同一标定版本，各自地图必须对应标定中记录的地图版本。
- 不混用 common_position 与 source_map_position。保留原始本机坐标在本地日志即可。
- Command 区分执行命令与 SYNC / PAUSE / CANCEL / GRANT_RESOURCE 控制事件。资源授予关联当前主任务，不取消或重启其导航；只更新许可。
- EXECUTE 命令单调编号、幂等；每台最多一个活动主任务，内部扫描/过门子动作不产生取物完成。
- agent 持续回报 ready / deposited 和命令终态，不依赖一次性信号；状态变化立即发布。
- SYNC 为 coordinator 心跳和阶段快照，不触发新任务或导航。
- 未识别/不确定/API失败必须区分。无效识别或深度不发布可导航位置；API调用不能阻塞心跳。
- 若统一的三目标请求实拍不可靠，可同图分别询问，保持相同 Observation 语义。模型名称、地址和超时配置化；API key 使用现有环境变量。
- observation_id 保留同一源线索身份；仅做坐标转换不会生成“另一个 C”。

### 6.3 定向话题与桥接

| 建议话题 | 源域 | 方向 |
|---|---|---|
| /multi_agent/ranger/state | Ranger PC，0 | Ranger PC 0→40→Go2 PC 0 |
| /multi_agent/ranger/observation | Ranger PC，0 | Ranger PC 0→40→Go2 PC 0 |
| /multi_agent/go2/state | Go2 PC，0 | Go2 PC 0→40→Ranger PC 0 |
| /multi_agent/go2/observation | Go2 PC，0 | Go2 PC 0→40→Ranger PC 0 |
| /multi_agent/go2/command | Go2 PC，0 | Go2 PC 0→40→Ranger PC 0；Go2 agent本地订阅 |

三种任务类型映射到上述五个物理话题。保留旧multi_agent_msgs及 /multi_agent/agents/*/state 里程计消费者，新任务协议使用multi_agent_task_msgs和上表独立话题；不要在同一话题混发两种类型。Domain0重复时不能仅写数字0→40→0而忽略PC来源：Ranger桥只导出Ranger源话题并导入Go2源话题，Go2桥反之；同一话题不得在两个桥都配置双向回灌。消息类型/QoS必须两端和桥接配置一致；可靠传输仍需应用层去重与回执。

## 7. 窄门互斥与实时障碍

### 7.1 资源在 common 中定义，本地判断进出

- 通道 resource_id 在 common 中唯一，区域覆盖整段不能会车的空间；两端将区域和入口/出口/等待点转到本机地图。
- agent 用本机实际路径检测是否经过通道，通过 AgentState 请求许可；coordinator FIFO 授予 GRANT_RESOURCE，token 绑定任务、主命令与机器人。
- 无许可在区域外停止。获得许可后按等待点/出口点分段通过，再继续主任务。
- 监护重规划、区域边界及实际停止余量，普通导航不得绕过许可。不能仅检查最初路径。
- 确认整机 footprint 离开区域后释放；定位无效、持有者失联或退出失败时保留占用，不按超时推断区域已空。
- 与对齐误差有关的等待距离/区域余量需现场验证；数学变换通过不等于窄门真实通行通过。

### 7.2 C 接近区

- 同一个 C 在 common 中定义受控接近区域，覆盖两台实际选用的接近与退出路径需要保护的空间。
- 本地 agent 选择可达接近点与退出点，coordinator 维护共同资源占用；不共享源机器人停车点替代本机判断。
- deposited=true 不释放许可；整机退出才释放。
- 先过门再申请 C 接近许可，第一版同一机器人最多持有一个互斥资源，避免持篮子许可等待门。

### 7.3 各自实时 costmap

- 各自使用本机实时 scan/点云、机身 footprint 和障碍筛选配置。
- 验证能够标记对方机身，并在实际观测空闲后清除；静态图不是动态机器人障碍来源。
- 地图高度/占据不同是正常输入差异，禁止强行复制对方障碍阈值或覆盖本机地图。
- Go2 已验证 AMCL 输入保持，必要时单独提供避障输入；检查实际运动方向感知覆盖。
- 共享位姿只辅助许可协调，本机实时感知负责房间内避碰。

## 8. 配置与后续扩展

四类配置足够，文件名为建议设计名：

| 配置 | 内容 |
|---|---|
| alignment.yaml | 两台地图身份/版本、common身份、标定变换、alignment_id、验证误差 |
| scene.yaml | common中的房间/通道/观察点/资源/等待点、ABC描述、固定分工与任务规则 |
| robot_go2.yaml / robot_ranger.yaml | 各自地图路径、实际接口、footprint、扫描/接近策略、模型配置 |
| 既有 bridge 配置 | 上层话题、真实消息类型、Domain与定向转发 |

策略边界保留 decide(snapshot) → planned_actions。snapshot 包含双方 common 位姿、目标线索、观察记录、资源状态与本机执行/可达性反馈；输出引用合法技能和已有目标/观察点 ID。

以后 ModelPlanner 可替换 RulePlanner，决定区域优先级、动态帮搜、重访和顺序；coordinator仍负责状态、校验、回执与互斥。模型不根据 Go2 地图臆断 Ranger 可达性，不自行生成未校验坐标。API感知模型与规划模型是不同逻辑角色，名称可独立配置，同一个服务也可复用。本轮不调用新规划模型。

后续如需原图参与联合规划，按事件传带源机器人/时间/位姿的关键帧；当前无需增加图像桥接。

## 9. 后续落地顺序与验收边界

本节是后续顺序，不是立即执行授权；当前仍处于设计阶段。

1. 确认两端实际代码/脚本/消息/桥接接口，保持已验证单机栈；各自地图路径分别配置。
2. 采集对应参照点，拟合并独立验证 alignment；检查点、姿态和区域转换。
3. 公共三类消息 + agent + 规则 coordinator，以两个模拟 agent 测试 READY屏障、旧命令回调、重复事件、失联和资源退出。
4. 在 Go2 电脑接 Go2真实接口；同机模拟 Ranger 只能标记为逻辑测试。向 Ranger 电脑交接相同公共代码/消息/标定版本。
5. 在 Ranger 电脑接 Ranger真实接口，保持其独立地图；两端分别验证本机目标接近、取消和扫描状态。
6. 现场安排后，通过Ranger PC 0↔共享40↔Go2 PC 0桥接验证真实状态/命令回执、网卡发现隔离；验证共享目标经坐标转换后确实对应同一物理位置。
7. 低速联合验收对方机身实时障碍、窄门竞争和C放入后退出。
8. 完成 Go2→A、Ranger→B、双方READY、同一C的两次模拟放入与退出；至少包括一次“对方发现本机目标”的线索利用。

必要验证包括：
- alignment独立验证点误差达标；正反转换一致；姿态和多边形转换正确；地图/标定版本不匹配被拒绝。
- 本机不可达只拒绝该机器人候选，不阻止另一台利用自身可达路径；同一物体两台停车点不同合法。
- 未检出不等于目标不存在，API失败不等于扫描成功。
- 只有当前有效A/B接近主任务Nav2成功置READY，过门/等待/扫描子目标成功不触发。
- 两端均收到READY，只有屏障后执行C任务；同一C线索与资源身份保持。
- 失联实际停止、资源不误释放；重复命令不重复取放。
- 记录逻辑测试/本机真机/跨端通信/完整真机四类结果，不将用户单机验收背景扩写为本次代码验证通过。

## 10. 给后续 Codex 的边界说明

架构代码目前未写。实现前读取本文件和实际仓库，优先参考 VLM_Nav/scripts。按两台不同地图接入，Go2电脑Domain0部署coordinator与任务agent，Ranger电脑Domain0独立部署任务agent；公共接口中的坐标均为lab_ref，FrameAdapter在各端通信边界完成转换。地图文件可以不同，标定关联身份与版本必须正确；可达性/路径/接近点由各自本机判断。保留各PC Domain0↔共享Domain40桥接及Go2 enp4s0/wlo1隔离，准备按PC来源定向的新任务消息配置。未获现场新的通信安排前，不尝试连接Ranger或跨端测试；单机恢复可用不代表当前能远程访问。禁止重写已验证定位栈、强制两端使用同一occupancy、把Go2自由空间作为Ranger通行依据，或把模拟结果当真机成功。本轮只实施规则策略，预留模型planner边界。

本文未填入具体标定数值、场景坐标、接口文件路径或模型迁移结果，这些需要现场数据或仓库读取，不生成虚假值。
