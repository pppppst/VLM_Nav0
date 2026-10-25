# Go2 障碍栅格连续周期确认 Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans; 用户已明确周期与次数，本轮直接实现、离线验证，不提交或部署。

**Goal:** Go2 协同动态障碍只有在同一平面地图栅格连续3个0.8秒周期命中后才添加。

**Architecture:** 复用现有 obstacle_cloud_filter，空间过滤后的原话题保留供清除与原消费者使用；增加确认输出供协同 Nav2 两张 costmap 标记。依据点云原始时间戳，按 /map 元数据与对应时间 TF 把点投影到固定平面栅格；每周期每格最多计一次，完成第三个完整周期后允许当前帧命中点输出。无点的周期/数据断档断开连续性，地图几何或时间回退重置。新输出不重放历史点。

**Tech Stack:** 现有 C++17 / ROS2 Humble / PCL / gtest / pytest；nav_msgs 已有运行依赖，提升为编译依赖。

**Spec:** 用户本轮明确要求“以0.8s为周期，连续3个周期探测到平面地图的同一栅格处有障碍才添加到障碍栅格”；下列周期边界/接线为实现选择。

## Global Constraints

- 同一周期中任意多帧/多点只算一次；周期由首个有效云时间起算，在下个有效云到达时结算完成周期。
- 使用实际 /map 原点、方向、分辨率及范围，不能在 base_link 或滚动 local 网格中累计。
- 缺失 map/对应时间TF时不得绕过确认；原始空间过滤点云仍可用于 clearing。
- 清除沿用 Nav2 射线，不新增未观测即自由或定时清图；静态/Keepout层不受影响。
- Go2 开启确认输出，仅协同最终 overlay 消费；Ranger及原单机 costmap 标记链路不改。
- 新输出构建用于隔离验证，不替换默认overlay，不启动现场ROS或运动。
- 新障碍确认通常延迟1.6～2.4秒（启动首周期起约2.4秒），快速移动障碍可能被过滤；本次不是硬件安全认证。

## Review Focus

地图原点/方向与机器人运动；周期边界、跳周期、重复/倒序时间戳；地图/TF缺失；标记与清除源接错；点字段及header保留。

### Task 1: 时序核心、节点与Go2协同接线

- [x] RED：新增 C++ temporal confirmation 用例，覆盖3周期、密集帧不加速、漏周期重置、边界、地图变更/旋转/负原点、乱序、字段保留；扩展实际合并配置断言，先记录失败。
- [x] GREEN：新增 temporal_obstacle_filter.hpp/.cpp 接入现有pipeline库；在现有节点发布原始过滤云和确认云，读取 /map 元数据与云时刻TF。Go2配置开启0.8/3，协同overlay将go2_cloud设为仅清除、go2_confirmed设为仅标记。
- [x] 验证：新独立成对构建（BUILD_TESTING=ON），运行原空间过滤与新时序gtest、相关Python配置测试；基线失败单列，不顺手修复。

### Task 2: 接线验证、审查与交接

- [x] RED/GREEN：扩展现有Domain31节点测试验证原云立即发布、确认云延迟、多帧TF移动仍对齐；原生costmap测试分别验证未确认点不能mark、确认点可mark、原云可clear。
- [x] 独立审查本轮增量，修复重要问题并运行针对性回归。
- [x] 更新runbook与session_handoff：准确记录构建输出、用例结果、未部署、延迟与残留障碍清除限制。

## 实际验证与审查结论

14项C++、35项Python/Domain31通过，新增11项；2项旧测试失败明确排除。详情与命令见runbook新增节及session_handoff。独立审查诊断OK问题已修复；TF晚到的原始判断经实验否定，实际同步等待阻塞raw云已先红测再改为有界异步等待并通过。测试构建仅在go2-temporal-20261025独立输出，默认未切换，不提交、不清图、不做现场运动。
