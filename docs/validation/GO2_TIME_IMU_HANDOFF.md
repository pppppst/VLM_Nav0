# Go2 系统时间与 `/utlidar/imu` 时间戳交接

Go2 与主机 NTP 同步已确认正常。`/utlidar/imu` header 相对 callback 时的
`system_now` 存在稳定正偏移；该值继续记录，但不再作为 FAST-LIO、
`SYSTEM_READY` 或 ARM blocker。没有加入 timestamp offset adapter，也未启动正式
FAST-LIO。

## NTP 配置与状态

- 本机 `192.168.123.222` 的系统时钟由原有 NTP 同步。
- 临时 chronyd 仅监听 `192.168.123.222:123`，使用 `-x`，不调整本机时钟。
- Go2 `/etc/systemd/timesyncd.conf.d/go2-lab.conf`：

  ```ini
  [Time]
  NTP=192.168.123.222
  ```

- Go2 显示 `System clock synchronized: yes`，ServerName/Address 正确，RTC 已由
  1970 校正。
- 第 3、4 个 NTP 样本 jitter 分别为 1.113 ms、1.087 ms，NTP 四个 timestamp
  一致，Packet 未被忽略。
- 本机 chronyd 从 `/tmp/go2-chrony.OVYlE2` 运行，主机重启后不会自动恢复。

## 两次跨机共同窗口结果

第一次 5 秒：

- LiDAR 77 帧，message age median `+14.376 ms`、minimum `+13.628 ms`。
- IMU 1254 帧，message age median `−52.137 ms`、minimum `−52.337 ms`。
- 最近 LiDAR/IMU header 差 median `−1.192 ms`，范围约 6.75 ms。

第 4 个稳定 NTP poll 后重复 5 秒：

- LiDAR 77 帧，message age median `+13.464 ms`、minimum `+12.389 ms`。
- IMU 1251 帧，message age median `−52.978 ms`、minimum `−53.210 ms`。
- 最近 LiDAR/IMU header 差 median `−1.153 ms`，范围约 7.03 ms。
- 两路都没有 timestamp backward。

## Go2 本机边界检查

直接在 Go2 本机运行，不经过以太网：

```bash
timeout 10 ros2 topic delay /utlidar/imu
timeout 10 ros2 topic delay /utlidar/cloud
```

结果：

- IMU average delay 稳定约 `−0.055 s`，std dev 约 0.05–0.09 ms。
- cloud average delay 约 `+0.011 s`。

因此 IMU 负 delay 不是 x86 接收端、跨机 DDS 或网络 RTT 导致，而是 Unitree
publisher、LiDAR/IMU 驱动或设备时钟到系统时钟的映射行为。

## Go2 本机 A/B/C 联合测试

2026-08-25 在 Go2 本机连续执行两轮 10 秒联合订阅。A/B 分别使用各自 callback
时的 `system_now`；C 为每帧 LiDAR header 与最近 IMU header 的差：

```text
A = imu.header.stamp   - imu_callback_system_now
B = lidar.header.stamp - lidar_callback_system_now
C = nearest_imu.header.stamp - lidar.header.stamp
```

- 第一轮：A median `+31.15 ms`，B median `−35.71 ms`，C median `+1.59 ms`。
- 第二轮：A median `+31.15 ms`，B median `−35.50 ms`，C median `+1.17 ms`。
- 约 250.5 Hz IMU、15.4 Hz LiDAR；两轮均无 header timestamp backward。
- 独立 A/B 使用不同 callback 时刻，不能直接用 `A-B` 代表 C。统一使用 LiDAR
  callback 时刻后，两轮均严格满足 `C=A_pair-B_pair`。

该结果暂未显示明显的 LiDAR/IMU 相对时间轴失配。A 的数值在不同测试阶段曾约为
31–55 ms，作为已知时间戳现象持续观察；其绝对值和正号不参与正式 timing gate。

## 原始数据健康度

- IMU 无 NaN/Inf、无 timestamp backward。
- 静止加速度范数约 10.12–10.61 m/s²，median 约 10.38 m/s²。
- 静止 gyro 范数约 0.0011–0.0215 rad/s，median 约 0.0087 rad/s。
- LiDAR x/y/z finite；point-level `time` 存在且变化。
- 点时间 span median 约 62.7 ms；点云约 15.37 Hz。
- 整帧 ring 单值现象仅作诊断，不作为 blocker。

## point-level time 静止验证更新

Unitree SDK 源码注释、ROS wrapper 和三轮 60 秒实测已经确认：cloud header 是
cloud/扫描起点 stamp，point `time` 是相对该 stamp 的正向秒偏移；固定版 SPARK
应使用 `timestamp_unit=0`。第三轮重建并匹配 3,353,118 个 point，最近 IMU 的
绝对差 P99 为 1.884 ms、最大 4.750 ms，无 header backward 或明显漂移。详细证据
见 `2026-08-25_go2_point_time.md`。

point-time 语义已经 PASS；静止数据只能证明共同时间轴和数值覆盖，不能证明动态
物理相位，因此 measurement-alignment gate 仍 fail-closed。不因
`imu.header.stamp > system_now` 单独 BLOCKED，也不添加固定补偿。Go2 充电后需在
bridge DISARMED 下由操作者手动小幅运动录制动态 raw bag；通过相关性与点云质量
复核前不进入正式 FAST-LIO 测试。
