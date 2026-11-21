# Piper 开机初始化与使能实施计划

目标：按用户明确选择，在 Go2 上位机开机后初始化实际 Piper USB-CAN 并使能六关节，保持当时姿态，不发送回位、夹爪、失能或复位。用户确认 Go2 的首次运动由开机完成后的遥控或导航触发。

方案：独立 systemd 后台服务（Type=simple、RemainAfterExit=yes），按实测 USB 序列号识别设备；等待设备、完整新鲜反馈与已有故障恢复，使用已安装的 SDK 0.6.2 V2 接口。成功确认后结束并保留 systemd active 状态，不在后续抓取/人工失能时反复抢占控制。不开机启动 ROS，不改变底盘启动顺序或增加导航门槛。开机服务无图形会话时用 journal 记录 state/reason_class/reason，人工诊断入口单独打开日志终端。

取舍：实验入口使能已由用户明确否决；固定延时盲发不能处理设备晚到；常驻重复使能会干扰之后抓取，因此采用等待就绪、一次开机流程确认完成的服务。

范围：新增 scripts/go2_piper_boot.py、scripts/install_go2_piper_boot.sh、scripts/piper_boot_status.sh、config/piper_boot/、test/test_piper_boot.py；CMake 注册定向测试，runbook 与交接记录实际路径和验收边界。保留现有 dirty 工作，不提交。部署服务仅 enable、不 --now；sudo 需上位机用户现场输入密码，不收集密码。

- [x] 先写并运行设备身份、CAN 配置、逐关节新鲜度、故障等待、使能与只读分支测试。
- [x] 实现最小服务、安装及诊断入口；原始 API 仅 EnableArm(7) 可发送动作类命令。
- [x] 定向测试、上位机 Python 3.8 语法/服务校验和只读反馈验证；不发送使能。
- [x] 按 runbook 构建三包，新输出验证服务资产；独立上位机服务不切换无关导航 overlay。
- [x] 复核、上传实际构建产物、记录 hash 和用户最后安装命令；安装后启动/重启验收仍待现场。

已核实：unitree-go，systemd 245，Python 3.8/aarch64，SDK 0.6.2；USB 1d50:606f，serial=0046002F5246570520323934。sudo -n 提示需要密码。任务由主代理连续实施，最后做一次独立复核。

复核决定：不用 Type=oneshot，避免设备缺失时把 multi-user.target 和图形登录阻塞；READY 日志及 active(exited) 才代表本次开机完成，active(running) 可能仍在等待。

交付状态：最终部署文件已上传并逐项校验，系统安装等待用户sudo；未执行服务启动或重启。新用例24项均通过；独立复核通过。
