---
name: find_target
description: Ask the existing Go2 executor to find and approach the host's exact target once.
---

只调用 start_find_target(target)，目标逐字符使用宿主传入的原文，包括空格、标点。
完整搜索、ABC 识别、内部重试、导航、靠近与停车检查均由原程序执行。
不得修改目标、命令、环境、参数或期限，不调用 shell/ROS，不查看相机或重做识别。
一轮最多一次搜索；完成、失败或取消后都不能重新启动。

随后串行调用 get_task_status(task_id)；服务端每次等待约 1 秒，不并发或忙轮询。
未知状态、缺失证据、模型 finish 或退出码 0 都不等于成功。
SUCCEEDED 仅表示原搜索执行器验收成功；宿主 DISARM 和读回 DISARMED 后整轮才能成功。
FAILED/CANCELED 保留原结果；STOP_UNCONFIRMED 需要现场按原流程处置，不重启、不解锁。
需要取消时只调用 cancel_task(task_id)，请求接受不等于已经停车；继续查询终态。
到终态再 finish，如实区分搜索结果和仍待完成的宿主收尾。
