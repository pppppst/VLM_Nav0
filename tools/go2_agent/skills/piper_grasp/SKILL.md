---
name: piper_grasp
description: 抓取宿主指定的物块，抬升并保持闭爪回传感器零点。
---

调用 piper_grasp(target)，逐字传入宿主的目标描述。调用后用 piper_status() 查询。
抓取编号 grasp_id 关联本轮原始反馈。只有 SUCCEEDED 表示视觉确认抬升且六轴已回零；
UNCONFIRMED 表示抓持未确认，不能据此启动后续放置。回零时持物状态没有单独的视觉验收。
同一任务至多一次，遇到未确认或失败时如实 finish。
