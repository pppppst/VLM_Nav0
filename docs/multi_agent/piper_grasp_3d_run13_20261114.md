# Piper 第13轮抓取结果三维图

用户要求将最新结果可视化。已退出 `semantic-red-grasp-20261114-13` 的闭爪等待并核实没有活动抓取进程；没有发送新的移动、开爪、闭爪或回零指令。本次绘图完全读取保存的数据，保留旧图。

## 文件

- [静态 PNG：整体及夹指局部](../../build/piper-diagnostic3d-20261114-run13/diagnostic3d.png)
- [离线交互 3D](../../build/piper-diagnostic3d-20261114-run13/diagnostic-standalone.html)：内嵌 Plotly 3.7.0，拖动旋转、缩放、图例切换图层。
- [PDF](../../build/piper-diagnostic3d-20261114-run13/diagnostic3d.pdf)
- [坐标与图层](../../build/piper-diagnostic3d-20261114-run13/scene.json)、[来源与反馈](../../build/piper-diagnostic3d-20261114-run13/evidence.json)、[数值核对](../../build/piper-diagnostic3d-20261114-run13/findings.json)。

图内统一采用 Piper 基座坐标，单位 mm。显示本轮抓取前最后一组有效 RGB-D 的三帧 SAM 点云/中心/夹取轴、VLM 建议点、可见表面外包络；闭爪后最后反馈关节对应的 URDF/STL 机械臂和48.7mm夹爪；程序 TCP/法兰目标、接近路径，以及最新相机坐标轴。点云与闭爪后反馈相隔约85.1秒，明确是先后采样的同轮结果，不声称是闭爪后的实时目标观测。隐藏目标几何不补造。

## 最新参数与结果

相机位置 `[415.9578,-57.0989,54.6799]` mm；光轴Z+与Piper X+同向。相对原manual04累计基座Z+180mm、Y−110mm并施加此前用户选择的最小旋转。使用本次冻结的 `config-resolved.json` 和 `extrinsics.json`，不随未来配置变动重解释历史。

程序目标TCP `[617.958,-82.790,16.423]` mm；实际反馈关节计算的模型TCP `[617.778,-83.256,16.552]` mm，差值 `[-0.180,-0.466,+0.129]` mm，距离约0.516mm。三帧SAM中心最大间距约1.026mm，VLM点与SAM中心的Z差不超过0.336mm。

这说明本轮软件目标与反馈模型基本一致，**不证明真实TCP或外参准确，也不证明夹住**。TCP仍使用135mm试验偏移，URDF外形未独立实物核验。闭爪目标47.4769mm，反馈48.7mm，状态WAIT_CONTACT；没有lift或抓取成功确认。

## 验证

- URDF法兰与控制FK逐元素一致；点坐标有限、三角面索引合法、Plotly schema通过。
- 内联图约485KB；离线文件约6.1MB，无外部脚本依赖，所有内联脚本通过Node语法检查。
- 静态PNG已查看并修正局部裁切与标注重叠。当前没有可用浏览器，未实测浏览器旋转/图例交互。
- 无相机/CAN新增采样或发送，未改产品代码/参数，未因此构建或提交。

仓库根目录离线重算：

```bash
/home/isee-pst/venv/co-nav-real/bin/python build/piper-diagnostic3d-20261114-run13/build_scene.py
MPLCONFIGDIR=/tmp/piper-diagnostic3d-mpl /usr/bin/python3 build/piper-diagnostic3d-20261114-run13/render_static.py
```

旧图仍位于 `build/piper-diagnostic3d-20261114/`，对应旧外参及更早姿态，不能代替本轮结果。
