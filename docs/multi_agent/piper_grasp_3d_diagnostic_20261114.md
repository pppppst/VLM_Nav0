# Piper 视觉抓取三维排障图（2026-11-14）

本轮用户将任务转为三维可视化排障。机械臂保持右移后的姿态，夹爪约68.3mm张开，未继续闭爪/抓取。为同一时段的几何对照，有限采集3帧RGB-D及前后被动关节反馈，CAN发送0；绘图和后续分析全部离线。

## 图中内容

- [可独立打开的交互3D图](../../build/piper-diagnostic3d-20261114/diagnostic-standalone.html)：拖动旋转、滚轮缩放，点击图例切换图层；悬停点可查看毫米坐标。
- [静态PNG](../../build/piper-diagnostic3d-20261114/diagnostic3d.png) / [PDF](../../build/piper-diagnostic3d-20261114/diagnostic3d.pdf)。
- [坐标与图层数据](../../build/piper-diagnostic3d-20261114/scene.json)、[来源、参数及反馈](../../build/piper-diagnostic3d-20261114/evidence.json)、[数值差异](../../build/piper-diagnostic3d-20261114/findings.json)。

所有图层使用Piper基座坐标、单位mm；不同采样时刻明确分层：

1. 新三帧的VLM原建议像素，用该像素实际深度反投影得到V1/V2/V3。三帧均有原像素深度，未采用深度填补。
2. 新三帧的SAM分割点云、程序实际采用的中心S1/S2/S3、三条夹取方向。与运行代码相同的IQR深度过滤后绘制，显示点云降采样，不改变目标坐标。
3. 目标形状用可见深度点的外包络表示；隐藏面、厚度未推测，没有把平面观测伪装成完整实心方块。
4. 抓取前的历史SAM表面、历史程序TCP/法兰目标与抓空关节链，分别保留采样来源；它们不是当前测量，不拼接成同一静态场景的真值。
5. 当前六关节反馈对应的URDF/STL机械臂与张开夹爪。模型来自本机`/home/isee-pst/unitree_ros2/piper/agent/piper_pinocchio/piper_description/urdf/piper_description.urdf`；逐关节解析origin/axis和关节角，法兰矩阵与当前geometry.fk逐项一致。显示时对连杆8mm、夹爪3mm网格合并顶点，保留16569个三角面；只是显示简化，不用于碰撞判断。
6. 当前与抓空时由反馈关节+135mm工具轴偏移计算的TCP、人工调整差向量、Piper基座/相机光学/当前法兰坐标轴。
7. 可展开的稀疏场景RGB-D点云，提供周围物体背景；不作为全场景重建或碰撞地图。

## 已确认差异

新的采样`red-diagnostic3d-20261114-01`中，三帧均独立DETECTED，但合并结果为WAIT_ORIENTATION，不是可执行抓取观测。三帧SAM中心最大距离约1.03mm，夹取轴最大夹角约79.11°；前两帧轴接近基座-X，第三帧接近+Y。因此本次主要跨帧不一致发生在夹取方向，不能把全部等待归因于物体中心抖动。

VLM建议点与SAM中心在这三帧中的高度差分别约0.03、0.44、-0.01mm。它们在本次样本中的高度基本一致，不能用二者选择差异解释约13cm高度问题。

当前模型TCP约[649.25,-9.35,-4.56]mm，新的SAM中心中位数约[707.81,9.31,-139.61]mm，差约[-58.56,-18.66,+135.05]mm。该TCP是**按试验135mm偏移推算**，不是独立测得的真实夹持中心。用户此前确认上抬130mm后高度对了，之后右移仅改变Y；因此现场观察与当前坐标/工具模型存在待解释差异。135mm差值接近试验工具长度只能作为线索，不能直接认定重复TCP补偿、外参错误或某一个根因。

当前SAM目标比抓取前历史目标在X上更远，图中保留两次表面以显示时间差；没有确认物体是否被推移或其他因素变化，不能把历史与当前差值直接写成固定校正量。外参physical_validation仍NOT_PERFORMED，URDF夹爪几何与实物版本、实际TCP、相机安装/零位对应仍需核对。

## 验证和复现

- 新采集前后关节变化小于原0.3°判据，CAN发送0；真实反馈、RGB-D、mask、观察记录均已回传。
- URDF法兰变换与控制代码FK一致；所有坐标有限，三角形索引合法，输入文件/网格SHA256已保存。
- Plotly数据结构校验、内联JavaScript语法检查通过；CDN固定版本3.7.0返回HTTP200；内联内容小于1MB。
- 已渲染并查看静态图。当前环境未找到可用浏览器，交互图未做实际浏览器旋转/图层点击测试，不声称已验证浏览器渲染。
- 未改产品源码、运行配置、SDK或导航环境，未构建/部署产品，未提交。

在仓库根目录仅离线复算：

```bash
/home/isee-pst/venv/co-nav-real/bin/python build/piper-diagnostic3d-20261114/build_scene.py
MPLCONFIGDIR=/tmp/piper-diagnostic3d-mpl /usr/bin/python3 build/piper-diagnostic3d-20261114/render_static.py
```

脚本读取已保存的RGB-D/反馈及本地URDF/STL，不连接相机、CAN或云模型。内联模板位于同一输出目录；独立交互HTML已内嵌Plotly库，可离线打开。build证据不受Git保护，不清理。
