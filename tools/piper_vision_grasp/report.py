"""Offline RGB/3D/transform/joint reports from frozen grasp evidence, without I/O to hardware."""
import csv
import html
import json
from pathlib import Path

import cv2
import numpy as np

from . import geometry as g
from .cloud_report import depth_points, render_clouds
from .semantic import target_points


def project_base(points, extrinsic, camera):
    ext=g.transform(extrinsic)
    optical=(np.asarray(points)-ext[:3,3])@ext[:3,:3]
    matrix=np.array([[camera['fx'],0,camera['ppx']],[0,camera['fy'],camera['ppy']],[0,0,1.]])
    result=[]
    for point in optical:
        if not np.all(np.isfinite(point)) or point[2]<=0:
            result.append(None);continue
        uv,_=cv2.projectPoints(point.reshape(1,3),np.zeros(3),np.zeros(3),matrix,np.asarray(camera['coeffs'],dtype=float))
        result.append(uv.reshape(2).tolist())
    return result


def joint_comparison(plan, events):
    rows=[]
    for stage in plan.get('stages',[]):
        related=[e for e in events if e.get('stage')==stage['stage_id']]
        samples=[e for e in related if e.get('state') in ('AT_TARGET','WAIT_TARGET')
                 and (e.get('measured') or {}).get('joints_deg') is not None
                 and not e['measured'].get('missing_feedback')]
        last=samples[-1] if samples else None
        actual=last['measured']['joints_deg'] if last else None
        target=stage['angles_deg']
        rows.append(dict(stage=stage['stage_id'],target_deg=target,actual_deg=actual,
                         error_deg=(np.asarray(actual)-target).tolist() if actual is not None else None,
                         state=related[-1]['state'] if related else 'NOT_EXECUTED',
                         sample_state=last['state'] if last else None,
                         time=last['time'] if last else None))
    return rows


def link_points(angles, model, tcp_offset_m=None):
    pose=np.eye(4);points=[pose[:3,3].copy()]
    for angle,joint in zip(g.model_angles(angles,model),model['joints']):
        origin=np.eye(4);origin[:3,3]=joint['xyz_m']
        origin[:3,:3]=g.rotation_matrix(g.Rotation.from_euler('xyz',joint['rpy_rad']))
        motion=np.eye(4)
        motion[:3,:3]=g.rotation_matrix(g.Rotation.from_rotvec(np.asarray(joint['axis'])*np.radians(angle)))
        pose=pose@origin@motion;points.append(pose[:3,3].copy())
    if tcp_offset_m is not None:
        points.append(pose[:3,3]+pose[:3,:3]@np.asarray(tcp_offset_m))
    return np.asarray(points)


def joint_table(rows):
    table=['<table border="1" cellpadding="6"><tr><th>数据</th>'+''.join(
        '<th>J%d (°)</th>'%(j+1) for j in range(6))+'</tr>']
    for label,values in rows:
        cells=('<td colspan="6">无有效反馈</td>' if values is None else
               ''.join('<td>%.3f</td>'%value for value in values))
        table.append('<tr><th>'+html.escape(label)+'</th>'+cells+'</tr>')
    return '\n'.join(table+['</table>'])


def render(config, source, output, urdf=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import plotly.graph_objects as go

    source,output=Path(source),Path(output)
    output.mkdir(parents=True,exist_ok=True)
    read=lambda path:json.loads(path.read_text())
    frozen=read(source/'config_snapshot.json')
    if frozen.get('mode')!='semantic' or frozen.get('grasp_backend','legacy')!='legacy':
        raise ValueError('report requires a saved heuristic semantic run')
    events=[json.loads(line) for line in (source/'events.jsonl').read_text().splitlines()]
    accepted=read(source/'plan.json') if (source/'plan.json').exists() else {}
    comparison=joint_comparison(accepted,events)
    result=read(source/'result.json') if (source/'result.json').exists() else dict(state='NO_FINAL_RESULT')
    summary=dict(result=result,calibration_source='frozen capture/run configuration',
                 physical_validation=frozen['extrinsics'].get('physical_validation','NOT_PERFORMED'),
                 joint_comparison=comparison,captures=[],can_transmit_count=0,last_event=events[-1] if events else None)
    operator=source/'operator-verification.json'
    if operator.exists():summary['operator_verification']=read(operator)
    ext=g.transform(frozen['extrinsics']['T_base_color_optical'])
    transform_data=dict(T_base_color_optical=ext.tolist(),T_color_optical_base=np.linalg.inv(ext).tolist(),
                        equation='p_base = R_base_color_optical @ p_color_optical + t_base_color_optical',
                        translation_unit='m',tcp_offset_m=frozen['semantic']['tcp_offset_m'],
                        source_sha256=frozen['extrinsics']['source_sha256'])
    (output/'transforms.json').write_text(json.dumps(transform_data,indent=2)+'\n')

    fig=plt.figure(figsize=(15,7));ax=fig.add_subplot(121,projection='3d')
    for label,pose in [('Piper base',np.eye(4)),('Camera optical',ext)]:
        for i,color in enumerate(('r','g','b')):
            pts=np.array([pose[:3,3],pose[:3,3]+pose[:3,i]*.08])
            ax.plot(*pts.T,color=color);ax.text(*pts[-1],label+' '+'XYZ'[i],fontsize=8)
    ax.plot(*np.array([[0,0,0],ext[:3,3]]).T,'k--',label='camera translation')
    ax.set(xlabel='Base X / m',ylabel='Base Y / m',zlabel='Base Z / m',title='Axes: X red, Y green, Z blue')
    ax.set_box_aspect([np.ptp(ax.get_xlim()),np.ptp(ax.get_ylim()),np.ptp(ax.get_zlim())]);ax.view_init(24,-65)
    text=fig.add_subplot(122);text.axis('off')
    text.text(0,.9,'p_base = R @ p_camera + t\n\nT_base_color_optical (m):\n'+np.array2string(ext,precision=6)+
              '\n\nT_color_optical_base (m):\n'+np.array2string(np.linalg.inv(ext),precision=6)+
              '\n\nTCP offset (tool m): '+str(frozen['semantic']['tcp_offset_m'])+
              '\nCalibration/TCP physical accuracy unverified.',va='top',family='monospace',fontsize=9)
    ax.tick_params(labelsize=8)
    fig.subplots_adjust(left=.03,right=.98,bottom=.15,top=.88,wspace=.35)
    fig.savefig(output/'transforms.png',dpi=150);plt.close(fig)

    plans=sorted(source.glob('before-*/planning.json'),
                 key=lambda p:read(p.parent/'observation.json')['metadata']['frames'][0]['host_unix'])
    for index,path in enumerate(plans):
        record=read(path);cfg=record['config'];plan=record['plan'];directory=path.parent
        saved=read(directory/'observation.json');meta=saved['metadata'];frame=meta['frames'][-1]
        camera=dict(meta['color_intrinsics'],depth_scale=meta['depth_scale'],aligned_intrinsics=frame['aligned_intrinsics'])
        image=cv2.imread(str(directory/(frame['stem']+'-color.png')))
        if image is None:raise ValueError('missing RGB image: '+str(directory))
        depth=np.load(directory/(frame['stem']+'-depth.npz'),allow_pickle=False)['aligned_depth']
        mask=cv2.imread(str(directory/('mask%02d.png'%(len(meta['frames'])-1))),cv2.IMREAD_GRAYSCALE)
        if mask is None:raise ValueError('missing target mask: '+str(directory))
        _,cloud=depth_points(depth,camera)
        _,object_cloud,_=target_points(depth,camera,mask!=0)
        pose=g.transform(cfg['extrinsics']['T_base_color_optical'])
        cloud=cloud@pose[:3,:3].T+pose[:3,3]
        object_cloud=object_cloud@pose[:3,:3].T+pose[:3,3]
        obs=saved['observation'];obj=pose[:3,:3]@obs['point_color_m']+pose[:3,3]
        target=plan.get('target') or g.absolute_target(obs,cfg)
        tcp=np.asarray(target['T_base_tcp'])[:3,3]
        executed_capture=Path(accepted.get('capture_dir','')).name==directory.name
        actual=next((r for r in comparison if r['stage']=='grasp' and r['actual_deg'] is not None),None) if executed_capture else None
        feedback=None
        if actual:
            actual_links=link_points(actual['actual_deg'],cfg['kinematics'],cfg['semantic']['tcp_offset_m'])
            actual_tcp=actual_links[-1]
            feedback=dict(tcp_base_m=actual_tcp.tolist(),delta_to_candidate_m=(actual_tcp-tcp).tolist(),
                joints_deg=actual['actual_deg'],sample_state=actual['sample_state'],time=actual['time'],
                label='Grasp feedback TCP (FK, '+actual['sample_state']+')')
        pre=tcp+np.asarray(target.get('pregrasp_offset_base_m',cfg['semantic'].get('pregrasp_offset_base_m',
            -np.asarray(target['T_base_tcp'])[:3,2]*cfg['semantic']['pregrasp_distance_m'])))
        positions=np.array([obj,tcp,pre]);names=['Object visible center','Candidate grasp TCP','Pregrasp TCP']
        colors=['#e53935','#00a65a','#1565c0'];bgr=[(40,40,230),(30,180,0),(230,100,0)]
        pixels=project_base(positions,pose,camera)
        overlay=image.copy();overlay[mask!=0]=(overlay[mask!=0]*.65+np.array([40,40,230])*.35).astype(np.uint8)
        h,w=image.shape[:2]
        for n,(name,pixel,color) in enumerate(zip(names,pixels,bgr)):
            valid=pixel is not None and 0<=pixel[0]<w and 0<=pixel[1]<h
            label=name if valid else name+' (outside RGB view)'
            y=24+n*25
            cv2.putText(overlay,label,(12,y),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1,cv2.LINE_AA)
            if valid:
                xy=tuple(np.rint(pixel).astype(int))
                cv2.circle(overlay,xy,6+5*n,color,2)
                cv2.line(overlay,(260,y-5),xy,color,1)
        cv2.putText(overlay,plan['status'],(12,h-14),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
        stem='candidate-%02d'%index
        clouds=render_clouds(directory,saved,pose,positions,output,stem,feedback)
        if not cv2.imwrite(str(output/(stem+'-rgb.png')),overlay):raise OSError('RGB overlay write failed')
        figure=go.Figure();static=plt.figure(figsize=(11,8));ax=static.add_subplot(111,projection='3d',computed_zorder=False)
        def scatter(name,pts,color,size,symbol='circle'):
            figure.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='markers',name=name,marker=dict(size=size,color=color,symbol=symbol)))
            if symbol=='circle-open':ax.scatter(*pts.T,s=size*12,facecolors='none',edgecolors=color,label=name,zorder=8)
            else:ax.scatter(*pts.T,s=size*4,c=color,label=name,alpha=.6,marker={'cross':'x','diamond':'D'}.get(symbol,'o'),zorder=1 if size==1 else 7)
        def line(name,pts,color):
            figure.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='lines+markers',name=name,line=dict(color=color,width=5)))
            ax.plot(*pts.T,color=color,label=name,linewidth=2)
        scatter('Scene observed points',cloud[::max(1,len(cloud)//7000)],'#bbbbbb',1)
        scatter('Detected object surface',object_cloud[::max(1,len(object_cloud)//1500)],'#e53935',2)
        for n,(name,pt,color) in enumerate(zip(names,positions,colors)):
            scatter(name,pt[None],color,9 if n==0 else 6,('circle-open','cross','circle')[n])
        line('Approach',np.array([pre,tcp]),'#00a65a')
        for label,axes in [('Base',np.eye(4)),('Camera',pose),('Grasp TCP',np.asarray(target['T_base_tcp']))]:
            for i,color in enumerate(('#dd3333','#33aa44','#3355dd')):
                line(label+' '+'XYZ'[i],np.array([axes[:3,3],axes[:3,3]+axes[:3,i]*.06]),color)
        initial=link_points(record['initial_joints_deg'],cfg['kinematics'])
        line('Robot at capture (feedback)',initial,'#555555')
        grasp=next((s for s in plan.get('stages',[]) if s['stage_id']=='grasp'),None)
        if grasp:line('IK grasp joint positions',link_points(grasp['angles_deg'],cfg['kinematics']),'#9c27b0')
        if feedback:
            line('Grasp feedback links and TCP',actual_links,'#ff9800')
            scatter(feedback['label'],actual_tcp[None],'#ff9800',10,'diamond')
            line('Candidate to feedback TCP',np.array([tcp,actual_tcp]),'#ff9800')
            ax.text(*actual_tcp,'  FK TCP',color='#b35b00',fontsize=8)
        ik=grasp['angles_deg'] if grasp else (plan.get('failed') or {}).get('angles_deg')
        focus=np.vstack([object_cloud,positions,initial,pose[:3,3][None]])
        if feedback:focus=np.vstack([focus,actual_links])
        lo,hi=focus.min(axis=0)-.08,focus.max(axis=0)+.08
        figure.update_layout(title=directory.name+' / '+plan['status']+' (saved evidence; clearance unverified)',scene=dict(
            xaxis=dict(title='Base X / m',range=[lo[0],hi[0]]),yaxis=dict(title='Base Y / m',range=[lo[1],hi[1]]),
            zaxis=dict(title='Base Z / m',range=[lo[2],hi[2]]),aspectmode='data'))
        figure.write_html(str(output/(stem+'-3d.html')),include_plotlyjs=True)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='Base X / m',ylabel='Base Y / m',zlabel='Base Z / m',title=directory.name+' / '+plan['status'])
        from matplotlib.ticker import MaxNLocator
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_major_locator(MaxNLocator(4))
        ax.set_box_aspect(hi-lo);ax.view_init(24,-65);ax.legend(fontsize=7,loc='upper left')
        static.tight_layout();static.savefig(output/(stem+'-3d.png'),dpi=140);plt.close(static)
        summary['captures'].append(dict(capture=directory.name,status=plan['status'],object_base_m=obj.tolist(),
            grasp_tcp_base_m=tcp.tolist(),pregrasp_tcp_base_m=pre.tolist(),rgb_pixels=pixels,stem=stem,
            ik_candidate_deg=ik,captured_actual_deg=record['initial_joints_deg'],
            actual_reached_deg=actual['actual_deg'] if actual and actual['sample_state']=='AT_TARGET' else None,
            grasp_feedback_fk=feedback,
            ik_failure=plan.get('failed'),
            execution_plan=executed_capture,source_sha256=cfg['extrinsics']['source_sha256'],point_clouds=clouds,
            orientation_candidates=plan.get('orientation_candidates',[]),selected_candidate_index=plan.get('selected_candidate_index'),
            ik_position_tolerance_mm=1000*cfg['semantic'].get('ik_position_tolerance_m',.001)))

    with (output/'joints.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['stage','state','joint','target_deg','actual_deg','error_deg'])
        for row in comparison:
            for j in range(6):writer.writerow([row['stage'],row['state'],j+1,row['target_deg'][j],
                row['actual_deg'][j] if row['actual_deg'] is not None else '',row['error_deg'][j] if row['error_deg'] is not None else ''])
    from .arm_outline import render as render_arm_outline
    arm_overlay=render_arm_outline(source,output,frozen,urdf)
    if arm_overlay is not None:summary['arm_overlay']=arm_overlay
    (output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    page=['<!doctype html><meta charset="utf-8"><title>Piper 抓取证据</title>',
          '<style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}iframe{width:100%;height:700px;border:0}pre{white-space:pre-wrap}</style>',
          '<h1>'+html.escape(' '.join(frozen['target'].values()))+'夹取：中间结果</h1><p>实际结果：'+html.escape(result['state'])+'</p>',
          '<p>最后记录状态：'+html.escape(events[-1]['state'] if events else 'NO_EVENTS')+'</p>',
          '<p>红色：识别到的物块；绿色：候选抓取TCP；蓝色：预抓TCP。RGB重合点用同心圈表示；3D物块中心用空心圈，抓取点用十字。机械臂以运动学连杆线显示；不是碰撞验证。</p>',
          '<p>使用采集时冻结的坐标变换。标定/TCP物理精度尚未验证。没有执行反馈的阶段保持空值。</p>']
    page.append('<p>空间图橙色菱形：抓取阶段最后有效实测关节角经FK并加工具坐标系TCP偏移得到的夹爪位置。场景图橙色连线连接候选与反馈TCP。该点来自历史抓取反馈，不是当前零位，也不是独立实测空间坐标；WAIT_TARGET不表示到位。无有效反馈不显示该点。</p>')
    if 'operator_verification' in summary:
        page.append('<h2>现场人工反馈（独立于程序状态）</h2><pre>'+html.escape(json.dumps(summary['operator_verification'],ensure_ascii=False,indent=2))+'</pre>')
    if arm_overlay is not None:
        page.extend(['<h2>四姿态：相机深度点云与反馈FK机械臂外形</h2>',
            '<p>原点为 Piper 基座原点（黑色标记），灰色点仅保留距基座原点不超过 1 米的深度场景点（先按三维距离过滤，再抽样显示）；蓝色半透明表面为实测关节角和夹爪开口驱动的Piper模型。空间图已去除橙色掩膜轮廓点，可直接观察点云形状。下拉菜单切换姿势，均在机械臂基座系（米）。无效深度不补点，外参仍需现场核验；原始掩膜与轮廓数据保留供追溯。</p>',
            '<p><a href="arm-overlay.html">独立打开三维叠加</a> · <a href="arm-overlay.json">四姿态关节/深度覆盖率/模型哈希</a></p>',
            '<iframe loading="lazy" src="arm-overlay.html"></iframe>'])
        for row in arm_overlay['poses']:
            if row['visibility']=='NOT_DETECTED':
                page.append('<p>本姿态未识别到可用机械臂轮廓，只保留图像、关节反馈及模型；没有相机轮廓点，不代表两者已对齐。</p>')
            page.append('<h3>'+html.escape(row['stage'])+'</h3><p>夹爪开口 %.3f mm；有效轮廓深度 %d / %d 像素。RGB中橙线是分割边界，绿色点是有有效深度的边界像素。</p>'%
                (row['gripper_width_mm'],row['depth']['valid_depth_pixels'],row['depth']['boundary_pixels']))
            page.append(joint_table([('拍摄时实测',row['joints_deg'])]))
            page.append('<img loading="lazy" src="'+row['stem']+'-rgb.png"><p><a href="'+row['stem']+'-contour.ply">基座系轮廓点PLY</a> · <a href="'+row['stem']+'-contour.npz">像素与相机/基座坐标</a></p>')
    for capture in summary['captures']:
        stem=capture['stem']
        page.append('<p>本捕获的IK位置通过阈值：'+str(capture['ik_position_tolerance_mm'])+' mm；姿态阈值：1°。通过不表示零误差。</p>')
        if capture['orientation_candidates']:
            page.extend(['<h2>夹爪角度搜索：'+html.escape(capture['capture'])+'</h2>',
                '<p>按顺序检查完整路径，首个通过即停止。俯仰0°为水平、90°为向下；水平偏角相对深度几何估计方向。数值可行不代表已验证实物碰撞或隐藏厚度。</p>',
                '<table border="1" cellpadding="6"><tr><th>序号</th><th>俯仰°</th><th>水平偏角°</th><th>结果</th><th>位置残差mm</th><th>原因</th></tr>'])
            for candidate in capture['orientation_candidates']:
                failure=candidate.get('failed') or {};error=failure.get('position_error_m')
                values=[candidate['index']+1,candidate['pitch_deg'],candidate['yaw_offset_deg'],
                        '已选择' if candidate['index']==capture['selected_candidate_index'] else candidate['status'],
                        '—' if error is None else round(error*1000,3),candidate['reason']]
                page.append('<tr>'+''.join('<td>'+html.escape(str(value))+'</td>' for value in values)+'</tr>')
            page.append('</table>')
        page.extend(['<h2>'+html.escape(capture['capture']+' / '+capture['status'])+'</h2>',
            '<img loading="lazy" src="'+stem+'-rgb.png"><img loading="lazy" src="'+stem+'-3d.png">','<iframe loading="lazy" src="'+stem+'-3d.html"></iframe>'])
        feedback=capture['grasp_feedback_fk']
        if feedback:
            page.append('<p>反馈FK TCP（基座系，mm）：'+', '.join('%.3f'%(v*1000) for v in feedback['tcp_base_m'])+
                '；相对候选点偏差 ΔX/ΔY/ΔZ（mm）：'+', '.join('%+.3f'%(v*1000) for v in feedback['delta_to_candidate_m'])+
                '。反馈状态：'+html.escape(feedback['sample_state'])+'</p>')
        page.extend(['<h3>深度场景点云与启发式候选输入点云</h3>',
            '<p>左：相机全部有效深度形成的RGB点云（仅显示抽样，不裁剪深度范围）；右：实际用于启发式几何计算的目标点云（分割后按深度IQR过滤，全部显示）。下拉菜单切换每帧输入；标记使用多帧合成的物块中心与候选点。PLY保留全部有效点，坐标为机械臂基座系，单位米。静态预览使用最后一帧。</p>',
            '<img loading="lazy" src="'+stem+'-clouds.png">',
            '<p><a href="'+stem+'-clouds.html">独立交互点云（旋转/缩放/切换帧）</a> · <a href="'+stem+'-clouds.json">点数、内参、变换与过滤说明</a></p>',
            '<iframe loading="lazy" src="'+stem+'-clouds.html"></iframe>'])
        for frame in capture['point_clouds']['frames']:
            page.append('<p>'+html.escape(frame['stem'])+': <a href="'+frame['scene_ply']+'">场景PLY（'+str(frame['scene_count'])+'点）</a> · <a href="'+frame['target_ply']+'">启发式输入PLY（'+str(frame['target_count'])+'点）</a></p>')
        if capture['ik_candidate_deg'] is not None:
            page.append('<h3>候选关节角（度）</h3>')
            page.append(joint_table([
                ('候选IK（未下发）' if capture['ik_failure'] else '抓取IK目标',capture['ik_candidate_deg']),
                ('采图时反馈（不是到位结果）',capture['captured_actual_deg']),
                ('抓取到位反馈',capture['actual_reached_deg'])]))
    page.extend(['<h2>相机与机械臂坐标变换</h2><img loading="lazy" src="transforms.png"><a href="transforms.json">变换矩阵数据</a>',
        '<h2>逐阶段目标与实测关节角（度）</h2><p>误差＝实测−目标。反馈状态为 AT_TARGET 才表示到位；无反馈不记作零。</p><p><a href="joints.csv">逐段角度与误差CSV</a> · <a href="summary.json">全部摘要JSON</a></p>'])
    for row in comparison:
        page.append('<h3>'+html.escape(row['stage']+' / '+row['state'])+'</h3><p>反馈状态：'+
                    html.escape(row['sample_state'] or '无有效反馈')+'</p>')
        page.append(joint_table([('目标',row['target_deg']),('最后有效实测',row['actual_deg']),
                                 ('误差（实测−目标）',row['error_deg'])]))
    if not comparison:page.append('<p>没有可执行关节计划或反馈；未补造实际到达结果。</p>')
    (output/'index.html').write_text('\n'.join(page))
    return dict(state='VISUALIZED',offline=True,can_transmit_count=0,html=str(output/'index.html'),
                captures=len(plans),arm_poses=len(arm_overlay['poses']) if arm_overlay is not None else 0)
