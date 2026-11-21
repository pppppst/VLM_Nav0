"""Independent image contours and measured-joint URDF surfaces, in the base frame."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from . import geometry as g
from .vision import intrinsics_valid, rays


def contour_points(mask,depth,camera):
    intrinsics_valid(camera,depth.shape)
    mask=np.asarray(mask,dtype=np.uint8)
    if mask.shape!=depth.shape:raise ValueError('arm mask/depth shape mismatch')
    mask=(mask!=0).astype(np.uint8)
    boundary=(mask!=0)&(cv2.erode(mask,np.ones((3,3),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)==0)
    valid=boundary&np.isfinite(depth)&(depth>0)
    y,x=np.nonzero(valid);pixels=np.column_stack([x,y])
    points=rays(pixels,camera)*(depth[valid]*camera['depth_scale'])[:,None] if len(pixels) else np.empty((0,3))
    return pixels,points,dict(boundary_pixels=int(boundary.sum()),valid_depth_pixels=len(pixels),
        missing_depth_pixels=int(boundary.sum())-len(pixels),method='inner 1-pixel mask edge; measured depth only; no filling/filtering')


def segment_capture(directory,config,stage):
    from .semantic import locate_and_segment
    from .main import save
    directory=Path(directory);metadata=json.loads((directory/'metadata.json').read_text())
    frame=metadata['frames'][-1]
    rgb=cv2.imread(str(directory/(frame['stem']+'-color.png')))
    if rgb is None:raise ValueError('missing arm capture RGB')
    selection_config=copy.deepcopy(config)
    selection_config['target']=dict(color='黑色及金属色',description=
        '画面中可见的机器人机械夹爪或机械臂局部，包括夹指、横梁及露出的连杆。即使整台机械臂不在画面里、只看见部分夹指，也要定位这些可见部分。排除红色方块、纸盒等被抓物和背景。')
    try:
        selection=locate_and_segment(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB),selection_config)
    except RuntimeError as error:
        if str(error)!='requested object not found':raise
        selection=dict(mask=np.zeros(rgb.shape[:2],np.uint8),bbox_px=None,label='arm not detected')
    mask=np.asarray(selection['mask'],dtype=np.uint8)
    visibility='DETECTED' if mask.any() else 'NOT_DETECTED'
    if not cv2.imwrite(str(directory/'arm-mask.png'),mask*255):raise OSError('arm mask write failed')
    save(directory/'arm-observation.json',dict(stage=stage,frame=frame['stem'],visibility=visibility,
        target=selection_config['target'],bbox_px=selection['bbox_px'],label=selection['label'],
        robot_snapshot=frame['paired_robot'][-1]['robot_snapshot'],
        mask_source='independent RGB Qwen+SAM; no FK/model projection input',
        segmentation_validation='UNVERIFIED; inspect RGB mask overlay'))


def origin(node):
    result=np.eye(4)
    if node is not None:
        result[:3,3]=np.fromstring(node.get('xyz','0 0 0'),sep=' ')
        result[:3,:3]=g.rotation_matrix(g.Rotation.from_euler('xyz',np.fromstring(node.get('rpy','0 0 0'),sep=' ')))
    return result


def urdf_transforms(robot,joints_deg,gripper_width_mm):
    q=np.asarray(joints_deg,dtype=float)
    if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError('six finite feedback angles required')
    if not np.isfinite(gripper_width_mm) or gripper_width_mm<0:raise ValueError('invalid gripper width feedback')
    joints=list(robot.findall('joint'));children={j.find('child').get('link') for j in joints}
    roots=[link.get('name') for link in robot.findall('link') if link.get('name') not in children]
    if len(roots)!=1:raise ValueError('URDF requires one root')
    poses={roots[0]:np.eye(4)};pending=joints.copy()
    while pending:
        progress=False
        for joint in pending[:]:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in poses:continue
            motion=np.eye(4);kind=joint.get('type');name=joint.get('name')
            if kind!='fixed':
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                if not np.isclose(np.linalg.norm(axis),1):raise ValueError('invalid URDF axis')
                if kind=='revolute' and name in ['joint'+str(i) for i in range(1,7)]:
                    motion[:3,:3]=g.rotation_matrix(g.Rotation.from_rotvec(axis*np.radians(q[int(name[5:])-1])))
                elif kind=='prismatic' and name in ('joint7','joint8'):
                    motion[:3,3]=axis*gripper_width_mm/2000*(1 if name=='joint7' else -1)
                else:raise ValueError('unsupported Piper joint: '+name)
            poses[child]=poses[parent]@origin(joint.find('origin'))@motion
            pending.remove(joint);progress=True
        if not progress:raise ValueError('disconnected/cyclic URDF')
    base=np.linalg.inv(poses['base_link'])
    return {name:base@pose for name,pose in poses.items()}


def load_model(urdf):
    urdf=Path(urdf);raw=urdf.read_bytes();robot=ET.fromstring(raw);meshes=[]
    provenance=dict(urdf=str(urdf.resolve()),urdf_sha256=hashlib.sha256(raw).hexdigest(),meshes=[])
    normalized=(raw.decode('utf-8').replace('\r\n','\n').rstrip('\n')+'\n').encode('utf-8')
    provenance['urdf_lf_sha256']=hashlib.sha256(normalized).hexdigest()
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            filename=mesh.get('filename')
            if not filename.startswith('package://piper_description/'):
                raise ValueError('unsupported mesh URI: '+filename)
            path=urdf.parent.parent/filename.split('package://piper_description/')[1]
            raw=path.read_bytes()
            if len(raw)<84:raise ValueError('invalid binary STL: '+str(path))
            count=int.from_bytes(raw[80:84],'little')
            if len(raw)!=84+count*50 or count==0:raise ValueError('expected nonempty binary STL: '+str(path))
            triangles=np.frombuffer(raw,offset=84,dtype=[('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])['vertices'].astype(float)
            scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            if scale.shape!=(3,) or not np.all(np.isfinite(triangles)) or not np.all(np.isfinite(scale)):
                raise ValueError('invalid mesh geometry')
            transform=origin(visual.find('origin'))
            vertices,indices=np.unique((triangles*scale).reshape(-1,3),axis=0,return_inverse=True)
            vertices=vertices@transform[:3,:3].T+transform[:3,3]
            meshes.append((link.get('name'),vertices,indices.reshape(-1,3)))
            provenance['meshes'].append(dict(link=link.get('name'),path=str(path),sha256=hashlib.sha256(raw).hexdigest(),triangles=count))
    if not meshes:raise ValueError('URDF contains no mesh surfaces')
    return robot,meshes,provenance


def render(source,output,config,urdf):
    """Read saved captures only; no model service, camera, or robot access."""
    import plotly.graph_objects as go
    from .cloud_report import depth_points,write_ply
    paths=sorted(Path(source).glob('arm-*/arm-observation.json'),key=lambda p:
        json.loads((p.parent/'metadata.json').read_text())['frames'][0]['host_unix'])
    if not paths:return None
    robot,meshes,provenance=load_model(urdf)
    if config['kinematics']['source_sha256'] not in (provenance['urdf_sha256'],provenance['urdf_lf_sha256']):
        raise ValueError('outline URDF differs from frozen kinematic model')
    ext=g.transform(config['extrinsics']['T_base_color_optical'])
    figure=go.Figure();groups=[];records=[]
    for index,path in enumerate(paths):
        directory=path.parent;record=json.loads(path.read_text());metadata=json.loads((directory/'metadata.json').read_text())
        frame=next(f for f in metadata['frames'] if f['stem']==record['frame'])
        camera=dict(metadata['color_intrinsics'],depth_scale=metadata['depth_scale'],aligned_intrinsics=frame['aligned_intrinsics'])
        mask=cv2.imread(str(directory/'arm-mask.png'),cv2.IMREAD_GRAYSCALE)
        rgb=cv2.imread(str(directory/(frame['stem']+'-color.png')))
        if mask is None or rgb is None:raise ValueError('missing saved arm mask/RGB')
        depth=np.load(directory/(frame['stem']+'-depth.npz'),allow_pickle=False)['aligned_depth']
        pixels,points,stats=contour_points(mask,depth,camera)
        world=points@ext[:3,:3].T+ext[:3,3]
        stem='arm-pose-%02d'%index
        write_ply(Path(output)/(stem+'-contour.ply'),world,np.tile([255,80,20],(len(world),1)))
        np.savez_compressed(Path(output)/(stem+'-contour.npz'),pixels=pixels,points_color_m=points,points_base_m=world)
        overlay=rgb.copy();overlay[mask!=0]=(overlay[mask!=0]*.7+np.array([0,180,255])*.3).astype(np.uint8)
        contours,_=cv2.findContours((mask!=0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay,contours,-1,(0,80,255),1)
        overlay[pixels[:,1],pixels[:,0]]=[0,255,0]
        if not cv2.imwrite(str(Path(output)/(stem+'-rgb.png')),overlay):raise OSError('outline overlay write failed')
        snapshot=record['robot_snapshot']
        poses=urdf_transforms(robot,snapshot['joints_deg'],snapshot['gripper_width_mm'])
        group=[]
        def add(trace):
            group.append(len(figure.data));trace.visible=index==0;figure.add_trace(trace)
        for name,vertices,faces in meshes:
            vertices=vertices@poses[name][:3,:3].T+poses[name][:3,3]
            add(go.Mesh3d(x=vertices[:,0],y=vertices[:,1],z=vertices[:,2],i=faces[:,0],j=faces[:,1],k=faces[:,2],
                name='FK model / '+name,color='#408ac9',opacity=.3,flatshading=True))
        _,scene=depth_points(depth,camera);scene=scene@ext[:3,:3].T+ext[:3,3]
        scene_count=len(scene)
        scene=scene[np.linalg.norm(scene,axis=1)<=1.0]
        nearby_count=len(scene)
        scene=scene[::max(1,int(np.ceil(len(scene)/5000)))]
        add(go.Scatter3d(x=scene[:,0],y=scene[:,1],z=scene[:,2],mode='markers',name='Observed scene (sampled)',
            marker=dict(color='#aaaaaa',size=1,opacity=.2)))
        add(go.Scatter3d(x=[0],y=[0],z=[0],mode='markers+text',name='Piper base origin',
            text=['Piper base (0, 0, 0)'],textposition='top center',marker=dict(color='black',size=5)))
        groups.append(group)
        records.append(dict(stage=record['stage'],capture=directory.name,frame=frame['stem'],host_unix=frame['host_unix'],
            visibility=record.get('visibility','DETECTED'),
            processing=record.get('processing','capture-time image segmentation'),
            robot_pairing=frame.get('robot_pairing'),
            scene_points=dict(valid=scene_count,within_1m=nearby_count,displayed=len(scene)),
            joints_deg=snapshot['joints_deg'],gripper_width_mm=snapshot['gripper_width_mm'],depth=stats,
            stem=stem,segmentation_validation=record['segmentation_validation']))
    buttons=[dict(label=r['stage'],method='update',args=[dict(visible=[j in group for j in range(len(figure.data))])])
             for r,group in zip(records,groups)]
    figure.update_layout(title='Observed depth cloud vs measured-joint Piper model (base frame)',
        scene=dict(xaxis=dict(title='Base X / m',range=[-1,1]),
                   yaxis=dict(title='Base Y / m',range=[-1,1]),
                   zaxis=dict(title='Base Z / m',range=[-1,1]),aspectmode='cube'),
        updatemenus=[dict(buttons=buttons,direction='down',x=0,y=1.12)],height=800)
    figure.write_html(str(Path(output)/'arm-overlay.html'),include_plotlyjs=True)
    summary=dict(poses=records,model=provenance,T_base_color_optical=ext.tolist(),
        display_frame='piper_base',display_origin_m=[0,0,0],scene_radius_m=1.0,
        scope='visible image contour depth vs URDF surfaces; segmentation/TCP/extrinsics not physically validated',
        gripper_model='joint7=width/2; joint8=-width/2 in metres; measured opening, not calibrated finger deformation')
    (Path(output)/'arm-overlay.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary
