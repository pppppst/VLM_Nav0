"""Offline full RGB-D scene and exact heuristic-input point clouds."""
import json

import cv2
import numpy as np

from .semantic import target_points
from .vision import intrinsics_valid, rays


def depth_points(depth,camera):
    intrinsics_valid(camera,depth.shape)
    valid=np.isfinite(depth)&(depth>0)
    y,x=np.nonzero(valid)
    pixels=np.column_stack([x,y])
    return pixels,rays(pixels,camera)*(depth[valid].astype(float)*camera['depth_scale'])[:,None]


def write_ply(path,points,colors):
    data=np.empty(len(points),dtype=[('xyz','<f8',(3,)),('rgb','u1',(3,))])
    data['xyz']=points;data['rgb']=colors
    header=('ply\nformat binary_little_endian 1.0\ncomment frame piper_base; units m\n'
            'element vertex %d\nproperty double x\nproperty double y\nproperty double z\n'
            'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')%len(points)
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'));stream.write(data.tobytes())


def render_clouds(directory,saved,pose,positions,output,stem,feedback=None):
    import matplotlib.pyplot as plt
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    figure=make_subplots(rows=1,cols=2,specs=[[{'type':'scene'},{'type':'scene'}]],
                        subplot_titles=['Full RGB-D scene (display sampled)','Heuristic input (all target points)'])
    metadata=dict(frame='piper_base',units='m',source_capture=directory.name,
                  T_base_color_optical=pose.tolist(),depth_scale=saved['metadata']['depth_scale'],
                  target_filter='SAM mask & finite positive depth; depth IQR +/- 1.5*max(IQR,depth_scale)',
                  fusion='Per-frame target geometry; candidate uses saved multi-frame observation',frames=[],
                  grasp_feedback_fk=feedback)
    names=['Object visible center (combined)','Candidate grasp TCP','Pregrasp TCP']
    colors=['#e53935','#00a65a','#1565c0']
    if feedback:
        names.append(feedback['label']);colors.append('#ff9800')
        positions=np.vstack([positions,feedback['tcp_base_m']])
    buttons=[];groups=[]
    for i,frame in enumerate(saved['metadata']['frames']):
        camera=dict(saved['metadata']['color_intrinsics'],depth_scale=metadata['depth_scale'],
                    aligned_intrinsics=frame['aligned_intrinsics'])
        rgb=cv2.imread(str(directory/(frame['stem']+'-color.png')))
        mask=cv2.imread(str(directory/('mask%02d.png'%i)),cv2.IMREAD_GRAYSCALE)
        depth=np.load(directory/(frame['stem']+'-depth.npz'),allow_pickle=False)['aligned_depth']
        if rgb is None or mask is None:raise ValueError('missing RGB/mask: '+str(directory))
        if rgb.shape[:2]!=depth.shape:raise ValueError('RGB/depth shape mismatch')
        rgb=cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
        pixels,scene=depth_points(depth,camera)
        target_pixels,target,valid=target_points(depth,camera,mask!=0)
        row=dict(stem=frame['stem'],host_unix=frame['host_unix'],
                 scene_count=len(scene),target_count=len(target),mask_valid_depth_count=int(valid.sum()),
                 color_intrinsics=camera)
        group=[];display=[]
        for col,(kind,points,px) in enumerate([('scene',scene,pixels),('target',target,target_pixels)],1):
            world=points@pose[:3,:3].T+pose[:3,3]
            rgb_points=rgb[px[:,1],px[:,0]]
            filename=stem+'-'+frame['stem']+'-'+kind+'.ply'
            write_ply(output/filename,world,rgb_points);row[kind+'_ply']=filename
            step=max(1,int(np.ceil(len(world)/30000))) if kind=='scene' else 1
            shown=world[::step];shown_rgb=rgb_points[::step];row[kind+'_display_count']=len(shown)
            display.append((shown,shown_rgb))
            group.append(len(figure.data))
            figure.add_trace(go.Scatter3d(x=shown[:,0],y=shown[:,1],z=shown[:,2],mode='markers',
                name=kind+' %d / %d'%(len(shown),len(world)),visible=i==0,
                marker=dict(size=1.5 if kind=='scene' else 2,color=['rgb(%d,%d,%d)'%tuple(c) for c in shown_rgb]),showlegend=col==2),row=1,col=col)
            for n,(name,point,color) in enumerate(zip(names,positions,colors)):
                group.append(len(figure.data))
                figure.add_trace(go.Scatter3d(x=[point[0]],y=[point[1]],z=[point[2]],mode='markers',
                    name=name,visible=i==0,showlegend=col==2,
                    marker=dict(size=8 if n in (0,3) else 6,color=color,symbol=['circle-open','cross','circle','diamond'][n])),row=1,col=col)
        groups.append(group);metadata['frames'].append(row)
    for i,group in enumerate(groups):
        buttons.append(dict(label=metadata['frames'][i]['stem'],method='update',
                            args=[dict(visible=[j in group for j in range(len(figure.data))])]))
    axes=dict(xaxis_title='Base X / m',yaxis_title='Base Y / m',zaxis_title='Base Z / m',aspectmode='data')
    figure.update_layout(title=directory.name+' / depth and heuristic input',scene=axes,scene2=axes,
        height=700,margin=dict(l=0,r=0,b=0,t=100),legend=dict(orientation='h',y=-.05),
        updatemenus=[dict(buttons=buttons,direction='down',x=.45,y=1.15)])
    figure.write_html(str(output/(stem+'-clouds.html')),include_plotlyjs=True)
    # Last frame gives a quick static preview; the interactive report includes every input frame.
    fig=plt.figure(figsize=(15,7))
    for col,((points,rgb_points),kind) in enumerate(zip(display,['scene','target']),1):
        ax=fig.add_subplot(1,2,col,projection='3d',computed_zorder=False)
        ax.scatter(*points.T,c=rgb_points/255.,s=1,alpha=.7,zorder=1)
        for n,(name,point,color) in enumerate(zip(names,positions,colors)):
            style=dict(facecolors='none',edgecolors=color) if n==0 else dict(c=color)
            ax.scatter(*point,**style,s=85 if n in (0,3) else 55,marker=['o','x','^','D'][n],
                       label=name,depthshade=False,zorder=5)
        bounds=np.vstack([points,positions]);span=np.maximum(np.ptp(bounds,axis=0),.01)
        center=(bounds.min(axis=0)+bounds.max(axis=0))/2;radius=max(span)*.55
        ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),
               zlim=(center[2]-radius,center[2]+radius))
        ax.set_box_aspect([1,1,1]);ax.view_init(20,-25)
        ax.set(xlabel='Base X / m',ylabel='Base Y / m',zlabel='Base Z / m',
               title=frame['stem']+' '+kind+' (%d/%d points)'%(len(points),row[kind+'_count']))
        if col==2:ax.legend(fontsize=7,loc='upper right')
    fig.subplots_adjust(left=.02,right=.94,bottom=.15,top=.88,wspace=.2)
    fig.savefig(output/(stem+'-clouds.png'),dpi=140);plt.close(fig)
    (output/(stem+'-clouds.json')).write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata
