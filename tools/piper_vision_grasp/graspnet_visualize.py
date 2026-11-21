"""Offline saved-plan rendering; uses URDF meshes, never reads hardware."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from . import geometry as g
from .graspnet import capture_cloud, load_saved_capture


def origin(node):
    pose = np.eye(4)
    if node is not None:
        pose[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
        pose[:3, :3] = g.rotation_matrix(g.Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')))
    return pose


def robot_meshes(urdf, joints, opening_m):
    urdf = Path(urdf); robot = ET.parse(urdf).getroot()
    pending = list(robot.findall('joint'))
    children = {j.find('child').get('link') for j in pending}
    transforms = {l.get('name'): np.eye(4) for l in robot.findall('link') if l.get('name') not in children}
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in transforms:continue
            motion = np.eye(4); name = joint.get('name'); kind = joint.get('type')
            if kind in ('revolute', 'prismatic'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                if kind == 'revolute':
                    motion[:3, :3] = g.rotation_matrix(g.Rotation.from_rotvec(axis * np.radians(joints[int(name[5:])-1])))
                else:
                    motion[:3, 3] = axis * opening_m / 2 * (1 if name == 'joint7' else -1)
            transforms[child] = transforms[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint); progressed = True
        if not progressed:raise ValueError('unresolved URDF joint tree')
    meshes = []
    for link in robot.findall('link'):
        visual = link.find('visual')
        if visual is None or visual.find('geometry/mesh') is None:continue
        mesh = visual.find('geometry/mesh'); filename = mesh.get('filename')
        path = urdf.parent.parent / filename.split('package://piper_description/')[-1]
        raw = path.read_bytes()
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        if len(raw) < 84 or (len(raw)-84) % dtype.itemsize:
            raise ValueError('expected binary STL: ' + str(path))
        triangles = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].astype(float)
        triangles *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        pose = transforms[link.get('name')] @ origin(visual.find('origin'))
        # Display-only surface simplification; never use this mesh for collision decisions.
        if len(triangles) > 2500:
            import open3d as o3d
            surface = o3d.geometry.TriangleMesh(
                o3d.utility.Vector3dVector(triangles.reshape(-1,3)),
                o3d.utility.Vector3iVector(np.arange(len(triangles)*3).reshape(-1,3)))
            surface.remove_duplicated_vertices()
            surface = surface.simplify_quadric_decimation(2500)
            triangles = np.asarray(surface.vertices)[np.asarray(surface.triangles)]
        meshes.append((link.get('name'), triangles @ pose[:3, :3].T + pose[:3, 3]))
    return meshes


def render(config, source, output, urdf):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import plotly.graph_objects as go

    source, output = Path(source), Path(output)
    plan = json.loads((source/'plan.json').read_text())
    if 'planning_config' not in plan['graspnet']:
        raise ValueError('saved plan has no calibration snapshot; regenerate offline preview before visualization')
    config = plan['graspnet']['planning_config']
    saved = json.loads((source/'observation.json').read_text())
    captured = load_saved_capture(saved['capture_dir'], config)
    points, target = capture_cloud(captured)
    ext = g.transform(config['extrinsics']['T_base_color_optical'])
    points = points @ ext[:3, :3].T + ext[:3, 3]
    fig = go.Figure(); static = plt.figure(figsize=(12, 9)); ax = static.add_subplot(111, projection='3d', computed_zorder=False)

    def scatter(name, pts, color, size=1):
        fig.add_trace(go.Scatter3d(x=pts[:,0], y=pts[:,1], z=pts[:,2], mode='markers', name=name,
                                  marker=dict(size=size, color=color)))
        ax.scatter(*pts.T, s=size, c=color, alpha=.5, zorder=6 if size>1 else 1)

    def line(name, pts, color, width=2):
        fig.add_trace(go.Scatter3d(x=pts[:,0], y=pts[:,1], z=pts[:,2], mode='lines', name=name,
                                  line=dict(color=color, width=width)))
        ax.plot(*pts.T, color=color, linewidth=width/2)

    def mesh(name, triangles, color, opacity):
        vertices = triangles.reshape(-1,3); faces = np.arange(len(vertices)).reshape(-1,3)
        fig.add_trace(go.Mesh3d(x=vertices[:,0],y=vertices[:,1],z=vertices[:,2],i=faces[:,0],j=faces[:,1],k=faces[:,2],
                               color=color, opacity=opacity, name=name, showlegend=True))
        ax.add_collection3d(Poly3DCollection(triangles, facecolor=color, alpha=opacity, linewidths=0, zorder=2))

    scatter('Scene', points[::max(1,len(points)//12000)], '#aaaaaa')
    object_points = points[target]
    scatter('Target observed surface', object_points[::max(1,len(object_points)//2500)], '#dd3333', 2)
    try:
        hull = ConvexHull(object_points)
        mesh('Target visible-surface hull (not full object)', object_points[hull.simplices], '#dd3333', .15)
    except QhullError:pass
    for axis, color in enumerate(('#cc3333', '#339944', '#3366cc')):
        line('Camera '+'XYZ'[axis], np.array([ext[:3,3],ext[:3,3]+ext[:3,axis]*.06]),color,4)
    info = plan['graspnet']
    for index, candidate in enumerate(info['candidates']):
        rotation = np.asarray(candidate['rotation']); center = np.asarray(candidate['translation'])
        width, depth = candidate['width'], candidate['depth']
        vertices = np.array([[depth,-width/2,0],[-.02,-width/2,0],[-.02,width/2,0],[depth,width/2,0]])
        vertices = (vertices @ rotation.T + center) @ ext[:3,:3].T + ext[:3,3] + config['semantic']['grasp_offset_base_m']
        selected = index == info['selected_index']
        line('Selected' if selected else 'Candidate %d'%index, vertices, '#00aa44' if selected else '#cc9933', 6 if selected else 1)
    q = plan.get('initial_joints_deg', captured['metadata']['paired_robot'][-1]['robot_snapshot']['joints_deg'])
    if plan['status'] == 'PLANNED':
        q = next(s['angles_deg'] for s in plan['stages'] if s['stage_id'] == 'grasp')
        poses = [g.fk(plan['initial_joints_deg'],config['kinematics'])]
        poses.extend(np.asarray(s['T_base_flange']) for s in plan['stages'])
        line('Planned TCP waypoints', np.array([p[:3,3]+p[:3,:3]@config['semantic']['tcp_offset_m'] for p in poses]), '#2266cc', 4)
    meshes=robot_meshes(urdf,q,config['gripper']['open_mm']/1000)
    for name, triangles in meshes:
        mesh('Piper '+name,triangles,'#8899aa',.4)
    title = 'GraspNet offline: '+plan['status']+' — extrinsics/TCP and physical path clearance unverified'
    ax.set(xlabel='Piper X / m',ylabel='Piper Y / m',zlabel='Piper Z / m',title=title)
    # Bound the view around the robot and target, not distant background pixels.
    bounds=[tri.reshape(-1,3) for _,tri in meshes]
    focus=np.vstack([object_points,np.array([[0,0,0],[.65,0,.45]]),ext[:3,3][None]]+bounds)
    lo,hi=focus.min(axis=0)-.08,focus.max(axis=0)+.08
    fig.update_layout(title=title,scene=dict(xaxis=dict(title='Piper X / m',range=[lo[0],hi[0]]),
        yaxis=dict(title='Piper Y / m',range=[lo[1],hi[1]]),zaxis=dict(title='Piper Z / m',range=[lo[2],hi[2]]),aspectmode='data'))
    fig.write_html(str(output/'graspnet.html'),include_plotlyjs=True,full_html=True)
    ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2]);ax.set_box_aspect(hi-lo)
    ax.view_init(elev=24,azim=-65);static.savefig(output/'graspnet.png',dpi=150);plt.close(static)
    return dict(state='VISUALIZED',offline=True,can_transmit_count=0,html=str(output/'graspnet.html'),png=str(output/'graspnet.png'),
                calibration_source='saved plan snapshot',extrinsics_source_sha256=config['extrinsics']['source_sha256'],
                target_spec=config['target'])
