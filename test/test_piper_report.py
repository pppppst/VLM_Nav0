"""Saved grasp evidence must not invent geometry or executed joint feedback."""
import importlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))


def report():
    return importlib.import_module('piper_vision_grasp.report')


def test_project_base_uses_inverse_camera_transform_and_rejects_behind():
    ext=np.eye(4);ext[:3,3]=[.4,.1,.2]
    camera=dict(fx=100.,fy=100.,ppx=50.,ppy=40.,coeffs=[0]*5)
    pixels=report().project_base([[.4,.1,.7],[.4,.1,.1]],ext,camera)
    assert np.allclose(pixels[0],[50,40])
    assert pixels[1] is None


def test_link_positions_match_existing_fk_and_rotated_projection():
    from piper_vision_grasp import geometry
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json')
    q=[10,40,-30,20,15,-10]
    assert np.allclose(report().link_points(q,cfg['kinematics'])[-1],geometry.fk(q,cfg['kinematics'])[:3,3])
    ext=np.array([[0,0,1,.4],[1,0,0,.1],[0,1,0,.2],[0,0,0,1.]])
    camera=dict(fx=100.,fy=100.,ppx=50.,ppy=40.,coeffs=[0]*5)
    base=ext[:3,:3]@np.array([.1,.2,1.])+ext[:3,3]
    assert np.allclose(report().project_base([base],ext,camera)[0],[60,60])


@pytest.mark.parametrize('q',[[10,40,-30,20,15,-10],[-4.642,179.926,-156.774,-19.757,-8.605,19.002]])
def test_feedback_tcp_uses_rotated_tool_offset_without_clipping_measurements(q):
    from piper_vision_grasp import geometry
    import copy
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json')
    model=copy.deepcopy(cfg['kinematics'])
    # Widen only the independent test oracle to evaluate the slightly over-limit measured J2.
    model['joints'][1]['limit_rad'][1]=np.pi+1
    pose=geometry.fk(q,model)
    offset=np.array([.012,-.008,.135])
    points=report().link_points(q,cfg['kinematics'],offset)
    assert np.allclose(points[-2],pose[:3,3])
    assert np.allclose(points[-1],pose[:3,3]+pose[:3,:3]@offset)
    assert not np.allclose(points[-1],pose[:3,3]+offset)


def test_unexecuted_stages_keep_actual_missing_and_real_error_is_signed():
    stages=[dict(stage_id='approach',angles_deg=[1]*6),dict(stage_id='grasp',angles_deg=[2]*6)]
    events=[dict(state='AT_TARGET',stage='approach',target=[1]*6,time=10,
                 measured=dict(joints_deg=[1.1]*6,missing_feedback=[],sample_time=9))]
    rows=report().joint_comparison(dict(stages=stages),events)
    assert rows[0]['actual_deg']==[1.1]*6
    assert np.allclose(rows[0]['error_deg'],[.1]*6)
    assert rows[1]['actual_deg'] is None and rows[1]['state']=='NOT_EXECUTED'


def test_stale_joint_feedback_is_never_plotted_as_reached():
    plan=dict(stages=[dict(stage_id='grasp',angles_deg=[2]*6)])
    events=[dict(state='WAIT_FEEDBACK',stage='grasp',target=[2]*6,time=10,
                 measured=dict(joints_deg=[2]*6,missing_feedback=['0x2a5']))]
    assert report().joint_comparison(plan,events)[0]['actual_deg'] is None


def test_feedback_outage_preserves_last_sample_but_not_its_old_status():
    plan=dict(stages=[dict(stage_id='grasp',angles_deg=[2]*6)])
    events=[dict(state='WAIT_TARGET',stage='grasp',target=[2]*6,time=10,
                 measured=dict(joints_deg=[1]*6,missing_feedback=[])),
            dict(state='WAIT_FEEDBACK',stage='grasp',time=11,measured=None)]
    row=report().joint_comparison(plan,events)[0]
    assert row['actual_deg']==[1]*6 and row['state']=='WAIT_FEEDBACK'
    assert row['sample_state']=='WAIT_TARGET' and row['time']==10


@pytest.mark.parametrize('frame_count',[1,3])
def test_render_portable_legacy_geometry_and_missing_final_result(tmp_path,frame_count):
    pytest.importorskip('plotly')
    import cv2
    from piper_vision_grasp import geometry
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    cfg['target']=dict(color='绿色',description='盒子')
    source=tmp_path/'copied-run';source.mkdir()
    (source/'config_snapshot.json').write_text(json.dumps(cfg))
    (source/'plan.json').write_text(json.dumps(dict(stages=[
        dict(stage_id='approach',angles_deg=[1]*6),dict(stage_id='lift',angles_deg=[2]*6)])))
    (source/'events.jsonl').write_text(json.dumps(dict(state='AT_TARGET',stage='approach',time=10,target=[1]*6,
        measured=dict(joints_deg=[1.125,.875,1,1,1,1],missing_feedback=[])))+'\n')
    (source/'operator-verification.json').write_text(json.dumps(dict(status='OPERATOR_REPORTED_GRASP_FAILED',operator_statement='抓空或位置偏了')))
    capture=source/'before-portable';capture.mkdir()
    camera=dict(width=20,height=20,fx=100.,fy=100.,ppx=10.,ppy=10.,coeffs=[0.]*5)
    metadata=dict(color_intrinsics=camera,depth_scale=.001,frames=[dict(stem='frame%02d'%i,host_unix=i+1,aligned_intrinsics=camera) for i in range(frame_count)])
    observation=dict(point_color_m=[0,0,.5],grasp_axis_base=[0,1,0],grasp_width_m=.04)
    (capture/'observation.json').write_text(json.dumps(dict(metadata=metadata,observation=observation,capture_dir='/obsolete/remote/path')))
    plan=dict(status='WAIT_IK',target=geometry.absolute_target(observation,cfg),
              failed=dict(status='WAIT_IK',angles_deg=[1,2,3,4,5,6],position_error_m=.075,rotation_error_deg=13.6))
    plan['target']['pregrasp_offset_base_m']=[-.1,0,0]
    plan['orientation_candidates']=[dict(index=0,pitch_deg=0.,yaw_offset_deg=90.,status='WAIT_IK',reason='entry unavailable',failed=plan['failed'])]
    plan['selected_candidate_index']=None
    (capture/'planning.json').write_text(json.dumps(dict(config=cfg,plan=plan,initial_joints_deg=[0]*6)))
    for i in range(frame_count):
        cv2.imwrite(str(capture/('frame%02d-color.png'%i)),np.zeros((20,20,3),np.uint8))
        cv2.imwrite(str(capture/('mask%02d.png'%i)),np.full((20,20),255,np.uint8))
        np.savez(capture/('frame%02d-depth.npz'%i),aligned_depth=np.full((20,20),500+i,np.uint16))
    out=tmp_path/'report'
    report().render({},source,out)
    summary=json.loads((out/'summary.json').read_text())
    assert summary['result']['state']=='NO_FINAL_RESULT'
    assert summary['captures'][0]['ik_position_tolerance_mm']==8.
    target=plan['target']['T_base_tcp']
    expected=np.array(target)[:3,3]+[-.1,0,0]
    assert np.allclose(summary['captures'][0]['pregrasp_tcp_base_m'],expected)
    assert '绿色' in (out/'index.html').read_text()
    assert '抓空或位置偏了' in (out/'index.html').read_text()
    assert summary['operator_verification']['status']=='OPERATOR_REPORTED_GRASP_FAILED'
    assert not summary['captures'][0]['execution_plan']
    assert summary['captures'][0]['ik_candidate_deg']==[1,2,3,4,5,6]
    assert summary['captures'][0]['actual_reached_deg'] is None
    assert summary['captures'][0]['grasp_feedback_fk'] is None
    page=(out/'index.html').read_text()
    assert not list(out.glob('*joints.png')) and 'joints.png' not in page
    assert '候选IK（未下发）' in page and '采图时反馈（不是到位结果）' in page
    assert '<td>1.125</td>' in page and '<td>0.125</td>' in page
    assert '<td>-0.125</td>' in page
    assert 'NOT_EXECUTED' in page and '无有效反馈' in page
    assert '<th>J6 (°)</th>' in page and 'joints.csv' in page
    assert (out/'candidate-00-rgb.png').is_file() and (out/'candidate-00-3d.html').is_file()
    clouds=json.loads((out/'candidate-00-clouds.json').read_text())
    assert len(clouds['frames'])==frame_count
    assert clouds['units']=='m' and clouds['frame']=='piper_base'
    for row in clouds['frames']:
        assert row['scene_count']==row['target_count']==400
        for key in ('scene_ply','target_ply'):
            raw=(out/row[key]).read_bytes()
            header,body=raw.split(b'end_header\n',1)
            assert b'element vertex 400' in header
            assert len(body)==400*(3*8+3)
    assert 'candidate-00-clouds.html' in (out/'index.html').read_text()
    assert 'entry unavailable' in (out/'index.html').read_text()
    assert summary['captures'][0]['orientation_candidates'][0]['yaw_offset_deg']==90
    if frame_count==3:
        plan.update(status='PLANNED',stages=[dict(stage_id='grasp',angles_deg=[2]*6)])
        plan.pop('failed')
        (capture/'planning.json').write_text(json.dumps(dict(config=cfg,plan=plan,initial_joints_deg=[0]*6)))
        (source/'plan.json').write_text(json.dumps(dict(plan,capture_dir='/old/path/before-portable')))
        measured_q=[1,1,-1,1,1,1]
        (source/'events.jsonl').write_text(json.dumps(dict(state='AT_TARGET',stage='grasp',time=11,
            measured=dict(joints_deg=measured_q,missing_feedback=[])))+'\n')
        actual_out=tmp_path/'report-with-feedback'
        report().render({},source,actual_out)
        actual=json.loads((actual_out/'summary.json').read_text())['captures'][0]['grasp_feedback_fk']
        flange=geometry.fk(measured_q,cfg['kinematics'])
        expected_tcp=flange[:3,3]+flange[:3,:3]@cfg['semantic']['tcp_offset_m']
        assert np.allclose(actual['tcp_base_m'],expected_tcp)
        assert np.allclose(actual['delta_to_candidate_m'],expected_tcp-np.array(target)[:3,3])
        assert actual['sample_state']=='AT_TARGET' and actual['time']==11
        for suffix in ('-3d.html','-clouds.html'):
            assert 'Grasp feedback TCP (FK, AT_TARGET)' in (actual_out/('candidate-00'+suffix)).read_text()
        cloud_metadata=json.loads((actual_out/'candidate-00-clouds.json').read_text())
        assert cloud_metadata['grasp_feedback_fk']==actual


def test_target_cloud_is_exact_heuristic_input_and_scene_keeps_background(tmp_path):
    from piper_vision_grasp.semantic import target_points
    from piper_vision_grasp.cloud_report import depth_points,write_ply
    depth=np.full((5,5),500.,dtype=float)
    depth[0,:]=[0,np.nan,3000,500,800]
    mask=np.ones((5,5),bool);mask[0,4]=False
    camera=dict(width=5,height=5,fx=100.,fy=100.,ppx=2.,ppy=2.,coeffs=[0.]*5,depth_scale=.001)
    pixels,points,valid=target_points(depth,camera,mask)
    assert valid.sum()==22 and len(points)==21
    assert np.all(points[:,2]==.5)
    assert not any(np.array_equal(p,[2,0]) for p in pixels)
    scene_pixels,scene=depth_points(depth,camera)
    assert len(scene)==23 and scene[:,2].max()==3
    assert np.allclose(scene[0], [0,-.06,3])
    rgb=np.arange(75,dtype=np.uint8).reshape(5,5,3)
    colors=rgb[pixels[:,1],pixels[:,0]]
    path=tmp_path/'target.ply';write_ply(path,points+[.1,.2,.3],colors)
    body=path.read_bytes().split(b'end_header\n',1)[1]
    data=np.frombuffer(body,dtype=[('xyz','<f8',(3,)),('rgb','u1',(3,))])
    assert np.array_equal(data['rgb'],colors)
    assert np.array_equal(data['xyz'],points+[.1,.2,.3])
    empty=target_points(np.zeros_like(depth),camera,mask)
    assert empty[0].shape==empty[1].shape[:1]+(2,) and len(empty[1])==0
    with pytest.raises(ValueError,match='shape mismatch'):
        target_points(depth,camera,mask[:2])


def test_failed_plan_is_saved_with_its_exact_capture_and_configuration(tmp_path,monkeypatch):
    from piper_vision_grasp import main,geometry
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json')
    capture=tmp_path/'before-test';capture.mkdir()
    captured=dict(capture_dir=str(capture),observation={})
    monkeypatch.setattr(main,'acquire',lambda *args:(captured,dict(joints_deg=[0]*6)))
    monkeypatch.setattr(main,'make_plan',lambda *args:dict(status='WAIT_IK',reason='unreachable',target={'candidate':1}))
    def stop(_):raise KeyboardInterrupt
    monkeypatch.setattr(main.time,'sleep',stop)
    with pytest.raises(KeyboardInterrupt):main.prepare_plan(cfg,None,None,main.Events(cfg,tmp_path))
    saved=json.loads((capture/'planning.json').read_text())
    assert saved['plan']['status']=='WAIT_IK'
    assert saved['config']['extrinsics']==cfg['extrinsics']
    assert saved['initial_joints_deg']==[0]*6
    assert not (tmp_path/'plan.json').exists()


def test_arm_contour_backprojects_only_observed_valid_boundary_depth():
    from piper_vision_grasp.arm_outline import contour_points
    mask=np.zeros((7,7),np.uint8);mask[1:6,1:6]=1
    depth=np.full((7,7),500.,dtype=float);depth[1,1]=0;depth[1,2]=np.nan
    camera=dict(width=7,height=7,fx=100.,fy=100.,ppx=3.,ppy=3.,coeffs=[0.]*5,depth_scale=.001)
    pixels,points,stats=contour_points(mask,depth,camera)
    assert stats['boundary_pixels']==16 and stats['valid_depth_pixels']==14
    assert len(points)==14 and not np.any(np.all(pixels==[3,3],axis=1))
    assert np.allclose(points[:,2],.5)
    assert np.allclose(points[:,:2],(pixels-3)/100.*.5)


def test_arm_urdf_fk_matches_flange_and_uses_measured_gripper_opening():
    import xml.etree.ElementTree as ET
    from piper_vision_grasp.arm_outline import urdf_transforms
    from piper_vision_grasp import geometry
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    robot=ET.parse(ROOT/'config/piper_vision_grasp/provenance/piper_description.urdf').getroot()
    q=[10,40,-30,20,15,-10]
    closed=urdf_transforms(robot,q,0);opened=urdf_transforms(robot,q,48.3)
    assert np.allclose(opened['link6'],geometry.fk(q,cfg['kinematics']))
    assert np.allclose(closed['link6'],opened['link6'])
    for finger in ('link7','link8'):
        assert np.isclose(np.linalg.norm(opened[finger][:3,3]-closed[finger][:3,3]),.02415)
    with pytest.raises(ValueError):urdf_transforms(robot,[0]*5,0)
    with pytest.raises(ValueError):urdf_transforms(robot,q,float('nan'))


def test_four_pose_arm_overlay_from_saved_images_is_offline_and_preserves_joints(tmp_path,monkeypatch):
    pytest.importorskip('plotly')
    import cv2
    import xml.etree.ElementTree as ET
    from piper_vision_grasp import geometry,arm_outline,semantic,cloud_report
    import plotly.graph_objects as go
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    ext=np.eye(4);ext[0,3]=.5
    cfg['extrinsics']['T_base_color_optical']=ext.tolist()
    camera_points=np.array([[.49,0,0],[.5,0,0],[.51,0,0],[-1.49,0,0],[-1.51,0,0],[0,1,0]])
    monkeypatch.setattr(cloud_report,'depth_points',lambda *a:(None,camera_points))
    figures=[];write_html=go.Figure.write_html
    def save_figure(figure,*args,**kwargs):
        figures.append(figure)
        return write_html(figure,*args,**kwargs)
    monkeypatch.setattr(go.Figure,'write_html',save_figure)
    original_target=cfg['target'].copy()
    source=tmp_path/'run';source.mkdir();out=tmp_path/'report';out.mkdir()
    (source/'config_snapshot.json').write_text(json.dumps(cfg));(source/'events.jsonl').write_text('')
    urdf=tmp_path/'model/urdf/piper_description.urdf';urdf.parent.mkdir(parents=True)
    urdf.write_bytes((ROOT/'config/piper_vision_grasp/provenance/piper_description.urdf').read_bytes().replace(b'\n',b'\r\n').rstrip(b'\r\n'))
    mesh_dir=tmp_path/'model/meshes';mesh_dir.mkdir()
    triangle=np.zeros(1,dtype=[('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
    triangle['vertices']=[[[0,0,0],[.01,0,0],[0,.01,0]]]
    for node in ET.parse(urdf).getroot().findall('link/visual/geometry/mesh'):
        (mesh_dir/Path(node.get('filename')).name).write_bytes(b'\0'*80+(1).to_bytes(4,'little')+triangle.tobytes())
    camera=dict(width=20,height=20,fx=100.,fy=100.,ppx=10.,ppy=10.,coeffs=[0.]*5)
    selected=[]
    def segment(rgb,config):
        selected.append(config['target'])
        mask=np.zeros((20,20),np.uint8);mask[5:15,5:15]=1
        return dict(mask=mask,bbox_px=[5,5,15,15],label='Piper arm')
    monkeypatch.setattr(semantic,'locate_and_segment',segment)
    for i,stage in enumerate(['pregrasp','grasp-1','grasp','lift']):
        directory=source/('arm-'+stage+'-test');directory.mkdir()
        snapshot=dict(joints_deg=[0,20+i,-10,0,0,0],gripper_width_mm=48.3 if i==3 else 70.,state='READY',missing_feedback=[])
        frame=dict(stem='frame00',host_unix=100+i,aligned_intrinsics=camera,
                   paired_robot=[dict(robot_snapshot=snapshot),dict(robot_snapshot=snapshot)])
        metadata=dict(color_intrinsics=camera,depth_scale=.001,frames=[frame])
        (directory/'metadata.json').write_text(json.dumps(metadata))
        cv2.imwrite(str(directory/'frame00-color.png'),np.zeros((20,20,3),np.uint8))
        np.savez(directory/'frame00-depth.npz',aligned_depth=np.full((20,20),500,np.uint16))
        arm_outline.segment_capture(directory,cfg,stage)
    assert len(selected)==4 and all('机械夹爪' in t['description'] and '局部' in t['description'] for t in selected)
    assert all(t['color']=='黑色及金属色' for t in selected)
    assert cfg['target']==original_target
    monkeypatch.setattr(semantic,'locate_and_segment',lambda *a:pytest.fail('offline report contacted model service'))
    result=report().render({},source,out,urdf)
    assert result['offline'] and result['can_transmit_count']==0
    assert result['arm_poses']==4
    data=json.loads((out/'arm-overlay.json').read_text())
    assert [p['stage'] for p in data['poses']]==['pregrasp','grasp-1','grasp','lift']
    assert data['poses'][-1]['gripper_width_mm']==48.3
    assert data['poses'][-1]['joints_deg']==[0,23,-10,0,0,0]
    assert all(p['depth']['valid_depth_pixels']==36 for p in data['poses'])
    assert data['display_origin_m']==[0,0,0] and data['scene_radius_m']==1.0
    assert all(p['scene_points']==dict(valid=6,within_1m=3,displayed=3) for p in data['poses'])
    shown=[t for f in figures for t in f.data if t.name=='Observed scene (sampled)']
    assert len(shown)==4
    for trace in shown:
        np.testing.assert_allclose(trace.x,[.99,1.,-.99])
        assert np.all(np.linalg.norm(np.column_stack([trace.x,trace.y,trace.z]),axis=1)<=1.)
    assert len(data['model']['meshes'])==10
    assert data['model']['urdf_lf_sha256']==cfg['kinematics']['source_sha256']
    assert data['model']['urdf_sha256']!=data['model']['urdf_lf_sha256']
    assert 'arm-overlay.html' in (out/'index.html').read_text()
    overlay=(out/'arm-overlay.html').read_text()
    assert 'RGB contour + measured depth' not in overlay
    assert 'Observed scene (sampled)' in overlay and 'FK model' in overlay
    assert 'Piper base origin' in overlay
    assert len(list(out.glob('arm-pose-*-contour.ply')))==4
    assert not list(out.glob('*joints.png'))


def test_arm_not_visible_saves_explicit_missing_contour_not_a_motion_gate(tmp_path,monkeypatch):
    import cv2
    from piper_vision_grasp import geometry,arm_outline,semantic
    cfg=geometry.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    frame=dict(stem='frame00',paired_robot=[dict(robot_snapshot=dict(joints_deg=[0]*6,gripper_width_mm=70))])
    (tmp_path/'metadata.json').write_text(json.dumps(dict(frames=[frame])))
    cv2.imwrite(str(tmp_path/'frame00-color.png'),np.zeros((20,20,3),np.uint8))
    def absent(*a):raise RuntimeError('requested object not found')
    monkeypatch.setattr(semantic,'locate_and_segment',absent)
    arm_outline.segment_capture(tmp_path,cfg,'pregrasp')
    record=json.loads((tmp_path/'arm-observation.json').read_text())
    assert record['visibility']=='NOT_DETECTED' and record['bbox_px'] is None
    assert not cv2.imread(str(tmp_path/'arm-mask.png'),cv2.IMREAD_GRAYSCALE).any()
    def failed(*a):raise RuntimeError('model HTTP status 503')
    monkeypatch.setattr(semantic,'locate_and_segment',failed)
    with pytest.raises(RuntimeError,match='503'):arm_outline.segment_capture(tmp_path,cfg,'pregrasp')
