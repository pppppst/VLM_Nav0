"""Semantic RGB-D contracts, without network, models or hardware at collection time."""
import copy
import importlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))

def module():
    assert (ROOT/'tools/piper_vision_grasp/semantic.py').exists(), 'semantic pipeline not implemented'
    return importlib.import_module('piper_vision_grasp.semantic')


def test_default_entry_selects_heuristic_at_updated_speed(tmp_path, monkeypatch):
    from piper_vision_grasp import main, geometry, graspnet
    loaded=[]
    load=geometry.load_config
    def capture(path):
        config=load(path);loaded.append(config)
        return config
    monkeypatch.setattr(geometry,'load_config',capture)
    monkeypatch.setattr(main,'dependency_report',lambda:{})
    monkeypatch.delenv('DISPLAY',raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY',raising=False)
    monkeypatch.setattr(sys,'argv',['piper','check','--output-root',str(tmp_path),'--run-id','default'])
    assert main.main()==0
    config=loaded[0]
    assert config['mode']=='semantic'
    assert config.get('grasp_backend','legacy')=='legacy'
    assert config['motion']['speed_percent']==30
    assert config['semantic']['pregrasp_offset_base_m']==[0,0,.1]
    def forbidden(*args,**kwargs):raise AssertionError('GraspNet must not be called')
    monkeypatch.setattr(graspnet,'plan_capture',forbidden)
    monkeypatch.setattr(geometry,'plan_absolute',lambda joints,cfg,target:dict(status='PLANNED',target=target))
    observation=dict(point_color_m=[0,0,.5],grasp_axis_base=[0,1,0],grasp_width_m=.04)
    plan=main.make_plan(config,[0]*6,observation)
    assert plan['status']=='PLANNED' and plan['selected_candidate_index']==0
    assert plan['target']['approach_pitch_deg']==40


def test_user_extrinsic_raise_is_base_z_translation_with_original_rotation():
    from piper_vision_grasp import geometry as g
    historical=json.loads((ROOT/'config/piper_vision_grasp/provenance/manual04-user-z130-20261114.json').read_text())
    original=json.loads((ROOT/'config/piper_vision_grasp/provenance/manual04-result.json').read_text())
    old=np.array(original['T_base_color_optical']);new=np.array(historical['T_base_color_optical'])
    assert np.array_equal(new[:3,:3],old[:3,:3])
    assert np.array_equal(new[:2,3],old[:2,3])
    assert np.allclose(new[:3,3]-old[:3,3],[0,0,.130],atol=1e-12)
    points=np.array([[.01,.07,.32],[-.02,.05,.24],[0,0,1.]])
    assert np.allclose((points@new[:3,:3].T+new[:3,3])-(points@old[:3,:3].T+old[:3,3]),[0,0,.130])

def scene():
    rgb=np.zeros((80,100,3),np.uint8);rgb[20:60,30:70]=[0,180,0]
    depth=np.full((80,100),500,np.uint16)
    camera=dict(width=100,height=80,fx=100.,fy=100.,ppx=50.,ppy=40.,coeffs=[0]*5,depth_scale=.001)
    mask=np.zeros(depth.shape,bool);mask[20:60,30:70]=True
    config=dict(target=dict(color='绿色',description='小盒子'),semantic=dict(tcp_offset_m=[0,0,.135],grasp_offset_base_m=[0,0,-.013]),
                extrinsics=dict(T_base_color_optical=np.eye(4).tolist()))
    selection=dict(bbox_px=[30,20,70,60],keypoint_px=[50,40],mask=mask)
    return rgb,depth,camera,config,selection


def test_color_description_are_both_sent_to_detector():
    m=module();prompt=m.target_prompt(dict(color='绿色',description='小盒子'))
    assert '绿色' in prompt and '小盒子' in prompt
    assert 'bbox_2d' in prompt and 'point_2d' in prompt


def test_normalized_response_sorted_clamped_and_semantic_point_preserved():
    m=module()
    row=m.parse_detection('```json\n{"bbox_2d":[700,750,300,250],"point_2d":[450,500]}\n```',(80,100))
    assert row['bbox_px']==[30,20,70,60]
    assert row['keypoint_px']==[45,40]
    with pytest.raises(ValueError):m.parse_detection('{"bbox_2d":[0,0,0,0],"point_2d":[1,1]}',(80,100))
    with pytest.raises(ValueError):m.parse_detection('{"bbox_2d":[0,0,1000,1000],"point_2d":[NaN,1]}',(80,100))


def test_mask_dimensions_and_tcp_are_measured_without_old_blue_limits():
    m=module();rgb,depth,camera,cfg,sel=scene()
    result=m.measure(rgb,depth,camera,cfg,sel)
    assert result['status']=='DETECTED'
    assert np.allclose(result['point_color_m'],[0,0,.5],atol=.005)
    assert .18<result['grasp_width_m']<.21
    assert np.allclose(result['dimensions_base_m'],[.195,.195,0])
    assert result['target']==cfg['target']
    # A 195mm green face exceeds all old blue reference bounds but is still measured.
    assert result['width_m']>.052


def test_measurement_excludes_background_and_invalid_depth():
    m=module();rgb,depth,camera,cfg,sel=scene()
    depth[~sel['mask']]=5000;depth[40,50]=0
    obs=m.measure(rgb,depth,camera,cfg,sel)
    assert obs['status']=='DETECTED' and obs['point_color_m'][2]==pytest.approx(.5)
    depth[:]=0
    assert m.measure(rgb,depth,camera,cfg,sel)['status']=='WAIT_DEPTH'


def test_absolute_target_applies_full_extrinsic_and_rotated_tcp_once():
    m=module();rgb,depth,cam,cfg,sel=scene()
    cfg['extrinsics']['T_base_color_optical'][0][3]=.4
    obs=m.measure(rgb,depth,cam,cfg,sel)
    from piper_vision_grasp import geometry as g
    assert hasattr(g,'absolute_target')
    target=g.absolute_target(obs,cfg)
    tcp=np.array(target['T_base_tcp']);flange=np.array(target['T_base_flange'])
    assert np.allclose(tcp[:3,3],[.4,0,.487],atol=.005)
    assert np.allclose(tcp[:3,2],[0,0,-1])
    assert np.allclose(flange[:3,3],tcp[:3,3]-tcp[:3,:3]@np.array([0,0,.135]))
    assert np.linalg.det(flange[:3,:3])==pytest.approx(1)


def test_semantic_config_and_cli_target_do_not_use_blue_reference():
    module()
    from piper_vision_grasp import geometry as g, main
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    assert cfg['mode']=='semantic'
    assert cfg['reference']['reference_id']=='semantic-rgbd-v1'
    main.configure_target(cfg,'绿色','小盒子')
    assert cfg['target']==dict(color='绿色',description='小盒子')
    assert cfg['gripper']['close_mm']!=36


def test_absolute_plan_matches_computed_pose_and_measured_closure():
    module()
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    q=[0,117.957,-50.003,0,27.042,0]
    goal=g.fk(q,cfg['kinematics'])
    target=dict(T_base_flange=goal.tolist(),grasp_width_m=.045,reference_id='semantic-rgbd-v1')
    plan=g.plan_absolute(q,cfg,target)
    assert plan['status']=='PLANNED',plan
    assert plan['close_mm']==pytest.approx(41.)
    grasp=next(s for s in plan['stages'] if s['stage_id']=='grasp')
    assert np.allclose(grasp['T_base_flange'],goal)


def test_base_vertical_pregrasp_raises_entry_and_descends_without_sideways_motion():
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    cfg['semantic'].update(pregrasp_offset_base_m=[0,0,.1],entry_clearance_m=.1)
    fixture=json.loads((ROOT/'test/fixtures/piper_vision_grasp/camera-avoidance-target.json').read_text())
    plan=g.plan_absolute(fixture['initial_joints_deg'],cfg,fixture['target'])
    assert plan['status']=='PLANNED',plan
    poses={s['stage_id']:np.array(s['T_base_flange']) for s in plan['stages']}
    # These base-frame positions are from the independently frozen 60 degree target.
    assert np.allclose(poses['entry'][:3,3],[.4660365854362292,.021695940055537367,.22516101676969858],atol=.001)
    assert np.allclose(poses['pregrasp'][:3,3],[.4660365854362292,.021695940055537367,.12516101676969858],atol=.001)
    assert np.allclose(poses['grasp'],fixture['target']['T_base_flange'],atol=.001)
    descent=[poses['entry']]+[np.array(s['T_base_flange']) for s in plan['stages'] if s['phase']=='approach']
    for a,b in zip(descent,descent[1:]):
        assert np.allclose(a[:2,3],b[:2,3],atol=.001)
        assert b[2,3]<=a[2,3]+.001
        assert np.allclose(a[:3,:3],b[:3,:3],atol=.001)


@pytest.mark.parametrize('field,value',[
    ('pregrasp_offset_base_m',[0,0]),('pregrasp_offset_base_m',[0,0,float('nan')]),
    ('entry_clearance_m',-.1),('entry_clearance_m',float('inf')),
])
def test_absolute_entry_rejects_invalid_geometry_configuration(field,value):
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json');cfg['semantic'][field]=value
    fixture=json.loads((ROOT/'test/fixtures/piper_vision_grasp/camera-avoidance-target.json').read_text())
    with pytest.raises(ValueError,match='pregrasp|clearance'):
        g.plan_absolute(fixture['initial_joints_deg'],cfg,fixture['target'])


def test_service_mask_png_roundtrip_and_shape_rejection():
    m=module();_,_,_,_,sel=scene()
    encoded=m.encode_mask(sel['mask'])
    assert np.array_equal(m.decode_mask(encoded,(80,100)),sel['mask'])
    with pytest.raises(ValueError):m.decode_mask(encoded,(10,10))


@pytest.mark.parametrize('motion_status', [0, 1])
def test_semantic_cycle_uses_measured_close_and_existing_once_only_execution(tmp_path,monkeypatch,motion_status):
    module()
    from piper_vision_grasp import main as m, geometry as g
    from test_piper_vision_grasp import FakeSession
    import time
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    cfg['motion'].update(record_arm_outline=False,return_zero_after_grasp=False)
    cfg['gripper']['stable_seconds']=.2;cfg['poll_seconds']=0
    session=FakeSession();session.joints=[0,117.957,-50.003,0,27.042,0]
    snapshot=session.snapshot
    def measured():
        return dict(snapshot(),motion_status=motion_status)
    session.snapshot=measured
    flange=g.fk(session.joints,cfg['kinematics'])
    # Build a visible observation whose absolute TCP offset recovers a reachable flange pose.
    ext=np.asarray(cfg['extrinsics']['T_base_color_optical'])
    tcp=flange[:3,3]+np.array([0,0,-.135])
    point=ext[:3,:3].T@(tcp-ext[:3,3]-np.array(cfg['semantic']['grasp_offset_base_m']))
    obs=dict(status='DETECTED',reason='fake semantic observation',detector_version='semantic-rgbd-v1',
             point_color_m=point.tolist(),grasp_axis_base=[0,1,0],grasp_width_m=.045,
             width_m=.045,height_m=.06,normal_color=[0,0,1],long_axis_color=[0,1,0])
    calls=[0]
    def camera(config,directory,count=3):
        calls[0]+=1
        if calls[0]>6:raise KeyboardInterrupt
        row=copy.deepcopy(obs)
        if any(c[0]=='lift' for c in session.calls):row['point_color_m']=(point+ext[:3,:3].T@[0,0,.02]).tolist()
        return dict(observation=row,observations=[copy.deepcopy(row) for _ in range(3)],
                    metadata=dict(frames=[dict(host_unix=time.time(),color_frame=i,depth_frame=i) for i in range(3)]))
    sleeps=[0]
    def sleep(_):
        sleeps[0]+=1
        if sleeps[0]>300:raise AssertionError('cycle blocked despite fresh positions at target')
    monkeypatch.setattr(m.time,'sleep',sleep)
    result=m.run_once(cfg,session,camera,tmp_path)
    assert result['state']=='GRASP_CONFIRMED',result
    assert [c[2] for c in session.calls if c[0]=='close']==[41.]
    assert len({c[0] for c in session.calls})==len(session.calls)
    assert len([c for c in session.calls if c[0]=='open'])==1


def test_metric_extents_remove_sparse_far_background_inside_sam_mask():
    m=module();rgb,depth,cam,cfg,sel=scene()
    depth[20:22,30:70]=5000
    result=m.measure(rgb,depth,cam,cfg,sel)
    assert result['status']=='DETECTED'
    assert .18<result['grasp_width_m']<.21
    assert max(result['dimensions_base_m'])<.21


def test_target_center_is_stable_when_model_keypoint_jitters_on_same_mask():
    m=module();rgb,depth,cam,cfg,sel=scene()
    sel['keypoint_px']=[35,30]
    first=m.measure(rgb,depth,cam,cfg,sel)
    sel['keypoint_px']=[65,50]
    second=m.measure(rgb,depth,cam,cfg,sel)
    assert np.allclose(first['point_color_m'],second['point_color_m'])


def test_real_green_box_masks_keep_three_frame_geometry_stable():
    m=module()
    from piper_vision_grasp import geometry as g,vision
    import cv2
    directory=ROOT/'test/fixtures/piper_vision_grasp/green-box-semantic-20261114'
    rows=json.loads((directory/'manifest.json').read_text())['frames']
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json');observations=[]
    for row in rows:
        with np.load(directory/row['depth']) as data:depth=data['aligned_depth']
        mask=cv2.imread(str(directory/row['mask']),0)>0
        sel=dict(row,mask=mask)
        obs=m.measure(np.zeros(depth.shape+(3,),np.uint8),depth,row['camera'],cfg,sel)
        observations.append(obs)
    combined=vision.combine(observations,cfg['detector'])
    assert combined['status']=='DETECTED',combined
    assert .075<combined['grasp_width_m']<.09
    assert max(combined['dimensions_base_m'])<.13


def test_approach_pitch_rotates_tcp_offset_without_moving_visual_target():
    m=module();rgb,depth,cam,cfg,sel=scene()
    from piper_vision_grasp import geometry as g
    obs=m.measure(rgb,depth,cam,cfg,sel)
    vertical=g.absolute_target(obs,cfg)
    cfg['semantic']['approach_pitch_deg']=30.
    tilted=g.absolute_target(obs,cfg)
    a,b=np.array(vertical['T_base_tcp']),np.array(tilted['T_base_tcp'])
    assert np.allclose(a[:3,3],b[:3,3])
    assert b[2,2]==pytest.approx(-.5)
    assert np.linalg.det(b[:3,:3])==pytest.approx(1.)
    assert np.allclose(np.array(tilted['T_base_flange'])[:3,3],b[:3,3]-b[:3,:3]@[0,0,.135])


@pytest.mark.parametrize('camera_roll_deg,expected_width', [(0, .195), (90, .120)])
def test_grasp_axis_uses_horizontal_3d_edge_despite_wide_image_box(camera_roll_deg,expected_width):
    m=module();rgb,depth,cam,cfg,sel=scene()
    from piper_vision_grasp import geometry as g
    transform=np.array(g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')['extrinsics']['T_base_color_optical'])
    a=np.radians(camera_roll_deg)
    roll=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    transform[:3,:3]=transform[:3,:3]@roll
    cfg['extrinsics']['T_base_color_optical']=transform.tolist()
    sel['mask'][:]=False;sel['mask'][20:45,30:70]=True
    sel['bbox_px']=[30,20,70,45]
    obs=m.measure(rgb,depth,cam,cfg,sel)
    assert abs(obs['grasp_axis_base'][1])>.95
    assert obs['grasp_width_m']==pytest.approx(expected_width,abs=.015)


def test_historical_user_extrinsic_y_shift_z_raise_and_minimal_camera_z_alignment():
    historical=json.loads((ROOT/'config/piper_vision_grasp/provenance/manual04-user-z180-y60-alignx-20261114.json').read_text())
    previous=json.loads((ROOT/'config/piper_vision_grasp/provenance/manual04-user-z130-20261114.json').read_text())
    old=np.array(previous['T_base_color_optical']);new=np.array(historical['T_base_color_optical'])
    assert np.allclose(new[:3,3]-old[:3,3],[0,-.060,.050],atol=1e-12,rtol=0)
    aligned=json.loads((ROOT/'config/piper_vision_grasp/provenance/manual04-user-z130-y110-alignx-20261114.json').read_text())
    assert np.array_equal(new[:3,:3],np.array(aligned['T_base_color_optical'])[:3,:3])
    assert np.allclose(new[:3,2],[1,0,0],atol=1e-12,rtol=0)
    assert np.allclose(new[:3,:3].T@new[:3,:3],np.eye(3),atol=1e-12)
    assert np.linalg.det(new[:3,:3])==pytest.approx(1)
    relative=new[:3,:3]@old[:3,:3].T
    assert (np.trace(relative)-1)/2==pytest.approx(old[0,2],abs=1e-12)
    # An optical-axis metre must point exactly along base +X, with no extra translation.
    assert np.allclose(new@np.array([0,0,1,1]),[new[0,3]+1,new[1,3],new[2,3],1],atol=1e-12)


def test_historical_complete_recalibration_is_preserved():
    import hashlib
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    source=ROOT/'docs/multi_agent/piper_handeye_recalib_20261117_result.json'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest=='fad58acb3ee96f3f3a57b8e0c27b7bbcac900b4116c3ee82267a1585f6523656'
    result=json.loads(source.read_text())
    historical=json.loads((ROOT/'config/piper_vision_grasp/provenance/handeye-recalib-20261117-result.json').read_text())
    assert historical==result
    transform=np.array(historical['T_base_color_optical'])
    inverse=np.array(result['T_color_optical_base'])
    assert np.allclose(transform@inverse,np.eye(4),atol=1e-12)
    assert np.allclose(transform[:3,3],[.35848696711616207,.038146424141330196,-.05656711561965355],atol=1e-12,rtol=0)


def test_historical_extrinsic_applies_recorded_translation_once_and_preserves_rotation():
    historical=json.loads((ROOT/'config/piper_vision_grasp/provenance/handeye-manual-translation-20261117.json').read_text())
    recorded=json.loads((ROOT/'docs/multi_agent/piper_handeye_translation_candidate_20261117.json').read_text())
    actual=np.array(historical['T_base_color_optical'])
    baseline=np.array(recorded['baseline_T_base_color_optical'])
    assert historical['T_base_color_optical']==recorded['candidate_T_base_color_optical']
    assert np.array_equal(actual[:3,:3],baseline[:3,:3])
    assert np.allclose(actual[:3,3]-baseline[:3,3],recorded['delta_translation_base_m'],atol=1e-12,rtol=0)
    assert np.allclose(actual[:3,:3]@recorded['point_color_m']+actual[:3,3],recorded['tcp_base_m'],atol=1e-12,rtol=0)
    assert historical['physical_validation']=='NOT_PERFORMED'


def test_current_extrinsic_uses_full_r3_100mm_result():
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json')
    result=json.loads((ROOT/'docs/multi_agent/piper_handeye_r3_100mm_20plus8_result.json').read_text())
    assert cfg['extrinsics']['T_base_color_optical']==result['T_base_color_optical']
    assert cfg['extrinsics']['physical_validation']==result['physical_validation']
    assert result['board']['marker_size_mm']==100
    assert (result['training']['count'],result['validation']['count'])==(20,8)


def angle_scene():
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    cfg['semantic']['angle_search']=dict(pitch_deg=[40.,0.],yaw_offset_deg=[0.,90.])
    cfg['extrinsics']['T_base_color_optical']=np.eye(4).tolist()
    cfg['semantic']['grasp_offset_base_m']=[0,0,0]
    cfg['semantic']['entry_clearance_m']=.1
    obs=dict(point_color_m=[.5,0,.1],grasp_axis_base=[0,1,0],grasp_width_m=.04,
             visible_hull_base_xy_m=[[.47,-.02],[.53,-.02],[.53,.02],[.47,.02]])
    return cfg,obs


def test_angle_search_retries_whole_path_then_stops_and_preserves_input(monkeypatch):
    from piper_vision_grasp import geometry as g,main
    cfg,obs=angle_scene();before=copy.deepcopy(cfg);calls=[];progress=[]
    def planner(joints,active,target):
        calls.append(copy.deepcopy(target))
        assert 'pregrasp_offset_base_m' in target and 'entry_offset_base_m' in target
        return dict(status='PLANNED' if len(calls)==3 else 'WAIT_IK',target=target,
                    reason='local entry branch unavailable',failed=dict(position_error_m=.002))
    monkeypatch.setattr(g,'plan_absolute',planner)
    result=main.make_plan(cfg,[0]*6,obs,None,progress.append)
    assert len(calls)==3 and len(progress)==3
    assert [(r['approach_pitch_deg'],r['approach_yaw_offset_deg']) for r in calls]==[(40.,0.),(0.,0.),(40.,90.)]
    assert result['status']=='PLANNED' and result['selected_candidate_index']==2
    assert [r['status'] for r in result['orientation_candidates']]==['WAIT_IK','WAIT_IK','PLANNED']
    assert cfg==before and all(np.allclose(np.array(t['T_base_tcp'])[:3,3],obs['point_color_m']) for t in calls)
    assert calls[-1]['grasp_width_m']==pytest.approx(.06)
    assert calls[0]['grasp_width_m']==pytest.approx(.04)
    for target in calls:
        tcp=np.array(target['T_base_tcp']);flange=np.array(target['T_base_flange'])
        assert np.allclose(tcp[:3,:3].T@tcp[:3,:3],np.eye(3))
        assert np.linalg.det(tcp[:3,:3])==pytest.approx(1)
        assert np.allclose(target['pregrasp_offset_base_m'],-tcp[:3,2]*.1)
        assert np.allclose(target['entry_offset_base_m'],-tcp[:3,2]*.1)
        assert np.allclose(flange[:3,3],tcp[:3,3]-tcp[:3,:3]@cfg['semantic']['tcp_offset_m'])
    assert abs(calls[1]['pregrasp_offset_base_m'][2])<1e-12


def test_angle_search_all_rejected_preserves_failures_without_executable_stages(monkeypatch):
    from piper_vision_grasp import geometry as g
    cfg,obs=angle_scene()
    monkeypatch.setattr(g,'plan_absolute',lambda *args:dict(status='WAIT_IK',reason='unreachable',failed=dict(position_error_m=.1)))
    result=g.search_orientations([0]*6,cfg,obs)
    assert result['status']=='WAIT_IK' and result['selected_candidate_index'] is None
    assert len(result['orientation_candidates'])==4
    assert 'stages' not in result
    assert all(row['failed']['position_error_m']==.1 for row in result['orientation_candidates'])


@pytest.mark.parametrize('settings',[
    dict(pitch_deg=[],yaw_offset_deg=[0]),dict(pitch_deg=[float('nan')],yaw_offset_deg=[0]),
    dict(pitch_deg=[-10],yaw_offset_deg=[0]),dict(pitch_deg=[40],yaw_offset_deg=[181]),
])
def test_angle_search_invalid_grid_never_calls_ik(monkeypatch,settings):
    from piper_vision_grasp import geometry as g
    cfg,obs=angle_scene();cfg['semantic']['angle_search']=settings
    def forbidden(*args):raise AssertionError('invalid grid reached IK')
    monkeypatch.setattr(g,'plan_absolute',forbidden)
    with pytest.raises(ValueError,match='angle search'):g.search_orientations([0]*6,cfg,obs)


def test_manual_pitch_override_keeps_explicit_single_orientation():
    from piper_vision_grasp import main
    cfg,_=angle_scene();main.configure_target(cfg,approach_pitch=50)
    assert cfg['semantic']['approach_pitch_deg']==50
    assert 'angle_search' not in cfg['semantic']


def test_depth_hull_supports_rotated_width_and_prompt_has_no_top_grasp_instruction():
    rgb,depth,cam,cfg,sel=scene()
    obs=module().measure(rgb,depth,cam,cfg,sel)
    hull=np.array(obs['visible_hull_base_xy_m'])
    assert np.allclose(np.ptp(hull,axis=0),[.195,.195])
    assert 'vertically from above' not in module().target_prompt(cfg['target'])


def test_entry_follows_candidate_approach_instead_of_base_vertical(monkeypatch):
    from piper_vision_grasp import geometry as g
    cfg,obs=angle_scene();cfg['semantic'].update(approach_pitch_deg=0.,pregrasp_offset_base_m=[-.1,0,0],entry_offset_base_m=[-.1,0,0])
    target=g.absolute_target(obs,cfg);reference={}
    monkeypatch.setattr(g,'solve_ik',lambda *a:dict(status='SOLVED',angles_deg=[0]*6))
    def motion(joints,ref,target,model):reference.update(ref);return dict(status='PLANNED')
    monkeypatch.setattr(g,'plan_motion',motion)
    g.plan_absolute([0]*6,cfg,target)
    assert reference['entry_offset_base_m']==[-.1,0,0]


def test_local_branch_queries_sideways_entry_not_vertical(monkeypatch):
    from piper_vision_grasp import geometry as g
    cfg,_=angle_scene();model=cfg['kinematics'];q=[0,100,-70,0,30,0];goal=g.fk(q,model);queries=[]
    def solve(pose,seed,model):
        queries.append(pose.copy())
        return dict(status='SOLVED',angles_deg=q,position_error_m=0.,rotation_error_deg=0.)
    monkeypatch.setattr(g,'solve_ik',solve)
    ref=dict(joints_deg=q,transition_joints_deg=[],pregrasp_offset_base_m=[-.1,0,0],
             entry_clearance_m=.1,entry_offset_base_m=[-.1,0,0],lift_m=.02,sample_spacing_m=.01,reference_id='test')
    g.plan_motion(q,ref,dict(T_base_flange=goal.tolist()),model)
    assert np.allclose(queries[11][:3,3],goal[:3,3]+[-.2,0,0])


def test_angle_search_skips_direction_wider_than_existing_gripper_opening(monkeypatch):
    from piper_vision_grasp import geometry as g
    cfg,obs=angle_scene();cfg['semantic']['angle_search']=dict(pitch_deg=[40],yaw_offset_deg=[90,0])
    obs['visible_hull_base_xy_m']=[[.44,-.02],[.56,-.02],[.56,.02],[.44,.02]]
    calls=[]
    def planner(joints,config,target):
        calls.append(target);return dict(status='PLANNED',target=target)
    monkeypatch.setattr(g,'plan_absolute',planner)
    result=g.search_orientations([0]*6,cfg,obs)
    assert result['selected_candidate_index']==1 and len(calls)==1
    assert result['orientation_candidates'][0]['status']=='WAIT_WIDTH'
    assert result['orientation_candidates'][0]['target']['grasp_width_m']==pytest.approx(.12)
    assert result['target']['grasp_width_m']==pytest.approx(.04)


@pytest.mark.parametrize('name,clearance',[('semantic.json',0.),('semantic-camera-avoid.json',.1)])
def test_search_preserves_each_profiles_existing_entry_distance(monkeypatch,name,clearance):
    from piper_vision_grasp import geometry as g
    cfg=g.load_config(ROOT/'config/piper_vision_grasp'/name);_,obs=angle_scene()
    monkeypatch.setattr(g,'plan_absolute',lambda joints,cfg,target:dict(status='PLANNED',target=target))
    result=g.search_orientations([0]*6,cfg,obs)
    assert np.linalg.norm(result['target']['entry_offset_base_m'])==pytest.approx(clearance)


@pytest.mark.parametrize('tolerance,error_m,rotation_deg,status',[
    (.008,.0079,0,'SOLVED'),(.008,.0081,0,'WAIT_IK'),
    (.008,.001,1.1,'WAIT_IK'),(.001,.0079,0,'WAIT_IK'),
])
def test_ik_position_tolerance_changes_only_acceptance(monkeypatch,tolerance,error_m,rotation_deg,status):
    from piper_vision_grasp import geometry as g
    from types import SimpleNamespace
    cfg=g.load_config(ROOT/'config/piper_vision_grasp/semantic.json')
    model=dict(cfg['kinematics'],ik_position_tolerance_m=tolerance);q=[0,100,-70,0,30,0]
    goal=g.fk(q,model);goal[0,3]+=error_m
    goal[:3,:3]=goal[:3,:3]@g.rotation_matrix(g.Rotation.from_euler('z',rotation_deg,degrees=True))
    monkeypatch.setattr(g,'least_squares',lambda fun,seed,**kw:SimpleNamespace(x=np.radians(q)))
    result=g.solve_ik(goal,q,model)
    assert result['status']==status and result['position_error_m']==pytest.approx(error_m)
    assert result['position_tolerance_m']==tolerance
    assert result['rotation_tolerance_deg']==1


def test_absolute_plan_propagates_tolerance_without_mutating_model(monkeypatch):
    from piper_vision_grasp import geometry as g
    cfg,obs=angle_scene();cfg['semantic']['ik_position_tolerance_m']=.008
    original=copy.deepcopy(cfg['kinematics']);target=g.absolute_target(obs,cfg);seen=[]
    def solve(pose,seed,model):
        seen.append(model['ik_position_tolerance_m']);return dict(status='SOLVED',angles_deg=[0]*6)
    def motion(joints,ref,target,model):
        seen.append(model['ik_position_tolerance_m']);return dict(status='PLANNED')
    monkeypatch.setattr(g,'solve_ik',solve);monkeypatch.setattr(g,'plan_motion',motion)
    g.plan_absolute([0]*6,cfg,target)
    assert seen==[.008,.008] and cfg['kinematics']==original


@pytest.mark.parametrize('value',[0.,-.001,float('nan')])
def test_invalid_ik_tolerance_rejected_before_solver(monkeypatch,value):
    from piper_vision_grasp import geometry as g
    cfg,_=angle_scene();model=dict(cfg['kinematics'],ik_position_tolerance_m=value)
    def forbidden(*a,**kw):raise AssertionError('invalid tolerance reached solver')
    monkeypatch.setattr(g,'least_squares',forbidden)
    with pytest.raises(ValueError,match='IK position tolerance'):g.solve_ik(np.eye(4),[0]*6,model)
