"""Offline contracts for the local, one-cycle Piper visual grasp experiment."""
import copy
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
CONFIG = ROOT / 'config/piper_vision_grasp'
FIXTURES = ROOT / 'test/fixtures/piper_vision_grasp'


def read(path):
    return json.loads(path.read_text())


def test_reference_provenance_and_phase():
    assert (CONFIG / 'reference.json').is_file(), 'versioned reference is missing'
    ref = read(CONFIG / 'reference.json')
    assert ref['joints_deg'] == [4.296, 124.232, -78.909, -28.320, -19.013, 29.113]
    assert ref['phase'] == 'before_close'
    assert max(ref['timestamps']['rgbd']) < ref['timestamps']['close'] < ref['timestamps']['lift']
    for path, digest in ref['source_sha256'].items():
        assert hashlib.sha256((CONFIG / path).read_bytes()).hexdigest() == digest
    manifest = read(FIXTURES / 'manifest.json')
    for sample in manifest['samples']:
        for name, digest in sample['sha256'].items():
            assert hashlib.sha256((FIXTURES / sample['directory'] / name).read_bytes()).hexdigest() == digest
    assert manifest['samples'][-1]['phase'] == 'after_lift'
    assert read(CONFIG / 'extrinsics.json')['physical_validation'] == 'NOT_PERFORMED'


def test_reference_rejects_corrupt_or_missing_source(tmp_path):
    assert (ROOT / 'tools/piper_vision_grasp/geometry.py').is_file()
    from piper_vision_grasp.geometry import validate_reference
    ref = read(CONFIG / 'reference.json')
    with pytest.raises(ValueError, match='source'):
        validate_reference(ref, tmp_path)
    ref['source_sha256'][next(iter(ref['source_sha256']))] = '0' * 64
    with pytest.raises(ValueError, match='source'):
        validate_reference(ref, CONFIG)
    ref = read(CONFIG / 'reference.json')
    ref['phase'] = 'after_lift'
    with pytest.raises(ValueError, match='phase'):
        validate_reference(ref, CONFIG)


def vision_module():
    assert (ROOT / 'tools/piper_vision_grasp/vision.py').is_file(), 'detector not implemented'
    return importlib.import_module('piper_vision_grasp.vision')


def sample(name='blue-block-after-retreat-rgbd01', index=0):
    import cv2
    directory = FIXTURES / name
    meta = read(directory / 'metadata.json')
    frame = meta['frames'][index]
    rgb = cv2.cvtColor(cv2.imread(str(directory / (frame['stem'] + '-color.png'))), cv2.COLOR_BGR2RGB)
    depth = np.load(directory / (frame['stem'] + '-depth.npz'))['aligned_depth']
    camera = dict(meta['color_intrinsics'], depth_scale=meta['depth_scale'],
                  aligned_intrinsics=frame['aligned_intrinsics'])
    return rgb, depth, camera


def detect(rgb=None, depth=None, camera=None):
    original = sample()
    return vision_module().detect_blue_block(
        original[0] if rgb is None else rgb,
        original[1] if depth is None else depth,
        original[2] if camera is None else camera, read(CONFIG / 'config.json')['detector'])


def test_real_blue_face_and_crosscheck():
    first = detect()
    assert first['status'] == 'DETECTED', first
    assert .037 < first['width_m'] < .047
    assert .108 < first['height_m'] < .127
    second = detect(*sample('blue-block-half-horizontal-rgbd01'))
    assert second['status'] == 'DETECTED', second
    assert np.linalg.norm(np.array(first['point_color_m']) - second['point_color_m']) < .003


def test_zero_depth_no_target():
    out = detect(depth=np.zeros((480, 640), dtype=np.uint16))
    assert out['status'] == 'WAIT_DEPTH' and out['point_color_m'] is None


@pytest.mark.parametrize('kind', ['two', 'cropped', 'occluded', 'absent'])
def test_incomplete_or_ambiguous_target(kind):
    rgb, depth, camera = sample()
    if kind == 'two':
        rgb[80:440, 70:235] = rgb[80:440, 278:443]
    elif kind == 'cropped':
        rgb = np.roll(rgb, -280, axis=1)
    elif kind == 'occluded':
        rgb[180:260, 280:440] = 0
    else:
        rgb[:] = 0
    out = detect(rgb, depth, camera)
    assert out['status'].startswith('WAIT_') and out['point_color_m'] is None


@pytest.mark.parametrize('kind', ['shape', 'intrinsics', 'distortion', 'nan'])
def test_invalid_camera_or_depth(kind):
    rgb, depth, camera = sample()
    if kind == 'shape': depth = depth[:100]
    if kind == 'intrinsics': camera['aligned_intrinsics']['fx'] += 10
    if kind == 'distortion': camera['coeffs'][0] = .1
    if kind == 'nan': depth = np.full(depth.shape, np.nan)
    out = detect(rgb, depth, camera)
    assert out['status'].startswith('WAIT_') and out['point_color_m'] is None


def test_object_lift_requires_visible_stable_object_motion():
    v = vision_module()
    before = detect()
    ext = read(CONFIG / 'extrinsics.json')
    r = np.array(ext['T_base_color_optical'])[:3, :3]
    after = copy.deepcopy(before)
    after['point_color_m'] = (np.array(before['point_color_m']) + r.T @ [0, 0, .02]).tolist()
    assert v.verify_lift(before, [after] * 3, [0, 0, .02], ext)['status'] == 'VERIFIED'
    assert v.verify_lift(before, [before] * 3, [0, 0, .02], ext)['status'] == 'WAIT_VERIFY'
    after['height_m'] *= .7  # occlusion changes the mask centroid
    assert v.verify_lift(before, [after] * 3, [0, 0, .02], ext)['status'] == 'WAIT_VERIFY'
    hold = detect(*sample('blue-block-grasp-hold-rgbd01'))
    assert v.verify_lift(before, [hold], [0, 0, .02], ext)['status'] == 'WAIT_VERIFY'


def test_raw_depth_rotation_column_major():
    v = vision_module()
    cam = dict(width=1, height=1, fx=1, fy=1, ppx=0, ppy=0, coeffs=[0]*5)
    meta = dict(depth_intrinsics=cam, depth_scale=.001,
                depth_to_color=dict(rotation=[0, 0, -1, 0, 1, 0, 1, 0, 0], translation=[.1, 0, 0]))
    assert np.allclose(v.raw_depth_to_color(np.array([[1000]]), meta), [[1.1, 0, 0]])


def geometry_module():
    return importlib.import_module('piper_vision_grasp.geometry')


def test_delta_zero_reproduces_reference_and_no_second_tcp_offset():
    g = geometry_module()
    assert hasattr(g, 'retarget'), 'retarget missing'
    ref = read(CONFIG / 'reference.json')
    target = g.retarget(ref, ref['observation'], read(CONFIG / 'extrinsics.json'))
    assert np.allclose(target['T_base_flange'], ref['T_base_flange'])
    assert np.allclose(target['delta_base_m'], [0, 0, 0])


def test_camera_delta_rotates_into_base_and_translation_cancels():
    g = geometry_module()
    assert hasattr(g, 'retarget')
    ref = read(CONFIG / 'reference.json')
    ext = read(CONFIG / 'extrinsics.json')
    obs = copy.deepcopy(ref['observation'])
    obs['point_color_m'][0] += .01
    result = g.retarget(ref, obs, ext)
    assert np.allclose(result['delta_base_m'], np.array(ext['T_base_color_optical'])[:3, 0]*.01)
    for row in ext['T_base_color_optical'][:3]: row[3] += 10
    assert np.allclose(result['T_base_flange'], g.retarget(ref, obs, ext)['T_base_flange'])


def test_fk_matches_sdk_zero_and_success():
    g = geometry_module()
    assert hasattr(g, 'fk')
    model, ref = read(CONFIG/'kinematics.json'), read(CONFIG/'reference.json')
    for joints, sdk in [([0]*6, [56.127, 0, 213.266]), (ref['joints_deg'], [476.249,49.884,153.501])]:
        assert np.linalg.norm(g.fk(joints, model)[:3, 3] - np.array(sdk)/1000) < .001


def test_plan_responds_to_visual_motion_and_current_joints():
    g = geometry_module()
    assert hasattr(g, 'plan_motion')
    ref, model, ext = read(CONFIG/'reference.json'), read(CONFIG/'kinematics.json'), read(CONFIG/'extrinsics.json')
    target = g.retarget(ref, ref['observation'], ext)
    plan = g.plan_motion([0]*6, ref, target, model)
    assert plan['status'] == 'PLANNED', plan
    moved = copy.deepcopy(ref['observation'])
    moved['point_color_m'] = (np.array(moved['point_color_m']) + np.array(ext['T_base_color_optical'])[:3,:3].T @ [0,.02,0]).tolist()
    other = g.plan_motion(ref['pregrasp_joints_deg'], ref, g.retarget(ref,moved,ext), model)
    assert other['status'] == 'PLANNED', other
    assert other['initial_joints_deg'] != plan['initial_joints_deg']
    a = next(s for s in plan['stages'] if s['stage_id']=='grasp')
    b = next(s for s in other['stages'] if s['stage_id']=='grasp')
    assert np.allclose(np.array(b['T_base_flange'])[:3,3]-np.array(a['T_base_flange'])[:3,3], [0,.02,0])
    assert not np.allclose(a['angles_deg'],b['angles_deg'])
    assert all(s['position_error_m'] <= .001 and s['rotation_error_deg'] <= 1 for s in plan['stages'])
    moved['point_color_m'][0] += .5
    with pytest.raises(ValueError, match='local'):
        g.retarget(ref,moved,ext)


def test_invalid_ik_and_joint_limits():
    g = geometry_module()
    assert hasattr(g, 'solve_ik')
    model = read(CONFIG/'kinematics.json')
    unreachable = np.eye(4); unreachable[0,3]=3
    assert g.solve_ik(unreachable, [0]*6, model)['status'] == 'WAIT_IK'
    with pytest.raises(ValueError, match='limit'):
        g.fk([999]*6, model)


def test_old_scipy_rotation_api(monkeypatch):
    g = geometry_module()
    assert hasattr(g, 'rotation_matrix')
    from scipy.spatial.transform import Rotation
    class OldRotation:
        @staticmethod
        def from_dcm(matrix): return Rotation.from_matrix(matrix)
    assert np.allclose(g.rotation_from_matrix(np.eye(3), OldRotation).as_rotvec(), [0]*3)
    class OldResult:
        def as_dcm(self): return np.eye(3)
    assert np.allclose(g.rotation_matrix(OldResult()), np.eye(3))


def io_module():
    assert (ROOT/'tools/piper_vision_grasp/piper_io.py').is_file(), 'session not implemented'
    return importlib.import_module('piper_vision_grasp.piper_io')


def fake_arm(io):
    from types import SimpleNamespace as NS
    flags = {k:False for k in set(io.GRIPPER_FAULTS + io.common.FAULTS)}
    flags['driver_enable_status'] = True
    motors = NS(**{'motor_'+str(i):NS(foc_status=NS(**flags)) for i in range(1,7)})
    state = NS(ctrl_mode=1,arm_status=0,err_code=0,teach_status=0,motion_status=0)
    joints = NS(joint_state=NS(**{'joint_'+str(i):0 for i in range(1,7)}),time_stamp=1.)
    pose = NS(end_pose=NS(**{k:0 for k in ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')}),time_stamp=1.)
    grip = NS(gripper_state=NS(grippers_angle=68000,grippers_effort=500,foc_status=NS(**flags)),time_stamp=1.)
    a=NS(GetArmStatus=lambda:NS(arm_status=state),GetArmLowSpdInfoMsgs=lambda:motors,
         GetArmJointMsgs=lambda:joints,GetArmEndPoseMsgs=lambda:pose,GetArmGripperMsgs=lambda:grip,calls=[])
    a.MotionCtrl_2=lambda *args:a.calls.append(('mode',args))
    a.JointCtrl=lambda *args:a.calls.append(('joint',args))
    a.GripperCtrl=lambda *args:a.calls.append(('gripper',args))
    a.DisconnectPort=lambda:None
    return a


def ready_session(tmp_path, monkeypatch, readonly=False):
    io=io_module()
    c=read(CONFIG/'config.json');c.update(readonly=readonly,run_dir=str(tmp_path),run_id='unit-test')
    session=io.PiperSession(c)
    session.arm=fake_arm(io)
    session.feedback=io.Feedback()
    now=io.time.monotonic()
    session.feedback.seen={i:now for i in io.FEEDBACK_IDS}
    session.feedback.limits={i:(-180000,180000,now) for i in range(1,7)}
    session.feedback.max_width=(100000,now)
    monkeypatch.setattr(session,'poll',lambda:None)
    return session


def test_attempt_recorded_before_partial_send(tmp_path, monkeypatch):
    s=ready_session(tmp_path,monkeypatch)
    def partial(*args):
        receipts=list((tmp_path/'attempts').glob('*.json'))
        assert len(receipts)==1
        assert read(receipts[0])['stage_id']=='approach'
        s.arm.calls.append(('partial',args));raise OSError('partial send')
    s.arm.MotionCtrl_2=partial
    out=s.attempt_joint('approach',[0,10,-5,0,0,0],5)
    assert out['state']=='COMMAND_UNCERTAIN'
    assert len(s.arm.calls)==1
    assert s.attempt_joint('approach',[0,10,-5,0,0,0],5)['state']=='COMMAND_UNCERTAIN'
    assert len(s.arm.calls)==1


@pytest.mark.parametrize('name', ['config.json', 'semantic.json', 'semantic-camera-avoid.json', 'semantic-graspnet.json'])
def test_grasp_config_uses_faster_joint_speed(name):
    config=geometry_module().load_config(CONFIG/name)
    assert config['motion']['speed_percent']==30


@pytest.mark.parametrize('name', ['config.json', 'semantic-graspnet.json'])
@pytest.mark.parametrize('speed', [-1, 101, 30.5, True, '30', None])
def test_config_rejects_invalid_joint_speed(monkeypatch, name, speed):
    original=json.loads
    def invalid(text, *args, **kwargs):
        value=original(text,*args,**kwargs)
        if 'motion' in value:value['motion']['speed_percent']=speed
        return value
    monkeypatch.setattr(json,'loads',invalid)
    with pytest.raises(ValueError,match='speed'):
        geometry_module().load_config(CONFIG/name)


@pytest.mark.parametrize('speed', [0, 5, 30, 100])
def test_joint_speed_reaches_sdk_and_receipt(tmp_path, monkeypatch, speed):
    from types import SimpleNamespace as NS
    s=ready_session(tmp_path,monkeypatch)
    s.config['motion']['speed_percent']=speed
    events=NS(recovery=False,attempted=False,emit=lambda *args:None)
    monkeypatch.setattr(main_module().time,'sleep',lambda _:pytest.fail('joint command did not send'))
    main_module().send_once(s.config,s,events,'approach','joint',[0,10,-5,0,0,0])
    assert s.arm.calls==[('mode',(1,1,speed,0)),('joint',(0,10000,-5000,0,0,0))]
    assert read(tmp_path/'attempts/approach.json')['speed_percent']==speed


@pytest.mark.parametrize('speed', [-1, 101, 30.5, True, '30', None])
def test_invalid_joint_speed_never_sends(tmp_path, monkeypatch, speed):
    s=ready_session(tmp_path,monkeypatch)
    with pytest.raises(ValueError,match='speed'):
        s.attempt_joint('approach',[0,10,-5,0,0,0],speed)
    assert s.arm.calls==[]
    assert not (tmp_path/'attempts').exists()


def test_reconnect_never_replays_stage(tmp_path, monkeypatch):
    s=ready_session(tmp_path,monkeypatch)
    assert s.attempt_gripper('open',70,.5)['command_attempted']
    assert s.arm.calls==[('gripper',(70000,500,1,0))]
    other=ready_session(tmp_path,monkeypatch)
    assert other.attempt_gripper('open',70,.5)['state']=='COMMAND_UNCERTAIN'
    assert other.arm.calls==[]


@pytest.mark.parametrize('partial', [False, True])
def test_standby_gripper_selects_can_mode_after_receipt(tmp_path, monkeypatch, partial):
    s=ready_session(tmp_path,monkeypatch)
    s.arm.GetArmStatus().arm_status.ctrl_mode=0
    def mode(*args):
        assert read(tmp_path/'attempts/open.json')['speed_percent']==30
        s.arm.calls.append(('mode',args))
        if partial:raise OSError('mode write uncertain')
    s.arm.MotionCtrl_2=mode
    result=s.attempt_gripper('open',70,.5)
    expected=[('mode',(1,1,30,0))]
    if not partial:expected.append(('gripper',(70000,500,1,0)))
    assert s.arm.calls==expected
    assert result['state']==('COMMAND_UNCERTAIN' if partial else 'COMMAND_ATTEMPTED')
    s.attempt_gripper('open',70,.5)
    assert s.arm.calls==expected


@pytest.mark.parametrize('stage,effort', [('open', 1.), ('close', .5)])
def test_open_effort_override_preserves_closing_effort(tmp_path, monkeypatch, stage, effort):
    from types import SimpleNamespace as NS
    s=ready_session(tmp_path,monkeypatch)
    s.config['gripper']['open_effort_nm']=1.
    m=main_module()
    events=NS(recovery=False,attempted=False,emit=lambda *args:None)
    m.send_once(s.config,s,events,stage,'gripper',70 if stage=='open' else 45)
    assert s.arm.calls==[('gripper',(70000 if stage=='open' else 45000,round(effort*1000),1,0))]
    assert read(tmp_path/'attempts'/(stage+'.json'))['target'][1]==round(effort*1000)


def test_joint_arrival_uses_measured_error_with_unreached_firmware_flag(monkeypatch):
    from types import SimpleNamespace as NS
    m=main_module();config=read(CONFIG/'config.json');states=[]
    rows=iter([dict(state='READY',missing_feedback=[],motion_status=1,joints_deg=[1,0,0,0,0,0]),
               dict(state='READY',missing_feedback=[],motion_status=1,joints_deg=[.2,0,0,0,0,0])])
    session=NS(snapshot=lambda:next(rows))
    events=NS(emit=lambda state,*args,**kwargs:states.append(state))
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    result=m.wait_joints(config,session,events,dict(stage_id='entry-1',angles_deg=[0]*6))
    assert states==['WAIT_TARGET','AT_TARGET']
    assert result['joints_deg'][0]==.2


def test_stale_feedback_cannot_advance(tmp_path,monkeypatch):
    s=ready_session(tmp_path,monkeypatch)
    s.feedback.seen[0x2A5]-=2
    assert s.attempt_joint('approach',[0,10,-5,0,0,0],5)['state']=='WAIT_FEEDBACK'
    assert s.arm.calls==[]


def test_gripper_fault_waits(tmp_path,monkeypatch):
    s=ready_session(tmp_path,monkeypatch)
    s.arm.GetArmGripperMsgs().gripper_state.foc_status.driver_error_status=True
    assert s.attempt_gripper('close',36,.5)['state']=='WAIT_GRIPPER'
    assert s.arm.calls==[]


def test_readonly_session_cannot_transmit(tmp_path,monkeypatch):
    s=ready_session(tmp_path,monkeypatch,readonly=True)
    assert s.snapshot()['state']=='READY'
    assert s.attempt_gripper('open',70,.5)['state']=='READ_ONLY'
    assert s.attempt_joint('move',[0,10,-5,0,0,0],5)['state']=='READ_ONLY'
    assert s.arm.calls==[]


def test_send_invalidates_cached_feedback(tmp_path,monkeypatch):
    s=ready_session(tmp_path,monkeypatch)
    s.attempt_joint('move',[0,10,-5,0,0,0],5)
    assert s.snapshot()['state']=='WAIT_FEEDBACK'
    assert s.arm.calls==[('mode',(1,1,5,0)),('joint',(0,10000,-5000,0,0,0))]


def test_import_and_constructor_have_no_device_side_effects(monkeypatch,tmp_path):
    import builtins
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        assert name not in ('can','piper_sdk','pyrealsense2'), 'hardware dependency imported eagerly'
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    io=io_module();importlib.reload(io)
    io.PiperSession(dict(read(CONFIG/'config.json'),run_dir=str(tmp_path),run_id='offline'))


def main_module():
    assert (ROOT/'tools/piper_vision_grasp/main.py').is_file(), 'runner missing'
    return importlib.import_module('piper_vision_grasp.main')


@pytest.mark.parametrize('start', [[11.913,143.336,-105.329,-52.211,12.617,60.007], [0]*6])
def test_return_zero_only_sends_segmented_joints(tmp_path, monkeypatch, start):
    m=main_module();config=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json')
    session=FakeSession(fault_after_send=True);session.joints=list(start)
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    result=m.run_zero(config,session,tmp_path)
    assert result['state']=='ZERO_REACHED'
    assert result['measured']['joints_deg']==[0]*6
    assert result['measured']['gripper_width_mm']==68.
    assert all(call[1]=='joint' for call in session.calls)
    targets=[start]+[call[2] for call in session.calls]
    assert np.max(np.abs(np.diff(targets,axis=0)))<=10.000001
    assert len(session.calls)==(15 if start[0] else 1)
    assert read(tmp_path/'plan.json')['speed_percent']==30
    states=[json.loads(line)['state'] for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert 'WAIT_FEEDBACK' in states
    assert states.count('AT_TARGET')>=len(session.calls)


@pytest.mark.parametrize('failure', ['uncertain','runtime','invalid'])
def test_return_zero_waits_without_replay_on_failure(tmp_path, monkeypatch, failure):
    m=main_module();config=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json')
    session=FakeSession();session.joints=[0,20,-10,0,0,0]
    if failure=='invalid':session.joints=[float('nan')]*6
    attempts=[]
    def attempt(*args):
        attempts.append(args)
        if failure=='runtime':raise RuntimeError('partial send')
        return dict(state='COMMAND_UNCERTAIN',command_attempted=True,reason='partial write')
    session.attempt_joint=attempt
    sleeps=[]
    def stop(_):
        sleeps.append(1)
        if len(sleeps)==3:raise KeyboardInterrupt
    monkeypatch.setattr(m.time,'sleep',stop)
    result=m.run_zero(config,session,tmp_path)
    assert result['state']=='EXITED' and result['stop_confirmed'] is None
    assert len(attempts)==(0 if failure=='invalid' else 1)
    assert result['previous_state']==('WAIT_PLAN' if failure=='invalid' else 'COMMAND_UNCERTAIN')
    assert result['command_attempted']==(failure!='invalid')


def test_zero_cli_uses_writable_session_without_camera(tmp_path,monkeypatch):
    m=main_module();sessions=[]
    class Session(FakeSession):
        def __init__(self,config):
            super().__init__();assert config['readonly'] is False
            self.joints=[0,20,-10,0,0,0];sessions.append(self)
        def __enter__(self):return self
        def __exit__(self,*_):pass
        def attempt_joint(self,stage_id,angles_deg,speed_percent):
            assert speed_percent==30
            return super().attempt_joint(stage_id,angles_deg,speed_percent)
    monkeypatch.setattr(m,'PiperSession',Session)
    monkeypatch.setattr(m.vision,'capture_rgbd',lambda *a,**k:pytest.fail('zero opened camera'))
    monkeypatch.setattr(m.shutil,'which',lambda _:None)
    monkeypatch.setattr(sys,'argv',['main.py','zero','--output-root',str(tmp_path),'--run-id','zero-test'])
    assert m.main()==0
    assert read(tmp_path/'zero-test/result.json')['state']=='ZERO_REACHED'
    assert len(sessions)==1 and len(sessions[0].calls)==2
    assert m.main()==2
    assert len(sessions)==1


@pytest.mark.parametrize('failure',['not_arrived','storage'])
def test_zero_does_not_continue_after_stall_or_storage_error(tmp_path,monkeypatch,failure):
    m=main_module();config=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json')
    session=FakeSession();session.joints=[0,20,-10,0,0,0]
    def stalled(*args):
        session.calls.append(args)
        return dict(state='COMMAND_ATTEMPTED',command_attempted=True)
    session.attempt_joint=stalled
    original_save=m.save
    def save(path,value):
        if Path(path).name=='plan.json':raise OSError('disk unavailable')
        return original_save(path,value)
    if failure=='storage':monkeypatch.setattr(m,'save',save)
    sleeps=[]
    def stop(_):
        sleeps.append(1)
        if len(sleeps)==3:raise KeyboardInterrupt
    monkeypatch.setattr(m.time,'sleep',stop)
    result=m.run_zero(config,session,tmp_path)
    assert result['state']=='EXITED'
    assert result['previous_state']==('WAIT_RUNTIME' if failure=='storage' else 'WAIT_TARGET')
    assert len(session.calls)==(0 if failure=='storage' else 1)


def test_zero_waits_when_diagnostic_storage_recovers(tmp_path,monkeypatch):
    m=main_module();config=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json')
    session=FakeSession();session.joints=[0,20,-10,0,0,0]
    original_emit=m.Events.emit
    sleeps=[]
    def emit(self,*args,**kwargs):
        if session.calls and len(sleeps)<2:raise OSError('disk full')
        return original_emit(self,*args,**kwargs)
    def stop(_):
        sleeps.append(1)
        if len(sleeps)==3:raise KeyboardInterrupt
    monkeypatch.setattr(m.Events,'emit',emit)
    monkeypatch.setattr(m.time,'sleep',stop)
    result=m.run_zero(config,session,tmp_path)
    assert result['state']=='EXITED' and result['previous_state']=='WAIT_RUNTIME'
    assert len(session.calls)==1
    states=[json.loads(line)['state'] for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert 'WAIT_RUNTIME' in states and 'ZERO_REACHED' not in states


class FakeSession:
    def __init__(self, empty=False, fault_after_send=False):
        self.joints=[0.]*6; self.width=68.; self.calls=[]; self.tick=0.
        self.empty=empty; self.fault_after_send=fault_after_send;self.faulted=False
    def snapshot(self):
        self.tick+=.2
        state='READY'
        if self.fault_after_send and self.calls and not self.faulted:
            state='WAIT_FEEDBACK';self.faulted=True
        return dict(state=state,reason='',joints_deg=list(self.joints),gripper_width_mm=self.width,
            gripper_enabled=True,motion_status=0,arm_status=0,err_code=0,joints_enabled=[True]*6,
            missing_feedback=[] if state=='READY' else ['0x2a5'],sample_time=self.tick)
    def poll(self): pass
    def attempt_joint(self,stage_id,angles_deg,speed_percent):
        assert stage_id not in [c[0] for c in self.calls]
        self.calls.append((stage_id,'joint',angles_deg));self.joints=list(angles_deg)
        return dict(state='COMMAND_ATTEMPTED',command_attempted=True)
    def attempt_gripper(self,stage_id,width_mm,effort_nm):
        assert stage_id not in [c[0] for c in self.calls]
        self.calls.append((stage_id,'gripper',width_mm))
        self.width=68. if width_mm==70 else 36. if self.empty else 52.8
        return dict(state='COMMAND_ATTEMPTED',command_attempted=True)


def run_fake(tmp_path,monkeypatch,kind='success'):
    m=main_module();g=geometry_module()
    config=g.load_config(CONFIG/'config.json')
    config['gripper']['stable_seconds']=.2
    config['poll_seconds']=0
    config['run_id']='test-cycle'
    session=FakeSession(empty=kind=='empty',fault_after_send=kind=='dropout')
    rgb,depth,camera=sample()
    before=detect(np.roll(rgb,12,axis=1),np.roll(depth,12,axis=1),camera)
    assert before['status']=='DETECTED'
    calls=[0]
    def capture(c,d,count=3):
        import time
        calls[0]+=1
        if calls[0]>8:raise KeyboardInterrupt
        obs=copy.deepcopy(before)
        if kind=='no_target' and calls[0]==1:obs=dict(status='WAIT_TARGET',reason='none',point_color_m=None)
        if any(s[0]=='lift' for s in session.calls):
            if kind=='occluded':obs=dict(status='WAIT_VISIBILITY',reason='occluded',point_color_m=None)
            elif kind!='not_lifted':
                r=np.array(config['extrinsics']['T_base_color_optical'])[:3,:3]
                obs['point_color_m']=(np.array(obs['point_color_m'])+r.T@[0,0,.02]).tolist()
        return dict(observation=obs,observations=[copy.deepcopy(obs) for _ in range(3)],
                    metadata=dict(serial=c['camera_serial'],frames=[dict(host_unix=time.time(),color_frame=i,depth_frame=i) for i in range(3)]))
    sleep_calls=[0]
    def sleep(_):
        sleep_calls[0]+=1
        if sleep_calls[0]>300:raise KeyboardInterrupt
    monkeypatch.setattr(m.time,'sleep',sleep)
    result=m.run_once(config,session,capture,tmp_path)
    return result,session


@pytest.mark.parametrize('kind',['success','no_target','dropout'])
def test_one_cycle_autonomous_fake(tmp_path,monkeypatch,kind):
    result,s=run_fake(tmp_path,monkeypatch,kind)
    assert result['state']=='GRASP_CONFIRMED',result
    assert s.calls[0][:2]==('open','gripper')
    assert s.calls[-1][0]=='lift'
    assert len({c[0] for c in s.calls})==len(s.calls)
    plan=read(tmp_path/'plan.json')
    assert np.linalg.norm(plan['target']['delta_base_m'])>.001
    assert next(i for i,c in enumerate(s.calls) if c[0]=='close') < next(i for i,c in enumerate(s.calls) if c[0].startswith('lift'))


@pytest.mark.parametrize('kind',['empty','not_lifted','occluded'])
def test_failure_never_claims_grasp_or_releases(tmp_path,monkeypatch,kind):
    result,s=run_fake(tmp_path,monkeypatch,kind)
    assert result['state']!='GRASP_CONFIRMED'
    assert result['grasp_confirmed'] is False
    assert len([c for c in s.calls if c[0]=='open'])==1
    if kind=='empty':assert not any(c[0].startswith('lift') for c in s.calls)


def test_offline_preview_cannot_connect_devices(tmp_path,monkeypatch):
    m=main_module();g=geometry_module()
    def forbidden(*a,**kw):raise AssertionError('hardware connected')
    monkeypatch.setattr(io_module(),'dependencies',forbidden)
    monkeypatch.setattr(vision_module(),'capture_rgbd',forbidden)
    result=m.preview_input(g.load_config(CONFIG/'config.json'),FIXTURES/'blue-block-after-retreat-rgbd01',tmp_path)
    assert result['state']=='PREVIEW_READY' and result['can_transmit_count']==0


def test_duplicate_run_id_rejected(tmp_path):
    m=main_module()
    m.reserve_run(tmp_path,'fixed')
    with pytest.raises(FileExistsError):m.reserve_run(tmp_path,'fixed')


@pytest.mark.parametrize('field,value', [('coeffs',[float('nan'),0,0,0,0]),('fx',float('nan'))])
def test_nonfinite_camera_rejected(field,value):
    camera=sample()[2];camera[field]=value
    assert detect(camera=camera)['point_color_m'] is None


def test_reference_derived_fields_must_match_sources(tmp_path):
    import shutil
    shutil.copytree(CONFIG,tmp_path/'config')
    p=tmp_path/'config/reference.json';ref=read(p)
    ref['joints_deg'][0]+=1
    # Recomputing FK must not make forged historical joint feedback valid.
    ref['T_base_flange']=geometry_module().fk(ref['joints_deg'],read(CONFIG/'kinematics.json')).tolist()
    p.write_text(json.dumps(ref))
    with pytest.raises(ValueError,match='source'):
        geometry_module().load_config(tmp_path/'config/config.json')


def test_stability_cannot_bridge_a_feedback_outage(tmp_path,monkeypatch):
    m=main_module();config=geometry_module().load_config(CONFIG/'config.json')
    events=m.Events(config,tmp_path)
    s=FakeSession()
    samples=iter([dict(s.snapshot(),sample_time=1),dict(s.snapshot(),sample_time=10)])
    def snapshot():
        try:return next(samples)
        except StopIteration:raise KeyboardInterrupt
    s.snapshot=snapshot
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    with pytest.raises(KeyboardInterrupt):m.wait_gripper(config,s,events,'open')
    assert 'OPEN_READY' not in (tmp_path/'events.jsonl').read_text()


def test_capture_failure_releases_camera(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS
    v=vision_module();calls=[]
    class Pipeline:
        def start(self,settings):calls.append('start');raise RuntimeError('start failed after partial allocation')
        def stop(self):calls.append('stop')
    settings=NS(enable_device=lambda s:calls.append(s),enable_stream=lambda *a:None)
    monkeypatch.setitem(sys.modules,'pyrealsense2',NS(context=lambda _:object(),pipeline=lambda _:Pipeline(),config=lambda:settings,
                         stream=NS(color='color',depth='depth'),format=NS(rgb8='rgb8',z16='z16')))
    with pytest.raises(RuntimeError):v.capture_rgbd(read(CONFIG/'config.json'),tmp_path/'capture')
    assert calls==['151222077264','start','stop']


def test_run_result_retains_last_wait_state_on_interrupt(tmp_path,monkeypatch):
    result,_=run_fake(tmp_path,monkeypatch,'empty')
    assert result['previous_state']=='WAIT_CONTACT'


def test_cli_rejects_offline_input_for_run_without_devices(tmp_path):
    import subprocess
    result=subprocess.run(['bash',str(ROOT/'scripts/piper_vision_grasp.sh'),'run','--input',str(FIXTURES),
                           '--output-root',str(tmp_path)],capture_output=True,text=True)
    assert result.returncode==2
    assert not list(tmp_path.iterdir())


def test_recovery_relocalizes_and_does_not_use_changed_target(tmp_path,monkeypatch):
    m=main_module();config=geometry_module().load_config(CONFIG/'config.json')
    assert hasattr(m,'revalidate_after_wait')
    events=m.Events(config,tmp_path);events.recovery=True
    before=dict(observation=detect())
    changed=copy.deepcopy(before);changed['observation']['point_color_m'][0]+=.02
    count=[0]
    def acquire(*args):
        count[0]+=1
        if count[0]>1:raise KeyboardInterrupt
        return changed,{}
    monkeypatch.setattr(m,'acquire',acquire)
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    with pytest.raises(KeyboardInterrupt):m.revalidate_after_wait(config,FakeSession(),None,events,before)
    assert events.last['state']=='WAIT_REPLAN'


@pytest.mark.parametrize('readonly',[True,False])
def test_session_connect_queries_and_cleanup(tmp_path,monkeypatch,readonly):
    from types import SimpleNamespace as NS
    io=io_module();a=fake_arm(io);calls=[]
    clock=[10.]
    monkeypatch.setattr(io.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(io.time,'time',lambda:1000+clock[0])
    monkeypatch.setattr(io.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(io.common,'discover',lambda serial:'fakecan')
    monkeypatch.setattr(io.common,'prepare_can',lambda interface,readonly:calls.append(('prepare',readonly)))
    a.ConnectPort=lambda **kw:calls.append(('connect',kw))
    a.DisconnectPort=lambda:calls.append(('disconnect',))
    a.SearchAllMotorMaxAngleSpd=lambda:calls.append(('limits',))
    class Bus:
        def __init__(self,**kwargs):calls.append(('bus',kwargs['channel']))
        def recv(self,timeout):clock[0]+=.03;return None
        def shutdown(self):calls.append(('shutdown',))
    monkeypatch.setattr(io,'dependencies',lambda:(NS(Bus=Bus),lambda interface:a))
    config=dict(read(CONFIG/'config.json'),readonly=readonly,run_id='fake',run_dir=str(tmp_path/'run'))
    with io.PiperSession(config) as s:
        s.query_kind='joint';s.poll()
        assert s.snapshot()['state']=='WAIT_FEEDBACK'
    assert ('prepare',True) in calls
    assert ('connect',dict(can_init=True,piper_init=False,start_thread=False)) in calls
    assert (('limits',) in calls)==(not readonly)
    assert calls[-2:]==[('shutdown',),('disconnect',)]
    assert a.calls==[]


@pytest.mark.parametrize('change',[{'timestamp':8.}, {'dlc':7}, {'is_extended_id':True}, {'is_fd':True}])
def test_frame_filter_rejects_old_or_malformed(change):
    from types import SimpleNamespace as NS
    io=io_module();a=fake_arm(io)
    a.ParseCANFrame=lambda _:pytest.fail('invalid frame parsed')
    fields=dict(timestamp=10.,dlc=8,is_error_frame=False,is_remote_frame=False,is_extended_id=False,is_fd=False,arbitration_id=0x2a8)
    fields.update(change)
    assert not io.Feedback().accept(a,NS(**fields),9.,10.,1.)


def test_filtered_sdk_message_cannot_refresh_feedback():
    from types import SimpleNamespace as NS
    io=io_module();a=fake_arm(io);a.ParseCANFrame=lambda _:None
    fb=io.Feedback();fb.seen[0x2a8]=1
    msg=NS(timestamp=10.,dlc=8,is_error_frame=False,is_remote_frame=False,is_extended_id=False,is_fd=False,arbitration_id=0x2a8)
    assert not fb.accept(a,msg,9.,10.,2.)
    assert 0x2a8 not in fb.seen


@pytest.mark.parametrize('pairs,error', [
    ([(21,21),(22,22),(23,23)], None),
    ([(21,21),(21,22),(22,23),(23,24)], None),
    ([(21,21),(22,21),(23,22),(24,23)], None),
    ([(21,21),(21,21),(22,22),(23,23)], None),
    ([(21,21),(20,22)], 'camera frame numbers moved backwards'),
    ([(21,21),(22,20)], 'camera frame numbers moved backwards'),
    ([(21,21)]+[(21,n) for n in range(22,62)], 'timed out waiting for distinct RGB-D frames'),
    ([(21,21)]+[(n,21) for n in range(22,62)], 'timed out waiting for distinct RGB-D frames'),
], ids=['normal','repeat-color','repeat-depth','repeat-both','rollback-color','rollback-depth','stalled-color','stalled-depth'])
@pytest.mark.parametrize('raw_only',[False,True])
def test_capture_warms_up_saves_metadata_and_stops(tmp_path,monkeypatch,pairs,error,raw_only):
    from types import SimpleNamespace as NS
    v=vision_module();calls=[];counter=[0]
    monkeypatch.setattr(v.time,'monotonic',lambda:counter[0]*.1)
    rgb,depth,camera=sample()
    intr=NS(**{k:camera[k] for k in ('width','height','fx','fy','ppx','ppy','coeffs')},model='none')
    profile=NS(as_video_stream_profile=lambda:NS(get_intrinsics=lambda:intr),
               get_extrinsics_to=lambda _:NS(rotation=np.eye(3).flatten(order='F'),translation=[0,0,0]))
    def frame(data,number):
        return NS(get_data=lambda:data,get_frame_number=lambda:number,get_timestamp=lambda:number*1000/15,
                  get_frame_timestamp_domain=lambda:'hardware_clock',profile=profile)
    class Pipeline:
        def start(self,settings):
            calls.append('start')
            return NS(get_stream=lambda _:profile,get_device=lambda:NS(first_depth_sensor=lambda:NS(get_depth_scale=lambda:.001)))
        def wait_for_frames(self,timeout):
            counter[0]+=1
            numbers=(counter[0],counter[0]) if counter[0]<=20 else pairs[min(counter[0]-21,len(pairs)-1)]
            return NS(get_color_frame=lambda:frame(rgb,numbers[0]),get_depth_frame=lambda:frame(depth,numbers[1]))
        def stop(self):calls.append('stop')
    settings=NS(enable_device=lambda serial:calls.append(serial),enable_stream=lambda *a:calls.append(a))
    monkeypatch.setitem(sys.modules,'pyrealsense2',NS(context=lambda _:object(),pipeline=lambda _:Pipeline(),config=lambda:settings,
        stream=NS(color='color',depth='depth'),format=NS(rgb8='rgb8',z16='z16'),align=lambda _:NS(process=lambda f:f)))
    if error:
        with pytest.raises(ValueError,match=error) as caught:
            v.capture_rgbd(read(CONFIG/'config.json'),tmp_path/'capture')
        assert 'previous=(21, 21)' in str(caught.value) and 'current=' in str(caught.value)
        assert calls.count('start')==1 and calls[-1]=='stop'
        assert not (tmp_path/'capture'/'metadata.json').exists()
        return
    snapshots=[]
    def paired():
        row=dict(joints_deg=[len(snapshots)*.001]*6,gripper_width_mm=48.3)
        snapshots.append(row);return row
    result=v.capture_rgbd(read(CONFIG/'config.json'),tmp_path/'capture',detect=not raw_only,snapshot=paired if raw_only else None)
    assert counter[0]==20+len(pairs) and calls.count('start')==1 and calls[-1]=='stop'
    if raw_only:
        assert 'observation' not in result and len(snapshots)==6
        assert [p['robot_snapshot'] for f in result['metadata']['frames'] for p in f['paired_robot']]==snapshots
    else:assert result['observation']['status']=='DETECTED'
    expected=[pairs[0]]+pairs[-2:]
    assert [(f['color_frame'],f['depth_frame']) for f in result['metadata']['frames']]==expected
    assert result['metadata']['frames'][0]['color_time_domain']=='hardware_clock'
    assert len(list((tmp_path/'capture').glob('*-depth.npz')))==3


def test_receipt_parent_directory_is_durable_before_send(tmp_path,monkeypatch):
    import os
    io=io_module();s=ready_session(tmp_path,monkeypatch)
    synced=[];original=os.fsync
    def fsync(fd):
        synced.append(Path(os.readlink('/proc/self/fd/'+str(fd))))
        original(fd)
    monkeypatch.setattr(io.os,'fsync',fsync)
    def write(*args):
        assert tmp_path in synced
        assert tmp_path/'attempts' in synced
        s.arm.calls.append(args)
    s.arm.GripperCtrl=write
    assert s.attempt_gripper('open',70,.5)['state']=='COMMAND_ATTEMPTED'


def test_brief_fault_resets_gripper_stability(tmp_path,monkeypatch):
    m=main_module();config=geometry_module().load_config(CONFIG/'config.json')
    events=m.Events(config,tmp_path);s=FakeSession()
    rows=[dict(s.snapshot(),sample_time=t) for t in (1.,1.5,2.)]
    rows.insert(2,dict(rows[1],state='WAIT_FAULT'))
    iterator=iter(rows)
    def snapshot():
        try:return next(iterator)
        except StopIteration:raise KeyboardInterrupt
    s.snapshot=snapshot;monkeypatch.setattr(m.time,'sleep',lambda _:None)
    with pytest.raises(KeyboardInterrupt):m.wait_gripper(config,s,events,'open')


def test_readonly_offline_preview_refuses_unpaired_or_wrong_phase_data(tmp_path):
    import shutil
    source=tmp_path/'source';shutil.copytree(FIXTURES/'blue-block-after-retreat-rgbd01',source)
    meta=read(source/'metadata.json');meta['paired_robot'][0]['robot_snapshot']['joints_deg'][0]+=5
    (source/'metadata.json').write_text(json.dumps(meta))
    out=tmp_path/'out';out.mkdir()
    with pytest.raises(ValueError,match='moved'):
        main_module().preview_input(geometry_module().load_config(CONFIG/'config.json'),source,out)


def test_same_output_cannot_start_second_cycle(tmp_path,monkeypatch):
    result,_=run_fake(tmp_path,monkeypatch)
    previous=(tmp_path/'result.json').read_bytes()
    with pytest.raises(FileExistsError):run_fake(tmp_path,monkeypatch)
    assert (tmp_path/'result.json').read_bytes()==previous


def test_sample_spacing_config_is_applied():
    g=geometry_module();ref=read(CONFIG/'reference.json');model=read(CONFIG/'kinematics.json')
    ref['sample_spacing_m']=.005
    target=g.retarget(ref,ref['observation'],read(CONFIG/'extrinsics.json'))
    plan=g.plan_motion([0]*6,ref,target,model)
    points=[np.array(s['T_base_flange'])[:3,3] for s in plan['stages'] if s['phase']=='approach']
    assert max(np.linalg.norm(b-a) for a,b in zip(points,points[1:]))<=.00501


def test_capture_ignores_invalid_global_realsense_context(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS
    v=vision_module();calls=[];private_context=object()
    def context(settings):
        assert json.loads(settings)=={'inherit':False}
        return private_context
    class Pipeline:
        def __init__(self,ctx=None):
            if ctx is not private_context:
                raise RuntimeError('global config-file/context: expecting an object; got ""')
        def start(self,settings):calls.append('start');raise KeyboardInterrupt
        def stop(self):calls.append('stop')
    settings=NS(enable_device=lambda _:None,enable_stream=lambda *args:None)
    monkeypatch.setitem(sys.modules,'pyrealsense2',NS(context=context,pipeline=Pipeline,config=lambda:settings,
        stream=NS(color='color',depth='depth'),format=NS(rgb8='rgb8',z16='z16')))
    with pytest.raises(KeyboardInterrupt):v.capture_rgbd(read(CONFIG/'config.json'),tmp_path/'capture')
    assert calls==['start','stop']


def test_four_arm_capture_stages_and_return_zero_with_closed_gripper(tmp_path,monkeypatch):
    m=main_module();g=geometry_module();config=g.load_config(CONFIG/'semantic-camera-avoid.json')
    config['poll_seconds']=0;config['motion'].update(record_arm_outline=True,return_zero_after_grasp=True)
    session=FakeSession();recorded=[]
    stages=[dict(stage_id=name,phase=phase,angles_deg=[0,20,-10,0,0,0]) for name,phase in
            [('entry','unloaded'),('pregrasp','approach'),('grasp-1','approach'),('grasp-2','approach'),('grasp','approach'),('lift','lift')]]
    plan=dict(stages=stages,close_mm=40.)
    monkeypatch.setattr(m,'prepare_plan',lambda *a:({'observation':{}},plan))
    monkeypatch.setattr(m,'wait_gripper',lambda *a:68.)
    monkeypatch.setattr(m,'acquire',lambda *a:({'observations':[]},None))
    monkeypatch.setattr(m.vision,'verify_lift',lambda *a:dict(status='VERIFIED',reason='test'))
    monkeypatch.setattr(m,'record_arm_pose',lambda c,s,e,label:recorded.append((label,list(s.joints),s.width)))
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    result=m.run_once(config,session,None,tmp_path)
    assert result['state']=='GRASP_RETURNED_ZERO'
    assert [x[0] for x in recorded]==['pregrasp','grasp-1','grasp','lift']
    assert recorded[-1][2]!=68. and session.joints==[0]*6
    assert [c[0] for c in session.calls if c[1]=='gripper']==['open','close']
    assert all(c[1]=='joint' for c in session.calls[session.calls.index(next(c for c in session.calls if c[0]=='close'))+1:])
    assert len({c[0] for c in session.calls})==len(session.calls)
    assert read(tmp_path/'return-plan.json')['stages'][-1]['angles_deg']==[0]*6


def test_zero_plan_preserves_near_limit_measured_start_and_bounds_all_commands():
    g=geometry_module();cfg=g.load_config(CONFIG/'semantic-camera-avoid.json')
    q=[-4.642,179.926,-156.774,-19.757,-8.605,19.002]
    stages=g.zero_stages(q,cfg['kinematics'])
    assert len(stages)==18 and stages[-1]['angles_deg']==[0]*6
    assert np.max(np.abs(np.diff([q]+[s['angles_deg'] for s in stages],axis=0)))<=10
    for s in stages:g.fk(s['angles_deg'],cfg['kinematics'])
    with pytest.raises(ValueError):g.zero_stages([float('nan')]*6,cfg['kinematics'])


@pytest.mark.parametrize('reason',['moved','feedback_outage','long_pair'])
def test_arm_capture_retries_changed_pose_without_command_or_replay(tmp_path,monkeypatch,reason):
    import time
    m=main_module();cfg=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json')
    cfg['poll_seconds']=0;session=FakeSession();attempts=[];segmented=[]
    from piper_vision_grasp import arm_outline
    snapshot=session.snapshot;reads=[0]
    def interrupted_snapshot():
        reads[0]+=1;row=snapshot()
        if reason=='feedback_outage' and reads[0]==4:
            row.update(state='WAIT_FEEDBACK',missing_feedback=['test gap'])
        return row
    session.snapshot=interrupted_snapshot
    def capture(config,directory,count,detect,snapshot):
        assert count==3 and detect is False
        attempts.append(directory);directory.mkdir()
        frames=[]
        for i in range(3):
            before=snapshot();after=snapshot()
            if reason=='moved' and len(attempts)==1 and i==2:after['joints_deg'][0]=1
            now=time.time();start=now-60 if reason=='long_pair' and len(attempts)==1 else now
            frames.append(dict(host_unix=now,paired_robot=[dict(robot_snapshot=before),dict(robot_snapshot=after)],
                robot_pairing=dict(before_read_unix=start,after_read_unix=now,span_s=now-start)))
        return dict(metadata=dict(frames=frames))
    monkeypatch.setattr(m.vision,'capture_rgbd',capture)
    monkeypatch.setattr(arm_outline,'segment_capture',lambda d,c,s:segmented.append((d,s)))
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    m.record_arm_pose(cfg,session,m.Events(cfg,tmp_path),'pregrasp')
    assert len(attempts)==2 and segmented==[(attempts[1],'pregrasp')]
    assert session.calls==[]
    events=[json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert [e['state'] for e in events if e['state']!='WAIT_FEEDBACK']==['WAIT_ARM_CAPTURE','ARM_POSE_RECORDED']


def test_failed_grasp_confirmation_never_starts_loaded_return(tmp_path,monkeypatch):
    m=main_module();cfg=geometry_module().load_config(CONFIG/'semantic-camera-avoid.json');s=FakeSession()
    cfg['poll_seconds']=0
    plan=dict(close_mm=40.,stages=[dict(stage_id=name,phase=phase,angles_deg=[0,20,-10,0,0,0]) for name,phase in
        [('pregrasp','approach'),('grasp-1','approach'),('grasp','approach'),('lift','lift')]])
    monkeypatch.setattr(m,'prepare_plan',lambda *a:({'observation':{}},plan))
    monkeypatch.setattr(m,'wait_gripper',lambda *a:68.)
    monkeypatch.setattr(m,'record_arm_pose',lambda *a:None)
    monkeypatch.setattr(m,'acquire',lambda *a:({'observations':[]},None))
    monkeypatch.setattr(m.vision,'verify_lift',lambda *a:dict(status='WAIT_VERIFY',reason='not lifted'))
    monkeypatch.setattr(m.time,'sleep',lambda _:(_ for _ in ()).throw(KeyboardInterrupt()))
    result=m.run_once(cfg,s,None,tmp_path)
    assert result['state']=='EXITED' and result['grasp_confirmed'] is False
    assert not any(c[0].startswith('return-zero') for c in s.calls)
    assert [c[0] for c in s.calls if c[1]=='gripper']==['open','close']


@pytest.mark.parametrize('name',['record_arm_outline','return_zero_after_grasp'])
def test_capture_and_return_switches_reject_string_booleans(tmp_path,name):
    cfg=read(CONFIG/'semantic-camera-avoid.json');cfg['motion'][name]='false'
    path=tmp_path/'invalid.json';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError,match='must be boolean'):geometry_module().load_config(path)
