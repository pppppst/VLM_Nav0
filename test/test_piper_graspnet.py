"""GraspNet adapter contracts: no hardware or model download during unit tests."""
import copy
import importlib
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))


def adapter():
    return importlib.import_module('piper_vision_grasp.graspnet')


def config():
    from piper_vision_grasp import geometry
    return geometry.load_config(ROOT / 'config/piper_vision_grasp/semantic-camera-avoid.json')


def candidate(**kwargs):
    return dict(dict(score=.8, rotation=np.eye(3).tolist(), translation=[.1, .2, .3],
                     width=.06, height=.02, depth=.03), **kwargs)


def test_tip_depth_axes_extrinsics_and_tcp_applied_once():
    cfg = config()
    cfg['extrinsics']['T_base_color_optical'] = np.eye(4).tolist()
    cfg['semantic']['grasp_offset_base_m'] = [0, 0, 0]
    target = adapter().absolute_target(candidate(), cfg)
    tip, flange = np.array(target['T_base_tcp']), np.array(target['T_base_flange'])
    assert np.allclose(tip[:3, 3], [.13, .2, .3])
    assert np.allclose(tip[:3, 2], [1, 0, 0])
    assert np.allclose(tip[:3, 1], [0, 1, 0])
    assert np.linalg.det(tip[:3, :3]) == pytest.approx(1)
    assert np.allclose(flange[:3, 3], [-.005, .2, .3])
    assert target['grasp_width_m'] == pytest.approx(.05)
    assert target['candidate']['width'] == .06


def test_rotated_camera_extrinsic_maps_tip_and_tool_together():
    cfg = config()
    cfg['extrinsics']['T_base_color_optical'] = [[0,-1,0,.4],[1,0,0,.1],[0,0,1,.2],[0,0,0,1]]
    target=adapter().absolute_target(candidate(),cfg)
    assert np.allclose(np.array(target['T_base_tcp'])[:3,3],[.2,.23,.5])
    assert np.allclose(np.array(target['T_base_flange'])[:3,3],[.2,.095,.5])


def test_no_candidates_waits_in_prepare_without_sending_commands(tmp_path, monkeypatch):
    from piper_vision_grasp import main
    cfg=config();cfg['grasp_backend']='graspnet'
    captured=dict(capture_dir=str(tmp_path),observation={})
    monkeypatch.setattr(main,'acquire',lambda *args:(captured,dict(joints_deg=[0]*6)))
    monkeypatch.setattr(adapter(),'plan_capture',lambda *args:adapter().select_plan(cfg,[0]*6,[]))
    class Session:
        def snapshot(self):raise AssertionError('should wait before further IO')
        def attempt_joint(self,*args):raise AssertionError('unexpected motion')
        def attempt_gripper(self,*args):raise AssertionError('unexpected gripper command')
    def stop(_):raise KeyboardInterrupt
    monkeypatch.setattr(main.time,'sleep',stop)
    events=main.Events(cfg,tmp_path)
    with pytest.raises(KeyboardInterrupt):main.prepare_plan(cfg,Session(),None,events)
    assert events.last['state']=='WAIT_PLAN' and not events.last['command_attempted']


@pytest.mark.parametrize('change', [dict(depth=-1), dict(width=float('nan')),
    dict(rotation=np.zeros((3, 3)).tolist()), dict(translation=[1, 2]), dict(score=float('inf'))])
def test_bad_candidates_rejected(change):
    with pytest.raises(ValueError):
        adapter().absolute_target(candidate(**change), config())


def test_cloud_metric_units_and_target_association():
    camera = dict(width=2, height=2, fx=2., fy=2., ppx=0., ppy=0., depth_scale=.001)
    depth = np.array([[1000, 0], [2000, 1000]], dtype=np.uint16)
    mask = np.array([[True, False], [False, True]])
    points, target = adapter().point_cloud(depth, camera, mask)
    assert np.allclose(points, [[0, 0, 1], [0, 1, 2], [.5, .5, 1]])
    assert target.tolist() == [True, False, True]


def test_mask_boundary_background_depth_remains_scene_but_is_not_target():
    camera = dict(width=10, height=10, fx=100., fy=100., ppx=5., ppy=5., depth_scale=.001)
    depth = np.full((10,10), 250, np.uint16); depth[0,0] = 1700
    points, target = adapter().point_cloud(depth, camera, np.ones(depth.shape,bool))
    assert len(points) == 100 and points[0,2] == pytest.approx(1.7)
    assert not target[0] and target.sum() == 99


def test_cloud_wire_roundtrip_and_invalid_mask():
    points = np.array([[0, 0, .3], [1, 0, .5]], np.float32)
    mask = np.array([True, False])
    decoded = adapter().decode_cloud(adapter().encode_cloud(points, mask))
    assert np.array_equal(decoded[0], points)
    assert np.array_equal(decoded[1], mask)
    with pytest.raises(ValueError):
        adapter().encode_cloud(points, np.array([True]))


def test_candidate_ranking_skips_wide_and_unreachable(monkeypatch):
    from piper_vision_grasp import geometry
    calls = []
    def planner(joints, cfg, target):
        score = target['candidate']['score']; calls.append(score)
        return dict(status='PLANNED' if score == .7 else 'WAIT_IK', reason='unreachable', target=target)
    monkeypatch.setattr(geometry, 'plan_absolute', planner)
    rows = [candidate(score=.7), candidate(score=.9, width=.08), candidate(score=.8)]
    original = copy.deepcopy(rows)
    result = adapter().select_plan(config(), [0]*6, rows)
    assert result['status'] == 'PLANNED'
    assert calls == [.8, .7]
    assert result['target']['candidate']['score'] == .7
    assert len(result['graspnet']['rejected']) == 2
    assert rows == original


def test_empty_candidates_wait_without_fallback():
    assert adapter().select_plan(config(), [0]*6, [])['status'] == 'WAIT_PLAN'


def test_pitch_override_rejected_only_for_graspnet():
    from piper_vision_grasp.main import configure_target
    cfg = config(); cfg['grasp_backend'] = 'graspnet'
    with pytest.raises(ValueError, match='GraspNet'):
        configure_target(cfg, approach_pitch=40)
    cfg.pop('grasp_backend')
    configure_target(cfg, approach_pitch=40)
    assert cfg['semantic']['approach_pitch_deg'] == 40


def test_worker_filters_background_and_width_before_nms():
    server = importlib.import_module('piper_vision_grasp.graspnet_server')
    points = np.array([[0, 0, .5], [.1, 0, .5], [.2, 0, .5]])
    keep, counts = server.target_candidates(points, np.array([True, False, True]), points,
                                             np.array([.06, .05, .08]), .07)
    assert keep.tolist() == [True, False, False]
    assert counts == dict(background=1, width=1)


def test_service_failure_is_recoverable_value_error(monkeypatch):
    m = adapter(); cfg = config()
    cfg['graspnet'] = dict(endpoint='http://127.0.0.1:18766', timeout_s=1)
    monkeypatch.setattr(m, 'capture_cloud', lambda _: (np.array([[0, 0, .5]]), np.array([True])))
    def unavailable(*args, **kwargs):
        raise OSError('connection refused')
    monkeypatch.setattr(m.request, 'urlopen', unavailable)
    with pytest.raises(ValueError, match='inference unavailable'):
        m.plan_capture(cfg, [0]*6, {})


def test_unknown_backend_does_not_silently_use_legacy(tmp_path):
    import json
    import shutil
    from piper_vision_grasp import geometry
    bundle = tmp_path / 'config'
    shutil.copytree(ROOT / 'config/piper_vision_grasp', bundle)
    path = bundle / 'semantic-camera-avoid.json'
    cfg = json.loads(path.read_text()); cfg['grasp_backend'] = 'graspet'
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError, match='backend'):
        geometry.load_config(path)


def test_replay_reuses_saved_masks_and_keeps_observation(tmp_path, monkeypatch):
    import json
    import cv2
    from piper_vision_grasp import main, semantic, vision
    cfg = config(); cfg['grasp_backend'] = 'graspnet'
    camera = dict(width=100, height=80, fx=100., fy=100., ppx=50., ppy=40., coeffs=[0]*5)
    depth = np.full((80,100), 500, np.uint16)
    rgb = np.zeros((80,100,3), np.uint8)
    mask = np.zeros(depth.shape, bool); mask[20:60,30:70] = True
    obs = semantic.measure(rgb, depth, dict(camera, depth_scale=.001), cfg,
                           dict(bbox_px=[30,20,70,60], keypoint_px=[50,40], mask=mask))
    rows = [dict(obs, host_unix=float(i)) for i in range(3)]
    meta = dict(serial=cfg['camera_serial'], color_intrinsics=camera, depth_scale=.001,
                paired_robot=[dict(robot_snapshot=dict(missing_feedback=[],joints_deg=[0]*6))],
                frames=[dict(stem='frame%02d'%i, host_unix=float(i), aligned_intrinsics=camera) for i in range(3)])
    captured = dict(metadata=meta, observations=rows, observation=vision.combine(rows, cfg['detector']))
    main.save(tmp_path/'metadata.json', meta)
    main.save_observation(tmp_path, captured)
    for i in range(3):np.savez(tmp_path/('frame%02d-depth.npz'%i), aligned_depth=depth)
    def forbidden(*args, **kwargs):raise AssertionError('offline replay called detector')
    monkeypatch.setattr(semantic, 'detect', forbidden)
    loaded = adapter().load_saved_capture(tmp_path, cfg)
    assert np.array_equal(loaded['observations'][-1]['mask'], mask)
    assert loaded['observation']['point_color_m'] == captured['observation']['point_color_m']
    points, target = adapter().capture_cloud(loaded)
    assert len(points) == 8000 and target.sum() == 1600
    original_target=copy.deepcopy(cfg['target'])
    cfg['target']=dict(color='绿色',description='小盒子')
    def inspect_plan(active_cfg,*args):
        assert active_cfg['target']==original_target
        return dict(status='PLANNED')
    monkeypatch.setattr(main,'make_plan',inspect_plan)
    out=tmp_path/'replay';out.mkdir()
    result=main.preview_input(cfg,tmp_path,out)
    assert result['target_spec']==original_target
    assert result['requested_target_spec']==cfg['target']
    assert 'saved target masks' in result['reason']


def test_plan_freezes_calibration_and_tcp_for_later_visualization(monkeypatch):
    import io,json
    m=adapter();cfg=config();cfg['graspnet']=dict(endpoint='http://localhost:18766',timeout_s=1)
    monkeypatch.setattr(m,'capture_cloud',lambda _:(np.array([[0,0,.5]]),np.array([True])))
    payload=dict(frame='color_optical',units='m',candidates=[],provenance=dict(checkpoint_sha256='test'))
    monkeypatch.setattr(m.request,'urlopen',lambda *args,**kwargs:io.BytesIO(json.dumps(payload).encode()))
    plan=m.plan_capture(cfg,[0]*6,dict(metadata=dict(frames=[dict(stem='frame02')])))
    frozen=plan['graspnet']['planning_config']
    expected=copy.deepcopy(cfg['extrinsics'])
    cfg['extrinsics']['T_base_color_optical'][0][3]+=1
    cfg['semantic']['tcp_offset_m'][2]=.3
    assert frozen['extrinsics']==expected
    assert frozen['semantic']['tcp_offset_m']==[0,0,.135]


def test_cached_preview_rejects_new_color_override(tmp_path):
    import subprocess,os,json
    env=dict(os.environ,DISPLAY='',WAYLAND_DISPLAY='')
    result=subprocess.run([sys.executable,'-I',str(ROOT/'tools/piper_vision_grasp/main.py'),'preview',
        '--config',str(ROOT/'config/piper_vision_grasp/semantic-graspnet.json'),
        '--input',str(tmp_path),'--color','绿色','--output-root',str(tmp_path),'--run-id','override'],
        env=env,capture_output=True,text=True,timeout=10)
    assert result.returncode==1
    assert 'saved target masks' in json.loads((tmp_path/'override/result.json').read_text())['reason']


def test_worker_http_boundary_recovers_after_bad_request():
    import json
    import threading
    from http.server import HTTPServer
    from urllib import request, error
    from piper_vision_grasp.graspnet_server import handler
    class Model:
        def infer(self, points, target, width):
            assert points.shape == (1,3) and target[0] and width == .07
            return dict(frame='color_optical', units='m', candidates=[candidate()], counts={}, inference_seconds=0.)
    class Events:
        def __init__(self):self.states=[]
        def emit(self,state,*args,**kwargs):self.states.append(state)
    events=Events()
    server=HTTPServer(('127.0.0.1',0),handler(Model(),events))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        endpoint='http://127.0.0.1:%d/grasps'%server.server_port
        with pytest.raises(error.HTTPError):
            request.urlopen(request.Request(endpoint,data=b'{}'),timeout=2)
        row=adapter().encode_cloud(np.array([[0,0,.5]]),np.array([True]));row['max_width_m']=.07
        with request.urlopen(request.Request(endpoint,data=json.dumps(row).encode()),timeout=2) as response:
            assert json.load(response)['candidates'][0]['width'] == .06
        assert events.states == ['WAIT_VISION','MODEL_RESULT']
    finally:
        server.shutdown();thread.join();server.server_close()


def test_urdf_render_mesh_respects_joint_and_visual_origins(tmp_path):
    from piper_vision_grasp.graspnet_visualize import robot_meshes
    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    triangle=np.zeros(1,dtype=dtype);triangle['vertices']=[[[0,0,0],[.01,0,0],[0,.01,0]]]
    (tmp_path/'meshes').mkdir();(tmp_path/'urdf').mkdir()
    (tmp_path/'meshes/link.STL').write_bytes(bytes(80)+(1).to_bytes(4,'little')+triangle.tobytes())
    urdf=tmp_path/'urdf/piper.urdf'
    urdf.write_text('''<robot name="piper"><link name="base"/><link name="tip"><visual>
      <origin xyz="0 0 0.135"/><geometry><mesh filename="package://piper_description/meshes/link.STL"/></geometry>
      </visual></link><joint name="joint1" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/></joint></robot>''')
    meshes=robot_meshes(urdf,[90,0,0,0,0,0],.07)
    assert np.allclose(meshes[0][1][0],[[.1,0,.135],[.1,.01,.135],[.09,0,.135]])
