"""Only synthetic targets and stored frames; never open CAN, a camera or a model endpoint."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from piper_vision_grasp import cube_calibration as cube


def test_tilted_face_uses_inward_half_side_and_rejects_partial_face():
    rot = Rotation.from_euler('xyz', [15, -20, 12], degrees=True).as_matrix()
    face_center = np.array([.02, -.03, .28])
    y, x = np.indices((240, 320))
    rays = np.stack([(x-160)/360, (y-120)/360, np.ones_like(x)], axis=-1)
    normal = rot[:, 2]
    points = rays*((face_center@normal)/(rays@normal))[:, :, None]
    local = (points-face_center)@rot
    mask = np.max(abs(local[:, :, :2]), axis=2) <= cube.SIDE/2
    def fit_mask(keep):
        return cube.cube_face(np.column_stack([x[keep], y[keep]]), points[keep], face_center, mask.shape)
    result = fit_mask(mask)
    assert np.linalg.norm(result['cube_center_m']-(face_center+cube.SIDE/2*normal)) < .001
    assert result['face_count'] == 1
    assert np.allclose(result['inward_normal'], normal)
    assert np.linalg.norm(result['face_center_m']-face_center) < .001
    with pytest.raises(ValueError):
        fit_mask(mask & (local[:,:,0] < 0))


def rendered_cube(pitch, shifted_side=0):
    center = np.array([0., 0., .32])
    axes = Rotation.from_euler('xy', [pitch, 36], degrees=True).as_matrix()
    y, x = np.indices((240, 320))
    rays = np.stack([(x-160)/420, (y-120)/420, np.ones_like(x)], axis=-1)
    pixels, points = [], []
    for i, axis in enumerate(axes.T):
        normal = axis if axis @ center > 0 else -axis
        face = center - cube.SIDE/2*normal
        if normal @ face <= 0:
            continue
        if i==0:
            face = face + np.array([0., shifted_side, 0.])
        with np.errstate(divide='ignore', invalid='ignore'):
            xyz = rays*((normal @ face)/(rays @ normal))[:, :, None]
            coords = (xyz-face) @ axes
        valid = (xyz[:,:,2]>0) & (np.max(abs(np.delete(coords, i, axis=2)), axis=2)<=cube.SIDE/2)
        pixels.extend(np.column_stack([x[valid], y[valid]]))
        points.extend(xyz[valid])
    return np.asarray(pixels), np.asarray(points), center, center-cube.SIDE/2*axes[:,2], x.shape


@pytest.mark.parametrize('pitch, count', [(0, 2), (30, 3)])
def test_visible_faces_project_independently(pitch, count):
    pixels, points, center, grasp, shape = rendered_cube(pitch)
    result = cube.cube_face(pixels, points, grasp, shape)
    assert result['face_count'] == count
    estimates = [f['cube_center_m'] for f in result['faces']]
    assert max(np.linalg.norm(estimates-center, axis=1)) < .001
    assert np.allclose(result['cube_center_m'], np.mean(estimates, axis=0))
    assert np.allclose(result['face_angles_deg'], 90., atol=1.)
    assert max(result['center_distances_mm']) < 1.


def test_disagreeing_full_faces_rejected():
    pixels, points, _, grasp, shape = rendered_cube(0, shifted_side=.006)
    with pytest.raises(ValueError, match='独立块心距离'):
        cube.cube_face(pixels, points, grasp, shape)


def test_partial_second_face_uses_single_reliable_face():
    pixels, points, center, grasp, shape = rendered_cube(0)
    normal = Rotation.from_euler('y', 36, degrees=True).as_matrix()[:,2]
    keep = (abs((points-grasp) @ normal) < 1e-6) | (points[:,1] < -.01)
    result = cube.cube_face(pixels[keep], points[keep], grasp, shape)
    assert result['face_count'] == 1 and result['discarded_plane_reasons']
    assert np.linalg.norm(result['cube_center_m']-center) < .001


def poses():
    rng = np.random.RandomState(42)
    rotation = Rotation.from_euler('xyz', [84,4,-89], degrees=True).as_matrix()
    shift = np.array([.352,.034,-.063])
    offset = np.array([.009,.014,.144])
    rows = []
    for i in range(55):
        flange = np.eye(4)
        flange[:3,:3] = Rotation.from_rotvec(rng.normal(size=3)).as_matrix()
        flange[:3,3] = rng.uniform([.2,-.2,.05],[.55,.2,.3])
        point = rotation.T @ (flange[:3,:3]@offset + flange[:3,3]-shift)
        rows.append(dict(id=i+1,role='train' if i<45 else 'test',T_base_flange=flange,cube_center_m=point))
    return rows,rotation,shift,offset


def test_full_split_and_cli(tmp_path, monkeypatch):
    rows,rot,shift,offset=poses()
    for row in rows:
        if row['role']=='test':
            row['cube_center_m']+=np.array([.01,0,0])
        directory=tmp_path/'pose{:02d}'.format(row['id'])
        directory.mkdir()
        cube.save(directory/'sample.json',row)
    monkeypatch.setattr(sys,'argv',['cube-calibrate','solve','--dataset',str(tmp_path)])
    cube.main()
    result=json.loads((tmp_path/'result.json').read_text())
    tf=np.asarray(result['T_base_color_optical'])
    assert np.allclose(tf[:3,:3],rot) and np.allclose(tf[:3,3],shift)
    assert np.allclose(result['flange_to_cube_center_m'],offset)
    assert np.allclose(tf@result['T_color_optical_base'],np.eye(4))
    assert result['train']['count']==45 and result['train']['rms_mm']<1e-5
    assert result['test']['count']==10 and result['test']['rms_mm']==pytest.approx(10)


def test_no_wrist_excitation_and_duplicate_rejected():
    rows,*_=poses()
    with pytest.raises(ValueError,match='重合'):
        cube.check_distinct(rows[0],rows[:1])
    for row in rows:
        row['T_base_flange'][:3,:3]=np.eye(3)
    with pytest.raises(ValueError,match='激励不足'):
        cube.fit(rows[:45])
    with pytest.raises(ValueError,match='45'):
        cube.report(rows[:-1])


def test_collect_resume_and_rejected_attempt(tmp_path, monkeypatch):
    rows,*_=poses()
    for row in rows[:54]:
        dest=tmp_path/'pose{:02d}'.format(row['id'])
        dest.mkdir()
        cube.save(dest/'sample.json',row)

    class Passive:
        def __init__(self, config):
            assert config['readonly'] is True
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def snapshot(self):
            return None

    def capture(cfg,directory,*,count,detect,snapshot,on_ready):
        assert count==3 and detect is False and callable(snapshot) and callable(on_ready)
        directory.mkdir()
        (directory/'review.jpg').write_bytes(b'test review')
    monkeypatch.setattr(cube,'PiperSession',Passive)
    monkeypatch.setattr(cube.vision,'capture_rgbd',capture)
    monkeypatch.setattr(cube.vision,'load_capture',lambda *args:{'observations':[]})
    monkeypatch.setattr(cube,'extract',lambda *args:dict(rows[-1],center_spread_mm=.1,face_measurements=[]))
    choices=iter(('NO','OK'))
    monkeypatch.setattr('builtins.input',lambda _:next(choices))
    cfg={'reference':{'reference_id':'unit'}}
    assert len(cube.collect(tmp_path,cfg))==55
    assert (tmp_path/'pose55'/'review.jpg').is_file()
    assert len(list((tmp_path/'attempts').iterdir()))==1
    assert len(cube.read_rows(tmp_path))==55


def test_trigger_shares_pipeline_and_saves_fresh_frames(tmp_path, monkeypatch):
    from types import SimpleNamespace as NS
    intr=NS(width=4,height=3,fx=2.,fy=2.,ppx=2.,ppy=1.,coeffs=[0.]*5,model='none')
    profile=NS(as_video_stream_profile=lambda:NS(get_intrinsics=lambda:intr),
               get_extrinsics_to=lambda _:NS(rotation=np.eye(3).ravel(),translation=[0.,0.,0.]))
    counter=[0]
    calls=[]
    def frame(data,number):
        return NS(get_data=lambda:data,profile=profile,get_frame_number=lambda:number,
                  get_timestamp=lambda:float(number),get_frame_timestamp_domain=lambda:'hardware_clock')
    class Pipeline:
        def start(self,settings):
            calls.append('start')
            return NS(get_stream=lambda _:profile,get_device=lambda:NS(first_depth_sensor=lambda:NS(get_depth_scale=lambda:.001)))
        def wait_for_frames(self,timeout):
            counter[0]+=1
            number=counter[0]
            return NS(get_color_frame=lambda:frame(np.zeros((3,4,3),np.uint8),number),
                      get_depth_frame=lambda:frame(np.ones((3,4),np.uint16),number))
        def stop(self):
            calls.append('stop')
    settings=NS(enable_device=lambda _:None,enable_stream=lambda *args:None)
    rs=NS(context=lambda _:None,pipeline=lambda _:Pipeline(),config=lambda:settings,
          stream=NS(color='color',depth='depth'),format=NS(rgb8='rgb',z16='depth'),align=lambda _:NS(process=lambda f:f))
    monkeypatch.setitem(sys.modules,'pyrealsense2',rs)
    cfg=dict(camera_serial='unit',camera=dict(width=4,height=3,fps=15,warmup_frames=0))
    result=cube.vision.capture_rgbd(cfg,tmp_path/'pose',detect=False,snapshot=lambda:dict(number=counter[0]),
                                    on_ready=lambda pipe:pipe.wait_for_frames(3000))
    frames=result['metadata']['frames']
    assert calls==['start','stop']
    assert [f['color_frame'] for f in frames]==[3,5,7]
    assert [f['depth_frame'] for f in frames]==[3,5,7]
    for f in frames:
        before,after=[r['robot_snapshot']['number'] for r in f['paired_robot']]
        assert before < f['color_frame'] <= after
