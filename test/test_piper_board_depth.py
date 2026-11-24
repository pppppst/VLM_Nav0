"""Offline RGB-D board geometry and review/solve contracts."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from piper_vision_grasp import board_depth as board


def rendered(marker=25):
    T=np.eye(4)
    T[:3,:3]=cv2.Rodrigues(np.array([.13,.34,-.18]))[0] @ np.diag([1.,-1.,-1.])
    T[:3,3]=[.014,-.008,.38]
    c=dict(width=640,height=480,fx=610,fy=606,ppx=320,ppy=240,coeffs=[0.]*5)
    pts=board.MODEL@T[:3,:3].T+T[:3,3]
    uv=(pts[:,:2]/pts[:,2:]*[610,606]+[320,240]).astype(np.float32)
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    ar=cv2.aruco
    tag=ar.generateImageMarker(d,marker,256) if hasattr(ar,'generateImageMarker') else ar.drawMarker(d,marker,256)
    H=cv2.getPerspectiveTransform(np.float32([[0,0],[255,0],[255,255],[0,255]]),uv)
    gray=cv2.warpPerspective(tag,H,(640,480),borderValue=255)
    image=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
    y,x=np.indices(gray.shape)
    rays=np.stack([(x-320)/610,(y-240)/606,np.ones_like(x)],axis=-1)
    n=T[:3,2]
    z=np.rint(1000*(T[:3,3]@n)/(rays@n)).astype('uint16')
    # A sparse real cloud: holes all around each actual RGB corner and far returns inside.
    for point in uv:cv2.circle(z,tuple(point.astype(int)),8,0,-1)
    z[::4,::2]=1100
    z[1::3,::3]=0
    meta=dict(color_intrinsics=c,depth_intrinsics=c,depth_scale=.001,
              depth_to_color=dict(rotation=np.eye(3).ravel().tolist(),translation=[0,0,0]))
    return image,z,meta,T


def test_raw_depth_roi_plane_and_aruco_pose(tmp_path):
    im,z,m,T=rendered()
    corners=board.marker_corners(im)
    uv,cloud=board.cloud_on_color(z,m)
    metrics,pix,face=board.board_pose(corners,uv,cloud,m['color_intrinsics'])
    mm,deg=board.pose_error(T,metrics['T_color_marker'])
    assert mm<1 and deg<1 and metrics['plane_rms_mm']<.5 and len(pix)==len(face)
    # Use a different raw-depth frame origin. The fit must be in color optical coordinates.
    m['depth_to_color']['translation']=[0,0,.01]
    one=np.zeros_like(z)
    one[240,320]=380
    uv,cloud=board.cloud_on_color(one,m)
    assert np.array_equal(uv,[[320,240]])
    assert np.allclose(cloud,[[0,0,.39]])
    board.write_ply(tmp_path/'face.ply',face)
    assert 'element vertex {}'.format(len(face)).encode() in (tmp_path/'face.ply').read_bytes()[:180]


def test_no_rgb_only_fallback_or_wrong_id():
    im,z,m,_=rendered(31)
    with pytest.raises(ValueError,match='ID 25'):board.marker_corners(im)
    im,_,_,_=rendered()
    uv,cloud=board.cloud_on_color(z*0,m)
    with pytest.raises(ValueError,match='深度'):board.board_pose(board.marker_corners(im),uv,cloud,m['color_intrinsics'])
    uv,cloud=board.cloud_on_color(np.full_like(z,1200),m)
    with pytest.raises(ValueError,match='四边长'):board.board_pose(board.marker_corners(im),uv,cloud,m['color_intrinsics'])


def pose(rng):
    T=np.eye(4)
    T[:3,:3]=cv2.Rodrigues(rng.normal(size=3))[0]
    T[:3,3]=rng.uniform(-.2,.2,3)
    return T


def samples(mount_seed, train=30, validation=5):
    rng=np.random.RandomState(42)
    X=pose(rng)
    mount=pose(np.random.RandomState(mount_seed))
    rows=[]
    for i in range(1,train+validation+1):
        arm=pose(rng)
        rows.append(dict(joint_offset_deg=board.calibration_model()['joint_offset_deg'],id='pose%02d'%i,role=board.role(i,train),T_base_end=arm,
                         T_color_marker=np.linalg.inv(X)@arm@mount))
    return rows,X,mount


@pytest.mark.parametrize('mount_seed',[1,83])
def test_independent_mount_and_held_out_validation(mount_seed):
    rows,X,mount=samples(mount_seed)
    for r in rows[30:]:r['T_color_marker'][0,3]+=.01
    result=board.report(rows)
    assert np.allclose(result['T_base_color_optical'],X)
    assert np.allclose(result['T_end_marker'],mount)
    assert result['training']['count']==30 and result['validation']['count']==5
    assert result['validation']['rms_mm_deg'][0]==pytest.approx(10)
    for r in rows:r['T_base_end']=np.eye(4)
    with pytest.raises(ValueError,match='激励'):board.report(rows)


def test_capture_files_and_review(tmp_path):
    im,z,meta,_=rendered()
    snapshot=dict(state='READY',arm_status=0,err_code=0,missing_feedback=[],joints_deg=[0.]*6,
                  sdk_pose_mm_xyz=[0,0,200],sdk_pose_euler_deg_xyz=[0,0,0],gripper_width_mm=40.)
    meta['frames']=[]
    for i in range(3):
        stem='frame%02d'%i
        cv2.imwrite(str(tmp_path/(stem+'-color.png')),im)
        np.savez(tmp_path/(stem+'-depth.npz'),raw_depth=z,aligned_depth=z)
        meta['frames'].append(dict(stem=stem,color_frame=i+2,depth_frame=i+3,
                                   paired_robot=[dict(robot_snapshot=snapshot)]*2))
    sample=board.measure(tmp_path,meta)
    assert len(list(tmp_path.glob('*-board.ply')))==len(list(tmp_path.glob('*-review.jpg')))==3
    assert np.allclose(sample['T_base_end'],board.geometry.fk([0]*6,board.calibration_model()))
    assert sample['max_frame_translation_mm']<1e-6


def test_border_previews_survive_depth_rejection_and_missing_marker(tmp_path,monkeypatch):
    image,z,meta,_ = rendered()
    frames=[]
    for i in range(3):
        stem='frame%02d'%i
        frames.append(dict(stem=stem,color_frame=i+1,depth_frame=i+1))
        rgb=image if i!=1 else np.full_like(image,255)
        assert cv2.imwrite(str(tmp_path/(stem+'-color.png')),rgb)
        np.savez(tmp_path/(stem+'-depth.npz'),raw_depth=np.full_like(z,1200))
    meta['frames']=frames
    corners=board.marker_corners(image)
    monkeypatch.setattr(board,'arm_pose',lambda _:np.eye(4))
    shown=[]
    with pytest.raises(ValueError,match='frame00.*四边长'):
        board.measure(tmp_path,meta,shown.extend)
    assert [p.name for p in shown]==['frame00-border.jpg','frame01-border.jpg','frame02-border.jpg']
    overlay=cv2.imread(str(shown[0]))
    for a,b in zip(corners,np.roll(corners,1,axis=0)):
        x,y=np.rint((a+b)*.5).astype(int)
        blue,green,red=overlay[y,x].astype(int)
        assert green>blue+80 and red>blue+80
    missing=cv2.imread(str(shown[1]))
    assert np.all(missing[60:]==255) and np.any(missing[:60]<170)
    assert np.array_equal(cv2.imread(str(tmp_path/'frame00-color.png')),image)


def test_reject_retake_and_resume(tmp_path,monkeypatch):
    rows,_,_=samples(3)
    for r in rows[:-1]:
        directory=tmp_path/r['id'];directory.mkdir()
        board.save(directory/'sample.json',r)
    cfg=json.loads((board.ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json').read_text())
    board.save(tmp_path/'settings.json',dict(schema=1,board=board.BOARD,train=30,validation=5,arm_frame='Piper SDK end',
                                            **{k:cfg[k] for k in ('camera','camera_serial','usb_serial')}))
    def capture(config,directory,**kwargs):
        assert kwargs['snapshot'].__self__.readonly and callable(kwargs['on_ready'])
        directory.mkdir(parents=True);return dict(metadata={'frames':[{'host_unix':1.}]*3})
    monkeypatch.setattr(board.vision,'capture_rgbd',capture)
    monkeypatch.setattr(board,'measure',lambda *_:dict(rows[-1],frames=[],max_frame_translation_mm=0.,max_frame_angle_deg=0.))
    choices=iter(['reject','accept'])
    monkeypatch.setattr(board,'command',lambda *_:next(choices))
    monkeypatch.setattr(sys,'argv',['worker','collect','--dataset',str(tmp_path)])
    board.main()
    assert json.loads((tmp_path/'result.json').read_text())['validation']['count']==5
    assert len(list((tmp_path/'attempts').iterdir()))==1


def load_launcher():
    import importlib.util
    source = Path(__file__).resolve().parents[1]/'scripts/piper_board_depth.py'
    spec = importlib.util.spec_from_file_location('depth_launcher', source)
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    return launcher


def test_terminal_prompts_and_repeated_frames(tmp_path,capsys):
    terminal = load_launcher().TerminalReview()
    preview = dict(state='preview',message='pose01',token='a')
    terminal.show(preview,[])
    assert terminal.respond('OK\n') is None
    assert terminal.respond('\n')=='shoot a'
    terminal.show(preview,[])
    assert terminal.respond('\n') is None
    review = dict(state='review',message='pose01',token='a',captured_at=42.)
    terminal.show(review,[tmp_path/'review.jpg'])
    assert terminal.respond('\n') is None
    assert terminal.respond(' no \n')=='reject a'
    assert terminal.respond('OK\n') is None
    terminal.show(dict(preview,token='b'),[])
    assert terminal.respond('\n')=='shoot b'
    terminal.show(dict(review,token='b'),[])
    assert terminal.respond('ok\n')=='accept b'
    terminal.show(dict(state='retry',message='深度不足',token='c'),[])
    assert terminal.respond('\n')=='retry c'
    assert terminal.respond('q\n')=='quit'
    assert terminal.respond('')=='quit'
    assert str(tmp_path/'review.jpg') in capsys.readouterr().out


@pytest.mark.parametrize('train,validation',[(30,5),(20,8)])
def test_deployed_bundle_can_solve_without_source_tree_or_hardware(tmp_path, monkeypatch,train,validation):
    import subprocess
    import os
    launcher = load_launcher()
    # Stand in for SSH only; exercise the actual archive, install shell, and isolated interpreter.
    ssh = tmp_path/'ssh'
    ssh.write_text('#!/bin/sh\nfor last do :; done\nexec sh -c "$last"\n')
    ssh.chmod(0o755)
    monkeypatch.setenv('PATH',str(tmp_path)+os.pathsep+os.environ['PATH'])
    monkeypatch.chdir(tmp_path)
    with (tmp_path/'deploy.log').open('w') as log:
        release = launcher.deploy('offline',log)
        assert release==launcher.deploy('offline',log)
    worker = tmp_path/release/'tools/piper_vision_grasp/board_depth.py'
    cfg=json.loads((tmp_path/release/'config/piper_vision_grasp/semantic-camera-avoid.json').read_text())
    root=tmp_path/'samples';root.mkdir()
    board.save(root/'settings.json',dict(schema=1,board=board.BOARD,train=train,validation=validation,arm_frame='Piper SDK end',
                                         **{k:cfg[k] for k in ('camera','camera_serial','usb_serial')}))
    rows,X,_=samples(55,train,validation)
    for row in rows:
        dest=root/row['id'];dest.mkdir()
        board.save(dest/'sample.json',row)
    subprocess.run([sys.executable,'-I',str(worker),'solve','--dataset',str(root)],check=True,capture_output=True)
    assert np.allclose(json.loads((root/'result.json').read_text())['T_base_color_optical'],X)

def test_resume_legacy_training_only_keeps_approved_files(tmp_path,monkeypatch):
    cfg=json.loads((board.ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json').read_text())
    fields={k:cfg[k] for k in ('camera','camera_serial','usb_serial')}
    board.save(tmp_path/'settings.json',dict(schema=1,board=board.BOARD,train=20,validation=8,
                                             arm_frame='Piper SDK end',**fields))
    rows,_,_=samples(25)
    originals={}
    for row in rows[:8]:
        dest=tmp_path/row['id'];dest.mkdir()
        board.save(dest/'sample.json',row)
        originals[dest/'sample.json']=(dest/'sample.json').read_bytes()
    def pause(root,config,approved,events,train_count,validation_count):
        assert len(approved)==8 and (train_count,validation_count)==(30,5)
        raise EOFError
    monkeypatch.setattr(board,'collect',pause)
    monkeypatch.setattr(sys,'argv',['worker','collect','--dataset',str(tmp_path)])
    board.main()
    assert all(path.read_bytes()==data for path,data in originals.items())
    plan=json.loads((tmp_path/'settings.json').read_text())
    assert (plan['train'],plan['validation'])==(30,5)
