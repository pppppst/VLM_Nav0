"""Eye-to-hand calibration from a rigidly held 49.5 mm red cube. All lengths in metres."""
import argparse
import fcntl
import json
import select
import sys
import uuid
from pathlib import Path

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = 'piper_vision_grasp'

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import geometry, semantic, vision
from .main import ROOT, Events, PiperSession, save, save_observation

SIDE = 0.0495
TRAIN, TEST = 45, 10
DEFAULT_CONFIG = ROOT / 'config/piper_vision_grasp/semantic-camera-avoid.json'


def connected_patch(pixels, points, image_shape):
    # Exclude isolated depth islands before measuring the square's outline.
    grid = np.zeros(image_shape, np.uint8)
    grid[pixels[:, 1], pixels[:, 0]] = 1
    joined = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, labels = cv2.connectedComponents(joined)
    ids = labels[pixels[:, 1], pixels[:, 0]]
    return points[ids == np.bincount(ids)[1:].argmax()+1]


def dominant_plane(pixels, points, image_shape):
    rng = np.random.RandomState(25)
    probe = points[rng.choice(len(points), min(len(points), 1500), replace=False)]
    best = 0
    for _ in range(120):
        a, b, c = probe[rng.choice(len(probe), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-7:
            continue
        n /= length
        score = np.count_nonzero(np.abs((probe-a) @ n) < .0018)
        if score > best:
            best, anchor, normal = score, a, n
    if best < .24 * len(probe):
        raise ValueError('剩余深度点没有明确的平面')
    for _ in range(3):
        keep = np.abs((points-anchor) @ normal) < .0018
        if keep.sum() < 100:
            raise ValueError('面内深度不足')
        face = connected_patch(pixels[keep], points[keep], image_shape)
        if len(face) < max(100, .20 * len(points)):
            raise ValueError('主平面的连续支持不足')
        anchor = face.mean(axis=0)
        _, axes = np.linalg.eigh((face-anchor).T @ (face-anchor))
        normal = axes[:, 0]
    return anchor, normal, np.abs((points-anchor) @ normal) <= .0018


def square_face(face):
    anchor = face.mean(axis=0)
    _, axes = np.linalg.eigh((face-anchor).T @ (face-anchor))
    normal = axes[:, 0]
    # For a convex cube, its interior is behind the observed face, away from the optical origin.
    if normal @ anchor < 0:
        normal = -normal
    basis = axes[:, 1:]
    planar = ((face-anchor) @ basis).astype(np.float32)
    rect = cv2.minAreaRect(planar)
    sides = np.asarray(rect[1])
    hull_area = cv2.contourArea(cv2.convexHull(planar))
    if np.any(np.abs(sides-SIDE) > .007) or hull_area < .78 * np.prod(sides):
        raise ValueError('可见面不完整或尺寸不符，两边实测 {} mm'.format((sides*1000).round(1)))
    center = anchor + basis @ np.asarray(rect[0])
    return dict(face_center_m=center, inward_normal=normal, cube_center_m=center+normal*(SIDE/2),
                face_corners_m=cv2.boxPoints(rect) @ basis.T+anchor,
                measured_sides_mm=sides*1000, face_points=len(face),
                plane_rms_mm=np.sqrt(np.mean(((face-anchor) @ normal)**2))*1000)


def cube_face(pixels, points, grasp_point, image_shape, agreement_m=.003):
    """Fit visible squares, then fuse their independent half-side projections."""
    if len(points) < 100:
        raise ValueError('红块有效深度点少于 100')
    remaining = np.arange(len(points))
    planes = []
    reasons = []
    for _ in range(3):
        if len(remaining) < 100:
            break
        try:
            anchor, normal, used = dominant_plane(pixels[remaining], points[remaining], image_shape)
        except ValueError as exc:
            reasons.append(str(exc))
            break
        planes.append((anchor, normal))
        remaining = remaining[~used]
    faces = []
    if planes:
        # Reassign edge points to their closest plane so extraction order does not truncate the next face.
        distances = np.column_stack([np.abs((points-a) @ n) for a, n in planes])
        nearest = distances.argmin(axis=1)
        for i in range(len(planes)):
            selected = (nearest == i) & (distances[:, i] <= .0018)
            uv = pixels[selected]
            if len(uv) < 100:
                continue
            patch = connected_patch(uv, points[selected], image_shape)
            if len(patch) < 100:
                continue
            try:
                faces.append(square_face(patch))
            except ValueError as exc:
                reasons.append(str(exc))
    if not faces:
        raise ValueError('无完整可用方形面：'+'；'.join(reasons))
    if len(faces)==1 and abs((np.asarray(grasp_point)-faces[0]['face_center_m']) @ faces[0]['inward_normal']) > .004:
        raise ValueError('抓取视觉点不在所选平面上')
    angles = []
    differences = []
    for i, face in enumerate(faces):
        for other in faces[:i]:
            angle = np.degrees(np.arccos(np.clip(face['inward_normal'] @ other['inward_normal'], -1, 1)))
            difference = 1000*np.linalg.norm(face['cube_center_m']-other['cube_center_m'])
            if abs(angle-90) > 12 or difference > 1000*agreement_m:
                raise ValueError('完整面互不一致：法向夹角 {:.1f}°，独立块心距离 {:.2f} mm'.format(angle, difference))
            angles.append(angle)
            differences.append(difference)
    center = np.mean([f['cube_center_m'] for f in faces], axis=0)
    return dict(faces[0], cube_center_m=center, faces=faces, face_count=len(faces), grasp_point_m=grasp_point,
                face_angles_deg=angles, center_distances_mm=differences, discarded_plane_reasons=reasons)


def draw_faces(image, camera, face):
    def project(xyz):
        uv = xyz[:, :2]/xyz[:, 2:3] * [camera['fx'], camera['fy']] + [camera['ppx'], camera['ppy']]
        return np.rint(uv).astype(np.int32)
    for square, color in zip(face['faces'], [(0,220,255), (230,160,0), (130,230,100)]):
        cv2.polylines(image, [project(square['face_corners_m'])], True, color, 2)
        a, b = project(np.array([square['face_center_m'], square['cube_center_m']]))
        cv2.arrowedLine(image, tuple(a), tuple(b), color, 2)
    fused = project(face['cube_center_m'][None, :])[0]
    cv2.drawMarker(image, tuple(fused), (255,0,255), cv2.MARKER_CROSS, 12, 2)


def extract(directory, config, captured):
    if captured['observation']['status'] != 'DETECTED':
        raise ValueError(captured['observation']['reason'])
    meta = captured['metadata']
    frames = meta['frames']
    if len(frames) != 3 or any(np.any(np.diff([f[k] for f in frames]) <= 0)
                               for k in ('color_frame', 'depth_frame')):
        raise ValueError('需要三组独立的 RGB-D 帧')
    snapshots = [entry['robot_snapshot'] for f in frames for entry in f['paired_robot']]
    for row in snapshots:
        stopped_teaching = (row['state']=='WAIT_CONTROL' and row['ctrl_mode']==2 and row['teach_status']==2)
        if (row['state']!='READY' and not stopped_teaching) or row['missing_feedback'] or row['err_code']:
            raise ValueError('机械臂反馈不可用或示教未停止：'+str(row.get('reason')))
    joints = np.array([row['joints_deg'] for row in snapshots])
    if np.max(np.ptp(joints, axis=0)) > .3 or np.ptp([r['gripper_width_mm'] for r in snapshots]) > .5:
        raise ValueError('采图期间机械臂或夹爪开度有变化')
    results = []
    for frame, obs in zip(frames, captured['observations']):
        stem = frame['stem']
        camera = dict(meta['color_intrinsics'], depth_scale=meta['depth_scale'],
                      aligned_intrinsics=frame['aligned_intrinsics'])
        with np.load(directory/(stem+'-depth.npz'), allow_pickle=False) as archive:
            depth = archive['aligned_depth']
        pixels, points, _ = semantic.target_points(depth, camera, obs['mask'])
        face = cube_face(pixels, points, obs['point_color_m'], depth.shape, config['detector']['stability_m'])
        results.append(face)
        if stem == 'frame01':
            image = cv2.imread(str(directory/(stem+'-color.png')))
            draw_faces(image, camera, face)
            if not cv2.imwrite(str(directory/'review.jpg'), image):
                raise OSError('无法保存面轮廓预览')
    centers = np.array([f['cube_center_m'] for f in results])
    center = np.median(centers, axis=0)
    spread = np.max(np.linalg.norm(centers-center, axis=1))
    if spread > config['detector']['stability_m']:
        raise ValueError('三帧推算的块心不稳定：{:.2f} mm'.format(spread*1000))
    return dict(center_estimator='visible-square-fusion-v2', T_base_flange=geometry.fk(np.median(joints, axis=0), config['kinematics']),
                cube_center_m=center, face_measurements=results, center_spread_mm=spread*1000,
                gripper_width_mm=np.median([r['gripper_width_mm'] for r in snapshots]),
                representative='frame01-color.png', host_unix=frames[1]['host_unix'])


def arrays(rows):
    flange = np.asarray([geometry.transform(r['T_base_flange']) for r in rows])
    points = np.asarray([r['cube_center_m'] for r in rows], dtype=float)
    if points.shape != (len(rows), 3) or not np.isfinite(points).all():
        raise ValueError('块心数据无效')
    return points, flange[:, :3, :3], flange[:, :3, 3]


def fit(train):
    # R p_camera + t = R_flange d + t_flange. d is a fixed, unknown flange-to-center vector.
    p, A, b = arrays(train)
    mean = p.mean(axis=0)
    centered = p-mean
    scale = np.sqrt(np.mean(np.sum(centered**2, axis=1)))
    if scale < 1e-4:
        raise ValueError('方块中心位置分布不足')
    def residual(x):
        R = geometry.rotation_matrix(Rotation.from_rotvec(x[:3]))
        return (centered @ R.T + x[3:6] - A @ x[6:] - b).ravel()
    # Initial rigid alignment only. The nominal 135 mm TCP and the r3 TF are not used.
    U, _, Vt = np.linalg.svd(centered.T @ (b-b.mean(axis=0)))
    D = np.diag([1., 1., np.linalg.det(Vt.T @ U.T)])
    R0 = Vt.T @ D @ U.T
    solution = least_squares(residual, np.r_[geometry.rotation_from_matrix(R0).as_rotvec(), b.mean(axis=0), np.zeros(3)],
                             ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if not solution.success:
        raise ValueError('刚体求解未收敛：'+solution.message)
    x = solution.x
    R = geometry.rotation_matrix(Rotation.from_rotvec(x[:3]))
    # Use a local angular perturbation, normalized by the RMS scene radius, for a dimensionless rank test.
    rotated = centered @ R.T / scale
    J = np.concatenate((np.cross(np.eye(3)[None,:,:], rotated[:,None,:]).transpose(0,2,1),
                        np.broadcast_to(np.eye(3), A.shape), -A), axis=2)
    singular = np.linalg.svd(J.reshape(-1,9), compute_uv=False)
    if singular[-1] < .01*singular[0]:
        raise ValueError('激励不足，改变块心位置并绕两个不平行轴转腕（奇异值比 {:.5f}）'.format(singular[-1]/singular[0]))
    tf = np.eye(4)
    tf[:3,:3], tf[:3,3] = R, x[3:6]-R@mean
    return tf, x[6:], singular


def errors(rows, tf, offset):
    p,A,b = arrays(rows)
    delta = 1000*(p @ tf[:3,:3].T + tf[:3,3] - (A @ offset + b))
    norm = np.linalg.norm(delta, axis=1)
    return dict(count=len(rows), rms_mm=np.sqrt(np.mean(norm**2)), median_mm=np.median(norm),
                max_mm=np.max(norm), mean_xyz_mm=delta.mean(axis=0),
                poses=[dict(id=r['id'], delta_base_mm=v, norm_mm=n) for r,v,n in zip(rows,delta,norm)])


def report(rows):
    if len(rows) != TRAIN+TEST or any(r['role']!=('train' if i<TRAIN else 'test')
                                      or r['id']!=i+1 for i,r in enumerate(rows)):
        raise ValueError('需要顺序完整的 45 个训练姿态和 10 个留出测试姿态')
    tf, offset, singular = fit(rows[:TRAIN])
    return dict(method='RGB-D cube-center eye-to-hand, joint least squares', units='metres',
                cube_side_m=SIDE, T_base_color_optical=tf, T_color_optical_base=np.linalg.inv(tf),
                flange_to_cube_center_m=offset, normalized_jacobian_singular_values=singular,
                train=errors(rows[:TRAIN],tf,offset), test=errors(rows[TRAIN:],tf,offset),
                physical_validation='NOT_PERFORMED')


def read_rows(root):
    rows = []
    for i, directory in enumerate(sorted(root.glob('pose[0-9][0-9]')), 1):
        row = json.loads((directory/'sample.json').read_text())
        if directory.name!='pose{:02d}'.format(i) or row['id']!=i or row['role']!=('train' if i<=TRAIN else 'test'):
            raise ValueError('采样目录编号或角色不连续：'+str(directory))
        rows.append(row)
    return rows


def preview_until_enter(pipeline, root):
    print('手摆并停止示教，保持静止。回车拍摄，Q 暂停。缩略图：'+str(root/'preview.jpg'), flush=True)
    while True:
        if select.select([sys.stdin], [], [], 2)[0]:
            line=sys.stdin.readline()
            if not line or line.strip().upper()=='Q':
                raise EOFError
            if not line.strip():
                return
            continue
        frames=pipeline.wait_for_frames(3000)
        frame=frames.get_color_frame()
        if frame:
            thumb=cv2.resize(np.asanyarray(frame.get_data()), (320,240),interpolation=cv2.INTER_AREA)
            tmp=root/'preview.tmp.jpg'
            if not cv2.imwrite(str(tmp),cv2.cvtColor(thumb,cv2.COLOR_RGB2BGR)):
                raise OSError('无法写缩略图')
            tmp.replace(root/'preview.jpg')


def check_distinct(sample, rows):
    current=geometry.transform(sample['T_base_flange'])
    for old in rows:
        prior=geometry.transform(old['T_base_flange'])
        cosine=(np.trace(current[:3,:3].T@prior[:3,:3])-1)/2
        if np.linalg.norm(current[:3,3]-prior[:3,3])<.001 and np.degrees(np.arccos(np.clip(cosine,-1,1)))<1:
            raise ValueError('与 pose{:02d} 重合，请换姿态'.format(old['id']))


def collect(root, config):
    rows=read_rows(root)
    if len(rows)>=TRAIN+TEST:
        return rows
    events=Events(config, root)
    events.emit('STARTING','passive RGB-D cube calibration', accepted=len(rows))
    print('诊断日志：'+str(root/'events.jsonl'),flush=True)
    attempts=root/'attempts'
    attempts.mkdir(exist_ok=True)
    with PiperSession(dict(config, readonly=True)) as session:
        while len(rows)<TRAIN+TEST:
            index=len(rows)+1
            role='train' if index<=TRAIN else 'test'
            name='pose{:02d}'.format(index)
            directory=attempts/(name+'-'+uuid.uuid4().hex[:8])
            print('{} ({})'.format(name,role),flush=True)
            try:
                vision.capture_rgbd(config,directory,count=3,detect=False,snapshot=session.snapshot,
                                    on_ready=lambda pipeline:preview_until_enter(pipeline,root))
                captured=vision.load_capture(directory,config)
                save_observation(directory,captured)
                sample=extract(directory,config,captured)
                sample.update(id=index,role=role)
                check_distinct(sample,rows)
                if index%5==0 and index<=TRAIN:
                    try:
                        tf,offset,_=fit(rows+[sample])
                        print('累计训练闭环 RMS {:.2f} mm'.format(errors(rows+[sample],tf,offset)['rms_mm']))
                    except ValueError as exc:
                        if index==TRAIN:
                            raise
                        print(str(exc))
                save(directory/'sample.json',sample)
                tmp=root/'preview.tmp.jpg'
                tmp.write_bytes((directory/'review.jpg').read_bytes())
                tmp.replace(root/'preview.jpg')
                events.emit('REVIEW','review square face and inferred center',name,review=str(directory/'review.jpg'))
                print('核对各色框是否分别包住完整方形面：'+str(directory/'review.jpg'),flush=True)
                for i, frame in enumerate(sample['face_measurements'], 1):
                    print('帧 {}：{} 面，面间块心距离 {} mm'.format(i, frame['face_count'],
                          np.round(frame['center_distances_mm'], 2).tolist()))
                print('块心三帧最大偏差 {:.2f} mm。'.format(sample['center_spread_mm']))
                choice=input('OK 收录，NO 重拍，Q 暂停：').strip().upper()
                while choice not in ('OK','NO','Q'):
                    choice=input('请输入 OK / NO / Q：').strip().upper()
                if choice=='Q':
                    raise EOFError
                if choice=='OK':
                    directory.rename(root/name)
                    rows.append(sample)
                    events.emit('ACCEPTED',role,name,accepted=len(rows))
            except (OSError,RuntimeError,ValueError,cv2.error) as exc:
                events.emit('WAIT_SAMPLE',str(exc),name,attempt=str(directory))
                print('采样未收录：'+str(exc),file=sys.stderr)
                if input('修正后回车重试，Q 暂停：').strip().upper()=='Q':
                    raise EOFError
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('collect','solve'))
    parser.add_argument('--dataset',required=True,type=Path)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    args=parser.parse_args()
    root=args.dataset.expanduser().resolve()
    if args.action=='collect':
        root.mkdir(parents=True,exist_ok=True)
    if not root.is_dir():
        parser.error('数据集不存在')
    with (root/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        path=root/'settings.json'
        if args.action=='collect':
            config=geometry.load_config(args.config)
            if config.get('mode')!='semantic' or config.get('grasp_backend','legacy')!='legacy':
                parser.error('需要实际抓取使用的语义 RGB-D 配置')
            config['target']=dict(color='红色',description='方块')
            settings=dict(schema=2,camera=config['camera'],serial=config['camera_serial'],usb=config['usb_serial'],
                          joint_offset_deg=config['kinematics'].get('joint_offset_deg'),
                          kinematics=config['kinematics']['source_sha256'],detector=config['detector'],
                          semantic=config['semantic'],target=config['target'],cube_side_m=SIDE,train=TRAIN,test=TEST,
                          acquisition_extrinsics=config['extrinsics'])
            if path.exists():
                previous = json.loads(path.read_text())
                legacy = previous.get('schema') == 1
                if legacy:
                    previous['schema'] = 2
                if previous != settings:
                    parser.error('恢复参数与已有数据不一致')
                if legacy:
                    save(path,settings)
            else:
                if any(p.name!='.lock' for p in root.iterdir()):
                    parser.error('请选择新目录或原数据集')
                save(path,settings)
            try:
                rows=collect(root,config)
            except (EOFError,KeyboardInterrupt):
                print('已暂停。使用同一数据集路径可继续。')
                return
        else:
            rows=read_rows(root)
        result=report(rows)
        save(root/'result.json',result)
        print('结果 {}，训练 RMS {:.2f} mm，测试 RMS {:.2f} mm，测试最大 {:.2f} mm'.format(
            root/'result.json',result['train']['rms_mm'],result['test']['rms_mm'],result['test']['max_mm']))


if __name__=='__main__':
    main()
