"""Metric board pose from measured RGB-D points. Frames use metres and column vectors."""
import argparse
import base64
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

from . import geometry, vision
from .main import ROOT, Events, clean, save
from .piper_io import PiperSession

TRAIN, VALID = 30, 5
LEGACY_SPLIT = (20, 8)
BOARD = dict(dictionary='DICT_ARUCO_ORIGINAL', marker_id=25, black_side_m=.100)
# ID-decoded order: top left, top right, bottom right, bottom left; +Z out of the print.
MODEL = .050 * np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float)
PREFIX = 'PIPER_BOARD '


def marker_corners(image):
    ar = cv2.aruco
    dictionary = ar.getPredefinedDictionary(ar.DICT_ARUCO_ORIGINAL)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(ar, 'ArucoDetector'):
        params = ar.DetectorParameters()
        params.cornerRefinementMethod = ar.CORNER_REFINE_SUBPIX
        corners, ids, _ = ar.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        params = ar.DetectorParameters_create()
        params.cornerRefinementMethod = ar.CORNER_REFINE_SUBPIX
        corners, ids, _ = ar.detectMarkers(gray, dictionary, parameters=params)
    hits = [] if ids is None else np.flatnonzero(ids.ravel() == BOARD['marker_id'])
    if len(hits) != 1:
        raise ValueError('需要完整可见且唯一的 ArUco ID 25，检出 {} 个'.format(len(hits)))
    return corners[hits[0]].reshape(4, 2).astype(float)


def cloud_on_color(raw, meta):
    """Project true depth-camera XYZ into RGB. Keep the nearest surface at each RGB pixel."""
    camera = meta['color_intrinsics']
    shape = (camera['height'], camera['width'])
    vision.intrinsics_valid(dict(camera, depth_scale=meta['depth_scale']), shape)
    cloud = vision.raw_depth_to_color(raw, meta)
    cloud = cloud[cloud[:, 2] > 0]
    uv = np.rint(cloud[:, :2]/cloud[:, 2:] * [camera['fx'], camera['fy']] +
                 [camera['ppx'], camera['ppy']]).astype(int)
    inside = np.all((uv >= 0) & (uv < [shape[1], shape[0]]), axis=1)
    uv, cloud = uv[inside], cloud[inside]
    index = uv[:, 1]*shape[1]+uv[:, 0]
    order = np.lexsort((cloud[:, 2], index))
    keep = order[np.unique(index[order], return_index=True)[1]]
    return uv[keep], cloud[keep]


def proper_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


def board_pose(corners, uv, cloud, camera):
    shape = (camera['height'], camera['width'])
    polygon = np.zeros(shape, np.uint8)
    inset = corners.mean(axis=0) + .88*(corners-corners.mean(axis=0))
    cv2.fillConvexPoly(polygon, np.rint(inset).astype(np.int32), 1)
    roi = polygon[uv[:, 1], uv[:, 0]] != 0
    points, pixels = cloud[roi], uv[roi]
    if len(points) < 100:
        raise ValueError('标记内部有效深度少于 100 点，请调整板的距离或倾角')
    rng = np.random.RandomState(25)
    probe = points[rng.choice(len(points), min(1500, len(points)), replace=False)]
    best = 0
    for _ in range(120):
        a, b, c = probe[rng.choice(len(probe), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        score = np.count_nonzero(np.abs((probe-a) @ normal) <= .002)
        if score > best:
            best, origin, n = score, a, normal
    if best < .55*len(probe):
        raise ValueError('标记内部没有一致的深度平面')
    for _ in range(3):
        valid = np.abs((points-origin) @ n) <= .002
        face = points[valid]
        if len(face) < max(100, .50*len(points)):
            raise ValueError('板面深度内点不足')
        origin = face.mean(axis=0)
        _, vec = np.linalg.eigh((face-origin).T @ (face-origin))
        n = vec[:, 0]
    support = pixels[valid].astype(np.float32)
    area = cv2.contourArea(cv2.convexHull(support)) / cv2.contourArea(inset.astype(np.float32))
    if area < .55:
        raise ValueError('深度点只覆盖标记局部，覆盖面积 {:.0%}'.format(area))
    rays = vision.rays(corners, camera)
    denom = rays @ n
    if np.min(np.abs(denom)) < .10:
        raise ValueError('标记过于侧向')
    xyz = rays * ((origin @ n)/denom)[:, None]
    lengths = np.linalg.norm(xyz-np.roll(xyz, 1, axis=0), axis=1)
    if np.any(xyz[:, 2] <= 0) or np.max(np.abs(lengths-.100)) > .006:
        raise ValueError('深度平面与 100 mm 黑框不符，四边长 {} mm'.format(np.round(1000*lengths, 1)))
    T = np.eye(4)
    T[:3, :3] = proper_rotation((xyz-xyz.mean(axis=0)).T @ MODEL)
    T[:3, 3] = xyz.mean(axis=0)
    metric_rms = np.sqrt(np.mean(np.sum((MODEL @ T[:3,:3].T + T[:3,3]-xyz)**2, axis=1)))
    if metric_rms > .003:
        raise ValueError('三维角点与正方形不一致')
    return dict(T_color_marker=T, corners_px=corners, corners_color_m=xyz, side_lengths_mm=lengths*1000,
                plane_rms_mm=1000*np.sqrt(np.mean(((face-origin) @ n)**2)), coverage=area,
                inliers=len(face), corner_fit_rms_mm=metric_rms*1000), support.astype(int), face


def write_ply(path, points):
    header = ('ply\nformat binary_little_endian 1.0\ncomment color optical frame; metres\n'
              'element vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
    with path.open('wb') as f:
        f.write(header.format(len(points)).encode('ascii'))
        f.write(points.astype('<f4').tobytes())


def mean_pose(poses):
    T = np.eye(4)
    T[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    T[:3, :3] = proper_rotation(sum(p[:3, :3] for p in poses))
    return T


def pose_error(a, b):
    angle = np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T @ b[:3,:3])-1)/2, -1., 1.)))
    return 1000*np.linalg.norm(a[:3,3]-b[:3,3]), angle


def calibration_model():
    directory = ROOT/'config/piper_vision_grasp'
    cfg = json.loads((directory/'semantic-camera-avoid.json').read_text())
    cfg['kinematics'] = json.loads((directory/cfg['kinematics']).read_text())
    geometry.load_joint_calibration(cfg, directory)
    return cfg['kinematics']


def arm_pose(frames):
    model = calibration_model()
    poses, joints, widths = [], [], []
    for frame in frames:
        for row in frame['paired_robot']:
            r = row['robot_snapshot']
            stopped = r.get('ctrl_mode') == 2 and r.get('teach_status') == 2 and r['state']=='WAIT_CONTROL'
            if (r['state']!='READY' and not stopped) or r.get('missing_feedback') or r.get('err_code') or r.get('arm_status'):
                raise ValueError('反馈不可用：'+str(r.get('reason', r['state'])))
            # R4 reprocessed archives explicitly retain the original sensor readings.
            sensor = r.get('joints_deg_raw', r['joints_deg'])
            poses.append(geometry.fk(sensor, model))
            joints.append(sensor)
            widths.append(r['gripper_width_mm'])
    delta = np.max([pose_error(poses[0], p) for p in poses], axis=0)
    if delta[0] > 1 or delta[1] > .5 or np.max(np.ptp(joints, axis=0)) > .3 or np.ptp(widths) > .5:
        raise ValueError('拍摄窗口内机械臂或夹爪发生移动')
    return mean_pose(poses)


def draw_border(image, corners):
    points = np.rint(corners).astype(np.int32)
    cv2.polylines(image, [points], True, (0,230,255), 2, cv2.LINE_AA)
    center = corners.mean(axis=0)
    for letter, corner, point in zip('ABCD', corners, points):
        cv2.drawMarker(image, tuple(point), (255,0,255), cv2.MARKER_CROSS, 9, 1)
        direction = corner-center
        label = point + np.rint(14*direction/max(np.linalg.norm(direction),1)).astype(int)
        origin = tuple(np.clip(label, (0,15), (image.shape[1]-15,image.shape[0]-3)))
        cv2.putText(image, letter, origin, cv2.FONT_HERSHEY_SIMPLEX, .55, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(image, letter, origin, cv2.FONT_HERSHEY_SIMPLEX, .55, (0,230,255), 1, cv2.LINE_AA)


def measure(directory, meta, on_borders=None):
    frames = meta['frames']
    detected = []
    border_paths = []
    for frame in frames:
        stem = frame['stem']
        image = cv2.imread(str(directory/(stem+'-color.png')))
        if image is None:
            raise OSError(stem+' 彩色图读取失败')
        try:
            corners = marker_corners(image)
            draw_border(image, corners)
            issue = None
        except ValueError as exc:
            corners, issue = None, str(exc)
            cv2.putText(image, 'ID 25: no unique detection', (8,26), cv2.FONT_HERSHEY_SIMPLEX,
                        .7, (0,0,255), 2, cv2.LINE_AA)
        path = directory/(stem+'-border.jpg')
        if not cv2.imwrite(str(path), image):
            raise OSError('黑框标注图写入失败')
        border_paths.append(path)
        detected.append((stem,image,corners,issue))
    if on_borders:
        on_borders(border_paths)
    if len(frames)!=3 or any(np.any(np.diff([f[k] for f in frames])<=0) for k in ('color_frame','depth_frame')):
        raise ValueError('需要三对独立的新 RGB-D 帧')
    arm = arm_pose(frames)
    details = []
    for stem,image,corners,issue in detected:
        if issue:
            raise ValueError(stem+' · '+issue)
        with np.load(directory/(stem+'-depth.npz'), allow_pickle=False) as z:
            uv, cloud = cloud_on_color(z['raw_depth'], meta)
        try:
            row, pixels, face = board_pose(corners, uv, cloud, meta['color_intrinsics'])
        except ValueError as exc:
            raise ValueError(stem+' · '+str(exc)) from exc
        write_ply(directory/(stem+'-board.ply'), face)
        details.append(row)
        sample = pixels[::max(1,len(pixels)//700)]
        image[sample[:,1], sample[:,0]] = (0,220,0)
        if not cv2.imwrite(str(directory/(stem+'-review.jpg')), image):
            raise OSError('审核图写入失败')
    poses = [r['T_color_marker'] for r in details]
    spread = np.max([pose_error(a,b) for i,a in enumerate(poses) for b in poses[i+1:]],axis=0)
    if spread[0] > 3 or spread[1] > 2:
        raise ValueError('三帧板位姿最大差 {:.2f} mm / {:.2f}°'.format(*spread))
    return dict(T_base_end=arm, T_color_marker=mean_pose(poses), frames=details,
                max_frame_translation_mm=spread[0], max_frame_angle_deg=spread[1])


def fit(rows):
    arms = [geometry.transform(r['T_base_end']) for r in rows]
    boards = [geometry.transform(r['T_color_marker']) for r in rows]
    rotvec = [cv2.Rodrigues(arms[0][:3,:3].T @ a[:3,:3])[0].ravel() for a in arms[1:]]
    singular = np.linalg.svd(rotvec, compute_uv=False)
    if len(singular)<2 or singular[1]<.01:
        raise ValueError('旋转激励不足，请绕不同轴改变姿态')
    inv = [np.linalg.inv(a) for a in arms]
    rot,t = cv2.calibrateHandEye([a[:3,:3] for a in inv], [a[:3,3] for a in inv],
                                 [b[:3,:3] for b in boards], [b[:3,3] for b in boards],
                                 method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4)
    X[:3,:3], X[:3,3] = rot,t.ravel()
    geometry.transform(X)
    return X, mean_pose([a @ X @ b for a,b in zip(inv,boards)]), singular


def residuals(rows, X, mount):
    err = np.array([pose_error(geometry.transform(r['T_base_end']) @ mount,
                               X @ geometry.transform(r['T_color_marker'])) for r in rows])
    return dict(count=len(rows), rms_mm_deg=np.sqrt(np.mean(err**2,axis=0)),
                max_mm_deg=err.max(axis=0), per_pose_mm_deg={r['id']:e for r,e in zip(rows,err)})


def report(rows, train_count=TRAIN, validation_count=VALID):
    if len(rows)!=train_count+validation_count or any(r['role']!=role(i,train_count) or r['id']!='pose%02d'%i
                                     for i,r in enumerate(rows,1)):
        raise ValueError('需要完整且有序的 {} 组训练与 {} 组验证'.format(train_count,validation_count))
    train = rows[:train_count]
    X,mount,sv = fit(train)
    return dict(board=BOARD, units='metres', method='raw depth cloud in color optical frame; PARK eye-to-hand',
                T_base_color_optical=X,T_color_optical_base=np.linalg.inv(X),T_end_marker=mount,
                rotation_singular_values_rad=sv,training=residuals(train,X,mount),
                validation=residuals(rows[train_count:],X,mount))


def role(index, train_count=TRAIN):
    return 'train' if index<=train_count else 'validation'


def send(state, message, token='', images=(), **fields):
    payload = dict(state=state, message=message, token=token, **fields)
    if images:
        payload['images'] = [base64.b64encode(data).decode('ascii') for data in images]
    print(PREFIX+json.dumps(clean(payload),ensure_ascii=False,allow_nan=False),flush=True)


def command(token, allowed):
    while True:
        line = sys.stdin.readline()
        if not line or line.strip()=='quit':
            raise EOFError
        for action in allowed:
            if line.strip() == action+' '+token:
                return action


def preview(pipe, token, title):
    send('busy', title+' · 等待新预览')
    while True:
        frame = pipe.wait_for_frames(3000).get_color_frame()
        if frame:
            rgb = cv2.resize(np.asanyarray(frame.get_data()), (320,240), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            if ok:
                send('preview',title,token,[jpg.tobytes()])
        if select.select([sys.stdin],[],[],2)[0]:
            line = sys.stdin.readline()
            if not line or line.strip()=='quit':
                raise EOFError
            if line.strip()=='shoot '+token:
                return


def collect(root, config, rows, events, train_count, validation_count):
    with PiperSession(dict(config,readonly=True)) as session:
        while len(rows)<train_count+validation_count:
            index = len(rows)+1
            name = 'pose%02d'%index
            title = '{} · {} · 已收录训练 {}/{}，验证 {}/{}'.format(
                name,role(index,train_count),min(len(rows),train_count),train_count,max(len(rows)-train_count,0),validation_count)
            token = uuid.uuid4().hex[:12]
            directory = root/'attempts'/(name+'-'+token)
            try:
                capture = vision.capture_rgbd(config,directory,detect=False,snapshot=session.snapshot,
                                              on_ready=lambda pipe:preview(pipe,token,title))
                def show_borders(paths):
                    send('border',name+' · 黄线为检测黑框，A–D 为实际角点（三帧从左到右）。',
                         token,[p.read_bytes() for p in paths],pose=name)
                sample = measure(directory,capture['metadata'],show_borders)
                sample.update(id=name,role=role(index,train_count),joint_offset_deg=config['kinematics']['joint_offset_deg'])
                for old in rows:
                    mm,deg = pose_error(sample['T_base_end'],geometry.transform(old['T_base_end']))
                    if mm<1 and deg<1:
                        raise ValueError('与已收录的 '+old['id']+' 姿态重复')
                trend = ''
                if index<=train_count and index%5==0:
                    try:
                        X,mount,_ = fit(rows+[sample])
                        rms = residuals(rows+[sample],X,mount)['rms_mm_deg']
                        trend = '；累计训练 RMS {:.2f} mm / {:.2f}°'.format(*rms)
                    except ValueError as exc:
                        if index==train_count:
                            raise
                        trend = '；'+str(exc)
                save(directory/'sample.json',sample)
                images = [p.read_bytes() for p in sorted(directory.glob('*-review.jpg'))]
                summary = ('{} · 三帧最大差 {:.2f} mm / {:.2f}°；平面 RMS {} mm{}'
                           .format(name,sample['max_frame_translation_mm'],sample['max_frame_angle_deg'],
                                   ', '.join('{:.2f}'.format(r['plane_rms_mm']) for r in sample['frames']),trend))
                events.emit('REVIEW',summary,name,attempt=str(directory))
                send('review',summary,token,images,pose=name,
                     captured_at=capture['metadata']['frames'][1]['host_unix'])
                if command(token,['accept','reject'])=='accept':
                    directory.rename(root/name)
                    rows.append(sample)
                    events.emit('ACCEPTED',role(index,train_count),name)
                    send('accepted','已收录 {}（{}）。'.format(name,role(index,train_count)))
                else:
                    events.emit('REJECTED','人工弃用',name)
                    send('rejected','已弃用本次拍摄，原始文件保留。请重新摆姿拍摄 '+name+'。')
            except (OSError,ValueError,RuntimeError,cv2.error) as exc:
                events.emit('WAIT_SAMPLE',str(exc),name,attempt=str(directory))
                send('retry',name+' · '+str(exc),token)
                command(token,['retry'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['collect','solve'])
    parser.add_argument('--dataset',required=True,type=Path)
    args = parser.parse_args()
    root = args.dataset.expanduser().resolve()
    if args.action=='collect':
        root.mkdir(parents=True,exist_ok=True)
    with (root/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        cfg = json.loads((ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json').read_text())
        config = {key:cfg[key] for key in ('camera','camera_serial','usb_serial')}
        model = calibration_model()
        config['kinematics'] = model
        settings = dict(schema=1,board=BOARD,train=TRAIN,validation=VALID,arm_frame='Piper SDK end',**{k:v for k,v in config.items() if k!='kinematics'})
        historical = settings
        settings = dict(settings,schema=2,arm_frame='link6 FK',joint_offset_deg=model['joint_offset_deg'])
        path = root/'settings.json'
        stored = None
        if path.exists():
            stored = json.loads(path.read_text())
            legacy = dict(settings,train=LEGACY_SPLIT[0],validation=LEGACY_SPLIT[1])
            if stored not in (settings,legacy,historical,dict(historical,train=LEGACY_SPLIT[0],validation=LEGACY_SPLIT[1])):
                raise ValueError('续采配置不匹配')
        elif args.action!='collect' or any(p.name!='.lock' for p in root.iterdir()):
            raise ValueError('请选择本方案的数据集')

        directories = sorted(root.glob('pose[0-9][0-9]'))
        plan = settings
        # Keep an existing holdout independent, including when replaying a completed legacy run.
        if stored and (args.action=='solve' or len(directories)>stored['train']):
            plan = dict(settings,train=stored['train'],validation=stored['validation'])
        train_count, validation_count = plan['train'], plan['validation']
        rows = []
        for index,d in enumerate(directories,1):
            row = json.loads((d/'sample.json').read_text())
            if row['id']!=d.name or d.name!='pose%02d'%index or row['role']!=role(index,train_count):
                raise ValueError('样本编号或角色不连续')
            if row.get('joint_offset_deg') != model['joint_offset_deg']:
                metadata = json.loads((d/'metadata.json').read_text())
                row['T_base_end'] = arm_pose(metadata['frames'])
            rows.append(row)
        if len(rows)>train_count+validation_count:
            raise ValueError('现有样本数超过本批次计划')
        if stored!=plan:
            save(path,plan)
        if args.action=='collect':
            events = Events(dict(reference=dict(reference_id='aruco25-depth-100mm')),root)
            events.emit('STARTING','等待手动摆姿',accepted=len(rows))
            try:
                collect(root,config,rows,events,train_count,validation_count)
            except (KeyboardInterrupt,EOFError):
                events.emit('PAUSED','人为暂停或控制端连接关闭',accepted=len(rows))
                return
        result = report(rows,train_count,validation_count)
        result.update(arm_frame=plan['arm_frame'],joint_offset_deg=plan['joint_offset_deg'])
        save(root/'result.json',result)
        send('done','求解完成，验证 RMS {:.2f} mm / {:.2f}°'.format(*result['validation']['rms_mm_deg']),
             result=result)


if __name__=='__main__':
    main()
