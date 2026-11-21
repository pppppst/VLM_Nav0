"""GraspNet transport and camera-grasp to Piper-tool adaptation (no Torch/CAN)."""
import base64
import copy
import io
import json
from pathlib import Path
from urllib import request
import zipfile

import cv2
import numpy as np

from . import geometry, vision

TOOL_AXES = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
MAX_POINTS = 640 * 480
MAX_BODY = 8 * 1024 * 1024


def validate_cloud(points, target):
    points, target = np.asarray(points), np.asarray(target)
    if (points.ndim != 2 or points.shape[1] != 3 or not 0 < len(points) <= MAX_POINTS
            or not np.all(np.isfinite(points)) or np.any(points[:, 2] <= 0)):
        raise ValueError('invalid metric optical-frame point cloud')
    if target.shape != (len(points),) or target.dtype != np.bool_ or not target.any():
        raise ValueError('target mask must identify valid cloud points')
    return points.astype(np.float32), target


def point_cloud(depth, camera, mask):
    depth, mask = np.asarray(depth), np.asarray(mask)
    vision.intrinsics_valid(camera, depth.shape)
    if mask.shape != depth.shape:
        raise ValueError('target/depth dimensions mismatch')
    z = depth.astype(float) * camera['depth_scale']
    valid = np.isfinite(z) & (z > 0)
    y, x = np.nonzero(valid)
    points = vision.rays(np.column_stack((x, y)), camera) * z[valid, None]
    target = mask.astype(bool)[valid]
    if target.any():
        # Match semantic.measure: SAM boundary pixels can contain distant background depth.
        q1, q3 = np.percentile(points[target, 2], [25, 75])
        spread = max(q3-q1, camera['depth_scale'])
        target &= (points[:, 2] >= q1-1.5*spread) & (points[:, 2] <= q3+1.5*spread)
    return validate_cloud(points, target)


def encode_cloud(points, target):
    points, target = validate_cloud(points, target)
    stream = io.BytesIO()
    np.savez_compressed(stream, points=points, target=target)
    return dict(frame='color_optical', units='m', cloud_npz=base64.b64encode(stream.getvalue()).decode('ascii'))


def decode_cloud(row):
    if row.get('frame') != 'color_optical' or row.get('units') != 'm':
        raise ValueError('expected color_optical frame in metres')
    raw = base64.b64decode(row['cloud_npz'], validate=True)
    if len(raw) > MAX_BODY:
        raise ValueError('cloud payload too large')
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        if sorted(archive.namelist()) != ['points.npy', 'target.npy'] or sum(i.file_size for i in archive.infolist()) > MAX_BODY:
            raise ValueError('invalid cloud archive')
    with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
        return validate_cloud(archive['points'], archive['target'])


def absolute_target(candidate, config):
    try:
        rotation = np.asarray(candidate['rotation'], dtype=float)
        translation = np.asarray(candidate['translation'], dtype=float)
        score, width, height, depth = [float(candidate[k]) for k in ('score', 'width', 'height', 'depth')]
        if (rotation.shape != (3, 3) or translation.shape != (3,) or
                not np.all(np.isfinite([score, width, height, depth])) or
                not 0 < width <= .1 or not 0 < height <= .1 or not 0 < depth <= .04 + 1e-8):
            raise ValueError('invalid GraspNet candidate values')
        grasp = np.eye(4); grasp[:3, :3] = rotation; grasp[:3, 3] = translation
        geometry.transform(grasp)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('invalid GraspNet candidate') from exc
    tip = grasp.copy()
    tip[:3, :3] = rotation @ TOOL_AXES
    tip[:3, 3] += depth * rotation[:, 0]
    tip = geometry.transform(config['extrinsics']['T_base_color_optical']) @ tip
    tip[:3, 3] += np.asarray(config['semantic']['grasp_offset_base_m'])
    flange = tip.copy()
    flange[:3, 3] -= tip[:3, :3] @ np.asarray(config['semantic']['tcp_offset_m'])
    geometry.transform(flange)
    # Decoded width includes the official 1.2 opening allowance; it is not object width.
    return dict(T_base_tcp=tip.tolist(), T_base_flange=flange.tolist(),
                grasp_width_m=width / 1.2, candidate=dict(candidate), reference_id='semantic-graspnet-v1',
                tcp_offset_m=config['semantic']['tcp_offset_m'],
                tcp_status='legacy tip offset; physical calibration unverified')


def select_plan(config, joints, candidates):
    rejected = []
    validated = []
    for index, candidate in enumerate(candidates):
        try:
            target = absolute_target(candidate, config)
            if candidate['width'] * 1000 > config['gripper']['open_mm']:
                raise ValueError('candidate opening exceeds configured gripper opening')
            validated.append((index, target))
        except ValueError as exc:
            rejected.append(dict(index=index, reason=str(exc)))
    for index, target in sorted(validated, key=lambda item: -item[1]['candidate']['score'])[:50]:
        try:
            plan = geometry.plan_absolute(joints, config, target)
        except ValueError as exc:
            plan = dict(status='WAIT_IK', reason=str(exc))
        if plan['status'] == 'PLANNED':
            plan['reference_id'] = 'semantic-graspnet-v1'
            plan['scope'] = 'GraspNet pose; existing Piper path and legacy TCP'
            plan['graspnet'] = dict(selected_index=index, rejected=rejected, candidates=candidates)
            return plan
        rejected.append(dict(index=index, reason=plan.get('reason', plan['status'])))
    return dict(status='WAIT_PLAN', reason='no GraspNet candidate passes existing complete path planning',
                graspnet=dict(selected_index=None, rejected=rejected, candidates=candidates))


def capture_cloud(captured):
    directory = Path(captured['capture_dir'])
    metadata = captured['metadata']; frame = metadata['frames'][-1]
    stem = frame['stem']
    if Path(stem).name != stem:
        raise ValueError('invalid frame path')
    with np.load(directory / (stem + '-depth.npz'), allow_pickle=False) as archive:
        depth = archive['aligned_depth']
    mask = captured['observations'][-1]['mask']
    camera = dict(metadata['color_intrinsics'], depth_scale=metadata['depth_scale'],
                  aligned_intrinsics=frame['aligned_intrinsics'])
    return point_cloud(depth, camera, mask)


def plan_capture(config, joints, captured):
    points, target = capture_cloud(captured)
    settings = config['graspnet']
    payload = encode_cloud(points, target)
    payload['max_width_m'] = config['gripper']['open_mm'] / 1000.
    req = request.Request(settings['endpoint'].rstrip('/') + '/grasps',
                          data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json'})
    try:
        with request.urlopen(req, timeout=settings['timeout_s']) as response:
            raw = response.read(MAX_BODY + 1)
        if len(raw) > MAX_BODY:
            raise ValueError('GraspNet response too large')
        result = json.loads(raw)
        if result.get('frame') != 'color_optical' or result.get('units') != 'm':
            raise ValueError('GraspNet response frame/units mismatch')
        if not isinstance(result.get('candidates'), list) or len(result['candidates']) > 50:
            raise ValueError('invalid GraspNet candidate list')
        plan = select_plan(config, joints, result['candidates'])
        plan['graspnet']['inference'] = {k: v for k, v in result.items() if k != 'candidates'}
        plan['graspnet']['frame_stem'] = captured['metadata']['frames'][-1]['stem']
        plan['graspnet']['planning_config'] = copy.deepcopy({key: config[key] for key in
            ('extrinsics','kinematics','semantic','detector','gripper','target','camera_serial')})
        return plan
    except (OSError, KeyError, TypeError) as exc:
        raise ValueError('GraspNet inference unavailable: ' + str(exc)) from exc


def load_saved_capture(directory, config):
    """Explicit offline replay: reuse saved semantic results, never call cloud/CAN."""
    directory = Path(directory)
    captured = json.loads((directory / 'observation.json').read_text())
    metadata = json.loads((directory / 'metadata.json').read_text())
    if metadata != captured['metadata'] or metadata['serial'] != config['camera_serial']:
        raise ValueError('saved observation/capture metadata mismatch')
    if len(captured['observations']) != 3 or len(metadata['frames']) != 3:
        raise ValueError('saved capture needs three observations')
    target = captured['observation']['target']
    if not isinstance(target, dict) or not all(isinstance(target.get(k), str) and target[k] for k in ('color', 'description')):
        raise ValueError('saved target specification missing')
    if any(obs.get('target') != target for obs in captured['observations']):
        raise ValueError('saved target specifications differ across frames')
    for index, obs in enumerate(captured['observations']):
        mask = cv2.imread(str(directory / ('mask{:02d}.png'.format(index))), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError('saved target mask missing')
        obs['mask'] = mask > 0
    captured['observation'] = vision.combine(captured['observations'], config['detector'])
    if captured['observation']['status'] != 'DETECTED':
        raise ValueError('saved object observation not stable: ' + captured['observation']['reason'])
    captured['capture_dir'] = str(directory.resolve())
    return captured
