"""Finite RGB-D capture and a measured blue planar face, without learned models."""
import json
from pathlib import Path
import time

import cv2
import numpy as np

VERSION = 'blue-face-v1'


def wait(status, reason):
    return dict(status=status, reason=reason, point_color_m=None, detector_version=VERSION)


def intrinsics_valid(camera, shape):
    if shape != (camera['height'], camera['width']):
        raise ValueError('image dimensions do not match intrinsics')
    if not all(np.isfinite(camera[k]) and camera[k] > 0 for k in ('fx', 'fy', 'depth_scale')):
        raise ValueError('invalid camera focal length/depth scale')
    if not np.all(np.isfinite([camera['ppx'], camera['ppy']])):
        raise ValueError('invalid principal point')
    if np.any(np.asarray(camera.get('coeffs', [])) != 0):
        raise ValueError('nonzero distortion is unsupported')
    aligned = camera.get('aligned_intrinsics', camera)
    for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
        if not np.isclose(aligned[key], camera[key], atol=1e-6, rtol=0):
            raise ValueError('aligned depth/color intrinsics mismatch: ' + key)
    if np.any(np.asarray(aligned.get('coeffs', [])) != 0):
        raise ValueError('nonzero aligned distortion is unsupported')


def rays(pixels, camera):
    pixels = np.asarray(pixels)
    return np.column_stack(((pixels[:, 0] - camera['ppx']) / camera['fx'],
                            (pixels[:, 1] - camera['ppy']) / camera['fy'],
                            np.ones(len(pixels))))


def detect_blue_block(rgb, depth, camera, config):
    try:
        if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2]:
            raise ValueError('RGB/depth dimensions mismatch')
        intrinsics_valid(camera, depth.shape)
    except (ValueError, KeyError, TypeError) as error:
        return wait('WAIT_CAMERA', str(error))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(config['hsv_low']), np.array(config['hsv_high']))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, count) if stats[i, 4] >= config['min_area_px']]
    if len(candidates) != 1:
        return wait('WAIT_TARGET' if not candidates else 'WAIT_AMBIGUOUS',
                    'blue components={}; no arbitrary candidate selection'.format(len(candidates)))
    component = (labels == candidates[0]).astype(np.uint8)
    y, x = np.nonzero(component)
    if x.min() <= 2 or y.min() <= 2 or x.max() >= rgb.shape[1]-3 or y.max() >= rgb.shape[0]-3:
        return wait('WAIT_VISIBILITY', 'target touches image boundary')
    contours = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    polygon = cv2.approxPolyDP(hull, .018 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    if len(polygon) != 4 or cv2.contourArea(contour) / max(cv2.contourArea(hull), 1) < config['min_fill']:
        return wait('WAIT_VISIBILITY', 'face boundary is incomplete/occluded')
    rect = cv2.minAreaRect(contour)
    if component.sum() / max(rect[1][0]*rect[1][1], 1) < config['min_fill']:
        return wait('WAIT_VISIBILITY', 'face is not a complete rectangular component')
    interior = cv2.erode(component, np.ones((7, 7), np.uint8)).astype(bool)
    yy, xx = np.nonzero(interior)
    z = depth[interior].astype(float) * camera['depth_scale']
    valid = np.isfinite(z) & (z > 0)
    ratio = float(valid.mean()) if len(valid) else 0.
    if ratio < config['min_depth_ratio']:
        return wait('WAIT_DEPTH', 'insufficient valid interior depth')
    # Inverse depth is linear in camera rays for a plane; trim depth outliers twice.
    rr = rays(np.column_stack((xx[valid], yy[valid])), camera)
    z = z[valid]
    keep = np.ones(len(z), dtype=bool)
    for _ in range(3):
        plane = np.linalg.lstsq(rr[keep], 1. / z[keep], rcond=None)[0]
        prediction = 1. / (rr @ plane)
        keep = np.abs(prediction - z) <= config['max_plane_residual_m'] * 2
        if keep.mean() < .85:
            return wait('WAIT_DEPTH', 'face depth is not a consistent plane')
    residual = float(np.sqrt(np.mean((prediction[keep]-z[keep])**2)))
    if residual > config['max_plane_residual_m']:
        return wait('WAIT_DEPTH', 'plane residual too large')
    corner_rays = rays(polygon, camera)
    corner_z = 1. / (corner_rays @ plane)
    if not np.all(np.isfinite(corner_z)) or np.any(corner_z <= 0):
        return wait('WAIT_DEPTH', 'invalid face-plane intersection')
    corners = corner_rays * corner_z[:, None]
    lengths = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
    sizes = sorted([(lengths[0]+lengths[2])/2, (lengths[1]+lengths[3])/2])
    if not (config['width_m'][0] <= sizes[0] <= config['width_m'][1] and
            config['height_m'][0] <= sizes[1] <= config['height_m'][1]):
        return wait('WAIT_VISIBILITY', 'measured face dimensions outside local object range: '+str(sizes))
    center = corners.mean(axis=0)
    normal = plane / np.linalg.norm(plane)
    long_edge = corners[(int(np.argmax(lengths))+1) % 4] - corners[int(np.argmax(lengths))]
    long_edge /= np.linalg.norm(long_edge)
    if long_edge[1] < 0: long_edge *= -1
    return dict(status='DETECTED', reason='complete blue planar face', point_color_m=center.tolist(),
                boundary_px=polygon.tolist(), mask=component, width_m=float(sizes[0]), height_m=float(sizes[1]),
                normal_color=normal.tolist(), long_axis_color=long_edge.tolist(), depth_valid_ratio=ratio,
                plane_residual_m=residual, detector_version=VERSION)


def orientation_difference(a, b):
    angles = []
    for key in ('normal_color', 'long_axis_color'):
        angles.append(float(np.degrees(np.arccos(np.clip(np.dot(a[key], b[key]), -1, 1)))))
    return max(angles)


def combine(observations, config):
    for observation in observations:
        if observation['status'] != 'DETECTED': return observation
    if len(observations) != 3:
        return wait('WAIT_STABILITY', 'three independent frames required')
    points = np.array([o['point_color_m'] for o in observations])
    point = np.median(points, axis=0)
    if np.max(np.linalg.norm(points-point, axis=1)) > config['stability_m']:
        return wait('WAIT_STABILITY', 'target changed between frames')
    if any(orientation_difference(observations[0], o) > config['orientation_tolerance_deg'] for o in observations):
        return wait('WAIT_ORIENTATION', 'face orientation changed between frames')
    result = dict(observations[-1], point_color_m=point.tolist())
    result.pop('mask', None)
    result.pop('annotation_bgr', None)
    result['frame_count'] = len(observations)
    result['samples_color_m'] = points.tolist()
    return result


def verify_lift(before, after, expected_delta_base_m, extrinsics):
    result = dict(status='WAIT_VERIFY', reason='three visible, consistent post-lift observations required')
    if before.get('status') != 'DETECTED' or len(after) < 3 or any(o.get('status') != 'DETECTED' for o in after):
        return result
    for o in after:
        if any(abs(o[k]/before[k]-1) > .05 for k in ('width_m', 'height_m')) or orientation_difference(before, o) > 5:
            return dict(result, reason='face shape/orientation changed; occlusion or different object')
    rotation = np.asarray(extrinsics['T_base_color_optical'])[:3, :3]
    deltas = np.array([rotation @ (np.array(o['point_color_m'])-before['point_color_m']) for o in after])
    delta = np.median(deltas, axis=0)
    result['object_delta_base_m'] = delta.tolist()
    if np.max(np.linalg.norm(deltas-delta, axis=1)) > .003:
        return dict(result, reason='object not stable after lift')
    if not .01 <= delta[2] <= .03 or np.linalg.norm(delta[:2]) > .01 or np.linalg.norm(delta-np.array(expected_delta_base_m)) > .012:
        return dict(result, reason='object displacement does not support requested lift')
    return dict(result, status='VERIFIED', reason='same visible object lifted and stable')


def raw_depth_to_color(raw_depth, metadata):
    """Diagnostic reprojection; RealSense extrinsics rotation is column-major."""
    camera = dict(metadata['depth_intrinsics'], depth_scale=metadata['depth_scale'])
    intrinsics_valid(camera, raw_depth.shape)
    y, x = np.nonzero(raw_depth)
    points = rays(np.column_stack((x, y)), camera) * (raw_depth[y, x]*camera['depth_scale'])[:, None]
    ex = metadata['depth_to_color']
    rotation = np.asarray(ex['rotation']).reshape(3, 3, order='F')
    return points @ rotation.T + np.asarray(ex['translation'])


def load_capture(directory, config):
    directory = Path(directory)
    metadata = json.loads((directory/'metadata.json').read_text())
    if metadata['serial'] != config['camera_serial']:
        raise ValueError('camera serial mismatch')
    observations = []
    for frame in metadata['frames']:
        stem = frame['stem']
        if Path(stem).name != stem: raise ValueError('invalid frame path')
        rgb = cv2.cvtColor(cv2.imread(str(directory/(stem+'-color.png'))), cv2.COLOR_BGR2RGB)
        with np.load(directory/(stem+'-depth.npz'), allow_pickle=False) as archive:
            depth = archive['aligned_depth']
        camera = dict(metadata['color_intrinsics'], depth_scale=metadata['depth_scale'],
                      aligned_intrinsics=frame['aligned_intrinsics'])
        if config.get('mode') == 'semantic':
            from .semantic import detect
            detection = detect(rgb, depth, camera, config)
        else:
            detection = detect_blue_block(rgb, depth, camera, config['detector'])
        if detection['status'] == 'DETECTED' and 'boundary_px' in detection:
            image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.polylines(image, [np.array(detection['boundary_px'], dtype=np.int32)], True, (0, 255, 0), 2)
            detection['annotation_bgr'] = image
        detection['host_unix'] = frame['host_unix']
        observations.append(detection)
    return dict(metadata=metadata, observations=observations, observation=combine(observations, config['detector']))


def capture_rgbd(config, out_dir, count=3, detect=True, snapshot=None):
    import pyrealsense2 as rs
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    # Viewer global context may be an empty string; use the established capture-local settings.
    context = rs.context('{"inherit":false}')
    pipeline, settings = rs.pipeline(context), rs.config()
    settings.enable_device(config['camera_serial'])
    camera = config['camera']
    settings.enable_stream(rs.stream.color, camera['width'], camera['height'], rs.format.rgb8, camera['fps'])
    settings.enable_stream(rs.stream.depth, camera['width'], camera['height'], rs.format.z16, camera['fps'])
    def intr(profile):
        i = profile.as_video_stream_profile().get_intrinsics()
        return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.ppx, ppy=i.ppy,
                    coeffs=list(i.coeffs), model=str(i.model))
    try:
        profile = pipeline.start(settings)
        color_profile, depth_profile = profile.get_stream(rs.stream.color), profile.get_stream(rs.stream.depth)
        ex = depth_profile.get_extrinsics_to(color_profile)
        metadata = dict(serial=config['camera_serial'], depth_scale=profile.get_device().first_depth_sensor().get_depth_scale(),
                        color_intrinsics=intr(color_profile), depth_intrinsics=intr(depth_profile),
                        depth_to_color=dict(rotation=list(ex.rotation), translation=list(ex.translation)), frames=[])
        align = rs.align(rs.stream.color)
        for _ in range(camera['warmup_frames']): pipeline.wait_for_frames(3000)
        previous = None
        for index in range(count):
            before_read_unix=time.time()
            before=snapshot() if snapshot else None
            deadline = time.monotonic() + 3.
            numbers = previous
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ValueError('timed out waiting for distinct RGB-D frames: previous={}, current={}'.format(previous, numbers))
                frames = pipeline.wait_for_frames(max(1, int(remaining * 1000)))
                color, raw = frames.get_color_frame(), frames.get_depth_frame()
                numbers = (color.get_frame_number(), raw.get_frame_number())
                if previous and any(n < p for n, p in zip(numbers, previous)):
                    raise ValueError('camera frame numbers moved backwards: previous={}, current={}'.format(previous, numbers))
                # A synchronized frameset can reuse one stream; only save pairs with both streams new.
                if previous is None or all(n > p for n, p in zip(numbers, previous)):
                    break
            received_unix = time.time()
            after=snapshot() if snapshot else None
            after_read_unix=time.time()
            previous = numbers
            aligned = align.process(frames).get_depth_frame()
            stem = 'frame{:02d}'.format(index)
            if not cv2.imwrite(str(out_dir/(stem+'-color.png')), cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)):
                raise OSError('failed to save color image')
            np.savez_compressed(out_dir/(stem+'-depth.npz'), raw_depth=np.asanyarray(raw.get_data()),
                                aligned_depth=np.asanyarray(aligned.get_data()))
            metadata['frames'].append(dict(stem=stem, host_unix=received_unix, color_frame=numbers[0], depth_frame=numbers[1],
                color_timestamp_ms=color.get_timestamp(), depth_timestamp_ms=raw.get_timestamp(),
                color_time_domain=str(color.get_frame_timestamp_domain()), depth_time_domain=str(raw.get_frame_timestamp_domain()),
                color_intrinsics=intr(color.profile), depth_intrinsics=intr(raw.profile), aligned_intrinsics=intr(aligned.profile),
                depth_scale=metadata['depth_scale'], depth_to_color=metadata['depth_to_color']))
            if snapshot:
                metadata['frames'][-1]['paired_robot']=[dict(robot_snapshot=before),dict(robot_snapshot=after)]
                metadata['frames'][-1]['robot_pairing']=dict(before_read_unix=before_read_unix,
                    after_read_unix=after_read_unix,span_s=after_read_unix-before_read_unix)
        (out_dir/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    finally:
        try: pipeline.stop()
        except RuntimeError: pass  # start may have failed before the pipeline became active

    return load_capture(out_dir, config) if detect else dict(metadata=metadata,capture_dir=str(out_dir.resolve()))
