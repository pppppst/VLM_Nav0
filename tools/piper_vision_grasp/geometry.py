"""Pure geometry and provenance; metres/radians internally, SDK degrees at edges."""
import hashlib
import json
from pathlib import Path


def validate_reference(reference, directory):
    if reference.get('phase') != 'before_close':
        raise ValueError('reference phase must be before_close')
    times = reference['timestamps']
    if not max(times['rgbd']) < times['grasp_pose'] < times['close'] < times['lift']:
        raise ValueError('reference phase timestamps are inconsistent')
    sources = reference.get('source_sha256', {})
    if not sources or set(sources) != set(reference.get('source_files', {})):
        raise ValueError('reference sources missing')
    for name, digest in sources.items():
        path = (Path(directory) / name).resolve()
        if Path(directory).resolve() not in path.parents:
            raise ValueError('source path outside bundle')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('source missing or hash mismatch: ' + name)
    def last_snapshot(name):
        rows = [json.loads(line) for line in (Path(directory)/name).read_text().splitlines() if line.startswith('{')]
        return [row for row in rows if row.get('joints_deg')][-1]
    grasp = last_snapshot(reference['grasp_pose_source'])
    if grasp['joints_deg'] != reference['joints_deg'] or grasp['time'] != times['grasp_pose']:
        raise ValueError('reference source grasp pose differs from derived fields')
    if grasp['sdk_pose_mm_xyz'] != reference['sdk_pose_mm_xyz']:
        raise ValueError('reference source SDK position mismatch')
    metadata = json.loads((Path(directory)/reference['rgbd_metadata_source']).read_text())
    if [f['host_unix'] for f in metadata['frames']] != times['rgbd'] or metadata['serial'] != reference['camera_serial']:
        raise ValueError('reference source capture phase/identity mismatch')
    return reference

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def rotation_matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def rotation_from_matrix(matrix, factory=Rotation):
    return factory.from_matrix(matrix) if hasattr(factory, 'from_matrix') else factory.from_dcm(matrix)


def transform(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)) or not np.allclose(result[3], [0,0,0,1]):
        raise ValueError('invalid homogeneous transform')
    r = result[:3,:3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('invalid rotation')
    return result


def limits(model):
    joints = model['joints']
    if len(joints) != 6: raise ValueError('six joints required')
    bounds = np.array([j['limit_rad'] for j in joints], dtype=float)
    if bounds.shape != (6,2) or not np.all(np.isfinite(bounds)) or np.any(bounds[:,0] >= bounds[:,1]):
        raise ValueError('invalid joint limits')
    return bounds


def model_angles(sensor_deg, model):
    """Actual URDF angles from controller angles, both in degrees."""
    return np.asarray(sensor_deg, dtype=float) + model.get('joint_offset_deg', 0.)


def load_joint_calibration(config, directory):
    if 'joint_zero_calibration' in config:
        source = (directory/config['joint_zero_calibration']).resolve()
        if directory not in source.parents:
            raise ValueError('joint calibration outside config bundle')
        offset = np.array(json.loads(source.read_text())['joint_offset_deg'], dtype=float)
        if offset.shape != (6,) or not np.all(np.isfinite(offset)):
            raise ValueError('six finite joint zero offsets required')
        config['kinematics']['joint_offset_deg'] = offset.tolist()


def fk(angles_deg, model):
    angles = np.radians(np.asarray(angles_deg, dtype=float))
    bounds = limits(model)
    if angles.shape != (6,) or not np.all(np.isfinite(angles)) or np.any(angles < bounds[:,0]-1e-9) or np.any(angles > bounds[:,1]+1e-9):
        raise ValueError('joint limit violation')
    # IK optimizes controller coordinates and thus returns controller-ready setpoints.
    angles = np.radians(model_angles(angles_deg, model))
    result = np.eye(4)
    for angle, joint in zip(angles, model['joints']):
        origin = np.eye(4)
        origin[:3,:3] = rotation_matrix(Rotation.from_euler('xyz', joint['rpy_rad']))
        origin[:3,3] = joint['xyz_m']
        axis = np.asarray(joint['axis'], dtype=float)
        if not np.isclose(np.linalg.norm(axis),1): raise ValueError('invalid joint axis')
        motion = np.eye(4)
        motion[:3,:3] = rotation_matrix(Rotation.from_rotvec(axis*angle))
        result = result @ origin @ motion
    return result


def solve_ik(target, seed_deg, model):
    target = transform(target)
    tolerance=float(model.get('ik_position_tolerance_m',.001))
    if not np.isfinite(tolerance) or tolerance<=0:raise ValueError('invalid IK position tolerance')
    bounds = limits(model)
    # IK setpoints must satisfy both the existing controller bounds and URDF
    # bounds expressed in sensor coordinates. FK can still describe measured zero.
    offset = np.radians(model_angles(np.zeros(6), model))
    bounds = np.column_stack((np.maximum(bounds[:,0], bounds[:,0]-offset),
                              np.minimum(bounds[:,1], bounds[:,1]-offset)))
    seed = np.clip(np.radians(seed_deg), bounds[:,0]+1e-9, bounds[:,1]-1e-9)
    def residual(q):
        actual = fk(np.degrees(q), model)
        rotation_error = rotation_from_matrix(target[:3,:3].T @ actual[:3,:3]).as_rotvec()
        return np.concatenate((actual[:3,3]-target[:3,3], rotation_error*.15))
    solution = least_squares(residual, seed, bounds=(bounds[:,0],bounds[:,1]),
                             max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    q = np.degrees(solution.x)
    error = residual(solution.x)
    pos, rot = float(np.linalg.norm(error[:3])), float(np.degrees(np.linalg.norm(error[3:])/.15))
    return dict(status='SOLVED' if pos <= tolerance and rot <= 1 else 'WAIT_IK', angles_deg=q.tolist(),
                position_error_m=pos, rotation_error_deg=rot,
                position_tolerance_m=tolerance,rotation_tolerance_deg=1.)


def retarget(reference, observation, extrinsics):
    # shortcut: same object face/installation near this reference; rebuild reference after geometry or zero changes.
    if reference.get('status') != 'ready' or observation.get('status') != 'DETECTED':
        raise ValueError('reference/observation not ready')
    if observation['detector_version'] != reference['detector_version']:
        raise ValueError('reference detector version mismatch')
    from .vision import orientation_difference
    if orientation_difference(reference['observation'], observation) > reference['orientation_tolerance_deg']:
        raise ValueError('local reference orientation exceeded')
    old, new = np.asarray(reference['point_color_m']), np.asarray(observation['point_color_m'])
    if old.shape != (3,) or new.shape != (3,) or not np.all(np.isfinite([old,new])):
        raise ValueError('invalid visual point')
    delta = transform(extrinsics['T_base_color_optical'])[:3,:3] @ (new-old)
    if np.any(np.abs(delta) > np.asarray(reference['local_limit_m'])+1e-9):
        raise ValueError('outside local reference experiment range; not a robot reachability claim')
    goal = transform(reference['T_base_flange']).copy()
    goal[:3,3] += delta
    return dict(delta_base_m=delta.tolist(), T_base_flange=goal.tolist(), reference_id=reference['reference_id'])


def plan_motion(current_deg, reference, target, model):
    start = fk(current_deg, model)
    goal = transform(target['T_base_flange'])
    stages = []
    seed = np.asarray(current_deg, dtype=float)
    spacing = reference['sample_spacing_m']
    if not np.isfinite(spacing) or spacing<=0: raise ValueError('invalid sampling interval')
    def record(name, q, pose, phase, errors=None):
        nonlocal seed
        entry = dict(stage_id=name, angles_deg=np.asarray(q).tolist(), T_base_flange=pose.tolist(), phase=phase,
                     position_error_m=0., rotation_error_deg=0., max_joint_change_deg=float(np.max(np.abs(q-seed))))
        if errors: entry.update({k:errors[k] for k in ('position_error_m','rotation_error_deg')})
        # Audit the assumed joint interpolation; it is not a Cartesian/collision guarantee.
        samples = [fk(seed + t*(q-seed), model)[:3,3].tolist() for t in np.linspace(0,1,11)]
        entry['joint_interpolation_flange_samples_m'] = samples
        stages.append(entry)
        seed = np.asarray(q)
    for i, waypoint in enumerate(reference['transition_joints_deg']):
        waypoint = np.asarray(waypoint)
        if waypoint[1] <= seed[1]: continue
        initial = seed.copy()
        count = max(1,int(np.ceil(np.max(np.abs(waypoint-initial))/10)))
        for j in range(1,count+1):
            q = initial+(waypoint-initial)*j/count
            record('unloaded-{}-{}'.format(i,j),q,fk(q,model),'unloaded')
    pre = goal.copy()
    pre[:3,3] += reference['pregrasp_offset_base_m']
    upper = pre.copy()
    upper[:3,3] += reference.get('entry_offset_base_m',[0,0,reference.get('entry_clearance_m', .06)])
    lift = goal.copy(); lift[2,3] += reference['lift_m']
    # Build the local branch upward from the measured grasp, avoiding a wrist flip at unloaded entry.
    branch_seed = np.asarray(reference['joints_deg'])
    for ratio in np.linspace(0, 1, 12):
        pose = goal.copy(); pose[:3,3] += ratio*(upper[:3,3]-goal[:3,3])
        solution = solve_ik(pose, branch_seed, model)
        if solution['status'] != 'SOLVED':
            return dict(status='WAIT_IK', reason='local entry branch unavailable', failed=solution)
        branch_seed = np.asarray(solution['angles_deg'])
    initial = seed.copy()
    count = max(1,int(np.ceil(np.max(np.abs(branch_seed-initial))/10)))
    for j in range(1,count+1):
        q = initial+(branch_seed-initial)*j/count
        record('entry' if j==count else 'entry-'+str(j),q,fk(q,model),'unloaded')
    for name, endpoint, phase in [('pregrasp',pre,'approach'),('grasp',goal,'approach'),('lift',lift,'lift')]:
        previous = fk(seed, model)
        count = max(1,int(np.ceil(np.linalg.norm(endpoint[:3,3]-previous[:3,3])/spacing)))
        initial = previous.copy()
        rotation_delta = rotation_from_matrix(initial[:3,:3].T @ endpoint[:3,:3]).as_rotvec()
        for j in range(1,count+1):
            pose = endpoint.copy()
            pose[:3,3] = initial[:3,3] + (endpoint[:3,3]-initial[:3,3])*j/count
            pose[:3,:3] = initial[:3,:3] @ rotation_matrix(Rotation.from_rotvec(rotation_delta*j/count))
            solution = solve_ik(pose, seed, model)
            if solution['status'] != 'SOLVED':
                return dict(status='WAIT_IK', reason='no usable local IK at '+name, failed=solution)
            q = np.asarray(solution['angles_deg'])
            if np.max(np.abs(q-seed)) > 30:
                return dict(status='WAIT_IK', reason='discontinuous adjacent IK samples at '+name)
            record(name if j==count else '{}-{}'.format(name,j),q,pose,phase,solution)
    return dict(status='PLANNED', reference_id=reference['reference_id'], initial_joints_deg=list(current_deg),
                initial_T_base_flange=start.tolist(), target=target, stages=stages,
                unverified=['physical collision clearance (links, fingers, Go2, support)',
                            'actual SDK interpolation', 'physical extrinsic/TCP accuracy'],
                scope='fixed orientation local reference; no second TCP correction')


def zero_stages(joints, model, prefix='zero'):
    start=np.asarray(joints,dtype=float)
    if start.shape!=(6,) or not np.all(np.isfinite(start)):raise ValueError('six finite measured joints required')
    count=max(1,int(np.ceil(np.max(np.abs(start))/10)))
    stages=[]
    for i in range(1,count+1):
        target=start*(1-i/count)
        # Validate commanded points without clipping a slightly over-limit feedback reading.
        stages.append(dict(stage_id=prefix+'-'+str(i),angles_deg=target.tolist(),
                           T_base_flange=fk(target,model).tolist(),phase='return_zero'))
    return stages


def load_config(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    if config.get('schema_version') != 1: raise ValueError('unsupported config schema')
    speed = config['motion']['speed_percent']
    if type(speed) is not int or not 0 <= speed <= 100:
        raise ValueError('joint speed_percent must be an integer in 0..100')
    for key in ('record_arm_outline','return_zero_after_grasp'):
        if key in config['motion'] and type(config['motion'][key]) is not bool:
            raise ValueError(key+' must be boolean')
    for key in ('camera_serial','usb_serial'):
        if not isinstance(config.get(key),str) or not config[key].isalnum(): raise ValueError('invalid '+key)
    semantic = config.get('mode') == 'semantic'
    backend = config.get('grasp_backend', 'legacy')
    if backend not in ('legacy', 'graspnet') or (backend == 'graspnet' and not semantic):
        raise ValueError('unsupported grasp backend/mode')
    if backend == 'graspnet':
        from urllib.parse import urlparse
        settings = config['graspnet']
        url = urlparse(settings['endpoint'])
        if url.scheme != 'http' or url.hostname not in ('127.0.0.1', 'localhost') or url.username or url.password:
            raise ValueError('GraspNet endpoint must be local HTTP')
        if not np.isfinite(settings['timeout_s']) or settings['timeout_s'] <= 0:
            raise ValueError('invalid GraspNet timeout')
    for key in (('extrinsics','kinematics') if semantic else ('reference','extrinsics','kinematics')):
        asset = (path.parent/config[key]).resolve()
        if path.parent not in asset.parents: raise ValueError('asset outside config bundle')
        config[key] = json.loads(asset.read_text())
    ref = None if semantic else validate_reference(config['reference'], path.parent)
    for key in ('extrinsics','kinematics'):
        asset = config[key]
        if hashlib.sha256((path.parent/asset['source_file']).read_bytes()).hexdigest() != asset['source_sha256']:
            raise ValueError('source mismatch: '+key)
    original_ext = json.loads((path.parent/config['extrinsics']['source_file']).read_text())
    if config['extrinsics']['T_base_color_optical'] != original_ext['T_base_color_optical']:
        raise ValueError('extrinsic source transform mismatch')
    if semantic:
        config['reference'] = dict(reference_id='semantic-rgbd-v1')
        transform(config['extrinsics']['T_base_color_optical'])
        load_joint_calibration(config, path.parent)
        limits(config['kinematics'])
        config['config_path'] = str(path)
        return config
    if ref['camera_serial'] != config['camera_serial'] or ref['usb_serial'] != config['usb_serial']:
        raise ValueError('reference hardware identity mismatch')
    if config['detector']['version'] != ref['detector_version']: raise ValueError('detector mismatch')
    if config['detector'] != ref['detector_parameters']: raise ValueError('detector parameters differ from reference')
    if ref['status'] != 'ready': raise ValueError('reference incomplete')
    transform(ref['T_base_flange']); transform(config['extrinsics']['T_base_color_optical'])
    if not np.allclose(fk(ref['joints_deg'],config['kinematics']), ref['T_base_flange'], atol=1e-8):
        raise ValueError('reference FK/model mismatch')
    if config['gripper']['effort_nm'] != .5:
        raise ValueError('configuration outside this experiment effort')
    load_joint_calibration(config, path.parent)
    ref['T_base_flange'] = fk(ref['joints_deg'],config['kinematics']).tolist()
    ref.update(local_limit_m=config['motion']['local_limit_m'], lift_m=config['motion']['lift_m'],
               sample_spacing_m=config['motion']['sample_spacing_m'],
               orientation_tolerance_deg=config['detector']['orientation_tolerance_deg'])
    config['config_path'] = str(path)
    return config


def absolute_target(observation, config):
    point = np.asarray(observation['point_color_m'], dtype=float)
    ext = transform(config['extrinsics']['T_base_color_optical'])
    y = np.asarray(observation['grasp_axis_base'], dtype=float)
    y /= np.linalg.norm(y)
    horizontal = np.cross(y, np.array([0., 0., 1.]))
    if horizontal[0] < 0: horizontal = -horizontal
    pitch = np.radians(config['semantic'].get('approach_pitch_deg', 90.))
    z = horizontal * np.cos(pitch) + np.array([0., 0., -1.]) * np.sin(pitch)
    x = np.cross(y, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    tcp = np.eye(4); tcp[:3,:3] = np.column_stack([x,y,z])
    yaw = float(config['semantic'].get('approach_yaw_offset_deg',0.))
    tcp[:3,:3] = rotation_matrix(Rotation.from_euler('z',yaw,degrees=True)) @ tcp[:3,:3]
    width = observation['grasp_width_m']
    if not np.isclose(np.sin(np.radians(yaw)),0,atol=1e-12):
        hull = np.asarray(observation['visible_hull_base_xy_m'],dtype=float)
        if hull.ndim!=2 or hull.shape[1]!=2 or not len(hull) or not np.all(np.isfinite(hull)):
            raise ValueError('invalid visible target hull')
        width = float(np.ptp(hull @ tcp[:2,1]))
    tcp[:3,3] = ext[:3,:3] @ point + ext[:3,3] + np.asarray(config['semantic']['grasp_offset_base_m'])
    flange = tcp.copy()
    flange[:3,3] -= tcp[:3,:3] @ np.asarray(config['semantic']['tcp_offset_m'])
    transform(flange)
    return dict(T_base_tcp=tcp.tolist(), T_base_flange=flange.tolist(),
                grasp_width_m=width, reference_id='semantic-rgbd-v1',
                tcp_offset_m=config['semantic']['tcp_offset_m'],
                approach_pitch_deg=float(np.degrees(pitch)),
                approach_yaw_offset_deg=yaw,
                tcp_status='configured tool offset; physical calibration unverified')


def plan_absolute(current_deg, config, target):
    model = dict(config['kinematics'],ik_position_tolerance_m=config['semantic'].get('ik_position_tolerance_m',.001))
    goal = transform(target['T_base_flange'])
    semantic = config['semantic']
    pregrasp = np.asarray(semantic.get('pregrasp_offset_base_m',
                         -goal[:3,2]*semantic['pregrasp_distance_m']), dtype=float)
    clearance = float(semantic.get('entry_clearance_m', 0.))
    if pregrasp.shape != (3,) or not np.all(np.isfinite(pregrasp)):
        raise ValueError('pregrasp offset must be three finite base-frame metres')
    if not np.isfinite(clearance) or clearance < 0:
        raise ValueError('entry clearance must be finite nonnegative metres')
    # Reuse the planner with a branch solved for this target, never the old block's endpoint.
    solution = solve_ik(goal, current_deg, model)
    if solution['status'] != 'SOLVED':
        solution = solve_ik(goal, np.degrees(limits(model).mean(axis=1)), model)
    if solution['status'] != 'SOLVED':
        return dict(status='WAIT_IK', reason='absolute grasp IK unavailable', failed=solution, target=target)
    reference = dict(reference_id='semantic-rgbd-v1', transition_joints_deg=[], entry_clearance_m=clearance,
                     joints_deg=solution['angles_deg'], sample_spacing_m=config['motion']['sample_spacing_m'],
                     pregrasp_offset_base_m=pregrasp.tolist(),
                     lift_m=config['motion']['lift_m'])
    if 'entry_offset_base_m' in semantic:
        offset=np.asarray(semantic['entry_offset_base_m'],dtype=float)
        if offset.shape!=(3,) or not np.all(np.isfinite(offset)):
            raise ValueError('invalid entry offset')
        reference['entry_offset_base_m']=offset.tolist()
    plan = plan_motion(current_deg, reference, target, model)
    plan['target'] = target
    plan['scope'] = 'description/RGB-D absolute target; configured TCP parameter'
    plan['close_mm'] = max(0., target['grasp_width_m']*1000-config['semantic']['close_margin_mm'])
    plan['measured_grasp_width_mm'] = target['grasp_width_m']*1000
    return plan


def search_orientations(current_deg,config,observation,on_candidate=None):
    """Try a finite ordered orientation grid; only accept a complete existing path plan."""
    semantic=config['semantic'];grid=semantic['angle_search']
    for key,low,high in [('pitch_deg',0,90),('yaw_offset_deg',-180,180)]:
        values=np.asarray(grid[key],dtype=float)
        if values.ndim!=1 or not len(values) or not np.all(np.isfinite(values)) or np.any(values<low) or np.any(values>high):
            raise ValueError('invalid angle search '+key)
    distance=float(semantic['pregrasp_distance_m']);clearance=float(semantic.get('entry_clearance_m',0.))
    if not np.isfinite(distance) or distance<=0 or not np.isfinite(clearance) or clearance<0:
        raise ValueError('invalid angle search approach distance')
    candidates=[]
    for yaw in grid['yaw_offset_deg']:
        for pitch in grid['pitch_deg']:
            settings=dict(semantic,approach_pitch_deg=float(pitch),approach_yaw_offset_deg=float(yaw))
            active=dict(config,semantic=settings)
            target=absolute_target(observation,active)
            approach=np.asarray(target['T_base_tcp'])[:3,2]
            for key,length in [('pregrasp_offset_base_m',distance),('entry_offset_base_m',clearance)]:
                settings[key]=(-approach*length).tolist();target[key]=settings[key]
            width=target['grasp_width_m']*1000
            opening=min(config['gripper']['open_mm'],config['gripper']['open_ready_mm'],100.)
            if not np.isfinite(width) or not 0<width<=opening:
                plan=dict(status='WAIT_WIDTH',reason='visible width {:.3f} mm outside existing opening capacity (0, {:.3f}] mm'.format(width,opening))
            else:
                plan=plan_absolute(current_deg,active,target)
            row=dict(index=len(candidates),pitch_deg=float(pitch),yaw_offset_deg=float(yaw),
                     status=plan['status'],reason=plan.get('reason','complete path available'),
                     failed=plan.get('failed'),target=target)
            candidates.append(row)
            if on_candidate:on_candidate(row)
            if plan['status']=='PLANNED':
                plan.update(orientation_candidates=candidates,selected_candidate_index=row['index'],
                            scope='enumerated pitch/yaw; first complete IK path; physical clearance unverified')
                return plan
    return dict(status='WAIT_IK',reason='all {} orientation candidates rejected; waiting for a new observation'.format(len(candidates)),
                target=candidates[-1]['target'],failed=candidates[-1]['failed'],
                orientation_candidates=candidates,selected_candidate_index=None)
