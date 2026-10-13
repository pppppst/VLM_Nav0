"""Serial local exploration using the existing Go2 adapter and resource guard."""
from dataclasses import asdict, replace
import math
import time
import uuid
from .config import valid_numbers, go2_local_gates
from .models import Pose2D
from .region_guard import RegionGuard
from vlm_nav.arrival import target_distance, stop_radius, arrival_decision, load_contract
from .semantic_search import FixedViewpoints, SemanticFurniture, SelectionState, load_catalog
from .generated_candidates import GeneratedCandidates, GeneratedCandidatesVLM, render_selection


class ExplorationStopped(RuntimeError):
    pass


def exploration_gates(config):
    errors=go2_local_gates(config,motion=True)
    robot=config['robot_go2']
    if robot.get('approach_on_detection'):
        distance = robot.get('standoff_distance')
        if not valid_numbers([distance], 1) or distance <= 0:
            errors.append('positive finite target standoff_distance required')
        try:load_contract(robot.get('arrival_config_files'))
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('arrival contract invalid: ' + str(error))
    if not robot.get('path_topic'):
        errors.append('executing/replanned path topic required')
    if not robot.get('goal_costmap_topic'):
        errors.append('global goal costmap topic required')
    angles=robot.get('scan_angles')
    if not isinstance(angles,list) or not angles or not valid_numbers(angles,len(angles)):
        errors.append('finite scan angles required')
    if robot.get('exploration_strategy', 'fixed_viewpoints') == 'semantic_furniture':
        try:load_catalog(config)
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('semantic catalog invalid: ' + str(error))
    elif robot.get('exploration_strategy') == 'generated_candidates_vlm':
        try:GeneratedCandidates(config)
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('generated candidates invalid: ' + str(error))
        targets=config['scene'].get('targets',{})
        if any(not isinstance(targets.get(t),str) or not targets[t].strip() or '现场填写' in targets[t] for t in 'ABC'):
            errors.append('generated scans require measured A/B/C target descriptions')
    elif robot.get('exploration_strategy', 'fixed_viewpoints') != 'fixed_viewpoints':
        errors.append('unknown exploration strategy')
    return errors


def measured_viewpoints(config):
    points = config['scene'].get('viewpoints', {}).get('go2', [])
    if not points or any(not isinstance(p, dict) or not isinstance(p.get('id'), str)
                         or not p['id'].strip() or not valid_numbers(p.get('pose'), 3) for p in points):
        raise ValueError('measured Go2 viewpoints required')
    if len({p['id'] for p in points}) != len(points):
        raise ValueError('duplicate Go2 viewpoint IDs')
    return points


def explore(adapter, spin, config, frame, deadline, target, record, owners,
            stop, before_motion, clock=time.monotonic):
    guard = RegionGuard(config['scene']['resources'], config['robot_go2']['stop_margin'])
    task_id = 'go2_explore/' + uuid.uuid4().hex
    current = {}
    strategy = config['robot_go2'].get('exploration_strategy', 'fixed_viewpoints')
    generated = strategy == 'generated_candidates_vlm'
    semantic = strategy in ('semantic_furniture', 'generated_candidates_vlm')
    geometry = GeneratedCandidates(config) if generated else None
    catalog = load_catalog(config) if semantic and not generated else None
    points = geometry.points if generated else catalog['points'] if semantic else measured_viewpoints(config)
    policy = GeneratedCandidatesVLM(adapter.client) if generated else SemanticFurniture(adapter.client) if semantic else FixedViewpoints(points)
    state = SelectionState(points)
    if generated:state.task_id = task_id
    selection_epoch = None
    history = []
    decision_id = None
    if generated:
        record(dict(state='CANDIDATE_GEOMETRY',task_id=state.task_id,candidates=points,
                    rotation_radius=geometry.radius,semantic_yaml=str(geometry.semantic_path)))

    def catalog_unchanged():
        return geometry.unchanged() if generated else load_catalog(config) == catalog

    def emit(state, **fields):
        record(dict(current, state=state, **fields))

    def update(status, **details):
        state.update(current['viewpoint_id'], status, clock(), **details)
        if generated and status in ('navigation_failed','observation_failed') and 'display_id' in current:
            history.append(dict(decision_id=decision_id,candidate_id=current['display_id'],
                furniture_id=current['furniture_id'],result=status))
        emit('OBSERVATION_RECORD', observation_status=status, records=state.records.copy())

    def health_check(path=None, navigating=False, began=0., acquired=False):
        health = adapter.get_health()
        if clock() >= deadline:
            raise ExplorationStopped('exploration deadline reached')
        if owners() or not health.nav_ready or not health.localization_valid or adapter.costmap_transform() is None:
            raise ExplorationStopped('motion owner/localization/Nav2/costmap health lost')
        if navigating:
            acquired = acquired or bool(health.path_valid and health.path)
            waiting = not acquired and health.stopped and clock()-began < config['robot_go2'].get('path_start_timeout', 1.)
            checked = replace(health, path_valid=True) if waiting else health
            checked_path = [health.pose] if waiting else health.path
        else:
            checked = replace(health, path_valid=True)
            checked_path = path if path is not None else [health.pose]
        decision = guard.check(health.pose, health.footprint, checked_path, {}, checked)
        if decision.action != 'ALLOW':
            raise ExplorationStopped(decision.reason or 'protected resource requires coordinator permit')
        return health, acquired

    def stopped():
        if not stop():
            raise ExplorationStopped('native action terminal and physical stop unconfirmed')
        health, _ = health_check()
        if not health.stopped or not health.motion_terminal:
            raise ExplorationStopped('stop prerequisites changed')

    def parking_free(goal, observation=True):
        free=adapter.costmap_candidate_free(goal,global_goal=True)
        if generated and observation:free=free and adapter.observation_candidate_free(goal)
        if adapter.costmap_transform(global_goal=True) is None:
            raise ExplorationStopped('global goal costmap missing, stale or TF unavailable')
        return free

    def navigate(goal, command_id, kind, target=None, radius=None):
        before_motion()
        goal_id = adapter.start_navigation(goal, task_id, command_id, kind)
        emit('NAVIGATING', task_id=task_id, command_id=command_id, goal_id=goal_id, goal_kind=kind)
        began = clock(); acquired = False; terminal = None
        while terminal is None:
            health, acquired = health_check(navigating=True, began=began, acquired=acquired)
            if target is not None and target_distance((health.pose.x,health.pose.y), (target.x,target.y)) <= radius:
                emit('APPROACH_DISTANCE_STOP', cancel_radius=radius)
                stopped()
                return 'DISTANCE_STOPPED'
            spin()
            while adapter.results:
                reply = adapter.results.popleft()
                if (reply.task_id, reply.command_id, reply.goal_id, reply.goal_kind) == (task_id, command_id, goal_id, kind):
                    terminal = reply.status
        stopped()
        return terminal

    def approach_attempt(observation, distance, radius):
        epoch = getattr(adapter, 'selection_epoch', 0)
        object_pose = frame.to_local(observation.common_position)
        emit('APPROACH_PLANNING', object_pose=asdict(object_pose), observation_id=observation.observation_id)
        future = adapter.select_approach(object_pose, [], standoff_distance=distance)
        expires = min(deadline, clock()+15.)
        while not future.done():
            health_check()
            if clock() >= expires:
                future.cancel()
                raise ExplorationStopped('target approach planning timeout')
            spin()
        if clock() >= expires:
            raise ExplorationStopped('late target approach plan')
        goal = future.result().goal
        if goal is None:
            emit('APPROACH_UNREACHABLE', reason='no safe standoff candidate')
            return 'APPROACH_UNREACHABLE'
        future = adapter.compute_path(goal)
        while not future.done():
            health_check()
            if clock() >= expires:
                future.cancel()
                raise ExplorationStopped('target approach path timeout')
            spin()
        result = future.result()
        health, _ = health_check()
        if (clock() >= expires or epoch != getattr(adapter, 'selection_epoch', 0)
                or not health.stopped or not health.motion_terminal):
            raise ExplorationStopped('target approach cancelled or prerequisites changed')
        if (not result.reachable or not result.path or not parking_free(goal, observation=False)
                or not adapter.costmap_candidate_free(goal)
                or guard.check(health.pose, health.footprint, result.path, {},
                               replace(health, path_valid=True)).action != 'ALLOW'):
            emit('APPROACH_UNREACHABLE', reason='standoff footprint/path rejected')
            return 'APPROACH_UNREACHABLE'
        current['pose'] = [goal.x, goal.y, goal.yaw]
        emit('APPROACH_SELECTED', standoff_distance=distance)
        return navigate(goal, uuid.uuid4().hex, 'PICKUP_NAV', object_pose, radius)

    def approach_target(observation):
        contract = load_contract(config['robot_go2'].get('arrival_config_files'))
        success = contract['target_success_radius']
        retries = contract['arrival_retry_limit']
        object_pose = frame.to_local(observation.common_position)
        distance = min(config['robot_go2']['standoff_distance'], success)
        emit('ARRIVAL_CONTRACT', **contract)
        for retry in range(retries+1):
            health, _ = health_check()
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('arrival verification requires confirmed stop')
            measured = target_distance((health.pose.x,health.pose.y), (object_pose.x,object_pose.y))
            if measured <= success:
                emit('TARGET_APPROACHED', measured_distance=measured, success_radius=success,
                     arrival_retry=retry, stop_confirmed=True)
                return 'TARGET_APPROACHED'
            radius = stop_radius(success, contract['approach_cancel_radius'], retry)
            terminal = approach_attempt(observation, distance, radius)
            if terminal not in ('SUCCEEDED', 'DISTANCE_STOPPED'):
                emit('APPROACH_FAILED', navigation_status=terminal)
                return 'APPROACH_FAILED'
            health, _ = health_check()
            measured = target_distance((health.pose.x,health.pose.y), (object_pose.x,object_pose.y))
            decision = arrival_decision(measured, success, retry, retries)
            emit('ARRIVAL_VERIFIED', measured_distance=measured, success_radius=success,
                 arrival_retry=retry, decision=decision, stop_confirmed=True)
            if decision == 'SUCCESS':
                emit('TARGET_APPROACHED', measured_distance=measured, success_radius=success, stop_confirmed=True)
                return 'TARGET_APPROACHED'
            if decision == 'FAILED':
                break
            # Compensate measured terminal overshoot; every new pose still needs both costmaps and a path.
            distance -= measured-success
            if not math.isfinite(distance) or distance <= 0:
                break
            emit('APPROACH_RETRY', arrival_retry=retry+1, standoff_distance=distance)
        emit('APPROACH_DISTANCE_FAILED', measured_distance=measured, success_radius=success, stop_confirmed=True)
        return 'APPROACH_DISTANCE_FAILED'

    def next_point():
        nonlocal selection_epoch, decision_id
        if not semantic:
            return policy.next_viewpoint(None)
        epoch = getattr(adapter, 'selection_epoch', 0)
        if not catalog_unchanged():raise ExplorationStopped('selection map assets changed')
        candidates = []
        for candidate in points:
            if candidate['id'] not in state.eligible(clock()):
                continue
            health, _ = health_check()
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('selection requires confirmed stop')
            goal = Pose2D(*candidate['pose'])
            if not parking_free(goal):
                state.update(candidate['id'], 'navigation_failed', clock(), reason='footprint blocked')
                continue
            future = adapter.compute_path(goal)
            plan_deadline = min(deadline, clock() + 15.)
            while not future.done():
                health_check()
                if clock() >= plan_deadline:
                    future.cancel()
                    raise ExplorationStopped('candidate planning timeout')
                spin()
            result = future.result()
            health, _ = health_check()
            if (not result.reachable or not result.path or generated and not parking_free(goal) or
                    guard.check(health.pose, health.footprint, result.path, {},
                                replace(health, path_valid=True)).action != 'ALLOW'):
                state.update(candidate['id'], 'navigation_failed', clock(), reason='no permitted path')
                continue
            length = sum(math.hypot(a.x-b.x, a.y-b.y) for a,b in zip(result.path, result.path[1:]))
            candidates.append(dict(candidate, path_length=length))
            if generated and len(candidates) == 12:break
        if generated:
            for candidate in candidates[:]:
                if not parking_free(Pose2D(*candidate['pose'])):
                    state.update(candidate['id'],'navigation_failed',clock(),reason='scan space changed during planning')
                    candidates.remove(candidate)
        if not candidates:
            return None
        if epoch != getattr(adapter, 'selection_epoch', 0):
            raise ExplorationStopped('selection cancelled during candidate planning')
        token = state.snapshot([p['id'] for p in candidates])
        selection_epoch = epoch
        decision_id = uuid.uuid4().hex
        snapshot = dict(task_id=state.task_id, decision_id=decision_id, revision=state.revision, main_target=target,
                        search_hint=config['robot_go2'].get('search_hint'), candidates=candidates,
                        records={k: dict(v) for k,v in state.records.items()})
        if getattr(adapter,'selection_evidence_dir',None) is not None:
            snapshot['evidence_directory']=str(adapter.selection_evidence_dir / decision_id)
        if generated:
            for index,candidate in enumerate(candidates,1):candidate['display_id']=f'P{index}'
            health, _ = health_check()
            snapshot.update(history=history[-12:], images=render_selection(geometry,candidates,state.records,
                health.pose,adapter.selection_evidence_dir / decision_id))
        else:
            snapshot.update(targets=config['scene'].get('targets', {}),current_room=catalog['current_room'],
                            furniture=list(catalog['furniture'].values()),image=catalog['image'])
        request_type = 'candidate_selection' if generated else 'furniture_selection'
        record(dict(state='CANDIDATE_REQUEST' if generated else 'FURNITURE_REQUEST', request_type=request_type, **snapshot))
        future = adapter.vision.submit(lambda: policy.next_viewpoint(snapshot))
        expires = min(deadline, clock() + config['robot_go2'].get('api_timeout', 20.))
        try:
            while not future.done():
                health_check()
                if clock() >= expires:
                    raise ExplorationStopped('furniture selection timeout')
                spin()
            if clock() >= expires:
                raise ExplorationStopped('late furniture selection discarded')
            point = future.result()
            health, _ = health_check()
            if (epoch != getattr(adapter, 'selection_epoch', 0) or
                    not state.accepts(token, point['id'], clock()) or
                    not health.stopped or not health.motion_terminal or not catalog_unchanged()):
                raise ExplorationStopped('stale furniture selection discarded')
        except BaseException:
            future.cancel()
            state.cancel()
            if getattr(adapter.client,'selection_evidence_write_failed',False):
                adapter.evidence_write_failed=True
                adapter.motion_fault=True
            record(dict(state='CANDIDATE_SELECTION_FAILED' if generated else 'FURNITURE_SELECTION_FAILED', request_type=request_type,
                        task_id=state.task_id, decision_id=decision_id, motion_sent=False))
            raise
        record(dict(state='CANDIDATE_SELECTED' if generated else 'FURNITURE_SELECTED', request_type=request_type,
                    furniture_id=point['furniture_id'], viewpoint_id=point['id'],
                    candidate_id=point.get('display_id'), decision_id=decision_id,
                    reason=point['selection_reason'], task_id=state.task_id))
        return point

    while True:
        point = next_point()
        if point is None:
            break
        goal = Pose2D(*point['pose']) if semantic else frame.to_local(Pose2D(*point['pose']))
        current = {'viewpoint_id': point['id'], 'pose': [goal.x, goal.y, goal.yaw], 'plan_reachable': None,
            'goal_costmap_topic': config['robot_go2'].get('goal_costmap_topic')}
        if semantic:current['furniture_id'] = point['furniture_id']
        if generated:current['display_id'] = point['display_id']
        emit('SELECTED')
        health, _ = health_check()
        if not health.stopped or not health.motion_terminal:
            raise ExplorationStopped('previous motion has not stopped')
        if not parking_free(goal):
            emit('FOOTPRINT_BLOCKED',reason='global costmap: footprint outside coverage or intersects obstacle/unknown')
            if semantic:update('navigation_failed')
            continue
        future = adapter.compute_path(goal)
        plan_deadline = min(deadline, clock()+15.)
        while not future.done():
            health_check()
            if clock() >= plan_deadline:
                future.cancel()
                raise ExplorationStopped('path planning timeout; reachability unknown')
            spin()
        result = future.result()
        current['plan_reachable'] = result.reachable
        emit('PLANNED' if result.reachable else 'PLAN_FAILED', reason=result.reason)
        if not result.reachable:
            if semantic:update('navigation_failed')
            continue
        if not result.path:
            raise ExplorationStopped('planner returned empty path')
        health, _ = health_check()
        decision = guard.check(health.pose, health.footprint, result.path, {}, replace(health, path_valid=True))
        if decision.action != 'ALLOW':
            emit('RESOURCE_BLOCKED', reason=decision.reason or 'no protected resource permit')
            if semantic:update('navigation_failed')
            continue
        if not health.stopped or not health.motion_terminal:
            raise ExplorationStopped('motion prerequisites changed during planning')
        if not parking_free(goal):
            emit('FOOTPRINT_BLOCKED',reason='global costmap: footprint outside coverage or intersects obstacle/unknown')
            if semantic:update('navigation_failed')
            continue
        command_id = uuid.uuid4().hex
        if semantic and (selection_epoch != getattr(adapter, 'selection_epoch', 0)
                         or not catalog_unchanged()
                         or point['id'] not in state.eligible(clock())):
            raise ExplorationStopped('selection invalidated before navigation')
        if semantic:update('executing')
        terminal = navigate(goal, command_id, 'VIEWPOINT_NAV')
        emit('ARRIVED' if terminal == 'SUCCEEDED' else 'NAV_FAILED', navigation_status=terminal, stop_confirmed=True)
        if terminal != 'SUCCEEDED':
            if semantic:update('navigation_failed')
            continue
        health, _ = health_check()
        if generated and not parking_free(health.pose):
            emit('SCAN_SPACE_BLOCKED',actual_stopped_pose=asdict(health.pose),stop_confirmed=True)
            update('observation_failed',reason='actual stopped pose has no full rotation space')
            continue
        before_motion()
        emit('SCANNING')
        adapter.scan_viewpoint_id=point['id']
        descriptions=dict(A=target,B=config['scene']['targets']['B'],C=config['scene']['targets']['C']) if generated else {'A':target}
        future = adapter.start_scan(descriptions, command_id)
        while not future.done():
            health_check()
            spin()
        if future.cancelled():
            raise ExplorationStopped('observation cancelled')
        try:
            scan_results = future.result()
            observations = [o for o in scan_results if o.target_id == 'A']
        except Exception:
            if not semantic:raise
            stopped()
            update('observation_failed', reason='scan failed')
            continue
        stopped()
        incomplete_abc = generated and (len(scan_results)!=len(descriptions)*len(config['robot_go2']['scan_angles'])
                or {(o.target_id,o.view_index) for o in scan_results} !=
                   {(t,i) for t in descriptions for i in range(len(config['robot_go2']['scan_angles']))}
                or any(o.status not in ('DETECTED','NOT_DETECTED','UNCERTAIN') for o in scan_results))
        if incomplete_abc:
            update('observation_failed', reason='incomplete ABC scan or image/API/validation error')
            if not any(o.status=='DETECTED' for o in observations):continue
            # A valid detection survives errors in other views/targets; the point stays unfinished.
            observations=[o for o in observations if o.status=='DETECTED']
        emit('SCAN_RESULT', observations=[asdict(o) for o in observations], stop_confirmed=True)
        if not observations or any(o.status not in ('DETECTED','NOT_DETECTED','UNCERTAIN') for o in observations):
            if semantic:
                update('observation_failed', reason='image/API/validation error')
                continue
            raise ExplorationStopped('target scan missing or API/validation error')
        if not incomplete_abc and {o.view_index for o in observations} != set(range(len(config['robot_go2']['scan_angles']))):
            if semantic:
                update('observation_failed', reason='incomplete scan directions')
                continue
            raise ExplorationStopped('target scan directions incomplete')
        detections = [o for o in observations if o.status == 'DETECTED']
        if semantic and not incomplete_abc:
            update('observed', results=[asdict(o) for o in (scan_results if generated else observations)])
        if generated:
            history.append(dict(decision_id=decision_id,candidate_id=point['display_id'],
                furniture_id=point['furniture_id'],observation_complete=not incomplete_abc,result='DETECTED' if detections else
                    'NOT_DETECTED' if all(o.status=='NOT_DETECTED' for o in observations) else 'UNCERTAIN'))
        if detections:
            outcome = 'TARGET_FOUND' if any(o.navigable for o in detections) else 'TARGET_DETECTED_UNLOCATED'
            emit(outcome)
            if outcome == 'TARGET_FOUND' and config['robot_go2'].get('approach_on_detection', False):
                return approach_target(max((o for o in detections if o.navigable), key=lambda o:o.confidence))
            return outcome
        emit('NOT_DETECTED' if all(o.status == 'NOT_DETECTED' for o in observations) else 'UNCERTAIN')
    if semantic and (any(row['status'] != 'observed' for row in state.records.values()) or generated and not points):
        emit('WAITING_VIEWPOINTS', records=state.records, reason='候选暂不可达或观察失败；不代表目标不存在')
        return 'WAITING_VIEWPOINTS'
    emit('EXHAUSTED', reason='当前视角已检查完；目标是否存在未知')
    return 'EXHAUSTED'


def exploration_markers(event):
    from visualization_msgs.msg import Marker, MarkerArray
    output = MarkerArray()
    if 'pose' not in event:
        return output
    x, y, yaw = event['pose']
    color = (.2, .2, .2) if event['state'] in ('TARGET_FOUND','TARGET_DETECTED_UNLOCATED','ARRIVED') else (.45, .45, .45)
    if event['state'] in ('PLAN_FAILED','FOOTPRINT_BLOCKED','RESOURCE_BLOCKED','NAV_FAILED','STOPPED','STOP_UNCONFIRMED','REJECTED'):
        color = (0., 0., 0.)
    for index, kind in enumerate((Marker.ARROW, Marker.TEXT_VIEW_FACING)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'go2_current_exploration_point'
        marker.id = index; marker.type = kind; marker.action = Marker.ADD
        marker.pose.position.x = x; marker.pose.position.y = y
        marker.pose.position.z = .1 if index == 0 else .6
        marker.pose.orientation.z = math.sin(yaw/2) if index == 0 else 0.
        marker.pose.orientation.w = math.cos(yaw/2) if index == 0 else 1.
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.
        if index == 0:
            marker.scale.x = .6; marker.scale.y = .12; marker.scale.z = .12
        else:
            marker.scale.z = .2
            marker.text = f"{event['viewpoint_id']} | {event['state']} | reachable={event.get('plan_reachable')}"
        output.markers.append(marker)
    return output


def generated_exploration_markers(display):
    from visualization_msgs.msg import Marker, MarkerArray
    candidates,selected=MarkerArray(),MarkerArray()
    for output in (candidates,selected):
        clear=Marker();clear.header.frame_id='map';clear.action=Marker.DELETEALL;output.markers.append(clear)
    records=display.get('records',{});selectable=display.get('selectable',{})
    for index,point in enumerate(display.get('points',[])):
        pid=point['id'];x,y,yaw=point['pose']
        status=records.get(pid,{}).get('status','unchecked')
        available=pid in selectable and status not in ('observed','executing')
        color=(.95,.5,.08) if available else (.4,.4,.4)
        for offset,kind in enumerate((Marker.SPHERE,Marker.TEXT_VIEW_FACING)):
            marker=Marker();marker.header.frame_id='map';marker.ns='generated_candidates'
            marker.id=index*2+offset;marker.type=kind;marker.action=Marker.ADD
            marker.pose.position.x=x;marker.pose.position.y=y;marker.pose.position.z=.1 if offset==0 else .35
            marker.pose.orientation.w=1.;marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=1.
            marker.scale.x=marker.scale.y=marker.scale.z=.12 if offset==0 else .16
            marker.text=f'{selectable.get(pid,pid)} | {point["furniture_id"]} | {status}'
            candidates.markers.append(marker)
        if display.get('selected')==pid:
            for offset,kind in enumerate((Marker.ARROW,Marker.TEXT_VIEW_FACING)):
                marker=Marker();marker.header.frame_id='map';marker.ns='vlm_selected_candidate'
                marker.id=offset;marker.type=kind;marker.action=Marker.ADD
                marker.pose.position.x=x;marker.pose.position.y=y;marker.pose.position.z=.2 if offset==0 else .6
                marker.pose.orientation.z=math.sin(yaw/2) if offset==0 else 0.
                marker.pose.orientation.w=math.cos(yaw/2) if offset==0 else 1.
                marker.color.g=.85;marker.color.b=.25;marker.color.a=1.
                marker.scale.x=.6 if offset==0 else 0.;marker.scale.y=.12 if offset==0 else 0.;marker.scale.z=.12 if offset==0 else .2
                marker.text=f'VLM selected: {selectable.get(pid,pid)} | {point["furniture_id"]}'
                selected.markers.append(marker)
    return candidates,selected
