"""Independent Go2 adapter acceptance. No task topics, peer, bridge, startup or ARM."""
import argparse
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import time
import uuid
from contextlib import contextmanager
from .config import load_config,live_gates,go2_local_gates,map_file_info
from .models import Pose2D,motion_path_error

MOTION=('navigate','scan','explore')
MOTION_LOCK='/tmp/go2-coop-acceptance-motion.lock'

class StopUnconfirmed(RuntimeError):
    pass

@contextmanager
def motion_slot():
    import fcntl
    fd=os.open(MOTION_LOCK,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('another Go2 acceptance owns motion')
        if os.read(fd,64):raise RuntimeError('previous acceptance needs field stop reconciliation')
        os.write(fd,b'IN_PROGRESS')  # Survives abrupt process exit; no automatic continuation.
        try:yield
        except StopUnconfirmed:
            os.lseek(fd,0,os.SEEK_SET);os.ftruncate(fd,0);os.write(fd,b'STOP_UNCONFIRMED')
            raise
        except BaseException:
            # Preserve IN_PROGRESS when exceptional cleanup cannot prove stop.
            raise
        else:os.ftruncate(fd,0)
    finally:os.close(fd)

def other_owners(node):
    names=node.get_node_names()
    owners=set(names) & {'vlm_nav','multi_agent_coordinator','multi_agent_go2_agent',
        'multi_agent_task_agent','multi_agent_go2_rgbd'}
    if names.count('multi_agent_go2_check_motion')>1:owners.add('multi_agent_go2_check_motion')
    return sorted(owners)

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-dir')
    p.add_argument('--action',choices=('inspect','plan','approach',*MOTION),default='inspect')
    p.add_argument('--offline',action='store_true',help='configuration report only; never initialize ROS')
    p.add_argument('--map-yaml',help='print file digests for a candidate map; does not write configuration')
    p.add_argument('--require-ready',action='store_true',help='exit 1 if reported prerequisites are incomplete')
    p.add_argument('--allow-motion',action='store_true',help='required in addition to an explicit motion action')
    p.add_argument('--output-dir',help='new motion evidence directory; generated strategy defaults to the managed experiment root')
    p.add_argument('--target',help='single exploration target description; defaults to scene.targets.A')
    p.add_argument('--search-hint', help='location clue for furniture selection only; not visual detection evidence')
    p.add_argument('--approach-target', action='store_true', help='after grounded detection, use the shared Go2 local approach executor')
    p.add_argument('--strategy', choices=('fixed_viewpoints', 'semantic_furniture', 'generated_candidates_vlm'),
                   help='Go2 internal exploration strategy; default fixed_viewpoints')
    p.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'),help='measured local Go2 map pose; inspect only probes costmap footprint; approach treats X/Y as object position')
    p.add_argument('--timeout',type=float,default=15.,help='inspection/action deadline in seconds')
    p.add_argument('--cancel-after',type=float,help='explicit cancellation acceptance during navigate/scan')
    return p

def terminal_report(value):
    """Keep failure diagnostics on screen; coordinate arrays remain in result.json."""
    if isinstance(value,dict):
        summary={}
        for key,item in value.items():
            if key in ('footprint','path') and isinstance(item,list):
                summary[key+'_count']=len(item)
            else:summary[key]=terminal_report(item)
        return summary
    if isinstance(value,list):return [terminal_report(item) for item in value]
    return value

def emit(value):
    print(json.dumps(terminal_report(value),ensure_ascii=False,allow_nan=False,indent=2))

def config_report(config,map_yaml=None):
    from .go2_exploration import exploration_gates
    path=map_yaml or config['robot_go2'].get('map_path')
    try:info=map_file_info(path) if path else None
    except (OSError,KeyError,TypeError,ValueError):info={'error':'map YAML/image unreadable'}
    semantic_report = None
    if config['robot_go2'].get('exploration_strategy') == 'semantic_furniture':
        from .semantic_search import load_catalog
        try:semantic_report = load_catalog(config)
        except (OSError, KeyError, TypeError, ValueError) as error:
            semantic_report = {'error': str(error)}
    elif config['robot_go2'].get('exploration_strategy') == 'generated_candidates_vlm':
        from .generated_candidates import GeneratedCandidates
        try:
            geometry=GeneratedCandidates(config)
            semantic_report=dict(points=geometry.points, semantic_yaml=str(geometry.semantic_path) if geometry.semantic_path else None,
                                 rotation_radius=geometry.radius, max_candidates_per_request=12)
        except (OSError, KeyError, TypeError, ValueError) as error:
            semantic_report={'error':str(error)}
    return {'mode':'configuration_only','map_files':info, 'semantic_search': semantic_report,
        'exploration_strategy': config['robot_go2'].get('exploration_strategy', 'fixed_viewpoints'),
        'go2_local_gates':go2_local_gates(config),'live_gates':live_gates(config),
        'single_robot_experiment_gates':exploration_gates(config),
        'path_monitoring':{'topic':config['robot_go2'].get('path_topic'),
            'receive_timeout_s':config['robot_go2'].get('path_receive_timeout',5.),
            'plan_age_checked':False,'source_field_verified':config['robot_go2'].get('path_source_verified',False)},
        'motion_sent':False}

def stop_owned(adapter,spin,timeout,record=None):
    adapter.stop_motion('independent acceptance completion/cancel')
    deadline=time.monotonic()+timeout
    previous=None
    while time.monotonic()<deadline:
        spin()
        health=adapter.get_health()
        observed=(health.stopped,health.motion_terminal)
        if record and observed!=previous:
            try:
                record(dict(event='STOP_OBSERVATION',stopped=health.stopped,
                    motion_terminal=health.motion_terminal,pose=asdict(health.pose)))
            except OSError:
                adapter.evidence_write_failed=True
                adapter.motion_fault=True
            previous=observed
        if health.stopped and health.motion_terminal:return True
    return False

def require_ros_environment():
    if os.getenv('ROS_DOMAIN_ID','0')!='0':raise ValueError('Go2 acceptance requires local Domain0')
    if os.getenv('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp' or os.getenv('GO2_NET_IFACE')!='enp4s0':
        raise ValueError('use runbook Domain0/CycloneDDS/enp4s0 environment')
    from urllib.parse import urlparse,unquote
    from ament_index_python.packages import get_package_share_directory
    uri=urlparse(os.getenv('CYCLONEDDS_URI',''))
    expected_xml=Path(get_package_share_directory('vlm_nav'))/'config/cyclonedds_go2.xml'
    if uri.scheme!='file' or uri.netloc or not uri.path or Path(unquote(uri.path)).read_bytes()!=expected_xml.read_bytes():
        raise ValueError('use the documented Go2-only CycloneDDS XML')

def standalone_explore_steps(adapter, config, frame, deadline, target, record, owners,
                             stop, before_motion, clock=time.monotonic):
    from .go2_exploration import explore_steps, ExplorationStopped
    from .local_approach_execution import LocalApproachExecution, local_target_accepted
    from .single_robot_approach import load_parameters
    task_id='go2_explore/'+uuid.uuid4().hex
    command_id=uuid.uuid4().hex
    active=True
    adapter.single_robot_mode=True
    def check():
        health=adapter.get_health()
        if not active or clock()>=deadline:
            raise ExplorationStopped('exploration deadline reached or task ended')
        if owners() or not health.nav_ready or not health.localization_valid or adapter.costmap_transform() is None:
            raise ExplorationStopped('motion owner/localization/Nav2/costmap health lost')
        error=motion_path_error(replace(health,path_valid=True),[health.pose])
        if error:raise ExplorationStopped(error)
        return health
    def motion_ready(task,command):
        try:health=check()
        except ExplorationStopped:return False
        scan=getattr(adapter,'scan',None)
        owned=task==task_id or (not task and scan and command==scan['id'])
        return bool(owned and health.stopped and health.motion_terminal)
    adapter.motion_guard=motion_ready
    def approaching(pose,kind,record):
        check()
        parameters=load_parameters(config['robot_go2'].get('arrival_config_files'))
        execution=LocalApproachExecution(adapter,pose,task_id,command_id,kind,parameters,record,clock)
        before_motion()
        outcome=yield from execution.run(check)
        if outcome=='TARGET_APPROACHED':
            while not local_target_accepted(check(),pose,1.0):yield
            execution.accepted()
        return outcome
    try:
        return (yield from explore_steps(adapter,config,frame,deadline,target,record,owners,
            stop,before_motion,clock=clock,task_id=task_id,parent_command_id=command_id,
            initial_camera_check=True,require_navigation_success=True,success_radius=1.0,
            approaching=approaching))
    finally:
        active=False


def run_ros(config,args,events=None):
    require_ros_environment()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from .adapters.go2 import Go2Adapter,make_observer
    from .frame_adapter import FrameAdapter
    motion=args.action in MOTION
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=adapter=executor=None
    started=False
    report=config_report(config,args.map_yaml)
    report['mode']='local_go2_'+args.action
    latest={}; marker_pub=None; run_id=uuid.uuid4().hex; sequence=0
    display=None
    def write_record(event):
        nonlocal latest, sequence
        if display:display.record(event)
        if events is None:return
        sequence+=1
        entry=config['alignment']['robots']['go2']
        if 'state' in event:
            latest=dict(event) if event['state']=='SELECTED' else dict(latest,**event)
        row=dict(latest,**event)
        row.update(run_id=run_id,sequence=sequence,wall_time=time.time(),
            local_map_id=entry['local_map_id'],map_version=entry['map_version'],
            common_frame_id='lab_ref',target=args.target)
        try:
            events.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            events.flush();os.fsync(events.fileno())
        except OSError as error:
            if adapter and getattr(adapter,'single_robot_mode',False):
                node.get_logger().warning('Task diagnostic write failed: '+str(error))
            else:
                report['evidence_write_failed']=True
                if adapter:
                    adapter.evidence_write_failed=True
                    adapter.motion_fault=True
        if marker_pub and 'state' in event:publish_marker()
    def record(event):
        try:write_record(event)
        except Exception as error:
            if not adapter or not getattr(adapter,'single_robot_mode',False):raise
            node.get_logger().warning('Task diagnostic write failed: '+str(error))
    def publish_marker():
        from .go2_exploration import exploration_markers
        markers=exploration_markers(latest)
        for marker in markers.markers:marker.header.stamp=node.get_clock().now().to_msg()
        marker_pub.publish(markers)
    try:
        node=make_observer(config['robot_go2'],read_only=not motion,
            node_name='multi_agent_go2_check_motion' if motion else 'multi_agent_go2_check_readonly')
        entry=config['alignment']['robots']['go2']
        frame=FrameAdapter(entry['local_map_id'],entry['map_version'],'lab_ref',
            config['alignment']['alignment_id'],entry['transform'])
        adapter=Go2Adapter(node,config['robot_go2'],frame,read_only=not motion,
            global_goal_checks=args.action in ('explore','approach'))
        adapter.single_robot_mode=args.action=='explore'
        adapter.record_event=record
        if args.action=='explore':
            adapter.scan_evidence_dir=Path(args.output_dir)/'vlm'
            adapter.selection_evidence_dir=Path(args.output_dir)/'selection'
            report['vlm_evidence_root']=str(adapter.scan_evidence_dir)
        executor=SingleThreadedExecutor();executor.add_node(node)
        if args.action=='explore':
            from visualization_msgs.msg import MarkerArray
            from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
            marker_pub=node.create_publisher(MarkerArray,'/vlm_nav/go2_exploration/current_point',
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
            from .exploration_display import ExplorationDisplay
            display=ExplorationDisplay(node,frame)
            adapter.display_path=lambda path:display.publish('planned_path',path)
            publish_marker()
            node.create_timer(.5,publish_marker)
        parameters=node.create_client(GetParameters,'/map_server/get_parameters')
        active_map=parameters.call_async(GetParameters.Request(names=['yaml_filename']))
        def spin():
            executor.spin_once(timeout_sec=.05)
            if started or args.action=='explore':adapter.tick()
            elif getattr(adapter, 'vision', None) is not None:adapter.vision.poll()
        deadline=time.monotonic()+args.timeout
        # Initial data collection is passive, even for an explicit future motion action.
        until=min(deadline,time.monotonic()+(args.timeout if args.action=='inspect' else 2.))
        began=time.monotonic();next_sample=began;last_checks=None
        input_checks=('localization_valid','local_costmap','rgbd_ready','depth_units_verified','odom_fresh')
        window=dict(samples=0,first_input_ready_elapsed_s=None,post_input_ready_samples=0,
            post_input_ready_observed_s=0.,minimum_observation_s=2.,
            failure_counts={},post_input_ready_failure_counts={},transitions=[])
        while time.monotonic()<until:
            spin()
            now=time.monotonic()
            if args.action!='inspect' or now<next_sample:continue
            next_sample=now+.1
            sample=adapter.get_health();details=getattr(adapter,'last_health_details',{})
            current_owners=other_owners(node)
            checks=dict(details.get('checks',{}),localization_valid=sample.localization_valid,
                nav_ready=sample.nav_ready,stopped=sample.stopped,motion_terminal=sample.motion_terminal,
                local_costmap=adapter.costmap_transform() is not None,
                rgbd_ready=node.vlm_input_ready(now),depth_units_verified=node.depth_units_verified,
                odom_fresh=details.get('odom_age_s',float('inf'))<=details.get('odom_max_age_s',0.),
                no_other_goal_owner=not current_owners)
            failed=[k for k,v in checks.items() if not v]
            window['samples']+=1
            if window['first_input_ready_elapsed_s'] is None and all(checks[k] for k in input_checks):
                window['first_input_ready_elapsed_s']=now-began
            if window['first_input_ready_elapsed_s'] is not None:
                window['post_input_ready_samples']+=1
                window['post_input_ready_observed_s']=now-began-window['first_input_ready_elapsed_s']
            for key in failed:
                window['failure_counts'][key]=window['failure_counts'].get(key,0)+1
                if window['first_input_ready_elapsed_s'] is not None and key in input_checks:
                    counts=window['post_input_ready_failure_counts'];counts[key]=counts.get(key,0)+1
            if checks!=last_checks:
                window['transitions'].append(dict(elapsed_s=now-began,failed_gates=failed,
                    health_details=details,other_goal_owners=current_owners))
                window['transitions']=window['transitions'][-50:]
                last_checks=checks
        if args.action=='inspect':
            window['non_motion_inputs_stable']=(window['first_input_ready_elapsed_s'] is not None
                and window['post_input_ready_samples']>1
                and window['post_input_ready_observed_s']>=window['minimum_observation_s']
                and not window['post_input_ready_failure_counts'])
            report['health_window']=window
        if motion and not node.vlm_input_ready(time.monotonic()):
            record({'state':'WAITING_INPUTS','reason':'waiting for RGB-D synchronization and image-time TF','motion_sent':False})
            # A new TF buffer may need longer than the initial two-second collection.
            while time.monotonic()<deadline and not node.vlm_input_ready(time.monotonic()):spin()
        actual_path=None
        if active_map.done() and active_map.exception() is None:
            values=active_map.result().values
            if values and values[0].string_value:actual_path=str(Path(values[0].string_value).expanduser().resolve())
        configured=config['robot_go2'].get('map_path')
        map_matches=bool(actual_path and configured and actual_path==str(Path(configured).expanduser().resolve()))
        health=adapter.get_health()
        report.update(health=asdict(health),health_details=getattr(adapter,'last_health_details',{}),active_map_yaml=actual_path,active_map_matches=map_matches,
            costmap_valid=adapter.costmap_transform() is not None,
            costmap_frame=adapter.costmap.header.frame_id if adapter.costmap else None,
            costmap_shape=[adapter.costmap.metadata.size_x,adapter.costmap.metadata.size_y] if adapter.costmap else None,
            depth_units_verified=node.depth_units_verified,rgbd_ready=node.vlm_input_ready(time.monotonic()),
            nav_ready_gate=node.nav_ready,control_armed_gate=node.control_armed,
            own_motion_terminal=health.motion_terminal,
            note='own_motion_terminal tracks only this process; inspect does not cancel or certify other clients')
        readonly_ready=not report['go2_local_gates'] and map_matches and health.localization_valid and health.stopped and report['costmap_valid']
        if args.action=='inspect':
            report['local_observation_ready']=readonly_ready
            if args.goal:
                probe=Pose2D(*args.goal)
                report['costmap_probe']={'local_map_pose':asdict(probe),
                    'footprint_free':adapter.costmap_candidate_free(probe)
                    if report['costmap_valid'] and health.footprint else None}
            return 0 if readonly_ready or not args.require_ready else 1
        gates=go2_local_gates(config,motion=motion)
        if args.action=='explore':
            report.update(goal_costmap_topic=config['robot_go2'].get('goal_costmap_topic'),
                goal_costmap_max_age=config['robot_go2'].get('goal_costmap_max_age',1.),
                goal_costmap_valid=adapter.costmap_transform(global_goal=True) is not None)
            if not report['goal_costmap_valid']:gates.append('fresh global goal costmap/TF required')
        if motion and time.monotonic()>=deadline:gates.append('startup input wait timed out')
        if not map_matches:gates.append('active map_server YAML does not match configured map_path')
        if not health.localization_valid or not health.stopped or adapter.costmap_transform() is None:
            gates.append('fresh localization/stationary/realtime costmap required')
        if motion and not health.nav_ready:gates.append('existing NAV_READY/CONTROL_ARMED/RGB-D/action gates closed')
        if motion:
            owners=other_owners(node)
            if owners:gates.append('other upper-level goal owner nodes present: '+', '.join(owners))
        if args.action in ('navigate','explore') and not config['robot_go2'].get('path_topic'):
            gates.append('executing/replanned path topic required')
        if gates:
            report['rejected']=gates;record({'state':'REJECTED','reasons':gates,'motion_sent':False});return 1
        if args.action=='explore':
            from .adapters.go2 import ABCClient
            from .go2_exploration import exploration_gates
            extra=exploration_gates(config)
            if extra:
                report['rejected']=extra;record({'state':'REJECTED','reasons':extra,'motion_sent':False});return 1
            adapter.client=ABCClient(config['robot_go2'])
            def before_motion():
                nonlocal started
                started=True;report['motion_sent']=True
            started=True  # Initial observation also owns cancellation and final stop confirmation.
            steps=standalone_explore_steps(adapter,config,frame,deadline,args.target,record,
                lambda:other_owners(node),lambda:stop_owned(adapter,spin,config['robot_go2'].get('cancel_timeout',5.),record),
                before_motion,clock=time.monotonic)
            try:
                while True:
                    try:next(steps)
                    except StopIteration as finished:
                        report['result']=finished.value
                        break
                    spin()
            finally:
                steps.close()
            return 0 if not args.approach_target or report['result']=='TARGET_APPROACHED' else 1
        if args.action=='scan':
            from .adapters.go2 import ABCClient
            if any(not isinstance(config['scene']['targets'].get(t),str) or '现场填写' in config['scene']['targets'][t]
                   or not config['scene']['targets'][t].strip() for t in ('A','B','C')):
                report['rejected']=['fill actual A/B/C descriptions before scan'];return 1
            adapter.client=ABCClient(config['robot_go2'])  # validate credentials/model before any Spin; no API call here
        goal=Pose2D(*args.goal) if args.goal else None
        future=None
        if args.action in ('plan','approach','navigate'):
            future=adapter.select_approach(goal,[]) if args.action=='approach' else adapter.compute_path(goal)
            while not future.done() and time.monotonic()<deadline:spin()
            if not future.done():future.cancel();report['result']='planning timeout';return 1
            result=future.result();report['preflight']=asdict(result)
            if args.action!='navigate':return 0 if (result.goal if args.action=='approach' else result.reachable) else 1
            if not result.reachable:return 1
        health=adapter.get_health()
        if not health.nav_ready or not health.localization_valid or not health.stopped or not health.motion_terminal or adapter.costmap_transform() is None or other_owners(node):
            report['rejected']=['motion prerequisites changed during preflight'];return 1
        check_path=result.path if args.action=='navigate' else [health.pose]
        error=motion_path_error(replace(health,path_valid=True),check_path)
        if error:report['rejected']=[error];return 1
        started=True
        began=time.monotonic();acquired=False
        if args.action=='navigate':adapter.start_navigation(goal,'go2_acceptance',uuid.uuid4().hex,'WAIT_NAV')
        else:future=adapter.start_scan(config['scene']['targets'],uuid.uuid4().hex)
        report['motion_sent']=True
        record({'state':'NAVIGATING' if args.action=='navigate' else 'SCANNING'})
        while time.monotonic()<deadline:
            spin();health=adapter.get_health()
            if not health.nav_ready or not health.localization_valid or other_owners(node):
                report['result']='health lost';return 1
            if args.cancel_after is not None and time.monotonic()-began>=args.cancel_after:
                report['result']='explicit cancellation requested';return 0
            if args.action=='navigate':
                if adapter.results:
                    result=adapter.results.popleft();report['result']=asdict(result)
                    record(dict(state=result.status,event='NAVIGATION_RESULT',**asdict(result)))
                    return 0 if result.status=='SUCCEEDED' else 1
                if health.path_valid and health.path:acquired=True
                waiting=not acquired and health.stopped and time.monotonic()-began<config['robot_go2'].get('path_start_timeout',1.)
                path=[health.pose] if waiting else health.path
                checked=replace(health,path_valid=True) if waiting else health
                if motion_path_error(checked,path):
                    report['result']='executing path check failed';return 1
            else:
                if motion_path_error(replace(health,path_valid=True),[health.pose]):
                    report['result']='spin stopping envelope failed';return 1
                if future.done():
                    observations=future.result();report['observations']=[asdict(o) for o in observations]
                    return 0 if {o.target_id for o in observations}==set('ABC') and all(o.status!='API_ERROR' for o in observations) else 1
        report['result']='action timeout';return 1
    except KeyboardInterrupt:
        report['result']='operator interrupt';record({'state':'STOPPING','reason':'operator interrupt'});return 1
    except Exception as error:
        from .go2_exploration import ExplorationStopped
        reason=str(error) if isinstance(error,ExplorationStopped) else type(error).__name__
        report['result']=reason;record({'state':'STOPPING','reason':reason});return 1
    finally:
        if started and adapter:
            report['last_native_action_uuid']=getattr(adapter,'last_action_uuid','')
            try:report['stop_confirmed']=stop_owned(adapter,spin,config['robot_go2'].get('cancel_timeout',5.),record)
            except Exception as error:
                report['stop_confirmed']=False
                report['stop_error']=type(error).__name__
            report['evidence_write_failed']=getattr(adapter,'evidence_write_failed',False)
        if args.action=='explore' and adapter:
            job=getattr(getattr(adapter,'vision',None),'job',None)
            if hasattr(args,'_evidence_lease'):args._evidence_lease['pending']=job
            report['vlm_evidence_pending']=job is not None and not job.done()
            for handle in adapter.plans.values():
                if handle:handle.cancel_goal_async()
        if events is not None:
            state=latest.get('state','STOPPED')
            if state=='STOPPING' and report.get('stop_confirmed'):state='STOPPED'
            record({'state':'STOP_UNCONFIRMED' if started and not report['stop_confirmed'] else state,
                'event':'SESSION_END','result':report.get('result'),
                'motion_sent':report.get('motion_sent',False),
                'stop_confirmed':report.get('stop_confirmed') if started else None})
            report['evidence_write_failed']=report.get('evidence_write_failed',False) or getattr(adapter,'evidence_write_failed',False)
            try:
                (Path(args.output_dir)/'result.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
            except OSError:report['evidence_write_failed']=True
        if adapter:adapter.close(stop=False)
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        emit(report)
        if started and not report['stop_confirmed']:
            raise StopUnconfirmed('action terminal and physical stop were not both confirmed')

def main(argv=None):
    p=parser();args=p.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout<=0:p.error('positive finite timeout required')
    if args.action in MOTION and not args.allow_motion:p.error('motion requires --allow-motion and an explicit action')
    if args.offline and args.action!='inspect':p.error('--offline supports inspection only')
    if (args.approach_target or args.search_hint) and args.action!='explore':
        p.error('--approach-target/--search-hint require --action explore')
    if args.action=='explore' and (args.goal or args.cancel_after is not None):
        p.error('explore requires --output-dir, configured viewpoints, and no --goal/--cancel-after')
    if (args.goal and not all(math.isfinite(v) for v in args.goal)) or (args.action in ('plan','approach','navigate') and not args.goal):
        p.error('finite measured --goal X Y YAW required')
    if args.cancel_after is not None and (args.action not in MOTION or not math.isfinite(args.cancel_after) or not 0<args.cancel_after<args.timeout):
        p.error('--cancel-after requires motion and 0 < seconds < timeout')
    try:
        config=load_config(args.config_dir)
        if args.strategy:config['robot_go2']['exploration_strategy'] = args.strategy
        if args.approach_target:config['robot_go2']['approach_on_detection'] = True
        if args.search_hint:config['robot_go2']['search_hint'] = args.search_hint
        if args.action=='explore':
            from .go2_exploration import measured_viewpoints
            from .semantic_search import load_catalog
            semantic = config['robot_go2'].get('exploration_strategy') in ('semantic_furniture','generated_candidates_vlm')
            if not args.output_dir:
                if config['robot_go2'].get('exploration_strategy')!='generated_candidates_vlm':
                    p.error('fixed/semantic exploration requires --output-dir')
                root=Path(config['robot_go2']['experiment_records_root'])
                args.output_dir=str(root/('experiment.'+time.strftime('%Y%m%d_%H%M%S')+'.'+uuid.uuid4().hex[:8]))
            if not semantic:measured_viewpoints(config)
            args.target=args.target or config['scene']['targets']['A']
            if not args.target.strip() or '现场填写' in args.target:
                raise ValueError('measured exploration target description required')
        if args.offline:
            report=config_report(config,args.map_yaml);emit(report)
            return 1 if args.require_ready and report['single_robot_experiment_gates'] else 0
        if args.action in MOTION:
            if args.output_dir:
                from .experiment_records import experiment_bundle
                root=config['robot_go2'].get('experiment_records_root',str(Path(args.output_dir).parent / 'managed-experiments'))
                with experiment_bundle(args.output_dir,root) as lease:
                    args._evidence_lease=lease
                    with (Path(args.output_dir)/'events.jsonl').open('x') as events:
                        from .go2_exploration import exploration_gates
                        reasons=exploration_gates(config) if args.action=='explore' else go2_local_gates(config,motion=True)
                        if args.action=='navigate' and not config['robot_go2'].get('path_topic'):
                            reasons.append('executing/replanned path topic required')
                        if not reasons:
                            try:require_ros_environment()
                            except Exception as error:
                                reasons=[str(error) if isinstance(error,ValueError) else type(error).__name__]
                        if reasons:
                            report=config_report(config)
                            report.update(mode='local_go2_'+args.action,rejected=reasons,target=args.target)
                            if args.action=='explore':
                                report['viewpoints'] = (report.get('semantic_search') or {}).get('points', []) if semantic else measured_viewpoints(config)
                            events.write(json.dumps({'state':'REJECTED','wall_time':time.time(),
                                'reasons':reasons,'motion_sent':False,'target':args.target,
                                'viewpoints':report.get('viewpoints')},ensure_ascii=False)+'\n')
                            events.flush();os.fsync(events.fileno())
                            (Path(args.output_dir)/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
                            emit(report);return 1
                        with motion_slot():return run_ros(config,args,events)
            with motion_slot():return run_ros(config,args)
        return run_ros(config,args)
    except Exception as error:
        import sys
        print('Go2 check failed: '+type(error).__name__,file=sys.stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())
