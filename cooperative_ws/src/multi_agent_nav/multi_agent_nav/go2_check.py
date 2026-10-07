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
from .models import Pose2D

MOTION=('navigate','scan')
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
    p.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'),help='measured local Go2 map pose; inspect only probes costmap footprint; approach treats X/Y as object position')
    p.add_argument('--timeout',type=float,default=15.,help='inspection/action deadline in seconds')
    p.add_argument('--cancel-after',type=float,help='explicit cancellation acceptance during navigate/scan')
    return p

def emit(value):
    print(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2))

def config_report(config,map_yaml=None):
    path=map_yaml or config['robot_go2'].get('map_path')
    try:info=map_file_info(path) if path else None
    except (OSError,KeyError,TypeError,ValueError):info={'error':'map YAML/image unreadable'}
    return {'mode':'configuration_only','map_files':info,
        'go2_local_gates':go2_local_gates(config),'live_gates':live_gates(config),
        'motion_sent':False}

def stop_owned(adapter,spin,timeout):
    adapter.stop_motion('independent acceptance completion/cancel')
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        spin()
        health=adapter.get_health()
        if health.stopped and health.motion_terminal:return True
    return False

def run_ros(config,args):
    if os.getenv('ROS_DOMAIN_ID','0')!='0':raise ValueError('Go2 acceptance requires local Domain0')
    if os.getenv('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp' or os.getenv('GO2_NET_IFACE')!='enp4s0':
        raise ValueError('use runbook Domain0/CycloneDDS/enp4s0 environment')
    from urllib.parse import urlparse,unquote
    from ament_index_python.packages import get_package_share_directory
    uri=urlparse(os.getenv('CYCLONEDDS_URI',''))
    expected_xml=Path(get_package_share_directory('vlm_nav'))/'config/cyclonedds_go2.xml'
    if uri.scheme!='file' or uri.netloc or not uri.path or Path(unquote(uri.path)).read_bytes()!=expected_xml.read_bytes():
        raise ValueError('use the documented Go2-only CycloneDDS XML')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from .adapters.go2 import Go2Adapter,make_observer
    from .frame_adapter import FrameAdapter
    from .region_guard import RegionGuard
    motion=args.action in MOTION
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=adapter=executor=None
    started=False
    report=config_report(config,args.map_yaml)
    report['mode']='local_go2_'+args.action
    try:
        node=make_observer(config['robot_go2'],read_only=not motion,
            node_name='multi_agent_go2_check_motion' if motion else 'multi_agent_go2_check_readonly')
        entry=config['alignment']['robots']['go2']
        frame=FrameAdapter(entry['local_map_id'],entry['map_version'],'lab_ref',
            config['alignment']['alignment_id'],entry['transform'])
        adapter=Go2Adapter(node,config['robot_go2'],frame,read_only=not motion)
        executor=SingleThreadedExecutor();executor.add_node(node)
        parameters=node.create_client(GetParameters,'/map_server/get_parameters')
        active_map=parameters.call_async(GetParameters.Request(names=['yaml_filename']))
        def spin():
            executor.spin_once(timeout_sec=.05)
            if started:adapter.tick()
        deadline=time.monotonic()+args.timeout
        # Initial data collection is passive, even for an explicit future motion action.
        until=min(deadline,time.monotonic()+(args.timeout if args.action=='inspect' else 2.))
        while time.monotonic()<until:spin()
        actual_path=None
        if active_map.done() and active_map.exception() is None:
            values=active_map.result().values
            if values and values[0].string_value:actual_path=str(Path(values[0].string_value).expanduser().resolve())
        configured=config['robot_go2'].get('map_path')
        map_matches=bool(actual_path and configured and actual_path==str(Path(configured).expanduser().resolve()))
        health=adapter.get_health()
        report.update(health=asdict(health),active_map_yaml=actual_path,active_map_matches=map_matches,
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
                    'footprint_free':adapter.costmap_candidate_free(probe,health.footprint)
                    if report['costmap_valid'] and health.footprint else None}
            return 0 if readonly_ready or not args.require_ready else 1
        gates=go2_local_gates(config,motion=motion)
        if not map_matches:gates.append('active map_server YAML does not match configured map_path')
        if not health.localization_valid or not health.stopped or adapter.costmap_transform() is None:
            gates.append('fresh localization/stationary/realtime costmap required')
        if motion and not health.nav_ready:gates.append('existing NAV_READY/CONTROL_ARMED/RGB-D/action gates closed')
        if motion:
            owners=other_owners(node)
            if owners:gates.append('other upper-level goal owner nodes present: '+', '.join(owners))
        if args.action=='navigate' and (not config['robot_go2'].get('path_topic') or not config['robot_go2'].get('path_source_verified')):
            gates.append('verified executing/replanned path source required')
        if gates:report['rejected']=gates;return 1
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
        guard=RegionGuard(config['scene']['resources'],config['robot_go2']['stop_margin'])
        # This entry has no resource authority: every protected region remains unpermitted.
        health=adapter.get_health()
        if not health.nav_ready or not health.localization_valid or not health.stopped or not health.motion_terminal or adapter.costmap_transform() is None or other_owners(node):
            report['rejected']=['motion prerequisites changed during preflight'];return 1
        check_path=result.path if args.action=='navigate' else [health.pose]
        decision=guard.check(health.pose,health.footprint,check_path,{},replace(health,path_valid=True))
        if decision.action!='ALLOW':report['rejected']=[decision.reason or 'protected resource requires real coordinator permit'];return 1
        started=True
        began=time.monotonic();acquired=False
        if args.action=='navigate':adapter.start_navigation(goal,'go2_acceptance',uuid.uuid4().hex,'WAIT_NAV')
        else:future=adapter.start_scan(config['scene']['targets'],uuid.uuid4().hex)
        report['motion_sent']=True
        while time.monotonic()<deadline:
            spin();health=adapter.get_health()
            if not health.nav_ready or not health.localization_valid or other_owners(node):
                report['result']='health lost';return 1
            if args.cancel_after is not None and time.monotonic()-began>=args.cancel_after:
                report['result']='explicit cancellation requested';return 0
            if args.action=='navigate':
                if health.path_valid and health.path:acquired=True
                waiting=not acquired and health.stopped and time.monotonic()-began<config['robot_go2'].get('path_start_timeout',1.)
                path=[health.pose] if waiting else health.path
                checked=replace(health,path_valid=True) if waiting else health
                if guard.check(health.pose,health.footprint,path,{},checked).action!='ALLOW':
                    report['result']='executing path/resource guard failed';return 1
                if adapter.results:
                    result=adapter.results.popleft();report['result']=asdict(result)
                    return 0 if result.status=='SUCCEEDED' else 1
            else:
                if guard.check(health.pose,health.footprint,[health.pose],{},replace(health,path_valid=True)).action!='ALLOW':
                    report['result']='spin stopping envelope failed';return 1
                if future.done():
                    observations=future.result();report['observations']=[asdict(o) for o in observations]
                    return 0 if {o.target_id for o in observations}==set('ABC') and all(o.status!='API_ERROR' for o in observations) else 1
        report['result']='action timeout';return 1
    except KeyboardInterrupt:
        report['result']='operator interrupt';return 1
    finally:
        if started and adapter:
            report['last_native_action_uuid']=getattr(adapter,'last_action_uuid','')
            try:report['stop_confirmed']=stop_owned(adapter,spin,config['robot_go2'].get('cancel_timeout',5.))
            except Exception as error:
                report['stop_confirmed']=False
                report['stop_error']=type(error).__name__
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
    if (args.goal and not all(math.isfinite(v) for v in args.goal)) or (args.action in ('plan','approach','navigate') and not args.goal):
        p.error('finite measured --goal X Y YAW required')
    if args.cancel_after is not None and (args.action not in MOTION or not math.isfinite(args.cancel_after) or not 0<args.cancel_after<args.timeout):
        p.error('--cancel-after requires motion and 0 < seconds < timeout')
    try:
        config=load_config(args.config_dir)
        if args.offline:
            report=config_report(config,args.map_yaml);emit(report)
            return 1 if args.require_ready and report['live_gates'] else 0
        if args.action in MOTION:
            with motion_slot():return run_ros(config,args)
        return run_ros(config,args)
    except Exception as error:
        import sys
        print('Go2 check failed: '+type(error).__name__,file=sys.stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())
