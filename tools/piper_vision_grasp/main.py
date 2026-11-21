"""One local-reference grasp cycle. No ROS, external project imports, or automatic replay."""
import argparse
import contextlib
import copy
from importlib.metadata import version
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = 'piper_vision_grasp'

import numpy as np
from . import geometry, vision
from .piper_io import PiperSession

ROOT = Path(__file__).resolve().parents[2]


def clean(value):
    if isinstance(value, dict): return {k:clean(v) for k,v in value.items() if k not in ('mask','annotation_bgr')}
    if isinstance(value, (list,tuple)): return [clean(v) for v in value]
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    return value


def save(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as stream:
        json.dump(clean(value),stream,indent=2,allow_nan=False);stream.write('\n')
        stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)


def reserve_run(root, run_id):
    if not run_id or Path(run_id).name!=run_id or run_id in ('.','..'):
        raise ValueError('invalid run id')
    directory=Path(root)/run_id
    directory.mkdir(parents=True,exist_ok=False)
    fd=os.open(str(directory.parent),os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)
    return directory


class Events:
    def __init__(self, config, directory):
        self.config=config;self.directory=Path(directory)
        self.last={};self.attempted=False
        self.recovery=False
        self.feedback_epoch=0
    def emit(self, state, reason='', stage='', target=None, measured=None, **fields):
        row=dict(time=time.time(),state=state,reason_class='NONE' if not state.startswith(('WAIT','COMMAND_UNCERTAIN')) else state,
                 reason=reason,stage=stage,target=target,measured=measured,command_attempted=self.attempted,
                 frame_age=None,reference_id=self.config['reference']['reference_id'],run_id=self.config.get('run_id',self.directory.name))
        row.update(fields)
        with (self.directory/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(clean(row),allow_nan=False)+'\n');stream.flush()
        self.last=row
        if state in ('WAIT_IO','WAIT_FEEDBACK','WAIT_FAULT','WAIT_GRIPPER','WAIT_CONTROL','DISABLED'):
            self.recovery=True
            self.feedback_epoch+=1
        return row


def save_observation(directory, captured):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    save(directory/'observation.json',captured)
    for i,obs in enumerate(captured['observations']):
        if 'mask' in obs:
            import cv2
            if not cv2.imwrite(str(directory/('mask{:02d}.png'.format(i))),np.asarray(obs['mask'],dtype=np.uint8)*255):
                raise OSError('mask write failed')
        if 'annotation_bgr' in obs:
            import cv2
            if not cv2.imwrite(str(directory/('detection{:02d}.png'.format(i))),obs['annotation_bgr']):
                raise OSError('annotation write failed')


def preview_input(config, source, out_dir):
    requested_target=copy.deepcopy(config.get('target'))
    if config.get('grasp_backend') == 'graspnet':
        from .graspnet import load_saved_capture
        captured=load_saved_capture(source,config)
        config=copy.deepcopy(config)
        config['target']=copy.deepcopy(captured['observation']['target'])
    else:
        captured=vision.load_capture(source,config)
    save_observation(out_dir,captured)
    metadata=captured['metadata']
    rows=[p['robot_snapshot'] for p in metadata['paired_robot']]
    if not rows or any(r['missing_feedback'] for r in rows):
        raise ValueError('offline input lacks fresh paired feedback')
    if max(np.max(np.abs(np.array(r['joints_deg'])-rows[0]['joints_deg'])) for r in rows)>.3:
        raise ValueError('robot moved during saved capture')
    plan=make_plan(config,rows[-1]['joints_deg'],captured['observation'],captured)
    save(Path(out_dir)/'plan.json',plan)
    return dict(state='PREVIEW_READY' if plan['status']=='PLANNED' else plan['status'],
                can_transmit_count=0,offline=True,source=str(source),plan_status=plan['status'],
                target_spec=config.get('target'),requested_target_spec=requested_target,
                grasp_confirmed=False,reason=(plan.get('reason','planning failed')+'; ' if plan['status']!='PLANNED' else '')+
                ('saved target masks: '+json.dumps(config['target'],ensure_ascii=False)+'; ' if config.get('grasp_backend')=='graspnet' else '')+
                'saved data only; not current hardware state')


def wait_snapshot(config,session,events,stage):
    while True:
        try: row=session.snapshot()
        except Exception as error: row=dict(state='WAIT_IO',reason=str(error))
        # Firmware target-reached status is not velocity; callers check actual positions.
        if row.get('state')=='READY' and not row.get('missing_feedback'):
            return row
        events.emit(row.get('state','WAIT_FEEDBACK'),row.get('reason','waiting for fresh feedback'),stage,measured=row)
        time.sleep(config['poll_seconds'])


def acquire(config,session,camera,events,label):
    while True:
        before=wait_snapshot(config,session,events,label)
        directory=events.directory/(label+'-'+uuid.uuid4().hex[:8])
        try:
            captured=camera(config,directory,count=3)
            after=wait_snapshot(config,session,events,label)
            frames=captured['metadata']['frames']
            age=time.time()-min(f['host_unix'] for f in frames)
            if not 0<=age<=config['frame_max_age_s']:
                raise ValueError('camera frames stale/future')
            if len(frames)!=3 or any(b['color_frame']<=a['color_frame'] or b['depth_frame']<=a['depth_frame'] for a,b in zip(frames,frames[1:])):
                raise ValueError('need three distinct RGB-D frames')
            if np.max(np.abs(np.array(before['joints_deg'])-after['joints_deg']))>.3:
                raise ValueError('robot changed during finite capture; reacquire')
            captured['metadata']['paired_robot']=[dict(robot_snapshot=before),dict(robot_snapshot=after)]
            captured['capture_dir']=str(directory.resolve())
            save_observation(directory,captured)
            if (directory/'metadata.json').exists():save(directory/'metadata.json',captured['metadata'])
            obs=captured['observation']
            if obs['status']!='DETECTED':
                events.emit(obs['status'],obs['reason'],label,frame_age=age)
                time.sleep(config['poll_seconds']);continue
            events.emit('OBSERVED','fresh finite RGB-D with stationary endpoint feedback',label,measured=obs,frame_age=age)
            return captured,after
        except Exception as error:
            events.emit('WAIT_VISION',str(error),label)
            time.sleep(config['poll_seconds'])



def configure_target(config, color=None, description=None, approach_pitch=None):
    if config.get('grasp_backend') == 'graspnet' and approach_pitch is not None:
        raise ValueError('GraspNet determines orientation; approach-pitch override is incompatible')
    if config.get('mode') == 'semantic':
        target=config['target'].copy()
        if color is not None:target['color']=color
        if description is not None:target['description']=description
        config['target']=target
        if approach_pitch is not None:
            config['semantic']['approach_pitch_deg']=approach_pitch
            config['semantic'].pop('angle_search',None)
    elif color is not None or description is not None or approach_pitch is not None:
        raise ValueError('color/description require semantic.json configuration')


def make_plan(config, joints, observation, captured=None, on_candidate=None):
    if config.get('grasp_backend') == 'graspnet':
        from .graspnet import plan_capture
        if captured is None:raise ValueError('GraspNet requires paired RGB-D capture and target mask')
        return plan_capture(config,joints,captured)
    if config.get('mode') == 'semantic':
        if config['semantic'].get('angle_search') is not None:
            return geometry.search_orientations(joints,config,observation,on_candidate)
        target=geometry.absolute_target(observation,config)
        return geometry.plan_absolute(joints,config,target)
    target=geometry.retarget(config['reference'],observation,config['extrinsics'])
    return geometry.plan_motion(joints,config['reference'],target,config['kinematics'])


def prepare_plan(config,session,camera,events):
    while True:
        captured,row=acquire(config,session,camera,events,'before')
        try:
            def candidate_progress(candidate):
                events.emit('SEARCHING_IK',candidate['reason'],'planning',measured=candidate)
            plan=make_plan(config,row['joints_deg'],captured['observation'],captured,candidate_progress)
            plan['capture_dir']=captured['capture_dir']
            save(Path(captured['capture_dir'])/'planning.json',dict(
                config=config,initial_joints_deg=row['joints_deg'],plan=plan))
            if config.get('grasp_backend') == 'graspnet':save(Path(captured['capture_dir'])/'graspnet-plan.json',plan)
            if plan['status']!='PLANNED':raise ValueError(plan.get('reason','IK failed'))
            now=wait_snapshot(config,session,events,'planning')
            if np.max(np.abs(np.array(now['joints_deg'])-row['joints_deg']))>.3:
                raise ValueError('initial joints changed during planning; replan')
            if time.time()-min(f['host_unix'] for f in captured['metadata']['frames'])>config['frame_max_age_s']:
                raise ValueError('planning outlasted visual freshness; reacquire')
            save(events.directory/'plan.json',plan)
            return captured,plan
        except ValueError as error:
            events.emit('WAIT_PLAN',str(error),'planning')
            time.sleep(config['poll_seconds'])


def send_once(config,session,events,stage,kind,target,recover=None):
    while True:
        if recover and events.recovery:recover()
        try:
            if kind=='joint':result=session.attempt_joint(stage,target,config['motion']['speed_percent'])
            else:
                grip=config['gripper']
                effort=grip.get('open_effort_nm',grip['effort_nm']) if stage=='open' else grip['effort_nm']
                result=session.attempt_gripper(stage,target,effort)
        except Exception as error:
            result=dict(state='COMMAND_UNCERTAIN',command_attempted=True,reason=str(error))
        if result.get('command_attempted'):events.attempted=True
        events.emit(result['state'],result.get('reason',''),stage,target)
        if result['state']=='COMMAND_ATTEMPTED':return
        if result['state']=='COMMAND_UNCERTAIN':
            while True:
                events.emit('COMMAND_UNCERTAIN','no resend or automatic stage continuation',stage,target,session.snapshot())
                time.sleep(config['poll_seconds'])
        time.sleep(config['poll_seconds'])


def revalidate_after_wait(config,session,camera,events,before):
    if not events.recovery:return
    while True:
        fresh,_=acquire(config,session,camera,events,'recovery')
        a,b=before['observation'],fresh['observation']
        if (np.linalg.norm(np.array(a['point_color_m'])-b['point_color_m'])<=config['detector']['stability_m'] and
                vision.orientation_difference(a,b)<=config['detector']['orientation_tolerance_deg']):
            events.recovery=False
            return
        events.emit('WAIT_REPLAN','target changed after motion; restore scene or end cycle and review executed stages; no old target sent','recovery',measured=b)
        time.sleep(config['poll_seconds'])


def wait_joints(config,session,events,stage):
    while True:
        row=wait_snapshot(config,session,events,stage['stage_id'])
        error=float(np.max(np.abs(np.array(row['joints_deg'])-stage['angles_deg'])))
        if error<=config['motion']['joint_tolerance_deg']:
            events.emit('AT_TARGET','fresh joint positions within configured tolerance',stage['stage_id'],stage['angles_deg'],row)
            return row
        events.emit('WAIT_TARGET','joint error_deg='+str(error),stage['stage_id'],stage['angles_deg'],row)
        time.sleep(config['poll_seconds'])


def wait_gripper(config,session,events,stage,opened_width=None):
    history=[];last_stamp=-1
    grip=config['gripper']
    while True:
        epoch=events.feedback_epoch
        row=wait_snapshot(config,session,events,stage)
        if events.feedback_epoch!=epoch:history=[]
        if not row.get('gripper_enabled'):
            history=[];events.emit('WAIT_GRIPPER','gripper not enabled',stage,measured=row)
            time.sleep(config['poll_seconds']);continue
        width=row['gripper_width_mm'];stamp=row['sample_time']
        if stamp<=last_stamp:
            events.emit('WAIT_FEEDBACK','no newer gripper sample',stage,measured=row)
            time.sleep(config['poll_seconds']);continue
        if stamp-last_stamp>1.:history=[]
        last_stamp=stamp
        history.append((stamp,width))
        while len(history)>2 and history[1][0]<=stamp-grip['stable_seconds']:history.pop(0)
        stable=stamp-history[0][0]>=grip['stable_seconds'] and max(w for _,w in history)-min(w for _,w in history)<=grip['stable_range_mm']
        target=grip['open_mm'] if stage=='open' else grip['close_mm']
        low_level='AT_TARGET' if abs(width-target)<=1 else 'WAIT_TARGET'
        valid=width>=grip['open_ready_mm'] if stage=='open' else (width>target+grip['contact_margin_mm'] and width<opened_width-grip['contact_margin_mm'])
        if stable and valid:
            events.emit('OPEN_READY' if stage=='open' else 'CONTACT_CANDIDATE','stable feedback; object grasp not yet verified',stage,target,row,low_level_state=low_level)
            return width
        events.emit('WAIT_OPEN' if stage=='open' else 'WAIT_CONTACT','stable nonempty contact required' if stage!='open' else 'waiting for opening',stage,target,row,low_level_state=low_level)
        time.sleep(config['poll_seconds'])


def arm_capture_stages(plan):
    names=[s['stage_id'] for s in plan['stages']]
    approach=[name for name in names if name.startswith('grasp-')]
    if not approach:raise ValueError('four-pose capture requires an intermediate grasp approach waypoint')
    selected=['pregrasp',approach[(len(approach)-1)//2],'grasp','lift']
    if not all(name in names for name in selected):raise ValueError('four-pose capture stages missing')
    return selected


def record_arm_pose(config,session,events,stage):
    from .arm_outline import segment_capture
    while True:
        directory=events.directory/('arm-'+stage+'-'+uuid.uuid4().hex[:8])
        try:
            feedback_epoch=events.feedback_epoch
            captured=vision.capture_rgbd(config,directory,count=3,detect=False,
                snapshot=lambda:wait_snapshot(config,session,events,stage))
            if events.feedback_epoch!=feedback_epoch:
                raise ValueError('feedback interrupted during arm capture; reacquire after recovery')
            frames=captured['metadata']['frames']
            snapshots=[p['robot_snapshot'] for f in frames for p in f['paired_robot']]
            for frame in frames:
                pair=frame['robot_pairing'];a,b=pair['before_read_unix'],pair['after_read_unix']
                # Existing 3s camera wait plus the 1s feedback freshness window.
                if not a<=frame['host_unix']<=b or not 0<=b-a<=4:
                    raise ValueError('arm frame/feedback pairing window invalid; reacquire')
            if not 0<=time.time()-min(f['host_unix'] for f in frames)<=config['frame_max_age_s']:
                raise ValueError('arm capture frames stale/future')
            q=np.asarray([s['joints_deg'] for s in snapshots],dtype=float)
            if q.shape!=(6,6) or not np.all(np.isfinite(q)) or np.max(np.ptp(q,axis=0))>.3:
                raise ValueError('arm moved during contour capture; reacquire')
            widths=np.asarray([s['gripper_width_mm'] for s in snapshots],dtype=float)
            if not np.all(np.isfinite(widths)) or np.any(widths<0) or np.ptp(widths)>1:
                raise ValueError('gripper moved during contour capture; reacquire')
            segment_capture(directory,config,stage)
            events.emit('ARM_POSE_RECORDED','RGB-D, independent image mask and paired joint feedback saved',stage,
                        measured=snapshots[-1],capture_dir=str(directory))
            return
        except Exception as error:
            events.emit('WAIT_ARM_CAPTURE',str(error),stage,capture_dir=str(directory))
            time.sleep(config['poll_seconds'])


def return_zero_holding(config,session,events,plan):
    while True:
        before=wait_snapshot(config,session,events,'return-zero')
        try:
            stages=geometry.zero_stages(before['joints_deg'],config['kinematics'],'return-zero')
            break
        except ValueError as error:
            events.emit('WAIT_PLAN',str(error),'return-zero');time.sleep(config['poll_seconds'])
    save(events.directory/'return-plan.json',dict(initial_joints_deg=before['joints_deg'],stages=stages,
        speed_percent=config['motion']['speed_percent'],gripper_command='hold existing close command'))
    plan['stages'].extend(stages);save(events.directory/'plan.json',plan)
    for stage in stages:
        send_once(config,session,events,stage['stage_id'],'joint',stage['angles_deg'])
        measured=wait_joints(config,session,events,stage)
    return measured


def run_once(config,session,camera,out_dir):
    out_dir=Path(out_dir);out_dir.mkdir(parents=True,exist_ok=True)
    with (out_dir/'cycle.json').open('x') as receipt:
        json.dump(dict(run_id=config.get('run_id',out_dir.name),time=time.time()),receipt)
        receipt.flush();os.fsync(receipt.fileno())
    fd=os.open(str(out_dir),os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)
    events=Events(config,out_dir)
    try:
        events.emit('STARTING','one cycle; no automatic release',return_zero=config['motion'].get('return_zero_after_grasp',False))
        _,plan=prepare_plan(config,session,camera,events)
        send_once(config,session,events,'open','gripper',config['gripper']['open_mm'])
        opened=wait_gripper(config,session,events,'open')
        # Opening may take time; never execute a plan based on stale pre-opening images.
        before,plan=prepare_plan(config,session,camera,events)
        capture_stages=arm_capture_stages(plan) if config['motion'].get('record_arm_outline') else []
        if capture_stages:
            plan['arm_capture_stages']=capture_stages;save(out_dir/'plan.json',plan)
        events.recovery=False
        def recover():revalidate_after_wait(config,session,camera,events,before)
        for stage in plan['stages']:
            if stage['phase']=='lift':continue
            send_once(config,session,events,stage['stage_id'],'joint',stage['angles_deg'],recover)
            wait_joints(config,session,events,stage)
            if stage['stage_id'] in capture_stages:record_arm_pose(config,session,events,stage['stage_id'])
        if config.get('mode')=='semantic':config['gripper']['close_mm']=plan['close_mm']
        send_once(config,session,events,'close','gripper',config['gripper']['close_mm'],recover)
        wait_gripper(config,session,events,'close',opened)
        for stage in plan['stages']:
            if stage['phase']!='lift':continue
            send_once(config,session,events,stage['stage_id'],'joint',stage['angles_deg'],recover)
            wait_joints(config,session,events,stage)
            if stage['stage_id'] in capture_stages:record_arm_pose(config,session,events,stage['stage_id'])
        while True:
            after,_=acquire(config,session,camera,events,'after-lift')
            verification=vision.verify_lift(before['observation'],after['observations'],[0,0,config['motion']['lift_m']],config['extrinsics'])
            save(out_dir/'verification.json',verification)
            if verification['status']=='VERIFIED':
                result=dict(events.emit('GRASP_CONFIRMED',verification['reason'],'hold',measured=verification),
                            grasp_confirmed=True,stop_confirmed=None,automatic_release=False,human_direction_corrections=0)
                if config['motion'].get('return_zero_after_grasp'):
                    measured=return_zero_holding(config,session,events,plan)
                    result=dict(events.emit('GRASP_RETURNED_ZERO','six joints returned; original close command held',
                        'return-zero-final',[0]*6,measured),grasp_confirmed=True,grasp_confirmed_at='after_lift',
                        holding_at_zero_verified=False,automatic_release=False,stop_confirmed=None)
                break
            events.emit('WAIT_VERIFY',verification['reason'],'hold',measured=verification)
            time.sleep(config['poll_seconds'])
    except KeyboardInterrupt:
        previous=events.last.get('state')
        result=dict(events.emit('EXITED','monitor exited; no physical stop/release/disable sent',events.last.get('stage','')),
                    grasp_confirmed=False,stop_confirmed=None,previous_state=previous)
    except Exception as error:
        # After an unexpected runtime error, preserve the executed stages and only monitor.
        try:
            while True:
                events.emit('WAIT_RUNTIME',str(error),events.last.get('stage',''),measured=session.snapshot())
                time.sleep(config['poll_seconds'])
        except KeyboardInterrupt:
            result=dict(events.emit('EXITED','runtime wait interrupted; no further commands'),grasp_confirmed=False,stop_confirmed=None)
    save(out_dir/'result.json',result)
    return result


def run_zero(config,session,out_dir):
    """Return six joint angles to zero; keep gripper and calibration unchanged."""
    out_dir=Path(out_dir);events=Events(config,out_dir)
    try:
        try:
            while True:
                before=wait_snapshot(config,session,events,'before-zero')
                try:
                    start=np.asarray(before['joints_deg'],dtype=float)
                    stages=geometry.zero_stages(start,config['kinematics'])
                    break
                except (ValueError,TypeError,KeyError) as error:
                    events.emit('WAIT_PLAN',str(error),'before-zero')
                    time.sleep(config['poll_seconds'])
            save(out_dir/'before.json',before)
            save(out_dir/'plan.json',dict(initial_joints_deg=start.tolist(),stages=stages,
                                         speed_percent=config['motion']['speed_percent']))
            for stage in stages:
                send_once(config,session,events,stage['stage_id'],'joint',stage['angles_deg'])
                measured=wait_joints(config,session,events,stage)
            result=events.emit('ZERO_REACHED','six joints within configured tolerance; gripper and calibration unchanged',
                               'zero-final',[0]*6,measured)
        except Exception as error:
            # Preserve command receipts on unexpected errors; never restart the motion sequence.
            while True:
                try:events.emit('WAIT_RUNTIME',str(error),events.last.get('stage',''))
                except OSError:pass  # Keep waiting even while diagnostic storage is unavailable.
                time.sleep(config['poll_seconds'])
    except KeyboardInterrupt:
        previous=events.last.get('state')
        result=events.emit('EXITED','monitor exited; no physical stop/release/disable sent',
                           events.last.get('stage',''),previous_state=previous,stop_confirmed=None)
    save(out_dir/'result.json',result)
    return result


def dependency_report():
    dependencies={}
    for package in ('numpy','scipy','pyrealsense2','piper-sdk','python-can'):
        dependencies[package]=version(package)
    import cv2
    dependencies['opencv']=cv2.__version__
    if dependencies['piper-sdk']!='0.6.2' or dependencies['python-can']!='4.5.0':
        raise ValueError('requires existing piper-sdk 0.6.2 and python-can 4.5.0')
    return dependencies


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['check','observe','preview','run','zero','diagnostics','serve','serve-graspnet','visualize'])
    parser.add_argument('--config',type=Path,default=ROOT/'config/piper_vision_grasp/semantic-camera-avoid.json')
    parser.add_argument('--input',type=Path)
    parser.add_argument('--urdf',type=Path,default=Path.home()/'unitree_ros2/piper/agent/piper_pinocchio/piper_description/urdf/piper_description.urdf')
    parser.add_argument('--color')
    parser.add_argument('--description')
    parser.add_argument('--approach-pitch-deg',type=float,help='use one fixed pitch and disable automatic pitch/yaw search')
    parser.add_argument('--output-root',type=Path,default=Path.home()/'.local/state/piper-vision-grasp')
    parser.add_argument('--run-id',default=None)
    args=parser.parse_args()
    if args.input and args.action not in ('preview','visualize'):parser.error('--input is only allowed for offline preview/visualize')
    if args.action=='visualize' and not args.input:parser.error('visualize requires --input saved preview directory')
    if args.action=='diagnostics':
        os.execvp('tail',['tail','-n','40','-F',str(args.output_root/'current.log')])
    run_id=args.run_id or args.action+'-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
    try:out=reserve_run(args.output_root,run_id)
    except (OSError,ValueError):
        print('Existing/invalid run directory: '+str(args.output_root/run_id),file=sys.stderr);return 2
    log=out/'events.jsonl';log.touch()
    link=args.output_root/('current-'+uuid.uuid4().hex+'.link');link.symlink_to(log.resolve());link.replace(args.output_root/'current.log')
    print('诊断日志：'+str(log),flush=True)
    print('运行结果：'+str(out/'result.json'),flush=True)
    diagnostic = shutil.which('gnome-terminal') if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') else None
    if diagnostic:
        try:subprocess.Popen([diagnostic,'--window','--title=Piper visual grasp','--','tail','-n','40','-F',str(log)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        except OSError:diagnostic=None
    if not diagnostic:print('诊断终端：bash scripts/piper_vision_grasp.sh diagnostics --output-root '+str(args.output_root),file=sys.stderr)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    with (out/'runtime.log').open('a') as runtime, contextlib.redirect_stdout(runtime),contextlib.redirect_stderr(runtime):
        try:
            config=geometry.load_config(args.config)
            if (args.action=='preview' and args.input and config.get('grasp_backend')=='graspnet'
                    and (args.color is not None or args.description is not None)):
                raise ValueError('offline GraspNet replay uses saved target masks; color/description require new segmentation')
            configure_target(config,args.color,args.description,args.approach_pitch_deg)
            save(out/'config_snapshot.json',config)
            config.update(run_id=run_id,run_dir=str(out),readonly=args.action not in ('run','zero'))
            events=Events(config,out)
            semantic_settings=config.get('semantic',{})
            events.emit('STARTING',python=sys.executable,module=str(Path(__file__).resolve()),config=str(args.config.resolve()),action=args.action,target_spec=config.get('target'),
                        approach_pitch_deg=None if semantic_settings.get('angle_search') is not None else semantic_settings.get('approach_pitch_deg'),
                        angle_search=semantic_settings.get('angle_search'))
            if args.action=='serve':
                from .semantic_server import serve
                serve(config['semantic'],events)
                return 0
            if args.action=='serve-graspnet':
                from .graspnet_server import serve
                serve(config['graspnet'],events)
                return 0
            if args.action=='visualize':
                if config.get('grasp_backend')=='graspnet':
                    from .graspnet_visualize import render
                else:
                    from .report import render
                result=render(config,args.input,out,args.urdf);events.emit(result['state'],'offline visualization exported')
            elif args.action=='check':
                result=dict(events.emit('CHECK_OK','offline configuration/dependencies/provenance verified',dependencies=dependency_report()),grasp_confirmed=False)
            elif args.input:
                result=preview_input(config,args.input,out);events.emit(result['state'],result['reason'])
            else:
                with PiperSession(config) as session:
                    if args.action=='run':result=run_once(config,session,vision.capture_rgbd,out)
                    elif args.action=='zero':result=run_zero(config,session,out)
                    elif args.action=='observe':
                        acquire(config,session,vision.capture_rgbd,events,'observe')
                        result=dict(events.emit('OBSERVED','finite capture complete'),can_transmit_count=0,grasp_confirmed=False)
                    else:
                        prepare_plan(config,session,vision.capture_rgbd,events)
                        result=dict(events.emit('PREVIEW_READY','read-only capture and pure planning'),can_transmit_count=0,grasp_confirmed=False)
            save(out/'result.json',result)
            return 0 if result['state'] in ('CHECK_OK','PREVIEW_READY','OBSERVED','GRASP_CONFIRMED','GRASP_RETURNED_ZERO','VISUALIZED','ZERO_REACHED') else 1
        except KeyboardInterrupt:
            save(out/'result.json',dict(state='EXITED',grasp_confirmed=False,stop_confirmed=None));return 130
        except Exception as error:
            result=dict(state='CHECK_FAILED' if args.action=='check' else 'WAIT_CONFIG',reason=str(error),grasp_confirmed=False)
            save(out/'result.json',result)
            with log.open('a') as stream:stream.write(json.dumps(result)+'\n')
            return 1


if __name__=='__main__':raise SystemExit(main())
