"""Run on the PiPER computer with agent_release.py in the same private directory."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

bundle = Path.home()/'piper-vision-current'
sys.path.insert(0, str(bundle/'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from piper_vision_grasp import geometry
from piper_vision_grasp.main import Events, save, wait_snapshot, wait_joints, wait_gripper, send_once
from piper_vision_grasp.piper_io import PiperSession
import agent_release


def load_route(source, config):
    def read(name):
        return json.loads((source/name).read_text())
    result = read('result.json')
    if not result.get('returned_to_sensor_zero') or result.get('automatic_release') is not False:
        raise ValueError('source grasp did not finish holding its close command at sensor zero')
    saved = read('config_snapshot.json')
    for field in ('usb_serial', 'kinematics', 'extrinsics'):
        if saved[field] != config[field]:
            raise ValueError('current grasp configuration differs: '+field)
    if saved['semantic']['tcp_offset_m'] != config['semantic']['tcp_offset_m']:
        raise ValueError('tool offset changed')
    events = [json.loads(line) for line in (source/'events.jsonl').read_text().splitlines()]
    reached = {r['stage']: r for r in events if r['state']=='AT_TARGET'}
    route = read('return-plan.json')['stages']
    lift = reached['lift']['measured']['joints_deg']
    points = [lift]
    for stage in route:
        q = stage['angles_deg']
        event = reached[stage['stage_id']]
        if event['target'] != q or not np.allclose(q,event['measured']['joints_deg'],rtol=0,atol=config['motion']['joint_tolerance_deg']):
            raise ValueError('source return point was not reached')
        points.append(q)
    if len(points)<2 or np.any(points[-1]) or not np.isfinite(points).all():
        raise ValueError('invalid return path')
    for q in points:
        geometry.fk(q,config['kinematics'])
    return points


def execute(config, session, events, points):
    tolerance = config['motion']['joint_tolerance_deg']
    start = wait_snapshot(config, session, events, 'before-place')
    if not np.allclose(start['joints_deg'],points[-1],rtol=0,atol=tolerance):
        raise ValueError('current joints differ from start of recorded reverse path')
    def move(stage, q):
        send_once(config, session, events, stage, 'joint', q)
        return wait_joints(config, session, events, dict(stage_id=stage,angles_deg=q))

    for i,q in enumerate(reversed(points[:-1])):
        at_lift = move('to-lift-'+str(i+1), q)
    def open_at_lift():
        now = wait_snapshot(config, session, events, 'before-release')
        if not np.allclose(now['joints_deg'],points[0],rtol=0,atol=tolerance):
            raise ValueError('measured joints moved away from the lift point before release')
        send_once(config, session, events, 'open', 'gripper', config['gripper']['open_mm'])
        return wait_gripper(config, session, events, 'open')
    agent_release.handle_arm = SimpleNamespace(open_gripper=open_at_lift)
    agent_release.release_gripper(SimpleNamespace(arm_used={'right':True}))
    opened = events.last['measured']['gripper_width_mm']
    for i,q in enumerate(points[1:]):
        final = move('to-zero-'+str(i+1), q)
    return dict(state='OPENED_AT_LIFT_RETURNED_ZERO',lift_sensor_deg=at_lift['joints_deg'],
                open_width_mm=opened,final_sensor_deg=final['joints_deg'],
                final_width_mm=final['gripper_width_mm'],object_landing_verified=False)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('grasp_id',nargs='?')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    root=Path.home()/'piper-r4-grasp-evidence'
    if args.grasp_id:
        if Path(args.grasp_id).name != args.grasp_id or args.grasp_id in ('.','..'):
            parser.error('invalid grasp id')
        source=root/args.grasp_id
    else:
        source=max(root.glob('*/cycle.json'),key=lambda p:p.stat().st_mtime).parent
    config=geometry.load_config(bundle/'config/piper_vision_grasp/semantic-camera-avoid.json')
    points=load_route(source,config)
    if args.check:
        print(json.dumps(dict(state='CHECK_OK',source=source.name,lift_sensor_deg=points[0],legs=len(points)-1)))
        return
    output=Path.home()/'piper-place-runs'/source.name
    output.mkdir(parents=True,exist_ok=False)
    print('机载诊断日志：'+str(output/'events.jsonl'),flush=True)
    print('机载结果：'+str(output/'result.json'),flush=True)
    config.update(run_id='place-'+source.name,run_dir=str(output),readonly=False)
    events=Events(config,output)
    events.emit('STARTING','reverse verified return path, release once at recorded lift, return to sensor zero')
    try:
        with PiperSession(config) as session:
            result=execute(config,session,events,points)
    except (Exception,KeyboardInterrupt) as exc:
        result=dict(state='INTERRUPTED',reason=str(exc) or type(exc).__name__,last_stage=events.last.get('stage'))
        save(output/'result.json',result)
        raise
    result['source_grasp_id'] = source.name
    save(output/'result.json',result)

if __name__=='__main__':
    main()
