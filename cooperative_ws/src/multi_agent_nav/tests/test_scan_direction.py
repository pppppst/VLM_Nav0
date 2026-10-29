"""Whole Go2 sweeps alternate direction within one task; no hardware or API."""
from dataclasses import replace
import json
import math

import pytest

from multi_agent_nav.models import Pose2D
from vlm_nav.geometry import normalize_angle
from test_go2_depth_retry import scanning


def finish_scan(s, reply):
    for _ in range(100):
        if reply.done():return reply.result()
        if s.a.scan['phase']=='FRAME':s.frame(status='NOT_DETECTED')
        else:s.a.tick()
    pytest.fail('scan did not finish')


@pytest.mark.parametrize('initial_yaw',[0.,math.pi-.1,-math.pi+.1])
def test_three_sweeps_alternate_actual_spin_goals_across_yaw_wrap(scanning,initial_yaw):
    s=scanning
    s.a.config['scan_angles']=[-i*math.pi/4 for i in range(1,9)]
    original=s.a.get_health
    s.a.get_health=lambda:replace(original(),pose=Pose2D(0.,0.,normalize_angle(original().pose.yaw+initial_yaw)))
    for number,sign in [(1,-1),(2,1),(3,-1)]:
        began=len(s.motions)
        reply=s.a.start_scan({'A':'target','B':'other'},f'task/command/{number}',task_id='task')
        results=finish_scan(s,reply)
        assert s.motions[began:]==pytest.approx([sign*math.pi/4]*8)
        assert len(results)==16 and {o.view_index for o in results}==set(range(8))
    records=[json.loads(p.read_text()) for p in sorted(s.tmp.glob('vlm/*/metadata.json'))]
    assert [r['scan_number'] for r in records]==[1]*8+[2]*8+[3]*8
    assert [r['scan_direction'] for r in records]==['clockwise']*8+['counterclockwise']*8+['clockwise']*8


def test_initial_single_frame_does_not_count_and_new_task_restarts_clockwise(scanning):
    s=scanning;s.a.config['scan_angles']=[-math.pi/4]
    initial=s.a.start_scan({'A':'target','B':'other'},'initial',current_view_only=True,task_id='task1')
    finish_scan(s,initial)
    assert not s.motions
    for task,command,sign in [('task1','A',-1),('task1','C',1),('task2','A',-1)]:
        began=len(s.motions)
        reply=s.a.start_scan({'A':'target','B':'other'},command,task_id=task)
        finish_scan(s,reply)
        assert s.motions[began:]==pytest.approx([sign*math.pi/4])


def test_early_detection_depth_retries_and_busy_rejection_keep_sweep_parity(scanning):
    s=scanning;s.a.config['scan_angles']=[-math.pi/4,-math.pi/2]
    reply=s.a.start_scan({'A':'target','B':'other'},'first',task_id='task',stop_on_target='A')
    with pytest.raises(RuntimeError,match='motion not serial'):
        s.a.start_scan({'A':'target'},'rejected',task_id='task')
    s.a.tick();s.a.tick()
    s.frame(located=False)
    s.frame(located=True)
    assert reply.done() and len(s.motions)==1
    next_reply=s.a.start_scan({'A':'target','B':'other'},'second',task_id='task')
    finish_scan(s,next_reply)
    assert s.motions==pytest.approx([-math.pi/4,math.pi/4,math.pi/4])


def test_exploration_passes_same_task_identity_to_initial_and_all_sweeps():
    from test_go2_exploration import LocalFixture,run_case
    adapter=LocalFixture();calls=[];original=adapter.start_scan
    def scan(*args,**kwargs):
        calls.append((kwargs.get('task_id'),kwargs.get('current_view_only',False)))
        return original(*args,**kwargs)
    adapter.start_scan=scan
    assert run_case(adapter,[],task_id='same-task',initial_camera_check=True)=='EXHAUSTED'
    assert calls==[('same-task',True)]+[('same-task',False)]*3
