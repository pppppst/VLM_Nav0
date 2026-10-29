"""Same real adapter/approach lifecycle, with or without cooperative ownership."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import Observation, Pose2D
from multi_agent_nav.adapters.fake import completed
from test_go2_executing_path import navigation, accept
from test_single_robot_approach_parity import live_stack


@pytest.fixture(params=['cooperative', 'standalone'])
def local_stack(request, live_stack):
    return configure_stack(live_stack,request.param)


def configure_stack(s,mode):
    import multi_agent_nav.go2_check as check
    from multi_agent_nav.frame_adapter import LocalFrame
    s.scans = []; s.outcome = None; s.owners = []
    s.mode = mode
    if s.mode == 'cooperative':
        return s
    assert hasattr(check, 'standalone_explore_steps'), 'standalone entry has no shared approach lifecycle'
    s.a.client = object()
    s.m.peer_received = s.m.peer_task_received = s.m.sync_received = None
    s.a.select_approach = lambda *a, **k: pytest.fail('legacy standoff approach reached')
    s.a.read_approach_parameters = lambda: pytest.fail('legacy approach budget reached')
    def scan(descriptions, scan_id, **kwargs):
        assert descriptions['A'] == 'explicit target'
        s.scans.append(kwargs)
        return completed([Observation(target_id='A', status='DETECTED', position_valid=True,
            common_position=Pose2D(2., 0.))])
    s.a.start_scan = scan
    s.r.config['robot_go2']['approach_on_detection'] = True
    steps = check.standalone_explore_steps(s.a, s.r.config, LocalFrame(), 610., 'explicit target',
        s.events.append, lambda:s.owners, lambda:True, lambda:None, clock=lambda:s.now[0])
    s.steps = steps
    def tick(dt=.05):
        s.now[0] += dt
        s.a.tick()
        try:next(steps)
        except StopIteration as result:s.outcome = result.value
    s.tick = tick
    return s


def success(s):
    return s.m.state().ready if s.mode == 'cooperative' else s.outcome == 'TARGET_APPROACHED'


@pytest.mark.parametrize('scenario', ['normal', 'inside', 'empty', 'plan_failed', 'aborted', 'overshoot'])
def test_both_wrappers_use_frozen_requests_and_outcomes(local_stack, scenario):
    s = local_stack
    if scenario=='overshoot' and s.mode=='standalone':
        # Keep subsequent observation goals blocked in this approach-only fixture.
        s.a.costmap_candidate_free=lambda *args,**kwargs:False
    if scenario == 'inside':s.health[0] = replace(s.health[0], pose=Pose2D(1.2, 0.))
    s.tick()
    if scenario != 'inside':
        goal, reply = s.plans[0]
        assert (goal.goal.pose.position.x, goal.goal.pose.position.y, goal.use_start) == (2., 0., False)
        if scenario == 'empty':
            reply.set_result(S(accepted=True,get_result_async=lambda:completed(S(status=4,result=S(path=S(poses=[]))))))
        else:s.planned(status=6 if scenario=='plan_failed' else 4)
        if scenario not in ('empty','plan_failed'):
            assert len(s.requests)==1
            assert next(iter(s.a.motions.values()))['target']==[2.,0.,.7]
            terminal=accept(s.requests[0])
            s.health[0]=replace(s.health[0],pose=Pose2D(1.18 if scenario=='overshoot' else 1.2,0.))
            terminal.set_result(S(status=6 if scenario=='aborted' else 4))
    for _ in range(3):s.odom()
    s.tick(.21);s.tick()
    if scenario=='overshoot':
        s.tick(.51);assert len(s.plans)==2
        for _ in range(3):s.odom()
        s.planned();terminal=accept(s.requests[1]);terminal.set_result(S(status=4))
        for _ in range(3):s.odom()
        s.tick(.21);s.tick()
    assert success(s)==(scenario in ('normal','inside'))
    assert len(s.requests)==(0 if scenario in ('inside','empty','plan_failed') else 2 if scenario=='overshoot' else 1)
    assert not s.a.results and s.a.approach is None
    if s.mode=='standalone':
        assert s.scans==[dict(stop_on_target='A',current_view_only=True,task_id=s.scans[0]['task_id'])]
        assert any(e.get('state')=='SELECTED' for e in s.events)==(scenario=='overshoot')
        assert any(e.get('state')=='TARGET_REJECTED' for e in s.events)==(scenario=='overshoot')


def test_success_event_uses_shared_stop_without_another_wait_or_motion(local_stack):
    s=local_stack;s.tick();s.planned();terminal=accept(s.requests[0])
    s.health[0]=replace(s.health[0],pose=Pose2D(1.2,0.),stopped=False)
    terminal.set_result(S(status=4))
    for _ in range(3):s.odom()
    for _ in range(5):s.tick(.21)
    assert not success(s)
    assert not any(e.get('state')=='TARGET_APPROACHED' for e in s.events)
    assert len(s.requests)==len(s.plans)==1
    s.health[0]=replace(s.health[0],stopped=True)
    for _ in range(3):s.odom()
    s.tick(.21);s.tick()
    assert success(s)
    assert len([e for e in s.events if e.get('state')=='TARGET_APPROACHED'])==1


@pytest.mark.parametrize('when', ['plan_pending','goal_pending','outer_wait'])
def test_cancel_fences_late_success_and_detaches_odom(local_stack,when):
    s=local_stack;s.tick()
    if when!='plan_pending':s.planned()
    if when=='outer_wait':
        terminal=accept(s.requests[0]);terminal.set_result(S(status=4))
        s.health[0]=replace(s.health[0],pose=Pose2D(1.2,0.),stopped=False)
        for _ in range(3):s.odom()
        s.tick(.21);s.tick()
    if s.mode=='cooperative':s.m.stop('external cancellation');s.tick()
    else:s.steps.close()
    if when=='plan_pending':
        cancelled=[]
        s.plans[0][1].set_result(S(accepted=True,cancel_goal_async=lambda:cancelled.append(True)))
        assert cancelled and not s.requests
    elif when=='goal_pending':
        terminal=accept(s.requests[0]);terminal.set_result(S(status=4))
    assert not success(s) and s.a.approach is None
    assert not any(e.get('state')=='TARGET_APPROACHED' for e in s.events)


def test_standalone_deadline_and_owner_guard_at_plan_callback(local_stack):
    s=local_stack;s.tick()
    if s.mode=='standalone':s.now[0]=611.
    else:s.r.owners=lambda:['another_owner']
    s.planned()
    assert not s.requests


@pytest.mark.parametrize('status,located,expected', [
    ('NOT_DETECTED',False,'EXHAUSTED'),('UNCERTAIN',False,'EXHAUSTED'),
    ('DETECTED',False,'EXHAUSTED'),('API_ERROR',False,None),
    ('missing',False,None),('exception',False,None)])
def test_standalone_initial_observation_keeps_cooperative_branches(status,located,expected):
    from multi_agent_nav.go2_check import standalone_explore_steps
    from multi_agent_nav.go2_exploration import ExplorationStopped
    from multi_agent_nav.frame_adapter import LocalFrame
    from test_go2_exploration import LocalFixture
    a=LocalFixture();events=[];scans=[];now=[10.]
    a.client=object()
    def scan(descriptions,scan_id,**kwargs):
        scans.append(kwargs.get('current_view_only',False))
        if status=='exception':
            f=Future();f.set_exception(RuntimeError('inference failed'));return f
        if status=='missing':return completed([])
        return completed([Observation(target_id=t,view_index=0,status=status if t=='A' and len(scans)==1 else 'NOT_DETECTED',
            position_valid=located) for t in descriptions])
    a.start_scan=scan
    config=dict(scene=dict(targets=dict(A='scene target',B='b',C='c'),viewpoints=dict(go2=[dict(id='p',pose=[1.,0.,0.])])),
        robot_go2=dict(approach_on_detection=True,scan_angles=[0.]))
    steps=standalone_explore_steps(a,config,LocalFrame(),30.,'explicit target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0])
    def run():
        for _ in range(500):
            try:next(steps)
            except StopIteration as r:return r.value
            now[0]+=.05;a.advance(now[0])
        pytest.fail('standalone stalled')
    if expected is None:
        with pytest.raises(ExplorationStopped):run()
    else:assert run()==expected
    assert scans[0]
    assert bool(a.calls)==(status in ('NOT_DETECTED','UNCERTAIN','DETECTED'))
    assert all(c[0]=='VIEWPOINT_NAV' for c in a.calls)


def test_standalone_scan_spin_has_local_ownership_without_a_task_id(live_stack):
    from multi_agent_nav.go2_check import standalone_explore_steps
    from multi_agent_nav.frame_adapter import LocalFrame
    s=live_stack;s.a.client=object()
    s.a.start_scan=lambda *a,**k:Future()
    steps=standalone_explore_steps(s.a,s.r.config,LocalFrame(),610.,'target',s.events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:s.now[0])
    next(steps)
    s.a.scan=dict(id='scan')
    assert s.a.motion_guard('', 'scan')
    assert not s.a.motion_guard('', 'unowned-scan')
    steps.close()
    assert not s.a.motion_guard('', 'scan')


@pytest.mark.parametrize('scenario',['normal','inside','empty','plan_failed','aborted'])
def test_local_decision_traces_match_between_wrappers(monkeypatch,scenario):
    traces=[]
    for mode in ('cooperative','standalone'):
        s=configure_stack(live_stack.__wrapped__(navigation.__wrapped__(monkeypatch),monkeypatch),mode)
        test_both_wrappers_use_frozen_requests_and_outcomes(s,scenario)
        events=[{k:v for k,v in e.items() if k not in ('task_id','command_id','goal_id')}
            for e in s.events if e.get('state','').startswith('APPROACH_') and e.get('state')!='APPROACH_PLANNING'
            or e.get('state')=='TARGET_APPROACHED']
        traces.append(dict(events=events,plans=[(g.goal.pose.position.x,g.goal.pose.position.y,
            g.goal.pose.orientation.z,g.use_start) for g,_ in s.plans],
            stops=len(s.zeros),requests=len(s.requests),success=success(s)))
    assert traces[0]==traces[1]


from test_semantic_search import catalog_config


@pytest.mark.parametrize('strategy',['fixed_viewpoints','semantic_furniture'])
def test_followup_scan_uses_actual_abc_client_after_negative_initial_view(catalog_config,strategy):
    import json
    import numpy as np
    from multi_agent_nav.adapters.go2 import ABCClient
    from multi_agent_nav.go2_check import standalone_explore_steps
    from multi_agent_nav.frame_adapter import LocalFrame
    from test_go2_exploration import LocalFixture
    config=catalog_config
    config['robot_go2']['exploration_strategy']=strategy
    config['scene']['targets']=dict(A='scene target',B='other target',C='deposit target')
    config['scene']['viewpoints']=dict(go2=[dict(id='p',pose=[1.,0.,0.])])
    client=ABCClient.__new__(ABCClient);client.mode='abc';client._is_qwen=False
    client._image_content=lambda rgb:dict(type='image_url',image_url=dict(url='offline'))
    grounding=dict(target_visible=False,object_match=False,qualifier_match=False,relation_match=False,
        confidence=0.,target_pixel=None,evidence_pixel=None)
    payload={t:dict(status='NOT_DETECTED',grounding=grounding) for t in 'ABC'}
    client._completion=lambda *a:S(choices=[S(message=S(content=json.dumps(payload)))])
    client.choose_furniture=lambda snapshot:dict(action='inspect_furniture',
        furniture_id=snapshot['candidates'][-1]['furniture_id'],reason='fixture')
    adapter=LocalFixture();adapter.client=client;adapter.vision=S(submit=lambda fn:completed(fn()))
    scans=[];now=[0.]
    def scan(descriptions,scan_id,**kwargs):
        parsed=client.infer_abc(np.zeros((5,5,3),dtype=np.uint8),descriptions)
        scans.append((kwargs.get('current_view_only',False),descriptions))
        return completed([Observation(target_id=t,status=status) for t,(status,_) in parsed.items()])
    adapter.start_scan=scan
    steps=standalone_explore_steps(adapter,config,LocalFrame(),10.,'explicit target',lambda e:None,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0])
    for _ in range(200):
        try:next(steps)
        except StopIteration as r:
            assert r.value=='EXHAUSTED'
            break
        now[0]+=.05;adapter.advance(now[0])
    else:pytest.fail('exploration stalled')
    assert scans[0][0] and len(scans)>1
    assert all(descriptions==dict(A='explicit target',B='other target',C='deposit target') for _,descriptions in scans)


def test_both_entries_continue_beyond_old_radius_and_still_accept_target(local_stack):
    s = local_stack
    s.tick(); s.planned(); terminal = accept(s.requests[0])
    s.health[0] = replace(s.health[0], pose=Pose2D(-3.6, 0.), stopped=False)
    s.tick(.21)
    assert s.a.approach is not None and s.a.approach.state == 'APPROACHING'
    assert not s.m.paused
    s.health[0] = replace(s.health[0], pose=Pose2D(1.2, 0.), stopped=True)
    terminal.set_result(S(status=4))
    for _ in range(3):s.odom()
    s.tick(.21); s.tick()
    assert success(s)
    assert len(s.plans) == len(s.requests) == 1


@pytest.mark.parametrize('resume',['healthy','cancel','timeout'])
def test_stopped_perception_wait_retains_plan_result_and_original_watchdog(live_stack,resume):
    s=configure_stack(live_stack,'standalone')
    s.tick()
    original=s.a.get_health
    waiting=[True]
    s.a.get_health=lambda:replace(original(),nav_ready=not waiting[0],stopped=True)
    s.a.perception_wait_allowed=lambda h:waiting[0] and h.stopped and h.motion_terminal
    approach=s.a.approach
    began=approach.goal_started
    if resume=='timeout':
        s.tick(120.01)
        assert approach.state=='FAILED'
        assert 'timed out' in approach.reason
    else:
        s.planned()
        assert approach.state=='APPROACHING' and not s.requests
        assert approach.goal_started==began
        if resume=='cancel':
            s.steps.close()
            waiting[0]=False
            approach.tick()
            assert not s.requests and approach.state=='FAILED'
        else:
            for _ in range(3):s.tick(.1)
            assert not s.requests
            waiting[0]=False
            s.tick(.21)
            assert len(s.requests)==1 and len(s.plans)==1


@pytest.mark.parametrize('mode',['standalone','cooperative'])
def test_completed_plan_can_wait_for_camera_beyond_action_timeout(live_stack,mode):
    s=configure_stack(live_stack,mode)
    s.tick()
    original=s.a.get_health
    s.a.get_health=lambda:replace(original(),stopped=True)
    waiting=[True]
    s.a.perception_wait_allowed=lambda h:waiting[0] and h.stopped and h.motion_terminal
    s.planned()
    approach=s.a.approach
    assert approach.pending_plan_result is not None and not s.requests
    s.tick(130.)
    assert approach.state=='APPROACHING' and not s.requests
    waiting[0]=False
    s.tick(.21)
    assert len(s.requests)==1 and len(s.plans)==1
