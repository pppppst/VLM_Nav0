"""Initial camera decision before any exploration point; synthetic inputs only."""
from concurrent.futures import Future
from dataclasses import replace

import pytest

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.models import Observation, Pose2D
from multi_agent_nav.go2_exploration import ExplorationStopped, explore_steps
from test_go2_exploration import LocalFixture, run_case
from test_generated_candidates import generated_config


@pytest.mark.parametrize('status,located,outcome', [
    ('DETECTED', True, 'TARGET_APPROACHED'),
    ('DETECTED', False, 'TARGET_DETECTED_UNLOCATED'),
    ('NOT_DETECTED', False, 'EXHAUSTED'),
    ('UNCERTAIN', False, 'EXHAUSTED'),
])
def test_initial_view_precedes_selection_and_only_found_target_skips_it(status, located, outcome):
    adapter=LocalFixture(); events=[]; scans=[]; observed=[]
    def scan(descriptions, scan_id, *, stop_on_target=None, current_view_only=False,task_id=''):
        scans.append(current_view_only)
        if current_view_only:
            assert not adapter.calls and not any(e['state']=='SELECTED' for e in events)
            return completed([Observation(target_id='A',scan_id=scan_id,status=status,
                position_valid=located,common_position=Pose2D(5.,2.))])
        return completed([Observation(target_id='A',scan_id=scan_id,status='NOT_DETECTED')])
    adapter.start_scan=scan
    assert run_case(adapter,events,approach=True,initial_camera_check=True,
        require_navigation_success=True,on_observations=observed.extend)==outcome
    assert scans[0] and scans.count(True)==1 and observed
    if status=='DETECTED':
        assert not any(e['state']=='SELECTED' for e in events)
        assert [c[0] for c in adapter.calls]==(['PICKUP_NAV'] if located else [])
        assert any(e['state']=='PATH_PLANNED' and e['path'] for e in events)==located
    else:
        assert [c[0] for c in adapter.calls]==['VIEWPOINT_NAV']*3
        assert len(scans)==4
        assert len([e for e in events if e['state']=='PATH_PLANNED' and e['path']])==3


@pytest.mark.parametrize('fault', ['api_error','missing','exception','moving'])
def test_initial_camera_fault_does_not_turn_into_exploration(fault):
    adapter=LocalFixture(); events=[]; scans=[]
    def scan(*args, **kwargs):
        scans.append(kwargs)
        if fault=='exception':
            reply=Future();reply.set_exception(RuntimeError('synthetic API failure'));return reply
        return completed([] if fault=='missing' else [Observation(target_id='A',status='API_ERROR')])
    adapter.start_scan=scan
    if fault=='moving':
        original=adapter.get_health
        adapter.get_health=lambda:replace(original(),stopped=False)
    with pytest.raises(ExplorationStopped):
        run_case(adapter,events,initial_camera_check=True)
    assert not adapter.calls and not any(e['state']=='SELECTED' for e in events)
    assert bool(scans)==(fault!='moving')


@pytest.mark.parametrize('interrupted', ['cancel','health','deadline'])
def test_initial_camera_pending_reply_keeps_executor_responsive_and_cannot_move_late(interrupted):
    from multi_agent_nav.frame_adapter import LocalFrame
    adapter=LocalFixture();pending=Future();events=[];now=[0.]
    adapter.start_scan=lambda *args,**kwargs:pending
    config=dict(scene=dict(targets={'A':'target'}),robot_go2=dict(exploration_strategy='fixed_viewpoints'))
    steps=explore_steps(adapter,config,LocalFrame(),10.,'target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0],initial_camera_check=True)
    next(steps)
    for _ in range(5):next(steps)
    assert not pending.done() and not adapter.calls
    if interrupted=='cancel':steps.close()
    else:
        if interrupted=='health':adapter.online=False
        else:now[0]=11.
        with pytest.raises(ExplorationStopped):next(steps)
    assert pending.cancelled() and not adapter.calls


@pytest.mark.parametrize('robot,simulation,kind,expected', [
    ('go2',False,'APPROACH_PICKUP',True),
    ('go2',False,'SEARCH_VIEWPOINT',True),
    ('go2',False,'APPROACH_DEPOSIT',False),
    ('go2',True,'APPROACH_PICKUP',False),
    ('ranger',False,'APPROACH_PICKUP',False),
])
def test_runner_enables_initial_camera_only_for_live_go2_search(monkeypatch,tmp_path,robot,simulation,kind,expected):
    from types import SimpleNamespace as S
    from multi_agent_nav.runner import AgentRunner
    import multi_agent_nav.go2_exploration as exploration
    captured=[];display_events=[]
    def explore(*args,**kwargs):
        captured.append(kwargs)
        args[5](dict(state='PATH_PLANNED',path=[]))
    monkeypatch.setattr(exploration,'explore_steps',explore)
    runner=AgentRunner.__new__(AgentRunner)
    command=S(kind=kind,target_id='A',command_id='command',task_id='task',observation_id='cached')
    runner.machine=S(command=command,context=S(robot_id=robot),frame=None,task_started=0.,
        success_radius=1.,local_c_observations={},single_robot_mode=robot=='go2' and not simulation)
    runner.config={'scene':{'task_timeout':600.},'robot_'+robot:{'experiment_records_root':str(tmp_path)}}
    runner.adapter=S(simulation=simulation)
    runner.on_event=display_events.append
    runner.clock=lambda:0.;runner.descriptions={'A':'target'};runner.owners=lambda:[]
    runner.start_local_execution()
    assert captured[0]['initial_camera_check'] is expected
    assert (captured[0]['approaching'] is not None) == (robot=='go2' and not simulation)
    assert display_events==[dict(state='DISPLAY_RESET',task_id='task'),dict(state='PATH_PLANNED',path=[])]


@pytest.mark.parametrize('found', [False, True])
def test_generated_candidates_are_only_requested_after_negative_initial_image(generated_config,tmp_path,found):
    from pathlib import Path
    from test_generated_candidates import run_generated
    generated_config['robot_go2'].update(approach_on_detection=True,standoff_distance=.9,
        approach_distance_margin=.03,arrival_config_files=[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')])
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,
        'found' if found else 'normal',initial_camera_check=True)
    names=[e['state'] for e in events]
    assert names[0]=='INITIAL_CAMERA_START'
    if found:
        assert outcome=='TARGET_APPROACHED' and not requests
        assert 'CANDIDATE_GEOMETRY' not in names and [c[0] for c in adapter.calls]==['PICKUP_NAV']
    else:
        assert requests and outcome=='EXHAUSTED'
        assert names.index('INITIAL_CAMERA_RESULT')<names.index('CANDIDATE_GEOMETRY')<names.index('CANDIDATE_REQUEST')


def test_live_go2_branch_completes_private_task_using_initial_image_and_cached_c(tmp_path):
    from test_local_tasks import private_mission
    mission=private_mission()
    adapter=mission.adapters['go2']
    # Exercise the live branch with the existing synthetic adapter, without ROS or API access.
    adapter.simulation=False;adapter.pose=Pose2D(-5.,-3.)
    mission.config['robot_go2']['experiment_records_root']=str(tmp_path)
    adapter.client.choose_candidate=lambda *a:pytest.fail('selected exploration points despite visible A/C')
    scans=[];original=adapter.start_scan
    def scan(*args,**kwargs):
        scans.append(kwargs.get('current_view_only',False))
        return original(*args,**kwargs)
    adapter.start_scan=scan
    assert mission.start().accepted
    mission.run(300)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert scans==[True]
    assert [c[0] for c in adapter.calls]==['PICKUP_NAV','DEPOSIT_NAV']
