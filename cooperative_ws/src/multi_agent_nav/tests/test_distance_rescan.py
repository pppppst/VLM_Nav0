"""Reject a distance-only approach failure without completing the task."""
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.local_approach_execution import LocalApproachExecution
from multi_agent_nav.models import Observation, Pose2D
from test_generated_candidates import generated_config, run_generated
from test_go2_exploration import LocalFixture, run_case
from test_single_robot_approach_parity import Harness
from multi_agent_nav.single_robot_approach import SingleRobotApproach, BASELINE


@pytest.mark.parametrize('distance,terminal,stopped,recoverable', [
    (.82, True, True, True), (.81, True, True, False),
    (float('inf'), True, True, False), (.82, False, True, False),
    (.82, True, False, False),
])
def test_only_terminal_stopped_finite_distance_retry_is_recoverable(distance,terminal,stopped,recoverable):
    h=Harness(SingleRobotApproach,BASELINE)
    a=h.a;a.state='APPROACH_STOPPING';a.arrival_retry_count=1
    a.arrival_action_terminal=terminal;a.arrival_stop_started=h.now
    a.arrival_cancel_requested_at=h.now
    a.adapter.get_health=lambda:S(stopped=stopped)
    a.target_reference_distance=lambda pose:distance
    a.finish_approach_success=lambda:setattr(a,'state','SUCCEEDED')
    a.advance_approach_stop()
    assert getattr(a,'distance_rejected',False) is recoverable
    if recoverable:
        a.reason='outside';a.close=lambda:None
        execution=LocalApproachExecution.__new__(LocalApproachExecution)
        execution.approach=a;execution.adapter=a.adapter
        with pytest.raises(StopIteration) as result:next(execution.run(lambda:None))
        assert result.value.value=='APPROACH_DISTANCE_FAILED'
        assert a.adapter.approach is None


@pytest.mark.parametrize('origin',['initial','scan','cached_c'])
def test_distance_rejection_selects_new_observation_then_can_succeed(origin):
    adapter=LocalFixture();events=[];attempts=[]
    target='C' if origin=='cached_c' else 'A'
    def scan(descriptions,scan_id,**kwargs):
        return completed([Observation(target_id=target,scan_id=scan_id,observation_id='new',
            status='DETECTED',position_valid=True,common_position=Pose2D(5.,2.))])
    adapter.start_scan=scan
    def approach(*args):
        attempts.append(args)
        yield
        return 'APPROACH_DISTANCE_FAILED' if len(attempts)==1 else 'TARGET_APPROACHED'
    cached=Observation(target_id='C',observation_id='cached',status='DETECTED',
        position_valid=True,common_position=Pose2D(4.,2.)) if origin=='cached_c' else None
    assert run_case(adapter,events,approach=True,approaching=approach,target_id=target,
        initial_camera_check=origin=='initial',initial_observation=cached)=='TARGET_APPROACHED'
    selected=[e['viewpoint_id'] for e in events if e['state']=='SELECTED']
    assert selected==(['p1','p2'] if origin=='scan' else ['p1'])
    rejected=[e for e in events if e['state']=='TARGET_REJECTED']
    assert len(rejected)==1 and rejected[0]['target_id']==target
    assert len(attempts)==2


@pytest.mark.parametrize('initial',[True,False])
def test_generated_rescan_excludes_rejected_view_without_inventing_scan_coverage(generated_config,tmp_path,initial):
    attempts=[]
    def approach(*args):
        attempts.append(args)
        yield
        return 'APPROACH_DISTANCE_FAILED' if len(attempts)==1 else 'TARGET_APPROACHED'
    generated_config['robot_go2']['approach_on_detection']=True
    generated_config['robot_go2']['scan_angles']=[0.,1.]
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'found',
        initial_camera_check=initial,approaching=approach)
    assert outcome=='TARGET_APPROACHED' and len(attempts)==2
    assert len(requests)==(1 if initial else 2)
    if initial:
        names=[e['state'] for e in events]
        assert names.index('TARGET_REJECTED')<names.index('CANDIDATE_GEOMETRY')
    else:
        first=next(e['viewpoint_id'] for e in events if e['state']=='SELECTED')
        assert requests[1]['records'][first]['status']=='target_rejected'
        assert all(p['id']!=first for p in requests[1]['candidates'])
        assert requests[1]['history'][-1]['result']=='TARGET_REJECTED'
        assert not requests[1]['history'][-1]['observation_complete']


def test_other_approach_failure_still_ends_execution():
    def approach(*args):
        yield
        return 'APPROACH_FAILED: localization lost'
    adapter=LocalFixture();events=[]
    observation=Observation(target_id='A',status='DETECTED',position_valid=True,common_position=Pose2D(5.,2.))
    assert run_case(adapter,events,approach=True,initial_observation=observation,
        approaching=approach)=='APPROACH_FAILED: localization lost'
    assert not adapter.calls


def test_runner_discards_only_rejected_c_cache(monkeypatch,tmp_path):
    from multi_agent_nav.runner import AgentRunner
    import multi_agent_nav.go2_exploration as exploration
    def explore(*args,**kwargs):
        args[5](dict(state='TARGET_REJECTED',target_id='C',observation_id='rejected'))
    monkeypatch.setattr(exploration,'explore_steps',explore)
    runner=AgentRunner.__new__(AgentRunner)
    command=S(kind='APPROACH_DEPOSIT',target_id='C',command_id='command',task_id='task',observation_id='rejected')
    runner.machine=S(command=command,context=S(robot_id='go2'),frame=None,task_started=0.,
        success_radius=1.,local_c_observations={'rejected':object(),'other':object()},single_robot_mode=True)
    runner.config={'scene':{'task_timeout':600.},'robot_go2':{'experiment_records_root':str(tmp_path)}}
    runner.adapter=S(simulation=False);runner.on_event=None
    runner.clock=lambda:0.;runner.descriptions={'C':'bucket'};runner.owners=lambda:[]
    runner.start_local_execution()
    assert list(runner.machine.local_c_observations)==['other']
