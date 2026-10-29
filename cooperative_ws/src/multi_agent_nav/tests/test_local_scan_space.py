"""Local rotation coverage gates scans, never distant observation navigation."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S
import math

import pytest

from multi_agent_nav.models import Pose2D
from multi_agent_nav.go2_exploration import ExplorationStopped
from test_approach_costmap import maps
from test_go2_exploration import LocalFixture, run_case
from test_generated_candidates import generated_config, run_generated


@pytest.mark.parametrize('x,expected',[(0.,'free'),(1.2,'coverage_insufficient'),(3.,'coverage_insufficient')])
def test_local_disk_coverage_never_vetoes_far_global_candidate(x,expected):
    a,_,_=maps()
    assert a.observation_candidate_free(Pose2D(x,0.))
    assert a.local_rotation_check(Pose2D(x,0.))['status']==expected


@pytest.mark.parametrize('value,expected',[(253,'free'),(254,'collision'),(255,'coverage_insufficient')])
def test_local_rotation_checks_sides_without_conflating_unknown_and_obstacles(value,expected):
    a,local,_=maps()
    local.data[18*30+15]=value  # y=.35, outside parked body but inside its rotation disk.
    assert a.costmap_candidate_free(Pose2D())
    assert a.local_rotation_check(Pose2D())['status']==expected


def test_local_rotation_uses_tf_grid_rotation_and_freshness():
    a,local,_=maps()
    a.node.tf_buffer.lookup_transform=lambda *args:S(transform=S(translation=S(x=1.,y=0.),
        rotation=S(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4))))
    # map (0,1) -> odom (0,0); grid rotated around origin needs independent conversion.
    local.metadata.origin.position.x=1.5
    local.metadata.origin.orientation=S(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4))
    assert a.local_rotation_check(Pose2D(0.,1.))['status']=='free'
    local.data[18*30+15]=254
    assert a.local_rotation_check(Pose2D(0.,1.))['status']=='collision'
    a.node.get_clock=lambda:S(now=lambda:S(nanoseconds=12_000_000_000))
    assert a.local_rotation_check(Pose2D(0.,1.))['status']=='unavailable'


def test_scan_waits_at_arrival_for_coverage_then_resumes():
    adapter=LocalFixture();events=[];checks=[]
    def check(pose):
        assert adapter.calls and not adapter.active  # Never called during selection/navigation.
        checks.append(pose)
        return dict(status='coverage_insufficient' if len(checks)<4 else 'free',reason='fixture')
    adapter.local_rotation_check=check
    assert run_case(adapter,events)=='EXHAUSTED'
    assert adapter.scans==3
    wait=next(i for i,e in enumerate(events) if e['state']=='WAITING_SCAN_MAP')
    assert events[wait-1]['state']=='ARRIVED'
    assert any(e['state']=='SCAN_MAP_READY' for e in events[wait+1:])
    assert not any(e['state']=='SCAN_SPACE_BLOCKED' for e in events)


@pytest.mark.parametrize('fault',['deadline','health','moving','nonterminal'])
def test_wait_keeps_original_deadline_health_and_stop_guards(fault):
    adapter=LocalFixture();events=[]
    health=adapter.get_health
    def check(pose):
        if fault=='health':adapter.online=False
        if fault=='moving':adapter.get_health=lambda:replace(health(),stopped=False)
        if fault=='nonterminal':adapter.get_health=lambda:replace(health(),motion_terminal=False)
        return dict(status='coverage_insufficient',reason='fixture')
    adapter.local_rotation_check=check
    with pytest.raises(ExplorationStopped):run_case(adapter,events)
    assert len(adapter.calls)==1 and adapter.scans==0
    assert not any(e['state']=='SCAN_SPACE_BLOCKED' for e in events)


def test_confirmed_local_obstacle_is_skipped_without_completed_scan(generated_config,tmp_path,monkeypatch):
    checks=[]
    def check(adapter,pose):
        assert adapter.calls and not adapter.active
        checks.append(pose)
        return dict(status='collision',reason='fixture side obstacle')
    monkeypatch.setattr(LocalFixture,'local_rotation_check',check,raising=False)
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    assert checks and adapter.scans==0 and outcome=='WAITING_VIEWPOINTS'
    selected=[e['viewpoint_id'] for e in events if e.get('state')=='SELECTED']
    assert len(selected)==len(set(selected))
    assert any(e.get('observation_status')=='scan_blocked' for e in events)
    assert not any(e.get('observation_status') in ('observed','covered') for e in events)


@pytest.mark.parametrize('confirmed',[True,False])
def test_aborted_spin_only_recovers_with_confirmed_local_collision(confirmed):
    from multi_agent_nav.adapters.base import ScanSpinAborted
    adapter=LocalFixture();events=[];checks=[0]
    def check(pose):
        checks[0]+=1
        return dict(status='collision' if confirmed and checks[0]%2==0 else 'free',reason='fixture')
    adapter.local_rotation_check=check
    def scan(*args,**kwargs):
        reply=Future();reply.set_exception(ScanSpinAborted('scan spin aborted'));return reply
    adapter.start_scan=scan
    if confirmed:
        assert run_case(adapter,events)=='EXHAUSTED'
        assert len(adapter.calls)==3
    else:
        with pytest.raises(ExplorationStopped,match='scan failed'):run_case(adapter,events)
        assert len(adapter.calls)==1


from test_go2_perception_wait import waiting_scan


def test_scan_coverage_uses_pose_after_local_map_wait(generated_config,tmp_path,monkeypatch):
    poses=[]
    def check(adapter,pose):
        poses.append(pose)
        if len(poses)==1:
            adapter.pose=Pose2D(pose.x+.2,pose.y+.1,pose.yaw)
            return dict(status='coverage_insufficient',reason='rolling map update')
        return dict(status='free',reason='updated')
    monkeypatch.setattr(LocalFixture,'local_rotation_check',check)
    _,events,_,_=run_generated(generated_config,tmp_path)
    observed=next(e for e in events if e.get('observation_status')=='observed')
    recorded=observed['records'][observed['viewpoint_id']]['scan_pose']
    assert (recorded['x'],recorded['y'])==(poses[1].x,poses[1].y)


@pytest.mark.parametrize('status',['ABORTED','CANCELED'])
def test_only_aborted_spin_has_recovery_classification(waiting_scan,status):
    from multi_agent_nav.adapters.base import ScanSpinAborted
    from multi_agent_nav.adapters.fake import completed
    a,reply,now,stops,events=waiting_scan
    a.scan.update(current_view_only=False,phase='SPIN_WAIT',pending=completed(status))
    a.tick()
    assert reply.done() and isinstance(reply.exception(),RuntimeError)
    assert isinstance(reply.exception(),ScanSpinAborted)==(status=='ABORTED')
    assert a.scan is None
