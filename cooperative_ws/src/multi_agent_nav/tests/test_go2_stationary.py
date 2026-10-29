"""One timed, fresh odometry decision shared by Go2 arrival and health; no ROS init."""
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import Pose2D
from multi_agent_nav.single_robot_approach import SingleRobotApproach, BASELINE
from test_go2_health_diagnostics import healthy_adapter
from test_field_preparation import stamp


@pytest.fixture
def stopped_stack(monkeypatch):
    now = [10.]
    monkeypatch.setattr('multi_agent_nav.adapters.go2.time.monotonic', lambda: now[0])
    a = healthy_adapter()
    a.config.update(stationary_linear_speed=.03, stationary_angular_speed=.05)
    a.odom_at = 0.; a.odom_stamp = None; a.stationary_since = None
    a.node.get_clock = lambda:S(now=lambda:S(nanoseconds=int(now[0]*1e9)))
    # Other health inputs stay healthy while testing odometry independently.
    a.costmap_transform = lambda *args, **kwargs: Pose2D()
    a.node.tf_buffer.lookup_transform = lambda *args:S(header=S(stamp=stamp(now[0])),
        transform=S(translation=S(x=0.,y=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    a.node.get_logger=lambda:S(info=lambda _:None, warn=lambda _:None, warning=lambda _:None)
    a.node.publish_stop=lambda:None
    a.node.make_pose_stamped=lambda *args:None
    approach=SingleRobotApproach(a, Pose2D(.8,0.), 'task','cmd','PICKUP_NAV',S(**BASELINE),lambda _:None,lambda:now[0])
    a.approach=approach
    approach.begin_approach_stop('test', action_terminal=True)
    def send(t, vx=.025, vy=0., wz=0., z=7., roll=8., message_time=None):
        now[0]=t
        a.on_odometry(S(header=S(stamp=stamp(t if message_time is None else message_time)),
            twist=S(twist=S(linear=S(x=vx,y=vy,z=z),angular=S(x=roll,y=9.,z=wz)))))
    return a, approach, now, send


@pytest.mark.parametrize('dt', [.05,.1,.25])
def test_same_stop_decision_requires_half_second_of_new_planar_samples(stopped_stack,dt):
    a,p,now,send=stopped_stack
    for i in range(round(.5/dt)):
        send(10.+i*dt)
        assert not a.get_health().stopped
        p.advance_approach_stop()
        assert p.state=='APPROACH_STOPPING'
    send(10.5)
    assert a.get_health().stopped
    p.advance_approach_stop()
    assert p.state=='SUCCEEDED'


@pytest.mark.parametrize('vx,wz,expected',[(.03,.05,True),(.030001,0.,False),(0.,.050001,False)])
def test_planar_threshold_boundaries(stopped_stack,vx,wz,expected):
    a,p,now,send=stopped_stack
    for t in (10.,10.25,10.5):send(t,vx=vx,wz=wz)
    assert a.get_health().stopped is expected
    p.advance_approach_stop()
    assert (p.state=='SUCCEEDED') is expected


@pytest.mark.parametrize('fault',['speed','nan','inf','stale','gap','old_stamp'])
def test_invalid_or_interrupted_odometry_restarts_duration(stopped_stack,fault):
    a,p,now,send=stopped_stack
    send(10.);send(10.25)
    if fault=='speed':send(10.3,vx=.04)
    elif fault=='nan':send(10.3,vx=float('nan'))
    elif fault=='inf':send(10.3,roll=float('inf'))
    elif fault=='stale':send(10.3,message_time=8.)
    elif fault=='old_stamp':send(10.3,message_time=10.1)
    else:now[0]=10.9
    start=now[0]+.05
    send(start);send(start+.25)
    assert not a.get_health().stopped
    p.advance_approach_stop();assert p.state=='APPROACH_STOPPING'
    send(start+.5)
    assert a.get_health().stopped


def test_three_fast_samples_waiting_and_duplicate_frames_are_not_stop_evidence(stopped_stack):
    a,p,now,send=stopped_stack
    send(10.);send(10.01);send(10.02)
    p.advance_approach_stop();assert p.state=='APPROACH_STOPPING'
    now[0]=10.5
    assert not a.get_health().stopped
    send(10.5,message_time=10.02)
    assert not a.get_health().stopped
    now[0]=10.6
    assert not a.get_health().stopped


@pytest.mark.parametrize('condition',['terminal','distance','no_odom','stale_query'])
def test_stop_alone_does_not_override_action_distance_or_freshness(stopped_stack,condition):
    a,p,now,send=stopped_stack
    if condition!='no_odom':
        for t in (10.,10.25,10.5):send(t)
    if condition=='terminal':p.arrival_action_terminal=False
    if condition=='distance':p.target_reference_position=(2.,0.)
    if condition=='stale_query':now[0]=11.01
    p.advance_approach_stop()
    assert p.state!='SUCCEEDED'
