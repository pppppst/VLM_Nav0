from dataclasses import replace
import pytest
from multi_agent_nav.models import *
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.agent import AgentMachine
from multi_agent_nav.adapters.fake import FakeAdapter

def machine():
    ctx = AgentContext('go2', 't', 'g', 'v', 'lab_ref', 'a', True)
    agent = AgentMachine(ctx, FrameAdapter('g', 'v', 'lab_ref', 'a', (0, 0, 0)))
    agent.accept(Command(task_id='t', recipient_id='go2', alignment_id='a', action='SYNC'), 0)
    agent.peer_received = 0
    agent.health=FakeAdapter('go2').get_health()
    return agent

def command(kind='APPROACH_PICKUP', sequence=1):
    return Command(task_id='t', command_id=f'c{sequence}', sequence=sequence,
                   recipient_id='go2', alignment_id='a', kind=kind, target_id='A',
                   common_goal=Pose2D(2, 0), object_position=Pose2D(2, 0), position_valid=True)

def navigate(agent, cmd):
    assert agent.accept(cmd, 0).accepted
    agent.take_effects()
    agent.on_path(PathResult(True, [Pose2D(), Pose2D(2, 0)]), Pose2D(2, 0), 0)
    agent.take_effects()
    agent.navigation_started('goal', 'PICKUP_NAV' if cmd.kind == 'APPROACH_PICKUP' else 'WAIT_NAV')
    return GoalResult('t', cmd.command_id, 'goal', agent.goal_kind, 'SUCCEEDED')

def test_current_goal_only_and_duplicate_execution():
    agent = machine()
    result = navigate(agent, command())
    agent.on_goal_result(replace(result, goal_id='stale'), 0)
    assert not agent.state().ready
    agent.on_goal_result(result, 0)
    assert not agent.state().ready
    agent.tick(0, replace(agent.health, pose=Pose2D(1, 0), stopped=True, motion_terminal=True))
    assert agent.state().ready
    assert agent.accept(command(), 0).accepted
    assert not agent.take_effects()

@pytest.mark.parametrize('kind', ['WAIT', 'SEARCH_VIEWPOINT'])
def test_internal_navigation_does_not_set_ready(kind):
    agent = machine()
    result = navigate(agent, command(kind))
    agent.on_goal_result(result, 0)
    assert not agent.state().ready

def test_pause_invalidates_late_success_and_requires_stopping():
    agent = machine()
    result = navigate(agent, command())
    pause = replace(command(), action='PAUSE')
    agent.accept(pause, .1)
    agent.on_goal_result(result, .2)
    assert not agent.state().ready
    assert not agent.accept(command(sequence=2), .3).accepted
    agent.tick(.4, Health(localization_valid=True, nav_ready=True, stopped=False, motion_terminal=True))
    assert agent.state().run_state == 'STOPPING'
    agent.tick(.5, Health(localization_valid=True, nav_ready=True, stopped=True, motion_terminal=True))
    assert agent.state().run_state == 'PAUSED'
    assert not agent.accept(command(sequence=2), .6).accepted  # explicit new task required

def test_unreachable_and_sync_loss_fail_without_ready():
    agent = machine()
    assert agent.accept(command(), 0).accepted
    agent.on_path(PathResult(False, reason='unreachable'), None, 0)
    assert agent.state().reply == 'FAILED'
    agent.tick(4, Health(localization_valid=True, nav_ready=True, stopped=True))
    assert agent.state().run_state == 'PAUSED'
    assert not agent.state().ready

def test_fake_has_no_ros_or_hardware_and_supports_cancel():
    fake = FakeAdapter('go2')
    goal = fake.start_navigation(Pose2D(2, 0), 't', 'c', 'PICKUP_NAV')
    fake.cancel_navigation(goal)
    fake.advance(1)
    assert fake.results.pop().status == 'CANCELED'
    assert fake.get_health().stopped

def test_deposit_cannot_execute_before_ready_and_phase_sync():
    agent=machine()
    cmd=replace(command('APPROACH_DEPOSIT'),target_id='C',phase='DEPOSIT')
    assert not agent.accept(cmd,0).accepted
    agent._state.ready=True
    assert not agent.accept(cmd,0).accepted
    agent.accept(replace(cmd,action='SYNC'),0)
    assert agent.accept(cmd,0).accepted

@pytest.fixture
def handshake():
    ctx = AgentContext('go2', '', 'g', 'v', 'lab_ref', 'a', True)
    agent = AgentMachine(ctx, FrameAdapter('g', 'v', 'lab_ref', 'a', (0, 0, 0)))
    health = Health(localization_valid=True, nav_ready=True, stopped=True)
    agent.tick(0, health)
    sync = replace(command(), action='SYNC')
    assert agent.accept(sync, 0).accepted
    return agent, sync, health

def test_handshake_waits_without_motion_until_peer_arrives(handshake):
    agent, sync, health = handshake
    assert not agent.tick(.1, health)
    assert not agent.paused
    assert not agent.accept(command(), .1).accepted
    assert not agent.take_effects()
    agent.peer_received = .2
    assert not agent.tick(.2, health)
    assert agent.accept(command(), .2).accepted
    assert agent.state().reply == 'ACCEPTED'

def test_repeated_sync_cannot_extend_missing_peer_handshake(handshake):
    agent, sync, health = handshake
    for now in (.1, 1., 2., 3.):
        assert agent.accept(sync, now).accepted
        assert not agent.tick(now, health)
        assert not agent.paused
    effects = agent.tick(3.01, health)
    assert agent.paused and agent.state().run_state == 'PAUSED'
    assert [effect.kind for effect in effects] == ['STOP']
    agent.peer_received = 3.02
    agent.accept(sync, 3.02)
    agent.tick(3.02, health)
    assert not agent.accept(command(), 3.02).accepted

@pytest.mark.parametrize('change', [
    {'localization_valid': False}, {'nav_ready': False},
])
def test_handshake_wait_does_not_relax_local_safety(handshake, change):
    agent, sync, health = handshake
    effects = agent.tick(.1, replace(health, **change))
    assert agent.paused
    assert [effect.kind for effect in effects] == ['STOP']

@pytest.mark.parametrize('change', [{'stopped': False}, {'motion_terminal': False}])
def test_handshake_waits_for_stop_without_starting_motion(handshake, change):
    agent, sync, health = handshake
    for now in (.1, 1., 3.):
        agent.accept(sync, now)
        assert not agent.tick(now, replace(health, **change))
        assert not agent.paused
        assert not agent.accept(command(), now).accepted
    assert agent.tick(3.01, replace(health, **change))
    assert agent.paused

def test_peer_loss_after_handshake_has_no_new_grace(handshake):
    agent, sync, health = handshake
    agent.peer_received = .1
    agent.tick(.1, health)
    agent.accept(sync, 3.)
    assert agent.tick(3.11, health)
    assert agent.paused

@pytest.mark.parametrize('pause_first', [False, True])
def test_new_task_resets_terminal_command_and_requires_new_peer(pause_first):
    agent = machine()
    old = command(sequence=4)
    result = navigate(agent, old)
    agent.on_goal_result(result, .1)
    health = Health(localization_valid=True, nav_ready=True, stopped=True)
    if pause_first:
        agent.accept(replace(old, action='PAUSE'), .1)
    agent.tick(.2, health)
    new_sync = replace(old, action='SYNC', task_id='new')
    assert agent.accept(new_sync, .3).accepted
    state = agent.state()
    assert (state.task_id, state.sequence, state.command_id, state.goal_id, state.reply, state.run_state) == (
        'new', 0, '', '', '', 'IDLE')
    assert not state.ready and not state.deposited and not state.exited
    assert agent.peer_received is None
    new_command = replace(command(), task_id='new', command_id='new/c1')
    assert not agent.accept(new_command, .3).accepted
    agent.on_goal_result(result, .3)
    assert not agent.state().ready
    assert not agent.accept(replace(old, action='SYNC'), .3).accepted
    agent.peer_received = .4
    assert agent.accept(new_command, .4).accepted
    assert agent.state().sequence == 1
