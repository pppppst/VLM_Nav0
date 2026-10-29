"""Asynchronous preparation and task-scoped handshake, with no ROS or hardware."""
from dataclasses import replace

import pytest

from multi_agent_nav.mock import mock_config, make_machine, make_coordinator
from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.models import AgentState, Command, Health, Pose2D


def setup(robot='go2'):
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    config['scene']['resources'] = {}
    config['scene']['exit_points'] = {}
    agent, _ = make_machine(config, robot, True)
    agent.local_execution_enabled = True
    health = FakeAdapter(robot).get_health()
    agent.tick(0., health)
    return config, agent, health


def peer(config, robot='ranger', **updates):
    identity = config['alignment']['robots'][robot]
    state = AgentState(robot_id=robot, local_map_id=identity['local_map_id'],
        map_version=identity['map_version'], common_frame_id='', alignment_id='',
        simulation=True, localization_valid=True, nav_ready=True, stopped=True,
        run_state='WAITING_PEER', stamp=1.)
    return replace(state, **updates)


def sync(robot='go2', task_id='task', phase='SYNCING'):
    return Command(task_id=task_id, recipient_id=robot, common_frame_id='', alignment_id='',
                   simulation=True, action='SYNC_LOCAL_C', phase=phase)


def activate(robot='go2'):
    config, agent, health = setup(robot)
    assert agent.accept(sync(robot), 0.).accepted
    other = 'ranger' if robot == 'go2' else 'go2'
    agent.receive_peer(peer(config, other, task_id='task', run_state='SYNCING'), .1)
    agent.tick(.1, health)
    assert agent.state().run_state == 'SYNCED'
    assert agent.accept(sync(robot, phase='SEARCH_AB'), .2).accepted
    command = replace(sync(robot, phase='SEARCH_AB'), action='EXECUTE', command_id='command',
        sequence=1, kind='APPROACH_PICKUP', target_id='A' if robot == 'go2' else 'B')
    assert agent.accept(command, .2).accepted
    agent.take_effects()
    return config, agent, health


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
def test_local_preparation_waits_sixty_seconds_without_a_peer(robot):
    _, agent, health = setup(robot)
    agent.tick(.1, replace(health, nav_ready=False))
    assert agent.state().run_state == 'PREPARING'
    for now in (1., 30., 60.):
        assert agent.tick(now, health) == []
        assert agent.state().run_state == 'WAITING_PEER'
        assert agent.state().nav_ready and not agent.paused
        assert not agent.state().ready


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
def test_online_unready_and_peer_restart_do_not_latch_pause(robot):
    config, agent, health = setup(robot)
    other = 'ranger' if robot == 'go2' else 'go2'
    for now in (1., 30., 60.):
        agent.receive_peer(peer(config, other, nav_ready=False, run_state='PREPARING'), now)
        agent.tick(now, health)
        assert agent.peer_received == now
        assert agent.state().run_state == 'WAITING_PEER' and not agent.paused
    agent.receive_peer(peer(config, other), 61.)
    agent.tick(61., health)
    assert agent.state().run_state == 'STANDBY'
    agent.receive_peer(peer(config, other, nav_ready=False, run_state='PREPARING'), 62.)
    agent.tick(62., health)
    assert agent.state().run_state == 'WAITING_PEER'
    agent.tick(122., health)
    assert agent.state().run_state == 'WAITING_PEER' and not agent.paused
    agent.receive_peer(peer(config, other), 123.)
    agent.tick(123., health)
    assert agent.state().run_state == 'STANDBY'


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
def test_accepting_sync_publishes_task_identity_before_peer_confirmation(robot):
    config, agent, health = setup(robot)
    assert agent.accept(sync(robot), 0.).accepted
    state = agent.state()
    assert state.task_id == 'task' and state.run_state == 'SYNCING'
    assert not state.ready and not state.command_id and not state.goal_id
    for now in (1., 30., 60.):
        assert agent.accept(sync(robot), now).accepted
        agent.tick(now, health)
        assert agent.state().run_state == 'SYNCING' and not agent.paused
    other = 'ranger' if robot == 'go2' else 'go2'
    agent.receive_peer(peer(config, other, task_id='task', run_state='SYNCING'), 60.)
    agent.tick(60., health)
    assert agent.state().run_state == 'SYNCED' and not agent.state().ready


@pytest.mark.parametrize('updates', [
    dict(task_id='old'), dict(command_id='old-command'), dict(goal_id='old-goal'),
    dict(ready=True), dict(deposited=True), dict(exited=True), dict(sequence=3), dict(reply='SUCCEEDED'),
])
def test_old_task_command_goal_or_acceptance_flags_cannot_complete_handshake(updates):
    config, agent, health = setup()
    assert agent.accept(sync(), 0.).accepted
    message = peer(config, task_id='task', run_state='SYNCED', reply='SYNCED')
    agent.receive_peer(replace(message, **updates), .1)
    agent.tick(.1, health)
    assert agent.state().run_state == 'SYNCING'
    command = replace(sync(phase='SEARCH_AB'), action='EXECUTE', kind='APPROACH_PICKUP',
                      target_id='A', command_id='command', sequence=1)
    assert not agent.accept(command, .1).accepted
    assert agent.take_effects() == []


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
@pytest.mark.parametrize('failure', ['peer_unready', 'peer_timeout', 'sync_timeout', 'local_unready'])
def test_transient_execution_failures_stop_then_resume_same_command(robot, failure):
    config, agent, health = activate(robot)
    other = 'ranger' if robot == 'go2' else 'go2'
    now = 4.
    if failure != 'sync_timeout':
        agent.accept(sync(robot, phase='SEARCH_AB'), now)
    if failure != 'peer_timeout':
        agent.receive_peer(peer(config, other, task_id='task', run_state='NAVIGATING',
            nav_ready=failure != 'peer_unready'), now)
    local_health = replace(health, nav_ready=False) if failure == 'local_unready' else health
    effects = agent.tick(now, local_health)
    assert agent.paused and any(e.kind == 'STOP' for e in effects)
    assert agent.state().run_state == 'PAUSED'
    assert ('heartbeat' in agent.state().reason) == (failure in ('peer_timeout', 'sync_timeout'))
    agent.accept(sync(robot, phase='SEARCH_AB'), now+.1)
    agent.receive_peer(peer(config, other, task_id='task', run_state='SYNCED', reply='SYNCED'), now+.1)
    command = agent.command
    effects = agent.tick(now+.1, health)
    assert not agent.paused and agent.state().run_state == 'PLANNING'
    assert agent.command is command and any(e.kind == 'LOCAL_EXECUTION' for e in effects)


def preparing_coordinator():
    config, _, _ = setup()
    coordinator = make_coordinator(config, True)
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot), 0.)
    assert coordinator.start(config['scene']['scenario_id'], 0.).accepted
    return config, coordinator


def test_coordinator_waits_for_explicit_synced_ack_from_both_robots():
    config, coordinator = preparing_coordinator()
    assert coordinator.phase == 'SYNCING'
    for now in (1., 30., 60.):
        for robot in ('go2', 'ranger'):
            coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id,
                run_state='SYNCING', nav_ready=robot != 'ranger'), now)
        output = coordinator.tick(now)
        assert output and all(c.action == 'SYNC_LOCAL_C' for c in output)
        assert coordinator.phase == 'SYNCING' and not coordinator.commands
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id,
                                      run_state='SYNCED', reply='SYNCED'), 61.)
    output = coordinator.tick(61.)
    assert coordinator.phase == 'SEARCH_AB'
    assert {c.recipient_id for c in output if c.action == 'EXECUTE'} == {'go2', 'ranger'}


@pytest.mark.parametrize('updates', [
    dict(task_id='old'), dict(run_state='SYNCING'), dict(command_id='old-command'),
    dict(goal_id='old-goal'), dict(ready=True), dict(deposited=True), dict(sequence=8),
])
def test_coordinator_rejects_stale_or_partial_handshake(updates):
    config, coordinator = preparing_coordinator()
    for robot in ('go2', 'ranger'):
        state = peer(config, robot, task_id=coordinator.task_id, run_state='SYNCED', reply='SYNCED')
        coordinator.ingest_state(replace(state, **updates) if robot == 'ranger' else state, .1)
    assert not any(c.action == 'EXECUTE' for c in coordinator.tick(.1))
    assert coordinator.phase == 'SYNCING'


def test_coordinator_upgrades_transient_health_to_unknown_peer_pause():
    config, coordinator = preparing_coordinator()
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id, run_state='SYNCED', reply='SYNCED'), .1)
    coordinator.tick(.1)
    coordinator.ingest_state(peer(config, 'go2', task_id=coordinator.task_id,
                                  run_state='PLANNING', nav_ready=False), .2)
    coordinator.ingest_state(peer(config, 'ranger', task_id=coordinator.task_id,
                                  run_state='PAUSED'), .2)
    coordinator.tick(.2)
    assert coordinator.phase == 'PAUSED'
    assert coordinator.reason == 'ranger: agent paused: '
    assert coordinator.resume_phase is None
    coordinator.pause('secondary cancellation')
    assert coordinator.reason == 'ranger: agent paused: '


@pytest.mark.parametrize('first_robot', ['go2', 'ranger'])
def test_completed_handshake_survives_asymmetric_execute_delivery(first_robot):
    _, first, first_health = setup(first_robot)
    other = 'ranger' if first_robot == 'go2' else 'go2'
    _, second, second_health = setup(other)
    for agent in (first, second):
        assert agent.accept(sync(agent.context.robot_id), 0.).accepted
    first.receive_peer(second.state(), .1)
    second.receive_peer(first.state(), .1)
    first.tick(.1, first_health)
    second.tick(.1, second_health)
    for agent in (first, second):
        assert agent.state().run_state == 'SYNCED'
        assert agent.accept(sync(agent.context.robot_id, phase='SEARCH_AB'), .2).accepted
    def execute(agent):
        robot = agent.context.robot_id
        return replace(sync(robot, phase='SEARCH_AB'), action='EXECUTE', sequence=1,
            command_id=robot+'/command', kind='APPROACH_PICKUP', target_id='A' if robot == 'go2' else 'B')
    assert first.accept(execute(first), .2).accepted
    second.receive_peer(first.state(), .3)
    second.tick(.3, second_health)
    assert second.state().run_state == 'SYNCED'
    assert second.accept(execute(second), .3).accepted


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
def test_restarted_agent_rejoins_current_handshake_after_sixty_seconds(robot):
    from test_independent_c import independent_mission
    from multi_agent_nav.runner import AgentRunner
    mission = independent_mission()
    assert mission.start().accepted
    mission.step()  # SYNC accepted; no EXECUTE yet.
    mission.online[robot] = False
    mission.run(600)
    assert mission.coordinator.phase == 'SYNCING' and not mission.executed
    assert all(not agent.paused for agent in mission.machines.values())
    machine, _ = make_machine(mission.config, robot, True)
    machine.tick(mission.now, mission.adapters[robot].get_health())
    mission.machines[robot] = machine
    mission.runners[robot] = AgentRunner(machine, mission.adapters[robot], mission.config['scene']['targets'],
        lambda obs:mission.coordinator.ingest_observation(obs, mission.now),
        clock=lambda:mission.now, config=mission.config)
    mission.online[robot] = True
    mission.run(300)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert all(agent.context.task_id == mission.coordinator.task_id for agent in mission.machines.values())
