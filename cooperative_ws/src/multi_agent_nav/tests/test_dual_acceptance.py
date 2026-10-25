"""Offline checks of both robots' arrival gates and asymmetric C discovery."""
from concurrent.futures import Future
from dataclasses import replace

import pytest

from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.mock import make_machine, mock_config
from multi_agent_nav.models import Command, CommandReply, GoalResult, Observation, Pose2D
from test_independent_c import independent_mission


def legacy_approach(robot, kind):
    agent, frame = make_machine(mock_config(), robot, True)
    agent.tick(0, FakeAdapter(robot).get_health())
    sync = Command(task_id='t', recipient_id=robot, alignment_id='SYNTHETIC_v1',
                   action='SYNC', phase='DEPOSIT' if kind == 'APPROACH_DEPOSIT' else 'SEARCH_AB')
    assert agent.accept(sync, 0).accepted
    agent.peer_received = 0
    agent._state.ready = kind == 'APPROACH_DEPOSIT'
    command = replace(sync, action='EXECUTE', command_id='c', sequence=1, kind=kind,
        target_id='C' if kind == 'APPROACH_DEPOSIT' else 'A' if robot == 'go2' else 'B',
        object_position=Pose2D(2, 0), position_valid=True)
    assert agent.accept(command, 0).accepted
    agent.take_effects()
    goal_kind = 'DEPOSIT_NAV' if kind == 'APPROACH_DEPOSIT' else 'PICKUP_NAV'
    agent.navigation_started('goal', goal_kind)
    return agent, frame.to_local(command.object_position), GoalResult('t', 'c', 'goal', goal_kind, 'SUCCEEDED')


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
@pytest.mark.parametrize('kind', ['APPROACH_PICKUP', 'APPROACH_DEPOSIT'])
@pytest.mark.parametrize('distance,stopped,terminal', [
    (1.01, True, True), (0.9, False, True), (0.9, True, False),
    (float('nan'), True, True), (1.0, True, True),
])
def test_legacy_success_waits_for_physical_arrival(robot, kind, distance, stopped, terminal):
    agent, target, result = legacy_approach(robot, kind)
    flag = 'ready' if kind == 'APPROACH_PICKUP' else 'deposited'
    agent.on_goal_result(result, .1)
    assert not getattr(agent.state(), flag)  # An action result alone cannot certify arrival.
    health = replace(agent.health, pose=Pose2D(target.x-distance, target.y),
                     stopped=stopped, motion_terminal=terminal)
    agent.tick(.2, health)
    assert getattr(agent.state(), flag) == (distance <= 1.0 and stopped and terminal)
    if not stopped or not terminal:
        agent.tick(.3, replace(health, stopped=True, motion_terminal=True))
        assert getattr(agent.state(), flag)


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
@pytest.mark.parametrize('kind,target', [
    ('APPROACH_PICKUP', None), ('SEARCH_VIEWPOINT', 'C'), ('APPROACH_DEPOSIT', 'C'),
])
def test_private_intent_without_local_executor_cannot_use_legacy_navigation(robot, kind, target):
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    agent, _ = make_machine(config, robot, True)
    agent.tick(0, FakeAdapter(robot).get_health())
    sync = Command(task_id='t', recipient_id=robot, action='SYNC_LOCAL_C',
        common_frame_id='', alignment_id='', phase='SYNCING')
    assert agent.accept(sync, 0).accepted
    from test_startup_handshake import peer
    agent.receive_peer(peer(config, 'ranger' if robot == 'go2' else 'go2',
        task_id='t', run_state='SYNCING'), 0)
    agent.tick(0, agent.health)
    sync = replace(sync, phase='SEARCH_AB')
    assert agent.accept(sync, 0).accepted
    if target == 'C':
        sync = replace(sync, phase='DEPOSIT')
        assert agent.accept(sync, 0).accepted
    agent._state.ready = target == 'C'
    agent.local_c_observations['own-C'] = Observation(task_id='t', target_id='C',
        observation_id='own-C', status='DETECTED', position_valid=True, common_position=Pose2D(9, 1))
    command = replace(sync, action='EXECUTE', command_id='c', sequence=1, kind=kind,
        target_id=target or ('A' if robot == 'go2' else 'B'), observation_id='own-C')
    assert agent.accept(command, 0).reason == 'local task executor required'
    assert not agent.take_effects() and agent.command is None


@pytest.mark.parametrize('finder', ['go2', 'ranger'])
def test_c_discovered_while_peer_searches_c_never_completes_peer(finder):
    mission = independent_mission()
    peer = 'ranger' if finder == 'go2' else 'go2'
    pending, work = Future(), []
    for robot, adapter in mission.adapters.items():
        del adapter.detections['C']
        scan = adapter.start_scan
        def delayed(descriptions, scan_id, *, stop_on_target=None, current_view_only=False, robot=robot, adapter=adapter, scan=scan):
            if stop_on_target == 'C':
                if robot == peer:
                    work.append((scan, descriptions, scan_id, stop_on_target))
                    return pending
                adapter.detections['C'] = Pose2D(9, 1)
            return scan(descriptions, scan_id, stop_on_target=stop_on_target, current_view_only=current_view_only)
        adapter.start_scan = delayed
    assert mission.start().accepted
    mission.run(120)
    assert work and mission.machines[finder].state().deposited
    assert not mission.machines[peer].state().deposited
    assert mission.coordinator.phase == 'DEPOSIT'
    assert not any(r == peer and c.kind == 'APPROACH_DEPOSIT' for r, c in mission.executed)
    mission.adapters[peer].detections['C'] = Pose2D(12, 4)
    scan, descriptions, scan_id, target = work[0]
    pending.set_result(scan(descriptions, scan_id, stop_on_target=target).result())
    mission.run(120)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert all(s.ready and s.deposited and s.stopped for s in mission.coordinator.states.values())
    assert all(c.observation_id.startswith(r + '/') for r, c in mission.executed
               if c.kind == 'APPROACH_DEPOSIT')


@pytest.mark.parametrize('drop_robots', [('go2',), ('ranger',), ('go2', 'ranger')])
def test_private_command_retry_and_duplicates_do_not_repeat_motion(drop_robots):
    mission = independent_mission()
    rejected = set()
    for robot, agent in mission.machines.items():
        accept = agent.accept
        def drop_first(command, now, robot=robot, accept=accept):
            if command.action == 'EXECUTE' and robot in drop_robots and robot not in rejected:
                rejected.add(robot)
                return CommandReply(False, 'test dropped first delivery')
            return accept(command, now)
        agent.accept = drop_first
    assert mission.start().accepted
    mission.run(10)
    mission.duplicate_commands = True
    mission.run(300)
    assert rejected == set(drop_robots) and mission.coordinator.phase == 'DONE'
    for adapter in mission.adapters.values():
        calls = [(kind, goal.x, goal.y, command) for kind, goal, command in adapter.calls]
        assert len(calls) == len(set(calls))
