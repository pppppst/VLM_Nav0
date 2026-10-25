from dataclasses import replace

import pytest

from multi_agent_nav.config import go2_local_gates, load_config
from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.mock import MockMission, make_machine, mock_config
from multi_agent_nav.models import AgentContext, AgentState, Command, Health, Observation, Pose2D, validate_command


def local_config():
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    return config


def scanning_agent(config=None):
    config = config or local_config()
    agent, _ = make_machine(config, 'go2', True)
    agent.local_execution_enabled = True
    agent.tick(0, Health(localization_valid=True, nav_ready=True, stopped=True))
    sync = Command(task_id='task', recipient_id='go2', alignment_id=agent.context.alignment_id,
                   common_frame_id=agent.context.common_frame_id,
                   action='SYNC' if agent.context.share_c_coordinates else 'SYNC_LOCAL_C',
                   phase='SEARCH_AB' if agent.context.share_c_coordinates else 'SYNCING')
    assert agent.accept(sync, 0).accepted
    entry = config['alignment']['robots']['ranger']
    agent.receive_peer(AgentState(task_id='task',robot_id='ranger',local_map_id=entry['local_map_id'],
        map_version=entry['map_version'],common_frame_id='',alignment_id='',run_state='SYNCED',reply='SYNCED',
        localization_valid=True,nav_ready=True),0)
    agent.tick(0,agent.health)
    sync = replace(sync, phase='SEARCH_AB')
    assert agent.accept(sync, 0).accepted
    scan = replace(sync, action='EXECUTE', command_id='scan', sequence=1,
                   kind='SEARCH_VIEWPOINT', target_id='C', position_valid=False)
    assert agent.accept(scan, 0).accepted
    agent.take_effects()
    return agent, sync


def detection(**changes):
    return replace(Observation(target_id='C', scan_id='scan', observation_id='own-C',
                               status='DETECTED', position_valid=True,
                               common_position=Pose2D(9, 1)), **changes)


def local_deposit(agent, sync, observation_id='own-C'):
    agent.scan_completed([Observation(target_id=t, scan_id='scan') for t in 'ABC'], 0)
    agent.take_effects()
    agent._state.ready = True
    assert agent.accept(replace(sync, phase='DEPOSIT'), 0).accepted
    return replace(sync, action='EXECUTE', phase='DEPOSIT', kind='APPROACH_DEPOSIT',
                   target_id='C', command_id='deposit', sequence=2,
                   observation_id=observation_id)


def test_early_c_stays_local_and_deposit_resolves_own_reference():
    agent, sync = scanning_agent()
    agent.on_observation(detection(), 0)
    wire = agent.take_effects()[0].payload
    assert wire.status == 'CACHED'
    assert not wire.position_valid and wire.common_position == Pose2D()
    assert wire.task_id == 'task' and wire.robot_id == 'go2'
    command = local_deposit(agent, sync)
    assert agent.accept(command, 0).accepted
    effects = agent.take_effects()
    assert effects[0].kind == 'LOCAL_EXECUTION'
    assert agent.local_c_observations['own-C'].common_position == Pose2D(9, 1)
    assert not command.position_valid and command.object_position == Pose2D()
    assert agent.accept(command, 0).accepted
    assert not agent.take_effects()


@pytest.mark.parametrize('changes', [
    {'status': 'UNCERTAIN'}, {'status': 'API_ERROR'}, {'position_valid': False},
    {'common_position': Pose2D(float('nan'), 1)}, {'observation_id': ''},
])
def test_unusable_c_is_scrubbed_and_cannot_supply_a_deposit(changes):
    agent, sync = scanning_agent()
    agent.on_observation(detection(**changes), 0)
    wire = agent.take_effects()[0].payload
    assert wire.status != 'CACHED'
    assert not wire.position_valid and wire.common_position == Pose2D()
    assert not agent.accept(local_deposit(agent, sync), 0).accepted


@pytest.mark.parametrize('paused,scan_id', [(True, 'scan'), (False, 'old-scan')])
def test_canceled_or_old_scan_cannot_populate_local_c(paused, scan_id):
    agent, sync = scanning_agent()
    if paused:
        agent.accept(replace(sync, action='PAUSE'), 0)
        agent.take_effects()
    agent.on_observation(detection(scan_id=scan_id), 0)
    assert not agent.take_effects()
    assert not agent.local_c_observations


def test_peer_reference_or_wire_coordinates_cannot_replace_local_c():
    agent, sync = scanning_agent()
    agent.on_observation(detection(), 0)
    command = local_deposit(agent, sync, 'peer-C')
    assert not agent.accept(command, 0).accepted
    command = replace(command, observation_id='own-C', position_valid=True,
                      object_position=Pose2D(15, 8))
    assert not agent.accept(command, 0).accepted
    command = replace(command, position_valid=False)
    assert not agent.accept(command, 0).accepted


def test_live_scene_uses_local_c_behavior():
    agent, _ = scanning_agent(load_config())
    agent.on_observation(detection(), 0)
    wire = agent.take_effects()[0].payload
    assert wire.status == 'CACHED' and not wire.position_valid


def test_private_mission_finishes_with_own_c_and_coordinate_free_wire():
    mission = MockMission(local_config())
    wire = []
    for runner in mission.runners.values():
        def publish(observation):
            wire.append(observation)
            mission.coordinator.ingest_observation(observation, mission.now)
        runner.on_observation = publish
    assert mission.start().accepted
    mission.run(100)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    deposits = [(r, c) for r, c in mission.executed if c.kind == 'APPROACH_DEPOSIT']
    assert [r for r, _ in deposits] == ['go2', 'ranger']
    for robot, command in deposits:
        assert command.observation_id.startswith(robot + '/')
        assert not command.position_valid
        assert command.object_position == command.common_goal == Pose2D()
        assert not command.token and not command.resource_id
    assert not any(c.phase == 'SEARCH_C' and c.kind == 'SEARCH_VIEWPOINT'
                   for _, c in mission.executed)
    assert all(not o.position_valid and o.common_position == Pose2D()
               for o in wire if o.target_id == 'C')
    assert any(o.target_id == 'A' for o in wire)
    assert all(not o.position_valid and o.common_position == Pose2D() for o in wire)
    assert not any(s.token for s in mission.coordinator.states.values())


@pytest.mark.parametrize('missing', ['go2', 'ranger'])
def test_peer_c_never_substitutes_for_missing_own_detection(missing):
    mission = MockMission(local_config())
    del mission.adapters[missing].detections['C']
    assert mission.start().accepted
    mission.run(100)
    assert mission.coordinator.phase == 'PAUSED'
    searches = [(r, c) for r, c in mission.executed
                if c.phase in ('SEARCH_C','DEPOSIT') and c.kind == 'SEARCH_VIEWPOINT']
    assert searches and all(r == missing for r, _ in searches)
    assert not any(r == missing and c.kind == 'APPROACH_DEPOSIT' for r, c in mission.executed)


def test_missing_robot_can_find_c_after_ready_and_finish():
    mission = MockMission(local_config())
    del mission.adapters['ranger'].detections['C']
    assert mission.start().accepted
    for _ in range(100):
        mission.step()
        if mission.coordinator.phase in ('SEARCH_C','DEPOSIT'):
            break
    assert mission.coordinator.phase in ('SEARCH_C','DEPOSIT')
    assert all(s.ready for s in mission.coordinator.states.values())
    mission.adapters['ranger'].detections['C'] = Pose2D(9, 1)
    mission.run(100)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason


def test_new_task_cannot_reuse_previous_local_c():
    mission = MockMission(local_config())
    first = mission.start()
    mission.run(100)
    assert mission.coordinator.phase == 'DONE'
    del mission.adapters['go2'].detections['C']
    second = mission.start()
    assert second.accepted and second.task_id != first.task_id
    mission.run(100)
    assert mission.coordinator.phase == 'PAUSED'
    assert not mission.machines['go2'].local_c_observations
    assert not any(r == 'go2' and c.kind == 'APPROACH_DEPOSIT' and c.task_id == second.task_id
                   for r, c in mission.executed)


@pytest.mark.parametrize('changes', [
    {}, {'task_id': 'old'}, {'alignment_id': 'old'}, {'map_version': 'old'},
    {'simulation': False}, {'target_id': 'A'}, {'position_valid': True},
    {'common_position': Pose2D(9, 1)},
])
def test_coordinator_only_accepts_current_coordinate_free_local_c(changes):
    mission = MockMission(local_config())
    assert mission.start().accepted
    mission.run(3)
    assert mission.coordinator.phase == 'SEARCH_AB'
    observation = Observation(task_id=mission.coordinator.task_id, robot_id='go2',
        local_map_id='synthetic_go2', map_version='1', common_frame_id='', alignment_id='',
        target_id='C', status='CACHED', observation_id='own-C')
    mission.coordinator.ingest_observation(replace(observation, **changes), 0)
    assert bool(mission.coordinator.observations) == (not changes)


def test_private_coordinator_ignores_old_peer_c_coordinates():
    mission = MockMission(local_config())
    assert mission.start().accepted
    observation = replace(detection(), task_id=mission.coordinator.task_id, robot_id='ranger',
        local_map_id='synthetic_ranger', map_version='2', alignment_id='SYNTHETIC_v1')
    mission.coordinator.ingest_observation(observation, 0)
    assert not mission.coordinator.observations


@pytest.mark.parametrize('mode', ['false', 0, None, []])
def test_invalid_sharing_mode_is_reported_before_live_start(mode):
    config = load_config()
    config['scene']['share_c_coordinates'] = mode
    assert 'share_c_coordinates must be boolean' in go2_local_gates(config)


@pytest.mark.parametrize('shared,action', [(False, 'SYNC_LOCAL_C'), (True, 'SYNC')])
def test_sync_requires_matching_c_policy_before_task_identity_is_established(shared, action):
    config = mock_config()
    config['scene']['share_c_coordinates'] = shared
    agent, _ = make_machine(config, 'go2', True)
    agent.tick(0, Health(localization_valid=True, nav_ready=True, stopped=True))
    command = Command(task_id='new', recipient_id='go2', alignment_id=agent.context.alignment_id,
                      common_frame_id=agent.context.common_frame_id, action=action,
                      phase='SEARCH_AB' if shared else 'SYNCING')
    mismatch = replace(command, action='SYNC' if action == 'SYNC_LOCAL_C' else 'SYNC_LOCAL_C')
    assert not agent.accept(mismatch, 0).accepted
    assert agent.context.task_id == '' and not agent.take_effects()
    assert agent.accept(command, 0).accepted
    assert agent.context.task_id == 'new'


def test_local_coordinator_sends_explicit_sync_that_legacy_agent_cannot_accept():
    mission = MockMission(local_config())
    assert mission.start().accepted
    commands = mission.coordinator.tick(0)
    assert [c.action for c in commands] == ['SYNC_LOCAL_C', 'SYNC_LOCAL_C']
    shared = AgentContext('go2', mission.coordinator.task_id, 'synthetic_go2', '1',
                          'lab_ref', 'SYNTHETIC_v1', True)
    assert not validate_command(commands[0], shared).accepted


def test_mixed_c_policies_cannot_start_scanning_or_publish_c():
    mission = MockMission(local_config())
    mission.machines['ranger'].context.share_c_coordinates = True
    assert mission.start().accepted
    mission.run(50)
    assert mission.coordinator.phase == 'SYNCING'
    assert not mission.executed and not mission.coordinator.observations
    mission.now = mission.coordinator.task_timeout
    mission.step()
    assert mission.coordinator.phase == 'PAUSED'


def test_local_c_scan_api_failure_pauses_without_deposit(monkeypatch):
    mission = MockMission(local_config())
    adapter = mission.adapters['go2']
    del adapter.detections['C']
    scan = adapter.start_scan
    attempts = []

    def failing_scan(descriptions, scan_id, **options):
        values = scan(descriptions, scan_id, **options).result()
        if mission.coordinator.phase in ('SEARCH_C','DEPOSIT'):
            attempts.append(scan_id)
            values = [replace(o, status='API_ERROR') if o.target_id == 'C' else o
                      for o in values]
        return completed(values)

    monkeypatch.setattr(adapter, 'start_scan', failing_scan)
    assert mission.start().accepted
    mission.run(120)
    assert attempts
    assert mission.coordinator.phase == 'PAUSED'
    assert not mission.machines['go2'].state().deposited
