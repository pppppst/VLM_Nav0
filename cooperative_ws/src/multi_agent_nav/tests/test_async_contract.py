"""Final Ranger/Go2 wire contract; no ROS graph or hardware is started."""
from dataclasses import replace

import pytest

from multi_agent_nav.models import Command, Pose2D, task_acknowledges, validate_command
from test_startup_handshake import setup, peer, sync, preparing_coordinator


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
@pytest.mark.parametrize('action,kind', [('SYNC', 'SYNC_LOCAL_C'),
    ('SYNC_LOCAL_C', 'WAIT'), ('SYNC_LOCAL_C', ''), ('SYNC_LOCAL_C', 'SYNC_LOCAL_C')])
def test_both_formal_sync_encodings_establish_identity(robot, action, kind):
    _, agent, _ = setup(robot)
    command = replace(sync(robot), action=action, kind=kind)
    assert agent.accept(command, 0.).accepted
    assert agent.state().task_id == 'task' and agent.state().run_state == 'SYNCING'
    assert agent.state().reply == '' and not agent.take_effects()


@pytest.mark.parametrize('robot', ['go2', 'ranger'])
@pytest.mark.parametrize('phase', ['IDLE', 'SEARCH_AB', 'SEARCH_C', 'DEPOSIT', 'PAUSED', 'DONE', 'FAILED'])
def test_restarted_agent_cannot_adopt_post_handshake_task(robot, phase):
    _, agent, _ = setup(robot)
    assert not agent.accept(sync(robot, phase=phase), 0.).accepted
    assert agent.context.task_id == '' and not agent.take_effects()


@pytest.mark.parametrize('updates', [dict(task_id=''), dict(sequence=1), dict(command_id='old'),
    dict(ack_sequence=1), dict(kind='other'), dict(position_valid=True),
    dict(common_goal=Pose2D(1., 0.)), dict(object_position=Pose2D(1., 0.)),
    dict(target_id='A'), dict(viewpoint_id='old'), dict(observation_id='old'),
    dict(parent_command_id='old'), dict(cancel_command_id='old'), dict(resource_id='old'), dict(token='old')])
def test_dirty_sync_cannot_establish_task(updates):
    _, agent, _ = setup()
    command = replace(sync(), **updates)
    assert not agent.accept(command, 0.).accepted
    assert not agent.context.task_id and not agent.take_effects()


def acknowledged_agent():
    config, agent, health = setup()
    assert agent.accept(sync(), 0.).accepted
    agent.receive_peer(peer(config, task_id='task', run_state='SYNCING'), .1)
    agent.tick(.1, health)
    return config, agent, health


def test_syncing_peer_can_trigger_the_canonical_double_field_ack():
    _, agent, _ = acknowledged_agent()
    assert (agent.state().run_state, agent.state().reply) == ('SYNCED', 'SYNCED')
    assert task_acknowledges(agent.state(), 'task')
    assert not agent.state().ready


@pytest.mark.parametrize('run_state,reply,accepted', [
    ('SYNCED', 'SYNCED', True), ('SYNCED', '', False), ('SYNCING', '', False),
    ('SYNCING', 'SYNCED', False), ('IDLE', 'SYNCED', False), ('STANDBY', 'SYNCED', False)])
def test_coordinator_requires_exact_ack_and_commits_phase_before_execute(run_state, reply, accepted):
    config, coordinator = preparing_coordinator()
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id,
            run_state=run_state, reply=reply), .1)
    commands = coordinator.tick(.1)
    execute = [c for c in commands if c.action == 'EXECUTE']
    assert bool(execute) == accepted
    if accepted:
        assert [c.phase for c in commands[:2]] == ['SEARCH_AB', 'SEARCH_AB']
        assert all(c.action in ('SYNC', 'SYNC_LOCAL_C') for c in commands[:2])
        assert {c.target_id for c in execute} == {'A', 'B'}
    else:
        assert coordinator.phase == 'SYNCING'


def test_ack_history_survives_health_withdrawal_and_moving_peer_before_phase_commit():
    config, agent, health = acknowledged_agent()
    agent.tick(.2, replace(health, nav_ready=False))
    assert agent.state().run_state == 'SYNCING' and agent.state().reply == ''
    agent.receive_peer(peer(config, task_id='task', run_state='NAVIGATING', stopped=False,
        sequence=1, command_id='peer/command', goal_id='peer/goal', reply='EXECUTING'), .3)
    agent.tick(.3, health)
    assert agent.accept(sync(phase='SEARCH_AB'), .4).accepted
    execute = replace(sync(phase='SEARCH_AB'), action='EXECUTE', command_id='command',
        sequence=1, kind='APPROACH_PICKUP', target_id='A')
    assert agent.accept(execute, .4).accepted


@pytest.mark.parametrize('failure', ['peer_timeout', 'sync_timeout', 'local_unready', 'peer_restart'])
def test_phase_commit_enables_watchdog_even_before_first_execute(failure):
    config, agent, health = acknowledged_agent()
    assert agent.accept(sync(phase='SEARCH_AB'), .2).accepted
    now = 4.
    if failure != 'sync_timeout':
        assert agent.accept(sync(phase='SEARCH_AB'), now).accepted
    if failure != 'peer_timeout':
        agent.receive_peer(peer(config, task_id='' if failure == 'peer_restart' else 'task',
            run_state='SYNCED', reply='SYNCED'), now)
    effects = agent.tick(now, replace(health, nav_ready=False) if failure == 'local_unready' else health)
    assert agent.paused and any(e.kind == 'STOP' for e in effects)


@pytest.mark.parametrize('updates', [dict(sequence=2), dict(reply='old'), dict(resource_id='old'),
    dict(token='old'), dict(permit_command_id='old'), dict(resource_status='old'),
    dict(requested_resource='old'), dict(requested_command_id='old'), dict(pose_valid=True),
    dict(common_pose=Pose2D(1., 0.))])
def test_dirty_preparation_cannot_enter_standby_or_start(updates):
    config, agent, health = setup()
    dirty = replace(peer(config), **updates)
    agent.receive_peer(dirty, .1)
    agent.tick(.1, health)
    assert agent.state().run_state == 'WAITING_PEER'
    config, coordinator = preparing_coordinator()
    coordinator = type(coordinator)(config['scene'], config['alignment']['alignment_id'],
        {robot:(identity['local_map_id'], identity['map_version'])
         for robot,identity in config['alignment']['robots'].items()}, True)
    coordinator.ingest_state(peer(config, 'go2'), .1)
    coordinator.ingest_state(dirty, .1)
    assert not coordinator.start(config['scene']['scenario_id'], .1).accepted


@pytest.mark.parametrize('updates', [dict(local_map_id='unknown'), dict(map_version='unknown'),
    dict(schema_version=2), dict(simulation=False), dict(common_frame_id='foreign')])
def test_invalid_peer_identity_blocks_preparation_and_has_a_reason(updates):
    config, agent, health = setup()
    agent.receive_peer(replace(peer(config), **updates), .1)
    agent.tick(.1, health)
    assert agent.state().run_state == 'WAITING_PEER'
    assert agent.peer_received is None
    assert 'mismatch' in agent.state().reason


def test_one_total_budget_covers_handshake_and_execution():
    config, coordinator = preparing_coordinator()
    task, started = coordinator.task_id, coordinator.started
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=task, run_state='SYNCING'), 590.)
    assert all(c.action != 'EXECUTE' for c in coordinator.tick(590.))
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=task, run_state='SYNCED', reply='SYNCED'), 599.)
    assert any(c.action == 'EXECUTE' for c in coordinator.tick(599.))
    assert coordinator.started == started
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=task, run_state='PLANNING',
            command_id='command', sequence=1), 601.)
    coordinator.tick(601.)
    assert coordinator.phase == 'PAUSED' and coordinator.reason == 'task timeout'


def test_syncing_uses_pause_and_commandless_cancel_is_rejected():
    _, agent, health = acknowledged_agent()
    cancel = replace(sync(), action='CANCEL')
    assert not agent.accept(cancel, .2).accepted
    assert agent.accept(replace(cancel, action='PAUSE'), .3).accepted
    agent.tick(.3, health)
    reason = agent.state().reason
    assert agent.accept(sync(), .4).accepted
    agent.tick(.4, health)
    assert agent.paused and agent.state().reason == reason


def test_old_task_packets_do_not_refresh_current_task_liveness():
    config, agent, health = acknowledged_agent()
    agent.accept(sync(phase='SEARCH_AB'), .2)
    agent.accept(sync(phase='SEARCH_AB'), 4.)
    agent.receive_peer(peer(config, task_id='old', run_state='NAVIGATING'), 4.)
    assert agent.peer_received == 4. and agent.peer_task_received == .1
    agent.tick(4., health)
    assert agent.paused and 'current task' in agent.state().reason


def test_execute_intent_rejects_private_resource_and_viewpoint_fields():
    _, agent, _ = acknowledged_agent()
    context = agent.context
    command = replace(sync(phase='SEARCH_AB'), action='EXECUTE', kind='APPROACH_PICKUP',
        target_id='A', command_id='command', sequence=1)
    for field in ('viewpoint_id', 'resource_id', 'token', 'parent_command_id'):
        assert not validate_command(replace(command, **{field:'old'}), context).accepted


def test_coordinator_pauses_on_post_commit_restart_without_initial_grace():
    config, coordinator = preparing_coordinator()
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id,
            run_state='SYNCED', reply='SYNCED'), .1)
    assert any(c.action == 'EXECUTE' for c in coordinator.tick(.1))
    coordinator.ingest_state(peer(config, 'ranger', task_id='', run_state='WAITING_PEER'), .2)
    coordinator.tick(.2)
    assert coordinator.phase == 'PAUSED' and 'restart' in coordinator.reason


def test_explicit_start_can_replace_stopped_paused_tasks_after_coordinator_restart():
    config, _, _ = setup()
    from multi_agent_nav.mock import make_coordinator
    coordinator = make_coordinator(config, True)
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id='previous', run_state='PAUSED',
            command_id='previous/command', sequence=3, reply='CANCELED'), 0.)
    reply = coordinator.start(config['scene']['scenario_id'], 0.)
    assert reply.accepted and reply.task_id != 'previous'
    assert coordinator.phase == 'SYNCING'
    command = coordinator.tick(.1)[0]
    _, agent, health = setup(command.recipient_id)
    agent.context = replace(agent.context, task_id='previous')
    agent.stop('old pause');agent.tick(0., health)
    assert agent.accept(command, .1).accepted
    assert agent.state().task_id == reply.task_id and not agent.state().command_id
    assert agent.state().sequence == 0 and not agent.state().ready and not agent.paused


def test_old_paused_cache_cannot_cancel_new_explicit_task_before_sync_delivery():
    config, coordinator = preparing_coordinator()
    old_task = coordinator.task_id
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=old_task, run_state='PAUSED',
            reply='CANCELED', reason='old pause'), .1)
    coordinator.pause('previous task paused')
    assert coordinator.start(config['scene']['scenario_id'], .2).accepted
    commands = coordinator.tick(.2)
    assert coordinator.phase == 'SYNCING' and coordinator.task_id != old_task
    assert len(commands) == 2 and all(c.action == 'SYNC_LOCAL_C' and c.phase == 'SYNCING' for c in commands)


def test_deployment_parameters_and_map_identities_match_the_final_contract():
    from multi_agent_nav.config import load_config
    # Literal values come from ans_async_startup.md, not from implementation defaults.
    config = load_config()
    assert config['scene']['share_c_coordinates'] is False
    assert {name:config['scene'][name] for name in ('heartbeat_hz','lost_timeout','task_timeout')} == dict(
        heartbeat_hz=2.,lost_timeout=3.,task_timeout=600.)
    assert {robot:(entry['local_map_id'],entry['map_version']) for robot,entry in config['alignment']['robots'].items()} == {
        'ranger':('lab_ranger_20261018161152','sha256:8b1a3f0587afd56e20431668647cf2624d83897a587090ef808424eb6049fc30'),
        'go2':('lab_rescan_20261018_155656_0pa6gm','sha256:6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279')}
