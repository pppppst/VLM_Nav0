"""Opt-in active task budgets; all freshness and stop checks retain wall time."""
from dataclasses import replace

import pytest

from multi_agent_nav.mock import MockMission, make_machine, mock_config
from multi_agent_nav.models import is_recoverable_pause
from test_startup_handshake import activate, peer, preparing_coordinator, setup, sync


def recovery_agent():
    config, agent, health = activate()
    agent.context = replace(agent.context, wait_recovery=True)
    return config, agent, health


def test_recovery_policy_is_explicit_robot_profile_and_old_defaults_remain_off():
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    config['robot_go2']['wait_recovery'] = True
    go2, _ = make_machine(config, 'go2', True)
    ranger, _ = make_machine(config, 'ranger', True)
    assert getattr(go2.context, 'wait_recovery', False)
    assert not getattr(ranger.context, 'wait_recovery', False)


@pytest.mark.parametrize('fault', ['localization', 'navigation', 'heartbeat'])
def test_pause_over_1270_seconds_keeps_same_task_and_pending_command(fault):
    config, agent, health = recovery_agent()
    command, started = agent.command, agent.task_started
    bad = replace(health, localization_valid=False) if fault == 'localization' else replace(health, nav_ready=False)
    agent.tick(.3, bad if fault != 'heartbeat' else health)
    if fault == 'heartbeat':agent.tick(4., health)
    assert agent.recovery_state is not None
    frozen = agent.task_clock(4.)
    for now in (605., 1275., 1300.):
        agent.tick(now, bad if fault != 'heartbeat' else health)
        assert agent.paused and agent.recovery_state is not None
        assert agent.task_clock(now) == frozen
        assert 'task timeout' not in agent.state().reason
    agent.accept(sync(phase='SEARCH_AB'), 1300.1)
    agent.receive_peer(peer(config, task_id='task', run_state='PLANNING'), 1300.1)
    effects = agent.tick(1300.1, health)
    assert not agent.paused and not agent.awaiting_peer_resume
    assert agent.command is command and agent.task_started == started
    assert any(e.kind == 'LOCAL_EXECUTION' for e in effects)
    assert agent.task_clock(1300.2) - frozen == pytest.approx(.1)


def test_budget_waits_for_late_peer_after_local_resume_and_freshness_is_real():
    config, agent, health = recovery_agent()
    agent.tick(.3, replace(health, nav_ready=False))
    agent.accept(sync(phase='SEARCH_AB'), .4)
    agent.receive_peer(peer(config, task_id='task', run_state='PAUSED', reason='peer paused for recovery'), .4)
    agent.tick(.4, health)
    assert agent.awaiting_peer_resume and not agent.paused
    frozen = agent.task_clock(.4)
    for now in (100., 700., 1300.):
        agent.accept(sync(phase='SEARCH_AB'), now)
        agent.receive_peer(peer(config, task_id='task', run_state='PAUSED', reason='peer paused for recovery'), now)
        agent.tick(now, health)
        assert agent.awaiting_peer_resume and agent.task_clock(now) == frozen
    agent.receive_peer(peer(config, task_id='task', run_state='PLANNING'), 1300.1)
    agent.tick(1300.1, health)
    assert not agent.awaiting_peer_resume
    assert agent.task_clock(1300.2) - frozen == pytest.approx(.1)
    agent.tick(1304., health)
    assert agent.paused and 'heartbeat' in agent.state().reason


@pytest.mark.parametrize('fault', ['manual', 'identity', 'mixed_peer', 'cleanup'])
def test_opted_in_policy_never_releases_hard_fault(fault):
    config, agent, health = recovery_agent()
    agent.tick(.3, replace(health, nav_ready=False))
    if fault == 'manual':agent.accept(replace(sync(phase='PAUSED'), action='PAUSE'), .4)
    elif fault == 'identity':agent.receive_peer(peer(config, task_id='task', map_version='wrong'), .4)
    elif fault == 'mixed_peer':agent.receive_peer(peer(config, task_id='task', run_state='PAUSED',
        reason='peer localization/navigation invalid', localization_valid=False), .4)
    else:agent.stop('local execution cleanup failed')
    assert agent.recovery_state is None
    agent.accept(sync(phase='SEARCH_AB'), 1300.)
    agent.receive_peer(peer(config, task_id='task', run_state='PLANNING'), 1300.)
    agent.tick(1300., health)
    assert agent.paused and agent.recovery_state is None


def test_new_localization_reason_whitelist_keeps_old_mixed_reason_hard():
    assert is_recoverable_pause('local localization temporarily unavailable')
    assert is_recoverable_pause('peer localization temporarily unavailable')
    assert not is_recoverable_pause('peer localization/navigation invalid')
    assert not is_recoverable_pause('local health invalid: nav_ready, localization_valid')


def test_coordinator_budget_freezes_in_pause_and_asymmetric_resume():
    config, coordinator = preparing_coordinator()
    coordinator.wait_recovery = True
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=coordinator.task_id, run_state='SYNCED', reply='SYNCED'), .1)
    coordinator.tick(.1)
    task_id, started = coordinator.task_id, coordinator.started
    coordinator.ingest_state(peer(config, 'go2', task_id=task_id, localization_valid=False,
        run_state='STOPPING', reason='local localization temporarily unavailable'), .2)
    coordinator.tick(.2)
    assert coordinator.resume_phase == 'SEARCH_AB'
    frozen = coordinator.task_clock(.2)
    coordinator.tick(1300.)
    assert coordinator.resume_phase == 'SEARCH_AB' and 'task timeout' not in coordinator.reason
    for robot in ('go2', 'ranger'):
        coordinator.ingest_state(peer(config, robot, task_id=task_id, run_state='PAUSED', reason='peer paused for recovery'), 1300.)
    coordinator.tick(1300.)
    assert coordinator.awaiting_resume == {'go2', 'ranger'}
    for now in (1400., 2600.):
        coordinator.ingest_state(peer(config, 'go2', task_id=task_id, run_state='PLANNING'), now)
        coordinator.ingest_state(peer(config, 'ranger', task_id=task_id, run_state='PAUSED', reason='peer paused for recovery'), now)
        coordinator.tick(now)
        assert coordinator.phase == 'SEARCH_AB' and coordinator.awaiting_resume == {'ranger'}
        assert coordinator.task_clock(now) == frozen
    coordinator.ingest_state(peer(config, 'ranger', task_id=task_id, run_state='PLANNING'), 2600.1)
    coordinator.tick(2600.1)
    assert not coordinator.awaiting_resume
    assert coordinator.task_clock(2600.2) - frozen == pytest.approx(.1)
    assert coordinator.task_id == task_id and coordinator.started == started


def test_opted_in_initial_handshake_wait_does_not_consume_budget():
    config, agent, health = setup()
    agent.context = replace(agent.context, wait_recovery=True)
    assert agent.accept(sync(), 0.).accepted
    config2, coordinator = preparing_coordinator()
    coordinator.wait_recovery = True
    for now in (1., 601., 1300.):
        agent.accept(sync(), now)
        agent.tick(now, health)
        coordinator.tick(now)
        assert not agent.paused and agent.state().run_state == 'SYNCING'
        assert coordinator.phase == 'SYNCING'
        assert agent.task_clock(now) == 0.
        assert coordinator.task_clock(now) == 0.


def test_runner_control_recovery_starts_after_stop_and_cancel_stays_cancelled():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    config, agent, health = recovery_agent()
    now = [.3]
    adapter = FakeAdapter('go2')
    adapter.get_health = lambda: replace(health, nav_ready=False)
    calls = []
    adapter.begin_control_task = lambda task: calls.append(('begin', task))
    adapter.recover_control = lambda **kw: calls.append(('recover', kw))
    adapter.cancel_control_recovery = lambda: calls.append(('cancel', None))
    adapter.stop_motion = lambda reason: calls.append(('stop', reason))
    runner = AgentRunner(agent, adapter, {}, lambda _: None, clock=lambda: now[0])
    def execution():
        try:yield
        finally:calls.append(('cleanup', None))
    runner.execution = execution();next(runner.execution)
    runner.tick(now[0])
    names = [name for name, _ in calls]
    assert names.index('cleanup') < names.index('stop') < names.index('recover')
    assert ('begin', 'task') in calls
    agent.accept(replace(sync(phase='PAUSED'), action='CANCEL', cancel_command_id='command'), .4)
    now[0] = .4
    runner.tick(now[0])
    count = sum(name == 'recover' for name, _ in calls)
    for stamp in (.5, 1300.):
        now[0] = stamp
        runner.tick(stamp)
    assert sum(name == 'recover' for name, _ in calls) == count
    assert sum(name == 'cancel' for name, _ in calls) == 1
    assert sum(name == 'begin' for name, _ in calls) == 1
    adapter.get_health = lambda: health
    now[0] = 1300.1
    runner.tick(now[0])
    assert agent.accept(sync(task_id='new-task'), now[0]).accepted
    runner.tick(now[0])
    assert [value for name, value in calls if name == 'begin'] == ['task', 'new-task']


def test_exploration_whole_deadline_uses_task_clock_but_planner_timeout_uses_wall_clock():
    from concurrent.futures import Future
    from multi_agent_nav.frame_adapter import LocalFrame
    from multi_agent_nav.go2_exploration import ExplorationStopped, explore_steps
    from test_go2_exploration import LocalFixture
    adapter = LocalFixture()
    plan = Future()
    adapter.compute_path = lambda goal: plan
    now = [1300.]
    config = dict(scene=dict(viewpoints=dict(go2=[dict(id='p1', pose=[1., 0., 0.])])),
        robot_go2=dict(exploration_strategy='fixed_viewpoints'))
    steps = explore_steps(adapter, config, LocalFrame(), 600., 'target', lambda _: None,
        lambda: [], lambda: True, lambda: None, clock=lambda: now[0], task_clock=lambda: .3)
    next(steps)
    assert not plan.done()
    now[0] += 16.
    with pytest.raises(ExplorationStopped, match='path planning timeout'):
        next(steps)
    assert plan.cancelled()


def test_default_coordinator_localization_loss_cannot_hide_behind_soft_pause():
    config, coordinator = preparing_coordinator()
    coordinator.phase = 'SEARCH_AB'
    coordinator.ingest_state(peer(config, 'go2', task_id=coordinator.task_id, run_state='PAUSED',
        reason='local navigation not ready', localization_valid=False), .2)
    coordinator.tick(.2)
    assert coordinator.phase == 'PAUSED' and coordinator.resume_phase is None


def test_shared_coordinate_profile_does_not_opt_into_localization_recovery():
    config = mock_config()
    config['robot_go2']['wait_recovery'] = True
    agent, _ = make_machine(config, 'go2', True)
    assert agent.task_clock(1300.) == 1300.


@pytest.mark.parametrize('fault', ['localization', 'navigation', 'peer_heartbeat'])
def test_complete_opted_in_dual_mission_after_long_pause(fault):
    config = mock_config()
    config['scene'].update(share_c_coordinates=False, wait_recovery=True, resources={}, exit_points={})
    # Test both opted-in consumers without changing Ranger's deployed default.
    for robot in ('go2', 'ranger'):config['robot_'+robot]['wait_recovery'] = True
    mission = MockMission(config)
    for adapter in mission.adapters.values():adapter.delay = 1.
    assert mission.start().accepted
    mission.run(10)
    task_id = mission.coordinator.task_id
    adapter = mission.adapters['go2']; health = adapter.get_health
    if fault == 'peer_heartbeat':mission.online['ranger'] = False
    else:adapter.get_health = lambda: replace(health(), **(
        {'localization_valid': False} if fault == 'localization' else {'nav_ready': False}))
    mission.run(40)
    assert all(a.paused for a in mission.machines.values())
    mission.step(1300.)
    assert mission.coordinator.resume_phase is not None
    assert all(a.recovery_state is not None for a in mission.machines.values())
    adapter.get_health = health
    mission.online['ranger'] = True
    mission.run(350)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert mission.coordinator.task_id == task_id
    assert all(a.state().ready and a.state().deposited for a in mission.machines.values())


@pytest.mark.parametrize('kind,detail,recoverable', [
    ('localization', 'motion owner/localization/Nav2/costmap health lost: localization_valid', True),
    ('runtime', 'global goal costmap missing, stale or TF unavailable', True),
    ('runtime', 'candidate planning timeout', True),
    ('runtime', 'initial camera inference failed', True),
    ('runtime', 'scan failed', True),
    ('runtime', 'candidate selection unavailable: selection API unavailable', True),
    ('runtime', 'selection map assets changed', False),
    ('runtime', 'fixture unknown failure', False),
    ('programmer', 'scan failed', False),
])
def test_runner_only_recovers_explicit_runtime_failures(kind, detail, recoverable):
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.go2_exploration import ExplorationStopped, LocalizationNotReady
    from multi_agent_nav.runner import AgentRunner
    config, agent, health = recovery_agent()
    adapter = FakeAdapter('go2')
    runner = AgentRunner(agent, adapter, {}, lambda _: None, clock=lambda: .3)
    error = LocalizationNotReady if kind == 'localization' else ExplorationStopped if kind == 'runtime' else ValueError
    def execution():
        raise error(detail)
        yield
    runner.execution = execution()
    runner.tick(.3)
    assert agent.paused
    assert (agent.recovery_state is not None) == recoverable
    if recoverable:assert is_recoverable_pause(agent.state().reason)


def test_local_rotation_tf_wait_keeps_budget_frozen_until_probe_recovers():
    config = mock_config()
    config['scene'].update(share_c_coordinates=False, wait_recovery=True, resources={}, exit_points={})
    for robot in ('go2', 'ranger'):config['robot_'+robot]['wait_recovery'] = True
    mission = MockMission(config)
    adapter = mission.adapters['go2']
    rotation = adapter.local_rotation_check
    adapter.local_rotation_check = lambda pose: dict(status='unavailable', reason='TF unavailable')
    assert mission.start().accepted
    mission.run(70)
    agent = mission.machines['go2']
    assert agent.paused and agent.recovery_state is not None
    frozen = agent.task_clock(mission.now)
    for _ in range(3):
        mission.step(1300.)
        mission.run(10)
        assert agent.paused and agent.recovery_state is not None
        assert agent.task_clock(mission.now) == frozen
    adapter.local_rotation_check = rotation
    mission.run(350)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason


def test_generated_viewpoint_wait_is_soft_and_retains_command_progress():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    _, agent, _ = recovery_agent()
    runner = AgentRunner(agent, FakeAdapter('go2'), {}, lambda _: None, clock=lambda: .3)
    runner.execution_identity = ('task', 'command')
    runner.execution_progress = {'completed': {'p1'}}
    runner.local_completion = (runner.execution_identity, 'WAITING_VIEWPOINTS')
    runner.tick(.3)
    assert agent.paused and agent.recovery_state is not None
    assert runner.execution_identity == ('task', 'command')
    assert runner.execution_progress['completed'] == {'p1'}


@pytest.mark.parametrize('cause', [TypeError('bug'), RuntimeError('unknown failure')])
def test_known_runtime_wrapper_does_not_hide_unknown_programmer_cause(cause):
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.go2_exploration import ExplorationStopped
    from multi_agent_nav.runner import AgentRunner
    _, agent, _ = recovery_agent()
    runner = AgentRunner(agent, FakeAdapter('go2'), {}, lambda _: None, clock=lambda: .3)
    def execution():
        raise ExplorationStopped('scan failed') from cause
        yield
    runner.execution = execution()
    runner.tick(.3)
    assert agent.paused and agent.recovery_state is None


def test_proven_motion_identity_fault_escalates_even_during_recoverable_wait():
    _, agent, health = recovery_agent()
    agent.tick(.3, replace(health, nav_ready=False))
    agent.tick(.4, replace(health, hard_fault='motion identity unknown'))
    assert agent.paused and agent.recovery_state is None
    assert agent.state().reason == 'motion identity unknown'


@pytest.mark.parametrize('blocked', ['moving', 'motion_pending'])
def test_stopping_freezes_budget_without_assuming_native_or_physical_stop(blocked):
    _, agent, health = recovery_agent()
    unsettled = replace(health, nav_ready=False, stopped=blocked != 'moving',
        motion_terminal=blocked != 'motion_pending')
    agent.tick(.3, unsettled)
    frozen = agent.task_clock(.3)
    for now in (700., 1300.):
        agent.tick(now, unsettled)
        assert agent.state().run_state == 'STOPPING'
        assert agent.task_clock(now) == frozen and agent.recovery_state is not None


def test_physical_stop_deadline_uses_real_time_while_task_budget_is_frozen():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    config, agent, health = recovery_agent()
    now = [.3]
    adapter = FakeAdapter('go2')
    adapter.get_health = lambda: replace(health, stopped=False, nav_ready=False)
    runner = AgentRunner(agent, adapter, {}, lambda _: None, config=config, clock=lambda: now[0])
    agent.tick(.3, adapter.get_health())
    stopping = runner.local_stop()
    next(stopping)
    now[0] = 6.
    with pytest.raises(RuntimeError, match='physical stop unconfirmed'):
        next(stopping)
    assert agent.task_clock(6.) == pytest.approx(.1)


def test_external_owner_during_soft_pause_revokes_auto_resume_even_without_executor():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    _, agent, health = recovery_agent()
    agent.context = replace(agent.context, simulation=False)
    agent.tick(.3, replace(health, nav_ready=False))
    runner = AgentRunner(agent, FakeAdapter('go2'), {}, lambda _: None, clock=lambda: .4)
    runner.owners = lambda: ['manual_navigator']
    runner.tick(.4)
    assert agent.paused and agent.recovery_state is None
    assert agent.state().reason == 'other navigation owner present'
