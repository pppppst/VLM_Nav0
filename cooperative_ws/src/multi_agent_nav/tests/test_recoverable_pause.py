"""Transient health loss stops motion, then resumes the same task without pause deadlock."""
from dataclasses import replace

import pytest

from test_startup_handshake import activate, peer, sync
from test_independent_c import independent_mission


@pytest.mark.parametrize('fault', ['navigation', 'peer_heartbeat', 'coordinator_heartbeat'])
def test_dual_task_recovers_transient_fault_without_new_task(fault):
    mission = independent_mission()
    for adapter in mission.adapters.values():adapter.delay = 1.
    assert mission.start().accepted
    for _ in range(25):
        mission.step()
        if any(a.active for a in mission.adapters.values()):break
    task_id, started = mission.coordinator.task_id, mission.coordinator.started
    adapter = mission.adapters['ranger']
    health = adapter.get_health
    tick = mission.coordinator.tick
    if fault == 'navigation':adapter.get_health = lambda: replace(health(), nav_ready=False)
    elif fault == 'peer_heartbeat':mission.online['ranger'] = False
    else:mission.coordinator.tick = lambda now: []
    mission.run(45)
    assert all(a.paused for a in mission.machines.values())
    assert all(not a.active for a in mission.adapters.values())
    adapter.get_health = health
    mission.online['ranger'] = True
    mission.coordinator.tick = tick
    mission.run(350)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert mission.coordinator.task_id == task_id and mission.coordinator.started == started
    assert all(a.state().ready and a.state().deposited for a in mission.machines.values())


@pytest.mark.parametrize('fault', ['localization', 'manual', 'identity', 'deadline'])
def test_hard_fault_during_transient_pause_cannot_auto_resume(fault):
    config, agent, health = activate()
    agent.tick(.3, replace(health, nav_ready=False))
    if fault == 'localization':agent.tick(.4, replace(health, localization_valid=False))
    elif fault == 'manual':agent.accept(replace(sync(phase='PAUSED'), action='PAUSE'), .4)
    elif fault == 'identity':agent.receive_peer(peer(config, task_id='task', map_version='wrong'), .4)
    else:agent.tick(601., health)
    now = 601.1 if fault == 'deadline' else .5
    agent.receive_peer(peer(config, task_id='task', run_state='SYNCED'), now)
    agent.accept(sync(phase='SEARCH_AB'), now)
    for _ in range(3):
        effects = agent.tick(now, health)
        assert not any(e.kind in ('LOCAL_EXECUTION', 'PLAN', 'NAVIGATE') for e in effects)
    assert agent.paused


def test_task_deadline_still_expires_while_both_heartbeats_missing():
    mission = independent_mission()
    assert mission.start().accepted
    mission.run(4)
    mission.online['ranger'] = False
    mission.step(601.)
    assert mission.coordinator.phase == 'PAUSED'
    assert 'task timeout' in mission.coordinator.reason
    mission.online['ranger'] = True
    mission.run(20)
    assert all(a.paused for a in mission.machines.values())
    assert mission.coordinator.phase == 'PAUSED'


@pytest.mark.parametrize('blocked', ['moving', 'motion_pending', 'peer_moving', 'peer_stopping',
                                   'peer_unhealthy', 'sync_missing', 'peer_missing', 'coordinator_paused'])
def test_resume_waits_for_all_current_task_prerequisites(blocked):
    config, agent, health = activate()
    command, started = agent.command, agent.task_started
    agent.tick(.3, replace(health, nav_ready=False))
    now = 4. if blocked in ('sync_missing', 'peer_missing') else .4
    if blocked != 'sync_missing':
        agent.accept(sync(phase='PAUSED' if blocked == 'coordinator_paused' else 'SEARCH_AB'), now)
    state = peer(config, task_id='task', run_state='PAUSED', reason='peer paused for recovery')
    if blocked == 'peer_moving':state.stopped = False
    if blocked == 'peer_stopping':state.run_state = 'STOPPING'
    if blocked == 'peer_unhealthy':state.nav_ready = False
    if blocked != 'peer_missing':agent.receive_peer(state, now)
    local = replace(health, stopped=False) if blocked == 'moving' else (
        replace(health, motion_terminal=False) if blocked == 'motion_pending' else health)
    assert not any(e.kind == 'LOCAL_EXECUTION' for e in agent.tick(now, local))
    assert agent.paused
    agent.accept(sync(phase='SEARCH_AB'), now+.1)
    agent.receive_peer(peer(config, task_id='task', run_state='PAUSED', reason='peer paused for recovery'), now+.1)
    effects = agent.tick(now+.1, health)
    assert not agent.paused and any(e.kind == 'LOCAL_EXECUTION' for e in effects)
    assert agent.command is command and agent.task_started == started


@pytest.mark.parametrize('reason', ['peer localization/navigation invalid', 'manual stop', '',
                                  'local health invalid: nav_ready, localization_valid'])
def test_unknown_or_mixed_peer_pause_never_recovers(reason):
    config, agent, health = activate()
    agent.receive_peer(peer(config, task_id='task', run_state='PAUSED', reason=reason, nav_ready=False), .3)
    agent.tick(.3, health)
    agent.receive_peer(peer(config, task_id='task', run_state='IDLE'), .4)
    agent.accept(sync(phase='SEARCH_AB'), .4)
    agent.tick(.4, health)
    assert agent.paused and agent.recovery_state is None


@pytest.mark.parametrize('late_robot', ['go2', 'ranger'])
def test_asymmetric_resume_does_not_move_before_peer_resumes(late_robot):
    mission = independent_mission()
    for adapter in mission.adapters.values():adapter.delay = 1.
    assert mission.start().accepted
    mission.run(5)
    adapter = mission.adapters['go2']; health = adapter.get_health
    adapter.get_health = lambda: replace(health(), nav_ready=False)
    mission.run(10)
    assert all(m.paused for m in mission.machines.values())
    calls = {r:len(a.calls) for r,a in mission.adapters.items()}
    late = mission.machines[late_robot]; accept = late.accept
    def delayed(command, now):
        if command.action in ('SYNC', 'SYNC_LOCAL_C') and command.phase != 'PAUSED':
            from multi_agent_nav.models import CommandReply
            return CommandReply(False, 'fixture transport delay')
        return accept(command, now)
    late.accept = delayed
    adapter.get_health = health
    mission.run(10)
    assert late.paused
    assert all(len(a.calls) == calls[r] for r,a in mission.adapters.items())
    late.accept = accept
    mission.run(350)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason


def test_successful_pickup_is_not_repeated_after_pause():
    mission = independent_mission()
    assert mission.start().accepted
    for _ in range(100):
        mission.step()
        if all(m.state().ready for m in mission.machines.values()):break
    assert all(m.state().ready for m in mission.machines.values())
    counts = {r:sum(kind == 'PICKUP_NAV' for kind,_,_ in a.calls) for r,a in mission.adapters.items()}
    adapter = mission.adapters['go2']; health = adapter.get_health
    adapter.get_health = lambda: replace(health(), nav_ready=False)
    mission.run(10)
    adapter.get_health = health
    mission.run(300)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert all(sum(kind == 'PICKUP_NAV' for kind,_,_ in a.calls) == counts[r] for r,a in mission.adapters.items())


def test_manual_coordinator_pause_overrides_transient_pause():
    mission = independent_mission()
    assert mission.start().accepted
    mission.run(5)
    mission.online['ranger'] = False
    mission.run(40)
    mission.pause('explicit local pause')
    mission.online['ranger'] = True
    mission.run(60)
    assert mission.coordinator.phase == 'PAUSED' and mission.coordinator.resume_phase is None
    assert all(m.paused and m.recovery_state is None for m in mission.machines.values())


def test_resumed_local_execution_waits_before_entering_real_approach_generator():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    config, agent, health = activate()
    adapter = FakeAdapter('go2')
    runner = AgentRunner(agent, adapter, config['scene']['targets'], lambda _: None, config=config, clock=lambda: .4)
    started = []
    runner.start_local_execution = lambda: started.append(agent.state().run_state)
    agent.tick(.3, replace(health, nav_ready=False))
    agent.receive_peer(peer(config, task_id='task', run_state='PAUSED', reason='peer paused for recovery'), .4)
    agent.accept(sync(phase='SEARCH_AB'), .4)
    runner.dispatch(agent.tick(.4, health), .4)
    assert not agent.paused and not started
    agent.receive_peer(peer(config, task_id='task', run_state='PLANNING'), .4)
    runner.tick(.4)
    assert started == ['PLANNING']


@pytest.mark.parametrize('localization_valid', [True, False])
def test_health_loss_first_sampled_inside_execution_is_classified(localization_valid):
    from concurrent.futures import Future
    mission = independent_mission(); adapter = mission.adapters['go2']; scan = []
    pending = Future()
    adapter.start_scan = lambda *args, **kwargs: (scan.append(True) or pending)
    assert mission.start().accepted
    for _ in range(100):
        mission.step()
        if scan:break
    runner, machine = mission.runners['go2'], mission.machines['go2']
    assert runner.execution is not None and scan
    health = adapter.get_health; calls = []
    def changed():
        calls.append(1)
        return health() if len(calls) <= 2 else replace(health(), nav_ready=False, localization_valid=localization_valid)
    adapter.get_health = changed
    mission.now += .01
    runner.tick(mission.now)
    assert machine.paused
    assert (machine.recovery_state is not None) == localization_valid


def test_cleanup_failure_escalates_soft_pause_and_still_sends_stop():
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    config, agent, health = activate()
    adapter = FakeAdapter('go2'); stops = []
    adapter.stop_motion = stops.append
    runner = AgentRunner(agent, adapter, {}, lambda _:None)
    def execution():
        try:yield
        finally:raise RuntimeError('fixture cleanup failure')
    runner.execution = execution(); next(runner.execution)
    runner.dispatch(agent.tick(.3, replace(health, nav_ready=False)), .3)
    assert stops and agent.paused and agent.recovery_state is None


# Controlled ROS futures with the real Go2 adapter and approach implementation; no ROS init.
from test_single_robot_approach_parity import live_stack, navigation


def test_real_approach_generator_waits_until_peer_active(live_stack):
    stack = live_stack
    stack.m.peer_state = peer(stack.r.config, task_id='task', run_state='PAUSED', reason='peer paused for recovery')
    stack.m.awaiting_peer_resume = True
    stack.tick()
    assert not stack.plans and not stack.requests
    stack.m.receive_peer(replace(stack.m.peer_state, run_state='PLANNING', reason='', simulation=False), stack.now[0])
    stack.tick()
    stack.planned()
    assert len(stack.requests) == 1 and stack.r.approach.state != 'FAILED'


@pytest.mark.parametrize('fault', ['sync', 'peer'])
def test_missing_heartbeat_does_not_mask_transient_localization_fault(fault):
    config, agent, health = activate()
    if fault == 'peer':agent.accept(sync(phase='SEARCH_AB'), 4.)
    else:agent.receive_peer(peer(config, task_id='task'), 4.)
    agent.tick(4., replace(health, localization_valid=False))
    agent.tick(4.1, health)
    agent.receive_peer(peer(config, task_id='task'), 4.2)
    agent.accept(sync(phase='SEARCH_AB'), 4.2)
    agent.tick(4.2, health)
    assert agent.paused and agent.recovery_state is None
    assert agent.state().reason == 'local localization invalid'


def test_late_scan_result_from_cancelled_execution_cannot_supply_cached_c():
    from concurrent.futures import Future
    from multi_agent_nav.models import Observation, Pose2D
    mission = independent_mission(); adapter = mission.adapters['go2']
    original = adapter.start_scan
    pending = Future(); pending.set_running_or_notify_cancel()
    scans = []
    def slow_scan(*args, **kwargs):
        scans.append(args[1])
        return pending if len(scans) == 1 else original(*args, **kwargs)
    adapter.start_scan = slow_scan
    assert mission.start().accepted
    for _ in range(100):
        mission.step()
        if scans:break
    assert scans
    health = adapter.get_health
    adapter.get_health = lambda: replace(health(), nav_ready=False)
    mission.run(10)
    assert mission.machines['go2'].paused
    pending.set_result([Observation(observation_id='old-c', target_id='C', scan_id=scans[0],
        status='DETECTED', common_position=Pose2D(50., 50.), position_valid=True)])
    adapter.get_health = health
    mission.run(300)
    assert mission.coordinator.phase == 'DONE'
    assert 'old-c' not in mission.machines['go2'].local_c_observations
    assert 'old-c' not in mission.coordinator.observations
