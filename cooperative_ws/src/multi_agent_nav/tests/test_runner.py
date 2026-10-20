from concurrent.futures import Future
from dataclasses import replace

import pytest

from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.models import GoalResult, Observation
from multi_agent_nav.runner import AgentRunner
from test_agent import command, machine


class ControlledFake(FakeAdapter):
    def __init__(self):
        super().__init__('go2', delay=10.)
        self.health_changes = {}
        self.health_hook = None

    def get_health(self):
        health = replace(super().get_health(), **self.health_changes)
        if self.health_hook:
            hook, self.health_hook = self.health_hook, None
            hook()
        return health


@pytest.fixture
def execution():
    agent, adapter, now, observations = machine(), ControlledFake(), [0.], []
    agent.tick(0., adapter.get_health())
    runner = AgentRunner(agent, adapter, {t: t for t in 'ABC'}, observations.append)
    runner.clock = lambda: now[0]
    return agent, adapter, runner, now, observations


def step(execution, when):
    agent, adapter, runner, now, observations = execution
    now[0] = when
    runner.tick(when)


@pytest.mark.parametrize('change', [{'stopped': False}, {'motion_terminal': False}])
def test_preflight_waits_for_real_stop_after_planning(execution, change):
    agent, adapter, runner, now, _ = execution
    assert agent.accept(command('WAIT'), 0.).accepted
    on_path = agent.on_path
    def unsettled(*args):
        on_path(*args)
        adapter.health_changes.update(change)
    agent.on_path = unsettled
    step(execution, 0.)
    step(execution, .1)
    assert not adapter.calls and not agent.paused
    assert not agent.state().stopped or not agent.health.motion_terminal
    step(execution, .2)
    assert not adapter.calls
    agent.on_path = on_path
    adapter.health_changes.clear()
    for when in (.3, .4, .5):
        step(execution, when)
    assert len(adapter.calls) == 1
    assert agent.state().run_state == 'NAVIGATING' and not agent.state().stopped
    step(execution, .6)
    assert not agent.paused  # Stop is a start gate, not a condition during navigation.


@pytest.mark.parametrize('change', [
    {'localization_valid': False}, {'nav_ready': False},
])
def test_health_change_after_planning_blocks_first_navigation(execution, change):
    agent, adapter, runner, now, _ = execution
    assert agent.accept(command('WAIT'), 0.).accepted
    on_path = agent.on_path
    def invalidated(*args):
        on_path(*args)
        adapter.health_changes.update(change)
    agent.on_path = invalidated
    step(execution, 0.)
    step(execution, .1)
    assert not adapter.calls and agent.paused
    assert agent.state().run_state in ('STOPPING', 'PAUSED')


@pytest.mark.parametrize('reason', ['cancel', 'communication_timeout'])
def test_cancel_or_timeout_during_preflight_cannot_send_goal(execution, reason):
    agent, adapter, runner, now, _ = execution
    assert agent.accept(command('WAIT'), 0.).accepted
    on_path = agent.on_path
    def preflight(*args):
        on_path(*args)
        def invalidate():
            if reason == 'cancel':
                agent.accept(replace(command(), action='CANCEL', cancel_command_id='c1'), now[0])
            else:
                now[0] = 4.
        adapter.health_hook = invalidate
    agent.on_path = preflight
    step(execution, 0.)
    step(execution, .1)
    assert not adapter.calls and agent.paused


def test_timeout_precedes_late_navigation_success_and_recovery(execution):
    agent, adapter, runner, now, _ = execution
    assert agent.accept(command(), 0.).accepted
    step(execution, 0.)
    step(execution, .1)
    step(execution, .2)
    assert len(adapter.calls) == 1
    state = agent.state()
    adapter.results.append(GoalResult('t', 'c1', state.goal_id, 'PICKUP_NAV', 'SUCCEEDED'))
    step(execution, 4.)
    assert agent.paused and not agent.state().ready
    assert agent.state().run_state == 'STOPPING'
    agent.peer_received = 4.1
    agent.accept(replace(command(), action='SYNC'), 4.1)
    step(execution, 4.1)
    assert agent.state().run_state == 'PAUSED' and not agent.state().ready
    assert len(adapter.calls) == 1


def test_cancel_during_native_send_cannot_overwrite_stopping_state(execution):
    agent, adapter, runner, now, _ = execution
    send = adapter.start_navigation
    def cancel_during_send(*args):
        goal_id = send(*args)
        agent.accept(replace(command(), action='CANCEL', cancel_command_id='c1'), now[0])
        return goal_id
    adapter.start_navigation = cancel_during_send
    assert agent.accept(command('WAIT'), 0.).accepted
    step(execution, 0.)
    step(execution, .1)
    assert agent.paused and agent.state().run_state == 'STOPPING'
    assert agent.state().reply == 'CANCELED'
    step(execution, .2)
    assert agent.state().run_state == 'PAUSED'
    assert len(adapter.calls) == 1


@pytest.mark.parametrize('during_health_check', [False, True])
def test_timeout_stops_scan_before_next_spin_and_discards_late_vlm(execution, during_health_check):
    agent, adapter, runner, now, observations = execution
    adapter.delay = .1
    inference = Future()
    inference.set_running_or_notify_cancel()
    spins, scanning = [], [False]
    def scan(*args):
        scanning[0] = True
        return inference
    adapter.start_scan = scan
    adapter.tick = lambda: spins.append(now[0]) if scanning[0] else None
    stop_motion = adapter.stop_motion
    def stop(reason):
        scanning[0] = False
        return stop_motion(reason)
    adapter.stop_motion = stop
    assert agent.accept(command('SEARCH_VIEWPOINT'), 0.).accepted
    for when in (0., .1, .2):
        step(execution, when)
    assert scanning[0]
    count = len(spins)
    agent._state.resource_id = 'basket'
    agent._state.token = 'held'
    if during_health_check:
        adapter.health_hook = lambda: now.__setitem__(0, 4.)
    step(execution, .3 if during_health_check else 4.)
    assert agent.paused and len(spins) == count
    inference.set_result([Observation(target_id=t, scan_id='c1', observation_id=t) for t in 'ABC'])
    agent.peer_received = 4.1
    agent.accept(replace(command(), action='SYNC'), 4.1)
    step(execution, 4.1)
    assert agent.state().run_state == 'PAUSED'
    assert not observations and not agent.state().ready and not agent.state().deposited
    assert agent.state().token == 'held'
    assert len(adapter.calls) == 1 and len(spins) == count


def test_generator_cleanup_failure_cannot_skip_physical_stop(execution):
    from multi_agent_nav.models import Effect
    agent,adapter,runner,now,_=execution
    stopped=[]
    def cleanup_failure():
        try:yield
        finally:raise RuntimeError('fixture evidence write failed')
    runner.execution=cleanup_failure();next(runner.execution)
    adapter.stop_motion=stopped.append
    runner.dispatch([Effect('STOP',('', 'cancel'))],now[0])
    assert stopped and runner.execution is None


def test_local_executor_setup_failure_reports_stop(execution):
    from multi_agent_nav.models import Effect
    agent,adapter,runner,now,_=execution
    assert agent.accept(command('WAIT'),0).accepted
    def setup_failure():raise OSError('fixture records directory unavailable')
    runner.start_local_execution=setup_failure
    runner.dispatch([Effect('LOCAL_EXECUTION')],now[0])
    assert agent.paused and agent.state().run_state=='STOPPING'
