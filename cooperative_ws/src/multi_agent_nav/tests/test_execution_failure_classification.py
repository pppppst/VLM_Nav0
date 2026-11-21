"""Single and cooperative execution share the same explicit retry boundary."""
from types import SimpleNamespace

import pytest

from multi_agent_nav.frame_adapter import LocalFrame
from multi_agent_nav.go2_check import standalone_explore_steps
from multi_agent_nav.go2_exploration import ExplorationStopped, NavigationNotReady
from multi_agent_nav.experiment_recovery import ExperimentRecovery
from test_experiment_recovery import fixture


@pytest.mark.parametrize('reason,cause', [
    ('selection map assets changed', None),
    ('fixture unknown failure', None),
    ('scan failed', TypeError('program bug')),
    ('initial camera inference failed', RuntimeError('unknown cause')),
])
def test_standalone_never_retries_static_identity_or_unknown_program_error(monkeypatch, reason, cause):
    adapter, config = fixture()
    error = ExplorationStopped(reason)
    error.__cause__ = cause
    def broken(*args, **kwargs):
        raise error
        yield
    monkeypatch.setattr('multi_agent_nav.go2_exploration.explore_steps', broken)
    steps = standalone_explore_steps(adapter, config, LocalFrame(), 20., 'target', lambda _: None,
        lambda: [], lambda: True, lambda: None, clock=lambda: 10.)
    with pytest.raises(ExplorationStopped, match=reason):next(steps)


@pytest.mark.parametrize('reason', ['candidate planning timeout', 'native action terminal and physical stop unconfirmed'])
def test_standalone_known_temporary_failure_resumes_same_task(monkeypatch, reason):
    adapter, config = fixture()
    calls = []
    def temporary(*args, **kwargs):
        calls.append(kwargs['task_id'])
        if len(calls) == 1:raise ExplorationStopped(reason)
        return 'TARGET_FOUND'
        yield
    monkeypatch.setattr('multi_agent_nav.go2_exploration.explore_steps', temporary)
    steps = standalone_explore_steps(adapter, config, LocalFrame(), 20., 'target', lambda _: None,
        lambda: [], lambda: True, lambda: None, clock=lambda: 10.)
    next(steps)
    with pytest.raises(StopIteration) as finished:next(steps)
    assert finished.value.value == 'TARGET_FOUND'
    assert len(calls) == 2 and calls[0] == calls[1]


def test_first_owner_seen_inside_exploration_is_latched_before_it_leaves(monkeypatch):
    adapter, config = fixture()
    owners = iter([[], ['manual_navigator']])
    def owner_source():return next(owners, [])
    recovery = ExperimentRecovery(adapter, owner_source, lambda _: None, clock=lambda: 10.)
    calls = []
    def execution(*args, **kwargs):
        calls.append(True)
        if args[6]():raise NavigationNotReady('owner appeared inside execution')
        return 'TARGET_FOUND'
        yield
    monkeypatch.setattr('multi_agent_nav.go2_exploration.explore_steps', execution)
    steps = standalone_explore_steps(adapter, config, LocalFrame(), 20., 'target', lambda _: None,
        owner_source, lambda: True, lambda: None, clock=lambda: 10., recovery=recovery)
    next(steps)
    assert recovery.hard_reason == 'external goal owner takeover'
    next(steps)
    assert len(calls) == 1 and recovery.recovering
    steps.close()


@pytest.mark.parametrize('reason,recoverable', [
    ('APPROACH_FAILED: Nav2 cannot plan to any approach candidate; last rejection: empty_path', True),
    ('APPROACH_FAILED: Nav2 cannot plan to any approach candidate; last rejection: planner_action_status_6', True),
    ('APPROACH_FAILED: approach goal failed with status 6', True),
    ('APPROACH_FAILED: approach goal failed with status 5', False),
    ('APPROACH_FAILED: Nav2 cannot plan to any approach candidate; last rejection: unknown_payload', False),
])
def test_confirmed_terminal_approach_failures_have_an_explicit_retry_boundary(reason, recoverable):
    from multi_agent_nav.go2_exploration import is_recoverable_execution_error
    assert is_recoverable_execution_error(ExplorationStopped(reason)) == recoverable
