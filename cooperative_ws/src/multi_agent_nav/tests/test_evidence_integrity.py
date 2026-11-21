"""Evidence acceptance is independent of physical completion and optional display."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.models import Effect
from multi_agent_nav.runner import AgentRunner
from test_startup_handshake import activate


@pytest.fixture
def recorded_runner(monkeypatch, tmp_path):
    config, machine, _ = activate()
    machine.context = replace(machine.context, simulation=False)
    config['robot_go2']['experiment_records_root'] = str(tmp_path)
    adapter = FakeAdapter('go2')
    adapter.simulation = False
    adapter.node = S(get_logger=lambda: S(warning=lambda *args, **kwargs: None))
    recorded = {}
    def exploration(*args, **kwargs):
        recorded['record'] = args[5]
        return (value for value in ())
    monkeypatch.setattr('multi_agent_nav.go2_exploration.explore_steps', exploration)
    runner = AgentRunner(machine, adapter, {'A': 'target'}, lambda _: None,
        config=config, clock=lambda: .4)
    runner.start_local_execution()
    return runner, recorded['record'], tmp_path / 'command' / 'events.jsonl'


@pytest.mark.parametrize('error', [OSError('disk full'), ValueError('closed stream')])
def test_required_event_failure_is_latched_even_after_later_write_succeeds(recorded_runner, monkeypatch, error):
    runner, record, path = recorded_runner
    original = Path.open
    def fail(target, *args, **kwargs):
        if target == path:raise error
        return original(target, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', fail)
    record(dict(state='SCANNING'))
    assert runner.adapter.evidence_write_failed and not runner.evidence_complete
    monkeypatch.setattr(Path, 'open', original)
    record(dict(state='TARGET_FOUND'))
    assert 'TARGET_FOUND' in path.read_text() and not runner.evidence_complete
    assert json.loads(path.read_text().splitlines()[-1])['evidence_write_failed'] is True
    assert not runner.machine.paused


def test_optional_display_failure_does_not_block_required_event_or_stop(recorded_runner):
    runner, record, path = recorded_runner
    def broken_display(event):raise RuntimeError('display unavailable')
    runner.on_event = broken_display
    record(dict(state='SCANNING'))
    assert 'SCANNING' in path.read_text() and runner.evidence_complete
    stops = []
    runner.adapter.stop_motion = stops.append
    runner.dispatch([Effect('STOP', ('', 'operator stop'))], .4)
    assert stops == ['operator stop']
    assert 'EXECUTION_STOP' in path.read_text()
    assert runner.evidence_complete


def test_required_event_serialization_failure_is_not_only_a_warning(recorded_runner):
    runner, record, _ = recorded_runner
    record(dict(state='SCANNING', invalid_payload=object()))
    assert runner.adapter.evidence_write_failed and not runner.evidence_complete


def test_agent_reports_incomplete_evidence_without_rewriting_physical_success(recorded_runner, monkeypatch):
    from multi_agent_nav.agent_node import AgentNode
    runner, record, _ = recorded_runner
    runner.adapter.evidence_write_failed = True
    runner.machine._state.reply = 'SUCCEEDED'
    runner.machine.task_completed = True
    node = AgentNode.__new__(AgentNode)
    node.machine, node.runner = runner.machine, runner
    states, statuses = [], []
    node.state_pub = S(publish=states.append)
    node.experiment_status = S(update=lambda **fields: statuses.append(fields))
    node.get_clock = lambda: S(now=lambda: S(nanoseconds=400000000))
    monkeypatch.setattr('multi_agent_nav.agent_node.time.monotonic', lambda: .4)
    node.publish_state()
    assert states[-1].reply == 'SUCCEEDED'
    assert statuses[-1]['state'] == 'INCOMPLETE'
    assert statuses[-1]['result'] == 'SUCCEEDED'
    assert statuses[-1]['evidence_complete'] is False


@pytest.mark.parametrize('display', ['record', 'marker'])
def test_standalone_display_failure_cannot_block_archive_or_final_stop(monkeypatch, tmp_path, display):
    from test_go2_exploration import test_exploration_entry_publishes_and_logs_before_destroying_node
    def unavailable(*args, **kwargs):raise RuntimeError('display unavailable')
    target = ('multi_agent_nav.exploration_display.ExplorationDisplay.record' if display=='record' else
        'multi_agent_nav.go2_exploration.exploration_markers')
    monkeypatch.setattr(target, unavailable)
    test_exploration_entry_publishes_and_logs_before_destroying_node(monkeypatch, tmp_path, True, display_failure=True)


def test_status_keeps_physical_success_and_evidence_gap_as_a_terminal_result(tmp_path):
    from multi_agent_nav.experiment_status import ExperimentStatus, format_status
    status = ExperimentStatus(tmp_path, clock=lambda: 10.)
    assert status.update(state='INCOMPLETE', result='SUCCEEDED', evidence_complete=False,
        evidence_write_failed=True, reason_class='EVIDENCE_INCOMPLETE')
    rendered = format_status(status.snapshot, 100.)
    assert '最终结果：SUCCEEDED' in rendered and '必要证据完整：否' in rendered
    assert '状态已过期' not in rendered


def test_optional_path_publisher_is_isolated_from_evidence_and_stop(recorded_runner):
    runner, _, _ = recorded_runner
    def unavailable(*args):raise OSError('path publisher unavailable')
    runner.display_call(unavailable, 'planned_path', object())
    assert runner.display_failed and runner.evidence_complete
