"""Read-only diagnostics never control the experiment or expose its config."""
import importlib
import json
from pathlib import Path
import sys

import pytest


def snapshot(state='RUNNING'):
    return dict(state=state, reason='定位恢复', updated_monotonic=100.,
                run_elapsed_s=10., pause_elapsed_s=2., paused_total_s=7.,
                run_remaining_s=90., health=dict(nav_ready=True, localization_valid=False))


def test_running_advances_only_running_budget():
    status = importlib.import_module('multi_agent_nav.experiment_status')
    rendered = status.format_status(snapshot(), 102.)
    assert '运行时间：12.0 秒' in rendered
    assert '本次暂停：2.0 秒' in rendered
    assert '累计暂停：7.0 秒' in rendered
    assert '剩余运行：88.0 秒' in rendered


def test_waiting_advances_only_pause_timers():
    status = importlib.import_module('multi_agent_nav.experiment_status')
    rendered = status.format_status(snapshot('WAITING'), 102.)
    assert '运行时间：10.0 秒' in rendered
    assert '本次暂停：4.0 秒' in rendered
    assert '累计暂停：9.0 秒' in rendered
    assert '剩余运行：90.0 秒' in rendered


def test_final_and_stale_states_do_not_guess_elapsed_time():
    status = importlib.import_module('multi_agent_nav.experiment_status')
    final = dict(snapshot('FINISHED'), result='完成', stop_confirmed=True)
    assert status.format_status(final, 102.) == status.format_status(final, 900.)
    assert '停车确认：是' in status.format_status(final, 900.)
    rendered = status.format_status(snapshot(), 110.)
    assert '状态已过期' in rendered
    assert '运行时间：10.0 秒' in rendered


def test_allowlist_and_control_characters_are_enforced():
    status = importlib.import_module('multi_agent_nav.experiment_status')
    value = dict(snapshot(), api_key='secret-config', config={'token': 'secret-config'},
                 reason='等待\x1b[31m\n\r\x07定位', result={'secret': 'secret-config'})
    value['health']['api_key'] = 'secret-config'
    rendered = status.format_status(value, 101.)
    assert 'secret-config' not in rendered
    assert '\x1b' not in rendered and '\r' not in rendered and '\x07' not in rendered
    assert '导航就绪：是' in rendered and '定位有效：否' in rendered
    assert '控制已解锁：未知' in rendered


@pytest.mark.parametrize('value', [None, [], {'health': None, 'run_elapsed_s': 'bad'},
                                         {'updated_monotonic': float('nan')},
                                         {'run_elapsed_s': 10 ** 400}])
def test_malformed_snapshots_stay_readable(value):
    status = importlib.import_module('multi_agent_nav.experiment_status')
    assert '状态：' in status.format_status(value, 102.)


def test_budget_never_goes_negative_and_nonboolean_health_is_unknown():
    status = importlib.import_module('multi_agent_nav.experiment_status')
    value = dict(snapshot(), run_remaining_s=1., phase='扫描',
                 health={'control_armed': 'false'})
    rendered = status.format_status(value, 102.)
    assert '剩余运行：0.0 秒' in rendered
    assert '阶段：扫描' in rendered
    assert '控制已解锁：未知' in rendered


def test_update_publishes_atomic_json_and_retains_fields(tmp_path, monkeypatch):
    status = importlib.import_module('multi_agent_nav.experiment_status')
    report = status.ExperimentStatus(tmp_path / 'records', clock=lambda: 100.)
    assert report.update(state='RUNNING', run_elapsed_s=10., config='secret-config')
    target = tmp_path / 'records' / 'status.json'
    original_replace = status.os.replace
    observed = []
    def replace(source, destination):
        observed.append(json.loads(target.read_text())['state'])
        assert json.loads(Path(source).read_text())['state'] == 'WAITING'
        original_replace(source, destination)
    monkeypatch.setattr(status.os, 'replace', replace)
    assert report.update(state='WAITING')
    assert observed == ['RUNNING']
    result = json.loads(target.read_text())
    assert result['run_elapsed_s'] == 10. and result['updated_monotonic'] == 100.
    assert 'secret-config' not in target.read_text()
    assert sorted(p.name for p in target.parent.iterdir()) == ['status.json']


def test_write_failure_preserves_previous_snapshot_and_returns_false(tmp_path, monkeypatch):
    status = importlib.import_module('multi_agent_nav.experiment_status')
    report = status.ExperimentStatus(tmp_path)
    assert report.update(state='RUNNING')
    original = (tmp_path / 'status.json').read_bytes()
    def fail(*args):
        raise OSError('read-only filesystem')
    monkeypatch.setattr(status.os, 'replace', fail)
    assert report.update(state='WAITING') is False
    assert (tmp_path / 'status.json').read_bytes() == original
    assert sorted(p.name for p in tmp_path.iterdir()) == ['status.json']


def test_launch_uses_literal_arguments_and_failure_is_nonfatal(tmp_path, monkeypatch):
    status = importlib.import_module('multi_agent_nav.experiment_status')
    calls = []
    def launch(argv, **kwargs):
        calls.append((argv, kwargs))
    monkeypatch.setattr(status.subprocess, 'Popen', launch)
    report = status.ExperimentStatus(tmp_path / 'space ; $HOME')
    assert report.launch()
    argv, kwargs = calls[0]
    assert argv == ['gnome-terminal', '--window', '--title=Go2 实验状态', '--',
                    sys.executable, str(Path(status.__file__).resolve()), '--watch',
                    str((tmp_path / 'space ; $HOME' / 'status.json').resolve())]
    assert kwargs['shell'] is False and kwargs['start_new_session'] is True
    def fail(*args, **kwargs):
        raise FileNotFoundError('gnome-terminal')
    monkeypatch.setattr(status.subprocess, 'Popen', fail)
    assert report.launch() is False


def test_watch_tolerates_missing_and_partial_files_and_keeps_final_visible(tmp_path, monkeypatch, capsys):
    status = importlib.import_module('multi_agent_nav.experiment_status')
    path = tmp_path / 'status.json'
    ticks = []
    def refresh(seconds):
        assert seconds == .5
        ticks.append(seconds)
        if len(ticks) == 1:
            path.write_text('{')
        elif len(ticks) == 2:
            path.write_text(json.dumps(dict(snapshot('FINISHED'), result='完成')))
        elif len(ticks) == 4:
            raise KeyboardInterrupt
    monkeypatch.setattr(status.time, 'sleep', refresh)
    status.watch(path)
    output = capsys.readouterr().out
    assert len(ticks) == 4
    assert output.count('最终结果：完成') == 2
    assert '等待状态文件' in output
