"""Read-only status display with fake ROS and clocks; no live subscriptions."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace as S

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def viewer():
    spec = importlib.util.spec_from_file_location('show_go2_status', ROOT/'scripts/show_go2_status.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def go2(**updates):
    values = dict(robot_id='go2', schema_version=1, simulation=False, task_id='task',
                  run_state='PAUSED', reason='peer heartbeat lost for current task',
                  localization_valid=True, nav_ready=True, stopped=True)
    return S(**dict(values, **updates))


def coordinator(**updates):
    return json.dumps(dict(dict(task_id='task', phase='PAUSED',
                               reason='go2: agent paused: peer heartbeat lost for current task'), **updates))


def test_initial_display_waits_without_inventing_state(viewer):
    text = viewer.StatusDisplay(3.).render(0.)
    assert 'Go2 当前状态' in text and '等待消息' in text
    assert 'PAUSED' not in text and '新鲜' not in text and '定位有效：未知' in text


@pytest.mark.parametrize('state', ['STANDBY', 'SYNCED', 'NAVIGATING', 'PAUSED'])
def test_displays_complete_status_and_reason(viewer, state):
    display = viewer.StatusDisplay(3.)
    reason = 'peer heartbeat lost for current task; '+('完整原因' * 80)
    display.receive_go2(go2(run_state=state, reason=reason), 0.)
    display.receive_coordinator(coordinator(), 0.)
    text = display.render(1.)
    assert f'Go2：{state}' in text and reason in text
    assert '任务阶段：PAUSED' in text and '协调器原因：go2: agent paused:' in text
    assert '定位有效：是' in text and '导航就绪：是' in text and '停稳报告：是' in text
    assert 'Go2消息：新鲜' in text and '协调器消息：新鲜' in text
    assert 'task' in text


def test_empty_reason_and_false_health_flags(viewer):
    display = viewer.StatusDisplay(3.)
    display.receive_go2(go2(reason='', localization_valid=False, nav_ready=False, stopped=False), 0.)
    display.receive_coordinator(coordinator(reason=''), 0.)
    text = display.render(0.)
    assert '原因：无' in text and '协调器原因：无' in text
    assert '定位有效：否' in text and '导航就绪：否' in text and '停稳报告：否' in text


def test_each_source_expires_independently_and_recovers_without_changing_report(viewer):
    display = viewer.StatusDisplay(.75)
    display.receive_go2(go2(), 0.)
    display.receive_coordinator(coordinator(), .5)
    assert 'Go2消息：新鲜' in display.render(.75)
    text = display.render(1.)
    assert 'Go2消息：消息已过期' in text and '协调器消息：新鲜' in text
    assert 'Go2：PAUSED（最后报告）' in text
    text = display.render(1.5)
    assert '任务阶段：PAUSED（最后报告）' in text
    display.receive_go2(go2(), 2.)
    assert 'Go2消息：新鲜' in display.render(2.)
    assert 'Go2：PAUSED\n' in display.render(2.)


def test_different_task_ids_remain_separate(viewer):
    display = viewer.StatusDisplay(3.)
    display.receive_go2(go2(task_id='old-task'), 0.)
    display.receive_coordinator(coordinator(task_id='new-task'), 0.)
    text = display.render(0.)
    assert 'Go2任务ID：old-task' in text and '协调器任务ID：new-task' in text
    assert '任务ID不一致' in text


@pytest.mark.parametrize('raw', ['not json', 'null', '[]', '{}',
                               '{"task_id":"task","phase":1,"reason":""}'])
def test_bad_coordinator_message_does_not_refresh_last_valid_report(viewer, raw):
    display = viewer.StatusDisplay(3.)
    display.receive_coordinator(coordinator(), 0.)
    display.receive_coordinator(raw, 4.)
    text = display.render(4.)
    assert '协调器消息格式错误' in text and '协调器消息：消息已过期' in text
    assert '任务阶段：PAUSED（最后报告）' in text
    display.receive_coordinator(coordinator(phase='SEARCH_AB', reason=''), 5.)
    assert '格式错误' not in display.render(5.) and '任务阶段：SEARCH_AB' in display.render(5.)


@pytest.mark.parametrize('changes', [dict(robot_id='ranger'), dict(simulation=True), dict(schema_version=2)])
def test_wrong_agent_identity_is_visible_and_does_not_refresh_valid_report(viewer, changes):
    display = viewer.StatusDisplay(3.)
    display.receive_go2(go2(), 0.)
    display.receive_go2(go2(**changes), 4.)
    text = display.render(4.)
    assert 'Go2消息身份不符' in text and 'Go2消息：消息已过期' in text
    display.receive_go2(go2(), 5.)
    assert '身份不符' not in display.render(5.)


def test_prints_on_change_and_every_five_seconds_without_clearing_history(viewer, capsys):
    display = viewer.StatusDisplay(3.)
    display.emit(0.)
    first = capsys.readouterr().out
    assert first.startswith('[') and 'Go2 当前状态' in first and '\x1b' not in first
    display.emit(.5)
    assert not capsys.readouterr().out
    display.receive_go2(go2(), 1.)
    display.emit(1.)
    assert 'PAUSED' in capsys.readouterr().out
    display.receive_go2(go2(), 3.)
    display.emit(3.)
    assert not capsys.readouterr().out
    display.receive_go2(go2(), 6.)
    display.emit(6.)
    assert 'PAUSED' in capsys.readouterr().out
    display.emit(9.5)
    assert '消息已过期' in capsys.readouterr().out


@pytest.mark.parametrize('interrupt_at', ['spin', 'setup'])
def test_runtime_uses_only_two_readonly_subscriptions_and_cleans_up(viewer, monkeypatch, capsys, interrupt_at):
    lifecycle, subscriptions, timers, options = [], {}, [], {}
    clock = [0.]
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: clock[0])
    monkeypatch.setenv('COOP_CONFIG', '/fake/config')

    def create_subscription(cls, topic, callback, qos):
        if interrupt_at == 'setup':
            raise KeyboardInterrupt
        subscriptions[topic] = (callback, qos)

    node = S(create_subscription=create_subscription,
             create_timer=lambda interval, callback: timers.append((interval, callback)),
             destroy_node=lambda: lifecycle.append('destroy'))

    def create_node(name, **kwargs):
        options.update(kwargs)
        return node

    def spin(node):
        callback, qos = subscriptions['/multi_agent/go2/state']
        assert qos == dict(depth=10, reliability='reliable', durability='volatile')
        callback(go2())
        assert 'PAUSED' in capsys.readouterr().out  # callbacks print immediately
        subscriptions['/multi_agent/coordinator/status'][0](S(data=coordinator()))
        assert timers[0][0] == .5
        clock[0] = 1.
        timers[0][1]()
        assert '消息已过期' in capsys.readouterr().out  # configured .75s, not hardcoded 3s
        raise KeyboardInterrupt

    monkeypatch.setitem(sys.modules, 'rclpy', S(init=lambda **kw:lifecycle.append('init'),
        create_node=create_node, spin=spin, ok=lambda:True, shutdown=lambda:lifecycle.append('shutdown')))
    monkeypatch.setitem(sys.modules, 'rclpy.signals', S(SignalHandlerOptions=S(NO=0)))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', S(QoSProfile=lambda **kw:kw,
        ReliabilityPolicy=S(RELIABLE='reliable'), DurabilityPolicy=S(VOLATILE='volatile')))
    monkeypatch.setitem(sys.modules, 'multi_agent_task_msgs.msg', S(AgentState=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', S(String=object))
    monkeypatch.setitem(sys.modules, 'multi_agent_nav.config', S(load_config=lambda directory:
        dict(scene=dict(lost_timeout=.75))))
    viewer.main()
    assert lifecycle == ['init', 'destroy', 'shutdown']
    assert options == dict(enable_rosout=False, start_parameter_services=False)
    if interrupt_at == 'spin':
        assert set(subscriptions) == {'/multi_agent/go2/state', '/multi_agent/coordinator/status'}


def test_watch_shell_routes_to_viewer_without_start_or_preflight(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copy2(ROOT/'scripts/cooperative.sh', scripts/'cooperative.sh')
    (tmp_path/'Cooperative_env.bash').write_text('export COOP_CONFIG=/fake/config\n')
    (scripts/'show_go2_status.py').write_text('print("READ_ONLY_STATUS")\n')
    binary = tmp_path/'bin'
    binary.mkdir()
    ros2 = binary/'ros2'
    ros2.write_text('#!/bin/sh\necho WRONG_RAW_WATCH\n')
    ros2.chmod(0o755)
    result = subprocess.run(['bash', str(scripts/'cooperative.sh'), 'go2', 'watch'],
                            capture_output=True, text=True,
                            env=dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH']))
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'READ_ONLY_STATUS'
