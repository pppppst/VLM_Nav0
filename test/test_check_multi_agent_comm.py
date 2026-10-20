"""PAUSED task communication checks; no real ROS initialization or networking."""
import importlib.util
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('comm', ROOT / 'scripts/check_multi_agent_comm.py')
comm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comm)


def message(**changes):
    values = dict(schema_version=1, robot_id='ranger', stamp=100., sequence=0,
                  simulation=False, run_state='PAUSED', reason='live configuration gates closed',
                  ready=False, deposited=False, local_map_id='', map_version='',
                  common_frame_id='lab_ref', alignment_id='', pose_valid=False,
                  localization_valid=False, stopped=True)
    values.update(changes)
    return SimpleNamespace(**values)


def evidence():
    result = comm.StateEvidence('ranger')
    for stamp, received in [(100., 1.), (100.5, 1.5), (101., 2.)]:
        result.observe(message(stamp=stamp), received)
    return result


def test_sequence_zero_and_invalid_pose_are_valid_in_blocked_branch():
    assert evidence().issues(2.2, 3.) == []


@pytest.mark.parametrize('changes', [dict(schema_version=2), dict(robot_id='go2'),
    dict(simulation=True), dict(run_state='IDLE'), dict(reason=''), dict(ready=True),
    dict(deposited=True), dict(stamp=float('nan')), dict(stamp=0)])
def test_wrong_or_active_state_is_rejected_even_after_good_samples(changes):
    result = evidence()
    result.observe(message(**changes), 2.1)
    result.observe(message(stamp=102), 2.2)
    assert result.issues(2.3, 3.)


def test_repeated_backward_and_missing_stamps_do_not_pass():
    for stamps in [[], [100, 100, 100], [100, 99, 101]]:
        result = comm.StateEvidence('ranger')
        for stamp in stamps:
            result.observe(message(stamp=stamp), 2.)
        assert result.issues(2.1, 3.)


def test_receive_timeout_and_long_gap_fail():
    result = evidence()
    assert result.issues(6., 3.)
    result.observe(message(stamp=105.), 6.)
    assert result.issues(6.1, 3.)


def test_sequence_may_stay_zero_but_cannot_go_backwards():
    result = evidence()
    result.observe(message(stamp=102., sequence=1), 2.1)
    result.observe(message(stamp=103., sequence=0), 2.2)
    assert result.issues(2.3, 3.)


@pytest.mark.parametrize('side', ['go2', 'ranger'])
def test_current_five_task_routes_are_accepted(side):
    data = yaml.safe_load((ROOT / f'cooperative_ws/src/multi_agent_nav/config/bridge_{side}_pending.yaml').read_text())
    assert comm.validate_bridge(data, side) == f'{side}_cooperative_task_bridge'


@pytest.mark.parametrize('change', ['old_topic', 'wrong_direction', 'extra_cmd_vel',
                                   'best_effort', 'bidirectional', 'wrong_domain'])
def test_unsafe_or_legacy_bridge_routes_are_rejected(change):
    data = yaml.safe_load((ROOT / 'cooperative_ws/src/multi_agent_nav/config/bridge_ranger_pending.yaml').read_text())
    topic = data['topics']['/multi_agent/ranger/state']
    if change == 'old_topic':
        data['topics']['/multi_agent/agents/ranger/state'] = data['topics'].pop('/multi_agent/ranger/state')
    elif change == 'wrong_direction':
        topic['reversed'] = True
    elif change == 'extra_cmd_vel':
        data['topics']['/cmd_vel'] = dict(type='geometry_msgs/msg/Twist')
    elif change == 'best_effort':
        topic['qos']['reliability'] = 'best_effort'
    elif change == 'bidirectional':
        topic['bidirectional'] = True
    else:
        data['from_domain'] = 31
    with pytest.raises(ValueError):
        comm.validate_bridge(data, 'ranger')


def xml(domains):
    return '<CycloneDDS xmlns="https://cdds.io/config">' + ''.join(
        f'<Domain Id="{domain}"><General><Interfaces><NetworkInterface name="{nic}"/>'
        '</Interfaces></General></Domain>' for domain, nic in domains.items()) + '</CycloneDDS>'


def test_ranger_uses_actual_interfaces_and_rejects_local_wifi_leak(tmp_path):
    local = tmp_path / 'local.xml'
    bridge = tmp_path / 'bridge.xml'
    local.write_text(xml({'any': 'eno2'}))
    bridge.write_text(xml({'0': 'eno2', '40': 'wlp2s0'}))
    assert comm.validate_dds(local, bridge, 'ranger') == {'0': 'eno2', '40': 'wlp2s0'}
    local.write_text(xml({'any': 'wlp2s0'}))
    with pytest.raises(ValueError):
        comm.validate_dds(local, bridge, 'ranger')


def test_actual_go2_dds_and_variable_expansion(monkeypatch):
    monkeypatch.setenv('GO2_NET_IFACE', 'enp4s0')
    assert comm.validate_dds(ROOT / 'config/cyclonedds_go2.xml',
        ROOT / 'config/cyclonedds_go2_multi_agent_bridge.xml', 'go2') == {'0': 'enp4s0', '40': 'wlo1'}


def test_domain_zero_can_use_a_file_that_also_defines_other_domains():
    path = ROOT / 'config/cyclonedds_go2_multi_agent_bridge.xml'
    assert comm.validate_dds(path, path, 'go2') == {'0': 'enp4s0', '40': 'wlo1'}


def publisher(name, reliability='RELIABLE'):
    return SimpleNamespace(node_name=name, node_namespace='/',
        topic_type='multi_agent_task_msgs/msg/AgentState', qos_profile=SimpleNamespace(
            reliability=SimpleNamespace(name=reliability), durability=SimpleNamespace(name='VOLATILE')))


def test_peer_requires_local_import_bridge_not_direct_or_replay_node():
    assert comm.publisher_issues([publisher('ranger_cooperative_task_bridge_0')],
                                'ranger_cooperative_task_bridge', bridge=True) == []
    for publishers in [[], [publisher('local_replay_node')], [publisher('multi_agent_go2_agent')],
                       [publisher('ranger_cooperative_task_bridge_0', 'BEST_EFFORT')]]:
        assert comm.publisher_issues(publishers, 'ranger_cooperative_task_bridge', bridge=True)


@pytest.mark.parametrize('side', ['go2', 'ranger'])
def test_launch_commands_match_document(side, tmp_path):
    agent = comm.launch_command('agent', side, tmp_path)
    assert agent[:4] == ['ros2', 'run', 'multi_agent_nav', 'multi_agent_agent_node']
    assert f'__node:=multi_agent_{side}_agent' in agent and f'robot_id:={side}' in agent
    assert 'simulation:=false' in agent and 'topic_prefix:=/multi_agent' in agent
    assert f'config_dir:={tmp_path}' in agent
    assert comm.launch_command('bridge', side, tmp_path) == [
        'ros2', 'run', 'domain_bridge', 'domain_bridge', str(tmp_path / f'bridge_{side}_pending.yaml')]


@pytest.mark.parametrize('side', ['go2', 'ranger'])
def test_help_needs_no_ros_or_environment(side, tmp_path):
    env = dict(os.environ, COMM_ENV_FILE=str(tmp_path / 'missing.bash'))
    result = subprocess.run(['bash', str(ROOT / f'scripts/check_{side}_comm.sh'), '--help'],
                            env=env, cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'PAUSED' in result.stdout and 'preflight' in result.stdout


@pytest.mark.parametrize('side', ['go2', 'ranger'])
def test_wrapper_loads_selected_environment_and_preserves_failure(side, tmp_path):
    setup = tmp_path / 'environment.bash'
    setup.write_text('echo LOADED_COMM_ENV\nreturn 19\n')
    env = dict(os.environ, COMM_ENV_FILE=str(setup))
    result = subprocess.run(['bash', str(ROOT / f'scripts/check_{side}_comm.sh'), 'agent'],
                            env=env, capture_output=True, text=True)
    assert result.returncode != 0 and 'LOADED_COMM_ENV' in result.stdout


def test_open_gates_refuse_paused_agent_launch(monkeypatch, tmp_path):
    import sys
    fake = SimpleNamespace(load_config=lambda path: {'scene': {}}, live_gates=lambda *a, **kw: [])
    monkeypatch.setitem(sys.modules, 'multi_agent_nav.config', fake)
    with pytest.raises(ValueError, match='gates'):
        comm.blocked_config(tmp_path, 'ranger')


@pytest.mark.parametrize('side,scenario,expected', [
    ('go2', 'good', 0), ('ranger', 'good', 0), ('go2', 'missing_peer', 1),
    ('ranger', 'replay_peer', 1), ('ranger', 'duplicate_peer', 1),
    ('ranger', 'best_effort', 1), ('go2', 'active_peer', 1),
    ('ranger', 'stale_local', 1), ('go2', 'stop_peer', 0),
    ('ranger', 'no_stop', 1), ('go2', 'active_during_stop', 1),
    ('ranger', 'missing_peer_stop', 1),
])
def test_monitor_with_fake_ros_only(monkeypatch, capsys, side, scenario, expected):
    import sys
    clock = [0.]
    callbacks = {}
    running = [False]
    peer = 'ranger' if side == 'go2' else 'go2'
    bridge_name = f'{side}_cooperative_task_bridge'
    stop_check = scenario in ('stop_peer', 'no_stop', 'active_during_stop', 'missing_peer_stop')

    def spin_once(node, timeout_sec):
        clock[0] += timeout_sec
        for robot, callback in callbacks.items():
            if robot == peer and (scenario in ('missing_peer', 'missing_peer_stop') or
                    (scenario in ('stop_peer', 'active_during_stop') and clock[0] > 1.)):
                continue
            if robot == side and scenario == 'stale_local' and clock[0] > .4:
                continue
            active = robot == peer and (scenario == 'active_peer' or
                      (scenario == 'active_during_stop' and clock[0] > .8))
            callback(message(robot_id=robot, stamp=100 + clock[0],
                             run_state='IDLE' if active else 'PAUSED'))

    def publishers(topic):
        robot = topic.split('/')[-2]
        name = f'multi_agent_{side}_agent' if robot == side else bridge_name + '_0'
        if scenario == 'replay_peer' and robot == peer:
            name = 'local_replay_node'
        result = publisher(name, 'BEST_EFFORT' if scenario == 'best_effort' and robot == peer
                           else 'RELIABLE')
        return [result, result] if scenario == 'duplicate_peer' and robot == peer else [result]

    node = SimpleNamespace(
        create_subscription=lambda msg_type, topic, callback, qos: callbacks.update({topic.split('/')[-2]: callback}),
        get_publishers_info_by_topic=publishers, destroy_node=lambda: callbacks.clear())
    ros = SimpleNamespace(init=lambda **kw: running.__setitem__(0, True),
        ok=lambda: running[0], create_node=lambda *a, **kw: node, spin_once=spin_once,
        shutdown=lambda: running.__setitem__(0, False))
    qos = SimpleNamespace(QoSProfile=lambda **kw: kw,
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1), DurabilityPolicy=SimpleNamespace(VOLATILE=2))
    for name, module in {'rclpy': ros, 'rclpy.qos': qos,
                         'multi_agent_task_msgs.msg': SimpleNamespace(AgentState=object)}.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(comm.time, 'monotonic', lambda: clock[0])
    assert comm.monitor(side, dict(bridge_name=bridge_name, lost_timeout=.3), .8, stop_check) == expected
    assert not running[0] and not callbacks
    if scenario == 'stop_peer':
        assert 'STOP_OBSERVED' in capsys.readouterr().out


@pytest.mark.parametrize('action', ['preflight', 'agent', 'bridge'])
def test_main_default_and_launch_environment(monkeypatch, tmp_path, action):
    context = dict(directory=tmp_path, bridge_xml=tmp_path / 'bridge.xml',
                   bridge_name='ranger_cooperative_task_bridge', lost_timeout=3.)
    monkeypatch.setattr(comm, 'preflight', lambda args: context)
    monkeypatch.setattr(comm, 'reject_existing_node', lambda name: None)
    monkeypatch.setattr(comm, 'blocked_config', lambda *args: ({}, ['real gate']))
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', 'file:///local.xml')
    captured = []
    def execute(command, argv, env):
        captured.append((argv, env))
        raise RuntimeError('exec intercepted before any process starts')
    monkeypatch.setattr(comm.os, 'execvpe', execute)
    result = comm.main(['--side', 'ranger', action])
    if action == 'preflight':
        assert result == 0 and not captured
    else:
        assert result == 2 and len(captured) == 1
        argv, env = captured[0]
        if action == 'bridge':
            assert 'ROS_DOMAIN_ID' not in env and env['CYCLONEDDS_URI'] == (tmp_path / 'bridge.xml').as_uri()
        else:
            assert env['ROS_DOMAIN_ID'] == '0' and env['CYCLONEDDS_URI'] == 'file:///local.xml'


def test_gates_opening_after_preflight_prevents_launch(monkeypatch, tmp_path):
    monkeypatch.setattr(comm, 'preflight', lambda args: dict(directory=tmp_path))
    monkeypatch.setattr(comm, 'reject_existing_node', lambda name: None)
    def gates_opened(*args):
        raise ValueError('live_gates 已为空')
    monkeypatch.setattr(comm, 'blocked_config', gates_opened)
    monkeypatch.setattr(comm.os, 'execvpe', lambda *args: pytest.fail('must not start agent'))
    assert comm.main(['--side', 'go2', 'agent']) == 2
