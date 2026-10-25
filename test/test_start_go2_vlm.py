"""Launcher checks run without ROS, SSH, API calls or hardware."""
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('start_go2_vlm', ROOT / 'scripts/start_go2_vlm.py')
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


@pytest.mark.parametrize('case', ['current', 'wrong_path', 'missing_fix'])
def test_shell_checks_imports_before_launching(tmp_path, case):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    entry = scripts / 'start_go2_vlm.sh'
    entry.write_text((ROOT / 'scripts/start_go2_vlm.sh').read_text())
    marker = tmp_path / 'launched'
    (scripts / 'start_go2_vlm.py').write_text(
        f'from pathlib import Path\nPath({str(marker)!r}).touch()\n')
    output = tmp_path / 'current'
    prefix = output / 'install/multi_agent_nav'
    modules = (tmp_path / 'old' if case == 'wrong_path' else prefix) / 'python'
    package = modules / 'multi_agent_nav'
    (package / 'adapters').mkdir(parents=True)
    for module in ['__init__.py', 'adapters/__init__.py', 'go2_exploration.py']:
        (package / module).touch()
    (package / 'adapters/go2.py').write_text(
        'class Go2Adapter:\n    ' + ('pass' if case == 'missing_fix'
                                   else 'def approach_candidate_free(self): pass') + '\n')
    (tmp_path / 'CoVLM_env.bash').write_text(
        f'export GO2_EXPLORE_OUTPUT={str(output)!r}\nexport PYTHONPATH={str(modules)!r}\n')
    result = subprocess.run(['bash', str(entry)], cwd=tmp_path, capture_output=True, text=True)
    if case == 'current':
        assert result.returncode == 0, result.stderr
        assert str(package / 'adapters/go2.py') in result.stdout
        assert str(package / 'go2_exploration.py') in result.stdout
        assert marker.exists()
    else:
        assert result.returncode != 0
        assert not marker.exists()
        assert '安装校验失败' in result.stderr


def test_shell_help_does_not_load_environment(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    entry = scripts / 'start_go2_vlm.sh'
    entry.write_text((ROOT / 'scripts/start_go2_vlm.sh').read_text())
    (scripts / 'start_go2_vlm.py').write_text('print("HELP_ONLY")\n')
    (tmp_path / 'CoVLM_env.bash').write_text('exit 99\n')
    result = subprocess.run(['bash', str(entry), '--help'], capture_output=True, text=True)
    assert result.returncode == 0
    assert result.stdout.strip() == 'HELP_ONLY'


def ready_report():
    return dict(active_map_matches=True, costmap_valid=True, rgbd_ready=True,
                depth_units_verified=True, health=dict(localization_valid=True, stopped=True),
                health_window=dict(non_motion_inputs_stable=True, transitions=[
                    dict(failed_gates=['nav_ready'], other_goal_owners=[])]))


def test_report_parsing_allows_ros_logs_before_json():
    report = ready_report()
    assert launcher.parse_report('[INFO] observer started\n' + json.dumps(report, indent=2)) == report


@pytest.mark.parametrize('field', ['active_map_matches', 'costmap_valid', 'rgbd_ready',
                                  'depth_units_verified'])
def test_inspect_rejects_failed_inputs(field):
    report = ready_report()
    report[field] = False
    with pytest.raises(RuntimeError):
        launcher.require_inputs(report)


def test_inspect_accepts_disarmed_bridge_but_requires_stable_camera():
    report = ready_report()
    launcher.require_inputs(report)
    report['health_window']['non_motion_inputs_stable'] = False
    with pytest.raises(RuntimeError):
        launcher.require_inputs(report)


def test_inspect_requires_configured_keepout_even_while_disarmed():
    report = ready_report()
    report['health_details'] = {'checks': {'control_armed_gate': False, 'keepout_applied': False}}
    with pytest.raises(RuntimeError, match='keepout'):
        launcher.require_inputs(report)
    report['health_details']['checks']['keepout_applied'] = True
    launcher.require_inputs(report)


@pytest.mark.parametrize('field', ['stopped', 'localization_valid'])
def test_inspect_rejects_moving_or_unlocalized_robot(field):
    report = ready_report()
    report['health'][field] = False
    with pytest.raises(RuntimeError):
        launcher.require_inputs(report)


def test_inspect_rejects_other_goal_owner():
    report = ready_report()
    report['health_window']['transitions'][-1]['other_goal_owners'] = ['vlm_nav']
    with pytest.raises(RuntimeError):
        launcher.require_inputs(report)


@pytest.mark.parametrize('output, expected', [("'ARMED'\n---\n", 'ARMED'),
    ('DISARMED\n---\n', 'DISARMED'), ("'FAULT: timeout'\n---\n", 'FAULT')])
def test_bridge_state_parsing(output, expected):
    assert launcher.parse_state(output) == expected


def test_failed_service_response_does_not_count_as_success():
    with pytest.raises(RuntimeError):
        launcher.require_service_success('response:\nTrigger_Response(success=False, message="busy")')
    launcher.require_service_success('response:\nTrigger_Response(success=True, message="ARMED")')


def test_camera_reuses_unique_publishers_and_rejects_partial_or_duplicate_camera(monkeypatch):
    app = launcher.Launcher(ROOT, prepare_only=True, target='灭火器')
    monkeypatch.setattr(app, 'confirm', lambda *args: pytest.fail('should not prompt or start SSH'))
    monkeypatch.setattr(app, 'window', lambda *args: pytest.fail('should not start another camera'))
    counts = iter([1, 1])
    monkeypatch.setattr(app, 'publishers', lambda topic: next(counts))
    app.camera()
    for values in ([1, 0], [2, 1]):
        counts = iter(values)
        with pytest.raises(RuntimeError):
            app.camera()


@pytest.mark.parametrize('result, stopped, pending, code, expected_services, success', [
    ('TARGET_APPROACHED', True, False, 0, ['arm', 'disarm'], True),
    ('APPROACH_FAILED', True, False, 1, ['arm', 'disarm'], False),
    ('TARGET_APPROACHED', False, False, 0, ['arm'], False),
    ('TARGET_APPROACHED', True, True, 0, ['arm', 'disarm'], False),
])
def test_experiment_disarms_only_after_confirmed_stop(monkeypatch, tmp_path, result,
        stopped, pending, code, expected_services, success):
    app = launcher.Launcher(ROOT, prepare_only=False, target='灭火器')
    app.logs = tmp_path
    for name in ['DASHSCOPE_API_KEY', 'DASHSCOPE_BASE_URL', 'DASHSCOPE_MODEL']:
        monkeypatch.setenv(name, 'test-placeholder')
    monkeypatch.setattr(app, 'confirm', lambda *args: None)
    monkeypatch.setattr(app, 'check_owners', lambda: None)
    monkeypatch.setattr(app, 'inspect', lambda: None)
    services = []
    monkeypatch.setattr(app, 'service', services.append)
    states = iter(['DISARMED', 'ARMED', 'DISARMED'])
    monkeypatch.setattr(app, 'state', lambda: next(states))
    report = dict(result=result, stop_confirmed=stopped, vlm_evidence_pending=pending,
                  evidence_write_failed=False)
    class Process:
        stdout = io.StringIO(json.dumps(report))
        returncode = code
        def wait(self):
            return self.returncode
    monkeypatch.setattr(launcher.subprocess, 'Popen', lambda *args, **kwargs: Process())
    if success:
        app.explore()
    else:
        with pytest.raises(RuntimeError):
            app.explore()
    assert services == expected_services
    assert json.loads((tmp_path / 'explore.log').read_text()) == report


def test_failed_inspect_prevents_arm_and_experiment(monkeypatch, tmp_path):
    app = launcher.Launcher(ROOT, prepare_only=False, target='灭火器')
    app.logs = tmp_path
    for name in ['DASHSCOPE_API_KEY', 'DASHSCOPE_BASE_URL', 'DASHSCOPE_MODEL']:
        monkeypatch.setenv(name, 'test-placeholder')
    monkeypatch.setattr(app, 'confirm', lambda *args: None)
    monkeypatch.setattr(app, 'check_owners', lambda: None)
    def failed_inspect():
        raise RuntimeError('RGB-D not ready')
    monkeypatch.setattr(app, 'inspect', failed_inspect)
    monkeypatch.setattr(app, 'service', lambda *args: pytest.fail('must not ARM'))
    monkeypatch.setattr(launcher.subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('must not explore'))
    with pytest.raises(RuntimeError, match='RGB-D not ready'):
        app.explore()


def test_cooperative_startup_expects_target_plan_install_by_default(monkeypatch,tmp_path):
    monkeypatch.delenv('COOP_SETUP_FILE',raising=False)
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path/'config'))
    monkeypatch.setenv('GO2_BRIDGE_OUTPUT',str(tmp_path/'old-bridge'))
    monkeypatch.setenv('GO2_EXPLORE_OUTPUT',str(tmp_path/'old-explore'))
    prefix=tmp_path/'build/go2-single-parity-20261025/overlay/install'
    for package in ('vlm_nav','multi_agent_nav'):(prefix/package).mkdir(parents=True)
    app=launcher.Launcher(tmp_path,prepare_only=True,target='cup',cooperative_hardware=True)
    monkeypatch.setattr(app,'check_owners',lambda:None)
    monkeypatch.setattr(app,'ros',lambda *args,**kwargs:SimpleNamespace(stdout=str(prefix/args[2])))
    def offline_boundary(*args,**kwargs):raise RuntimeError('reached offline configuration check')
    monkeypatch.setattr(app,'command',offline_boundary)
    with pytest.raises(RuntimeError,match='reached offline configuration check'):app.start()


@pytest.mark.parametrize('reuse', [False, True])
@pytest.mark.parametrize('cooperative,bridge_state', [
    (False, 'DISARMED'), (True, 'DISARMED'), (True, 'ARMED'), (True, 'FAULT')])
def test_startup_cold_or_reused_stack_never_arms(monkeypatch, tmp_path, capsys, reuse,
                                               cooperative, bridge_state):
    monkeypatch.setenv('COOP_CONFIG', str(tmp_path / 'actual-config'))
    monkeypatch.setenv('COOP_SETUP_FILE', '/tmp/current-cooperative/install/local_setup.bash')
    monkeypatch.setenv('GO2_PREFLIGHT_REPORT', str(tmp_path / 'formal-report.json'))
    mask = ROOT/'maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261022_29ab5c38/keepout_mask.yaml'
    (tmp_path/'actual-config').mkdir()
    (tmp_path/'actual-config/robot_go2.yaml').write_text(f'keepout_mask: {mask}\n')
    app = launcher.Launcher(ROOT, prepare_only=True, target='灭火器',
                           **({'cooperative_hardware': True} if cooperative else {}))
    app.logs = tmp_path
    monkeypatch.setenv('GO2_BRIDGE_OUTPUT', '/tmp/current-bridge')
    monkeypatch.setenv('GO2_EXPLORE_OUTPUT', '/tmp/current-explore')
    confirmations, windows, calls = [], [], []
    monkeypatch.setattr(app, 'confirm', lambda word, message: confirmations.append(word))
    topics = {'/lowstate': 1}
    nodes = []
    if reuse:
        topics.update({t: 1 for t in ['/camera/camera/color/image_raw',
            '/camera/camera/depth/image_rect_raw', '/lidar_points', '/body_imu', '/odometry']})
        nodes.extend(['/rviz2', '/twist_to_go2_sport_bridge'])
    def window(name, args):
        windows.append((name, [str(x) for x in args]))
        if name == 'Camera SSH':
            topics.update({'/camera/camera/color/image_raw': 1,
                           '/camera/camera/depth/image_rect_raw': 1})
        if name == 'Sport':
            nodes.append('/twist_to_go2_sport_bridge')
    monkeypatch.setattr(app, 'window', window)
    monkeypatch.setattr(launcher.time, 'sleep', lambda seconds: None)
    nav_ready = False
    def command(args, **kwargs):
        nonlocal nav_ready
        args = [str(x) for x in args]
        calls.append(args)
        if args[1:3] == ['node', 'list']:
            output = '\n'.join(nodes)
        elif args[1:3] == ['pkg', 'prefix']:
            output = {'vlm_nav': '/tmp/current-bridge/install/vlm_nav',
                      'multi_agent_nav': ('/tmp/current-cooperative/install/multi_agent_nav' if cooperative
                                          else '/tmp/current-explore/install/multi_agent_nav')}[args[3]]
        elif args[1:3] == ['topic', 'info']:
            output = f'Publisher count: {topics.get(args[3], 0)}'
        elif args[1:3] == ['param', 'get']:
            if args[4] == 'keepout_mask':
                assert nav_ready, 'System starts the supervisor only after AMCL readiness'
                return SimpleNamespace(returncode=0, stdout=f'String value is: {mask}')
            value = {'inflation_layer.inflation_radius': '.45',
                     'general_goal_checker.xy_goal_tolerance': '.25', 'GridBased.tolerance': '.5'}[args[4]]
            output = f'Double value is: {value}'
        elif args[1:3] == ['topic', 'echo']:
            if '/vlm_nav/nav_ready' in args:
                assert '/twist_to_go2_sport_bridge' in nodes, 'NAV_READY requires Sport through SYSTEM_READY'
                nav_ready = True
            output = repr(bridge_state) if 'String' in ' '.join(args) else 'data: true'
        elif '--offline' in args:
            output = json.dumps(dict(map_files=dict(yaml_path='/tmp/current-map.yaml')))
        elif 'inspect' in args:
            output = json.dumps(ready_report())
        else:
            pytest.fail(f'unexpected command: {args}')
        return SimpleNamespace(returncode=0, stdout=output)
    monkeypatch.setattr(app, 'command', command)
    preflights = []
    monkeypatch.setattr(launcher.subprocess, 'run', lambda args, **kwargs: preflights.append(args))
    if cooperative and bridge_state != 'DISARMED':
        with pytest.raises(RuntimeError, match='DISARMED'):
            app.start()
        assert not any('service' in args or '--allow-motion' in args for args in calls)
        assert not any('inspect' in args for args in calls)
        return
    app.start()
    if reuse:
        assert 'File → Open Config' in capsys.readouterr().out
    assert not any('service' in args or '--allow-motion' in args for args in calls)
    if reuse:
        assert windows == []
        assert preflights == []
        assert confirmations == ['READY', 'CURRENT']
    else:
        assert confirmations == ['CAMERA', 'START', 'READY']
        assert [name for name, args in windows] == ['Camera SSH', 'System', 'RViz', 'Sport']
        system = windows[1][1]
        assert 'start_bridge:=false' in system
        assert 'target_stage:=nav2' in system
        map_path = '/tmp/current-map.yaml' if cooperative else f'{ROOT}/maps/lab_rescan_20261018_155656_0pa6gm/lab.yaml'
        config = tmp_path / 'actual-config' if cooperative else ROOT / 'cooperative_ws/src/multi_agent_nav/config'
        assert f'map:={map_path}' in system
        assert f'costmap_overrides_file:={config}/nav2_go2_cooperative_costmap.yaml' in system
        if cooperative:
            assert f'sensor_preflight_report:={tmp_path}/formal-report.json' in system
            assert f'keepout_mask:={mask}' in system
        assert len(preflights) == 1


@pytest.mark.parametrize('help_only', [False, True])
def test_cooperative_hardware_shell_loads_only_cooperative_environment(tmp_path, help_only):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    entry = scripts / 'start_go2_cooperative_hardware.sh'
    entry.write_text((ROOT / 'scripts/start_go2_cooperative_hardware.sh').read_text())
    marker = tmp_path / 'environment-loaded'
    (tmp_path / 'Cooperative_env.bash').write_text(
        '[[ "$1" == go2 && "$2" == "$COOP_CONFIG" ]] || return 91\n'
        f'touch {str(marker)!r}\n')
    (scripts / 'start_go2_vlm.py').write_text('import sys; print(sys.argv[1:])\n')
    env = dict(os.environ, COOP_CONFIG=str(tmp_path / 'custom config'))
    result = subprocess.run(['bash', str(entry), *(['--help'] if help_only else [])],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert ('start_go2_cooperative_hardware.sh' if help_only else '--cooperative-hardware') in result.stdout
    assert marker.exists() is not help_only


def test_cooperative_hardware_windows_keep_environment_and_config(monkeypatch, tmp_path):
    config = tmp_path / 'custom config'
    monkeypatch.setenv('COOP_CONFIG', str(config))
    (tmp_path / 'Cooperative_env.bash').write_text(
        '[[ "$1" == go2 ]] || return 91\nexport CHILD_CONFIG="$2"\n')
    app = launcher.Launcher(tmp_path, False, 'unused', cooperative_hardware=True)
    def command(args):
        child = args[args.index('--') + 1:]
        result = subprocess.run([str(x) for x in child], input='\n', capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert str(config) in result.stdout
    monkeypatch.setattr(app, 'command', command)
    app.window('RViz', ['/bin/bash', '-c', 'echo "$CHILD_CONFIG"'])


def test_cooperative_hardware_main_cannot_explore_without_prepare_flag(monkeypatch, tmp_path):
    import builtins
    monkeypatch.setenv('REPO', str(ROOT))
    monkeypatch.setenv('COOP_SIDE', 'go2')
    monkeypatch.setenv('COOP_CONFIG', str(tmp_path))
    monkeypatch.setenv('DISPLAY', ':test')
    monkeypatch.setattr(sys, 'argv', ['start_go2_vlm.py', '--cooperative-hardware'])
    monkeypatch.setattr(sys, 'stdin', SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(launcher.shutil, 'which', lambda name: '/usr/bin/gnome-terminal')
    monkeypatch.setattr(launcher, 'open', lambda *a, **kw: builtins.open(tmp_path / 'start.lock', 'a'), raising=False)
    monkeypatch.setattr(launcher.tempfile, 'mkdtemp', lambda **kw: str(tmp_path))
    stages = []
    def start(app):
        assert app.prepare_only and app.cooperative_hardware
        assert app.config == tmp_path
        stages.append('hardware')
    monkeypatch.setattr(launcher.Launcher, 'start', start)
    monkeypatch.setattr(launcher.Launcher, 'explore', lambda app: pytest.fail('must not ARM or explore'))
    assert launcher.main() == 0
    assert stages == ['hardware']


def test_interrupt_waits_for_stop_report_before_disarming(monkeypatch, tmp_path):
    app = launcher.Launcher(ROOT, prepare_only=False, target='灭火器')
    app.logs = tmp_path
    for name in ['DASHSCOPE_API_KEY', 'DASHSCOPE_BASE_URL', 'DASHSCOPE_MODEL']:
        monkeypatch.setenv(name, 'test-placeholder')
    monkeypatch.setattr(app, 'confirm', lambda *args: None)
    monkeypatch.setattr(app, 'check_owners', lambda: None)
    monkeypatch.setattr(app, 'inspect', lambda: None)
    states = iter(['DISARMED', 'ARMED', 'DISARMED'])
    monkeypatch.setattr(app, 'state', lambda: next(states))
    events = []
    monkeypatch.setattr(app, 'service', lambda action: events.append(action))
    monkeypatch.setattr(launcher.os, 'killpg', lambda pid, sig: events.append(('interrupt', pid, sig)))
    class Output:
        def __iter__(self):
            raise KeyboardInterrupt
    class Process:
        stdout = Output()
        returncode = 1
        pid = 12345
        def poll(self):
            return None
        def communicate(self, timeout):
            events.append('stop-report')
            return json.dumps(dict(result='operator interrupt', stop_confirmed=True)), None
    def popen(args, **kwargs):
        assert kwargs['start_new_session'] is True
        return Process()
    monkeypatch.setattr(launcher.subprocess, 'Popen', popen)
    with pytest.raises(RuntimeError, match='未完整成功'):
        app.explore()
    assert events == ['arm', ('interrupt', 12345, launcher.signal.SIGINT), 'stop-report', 'disarm']


def test_command_timeout_interrupts_descendant_observer(tmp_path):
    pid_file, stopped = tmp_path / 'pid', tmp_path / 'stopped'
    child = (
        'import os, signal, sys, time; from pathlib import Path; '
        f'signal.signal(signal.SIGINT, lambda *_: (Path({str(stopped)!r}).touch(), sys.exit(0))); '
        f'Path({str(pid_file)!r}).write_text(str(os.getpid())); time.sleep(20)')
    parent = ('import signal, subprocess, sys; '
              'signal.signal(signal.SIGINT, lambda *_: None); '
              f'p=subprocess.Popen([sys.executable, "-c", {child!r}]); p.wait()')
    app = launcher.Launcher(ROOT, prepare_only=True, target='灭火器')
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            app.command([sys.executable, '-c', parent], timeout=.5)
        assert pid_file.exists(), 'fixture child failed to start'
        assert stopped.exists(), 'timeout left the observer running without cancellation'
    finally:
        if pid_file.exists():
            try:
                os.kill(int(pid_file.read_text()), signal.SIGTERM)
            except ProcessLookupError:
                pass
