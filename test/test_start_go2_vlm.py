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


@pytest.mark.parametrize('case', ['current', 'wrong_path', 'stale_code', 'wrong_prefix', 'arrival_order'])
def test_shell_checks_imports_before_launching(tmp_path, case):
    scripts = tmp_path/'scripts'; scripts.mkdir()
    entry = scripts/'start_go2_vlm.sh'
    entry.write_text((ROOT/'scripts/start_go2_vlm.sh').read_text())
    marker = tmp_path/'launched'
    script = (ROOT/'scripts/start_go2_vlm.py').read_text().replace('    sys.exit(main())',
        f"    sys.exit(main()) if '--check-install' in sys.argv else Path({str(marker)!r}).touch()")
    (scripts/'start_go2_vlm.py').write_text(script)
    install = tmp_path/'current/install'; paths=[]
    for package in ('vlm_nav','multi_agent_nav'):
        source = tmp_path/('vlm_nav' if package=='vlm_nav' else 'cooperative_ws/src/multi_agent_nav/multi_agent_nav')
        modules = (tmp_path/'old' if case=='wrong_path' else install/package)/'python'
        actual = modules/package
        source.mkdir(parents=True); actual.mkdir(parents=True);paths.append(str(modules))
        files={'__init__.py':'', 'local_approach_execution.py':'value = 1\n'}
        if package=='multi_agent_nav':
            files['config.py'] = 'def load_config(directory):\n    return '+repr(dict(robot_go2=dict(
                arrival_config_files=[str(tmp_path/'config'/f) for f in
                    (('robot_go2.yaml','robot.yaml') if case=='arrival_order' else ('robot.yaml','robot_go2.yaml'))])))+'\n'
            files['single_robot_approach.py']='def load_parameters(files): pass\n'
        for name,content in files.items():
            (source/name).write_text(content);(actual/name).write_text(content)
        if case=='stale_code' and package=='multi_agent_nav':
            (actual/'local_approach_execution.py').write_text('value = 0\n')
    ament=tmp_path/'stubs/ament_index_python';ament.mkdir(parents=True)
    (ament/'__init__.py').touch()
    (ament/'packages.py').write_text(
        'def get_package_prefix(name): return '+repr(str(tmp_path/'wrong' if case=='wrong_prefix' else install))+"+'/'+name\n"+
        'def get_package_share_directory(name): return '+repr(str(install))+"+'/'+name+'/share/'+name\n")
    (tmp_path/'config').mkdir()
    share=install/'vlm_nav/share/vlm_nav/config';share.mkdir(parents=True)
    for name in ('robot.yaml','robot_go2.yaml'):
        (tmp_path/'config'/name).write_text(name);(share/name).write_text(name)
    (tmp_path/'CoVLM_env.bash').write_text(
        'export COOP_SETUP_FILE='+repr(str(install/'local_setup.bash'))+'\n'+
        'export PYTHONPATH='+repr(':'.join([str(ament.parent),*paths]))+'\n')
    result=subprocess.run(['bash',str(entry)],cwd=tmp_path,capture_output=True,text=True)
    assert (result.returncode==0)==(case=='current'),result.stderr
    assert marker.exists()==(case=='current')
    if case!='current':assert '安装校验失败' in result.stderr


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


@pytest.mark.parametrize('field', ['active_map_matches', 'costmap_valid', 'depth_units_verified'])
def test_inspect_rejects_failed_inputs(field):
    report = ready_report()
    report[field] = False
    with pytest.raises(RuntimeError):
        launcher.require_inputs(report)


def test_inspect_accepts_disarmed_bridge_but_requires_stable_navigation_inputs():
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


def test_camera_reuses_publishers_during_reconnection_but_rejects_duplicates(monkeypatch):
    app = launcher.Launcher(ROOT, prepare_only=True, target='灭火器')
    monkeypatch.setattr(app, 'confirm', lambda *args: pytest.fail('should not prompt or start SSH'))
    monkeypatch.setattr(app, 'window', lambda *args: pytest.fail('should not start another camera'))
    counts = iter([1, 1])
    monkeypatch.setattr(app, 'publishers', lambda topic: next(counts))
    app.camera()
    for values in ([1, 0], [0, 1]):
        counts = iter(values)
        app.camera()
    for values in ([2, 1], [0, 2]):
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
    prefix=tmp_path/'build/go2-recoverable-pause-20261029/overlay/install'
    for package in ('vlm_nav','multi_agent_nav'):(prefix/package).mkdir(parents=True)
    app=launcher.Launcher(tmp_path,prepare_only=True,target='cup',cooperative_hardware=True)
    monkeypatch.setattr(app,'check_owners',lambda:None)
    monkeypatch.setattr(app,'ros',lambda *args,**kwargs:SimpleNamespace(stdout=str(prefix/args[2])))
    def offline_boundary(args,**kwargs):
        if '--check-install' in args:return SimpleNamespace(returncode=0,stdout='')
        raise RuntimeError('reached offline configuration check')
    monkeypatch.setattr(app,'command',offline_boundary)
    with pytest.raises(RuntimeError,match='reached offline configuration check'):app.start()


@pytest.mark.parametrize('reuse', [False, True])
@pytest.mark.parametrize('cooperative,bridge_state', [
    (False, 'DISARMED'), (True, 'DISARMED'), (True, 'ARMED'), (True, 'FAULT')])
def test_startup_cold_or_reused_stack_never_arms(monkeypatch, tmp_path, capsys, reuse,
                                               cooperative, bridge_state):
    monkeypatch.setenv('COOP_CONFIG', str(tmp_path / 'actual-config'))
    monkeypatch.setenv('GO2_EXPLORE_CONFIG', str(tmp_path / 'actual-config'))
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
            output = '/tmp/current-cooperative/install/'+args[3]
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
                assert kwargs['timeout'] == 240
                assert '/twist_to_go2_sport_bridge' in nodes, 'NAV_READY requires Sport through SYSTEM_READY'
                nav_ready = True
            output = repr(bridge_state) if 'String' in ' '.join(args) else 'data: true'
        elif '--check-install' in args:
            output = '{}'
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
        assert confirmations == ['READY']
        assert [name for name, args in windows] == ['Camera SSH', 'System', 'RViz', 'Sport']
        system = windows[1][1]
        assert 'start_bridge:=false' in system
        assert 'target_stage:=nav2' in system
        map_path = '/tmp/current-map.yaml'
        config = tmp_path / 'actual-config'
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
    (tmp_path/'scripts').mkdir()
    (tmp_path/'scripts/start_go2_vlm.py').write_text('import sys; assert "--check-install" in sys.argv\n')
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


def test_install_check_rejects_parent_child_selection_mismatch(monkeypatch,tmp_path):
    assert hasattr(launcher,'check_install'), 'no paired module/source validation'
    monkeypatch.setenv('COOP_SETUP_FILE',str(tmp_path/'old/install/local_setup.bash'))
    with pytest.raises(RuntimeError,match='父子终端'):
        launcher.check_install(tmp_path,tmp_path/'new/install/local_setup.bash',tmp_path/'config')


@pytest.mark.parametrize('cooperative',[False,True])
def test_child_reload_of_old_pair_blocks_component_start(monkeypatch,tmp_path,cooperative):
    scripts=tmp_path/'scripts';scripts.mkdir()
    (scripts/'start_go2_vlm.py').write_text((ROOT/'scripts/start_go2_vlm.py').read_text())
    old=tmp_path/'old/install/local_setup.bash'
    new=tmp_path/'new/install/local_setup.bash'
    config=tmp_path/'config';config.mkdir()
    monkeypatch.setenv('COOP_CONFIG',str(config))
    monkeypatch.setenv('GO2_EXPLORE_CONFIG',str(config))
    monkeypatch.setenv('COOP_SETUP_FILE',str(new))
    envfile=tmp_path/('Cooperative_env.bash' if cooperative else 'CoVLM_env.bash')
    envfile.write_text('export COOP_SETUP_FILE='+repr(str(old))+'\n')
    app=launcher.Launcher(tmp_path,True,'target',cooperative_hardware=cooperative)
    marker=tmp_path/'component-started'
    def command(args):
        child=[str(a) for a in args[args.index('--')+1:]]
        result=subprocess.run(child,capture_output=True,text=True,input='\n')
        assert result.returncode!=0
        assert '父子终端安装选择不一致' in result.stderr
    monkeypatch.setattr(app,'command',command)
    app.window('System',['touch',marker])
    assert not marker.exists()


def test_inspect_camera_disconnection_is_diagnostic_only():
    report = ready_report()
    report['rgbd_ready'] = False
    report['health_details'] = {'checks': {'rgbd_ready': False, 'camera_geometry': False}}
    report['health_window']['failure_counts'] = {'rgbd_ready': 100, 'camera_geometry': 100}
    launcher.require_inputs(report)


def test_camera_launch_does_not_wait_for_usb_or_input_ready(monkeypatch):
    app = launcher.Launcher(ROOT, prepare_only=True, target='灭火器')
    windows=[]; confirmations=[]
    monkeypatch.setattr(app, 'publishers', lambda topic: 0)
    monkeypatch.setattr(app, 'confirm', lambda *args: confirmations.append(args[0]))
    monkeypatch.setattr(app, 'window', lambda *args: windows.append(args))
    monkeypatch.setattr(launcher.time, 'sleep', lambda _: pytest.fail('camera availability blocks startup'))
    app.camera()
    assert confirmations==[]
    assert windows==[('Camera SSH', ['bash', ROOT/'scripts/go2_camera_ssh.sh'])]


def test_agent_opt_in_defaults_off():
    app=launcher.Launcher(ROOT,False,'灭火器')
    assert app.agent_framework is False


@pytest.mark.parametrize('agent', [False,True])
def test_both_modes_use_shared_search_argv(monkeypatch,tmp_path,agent):
    app=launcher.Launcher(ROOT,False,' 中文 "目标"; $(echo nope) ',agent_framework=agent)
    app.logs=tmp_path
    for key in ['DASHSCOPE_API_KEY','DASHSCOPE_BASE_URL','DASHSCOPE_MODEL']:monkeypatch.setenv(key,'offline')
    for name in ['confirm','check_owners','inspect']:monkeypatch.setattr(app,name,lambda *a:None)
    states=iter(['DISARMED','ARMED','DISARMED']);monkeypatch.setattr(app,'state',lambda:next(states))
    services=[];monkeypatch.setattr(app,'service',services.append)
    original=app.check
    monkeypatch.setattr(app,'check',lambda *a:original(*a)+['--shared-test-marker'])
    report=dict(result='TARGET_APPROACHED',stop_confirmed=True,evidence_write_failed=False,vlm_evidence_pending=False)
    captured=[]
    if agent:
        monkeypatch.setattr(app,'agent_preflight',lambda:None)
        def run(argv):
            captured.append(argv)
            return dict(state='SUCCEEDED',active_process=False,exit_code=0,original_report=report,framework_error=None)
        monkeypatch.setattr(app,'agent_search',run)
        monkeypatch.setattr(app,'agent_host_result',lambda result:None)
    else:
        class Process:
            stdout=io.StringIO(json.dumps(report));returncode=0
            def wait(self):return 0
        def spawn(argv,**kw):captured.append(argv);return Process()
        monkeypatch.setattr(launcher.subprocess,'Popen',spawn)
    app.explore()
    argv=captured[0]
    assert argv[-1]=='--shared-test-marker'
    assert argv[argv.index('--target')+1]==app.target
    assert argv[argv.index('--timeout')+1]=='300'
    assert services==['arm','disarm']


def test_agent_preflight_failure_precedes_arm(monkeypatch,tmp_path):
    app=launcher.Launcher(ROOT,False,'目标',agent_framework=True);app.logs=tmp_path
    for key in ['DASHSCOPE_API_KEY','DASHSCOPE_BASE_URL','DASHSCOPE_MODEL']:monkeypatch.setenv(key,'offline')
    monkeypatch.setattr(app,'agent_preflight',lambda:(_ for _ in ()).throw(RuntimeError('version mismatch')))
    monkeypatch.setattr(app,'confirm',lambda *a:pytest.fail('no ARM confirmation'))
    with pytest.raises(RuntimeError,match='version mismatch'):app.explore()


@pytest.mark.parametrize('state,active,stop,services',[
    ('STOP_UNCONFIRMED',True,True,['arm']),('STOP_UNCONFIRMED',False,True,['arm']),
    ('FAILED',False,True,['arm','disarm']),('CANCELED',False,True,['arm','disarm']),
])
def test_agent_stop_unknown_never_disarms(monkeypatch,tmp_path,state,active,stop,services):
    app=launcher.Launcher(ROOT,False,'目标',agent_framework=True);app.logs=tmp_path
    for key in ['DASHSCOPE_API_KEY','DASHSCOPE_BASE_URL','DASHSCOPE_MODEL']:monkeypatch.setenv(key,'offline')
    for name in ['confirm','check_owners','inspect','agent_preflight']:monkeypatch.setattr(app,name,lambda *a:None)
    states=iter(['DISARMED','ARMED','DISARMED']);monkeypatch.setattr(app,'state',lambda:next(states))
    actual=[];monkeypatch.setattr(app,'service',actual.append)
    report=dict(result='TARGET_APPROACHED',stop_confirmed=stop,evidence_write_failed=False,vlm_evidence_pending=False)
    monkeypatch.setattr(app,'agent_search',lambda argv:dict(state=state,active_process=active,original_report=report,exit_code=0))
    monkeypatch.setattr(app,'agent_host_result',lambda result:None)
    with pytest.raises(RuntimeError):app.explore()
    assert actual==services


@pytest.mark.parametrize('cause',[KeyboardInterrupt,subprocess.TimeoutExpired])
def test_agent_host_cancels_manager_without_kill_escalation(monkeypatch,tmp_path,cause):
    app=launcher.Launcher(ROOT,False,'目标',agent_framework=True)
    app.agent_directory=tmp_path
    result=dict(session_id=tmp_path.name,task_id=tmp_path.name+'/find-target',target=app.target,
                state='CANCELED',active_process=False,framework_error=None)
    calls=[]
    class Process:
        returncode=1
        def communicate(self,**kwargs):
            calls.append(kwargs)
            if len(calls)==1:
                if cause is KeyboardInterrupt:raise KeyboardInterrupt
                raise subprocess.TimeoutExpired('adapter',1270)
            (tmp_path/'agent-result.json').write_text(json.dumps(result))
        def poll(self):return None
        def send_signal(self,sig):assert sig==signal.SIGINT
        def kill(self):pytest.fail('no force kill')
        def terminate(self):pytest.fail('no SIGTERM')
    captured=[]
    def spawn(argv,**kwargs):captured.append((argv,kwargs));return Process()
    monkeypatch.setattr(launcher.subprocess,'Popen',spawn)
    monkeypatch.setenv('PYTHONPATH','/original/ros')
    monkeypatch.setenv('DASHSCOPE_API_KEY','secret-in-pipe-only')
    actual=app.agent_search(['ros2','--target',app.target])
    assert actual['state']=='CANCELED'
    assert calls[1]['timeout']==65
    payload=json.loads(calls[0]['input'])
    assert payload['ros_env']['PYTHONPATH']=='/original/ros'
    assert 'PYTHONPATH' not in captured[0][1]['env']
    assert captured[0][0][1]=='-I'
    assert all('secret-in-pipe-only' not in p.read_text() for p in tmp_path.iterdir())


def test_new_shell_delegates_to_original_entry(tmp_path):
    scripts=tmp_path/'scripts';scripts.mkdir()
    (scripts/'start_go2_agent_task.sh').write_text((ROOT/'scripts/start_go2_agent_task.sh').read_text())
    legacy=scripts/'start_go2_vlm.sh'
    legacy.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    legacy.chmod(0o755)
    target=' 中文 "目标"; $(false) '
    result=subprocess.run(['bash',str(scripts/'start_go2_agent_task.sh'),'--target',target],capture_output=True,text=True,check=True)
    assert json.loads(result.stdout)==['--agent-framework','--target',target]


def test_agent_prepare_only_does_not_import_or_check_framework(monkeypatch,tmp_path):
    import builtins
    monkeypatch.setenv('REPO',str(ROOT));monkeypatch.setenv('DISPLAY',':test')
    monkeypatch.setattr(sys,'argv',['start_go2_vlm.py','--agent-framework','--prepare-only'])
    monkeypatch.setattr(sys,'stdin',SimpleNamespace(isatty=lambda:True))
    monkeypatch.setattr(launcher.shutil,'which',lambda name:'/usr/bin/gnome-terminal')
    monkeypatch.setattr(launcher,'open',lambda *a,**k:builtins.open(tmp_path/'start.lock','a'),raising=False)
    monkeypatch.setattr(launcher.tempfile,'mkdtemp',lambda **k:str(tmp_path))
    monkeypatch.setattr(launcher.Launcher,'start',lambda app:None)
    monkeypatch.setattr(launcher.Launcher,'agent_preflight',lambda app:pytest.fail('framework preflight'))
    monkeypatch.setattr(launcher.Launcher,'explore',lambda app:pytest.fail('search'))
    assert launcher.main()==0
