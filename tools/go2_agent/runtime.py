"""Isolated RoboRSI entry. The stdin protocol is private, never a model reply."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

# Direct execution uses the framework interpreter with -I, without ROS PYTHONPATH.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.go2_agent.session import SearchSession, write_json

ADAPTER = Path(__file__).resolve().parent
FRAMEWORK = ADAPTER.parents[2]/'SenseNova-RoboRSI'
if not FRAMEWORK.is_dir():  # Installed share tree on the Go2 control computer.
    FRAMEWORK = Path.home()/'unitree_ros2/SenseNova-RoboRSI'
FRAMEWORK_PYTHON = FRAMEWORK/'.venv/bin/python'
PROMPT = '''只使用 start_find_target、get_task_status、cancel_task 和 finish 工具。
禁止 shell、ROS 命令、文件编辑、参数调整、其他工具及另起搜索。目标必须逐字符照传。
原程序完成全部搜索与靠近；不要干预内部策略，不重新看图。相同任务不重复启动。
每次状态查询后等待返回，不并发轮询。只有任务终态才 finish；未知不能自称成功。
工具成功只代表搜索执行器验收，整轮成功还需宿主 DISARM 并读回 DISARMED。'''


def framework_environment(original):
    env = dict(original)
    for key in list(env):
        if key.startswith(('ROS_', 'AMENT_', 'COLCON_', 'CMAKE_', 'CYCLONEDDS')) or key in {
                'PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV', 'LD_LIBRARY_PATH'}:
            env.pop(key)
    env['PATH'] = str(FRAMEWORK_PYTHON.parent) + ':/usr/local/bin:/usr/bin:/bin'
    return env


def load_config():
    import yaml
    return yaml.safe_load((ADAPTER/'config.yaml').read_text())


def preflight(directory):
    from roborsi.planner.codex_cli import preflight_codex_cli, _cli_arguments
    from roborsi.planner.codex_cli_config import CodexCliConfig
    import roborsi
    config = load_config()
    head = subprocess.check_output(['git', '-C', str(FRAMEWORK), 'rev-parse', 'HEAD'], text=True).strip()
    if head != config['framework_head'] or Path(roborsi.__file__).resolve().parent != FRAMEWORK/'roborsi':
        raise ValueError('RoboRSI checkout or import identity mismatch')
    if subprocess.check_output(['git', '-C', str(FRAMEWORK), 'status', '--porcelain'], text=True).strip():
        raise ValueError('RoboRSI working tree changed')
    cli_values = dict(config['codex_cli'])
    minimum = cli_values.pop('minimum_version')
    cli = CodexCliConfig.from_dict(dict(cli_values, expected_version=minimum))
    if not Path(cli.executable).is_absolute():
        raise ValueError('absolute Codex executable required')
    executable = str(Path(cli.executable).resolve(strict=True))
    probe = dict(cwd=directory, env=cli.subprocess_env(), stdin=subprocess.DEVNULL,
                 capture_output=True, text=True, check=True, timeout=5)
    version = subprocess.run([executable, '--version'], **probe).stdout.strip()
    versions = [re.fullmatch(r'codex-cli (\d+)\.(\d+)\.(\d+)', value) for value in (minimum, version)]
    if (not all(versions) or tuple(map(int, versions[1].groups())) < tuple(map(int, versions[0].groups()))):
        raise ValueError(f'Codex CLI requires a stable version >= {minimum}; got {version!r}')
    cli = CodexCliConfig.from_dict(dict(cli_values, executable=executable, expected_version=version))
    identity = preflight_codex_cli(cli, repo_root=directory)
    global_args, exec_args = _cli_arguments(cli.cli_args)
    required = set()
    for args, fixed in [(global_args, set()), (exec_args, {'--config', '--model', '--json', '--color'})]:
        command = ([executable, *global_args, 'exec', '--json', '--color', 'never',
                    '--model', config['model'], '-c',
                    'model_reasoning_effort=' + json.dumps(config['reasoning_effort']), *exec_args]
                   if fixed else [executable, *global_args])
        help_text = subprocess.run([*command, '--help'], **probe).stdout
        options = fixed | {arg.split('=')[0] for arg in args if arg.startswith('--')}
        missing = options - set(re.findall(r'(?<![\w-])--[\w-]+', help_text))
        if missing:
            raise ValueError('Codex CLI missing required options: ' + ', '.join(sorted(missing)))
        required.update(options)
    return dict(framework_head=head, framework_python=sys.executable,
                python_version=sys.version.split()[0],
                cli=identity,
                cli_compatibility=dict(minimum_version=minimum, required_options=sorted(required),
                                       model_protocol='unverified'),
                model=config['model'], reasoning_effort=config['reasoning_effort'],
                action_timeout_note='does not bound LocalToolSpec handlers',
                tool_policy='read-only sandbox plus prompt; exclusive tool use not verified')


def local_tools(session):
    from roborsi.embodied_agent import LocalToolSpec
    from roborsi.tools.toolkit import ToolResult
    result = []
    for name, argument, method, description in [
        ('start_find_target', 'target', session.start, 'Start this exact target once; returns immediately.'),
        ('get_task_status', 'task_id', session.status, 'Read original events/report; may wait one second.'),
        ('cancel_task', 'task_id', session.cancel, 'Request cancellation once; this does not confirm stopping.'),
    ]:
        def handler(values, name=name, argument=argument, method=method):
            if not isinstance(values, dict) or set(values) != {argument} or not isinstance(values[argument], str):
                raise ValueError('exact declared string argument required')
            if name == 'get_task_status':
                time.sleep(1)
            return ToolResult(name=name, result=method(values[argument]))
        result.append(LocalToolSpec(name=name, description=description,
            input_schema={'type': 'object', 'properties': {argument: {'type': 'string'}},
                          'required': [argument], 'additionalProperties': False}, handler=handler))
    return result


def build_agent(directory, cli_cwd, identity, tools):
    from roborsi.embodied_agent import EmbodiedAgent
    from roborsi.planner.codex_cli_config import CodexCliConfig
    config = load_config()
    cli_values = dict(config['codex_cli'])
    cli_values.pop('minimum_version')
    # Pin this session, so an updater cannot switch the CLI after preflight.
    cli_values.update(executable=identity['cli']['executable'],
                      expected_version=identity['cli']['version'],
                      expected_sha256=identity['cli']['sha256'])
    return EmbodiedAgent(mcp_servers=(), local_tools=tools,
        output_dir=directory/'framework', working_dir=cli_cwd, planner='codex_cli',
        codex_cli=CodexCliConfig.from_dict(cli_values),
        **{key: config[key] for key in ('model', 'reasoning_effort', 'max_turns',
                                      'max_tokens', 'planner_timeout_s', 'action_timeout_s')})


def run_session(spec):
    session = SearchSession(spec)
    directory = session.directory
    cli_cwd = directory/'cli-work'
    cli_cwd.mkdir(exist_ok=True)
    closing = False
    interrupted = False
    framework = {'status': 'NOT_STARTED', 'finish': None}
    def interrupt(signum, frame):
        nonlocal interrupted
        if closing or interrupted:
            return
        interrupted = True
        session.cancel(session.task_id)
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        identity = preflight(cli_cwd)
        write_json(directory/'preflight.json', identity)
        agent = build_agent(directory, cli_cwd, identity, local_tools(session))
        result = agent.run('本轮唯一目标（JSON 字符串，逐字符解码）：' + json.dumps(session.target, ensure_ascii=False),
                           system_prompt=PROMPT, skills=[ADAPTER/'skills/find_target/SKILL.md'])
        framework = {'status': 'FAILED' if result.error else 'FINISHED', 'finish': result.finish_result,
                     'error_kind': result.error_kind, 'stop_reason': result.stop_reason}
        if result.error:
            session.data['framework_error'] = 'PLANNER_FAILED'
    except (Exception, KeyboardInterrupt) as error:
        session.data['framework_error'] = 'INTERRUPTED' if interrupted else type(error).__name__
        framework['status'] = 'FAILED'
    finally:
        # The independently owned search survives a planner failure or early finish.
        try:
            snapshot = session.status(session.task_id)
            if snapshot.get('active_process') and not interrupted and snapshot.get('cancel_requested_at') is None:
                session.data.setdefault('framework_error', None)
                if not session.data['framework_error']:
                    session.data['framework_error'] = 'EARLY_FINISH'
                while snapshot.get('active_process') and snapshot.get('cancel_requested_at') is None:
                    time.sleep(.2)
                    snapshot = session.status(session.task_id)
        except KeyboardInterrupt:
            session.data['framework_error'] = 'INTERRUPTED'
        except Exception as error:
            session.data['framework_error'] = type(error).__name__
        finally:
            closing = True
        try:
            snapshot = session.close()
            snapshot['framework'] = framework
            write_json(directory/'agent-result.json', snapshot)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight', type=Path)
    args = parser.parse_args()
    if args.preflight:
        args.preflight.mkdir(parents=True, exist_ok=True)
        write_json(args.preflight/'preflight.json', preflight(args.preflight))
        return 0
    spec = json.load(sys.stdin)
    # The original ROS environment arrives through a pipe, never a spec/log file.
    os.environ.clear()
    os.environ.update(framework_environment(spec['ros_env']))
    result = run_session(spec)
    return 0 if result['state'] == 'SUCCEEDED' and result['framework_error'] is None else 1


if __name__ == '__main__':
    sys.exit(main())
