"""Isolated RoboRSI entry. The stdin protocol is private, never a model reply."""
import argparse
import json
import os
from pathlib import Path
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
    from roborsi.planner.codex_cli import preflight_codex_cli
    from roborsi.planner.codex_cli_config import CodexCliConfig
    import roborsi
    config = load_config()
    head = subprocess.check_output(['git', '-C', str(FRAMEWORK), 'rev-parse', 'HEAD'], text=True).strip()
    if head != config['framework_head'] or Path(roborsi.__file__).resolve().parent != FRAMEWORK/'roborsi':
        raise ValueError('RoboRSI checkout or import identity mismatch')
    if subprocess.check_output(['git', '-C', str(FRAMEWORK), 'status', '--porcelain'], text=True).strip():
        raise ValueError('RoboRSI working tree changed')
    cli = CodexCliConfig.from_dict(config['codex_cli'])
    if not Path(cli.executable).is_absolute():
        raise ValueError('absolute Codex executable required')
    return dict(framework_head=head, framework_python=sys.executable,
                python_version=sys.version.split()[0],
                cli=preflight_codex_cli(cli, repo_root=directory),
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
        from roborsi.embodied_agent import EmbodiedAgent
        from roborsi.planner.codex_cli_config import CodexCliConfig
        config = load_config()
        agent = EmbodiedAgent(mcp_servers=(), local_tools=local_tools(session),
            output_dir=directory/'framework', working_dir=cli_cwd, planner='codex_cli',
            codex_cli=CodexCliConfig.from_dict(config['codex_cli']),
            **{key: config[key] for key in ('model', 'reasoning_effort', 'max_turns',
                                          'max_tokens', 'planner_timeout_s', 'action_timeout_s')})
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
