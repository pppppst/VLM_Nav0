"""Real RoboRSI registration/dispatch, fake Planner and search; offline only."""
import json
import signal

import pytest
from roborsi.planner.base import PlannerResult

from tools.go2_agent import runtime
from test_go2_agent_session import finish, make_session


@pytest.mark.parametrize('mode,expected', [('success','SUCCEEDED'),('no-execution','FAILED'),
    ('failure','FAILED'),('early-finish','SUCCEEDED'),('exception','SUCCEEDED'),
    ('sigterm','CANCELED'),('timeout','SUCCEEDED')])
def test_real_tool_dispatch_fake_planner(monkeypatch,tmp_path,mode,expected):
    signals=[]
    s,p,calls,spec=make_session(tmp_path,killpg=lambda *a:stop(*a))
    def stop(*args):
        signals.append(args);finish(s,p,code=1,result='operator interrupt')
    monkeypatch.setattr(runtime,'SearchSession',lambda spec:s)
    monkeypatch.setattr(runtime,'preflight',lambda directory: {
        'cli': {'executable': '/offline/codex', 'version': 'codex-cli 0.167.0', 'sha256': 'a'*64}})
    seen={}
    class Planner:
        def preflight(self):pass
        def solve(self,*,toolkit,**kwargs):
            seen['schemas']=toolkit.get_tools_spec()
            assert '不得修改目标' in kwargs['user_message'] or '不得修改目标' in kwargs['system_prompt']
            if mode=='no-execution':return PlannerResult(finish_result={'status':'success'})
            result=toolkit.execute_tool('start_find_target',{'target':s.target}).result
            assert result['task_id']==s.task_id
            assert toolkit.execute_tool('get_task_status',{'task_id':s.task_id}).result['state']=='RUNNING'
            if mode=='success':finish(s,p)
            if mode=='failure':finish(s,p,result='NOT_FOUND')
            if mode in ('early-finish','exception','timeout'):
                monkeypatch.setattr(runtime.time,'sleep',lambda _:finish(s,p))
            if mode=='exception':raise RuntimeError('fake CLI crash')
            if mode=='timeout':raise TimeoutError('fake framework budget')
            if mode=='sigterm':signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
            final=toolkit.execute_tool('finish',{'status':'success','summary':'I succeeded'})
            assert final.is_finish
            return PlannerResult(finish_result=final.result)
    def build(*args,**kwargs):seen['config']=kwargs;return Planner()
    monkeypatch.setattr('roborsi.embodied_agent.build_planner',build)
    result=runtime.run_session(spec)
    assert result['state']==expected
    assert len(calls)==(0 if mode=='no-execution' else 1)
    assert result['host_cleanup']=='PENDING'
    assert seen['config']['max_tokens'] is None
    assert seen['config']['model']=='gpt-6-astra'
    assert seen['config']['reasoning_effort']=='high'
    cli = seen['config']['codex_cli']
    assert (cli.executable, cli.expected_version, cli.expected_sha256) == (
        '/offline/codex', 'codex-cli 0.167.0', 'a'*64)
    tools={t['name']:t['input_schema'] for t in seen['schemas']}
    assert set(tools)=={'start_find_target','get_task_status','cancel_task','finish'}
    assert set(tools['start_find_target']['properties'])=={'target'}
    assert set(tools['cancel_task']['properties'])=={'task_id'}
    assert all(sig==signal.SIGINT for _,sig in signals)
    assert 'secret-never-persist' not in (tmp_path/'agent-result.json').read_text()


def test_tool_arguments_are_validated_in_handler(tmp_path):
    s,_,calls,_=make_session(tmp_path)
    tool=runtime.local_tools(s)[0]
    for args in ({'target':'different'},{'target':s.target,'argv':['evil']},{'target':3}):
        with pytest.raises(ValueError):tool.handler(args)
    assert not calls


def test_preflight_failure_never_starts_robot(monkeypatch,tmp_path):
    s,_,calls,spec=make_session(tmp_path)
    monkeypatch.setattr(runtime,'SearchSession',lambda spec:s)
    monkeypatch.setattr(runtime,'preflight',lambda directory:(_ for _ in ()).throw(ValueError('version mismatch')))
    result=runtime.run_session(spec)
    assert not calls and result['state']=='FAILED'
    assert result['framework_error'] is not None


def test_fixed_config_and_framework_environment():
    config=runtime.load_config()
    assert config['codex_cli']['executable']=='/home/isee-pst/.local/bin/codex'
    assert config['codex_cli']['minimum_version']=='codex-cli 0.166.0'
    assert config['max_turns'] is None and config['max_tokens'] is None
    assert config['planner_timeout_s'] is None and config['action_timeout_s']==600
    original={'PATH':'/ros/bin:/old/venv/bin','PYTHONPATH':'/ros/python','PYTHONHOME':'bad',
              'VIRTUAL_ENV':'/old/venv','AMENT_PREFIX_PATH':'/ros','DASHSCOPE_API_KEY':'private',
              'HOME':'/unchanged','CODEX_HOME':'/unchanged-codex'}
    env=runtime.framework_environment(original)
    assert 'PYTHONPATH' not in env and 'PYTHONHOME' not in env and 'VIRTUAL_ENV' not in env
    assert '/ros' not in env['PATH'] and env['HOME']==original['HOME']
    assert env['CODEX_HOME']==original['CODEX_HOME']
    assert original['PYTHONPATH']=='/ros/python'


def test_public_preflight_rejects_wrong_cli_version_before_search(monkeypatch,tmp_path):
    config=runtime.load_config()
    executable=tmp_path/'fake-codex'
    executable.write_text('#!/usr/bin/python3\nprint("codex-cli wrong-version")\n')
    executable.chmod(0o755)
    config['codex_cli']['executable']=str(executable)
    monkeypatch.setattr(runtime,'load_config',lambda:config)
    s,_,calls,spec=make_session(tmp_path/'session')
    monkeypatch.setattr(runtime,'SearchSession',lambda spec:s)
    result=runtime.run_session(spec)
    assert result['state']=='FAILED' and result['framework_error']=='ValueError'
    assert not calls


@pytest.mark.parametrize('version', ['0.166.0', '0.167.0', '0.200.1', '1.0.0'])
def test_preflight_accepts_compatible_cli_updates(monkeypatch, tmp_path, version):
    executable = fake_cli(monkeypatch, tmp_path, 'codex-cli ' + version)
    identity = runtime.preflight(tmp_path)
    assert identity['cli']['version'] == 'codex-cli ' + version
    assert identity['cli']['executable'] == str(executable)
    assert identity['cli_compatibility']['minimum_version'] == 'codex-cli 0.166.0'
    assert identity['cli_compatibility']['model_protocol'] == 'unverified'


def fake_cli(monkeypatch, tmp_path, version, missing='', help_exit=0, global_only=''):
    config = runtime.load_config()
    executable = tmp_path / 'fake-codex'
    options = '--ask-for-approval --config --model --json --color --sandbox --skip-git-repo-check --ignore-rules'
    options = options.replace(missing, '') if missing else options
    exec_options = options.replace(global_only, '') if global_only else options
    executable.write_text('#!/usr/bin/python3\nimport sys\n'
        f'if sys.argv[1:] == ["--version"]: print({version!r})\n'
        f'elif "--help" in sys.argv: print({exec_options!r} if "exec" in sys.argv else {options!r}); sys.exit({help_exit})\n'
        'else: raise AssertionError("preflight must not invoke a model")\n')
    executable.chmod(0o755)
    config['codex_cli']['executable'] = str(executable)
    monkeypatch.setattr(runtime, 'load_config', lambda: config)
    return executable


@pytest.mark.parametrize('option', ['--json', '--color', '--model'])
def test_exec_options_cannot_be_satisfied_by_global_help(monkeypatch, tmp_path, option):
    fake_cli(monkeypatch, tmp_path, 'codex-cli 0.167.0', global_only=option)
    with pytest.raises(ValueError, match='missing required options'):
        runtime.preflight(tmp_path)


def test_real_framework_rejects_cli_replacement_after_adapter_preflight(monkeypatch, tmp_path):
    import roborsi.embodied_agent as framework
    executable = fake_cli(monkeypatch, tmp_path, 'codex-cli 0.167.0')
    session, _, calls, spec = make_session(tmp_path / 'session')
    monkeypatch.setattr(runtime, 'SearchSession', lambda spec: session)
    build = framework.build_planner
    preflight_errors = []
    def replace_cli(*args, **kwargs):
        executable.write_text(executable.read_text() + '\n# replaced, same version\n')
        planner = build(*args, **kwargs)
        preflight = planner.preflight
        def checked_preflight():
            try:
                return preflight()
            except ValueError as error:
                preflight_errors.append(str(error))
                raise
        planner.preflight = checked_preflight
        return planner
    monkeypatch.setattr(framework, 'build_planner', replace_cli)
    result = runtime.run_session(spec)
    assert result['state'] == 'FAILED'
    assert result['framework_error'] is not None
    assert len(preflight_errors) == 1 and 'SHA-256' in preflight_errors[0]
    assert not calls


@pytest.mark.parametrize('version,missing,help_exit', [
    ('codex-cli 0.165.9', '', 0),
    ('codex-cli 0.167.0-alpha.1', '', 0),
    ('unknown', '', 0),
    ('codex-cli 0.167.0', '--json', 0),
    ('codex-cli 0.167.0', '--ask-for-approval', 0),
    ('codex-cli 0.167.0', '--ignore-rules', 0),
    ('codex-cli 0.167.0', '', 2),
])
def test_incompatible_cli_cannot_start_search(monkeypatch, tmp_path, version, missing, help_exit):
    fake_cli(monkeypatch, tmp_path, version, missing, help_exit)
    session, _, calls, spec = make_session(tmp_path / 'session')
    monkeypatch.setattr(runtime, 'SearchSession', lambda spec: session)
    result = runtime.run_session(spec)
    assert result['state'] == 'FAILED'
    assert result['framework_error'] is not None
    assert not calls
