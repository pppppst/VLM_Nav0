"""Real RoboRSI registration/dispatch, fake Planner and search; offline only."""
import json
import signal

import pytest
from roborsi.planner.base import PlannerResult

from tools.go2_agent import runtime
from test_go2_agent_session import finish, make_session


@pytest.mark.parametrize('mode,expected', [('success','SUCCEEDED'),('no-execution','FAILED'),
    ('failure','FAILED'),('early-finish','CANCELED'),('exception','CANCELED'),
    ('sigterm','CANCELED'),('timeout','CANCELED')])
def test_real_tool_dispatch_fake_planner(monkeypatch,tmp_path,mode,expected):
    signals=[]
    s,p,calls,spec=make_session(tmp_path,killpg=lambda *a:stop(*a))
    def stop(*args):
        signals.append(args);finish(s,p,code=1,result='operator interrupt')
    monkeypatch.setattr(runtime,'SearchSession',lambda spec:s)
    monkeypatch.setattr(runtime,'preflight',lambda directory: {'offline_fake':True})
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
    assert config['codex_cli']['expected_version']=='codex-cli 0.166.0'
    assert config['max_turns'] is None and config['max_tokens'] is None
    assert config['planner_timeout_s']==1200 and config['action_timeout_s']==600
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
