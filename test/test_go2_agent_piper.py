"""Exercise real RoboRSI tool dispatch with a fake planner, arm launcher and SSH."""
import json
from pathlib import Path
import subprocess

import pytest
from roborsi.planner.base import PlannerResult

from tools.go2_agent import piper


@pytest.mark.parametrize('mode,expected', [
    ('grasp','SUCCEEDED'), ('place','SUCCEEDED'), ('both','SUCCEEDED'),
    ('unconfirmed','INCOMPLETE'), ('source-mismatch','INCOMPLETE'),
    ('old-place','INCOMPLETE'), ('early-finish','INCOMPLETE'),
])
def test_planner_dispatch(tmp_path,monkeypatch,mode,expected):
    target = '纸盒旁的 "红方块"; $(false)'
    place_only = mode in ('place','old-place')
    combined = mode in ('both','unconfirmed')
    session = piper.PiperTools(tmp_path,None if place_only else target,
                               'grasp-physical-01' if place_only else None,combined,{'USER_KEY':'do-not-log'})
    calls = []
    current = None
    def run(cmd,**kwargs):
        nonlocal current
        if cmd[0]=='bash':
            current = Path(cmd[1]).stem
            calls.append(current)
            assert kwargs['env']['USER_KEY']=='do-not-log'
            if current=='piper_grasp_once':
                assert cmd[2:]==[target] and kwargs['env']['PIPER_AGENT_GRASP_ID']==session.grasp_id
            else:
                assert cmd[2:]==[session.grasp_id]
            return subprocess.CompletedProcess(cmd,1 if mode in ('old-place','unconfirmed') else 0)
        assert cmd[0]=='ssh' and cmd[-1].endswith('/'+session.grasp_id+'/result.json')
        if current=='piper_grasp_once':
            data=dict(state='RETURNED_ZERO_UNCONFIRMED' if mode=='unconfirmed' else 'GRASP_RETURNED_ZERO',
                      run_id='wrong' if mode=='source-mismatch' else session.grasp_id,
                      returned_to_sensor_zero=True,grasp_confirmed=mode!='unconfirmed')
        else:
            data=dict(state='OPENED_AT_LIFT_RETURNED_ZERO',source_grasp_id=session.grasp_id,
                      object_landing_verified=False)
        return subprocess.CompletedProcess(cmd,0,stdout=json.dumps(data))
    monkeypatch.setattr(subprocess,'run',run)
    monkeypatch.setattr(piper.runtime,'preflight',lambda cwd:dict(cli=dict(
        executable='/offline/codex',version='codex-cli 0.167.0',sha256='a'*64)))
    class Planner:
        def preflight(self):pass
        def solve(self,*,toolkit,**kwargs):
            tool_names={r['name'] for r in toolkit.get_tools_spec()}
            assert tool_names=={'finish','piper_status'}|{'piper_'+a for a in session.actions}
            if mode!='early-finish':
                if combined:
                    with pytest.raises(ValueError):
                        session.start('place',{'grasp_id':session.grasp_id})
                for action in session.actions:
                    values={'target':target} if action=='grasp' else {'grasp_id':session.grasp_id}
                    with pytest.raises(ValueError):
                        session.start(action,dict(values,command='injected'))
                    toolkit.execute_tool('piper_'+action,values)
                    toolkit.execute_tool('piper_'+action,values)
                    row=toolkit.execute_tool('piper_status',{}).result['actions'][action]
                    assert row['state']!='RUNNING'
                    if row['state']!='SUCCEEDED':break
            final=toolkit.execute_tool('finish',{'status':'success','summary':'planner says OK'})
            return PlannerResult(finish_result=final.result)
    monkeypatch.setattr('roborsi.embodied_agent.build_planner',lambda *args,**kw:Planner())
    result=piper.run_task(session)
    assert result['state']==expected and result['framework_error'] is None
    assert calls==([] if mode=='early-finish' else ['piper_place_once'] if place_only else
                   ['piper_grasp_once','piper_place_once'] if mode=='both' else ['piper_grasp_once'])
    if mode=='unconfirmed':
        assert result['actions']['grasp']['state']=='UNCONFIRMED'
        with pytest.raises(ValueError): session.start('place',{'grasp_id':session.grasp_id})
    assert 'do-not-log' not in (tmp_path/'agent-result.json').read_text()


@pytest.mark.parametrize('value',['../bad','-flag','; echo hi',''])
def test_reject_invalid_grasp_id(tmp_path,value):
    with pytest.raises(ValueError):
        piper.PiperTools(tmp_path,None,value,False,{})
