"""RoboRSI tools for the existing single-cycle PiPER executors."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import uuid

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.go2_agent import runtime
from tools.go2_agent.session import write_json

SCRIPTS = runtime.ADAPTER.parents[1]/'scripts'
PROMPT = ('只使用本会话声明的 PiPER 技能、piper_status 和 finish。目标及编号逐字照传。'
          '启动后串行查询状态，RUNNING 时继续等待。只按宿主给定顺序执行。'
          '失败、抓持未确认或反馈未知时如实结束。不得自行重试或调用 shell。')


def execute(action, value, grasp_id, directory, environment):
    """Run the original launcher, then read the result of exactly this invocation."""
    env = dict(environment)
    if action == 'grasp':
        env['PIPER_AGENT_GRASP_ID'] = grasp_id
    script = SCRIPTS/f'piper_{action}_once.sh'
    folder = 'piper-r4-grasp-evidence' if action == 'grasp' else 'piper-place-runs'
    remote = f'/home/unitree/{folder}/{grasp_id}/result.json'
    outcome = dict(state='UNKNOWN', grasp_id=grasp_id, remote_result=remote)
    with (directory/f'{action}.log').open('x') as log:
        run = subprocess.run(['bash', str(script), value], env=env, stdin=subprocess.DEVNULL,
                             stdout=log, stderr=subprocess.STDOUT)
        outcome['exit_code'] = run.returncode
        # A refused repeat of place can leave an older successful result at the same path.
        if action == 'place' and run.returncode != 0:
            return outcome
        report = subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5',
                                 'unitree-go','cat',remote], env=env, capture_output=True,
                                text=True, timeout=15)
    if report.returncode:
        return outcome
    data = json.loads(report.stdout)
    if not isinstance(data,dict):
        return outcome
    if action == 'grasp':
        if data.get('run_id') != grasp_id:
            return outcome
        if data.get('returned_to_sensor_zero') is True:
            if data.get('state') == 'RETURNED_ZERO_UNCONFIRMED':
                outcome['state'] = 'UNCONFIRMED'
            elif (run.returncode == 0 and data.get('state') == 'GRASP_RETURNED_ZERO'
                  and data.get('grasp_confirmed') is True):
                outcome['state'] = 'SUCCEEDED'
    elif data.get('source_grasp_id') == grasp_id and data.get('state') == 'OPENED_AT_LIFT_RETURNED_ZERO':
        outcome['state'] = 'SUCCEEDED'
    write_json(directory/f'{action}-result.json',data)
    outcome['executor_state'] = data.get('state')
    for key in ('grasp_outcome','returned_to_sensor_zero','holding_at_zero_verified',
                'open_width_mm','final_sensor_deg','object_landing_verified'):
        if key in data:
            outcome[key] = data[key]
    return outcome


class PiperTools:
    def __init__(self, directory, target, grasp_id, then_place, environment):
        if target is not None and (not isinstance(target,str) or not target.strip() or '\0' in target):
            raise ValueError('nonempty grasp description required')
        if grasp_id is not None and not re.fullmatch('[a-zA-Z0-9][a-zA-Z0-9_-]*',grasp_id):
            raise ValueError('invalid source grasp id')
        if (target is None) == (grasp_id is None) or (then_place and target is None):
            raise ValueError('select grasp or place')
        self.directory = directory
        self.target = target
        self.grasp_id = grasp_id or 'grasp-'+uuid.uuid4().hex
        self.actions = ['grasp'] if target is not None else ['place']
        if then_place:
            self.actions.append('place')
        self.environment = environment
        self.results = {}
        self.lock = threading.RLock()
        self.thread = None

    def snapshot(self):
        with self.lock:
            return dict(grasp_id=self.grasp_id, actions={a:dict(r) for a,r in self.results.items()})

    def _run(self, action, value):
        try:
            # Serialize manipulation sessions on this host, including pre-motion recognition.
            with (self.directory.parent/'piper.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = execute(action,value,self.grasp_id,self.directory,self.environment)
        except BlockingIOError:
            result = dict(state='BUSY')
        except Exception as error:
            result = dict(state='UNKNOWN',error=type(error).__name__)
        with self.lock:
            self.results[action] = result
            write_json(self.directory/'actions.json',self.snapshot())

    def start(self, action, values):
        field,value = ('target',self.target) if action=='grasp' else ('grasp_id',self.grasp_id)
        if action not in self.actions or values != {field:value}:
            raise ValueError('action and argument must match the host task')
        with self.lock:
            if action in self.results:
                return self.snapshot()
            if action=='place' and 'grasp' in self.actions and self.results.get('grasp',{}).get('state')!='SUCCEEDED':
                raise ValueError('this grasp has not completed successfully')
            # Consume before spawn. A second dispatch must never transmit another motion.
            (self.directory/f'{action}.started').touch(exist_ok=False)
            self.results[action] = dict(state='RUNNING')
            write_json(self.directory/'actions.json',self.snapshot())
            self.thread = threading.Thread(target=self._run,args=(action,value))
            self.thread.start()
            return self.snapshot()

    def tools(self):
        from roborsi.embodied_agent import LocalToolSpec
        from roborsi.tools.toolkit import ToolResult
        specs = []
        for action in self.actions:
            name = 'piper_'+action
            field = 'target' if action=='grasp' else 'grasp_id'
            def handler(values, action=action, name=name):
                return ToolResult(name=name,result=self.start(action,values))
            description = ('抓取指定物块并闭爪回传感器零点' if action=='grasp'
                           else '到指定抓取记录的抬升位松爪并回零')
            specs.append(LocalToolSpec(name=name,description=description,handler=handler,
                input_schema=dict(type='object',properties={field:{'type':'string'}},
                                  required=[field],additionalProperties=False)))
        def status(values):
            if values != {}:
                raise ValueError('status takes no arguments')
            if self.thread:
                self.thread.join(1)
            return ToolResult(name='piper_status',result=self.snapshot())
        specs.append(LocalToolSpec(name='piper_status',description='查询本轮 PiPER 动作及机载结果',
            input_schema=dict(type='object',properties={},additionalProperties=False),handler=status))
        return specs


def run_task(session):
    directory = session.directory
    cwd = directory/'cli-work'
    cwd.mkdir()
    write_json(directory/'actions.json',session.snapshot())
    error = None
    try:
        identity = runtime.preflight(cwd)
        write_json(directory/'preflight.json',identity)
        agent = runtime.build_agent(directory,cwd,identity,session.tools())
        request = dict(actions=session.actions,target=session.target,grasp_id=session.grasp_id)
        skills = [runtime.ADAPTER/'skills'/('piper_'+a)/'SKILL.md' for a in session.actions]
        answer = agent.run(json.dumps(request,ensure_ascii=False),system_prompt=PROMPT,skills=skills)
        if answer.error:
            error = answer.error_kind or 'PLANNER_FAILED'
    except (Exception,KeyboardInterrupt) as exc:
        error = type(exc).__name__
    finally:
        # Planner termination is not evidence of a physical stop. Monitor the already-started cycle.
        if session.thread:
            session.thread.join()
    result = session.snapshot()
    success = all(result['actions'].get(a,{}).get('state')=='SUCCEEDED' for a in session.actions)
    result.update(state='SUCCEEDED' if success and error is None else 'INCOMPLETE',framework_error=error)
    write_json(directory/'agent-result.json',result)
    return result


def main():
    parser = argparse.ArgumentParser(description='Go2 Agent · PiPER 抓取和抬升位放置')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--grasp',metavar='目标')
    mode.add_argument('--place',metavar='抓取编号')
    parser.add_argument('--then-place',action='store_true',help='抓取确认后在该次抬升位放置')
    args = parser.parse_args()
    environment = dict(os.environ)
    directory = Path.home()/'.local/state/vlm-nav-agent'/('piper-'+uuid.uuid4().hex)
    try:
        session = PiperTools(directory,args.grasp,args.place,args.then_place,environment)
    except ValueError as exc:
        parser.error(str(exc))
    directory.mkdir(parents=True,exist_ok=False)
    print('Agent 结果：'+str(directory/'agent-result.json'),flush=True)
    print('动作状态：'+str(directory/'actions.json'),flush=True)
    os.environ.clear()
    os.environ.update(runtime.framework_environment(environment))
    return 0 if run_task(session)['state']=='SUCCEEDED' else 1


if __name__=='__main__':
    sys.exit(main())
