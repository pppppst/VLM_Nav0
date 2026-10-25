"""Live task preparation with fake ROS only; no hardware, ARM or model requests."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as S

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def task(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    spec = importlib.util.spec_from_file_location('cooperative_task_entry', ROOT / 'scripts/start_go2_cooperative_task.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def state(robot='go2', **updates):
    values = dict(schema_version=1, robot_id=robot, simulation=False, run_state='STANDBY',
                  ready=False, deposited=False, exited=False, localization_valid=True,
                  nav_ready=True, stopped=True, local_map_id=robot+'-map', map_version='v1',
                  common_frame_id='', alignment_id='', task_id='', command_id='', goal_id='',
                  reason='', stamp=123., sequence=0, reply='', pose_valid=False,
                  common_pose=S(x=0.,y=0.,theta=0.), resource_id='', token='', permit_command_id='',
                  resource_status='', requested_resource='', requested_command_id='')
    values.update(updates)
    return S(**values)


def config():
    return dict(scene=dict(lost_timeout=.5),alignment=dict(robots={
        robot:dict(local_map_id=robot+'-map',map_version='v1') for robot in ('go2','ranger')}))


def test_valid_idle_does_not_require_shared_pose(task):
    assert task.idle_issues(state(), 'go2', config()['alignment']['robots']['go2']) == []


@pytest.mark.parametrize('run_state',['WAITING_PEER','STANDBY'])
def test_preparation_accepts_both_noncyclic_waiting_states(task,run_state):
    assert task.idle_issues(state(run_state=run_state),'go2',config()['alignment']['robots']['go2']) == []


def test_legacy_idle_is_not_a_new_startup_ack(task):
    assert task.idle_issues(state(run_state='IDLE'),'go2',config()['alignment']['robots']['go2'])


@pytest.mark.parametrize('changes', [
    dict(simulation=True),dict(run_state='PAUSED'),dict(run_state='NAVIGATING'),
    dict(ready=True),dict(deposited=True),dict(stopped=False),dict(nav_ready=False),
    dict(localization_valid=False),dict(map_version='old'),dict(task_id='previous'),
    dict(command_id='active'),dict(common_frame_id='lab_ref'),dict(reason='failed'),
    dict(stamp=float('nan')),dict(robot_id='ranger'),dict(schema_version=0),
])
def test_unready_or_wrong_identity_is_rejected(task, changes):
    assert task.idle_issues(state(**changes), 'go2', config()['alignment']['robots']['go2'])


@pytest.mark.parametrize('case', ['good','paused','missing_peer','frozen_stamp','wrong_source','duplicate','best_effort',
                                'late_peer','late_healthy','interrupted'])
def test_live_sampling_requires_fresh_unique_healthy_idle(task,monkeypatch,capsys,case):
    clock=[0.];callbacks={};lifecycle=[]
    monkeypatch.setattr(task.time,'monotonic',lambda:clock[0])
    def spin_once(node,timeout_sec):
        clock[0]+=.1
        if case=='interrupted' and clock[0]>=20:raise KeyboardInterrupt
        for robot,callback in callbacks.items():
            if robot=='ranger' and (case=='missing_peer' or case in ('late_peer','interrupted') and clock[0]<20):continue
            callback(state(robot,stamp=123. if case=='frozen_stamp' else 123.+clock[0],
                           run_state='PAUSED' if case=='paused' or case=='late_healthy' and robot=='ranger' and clock[0]<20 else 'STANDBY'))
    def publishers(topic):
        robot=topic.split('/')[-2]
        name='multi_agent_go2_agent' if robot=='go2' else 'go2_cooperative_task_bridge_0'
        if case=='wrong_source':name='replay'
        item=S(node_name=name,topic_type='multi_agent_task_msgs/msg/AgentState',
            qos_profile=S(reliability=S(name='BEST_EFFORT' if case=='best_effort' else 'RELIABLE'),
                          durability=S(name='VOLATILE')))
        return [item,item] if case=='duplicate' else [item]
    node=S(create_subscription=lambda cls,topic,callback,qos:callbacks.update({topic.split('/')[-2]:callback}),
           get_publishers_info_by_topic=publishers,destroy_node=lambda:lifecycle.append('destroy'))
    ros=S(init=lambda **kw:lifecycle.append('init'),create_node=lambda *a,**kw:node,
          spin_once=spin_once,ok=lambda:True,shutdown=lambda:lifecycle.append('shutdown'))
    monkeypatch.setitem(sys.modules,'rclpy',ros)
    monkeypatch.setitem(sys.modules,'rclpy.qos',S(QoSProfile=lambda **kw:kw,
        ReliabilityPolicy=S(RELIABLE=1),DurabilityPolicy=S(VOLATILE=1)))
    monkeypatch.setitem(sys.modules,'multi_agent_task_msgs.msg',S(AgentState=object))
    if case in ('good','late_peer','late_healthy'):
        result=task.wait_idle(config(),'go2_cooperative_task_bridge',**({'timeout':1.} if case=='good' else {}))
        assert set(result)=={'go2','ranger'} and clock[0]>=.3
        if case!='good':
            assert clock[0]>=20.2
            output=capsys.readouterr().out
            assert '等待真实待命' in output
            assert ('no state received' if case=='late_peer' else 'PAUSED') in output
            assert 4<=output.count('等待真实待命')<=5
    elif case=='interrupted':
        with pytest.raises(KeyboardInterrupt):task.wait_idle(config(),'go2_cooperative_task_bridge')
    else:
        with pytest.raises(RuntimeError):task.wait_idle(config(),'go2_cooperative_task_bridge',timeout=1.)
    assert lifecycle==['init','destroy','shutdown']


def app_fixture(task,monkeypatch,tmp_path):
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    app=task.TaskLauncher(ROOT,True,'',cooperative_hardware=True)
    app.logs=tmp_path
    events=[]
    report=dict(live_gates=[],api_environment_complete=True,bridge='go2_cooperative_task_bridge',
                targets=dict(A='A',B='B',C='C'),scenario_id='lab')
    monkeypatch.setattr(app,'preflight',lambda:(events.append('preflight') or report))
    monkeypatch.setattr(app,'check_owners',lambda:events.append('owners'))
    monkeypatch.setattr(app,'nodes',lambda:[])
    monkeypatch.setattr(app,'component',lambda action,name:events.append(action))
    monkeypatch.setattr(app,'window',lambda name,args:events.append(name))
    monkeypatch.setattr(app,'confirm',lambda word,text:events.append(word))
    monkeypatch.setattr(app,'service',lambda action:events.append(action))
    monkeypatch.setattr(app,'idle',lambda report:events.append('idle'))
    monkeypatch.setattr(app,'cooperative',lambda action:(events.append(action) or S(stdout='accepted: true',returncode=0)))
    return app,events


def test_prepare_skips_paused_check_but_keeps_bridge_and_arm_before_agent(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    states=iter(['DISARMED','ARMED'])
    monkeypatch.setattr(app,'state',lambda:next(states))
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='data: true'))
    app.prepare()
    assert events.index('bridge')<events.index('arm')<events.index('agent')<events.index('idle')<events.index('coordinator')
    assert 'ARM' in events and 'start' not in events and 'check' not in events


def test_prepare_opens_go2_status_window_using_existing_watch_action(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'state',lambda:'ARMED')
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='data: true'))
    windows=[]
    monkeypatch.setattr(app,'window',lambda name,args:windows.append((name,args)))
    app.prepare()
    assert len(windows)==1
    title,args=windows[0]
    assert title=='当前状态'
    assert args==['bash',ROOT/'scripts/cooperative.sh','go2','watch']
    assert 'start' not in events


def test_prepare_fault_never_resets_or_launches_agent(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'state',lambda:'FAULT')
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='data: true'))
    with pytest.raises(RuntimeError):app.prepare()
    assert 'arm' not in events and 'reset_fault' not in events and 'agent' not in events


def test_start_rechecks_idle_after_confirmation_and_does_not_arm(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'state',lambda:'ARMED')
    monkeypatch.setattr(app,'nodes',lambda:['/multi_agent_coordinator'])
    app.start_task()
    assert events.index('START')<events.index('idle')<events.index('start')
    assert 'arm' not in events and 'agent' not in events


def test_failed_idle_check_cannot_start_task(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'state',lambda:'ARMED')
    monkeypatch.setattr(app,'nodes',lambda:['/multi_agent_coordinator'])
    def failed(report):raise RuntimeError('peer PAUSED')
    monkeypatch.setattr(app,'idle',failed)
    with pytest.raises(RuntimeError):app.start_task()
    assert 'start' not in events


@pytest.mark.parametrize('count',[0,2])
def test_start_reports_coordinator_count_without_reading_sport(task,monkeypatch,tmp_path,count):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'nodes',lambda:['/multi_agent_coordinator']*count)
    monkeypatch.setattr(app,'state',lambda:pytest.fail('coordinator check short-circuits Sport query'))
    with pytest.raises(RuntimeError,match=f'coordinator数量={count}'):
        app.start_task()
    assert 'START' not in events and 'start' not in events


@pytest.mark.parametrize('state',['DISARMED','FAULT'])
def test_start_reports_actual_sport_state_and_does_not_arm(task,monkeypatch,tmp_path,state):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'nodes',lambda:['/multi_agent_coordinator'])
    monkeypatch.setattr(app,'state',lambda:state)
    with pytest.raises(RuntimeError,match=f'Sport状态={state}'):
        app.start_task()
    assert 'START' not in events and 'start' not in events and 'arm' not in events


def test_pause_bypasses_preflight_and_disarm_remains_explicit(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    app.pause_task()
    assert events==['pause']
    events.clear()
    monkeypatch.setattr(app,'state',lambda:'DISARMED')
    app.disarm_task()
    assert events==['STOPPED','disarm']


@pytest.mark.parametrize('failure',[None,'pause','CLOSED','disarm','nav-ready','inspect','prepare','start'])
def test_restart_preserves_hardware_and_stops_before_next_stage_on_failure(task,monkeypatch,tmp_path,failure):
    import json
    app,events=app_fixture(task,monkeypatch,tmp_path)
    states=iter(['DISARMED','DISARMED','ARMED','ARMED'])
    monkeypatch.setattr(app,'state',lambda:next(states))
    monkeypatch.setattr(app,'nodes',lambda:['/multi_agent_coordinator'])
    def stage(name):
        events.append(name)
        if failure==name:raise RuntimeError(name)
    def cooperative(action):
        stage(action)
        return S(stdout='accepted: true')
    def ready(*args,**kwargs):
        assert args[:4]==('topic','echo','/vlm_nav/nav_ready','std_msgs/msg/Bool')
        assert '--filter' in args and 'm.data' in args
        stage('nav-ready' if kwargs['timeout']==120 else 'prepare-ready')
        return S(stdout='data: true')
    def inspect(args,**kwargs):
        assert '--action' in args and 'inspect' in args and '--require-ready' in args
        stage('inspect')
        return S(stdout=json.dumps(dict(active_map_matches=True,costmap_valid=True,rgbd_ready=True,
            depth_units_verified=True,health=dict(localization_valid=True,stopped=True),
            health_window=dict(non_motion_inputs_stable=True,transitions=[dict(other_goal_owners=[])]))))
    original_preflight=app.preflight
    monkeypatch.setattr(app,'cooperative',cooperative)
    monkeypatch.setattr(app,'confirm',lambda word,text:stage(word))
    monkeypatch.setattr(app,'service',stage)
    monkeypatch.setattr(app,'ros',ready)
    monkeypatch.setattr(app,'command',inspect)
    monkeypatch.setattr(app,'preflight',lambda:(stage('prepare') or original_preflight()))
    monkeypatch.setattr(task.Launcher,'start',lambda self:pytest.fail('restart must preserve hardware'))
    if failure:
        with pytest.raises(RuntimeError,match=failure):app.restart()
        assert events[-1]==failure
        assert 'start' not in events or failure=='start'
    else:
        app.restart()
        assert events==['pause','CLOSED','STOPPED','disarm','nav-ready','inspect',
            'prepare','preflight','owners','bridge','ARM','prepare-ready','arm','agent','idle',
            'coordinator','当前状态','prepare','preflight','START','idle','start']
    assert events.count('start')<=1


def test_live_preflight_refuses_blocked_or_incomplete_configuration(task,monkeypatch,tmp_path):
    app,_=app_fixture(task,monkeypatch,tmp_path)
    for gates,api in [(['vision required'],True),([],False)]:
        monkeypatch.setattr(app,'cooperative',lambda action:S(stdout=json_report(gates,api)))
        with pytest.raises(RuntimeError):task.TaskLauncher.preflight(app)


def json_report(gates,api):
    import json
    return json.dumps(dict(live_gates=gates,api_environment_complete=api))


def test_existing_component_is_never_launched_again(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'nodes',lambda:['/go2_cooperative_task_bridge_0'])
    with pytest.raises(RuntimeError):task.TaskLauncher.component(app,'bridge','go2_cooperative_task_bridge')
    assert not events


def test_failed_arm_prevents_agent_launch(task,monkeypatch,tmp_path):
    app,events=app_fixture(task,monkeypatch,tmp_path)
    monkeypatch.setattr(app,'state',lambda:'DISARMED')
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='data: true'))
    with pytest.raises(RuntimeError):app.prepare()
    assert 'arm' in events and 'agent' not in events


@pytest.mark.parametrize('word',['CAMERA','START','READY','CURRENT','ARM'])
def test_automatic_confirmation_never_reads_stdin(task,monkeypatch,tmp_path,word):
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    app=task.AutomaticLauncher(ROOT,True,'',cooperative_hardware=True)
    monkeypatch.setattr('builtins.input',lambda *a:pytest.fail('automatic entry must not read confirmation words'))
    values={'dry_run':'False','require_system_ready':'True','fault_on_foreign_sport_request':'False'}
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='Boolean value is: '+values[a[-1]]))
    app.confirm(word,'readiness checks still follow')


def test_automatic_reuse_checks_live_sport_parameters(task,monkeypatch,tmp_path):
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    app=task.AutomaticLauncher(ROOT,True,'',cooperative_hardware=True)
    monkeypatch.setattr(app,'ros',lambda *a,**kw:S(stdout='Boolean value is: True'))
    with pytest.raises(RuntimeError,match='dry_run'):app.confirm('CURRENT','reuse')


def test_automatic_entry_cannot_confirm_fault_reset(task,monkeypatch,tmp_path):
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    app=task.AutomaticLauncher(ROOT,True,'',cooperative_hardware=True)
    with pytest.raises(RuntimeError):app.confirm('RESET','do not reset')


@pytest.mark.parametrize('failure',[None,'preflight','hardware','prepare'])
def test_automatic_pipeline_preserves_order_and_stops_on_failure(task,monkeypatch,tmp_path,failure):
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    app=task.AutomaticLauncher(ROOT,True,'',cooperative_hardware=True)
    events=[]
    def stage(name):
        events.append(name)
        if name==failure:raise RuntimeError(name)
    monkeypatch.setattr(app,'preflight',lambda:stage('preflight'))
    monkeypatch.setattr(task.Launcher,'start',lambda app:stage('hardware'))
    monkeypatch.setattr(app,'prepare',lambda:stage('prepare'))
    monkeypatch.setattr(app,'start_task',lambda:stage('task-start'))
    if failure:
        with pytest.raises(RuntimeError):app.run()
        assert events==['preflight','hardware','prepare'][:['preflight','hardware','prepare'].index(failure)+1]
    else:
        app.run()
        assert events==['preflight','hardware','prepare','task-start']


@pytest.mark.parametrize('action',['run','restart'])
def test_main_selects_launcher_for_automatic_run_and_manual_restart(task,monkeypatch,tmp_path,action):
    import builtins
    monkeypatch.setenv('REPO',str(ROOT))
    monkeypatch.setenv('COOP_SIDE','go2')
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    monkeypatch.setenv('DISPLAY',':test')
    monkeypatch.setattr(sys,'argv',['start_go2_cooperative_task.py',action])
    monkeypatch.setattr(sys,'stdin',S(isatty=lambda:True))
    monkeypatch.setattr(task.shutil,'which',lambda name:'/usr/bin/gnome-terminal')
    monkeypatch.setattr(task.tempfile,'mkdtemp',lambda **kw:str(tmp_path))
    monkeypatch.setattr(task,'open',lambda path,*a,**kw:builtins.open(tmp_path/Path(path).name,*a,**kw),raising=False)
    calls=[]
    launcher=task.AutomaticLauncher if action=='run' else task.TaskLauncher
    monkeypatch.setattr(launcher,action,lambda app:calls.append(type(app)),raising=False)
    assert task.main()==0
    assert calls==[launcher]


@pytest.mark.parametrize('terminal,desktop',[(False,True),(True,False)])
def test_restart_rejects_missing_terminal_or_desktop_before_pause(task,monkeypatch,tmp_path,terminal,desktop):
    monkeypatch.setenv('REPO',str(ROOT))
    monkeypatch.setenv('COOP_SIDE','go2')
    monkeypatch.setenv('COOP_CONFIG',str(tmp_path))
    monkeypatch.setenv('DISPLAY',':test' if desktop else '')
    monkeypatch.setenv('WAYLAND_DISPLAY','')
    monkeypatch.setattr(sys,'argv',['start_go2_cooperative_task.py','restart'])
    monkeypatch.setattr(sys,'stdin',S(isatty=lambda:terminal))
    monkeypatch.setattr(task.shutil,'which',lambda name:'/usr/bin/gnome-terminal' if desktop else None)
    monkeypatch.setattr(task.TaskLauncher,'restart',lambda app:pytest.fail('invalid environment must not pause'),raising=False)
    with pytest.raises(SystemExit) as error:task.main()
    assert error.value.code==2


@pytest.mark.parametrize('help_only',[False,True])
def test_full_entry_loads_cooperative_environment_and_runs_automatic_mode(tmp_path,help_only):
    scripts=tmp_path/'scripts';scripts.mkdir()
    entry=scripts/'start_go2_cooperative.sh'
    entry.write_text((ROOT/'scripts/start_go2_cooperative.sh').read_text())
    marker=tmp_path/'loaded'
    (tmp_path/'Cooperative_env.bash').write_text('[[ "$1" == go2 ]] || return 91\n'+f'touch {str(marker)!r}\n')
    (scripts/'start_go2_cooperative_task.py').write_text('import sys;print(sys.argv[1:])\n')
    result=subprocess.run(['bash',str(entry),*(['--help'] if help_only else [])],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    if help_only:
        assert not marker.exists() and 'ARM' in result.stdout
    else:
        assert marker.exists() and "['run']" in result.stdout


@pytest.mark.parametrize('action',['prepare','start','pause','disarm','restart','--help'])
def test_shell_actions_and_help_need_no_real_environment(tmp_path,action):
    scripts=tmp_path/'scripts';scripts.mkdir()
    entry=scripts/'start_go2_cooperative_task.sh'
    entry.write_text((ROOT/'scripts/start_go2_cooperative_task.sh').read_text())
    marker=tmp_path/'loaded'
    (tmp_path/'Cooperative_env.bash').write_text('[[ "$1" == go2 ]] || return 91\n'+f'touch {str(marker)!r}\n')
    (scripts/'start_go2_cooperative_task.py').write_text('import sys;print(sys.argv[1:])\n')
    result=subprocess.run(['bash',str(entry),action],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    if action=='--help':assert not marker.exists()
    else:assert marker.exists() and action in result.stdout


@pytest.mark.parametrize('changes', [dict(sequence=1),dict(reply='SYNCED'),dict(resource_id='old'),
    dict(token='old'),dict(permit_command_id='old'),dict(resource_status='old'),
    dict(requested_resource='old'),dict(requested_command_id='old'),dict(pose_valid=True),
    dict(common_pose=S(x=1.,y=0.,theta=0.))])
def test_final_contract_rejects_dirty_waiting_samples(task,changes):
    assert task.idle_issues(state(**changes),'go2',config()['alignment']['robots']['go2'])
