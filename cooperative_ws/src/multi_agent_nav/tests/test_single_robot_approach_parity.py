"""Differential approach decisions against actual methods from the frozen Git object.

ROS transport is controlled here; no nodes, network, or hardware are started.
"""
import ast
from concurrent.futures import Future
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace as S

import pytest
import yaml

SHA = '32eab6244ec4b483807f8ed68e8667f5495874bb'
ROOT = Path(__file__).resolve().parents[4]
METHODS = ('navigation_tick', 'safety_tick', 'target_reference_distance',
    'approach_stop_radius', 'reset_approach_stop', 'request_approach_cancel',
    'on_approach_cancel_response', 'begin_approach_stop', 'advance_approach_stop',
    'finish_approach_success', 'on_arrival_odom', 'plan_then_navigate', 'goal_key',
    'request_next_plan', 'on_plan_response', 'on_plan_result', 'reject_current_plan',
    'send_navigation_goal', 'on_goal_response', 'on_navigation_result', 'elapsed_since')


def frozen(path):
    return subprocess.check_output(['git', 'show', SHA+':'+path], cwd=ROOT, text=True)


@pytest.fixture(scope='module')
def reference_source():
    tree = ast.parse(frozen('vlm_nav/vlm_navigator.py'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'VLMNavigator')
    node.bases = []; node.decorator_list = []
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    scope = dict(math=math, time=S(monotonic=lambda: 10.),
        GoalStatus=S(STATUS_SUCCEEDED=4, STATUS_CANCELED=5),
        ComputePathToPose=S(Goal=lambda: S()), NavigateToPose=S(Goal=lambda: S()),
        sample_polyline=lambda points, count: (points, None))
    for name in ('DISARMED API_ERROR SUCCEEDED FAILED SENSOR_WAITING TARGET_CONFIRMING '
                 'FRONTIER_SELECTING APPROACHING APPROACH_STOPPING TARGET_ALIGNING '
                 'TARGET_REOBSERVING SCANNING EXPLORING').split():scope[name] = name
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 SHA+':vlm_nav/vlm_navigator.py', 'exec'), scope)
    params = {}
    for path in ('config/robot.yaml', 'config/robot_go2.yaml'):
        params.update(yaml.safe_load(frozen(path))['vlm_nav']['ros__parameters'])
    return scope['VLMNavigator'], scope, params


@pytest.fixture
def reference(reference_source, monkeypatch):
    import multi_agent_nav.single_robot_approach as extracted
    monkeypatch.setattr(extracted, 'ComputePathToPose', S(Goal=lambda: S()))
    return reference_source


def done(value=None, error=None):
    f = Future()
    if error:f.set_exception(error)
    else:f.set_result(value)
    return f


class Harness:
    def __init__(self, cls, params, scope=None):
        self.now = 10.; self.pose = (0., 0., .7); self.events = []; self.plans = []; self.goals = []
        self.cancel = Future(); self.terminal = Future()
        a = self.a = cls.__new__(cls)
        a.p = S(**params); a.clock = lambda: self.now
        if scope is not None:scope['time'] = S(monotonic=lambda: self.now)
        a.state = 'APPROACHING'; a.target_reference_position = (2., 0.)
        a.goal_token = a.plan_token = 0
        a.goal_handle = a.plan_handle = None
        a.goal_pending = a.plan_pending = False
        a.goal_kind = a.goal_pose = a.active_motion_origin = None
        a.blocked_goals = {}; a.arrival_retry_count = 0; a.arrival_last_odom_received = 0.
        a.reset_approach_stop(); a.grid = a.map_message = object(); a.session_origin = (0., 0.)
        a.last_valid_rgbd_received = a.last_camera_tf_success = a.last_robot_tf_success = self.now
        a.get_parameter = lambda name: S(value=True)
        a.external_safety_gates_healthy = lambda: True
        a.robot_pose = lambda: self.pose
        a.easy_case_enabled = lambda: False
        a.publish_stop = lambda: self.events.append(('stop',))
        a.set_state = lambda state: (setattr(a, 'state', state), self.events.append(('state', state)))
        a.fail_safe = lambda reason: (setattr(a, 'state', 'FAILED'), self.events.append(('fail', reason)))
        a.get_logger = lambda: S(info=lambda *x: None, warn=lambda *x: None)
        a.make_pose_stamped = lambda x,y,yaw: (x,y,yaw)
        a.marker_pub = None
        a.publish_goal_marker = lambda *args: None
        a.emit = lambda *args, **kwargs: None
        a.path_planner = S(wait_for_server=lambda **kw: True, send_goal_async=self.plan)
        a.navigator = S(wait_for_server=lambda **kw: True, send_goal_async=self.navigate)
        a.adapter = S(start_approach_navigation=self.adapt_navigation)
        a.task_id = 'task'; a.command_id = 'command'; a.goal_id = ''; a.goal_kind_wire = 'PICKUP_NAV'

    def plan(self, goal):
        self.events.append(('plan', goal.goal, goal.use_start))
        f = Future(); self.plans.append(f); return f

    def planned(self, status=4, points=((0.,0.),(1.5,0.)), accepted=True):
        result = Future()
        self.plans[-1].set_result(S(accepted=accepted, get_result_async=lambda:result,
                                    cancel_goal_async=lambda:done()))
        path = [S(pose=S(position=S(x=x,y=y))) for x,y in points]
        result.set_result(S(status=status, result=S(path=S(poses=path))))

    def navigate(self, goal):
        self.events.append(('navigate', goal.pose)); f = Future(); self.goals.append(f); return f

    def adapt_navigation(self, owner, pose, token):
        f = self.navigate(S(pose=(pose.x,pose.y,pose.yaw)))
        def accepted(response):
            owner.on_goal_response(response,token,'approach')
            if response.result().accepted:
                response.result().get_result_async().add_done_callback(
                    lambda result:owner.on_navigation_result(result,token,'approach'))
        f.add_done_callback(accepted)
        return 'goal'

    def accept(self):
        self.goals[-1].set_result(S(accepted=True, get_result_async=lambda:self.terminal,
            cancel_goal_async=lambda:(self.events.append(('cancel',)) or self.cancel)))

    def odom(self, x=0., y=0., z=0., angular_z=0.):
        self.a.on_arrival_odom(S(twist=S(twist=S(linear=S(x=x,y=y,z=z),
            angular=S(x=10.,y=10.,z=angular_z)))))


def pair(reference):
    from multi_agent_nav.single_robot_approach import SingleRobotApproach
    cls, scope, params = reference
    return Harness(cls, params, scope), Harness(SingleRobotApproach, params)


def compare(reference, scenario):
    old, new = pair(reference)
    for h in (old,new):scenario(h)
    assert new.events == old.events
    assert new.a.state == old.a.state
    assert new.a.arrival_retry_count == old.a.arrival_retry_count
    return new


@pytest.mark.parametrize('status,points,accepted', [
    (4,((0.,0.),(1.5,0.)),True), (4,((1.5,0.),),True),
    (6,((0.,0.),),True), (4,(),True), (4,(),False)])
def test_plan_uses_original_request_or_original_failure(reference,status,points,accepted):
    def scenario(h):
        h.a.navigation_tick(); h.planned(status,points,accepted)
    h = compare(reference,scenario)
    if accepted and status == 4 and points:
        assert ('navigate',(2.,0.,.7)) in h.events
    else:assert h.a.state == 'FAILED'


@pytest.mark.parametrize('distance', [.889999,.89,.890001])
def test_initial_distance_boundary(reference,distance):
    def scenario(h):
        h.a.target_reference_position=(0.,0.); h.pose=(distance,0.,.7); h.a.navigation_tick()
    compare(reference,scenario)


@pytest.mark.parametrize('distance', [.809999,.81,.810001])
@pytest.mark.parametrize('status', [4,5,6])
def test_controlled_stop_terminal_and_distance(reference,distance,status):
    def scenario(h):
        h.a.navigation_tick(); h.planned(); h.accept()
        h.pose=(2.-.88,0.,.7); h.a.safety_tick()
        h.cancel.set_result(S(goals_canceling=[1]))
        for _ in range(3):h.odom()
        h.a.advance_approach_stop(); assert h.a.state == 'APPROACH_STOPPING'
        h.terminal.set_result(S(status=status)); h.pose=(2.-distance,0.,.7)
        h.a.advance_approach_stop()
    compare(reference,scenario)


@pytest.mark.parametrize('kind', ['moving_sample','stale','cancel_timeout','stop_timeout','double_retry','succeeded','aborted'])
def test_stop_sequences(reference,kind):
    def scenario(h):
        a=h.a; a.navigation_tick(); h.planned(); h.accept()
        if kind in ('succeeded','aborted'):
            h.terminal.set_result(S(status=4 if kind=='succeeded' else 6))
            if kind=='aborted':return
            assert a.state == 'APPROACH_STOPPING'
        else:
            h.pose=(1.12,0.,.7); a.safety_tick()
            h.cancel.set_result(None)
            if kind=='cancel_timeout':
                h.now+=3.; a.advance_approach_stop(); assert a.state=='APPROACH_STOPPING'
                h.now+=.001; a.advance_approach_stop(); return
            h.terminal.set_result(S(status=5))
        h.odom();h.odom()
        if kind=='moving_sample':h.odom(y=.04)
        h.odom(z=100.)
        if kind=='stale':h.now+=.501
        if kind=='stop_timeout':h.odom(x=.1);h.now+=5.001
        h.pose=(1.2,0.,.7); a.advance_approach_stop()
        if kind=='double_retry':
            # Re-enter original stopping with a terminal action twice outside 0.81.
            a.state='APPROACHING'; a.arrival_retry_count=0
            for _ in range(2):
                a.begin_approach_stop('nav2_action_succeeded',action_terminal=True)
                h.pose=(1.18,0.,.7)
                for _ in range(3):h.odom()
                a.advance_approach_stop()
    compare(reference,scenario)


@pytest.mark.parametrize('kind', ['radius','timeout','distance_first','terminal_clear'])
def test_watchdogs_use_parameters_and_source_order(reference,kind):
    def scenario(h):
        a=h.a; a.p.max_travel_radius=7.; a.p.goal_timeout=20.
        a.navigation_tick();h.planned();h.accept()
        assert a.active_motion_origin == (0.,0.)
        if kind=='radius':h.pose=(7.501,0.,0.)
        if kind=='timeout':h.now=30.001
        if kind=='distance_first':h.pose=(1.2,0.,0.);h.now=30.001
        if kind=='terminal_clear':
            h.terminal.set_result(S(status=4));assert a.active_motion_origin is None
        a.last_valid_rgbd_received = a.last_camera_tf_success = a.last_robot_tf_success = h.now
        a.safety_tick()
    compare(reference,scenario)


@pytest.mark.parametrize('simulation,share,robot,paused', [
    (False,False,'go2',False), (True,False,'go2',True),
    (False,True,'go2',True), (False,False,'ranger',True)])
def test_missing_path_monitor_is_mode_scoped(simulation,share,robot,paused):
    from test_local_tasks import private_mission
    from dataclasses import replace
    m=private_mission().machines['go2']
    m.context=replace(m.context,simulation=simulation,share_c_coordinates=share,robot_id=robot)
    m._monitor_path(replace(m.health,localization_valid=True,pose=__import__(
        'multi_agent_nav.models',fromlist=['Pose2D']).Pose2D(), path=[],path_valid=False,
        footprint=[[0.,0.],[1.,0.],[0.,1.]]),10.)
    assert m.paused is paused


def test_outer_acceptance_waits_without_failing_or_retrying():
    from test_local_tasks import private_mission
    from dataclasses import replace
    from multi_agent_nav.models import Command, Pose2D
    m=private_mission().machines['go2']
    m.context=replace(m.context,simulation=False)
    m.command=Command(task_id='t',command_id='c',kind='APPROACH_PICKUP')
    m.context=replace(m.context,task_id='t')
    m.local_target=Pose2D();m.local_approach_succeeded=True
    m.health=replace(m.health,pose=Pose2D(),localization_valid=True,nav_ready=True,stopped=False,motion_terminal=True)
    assert m.complete_local_execution(('t','c'),'TARGET_APPROACHED') is False
    assert not m.state().ready and m.state().reply != 'FAILED'
    assert not m.take_effects() and m.retry == 0
    m.health=replace(m.health,stopped=True)
    m.complete_local_execution(('t','c'),'TARGET_APPROACHED')
    assert m.state().ready


def test_motion_log_failure_does_not_change_health(navigation):
    a,_,_=navigation
    a.record_event=lambda event: (_ for _ in ()).throw(OSError('full'))
    warnings=[];a.node.get_logger=lambda:S(warning=warnings.append)
    a.single_robot_mode=True
    a._motion_event('STOP_COMMAND_SENT')
    assert not a.motion_fault and warnings

from test_go2_executing_path import navigation


@pytest.fixture
def live_stack(navigation, monkeypatch):
    from dataclasses import replace
    from test_local_tasks import private_mission
    from multi_agent_nav.models import Command, Health, Pose2D
    from multi_agent_nav.runner import AgentRunner
    from multi_agent_nav.single_robot_approach import load_parameters
    from geometry_msgs.msg import PoseStamped
    a,now,requests=navigation
    # Restore actual ROS message constructors after pure differential fixtures.
    import multi_agent_nav.single_robot_approach as module
    from nav2_msgs.action import ComputePathToPose
    monkeypatch.setattr(module,'ComputePathToPose',ComputePathToPose)
    plans=[];events=[];zeros=[];warnings=[]
    health=[Health(Pose2D(0.,0.,.7),True,True,True,[],False,[[0.,0.],[1.,0.],[0.,1.]])]
    def get_health():return replace(health[0],motion_terminal=not a.motions)
    a.get_health=get_health
    a.costmap_transform=lambda *args,**kwargs:object()
    a.vision=S(poll=lambda:None,cancel=lambda:None)
    a.plans={};a.simulation=False
    a.node.stop_pub=S(publish=lambda message:zeros.append(message))
    a.node.publish_stop=lambda:zeros.append('zero')
    a.node.get_logger=lambda:S(warning=warnings.append,warn=warnings.append,info=lambda *x:None)
    def pose(x,y,yaw):
        p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
        return p
    a.node.make_pose_stamped=pose
    def plan(goal):
        f=Future();plans.append((goal,f));return f
    a.node.path_planner=S(wait_for_server=lambda **kw:True,send_goal_async=plan)
    a.node.navigator.wait_for_server=lambda **kw:True
    mission=private_mission();m=mission.machines['go2']
    m.context=replace(m.context,simulation=False,task_id='task')
    m._state.task_id='task';m.command=Command(task_id='task',command_id='command',kind='APPROACH_PICKUP')
    m.peer_received=m.peer_task_received=m.sync_received=m.task_started=10.
    m.executing=True
    config=mission.config
    config['robot_go2']['arrival_config_files']=[str(ROOT/'config/robot.yaml'),str(ROOT/'config/robot_go2.yaml')]
    r=AgentRunner(m,a,{},lambda value:None,clock=lambda:now[0],config=config)
    r.execution_identity=('task','command')
    r.execution=r.local_approach(Pose2D(2.,0.),'PICKUP_NAV',events.append)
    def tick(dt=.05):
        now[0]+=dt;m.sync_received=m.peer_task_received=m.peer_received=now[0]
        r.tick(now[0])
    def planned(status=4,accepted=True):
        f=Future()
        plans[-1][1].set_result(S(accepted=accepted,get_result_async=lambda:f,cancel_goal_async=lambda:done()))
        f.set_result(S(status=status,result=S(path=S(poses=[pose(0.,0.,0.),pose(1.5,0.,0.)]))))
    def odom(x=0.):
        a.on_odometry(S(header=S(stamp=S(sec=int(now[0]),nanosec=int(now[0]%1*1e9))),
            twist=S(twist=S(linear=S(x=x,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.)))))
    return S(a=a,r=r,m=m,now=now,requests=requests,plans=plans,events=events,zeros=zeros,
        health=health,tick=tick,planned=planned,odom=odom,warnings=warnings)


def test_real_adapter_runner_distance_cancel_then_outer_wait(live_stack):
    from dataclasses import replace
    from multi_agent_nav.models import Pose2D
    s=live_stack;s.tick();s.planned()
    assert len(s.requests)==1 and len(s.plans)==1
    request=s.a.motions[s.r.approach.goal_id]
    assert request['target']==[2.,0.,.7]
    from test_go2_executing_path import accept
    terminal=accept(s.requests[0])
    s.health[0]=replace(s.health[0],pose=Pose2D(1.2,0.),stopped=False)
    s.tick(.21)
    assert s.r.approach.state=='APPROACH_STOPPING'
    for _ in range(3):s.odom()
    s.tick(.21)
    assert not s.m.state().ready
    terminal.set_result(S(status=5))
    s.tick(.21);s.tick()
    assert s.m.local_approach_succeeded and not s.m.state().ready
    assert s.r.local_completion is not None
    for _ in range(4):s.tick()
    assert len(s.plans)==len(s.requests)==1 and s.m.retry==0
    s.health[0]=replace(s.health[0],stopped=True)
    s.tick()
    assert s.m.state().ready and not s.a.results


@pytest.mark.parametrize('status',[5,6])
def test_real_adapter_uncontrolled_failure_never_retries(live_stack,status):
    from test_go2_executing_path import accept
    s=live_stack;s.tick();s.planned();terminal=accept(s.requests[0])
    terminal.set_result(S(status=status))
    s.tick();s.tick()
    assert s.m.state().reply=='FAILED' and not s.m.state().ready
    assert len(s.requests)==len(s.plans)==1


def test_late_goal_after_external_stop_cannot_complete(live_stack):
    from test_go2_executing_path import accept
    s=live_stack;s.tick();s.planned()
    s.m.stop('external CANCEL');s.tick()
    terminal=accept(s.requests[0]);terminal.set_result(S(status=4));s.tick()
    assert s.m.paused and not s.m.local_approach_succeeded and not s.m.state().ready


def test_approach_parameters_are_checked_without_rewriting(tmp_path):
    from multi_agent_nav.single_robot_approach import load_parameters
    files=[str(ROOT/'config/robot.yaml'),str(ROOT/'config/robot_go2.yaml')]
    p=load_parameters(files)
    assert p.arrival_odom_topic=='/odometry' and p.target_success_radius==.81
    bad=tmp_path/'bad.yaml';bad.write_text('vlm_nav:\n  ros__parameters:\n    goal_timeout: 121.0\n')
    with pytest.raises(ValueError,match='goal_timeout'):
        load_parameters(files+[str(bad)])
    assert '121.0' in bad.read_text()


def test_cancel_pending_plan_cancels_late_accepted_handle(live_stack):
    s=live_stack;s.tick();s.m.stop('task replaced');s.tick()
    cancelled=[]
    s.plans[-1][1].set_result(S(accepted=True,cancel_goal_async=lambda:cancelled.append(True)))
    assert cancelled and not s.requests and not s.m.state().ready


def test_native_terminal_already_available_does_not_restore_live_handle(live_stack):
    s=live_stack;s.tick();s.planned()
    s.requests[0].set_result(S(accepted=True,goal_id=S(uuid=[1]*16),
        get_result_async=lambda:done(S(status=4)),cancel_goal_async=lambda:done()))
    assert s.r.approach.arrival_action_terminal
    assert s.r.approach.goal_handle is None
    assert not s.a.motions


def test_late_motion_logs_keep_original_task_recorder(navigation):
    from multi_agent_nav.models import Pose2D
    from test_go2_executing_path import accept
    a,_,requests=navigation
    old=[];new=[];a.single_robot_mode=True;a.record_event=old.append
    a.start_navigation(Pose2D(2.,0.),'old','command','VIEWPOINT_NAV')
    terminal=accept(requests[0])
    a.record_event=new.append
    terminal.set_result(S(status=5))
    assert any(e['event']=='MOTION_TERMINAL' for e in old)
    assert not new


def test_wrong_context_cannot_release_completed_approach():
    from test_local_tasks import private_mission
    from dataclasses import replace
    from multi_agent_nav.models import Command, Pose2D
    m=private_mission().machines['go2'];m.context=replace(m.context,simulation=False,task_id='new')
    m.command=Command(task_id='old',command_id='command',kind='APPROACH_PICKUP')
    m.local_target=m.health.pose;m.local_approach_succeeded=True
    m.health=replace(m.health,nav_ready=True,localization_valid=True,stopped=True,motion_terminal=True)
    m.complete_local_execution(('old','command'),'TARGET_APPROACHED')
    assert not m.state().ready


def test_failure_clears_source_watchdog_state(live_stack):
    from test_go2_executing_path import accept
    s=live_stack;s.tick();s.planned();accept(s.requests[0])
    a=s.r.approach
    a.fail_safe('test failure')
    assert a.active_motion_origin is None
    assert not a.goal_pending and a.goal_handle is None


def test_non_path_health_validation_is_preserved():
    from test_local_tasks import private_mission
    from dataclasses import replace
    m=private_mission().machines['go2'];m.context=replace(m.context,simulation=False)
    m._monitor_path(replace(m.health,localization_valid=True,path_valid=False,footprint=[]),10.)
    assert m.paused


def test_goal_started_changes_at_planning_and_navigation(reference):
    def scenario(h):
        h.a.navigation_tick();assert h.a.goal_started==10.
        h.now=11.;h.planned();assert h.a.goal_started==11.
        h.accept();assert h.a.goal_started==11.
    compare(reference,scenario)


@pytest.mark.parametrize('broken_log',[False,True])
def test_logging_failure_does_not_change_approach_result(live_stack,broken_log):
    from dataclasses import replace
    from multi_agent_nav.models import Pose2D
    from test_go2_executing_path import accept
    s=live_stack
    def record(event):
        if broken_log:raise OSError('disk full')
        s.events.append(event)
    s.r.execution=s.r.local_approach(Pose2D(2.,0.),'PICKUP_NAV',record)
    s.tick();s.planned();terminal=accept(s.requests[0])
    s.health[0]=replace(s.health[0],pose=Pose2D(1.2,0.))
    terminal.set_result(S(status=4))
    for _ in range(3):s.odom()
    s.tick(.21);s.tick()
    assert s.m.state().ready and len(s.plans)==len(s.requests)==1
    assert not s.a.motion_fault
    if broken_log:assert s.warnings


def test_already_inside_success_radius_needs_samples_but_no_navigation(live_stack):
    from dataclasses import replace
    from multi_agent_nav.models import Pose2D
    s=live_stack;s.health[0]=replace(s.health[0],pose=Pose2D(1.2,0.))
    s.tick();assert not s.plans and not s.requests and not s.m.state().ready
    for _ in range(3):s.odom()
    s.tick(.21);s.tick()
    assert s.m.state().ready and not s.requests


def test_only_one_original_distance_retry_is_allowed(live_stack):
    from dataclasses import replace
    from multi_agent_nav.models import Pose2D
    from test_go2_executing_path import accept
    s=live_stack;s.tick();s.planned();terminal=accept(s.requests[0])
    s.health[0]=replace(s.health[0],pose=Pose2D(1.18,0.))
    s.tick(.21);terminal.set_result(S(status=5))
    for _ in range(3):s.odom()
    s.tick(.21);s.tick(.51)
    assert len(s.plans)==2
    assert s.r.approach.approach_stop_radius()==.81
    s.planned();terminal=accept(s.requests[1]);terminal.set_result(S(status=4))
    for _ in range(3):s.odom()
    s.tick(.21);s.tick()
    assert not s.m.state().ready and s.m.state().reply=='FAILED'
    assert len(s.requests)==len(s.plans)==2


@pytest.mark.parametrize('fault',['localization','heartbeat','replacement'])
def test_cooperative_interruption_blocks_success(live_stack,fault):
    from dataclasses import replace
    from test_go2_executing_path import accept
    s=live_stack;s.tick();s.planned();terminal=accept(s.requests[0])
    if fault=='localization':s.health[0]=replace(s.health[0],localization_valid=False)
    elif fault=='heartbeat':s.m.peer_task_received=s.now[0]-4.
    else:s.m.command=replace(s.m.command,task_id='other',command_id='new')
    s.r.tick(s.now[0])
    terminal.set_result(S(status=4));s.r.tick(s.now[0])
    assert not s.m.state().ready and not s.m.local_approach_succeeded
    assert len(s.requests)==1


@pytest.mark.parametrize('when',['entry','plan_response','active'])
def test_other_motion_owner_blocks_live_approach(live_stack,when):
    from test_go2_executing_path import accept
    s=live_stack
    if when!='entry':s.tick()
    if when=='active':s.planned();accept(s.requests[0])
    s.r.owners=lambda:['another_navigation_owner']
    if when=='plan_response':s.planned()
    s.tick()
    assert s.m.paused and not s.m.state().ready
    assert len(s.requests)==(1 if when=='active' else 0)
    if when=='entry':assert not s.plans


def test_owner_guard_does_not_add_pretask_pause(live_stack):
    s=live_stack;s.r.execution=None;s.m.command=None;s.m.executing=False
    s.r.owners=lambda:['another_navigation_owner']
    s.tick()
    assert not s.m.paused
