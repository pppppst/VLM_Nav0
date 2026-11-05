"""Paused experiments keep their task and never issue motion before recovery."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from multi_agent_nav.frame_adapter import LocalFrame
from multi_agent_nav.go2_check import standalone_explore_steps
from multi_agent_nav.models import Pose2D
from test_go2_exploration import LocalFixture
from test_semantic_search import catalog_config


def fixture():
    adapter = LocalFixture()
    adapter.detections = {'A': Pose2D(1., 0.)}
    config = {'scene': {'targets': {'A': 'target'}},
              'robot_go2': {'approach_on_detection': False}}
    return adapter, config


def test_initial_unstable_stop_waits_then_runs_initial_observation_once():
    adapter, config = fixture()
    now = [10.]
    stable = [False]
    original = adapter.get_health
    adapter.get_health = lambda: replace(original(), stopped=stable[0])
    events = []
    steps = standalone_explore_steps(adapter, config, LocalFrame(), 20., 'target',
        events.append, lambda: [], lambda: True, lambda: None, clock=lambda: now[0])
    for _ in range(4):
        next(steps)
        now[0] += 10.
        assert adapter.scans == 0 and not adapter.calls
    stable[0] = True
    for _ in range(20):
        try:
            next(steps)
        except StopIteration as result:
            assert result.value == 'TARGET_FOUND'
            break
        now[0] += .1
    else:
        pytest.fail('did not resume after confirmed stop')
    assert adapter.scans == 1
    assert any(e.get('state') == 'WAITING' for e in events)


@pytest.mark.parametrize('fault', ['localization', 'nav', 'costmap', 'owner', 'terminal'])
def test_runtime_gates_wait_without_ending_task_or_running_clock(fault):
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter, _ = fixture()
    now = [10.]
    original = adapter.get_health
    failed = [True]
    adapter.get_health = lambda: replace(original(), **(
        {'localization_valid': not failed[0]} if fault == 'localization' else
        {'nav_ready': not failed[0]} if fault == 'nav' else
        {'motion_terminal': not failed[0]} if fault == 'terminal' else {}))
    adapter.costmap_transform = lambda **kwargs: None if fault == 'costmap' and failed[0] else object()
    recovery = ExperimentRecovery(adapter, lambda: ['other'] if fault == 'owner' and failed[0] else [],
        lambda e: None, clock=lambda: now[0])
    wait = recovery.wait('test gate')
    next(wait)
    now[0] = 70.
    next(wait)
    assert recovery.clock() == 10.
    assert recovery.timing(20.)['paused_total_s'] == 60.
    assert not adapter.calls
    failed[0] = False
    with pytest.raises(StopIteration):
        next(wait)
    now[0] = 72.
    assert recovery.clock() == 12.
    assert recovery.timing(20.)['run_remaining_s'] == 8.


def test_closing_wait_never_resumes_or_clears_adapter_fault():
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter, _ = fixture()
    adapter.motion_fault = True
    adapter.online = False
    events = []
    recovery = ExperimentRecovery(adapter, lambda: [], events.append)
    wait = recovery.wait('control fault')
    next(wait)
    wait.close()
    assert adapter.motion_fault and not adapter.calls
    assert not any(e.get('event') == 'RECOVERED' for e in events)


def test_two_pauses_accumulate_without_resetting_run_timer():
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter, _ = fixture()
    now = [10.]
    recovery = ExperimentRecovery(adapter, lambda: [], lambda e: None, clock=lambda: now[0])
    for started, ended in [(12., 17.), (20., 27.)]:
        now[0] = started
        wait = recovery.wait('transient failure')
        next(wait)
        now[0] = ended
        with pytest.raises(StopIteration): next(wait)
    assert recovery.timing(30.) == dict(run_elapsed_s=5., pause_elapsed_s=0.,
        paused_total_s=12., run_remaining_s=15.)


def test_camera_wait_outside_scan_freezes_run_budget():
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter, _ = fixture()
    now = [10.]
    adapter.scan = None
    adapter.perception_wait_allowed = lambda health: True
    recovery = ExperimentRecovery(adapter, lambda: [], lambda e: None, clock=lambda: now[0])
    recovery.observe_camera()
    now[0] = 40.
    recovery.observe_camera()
    assert recovery.timing(20.)['run_elapsed_s'] == 0.
    assert recovery.timing(20.)['paused_total_s'] == 30.


def test_interrupted_target_approach_reobserves_instead_of_skipping_target():
    from multi_agent_nav.go2_exploration import explore_steps, ExplorationStopped
    adapter, config = fixture()
    config['robot_go2']['approach_on_detection'] = True
    progress = {}
    attempts = []
    def approach(*args):
        attempts.append(args)
        if len(attempts) == 1:
            raise ExplorationStopped('localization lost')
        yield
        return 'TARGET_APPROACHED'
    def steps():
        return explore_steps(adapter, config, LocalFrame(), 20., 'target', lambda e: None,
            lambda: [], lambda: True, lambda: None, clock=lambda: 10.,
            initial_camera_check=True, approaching=approach, progress=progress)
    with pytest.raises(ExplorationStopped): next(steps())
    restarted = steps()
    next(restarted)
    with pytest.raises(StopIteration) as result: next(restarted)
    assert result.value.value == 'TARGET_APPROACHED'
    assert len(attempts) == 2 and adapter.scans == 2 and not adapter.calls


@pytest.mark.parametrize('error',[TimeoutError('timeout'),ValueError('invalid response'),
    ConnectionError('connection reset')])
def test_candidate_request_error_waits_then_retries_without_negative_observation(catalog_config,error):
    from concurrent.futures import Future
    from multi_agent_nav.adapters.fake import completed
    adapter=LocalFixture()
    now=[0.];events=[];requests=[]
    def choose(snapshot):
        requests.append(snapshot)
        if len(requests)==1:raise error
        return dict(action='inspect_furniture',furniture_id=snapshot['candidates'][0]['furniture_id'],reason='visible')
    adapter.client=SimpleNamespace(choose_furniture=choose)
    def submit(fn):
        from multi_agent_nav.adapters.go2 import AsyncVision
        import time
        worker=AsyncVision.__new__(AsyncVision)
        worker.job=Future();worker.reply=Future();worker.deadline=time.monotonic()+10.
        try:worker.job.set_result(fn())
        except Exception as error:worker.job.set_exception(error)
        worker.poll()
        return worker.reply
    adapter.vision=SimpleNamespace(submit=submit)
    steps=standalone_explore_steps(adapter,catalog_config,LocalFrame(),10.,'target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0])
    for _ in range(200):
        try:next(steps)
        except StopIteration as result:
            assert result.value=='EXHAUSTED'
            break
        now[0]+=.05;adapter.advance(now[0])
    else:pytest.fail('candidate selection did not recover')
    assert any(e.get('state')=='WAITING' for e in events)
    assert len(requests)==3 and len(adapter.calls)==2


def test_recovery_waits_for_timed_out_vision_worker_without_queueing_more_work():
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter,_=fixture()
    adapter.vision.available=False
    now=[10.]
    recovery=ExperimentRecovery(adapter,lambda:[],lambda e:None,clock=lambda:now[0])
    steps=recovery.wait('candidate selection timeout')
    next(steps);now[0]=50.
    next(steps)
    assert recovery.timing(20.)['run_elapsed_s']==0.
    adapter.vision.available=True
    with pytest.raises(StopIteration):next(steps)


def test_unknown_rotation_map_counts_as_pause_and_resumes_scan():
    from multi_agent_nav.experiment_recovery import ExperimentRecovery
    adapter=LocalFixture();now=[0.];events=[];ready=[False]
    adapter.local_rotation_check=lambda pose:dict(status='free' if ready[0] else 'pending')
    config=dict(scene=dict(targets=dict(A='target'),viewpoints=dict(go2=[dict(id='p',pose=[1.,0.,0.])])),
                robot_go2=dict(scan_angles=[0.]))
    recovery=ExperimentRecovery(adapter,lambda:[],events.append,clock=lambda:now[0])
    steps=standalone_explore_steps(adapter,config,LocalFrame(),10.,'target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0],recovery=recovery)
    for _ in range(50):
        next(steps);now[0]+=.05;adapter.advance(now[0])
        if recovery.paused_at is not None:break
    else:pytest.fail('rotation map did not enter pause')
    elapsed=recovery.timing(10.)['run_elapsed_s']
    now[0]+=30.;next(steps)
    assert recovery.timing(10.)['run_elapsed_s']==elapsed
    ready[0]=True
    for _ in range(20):
        try:next(steps)
        except StopIteration as result:
            assert result.value=='EXHAUSTED';break
        now[0]+=.05;adapter.advance(now[0])
    else:pytest.fail('scan did not resume')
