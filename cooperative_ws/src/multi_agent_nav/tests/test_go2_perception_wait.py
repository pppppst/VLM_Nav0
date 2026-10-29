"""Stopped perception loss waits for reconnection, without granting motion authority."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.adapters.go2 import Go2Adapter
from multi_agent_nav.frame_adapter import LocalFrame
from multi_agent_nav.go2_check import standalone_explore_steps
from multi_agent_nav.go2_exploration import ExplorationStopped
from multi_agent_nav.models import Observation, Pose2D
from test_go2_health_diagnostics import healthy_adapter
from test_field_preparation import stamp


@pytest.fixture
def waiting_scan(monkeypatch):
    now=[10.]
    monkeypatch.setattr('multi_agent_nav.adapters.go2.time.monotonic',lambda:now[0])
    a=healthy_adapter();a.scan=None;a.config.update(scan_timeout=1.,scan_angles=[0.],api_timeout=.5)
    a.node.rgbd_sync_status['healthy']=False
    a.vision=S(poll=lambda:None,available=False)
    a.results=[];a.frame=LocalFrame();a.last_motion=('',{})
    events=[];stops=[]
    a.record_event=events.append
    def stop(reason):
        stops.append(reason)
        if a.scan:a.scan['reply'].cancel();a.scan=None
    a.stop_motion=stop
    reply=a.start_scan({'A':'target'},'scan',stop_on_target='A',current_view_only=True)
    return a,reply,now,stops,events


def test_stopped_rgbd_loss_preserves_scan_and_true_health_fields(waiting_scan):
    a,reply,now,stops,events=waiting_scan
    a.tick()
    assert not reply.done() and a.scan is not None and not stops
    assert a.get_health().nav_ready
    assert a.last_health_details['checks']['rgbd_ready'] is False
    a.node.rgbd_sync_status['healthy']=True
    a.tick()
    assert a.scan['phase']=='FRAME' and not stops


def test_perception_wait_excludes_disconnection_from_scan_deadline(waiting_scan):
    a,reply,now,stops,events=waiting_scan
    a.tick();now[0]=150.
    a.odom_at=now[0];a.odom_stamp=10.
    a.node.external_gate_receipts=dict.fromkeys(a.node.external_gate_receipts,now[0])
    a.costmap_transform=lambda *args,**kwargs:Pose2D()
    a.tick()
    assert not reply.done() and not stops
    a.node.rgbd_sync_status['healthy']=True
    a.tick()
    assert not reply.done() and not stops
    assert a.scan['deadline']==151.


@pytest.mark.parametrize('fault',['localization','armed','calibration','units','costmap','moving','terminal','cancel'])
def test_non_perception_failure_during_wait_keeps_original_stop(waiting_scan,fault):
    a,reply,now,stops,events=waiting_scan
    a.tick();assert not stops
    if fault=='localization':a.node.tf_buffer.lookup_transform=lambda *args:(_ for _ in ()).throw(RuntimeError('no tf'))
    elif fault=='armed':a.node.control_armed=False
    elif fault=='calibration':a.config['camera_calibrated']=False
    elif fault=='units':a.node.depth_units_verified=False
    elif fault=='costmap':a.costmap=None
    elif fault=='moving':a.stationary_since=None
    elif fault=='terminal':a.motions['pending']={'cancel':False}
    else:reply.cancel()
    a.tick()
    assert stops and a.scan is None


@pytest.mark.parametrize('interrupt',[None,'deadline','owner','localization','armed','cancel'])
def test_standalone_initial_scan_wait_resumes_only_while_task_valid(waiting_scan,interrupt):
    a,reply,now,stops,events=waiting_scan
    a.scan=None;owners=[]
    # Exercise both the real generator and adapter scan tick, with no ROS/API work.
    a.costmap_transform=lambda *args,**kwargs:Pose2D()
    config={'scene':{'targets':{'A':'target'}},'robot_go2':{'approach_on_detection':False}}
    steps=standalone_explore_steps(a,config,LocalFrame(),11.,'target',events.append,
        lambda:owners,lambda:True,lambda:None,clock=lambda:now[0])
    next(steps);a.tick()
    pending=a.scan['reply']
    assert not stops and not a.motions
    if interrupt=='deadline':now[0]=11.
    elif interrupt=='owner':owners.append('other')
    elif interrupt=='localization':a.node.tf_buffer.lookup_transform=lambda *args:(_ for _ in ()).throw(RuntimeError('no tf'))
    elif interrupt=='armed':a.node.control_armed=False
    elif interrupt=='cancel':steps.close()
    else:
        a.node.rgbd_sync_status['healthy']=True
        a.scan=None
        pending.set_result([Observation(target_id='A',status='DETECTED',position_valid=True,common_position=Pose2D(1.,0.))])
        with pytest.raises(StopIteration) as result:next(steps)
        assert result.value.value=='TARGET_FOUND'
        return
    if interrupt!='cancel':
        with pytest.raises(ExplorationStopped):next(steps)
    assert pending.cancelled() and not a.motions


@pytest.mark.parametrize('phase',['SPIN','FRAME','INFER'])
def test_full_scan_wait_does_not_rotate_or_replace_pending_inference(waiting_scan,phase):
    a,reply,now,stops,events=waiting_scan
    a.scan.update(current_view_only=False,phase=phase,pending=Future())
    pending=a.scan['pending']
    a._motion=lambda *args:pytest.fail('motion sent during perception wait')
    a.vision.submit=lambda *args:pytest.fail('new inference during perception wait')
    for _ in range(3):a.tick()
    assert a.scan['phase']==phase and a.scan['pending'] is pending and not stops
    assert not reply.done()


def test_image_time_tf_failure_is_waitable_but_missing_calibration_is_not(waiting_scan):
    from vlm_nav.vlm_navigator import VLMNavigator
    a,reply,now,stops,events=waiting_scan
    a.node.vlm_input_ready=lambda now:VLMNavigator.vlm_input_ready(a.node,now)
    a.node.elapsed_since=VLMNavigator.elapsed_since
    a.node.rgbd_sync_status['healthy']=True
    a.node.latest_snapshot.raw_depth_snapshot=object()
    a.node.last_camera_tf_success=6.
    a.tick()
    assert not reply.done() and not stops and a.get_health().nav_ready
    a.node.last_camera_tf_success=10.
    a.tick()
    assert a.get_health().nav_ready and not stops
    a.config['camera_calibrated']=False
    a.tick()
    assert stops and a.scan is None


@pytest.mark.parametrize('failure',[TimeoutError('API timeout'),RuntimeError('validation failed'),None])
def test_completed_inference_error_is_not_hidden_by_perception_wait(waiting_scan,failure):
    a,reply,now,stops,events=waiting_scan
    pending=Future()
    if failure is None:pending.set_result({})
    else:pending.set_exception(failure)
    a.scan.update(phase='INFER',pending=pending,snapshot=S(stamp=stamp(10.)))
    a.node.ground_pixel_with_reason=lambda *args:(None,'no depth')
    a.tick()
    assert reply.done() and isinstance(reply.exception(),RuntimeError)
    assert stops==['scan inference failed'] and a.scan is None


def test_initial_missing_camera_extrinsics_waits_until_camera_returns(waiting_scan):
    a,reply,now,stops,events=waiting_scan
    a.node.camera_extrinsic_matrix=None
    a.tick()
    assert a.get_health().nav_ready and not reply.done() and not stops
    assert a.last_health_details['checks']['rgbd_ready'] is False


def test_navigation_health_is_ready_during_camera_wait_for_cooperative_agent(waiting_scan):
    from test_local_tasks import private_mission
    mission=private_mission()
    assert mission.start().accepted
    mission.step(.1)
    a,reply,now,stops,events=waiting_scan
    machine=mission.machines['go2']
    machine.check_health(mission.now,a.get_health())
    assert not machine.paused
    assert machine.state().nav_ready


def test_disconnection_during_spin_finishes_current_action_then_waits(waiting_scan):
    a,reply,now,stops,events=waiting_scan
    pending=Future()
    a.scan.update(current_view_only=False,phase='SPIN_WAIT',pending=pending)
    a.motions={'spin':{'cancel':False}}
    a.stationary_since=None
    a.tick()
    assert not stops and not reply.done()
    pending.set_result('SUCCEEDED');a.motions.clear();a.stationary_since=now[0]-1.
    a.tick()
    assert a.scan['phase']=='FRAME' and not stops
    a.node.rgbd_sync_status['healthy']=True
    a.tick()
    assert a.scan['phase']=='FRAME' and not stops


@pytest.mark.parametrize('status',['ABORTED','CANCELED'])
def test_camera_wait_does_not_hide_completed_spin_failure(waiting_scan,status):
    a,reply,now,stops,events=waiting_scan
    pending=Future();pending.set_result(status)
    a.scan.update(current_view_only=False,phase='SPIN_WAIT',pending=pending)
    a.tick()
    assert reply.done() and isinstance(reply.exception(),RuntimeError)
    assert a.scan is None
