import time
from types import SimpleNamespace
import numpy as np
import pytest
from multi_agent_nav.adapters.go2 import ABCClient, observations_from_results, AsyncVision, Go2Adapter
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.models import Pose2D
from vlm_nav.models import VLMResult, Pixel

def grounding(visible):
    return dict(target_visible=visible, object_match=visible, qualifier_match=visible,
                relation_match=visible, confidence=.9,
                target_pixel={'u':2,'v':2} if visible else None,
                evidence_pixel={'u':2,'v':2} if visible else None)

def test_abc_structure_and_invalid_outputs():
    payload={t:dict(status='DETECTED' if t=='A' else 'NOT_DETECTED', grounding=grounding(t=='A')) for t in 'ABC'}
    results=ABCClient.parse(payload,5,5)
    assert results['A'][0]=='DETECTED' and results['B'][0]=='NOT_DETECTED'
    with pytest.raises(ValueError): ABCClient.parse({'A':payload['A']},5,5)
    payload['C']['grounding']['confidence']=float('nan')
    with pytest.raises(ValueError): ABCClient.parse(payload,5,5)

def test_detected_without_depth_is_not_navigation_position():
    frame=FrameAdapter('g','v','lab_ref','a',(0,0,0))
    result=VLMResult(True,.9,Pixel(2,2),None,True,True,True)
    snapshot=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0))
    outputs=observations_from_results({'A':('DETECTED',result),'B':('NOT_DETECTED',None),'C':('UNCERTAIN',None)},
        snapshot,lambda *args:(None,'invalid depth'),frame,'scan',0)
    assert len(outputs)==3 and outputs[0].status=='DETECTED' and not outputs[0].position_valid
    outputs=observations_from_results({'A':('DETECTED',result)},snapshot,lambda *args:((2,3,1),'ok'),frame,'scan',0)
    assert outputs[0].common_position==Pose2D(2,3) and outputs[0].position_valid

def test_async_vision_timeout_does_not_block_and_drops_canceled_result():
    worker=AsyncVision(.02)
    def slow():
        time.sleep(.08)
        return 'late'
    started=time.monotonic()
    future=worker.submit(slow)
    assert time.monotonic()-started<.02
    time.sleep(.03)
    worker.poll()
    assert future.done() and isinstance(future.exception(),TimeoutError)
    assert not worker.available  # one in-flight job; cannot accumulate timed-out work
    time.sleep(.08)
    worker.poll()
    assert worker.available
    future=worker.submit(slow)
    worker.cancel()
    assert future.cancelled()
    worker.close()

def test_motion_pending_cancel_then_late_handle_is_canceled_and_identity_retained():
    from concurrent.futures import Future
    from collections import deque
    adapter=Go2Adapter.__new__(Go2Adapter)
    adapter.motions={};adapter.results=deque();adapter.motion_fault=False
    sent=Future();terminal=Future()
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:terminal,
        goal_id=SimpleNamespace(uuid=[1]*16))
    canceled=[]
    handle.cancel_goal_async=lambda:canceled.append(True)
    client=SimpleNamespace(send_goal_async=lambda _:sent)
    goal_id,reply=adapter._motion(client,object(),'task','command','PICKUP_NAV')
    adapter.cancel_navigation(goal_id)
    assert adapter.motions and not reply.done()
    sent.set_result(handle)
    assert canceled
    assert adapter.last_action_uuid=='01'*16
    terminal.set_result(SimpleNamespace(status=5))
    assert not adapter.motions and reply.result()=='CANCELED'
    result=adapter.results.pop()
    assert (result.task_id,result.command_id,result.goal_id,result.goal_kind)==('task','command',goal_id,'PICKUP_NAV')

@pytest.mark.parametrize('remapped', [False, True])
@pytest.mark.parametrize('node_name', [None, 'multi_agent_go2_check_readonly'])
def test_observation_only_node_cannot_enable_old_autonomy(monkeypatch,tmp_path,remapped,node_name):
    import rclpy
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    monkeypatch.setenv('ROS_DOMAIN_ID','31');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    rclpy.init(args=['--ros-args','-r','__node:=multi_agent_go2_agent'] if remapped else [])
    agent=rclpy.create_node('multi_agent_task_agent')
    node=VLMNavigator(observation_only=True,node_name=node_name)
    try:
        assert agent.get_name()==('multi_agent_go2_agent' if remapped else 'multi_agent_task_agent')
        assert node.get_name()==(node_name or 'multi_agent_go2_rgbd')
        assert node.get_name()!=agent.get_name()
        assert len(list(node.timers))==1 and node.worker is None
        assert not node.set_parameters([Parameter('enabled',value=True)])[0].successful
    finally:
        node.destroy_node();agent.destroy_node();rclpy.shutdown()

def test_delayed_odometry_cannot_confirm_physical_stop():
    adapter=Go2Adapter.__new__(Go2Adapter)
    adapter.config={'odom_max_age':.5,'stationary_speed':.02}
    adapter.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    adapter.stationary_since=1;adapter.odom_at=1
    v=SimpleNamespace(linear=SimpleNamespace(x=0,y=0,z=0),angular=SimpleNamespace(z=0))
    message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),twist=SimpleNamespace(twist=v))
    adapter.on_odometry(message)
    assert adapter.stationary_since is None and adapter.odom_at==0

@pytest.mark.parametrize('failure', ['timeout', 'unsettled', 'none'])
def test_cooperative_native_send_rechecks_after_motion_preparation(failure):
    from concurrent.futures import Future
    from dataclasses import replace
    from multi_agent_nav.adapters.fake import FakeAdapter
    from multi_agent_nav.runner import AgentRunner
    from test_agent import machine, command
    agent, adapter, now, sent = machine(), FakeAdapter('go2'), [0.], []
    agent.tick(0., adapter.get_health())
    runner = AgentRunner(agent, adapter, {}, lambda observation: None, clock=lambda: now[0])
    assert agent.accept(command('WAIT'), 0.).accepted
    adapter.motions = {}
    get_health = adapter.get_health
    adapter.get_health = lambda: replace(get_health(), motion_terminal=not adapter.motions)
    def prepare(*args):
        if failure == 'timeout':now[0] = 3.01
        elif failure == 'unsettled':adapter.get_health = lambda: replace(get_health(), stopped=False)
    adapter._motion_event = prepare
    response = Future()
    client = SimpleNamespace(send_goal_async=lambda goal: sent.append(goal) or response)
    result = Go2Adapter._motion(adapter, client, object(), '', 'c1', 'SCAN_SPIN')
    if failure == 'none':
        assert result[0] in adapter.motions and len(sent) == 1
    else:
        assert result == (None, None) and not sent and not adapter.motions
    assert agent.paused == (failure == 'timeout')
