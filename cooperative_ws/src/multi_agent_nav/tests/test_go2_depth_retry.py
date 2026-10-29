"""Same-heading main-target depth retries use new image results and bounded evidence."""
from concurrent.futures import Future
from dataclasses import replace
import json
import math
from types import SimpleNamespace as S

import numpy as np
import pytest

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.frame_adapter import LocalFrame
from multi_agent_nav.models import Pose2D
from vlm_nav.models import VLMResult, Pixel
from test_go2_health_diagnostics import healthy_adapter


@pytest.fixture
def scanning(monkeypatch,tmp_path):
    now=[10.]
    monkeypatch.setattr('multi_agent_nav.adapters.go2.time.monotonic',lambda:now[0])
    a=healthy_adapter();a.scan=None;a.plans={};a.frame=LocalFrame()
    a.config.update(scan_timeout=20.,api_timeout=1.,scan_angles=[i*math.pi/4 for i in range(8)])
    a.scan_evidence_dir=tmp_path/'vlm';a.last_motion=('',{})
    a.costmap_transform=lambda *args,**kwargs:Pose2D()
    a.node.stop_pub=S(publish=lambda _:None)
    a.node.spinner=S()
    events=[];calls=[];motions=[];answers=[];projections=[]
    a.record_event=events.append
    a.get_health=lambda:replace(healthy,pose=Pose2D(0.,0.,yaw[0]))
    healthy=healthy_adapter().get_health();yaw=[0.]
    def motion(server,goal,*args):
        motions.append(goal.target_yaw);yaw[0]+=goal.target_yaw
        return 'spin',completed('SUCCEEDED')
    a._motion=motion
    def infer(rgb,targets):
        calls.append(int(rgb[0,0,0]));answer=answers.pop(0)
        if isinstance(answer,Exception):raise answer
        return {t:(status,VLMResult(True,True,True,True,.9,Pixel(len(calls),2),None) if status=='DETECTED' else VLMResult(False,False,False,False,0.,None,None))
            for t,status in answer.items()}
    a.client=S(infer_abc=infer)
    def submit(function):
        try:return completed(function())
        except Exception as error:
            future=Future();future.set_exception(error);return future
    a.vision=S(poll=lambda:None,available=True,submit=submit,cancel=lambda:None)
    def frame(status='DETECTED',located=False,other=False,repeat=False):
        now[0]+=.1
        if not repeat:
            a.node.latest_snapshot=S(stamp=S(sec=10,nanosec=round((now[0]-10.)*1e9)),
                captured_monotonic=now[0],frame_id='camera',sequence=len(calls)+1,
                rgb=np.full((4,4,3),len(calls)+1,dtype=np.uint8))
            answers.append({'A':'NOT_DETECTED' if other else status,'B':status if other else 'NOT_DETECTED'})
        def project(snapshot,pixel,*args):
            projections.append((snapshot.sequence,(pixel.u,pixel.v)))
            return ((float(snapshot.sequence),2.,0.) if located else None,'ok' if located else 'no depth')
        a.node.ground_pixel_with_reason=project
        a.tick();a.tick()
    return S(a=a,now=now,events=events,calls=calls,motions=motions,answers=answers,
        frame=frame,projections=projections,tmp=tmp_path)


@pytest.mark.parametrize('current',[True,False])
@pytest.mark.parametrize('success_at',[2,3,None])
def test_main_depth_retry_uses_fresh_results_without_extra_views(scanning,current,success_at):
    s=scanning
    reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=current)
    if not current:s.a.tick()
    deadline=s.a.scan['deadline']
    for attempt in range(1,(success_at or 3)+1):
        s.frame(located=attempt==success_at)
        if attempt<(success_at or 3):
            assert not reply.done() and not s.motions and s.a.scan['index']==0
            s.frame(repeat=True)
            assert len(s.calls)==attempt
            assert s.a.scan['deadline']==deadline
    if success_at or current:
        s.a.tick()
        assert reply.done()
        result=reply.result()
    else:
        assert s.a.scan['index']==1
        result=s.a.scan['observations']
    assert len(result)==2 and {(o.target_id,o.view_index) for o in result}=={('A',0),('B',0)}
    assert result[0].status=='DETECTED' and result[0].navigable==bool(success_at)
    assert len(s.calls)==(success_at or 3)
    assert s.projections==[(i,(i,2)) for i in range(1,(success_at or 3)+1)]
    records=[e for e in s.events if e.get('event')=='VLM_VIEW_RESULT']
    assert [e['attempt'] for e in records]==list(range(success_at or 3))
    metadata=[json.loads(p.read_text()) for p in s.tmp.glob('vlm/*/metadata.json')]
    assert sorted(m['attempt'] for m in metadata)==list(range(success_at or 3))


@pytest.mark.parametrize('later',['NOT_DETECTED','UNCERTAIN'])
def test_retry_exhaustion_preserves_detected_evidence_and_full_eight_directions(scanning,later):
    s=scanning;reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A')
    s.a.tick();s.frame();s.frame(status=later);s.frame(status=later)
    for view in range(1,8):
        s.a.tick();s.a.tick()
        s.frame(status='NOT_DETECTED')
    s.a.tick()
    result=reply.result()
    assert len(result)==16 and len(s.calls)==10 and len(s.motions)==7
    assert {(o.target_id,o.view_index) for o in result}=={(t,i) for t in ('A','B') for i in range(8)}
    assert result[0].status=='DETECTED' and not result[0].navigable


def test_other_target_without_depth_never_starts_retry(scanning):
    s=scanning;reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=True)
    s.frame(other=True);s.a.tick()
    assert reply.done() and len(s.calls)==1


def test_cancel_during_retry_discards_late_response(scanning):
    s=scanning;reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=True)
    s.frame();assert not reply.done()
    pending=Future();s.a.vision.submit=lambda _:pending
    s.frame(located=True)
    reply.cancel();s.a.tick()
    pending.set_result({'A':('DETECTED',S(confidence=.9,target_pixel=(1,2)))})
    s.a.tick()
    assert s.a.scan is None and reply.cancelled() and not s.motions


@pytest.mark.parametrize('failure',[TimeoutError('API timeout'),ValueError('invalid response')])
def test_api_failure_during_retry_keeps_original_error_branch(scanning,failure):
    s=scanning;reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=True)
    s.frame();assert not reply.done()
    s.answers.append(failure)
    s.frame()
    assert isinstance(reply.exception(),RuntimeError) and s.a.scan is None and len(s.calls)==2
    assert not s.motions


def test_new_receipt_with_same_image_stamp_is_not_another_attempt(scanning):
    s=scanning;reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=True)
    s.frame()
    s.a.node.latest_snapshot.captured_monotonic+=.01
    s.a.tick()
    assert len(s.calls)==1 and not reply.done() and s.a.scan['attempt']==1


@pytest.mark.parametrize('limit',[0,1])
def test_configured_extra_frame_count_is_used_once(scanning,limit):
    s=scanning;s.a.config['target_depth_retries']=limit
    reply=s.a.start_scan({'A':'target','B':'other'},'scan',stop_on_target='A',current_view_only=True)
    for _ in range(limit+1):s.frame()
    s.a.tick()
    assert reply.done() and len(s.calls)==limit+1 and reply.result()[0].status=='DETECTED'


@pytest.mark.parametrize('limit',[-1,True,1.5])
def test_bad_retry_config_is_rejected_before_starting_scan(scanning,limit):
    s=scanning;s.a.config['target_depth_retries']=limit
    with pytest.raises(ValueError,match='target_depth_retries'):
        s.a.start_scan({'A':'target'},'scan',stop_on_target='A')
    assert not s.a.scan and not s.motions and not s.calls
