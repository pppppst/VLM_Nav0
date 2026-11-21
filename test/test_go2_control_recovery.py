from concurrent.futures import Future
from types import SimpleNamespace as NS

from vlm_nav.go2_control_recovery import ControlRecovery


class Service:
    class Request:
        PREPARE, RESUME, CANCEL = 1, 2, 3
        def __init__(self, **values): self.__dict__.update(values)


def fixture():
    calls = []
    def call(request):
        result = Future()
        calls.append((request, result))
        return result
    client = NS(service_is_ready=lambda: True, call_async=call)
    node = NS(create_client=lambda *a: client, create_subscription=lambda *a: None)
    now = [1.]
    recovery = ControlRecovery(node, service_type=Service, clock=lambda: now[0])
    return recovery, calls, now


def snapshot(recovery, epoch=4, pending=False, allowed=True, instance='bridge-one'):
    fields = dict(instance_id=instance, epoch=str(epoch), state='DISARMED' if pending else 'FAULT',
                  fault_code='SYSTEM_READY_LOST', recovery_pending=str(pending).lower(),
                  recovery_allowed=str(allowed).lower())
    recovery.observe(NS(status=[NS(name='go2_bridge', values=[NS(key=k,value=v) for k,v in fields.items()])]))


def reply(calls, epoch=4, accepted=True):
    calls[-1][1].set_result(NS(accepted=accepted,instance_id='bridge-one',epoch=epoch,
                             state='DISARMED',reason='waiting'))


def test_prepare_then_wait_for_existing_stop_and_health_conditions():
    recovery,calls,_ = fixture()
    snapshot(recovery)
    recovery.step(False)
    assert calls[-1][0].phase == Service.Request.PREPARE
    reply(calls)
    snapshot(recovery,pending=True)
    recovery.step(False)
    assert len(calls)==1
    recovery.step(True)
    assert calls[-1][0].phase == Service.Request.RESUME
    assert calls[-1][0].expected_epoch==4


def test_cancel_in_flight_preserves_request_epoch_and_never_resumes():
    recovery,calls,_=fixture()
    snapshot(recovery)
    recovery.step(True)
    recovery.cancel()
    assert len(calls)==2
    calls[0][1].set_result(NS(accepted=False,instance_id='bridge-one',epoch=5,state='DISARMED',reason='changed'))
    assert calls[-1][0].phase==Service.Request.CANCEL
    assert calls[-1][0].expected_epoch==4
    recovery.step(True)
    assert len(calls)==2


def test_stale_diagnostics_and_hard_fault_do_not_send_recovery():
    recovery,calls,now=fixture()
    snapshot(recovery)
    now[0]=10.
    recovery.step(True)
    assert not calls
    snapshot(recovery,allowed=False)
    recovery.step(True)
    assert not calls


def test_bridge_restart_cannot_reuse_control_authority():
    recovery,calls,_=fixture()
    snapshot(recovery)
    snapshot(recovery,instance='bridge-two')
    recovery.step(True)
    assert not calls and recovery.reason=='bridge restarted; explicit ARM required'


def test_new_client_does_not_adopt_pending_recovery_from_previous_owner():
    recovery,calls,_=fixture()
    snapshot(recovery,pending=True)
    recovery.step(True)
    assert not calls


def test_manual_control_epoch_revokes_pending_resume():
    recovery,calls,_=fixture()
    snapshot(recovery)
    recovery.step(True)
    reply(calls)
    snapshot(recovery,epoch=6,pending=False,allowed=False)
    recovery.step(True)
    assert len(calls)==1


def test_lost_prepare_response_reconciles_only_own_pending_epoch():
    recovery,calls,now=fixture()
    snapshot(recovery)
    recovery.step(False)
    now[0]+=3.
    snapshot(recovery,pending=True)
    recovery.step(True)
    assert calls[-1][0].phase==Service.Request.RESUME
    assert len(calls)==2


def test_cancel_does_not_adopt_new_manual_arm_epoch():
    recovery,calls,_=fixture()
    snapshot(recovery)
    recovery.step(False)
    recovery.cancel()
    snapshot(recovery,epoch=8,allowed=False)
    calls[0][1].set_result(NS(accepted=False,instance_id='bridge-one',epoch=8,state='ARMED',reason='manual'))
    assert calls[-1][0].phase==Service.Request.CANCEL
    assert calls[-1][0].expected_epoch==4


def test_cancel_has_bounded_wait_for_lost_rpc():
    recovery,calls,now=fixture()
    snapshot(recovery)
    recovery.step(False)
    recovery.cancel()
    now[0]+=3.
    recovery.poll()
    assert len(calls)==3 and calls[-1][0].phase==Service.Request.CANCEL
    assert calls[-1][0].expected_epoch==4


def test_cancel_is_sent_before_resume_reply_and_keeps_source_epoch():
    recovery,calls,_=fixture()
    snapshot(recovery)
    recovery.step(False)
    reply(calls)
    snapshot(recovery,pending=True)
    recovery.step(True)
    resume=calls[-1][1]
    recovery.cancel()
    assert calls[-1][0].phase==Service.Request.CANCEL
    assert calls[-1][0].expected_epoch==4
    resume.set_result(NS(accepted=True,instance_id='bridge-one',epoch=5,state='ARMED',reason='armed'))
    reply(calls,epoch=6)
    assert recovery.finish_cancel(lambda:None,1.)
    assert len(calls)==3


def test_new_fault_seen_before_cancel_revokes_that_fault_epoch():
    recovery,calls,_=fixture()
    snapshot(recovery)
    recovery.step(False)
    reply(calls)
    snapshot(recovery,pending=True)
    recovery.step(True)
    reply(calls,epoch=5)
    snapshot(recovery,epoch=6)
    recovery.cancel()
    assert calls[-1][0].expected_epoch==6


def test_cancel_without_handshake_revokes_observed_epoch_only_once():
    recovery,calls,_=fixture()
    snapshot(recovery,allowed=False)
    recovery.cancel()
    assert calls[-1][0].phase==Service.Request.CANCEL
    assert calls[-1][0].expected_epoch==4
    calls[-1][1].set_result(NS(accepted=True,instance_id='bridge-one',epoch=5,state='ARMED',reason='revoked'))
    snapshot(recovery,epoch=8,allowed=False)
    recovery.cancel()
    assert len(calls)==1


def test_hard_bridge_fault_stays_latched_after_manual_rearm():
    recovery,calls,_=fixture()
    snapshot(recovery)
    fields=dict(recovery.snapshot,fault_code='FOREIGN_CONTROL',recovery_allowed='false')
    recovery.observe(NS(status=[NS(name='go2_bridge',values=[NS(key=k,value=v) for k,v in fields.items()])]))
    assert recovery.hard_fault=='bridge hard fault: FOREIGN_CONTROL'
    snapshot(recovery,epoch=8,allowed=True)
    recovery.step(True)
    assert not calls and recovery.closed


def test_finish_cancel_reports_unconfirmed_after_finite_rpc_wait():
    recovery,calls,now=fixture()
    snapshot(recovery)
    recovery.step(False)
    assert not recovery.finish_cancel(lambda:now.__setitem__(0,now[0]+1),3.)
    assert recovery.closed and recovery.cancel_epoch==4
    assert all(request.expected_epoch==4 for request,_ in calls)
