"""All signals are captured by fakes; never signal a system PID."""
import signal

from test_go2_agent_session import finish, make_session


def test_cancel_once_retains_original_result(tmp_path):
    signals=[]
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:signals.append(a))
    s.start(s.target)
    assert s.cancel(s.task_id)['state']=='CANCEL_REQUESTED'
    s.cancel(s.task_id)
    finish(s,p)
    snap=s.close()
    assert snap['state']=='CANCELED'
    assert snap['result']=='TARGET_APPROACHED'
    assert snap['cancel_requested_at']<=snap['finished_at']
    assert signals==[(p.pid,signal.SIGINT)]
    assert s.cancel(s.task_id)==snap


def test_completed_success_wins_late_cancel(tmp_path):
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:(_ for _ in ()).throw(AssertionError('signal')))
    s.start(s.target);finish(s,p)
    assert s.cancel(s.task_id)['state']=='SUCCEEDED'
    assert s.close()['cancel_requested_at'] is None


def test_close_waits_for_original_sigint_report(tmp_path):
    signals=[];clock=[0.]
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:signals.append(a),clock=lambda:clock[0],
                         sleep=lambda dt:advance(dt))
    def advance(dt):
        clock[0]+=dt
        if clock[0]>=.3:finish(s,p,code=1,result='operator interrupt')
    s.start(s.target)
    result=s.close()
    assert result['state']=='CANCELED' and result['stop_confirmed'] is True
    assert result['active_process'] is False
    assert signals==[(p.pid,signal.SIGINT)]


def test_ignored_interrupt_times_out_without_escalation(tmp_path):
    signals=[];clock=[0.]
    def advance(dt):clock[0]+=dt
    s,p,calls,_=make_session(tmp_path,killpg=lambda *a:signals.append(a),clock=lambda:clock[0],sleep=advance)
    s.start(s.target)
    original_lock=tmp_path/'original-motion.lock';original_lock.write_text('held')
    result=s.close()
    assert result['state']=='STOP_UNCONFIRMED' and result['active_process'] is True
    assert clock[0]==30 and signals==[(p.pid,signal.SIGINT)]
    assert original_lock.read_text()=='held'
    s.start(s.target)
    assert len(calls)==1


def test_identity_mismatch_never_signals(tmp_path):
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:(_ for _ in ()).throw(AssertionError('signal')))
    s.start(s.target);s._identity=lambda pid:(pid,'reused')
    assert s.cancel(s.task_id)['state']=='STOP_UNCONFIRMED'
    assert s.status(s.task_id)['active_process'] is True


def test_false_stop_after_cancel_remains_unconfirmed(tmp_path):
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:None)
    s.start(s.target);s.cancel(s.task_id);finish(s,p,stop_confirmed=False)
    assert s.close()['state']=='STOP_UNCONFIRMED'


def test_close_prevents_late_tool_start(tmp_path):
    s,_,calls,_=make_session(tmp_path)
    assert s.close()['framework_error']=='AGENT_NO_EXECUTION'
    assert s.start(s.target)['state']=='FAILED'
    assert not calls


def test_unreadable_events_cannot_prevent_interrupt(tmp_path):
    signals=[]
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:signals.append(a))
    s.start(s.target);s.output_dir.mkdir();(s.output_dir/'events.jsonl').mkdir()
    s.cancel(s.task_id)
    assert signals==[(p.pid,signal.SIGINT)]


def test_persistence_failure_after_spawn_cannot_prevent_interrupt(tmp_path,monkeypatch):
    import tools.go2_agent.session as module
    signals=[]
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:signals.append(a))
    s.start(s.target)
    def fail(*a):raise OSError('disk full')
    monkeypatch.setattr(module,'write_json',fail)
    s.cancel(s.task_id)
    assert signals==[(p.pid,signal.SIGINT)]


def test_cannot_spawn_without_durable_consumption(tmp_path,monkeypatch):
    import tools.go2_agent.session as module
    s,_,calls,_=make_session(tmp_path)
    def fail(*a):raise OSError('disk full')
    monkeypatch.setattr(module,'write_json',fail)
    import pytest
    with pytest.raises(OSError):s.start(s.target)
    assert not calls


def test_cancel_budget_is_not_restarted_by_close_or_status(tmp_path):
    clock=[0.];signals=[]
    def advance(dt):clock[0]+=dt
    s,p,_,_=make_session(tmp_path,killpg=lambda *a:signals.append(a),clock=lambda:clock[0],sleep=advance)
    s.start(s.target);s.cancel(s.task_id)
    clock[0]=31.
    assert s.status(s.task_id)['state']=='STOP_UNCONFIRMED'
    assert s.close()['state']=='STOP_UNCONFIRMED'
    assert clock[0]==31.
    assert s.close()['state']=='STOP_UNCONFIRMED'
    assert clock[0]==31. and signals==[(p.pid,signal.SIGINT)]
