"""Successful task results survive later health changes, without allowing new motion."""
from dataclasses import replace
import pytest
from multi_agent_nav.mock import MockMission, mock_config


def mission_to_done(deliver=True):
    config = mock_config()
    config['scene'].update(share_c_coordinates=False, resources={}, exit_points={})
    mission = MockMission(config)
    assert mission.start().accepted
    for _ in range(1000):
        mission.step()
        if mission.coordinator.phase == 'DONE':
            break
    assert mission.coordinator.phase == 'DONE'
    if deliver:mission.step()  # Deliver the first SYNC with phase DONE.
    return mission


def assert_completed(machine):
    state = machine.state()
    assert (state.run_state, state.reply, state.ready, state.deposited) == ('IDLE', 'SUCCEEDED', True, True)
    assert not machine.paused


def test_completed_task_survives_total_timeout_with_live_peers():
    m = mission_to_done()
    while m.now < 602:
        m.step()
    assert m.coordinator.phase == 'DONE'
    for a in m.machines.values():
        assert_completed(a)
    assert not m.coordinator.commands


@pytest.mark.parametrize('fault', ['nav_ready', 'localization_valid', 'stopped', 'motion_terminal'])
def test_completed_health_failure_is_reported_separately_and_recovers(fault):
    m = mission_to_done();a = m.machines['go2']
    healthy = m.adapters['go2'].get_health()
    a.tick(m.now, replace(healthy, **{fault:False}))
    assert_completed(a)
    assert 'post-completion health' in a.state().reason
    if fault in ('nav_ready','localization_valid','stopped'):
        assert getattr(a.state(), fault) is False
    a.tick(m.now, healthy)
    assert_completed(a)
    assert a.state().reason == ''


def test_completed_peer_loss_or_pause_does_not_revoke_success():
    m = mission_to_done();a = m.machines['go2']
    a.receive_peer(replace(m.machines['ranger'].state(),run_state='PAUSED',reason='heartbeat timeout'),m.now)
    a.tick(m.now,m.adapters['go2'].get_health())
    assert_completed(a)
    assert 'peer' in a.state().reason
    a.tick(m.now+10,m.adapters['go2'].get_health())
    assert_completed(a)
    assert 'heartbeat' in a.state().reason


@pytest.mark.parametrize('fault', ['deposited', 'stopped', 'motion_terminal', 'goal_id', 'local_motion', 'reply'])
def test_done_cannot_complete_unfinished_local_work(fault):
    m = mission_to_done(deliver=False);a = m.machines['go2']
    if fault in ('deposited','goal_id','reply'):
        setattr(a._state,fault,{'deposited':False,'goal_id':'active','reply':'EXECUTING'}[fault])
    elif fault=='local_motion':a.local_motion={'pending':True}
    else:a.health=replace(a.health,**{fault:False})
    command=m.coordinator.control('go2','SYNC')
    assert not a.accept(command,m.now).accepted


def test_done_is_not_inferred_from_local_c_success():
    m = mission_to_done();a = m.machines['go2']
    assert m.start().accepted;m.step()
    a._state.ready=a._state.deposited=True
    a._state.run_state='IDLE';a._state.reply='SUCCEEDED'
    a.committed=True
    a.tick(m.now+10,m.adapters['go2'].get_health())
    assert a.paused


def test_completed_rejects_old_commands_and_wrong_task_done_but_allows_new_handshake():
    m = mission_to_done();a=m.machines['go2']
    old_task=a.context.task_id
    old_command=next(c for _,c in reversed(m.executed) if c.recipient_id=='go2')
    assert not a.accept(old_command,m.now).accepted
    sync=m.coordinator.control('go2','SYNC')
    assert a.accept(sync,m.now).accepted
    assert not a.accept(replace(sync,phase='DEPOSIT'),m.now).accepted
    assert not a.accept(replace(sync,task_id='foreign'),m.now).accepted
    assert_completed(a)
    assert m.start().accepted;m.step()
    assert a.context.task_id != old_task
    assert not a.state().deposited
    assert not a.accept(sync,m.now).accepted


def test_explicit_stop_preserves_done_and_still_requests_physical_stop():
    m=mission_to_done();a=m.machines['go2']
    commands=m.coordinator.pause('shutdown')
    assert m.coordinator.phase=='DONE'
    for command in commands:
        m.machines[command.recipient_id].accept(command,m.now)
    assert_completed(a)
    assert any(e.kind=='STOP' for e in a.take_effects())


def test_late_results_cannot_rewrite_completed_state_or_schedule_motion():
    from multi_agent_nav.models import ApproachResult, GoalResult
    m=mission_to_done();a=m.machines['go2']
    state=a.state()
    a.on_approach(ApproachResult(None,'late failure'),m.now)
    a.on_goal_result(GoalResult(state.task_id,state.command_id,'old-goal','DEPOSIT_NAV','ABORTED'),m.now)
    a.fail('late failure')
    assert_completed(a)
    assert not a.take_effects()


def test_completed_stop_dispatches_to_adapter_without_restarting_execution(monkeypatch):
    m=mission_to_done();before=len(m.adapters['go2'].calls)
    stopped=[];adapter=m.adapters['go2'];stop=adapter.stop_motion
    def record_stop(reason):
        stopped.append(reason)
        return stop(reason)
    monkeypatch.setattr(adapter,'stop_motion',record_stop)
    m.pause('operator shutdown');m.run(5)
    assert m.coordinator.phase=='DONE'
    assert_completed(m.machines['go2'])
    assert not m.adapters['go2'].active
    assert len(m.adapters['go2'].calls)==before
    assert stopped


def test_completed_task_cannot_start_new_task_with_unhealthy_local_inputs():
    m=mission_to_done();a=m.machines['go2'];task=a.context.task_id
    a.tick(m.now,replace(a.health,nav_ready=False))
    sync=replace(m.coordinator.control('go2','SYNC'),task_id='next-task',phase='SYNCING')
    assert not a.accept(sync,m.now).accepted
    assert a.context.task_id==task
    assert_completed(a)
