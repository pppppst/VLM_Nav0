from dataclasses import replace
from test_agent import machine,command,navigate
from test_coordinator import coord
from multi_agent_nav.models import Health,Command,Pose2D,GoalResult,ResourceRequest
from multi_agent_nav.mock import MockMission

def test_new_task_cannot_discard_uncertain_old_resource():
    agent=machine()
    agent._state.token='held';agent._state.resource_id='basket'
    agent._state.run_state='PAUSED';agent._state.stopped=True
    sync=replace(command(),action='SYNC',task_id='new')
    assert not agent.accept(sync,1).accepted
    c=coord();c.pause('restart')
    c.states['go2'].token='held'
    assert not c.start('test',1).accepted

def test_exit_outside_goal_cannot_claim_release_without_entry():
    m=MockMission();m.start()
    for _ in range(100):
        m.step()
        if m.machines['go2'].state().deposited:break
    state=m.machines['go2'].state()
    assert state.token and state.resource_status=='ENTERED'
    m.coordinator.resources.freeze('go2','loss')
    m.run(80)
    assert 'basket' in m.coordinator.resources.held
    assert not m.machines['go2'].state().exited

def test_cancel_ack_without_action_terminal_is_not_stopped():
    agent=machine();navigate(agent,command())
    agent.accept(replace(command(),action='CANCEL',cancel_command_id='c1'),1)
    agent.tick(1,Health(localization_valid=True,nav_ready=True,stopped=True,motion_terminal=False))
    assert agent.state().run_state=='STOPPING'
    assert not agent.accept(command(sequence=2),1).accepted

def test_missing_footprint_cannot_confirm_resource_exit():
    m=MockMission();m.start()
    for _ in range(100):
        m.step()
        if m.machines['go2'].state().deposited:break
    agent=m.machines['go2']
    agent._state.run_state='EXIT_STOP'
    health=m.adapters['go2'].get_health()
    health.footprint=[];health.pose=Pose2D(99,99)
    agent._monitor_region(health)
    assert agent.state().resource_status!='EXIT_CONFIRMED'
    assert agent.state().token
