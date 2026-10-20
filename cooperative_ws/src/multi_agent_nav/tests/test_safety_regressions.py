from dataclasses import replace
from test_agent import machine,command,navigate
from test_coordinator import coord
from multi_agent_nav.models import Health,Command,Pose2D,GoalResult,ResourceRequest
from multi_agent_nav.mock import MockMission

def test_reserved_resource_fields_do_not_block_new_task():
    agent=machine()
    agent._state.token='held';agent._state.resource_id='basket'
    agent._state.run_state='PAUSED';agent._state.stopped=True
    sync=replace(command(),action='SYNC',task_id='new')
    assert agent.accept(sync,1).accepted
    c=coord();c.pause('restart')
    c.states['go2'].token='held'
    assert c.start('test',1).accepted

def test_completion_does_not_fabricate_an_exit_or_resource_release():
    m=MockMission();m.start();m.run(100)
    assert m.coordinator.phase=='DONE'
    assert all(s.deposited and not s.exited and not s.token for s in m.coordinator.states.values())
    assert not any(c.kind=='EXIT' for _,c in m.executed)


def test_cancel_ack_without_action_terminal_is_not_stopped():
    agent=machine();navigate(agent,command())
    agent.accept(replace(command(),action='CANCEL',cancel_command_id='c1'),1)
    agent.tick(1,Health(localization_valid=True,nav_ready=True,stopped=True,motion_terminal=False))
    assert agent.state().run_state=='STOPPING'
    assert not agent.accept(command(sequence=2),1).accepted

def test_missing_footprint_cannot_start_navigation():
    agent=machine();agent.health.footprint=[]
    navigate(agent,command())
    assert agent.paused and agent.state().reason=='invalid footprint'


def test_retired_task_sync_cannot_revive_canceled_execution():
    agent=machine()
    old=command()
    assert agent.accept(old,0).accepted
    agent.accept(replace(old,action='CANCEL',cancel_command_id=old.command_id),0)
    agent.tick(0,Health(localization_valid=True,nav_ready=True,stopped=True))
    new=replace(old,action='SYNC',task_id='new')
    assert agent.accept(new,0).accepted
    agent.accept(replace(new,action='PAUSE'),0)
    agent.tick(0,Health(localization_valid=True,nav_ready=True,stopped=True))
    agent.take_effects()
    assert not agent.accept(replace(old,action='SYNC'),0).accepted
    assert not agent.accept(old,0).accepted
    assert not agent.take_effects()

def test_executing_path_startup_waits_only_while_stopped_and_is_bounded():
    def started():
        agent=machine()
        agent.tick(0,replace(health,motion_terminal=True))
        navigate(agent,command())
        assert not agent.paused
        return agent
    health=Health(localization_valid=True,nav_ready=True,stopped=True,
        footprint=[[-.38,-.18],[.38,-.18],[.38,.18],[-.38,.18]],motion_terminal=False)
    agent=started()
    agent.tick(.1,health)
    assert agent.state().run_state=='NAVIGATING'
    agent.tick(.2,replace(health,path=[Pose2D(),Pose2D(2,0)],path_valid=True,stopped=False))
    assert agent.state().run_state=='NAVIGATING'
    agent.tick(.3,health)  # No grace after the first executing path.
    assert agent.paused
    agent=started()
    agent.tick(.1,replace(health,stopped=False))
    assert agent.paused  # Movement before a valid executing path fails closed.
    agent=started()
    agent.tick(1.1,health)
    assert agent.paused  # A planner that never publishes cannot wait forever.
