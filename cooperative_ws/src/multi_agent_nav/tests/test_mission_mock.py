from dataclasses import replace
from multi_agent_nav.mock import MockMission, mock_config

def test_two_maps_shared_a_ready_barrier_and_c_without_permits():
    mission=MockMission()
    assert mission.start().accepted
    mission.run(100)
    assert mission.coordinator.phase=='DONE', mission.coordinator.reason
    states=mission.coordinator.states
    assert all(s.ready and s.deposited and s.stopped for s in states.values())
    calls=mission.executed
    deposits=[(robot,cmd) for robot,cmd in calls if cmd.kind=='APPROACH_DEPOSIT']
    assert [r for r,c in deposits]==['go2','ranger']
    assert deposits[0][1].observation_id==deposits[1][1].observation_id
    assert all(not c.token and not c.resource_id for _,c in deposits)
    assert not any(c.kind=='EXIT' for _,c in calls)
    go2_pickup=next(c for r,c in calls if r=='go2' and c.kind=='APPROACH_PICKUP')
    assert '/ranger/' in go2_pickup.observation_id  # source clue from peer
    assert mission.frames['ranger'].yaw != 0
    assert not any(s.token for s in mission.coordinator.states.values())

def test_pause_late_success_cannot_cross_ready_barrier():
    m=MockMission()
    m.adapters['go2'].delay=5
    m.start();m.run(10)
    m.pause('test')
    m.run(100)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['go2'].state().ready
    assert not any(c.kind=='APPROACH_DEPOSIT' for r,c in m.executed)

def test_vlm_timeout_retries_once_and_no_false_not_found():
    m=MockMission();m.adapters['ranger'].api_error=True
    m.start();m.run(100)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['ranger'].state().ready
    assert any(o.status=='API_ERROR' for o in m.coordinator.observations.values())

def test_peer_loss_stops_independent_c_navigation():
    m=MockMission();m.adapters['go2'].delay=1.;m.start()
    for _ in range(150):
        m.step()
        if any(v[2].goal_kind=='DEPOSIT_NAV' for v in m.adapters['go2'].active.values()):break
    assert any(v[2].goal_kind=='DEPOSIT_NAV' for v in m.adapters['go2'].active.values())
    m.online['go2']=False;m.run(60)
    assert m.coordinator.phase=='PAUSED'
    assert all(s.stopped for s in (x.state() for x in m.machines.values()))


def test_local_unreachable_does_not_make_shared_clue_globally_unusable():
    m=MockMission();m.adapters['go2'].unreachable=True
    m.start();m.run(120)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['go2'].state().ready
    assert m.machines['ranger'].state().ready

def test_duplicate_wire_commands_dont_duplicate_navigation():
    m=MockMission();m.duplicate_commands=True
    m.start();m.run(100)
    assert m.coordinator.phase=='DONE'
    for r in ('go2','ranger'):
        ids=[cmd for kind,p,cmd in m.adapters[r].calls]
        assert len(ids)==len(set(ids))

def test_two_consecutive_tasks_finish_with_fresh_command_sequences():
    m = MockMission()
    task_ids = []
    for _ in range(2):
        start = m.start()
        assert start.accepted
        task_ids.append(start.task_id)
        m.run(100)
        assert m.coordinator.phase == 'DONE', m.coordinator.reason
        assert not any(s.token for s in m.coordinator.states.values())
        for robot in ('go2', 'ranger'):
            commands = [c for r, c in m.executed if r == robot and c.task_id == start.task_id]
            assert commands[0].sequence == 1
            assert m.coordinator.states[robot].sequence == commands[-1].sequence
            assert m.coordinator.states[robot].deposited
    assert task_ids[0] != task_ids[1]

def test_paused_task_can_handshake_and_finish_a_new_task():
    m = MockMission()
    first = m.start()
    assert first.accepted
    m.run(2)
    m.pause('explicit restart')
    m.run(5)
    assert all(s.run_state == 'PAUSED' for s in m.coordinator.states.values())
    second = m.start()
    assert second.accepted and second.task_id != first.task_id
    m.run(100)
    assert m.coordinator.phase == 'DONE', m.coordinator.reason
    assert all(s.task_id == second.task_id and s.deposited for s in m.coordinator.states.values())
    assert not any(s.token for s in m.coordinator.states.values())

def test_door_polygon_does_not_create_transit_or_permit_navigation():
    config=mock_config()
    config['scene']['resources']['door']={'polygon':[[-.5,-5],[.5,-5],[.5,5],[-.5,5]],
        'transit_exits':{r:[[2,-3,0],[-2,-3,3.141592653589793]] for r in ('go2','ranger')}}
    m=MockMission(config)
    m.adapters['go2'].delay=1.0
    m.start();m.run(220)
    assert m.coordinator.phase=='DONE',m.coordinator.reason
    calls=m.adapters['go2'].calls
    kinds=[kind for kind,_,_ in calls]
    assert 'DOOR_TRANSIT' not in kinds
    assert 'DEPOSIT_NAV' in kinds
    assert not any(s.token for s in m.coordinator.states.values())
