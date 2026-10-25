from dataclasses import replace

import pytest

from multi_agent_nav.mock import MockMission, mock_config
from multi_agent_nav.models import Pose2D


def private_mission():
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    config['alignment']['validated'] = False
    config['alignment']['alignment_id'] = ''
    config['alignment']['robots']['ranger']['transform'] = None
    return MockMission(config)


def test_planner_sends_target_intents_without_choosing_viewpoints():
    mission = private_mission()
    assert mission.start().accepted
    mission.run(5)
    commands = [c for _, c in mission.executed]
    assert len(commands) == 2
    assert [(c.recipient_id, c.target_id) for c in commands] == [('go2','A'), ('ranger','B')]
    assert all(c.kind == 'APPROACH_PICKUP' and not c.position_valid and not c.viewpoint_id
               and c.common_goal == c.object_position == Pose2D() for c in commands)


def test_dual_fake_local_tasks_keep_private_c_without_permits():
    mission = private_mission()
    wire = []
    for runner in mission.runners.values():
        def publish(o):
            wire.append(o)
            mission.coordinator.ingest_observation(o, mission.now)
        runner.on_observation = publish
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert all(s.ready and s.deposited and s.stopped for s in mission.coordinator.states.values())
    deposits = [(r,c) for r,c in mission.executed if c.kind == 'APPROACH_DEPOSIT']
    assert [r for r,_ in deposits] == ['go2','ranger']
    for robot, deposit in deposits:
        assert deposit.observation_id.startswith(robot + '/')
        assert not deposit.token and not deposit.resource_id
    assert all(not o.position_valid and o.common_position == Pose2D() for o in wire)
    assert all(not c.position_valid for _,c in mission.executed)
    assert not any(s.token for s in mission.coordinator.states.values())


@pytest.mark.parametrize('distance', [1.01, 1.5])
def test_successful_nav_outside_one_meter_cannot_ready(distance):
    mission = private_mission()
    adapter = mission.adapters['go2']
    advance = adapter.advance
    def overshoot(now):
        terminal = any(now >= v[0] and v[2].goal_kind == 'PICKUP_NAV' for v in adapter.active.values())
        advance(now)
        if terminal:
            adapter.pose = Pose2D(-3.-distance, -3.)
    adapter.advance = overshoot
    assert mission.start().accepted
    mission.run(300)
    assert not mission.machines['go2'].state().ready
    assert not any(c.kind == 'APPROACH_DEPOSIT' for _,c in mission.executed)


def test_successful_nav_without_stopping_cannot_ready():
    mission = private_mission()
    adapter = mission.adapters['go2']
    get_health = adapter.get_health
    def unconfirmed():
        health = get_health()
        if any(kind == 'PICKUP_NAV' for kind,_,_ in adapter.calls) and not adapter.active:
            return replace(health, stopped=False)
        return health
    adapter.get_health = unconfirmed
    assert mission.start().accepted
    mission.run(150)
    assert not mission.machines['go2'].state().ready


def test_peer_health_without_common_pose_keeps_local_heartbeat():
    from types import SimpleNamespace
    from multi_agent_nav.agent_node import AgentNode
    from multi_agent_nav.ros_codec import to_ros
    mission = private_mission()
    assert mission.start().accepted
    mission.run(5)
    agent = mission.machines['go2']
    agent.peer_received = None
    peer = mission.machines['ranger'].state()
    assert not peer.pose_valid and peer.common_pose == Pose2D()
    node = SimpleNamespace(machine=agent, simulation=True, config=mission.config, peer_map_identity=None)
    AgentNode.peer(node, to_ros(peer), 'ranger')
    assert agent.peer_received is not None and not agent.paused
    AgentNode.peer(node, to_ros(replace(peer,map_version='changed')), 'ranger')
    assert agent.paused


def test_live_local_gates_require_local_assets_but_not_remote_alignment():
    from multi_agent_nav.config import load_config,live_gates
    config = load_config()
    config['alignment']['robots']['ranger']['transform'] = None
    errors = live_gates(config)
    assert not any('alignment report' in error or 'ranger: map' in error for error in errors)
    assert not any('exit' in error for error in errors)
    assert not any('generated' in error for error in errors)
    assert any('vision_validated' in error for error in errors)


def test_map_identity_change_pauses_active_local_task():
    mission = private_mission()
    assert mission.start().accepted
    mission.run(5)
    assert mission.coordinator.phase == 'SEARCH_AB'
    state = mission.machines['ranger'].state()
    mission.coordinator.ingest_state(replace(state, map_version='new-map'),mission.now)
    assert mission.coordinator.phase == 'PAUSED'

from test_generated_candidates import generated_config


def generated_mission(config, tmp_path):
    from copy import deepcopy
    from pathlib import Path
    import yaml
    fixture=mock_config()
    fixture['scene']['share_c_coordinates']=False
    for robot in ('go2','ranger'):
        cfg=deepcopy(config['robot_go2'])
        cfg['experiment_records_root']=str(tmp_path/robot/'records')
        cfg['api_timeout']=20.
        semantic=yaml.safe_load(Path(cfg['generated_candidates']['semantic_yaml']).read_text())
        semantic['source_robot']=robot
        path=tmp_path/(robot+'-semantic.yaml')
        path.write_text(yaml.safe_dump(semantic))
        cfg['generated_candidates']['semantic_yaml']=str(path)
        fixture[f'robot_{robot}']=cfg
        fixture['alignment']['robots'][robot].update(config['alignment']['robots']['go2'])
        fixture['alignment']['robots'][robot]['local_map_id']=robot+'-fixture'
    return MockMission(fixture)


def test_both_local_vlms_choose_generated_ids(generated_config,tmp_path):
    mission=generated_mission(generated_config,tmp_path)
    requests={r:[] for r in mission.adapters}
    for robot,adapter in mission.adapters.items():
        def choose(snapshot,robot=robot):
            requests[robot].append(snapshot)
            return dict(candidate_id=snapshot['candidates'][0]['display_id'],reason='fixture choice')
        adapter.client.choose_candidate=choose
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert all(requests.values())
    assert requests['go2'][0]['main_target']=='A'
    assert requests['ranger'][0]['main_target']=='B'
    assert all(not c.position_valid and not c.viewpoint_id for _,c in mission.executed)


@pytest.mark.parametrize('interrupt', ['cancel','peer_loss'])
def test_pending_vlm_keeps_heartbeat_and_late_selection_cannot_move(generated_config,tmp_path,interrupt):
    from concurrent.futures import Future
    mission=generated_mission(generated_config,tmp_path)
    adapter=mission.adapters['go2']
    future=Future();future.set_running_or_notify_cancel()
    work=[]
    adapter.vision.submit=lambda fn:(work.append(fn) or future)
    assert mission.start().accepted
    mission.run(15)
    assert work and not future.done() and not adapter.calls
    assert mission.coordinator.phase=='SEARCH_AB'
    assert mission.machines['go2'].sync_received>=mission.now-.2
    if interrupt=='cancel':mission.pause('test cancel')
    else:mission.online['ranger']=False
    mission.run(40)
    assert mission.machines['go2'].paused
    future.set_result(work[0]())
    mission.run(10)
    assert not adapter.calls and mission.runners['go2'].execution is None


def test_local_navigation_ignores_legacy_door_and_basket_regions():
    mission=private_mission()
    # Recreate machines with the additional local resource polygon.
    mission.config['scene']['resources']['door']={'polygon':[[-.5,-5],[.5,-5],[.5,5],[-.5,5]],
        'transit_exits':{r:[[2,-3,0],[-2,-3,3.141592653589793]] for r in ('go2','ranger')}}
    mission=MockMission(mission.config)
    mission.config['scene']['viewpoints']['ranger'][0]['pose']=[2.5,-2,0]
    mission.adapters['go2'].delay=1.
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    calls=mission.adapters['go2'].calls
    deposit=next(c.command_id for r,c in mission.executed if r=='go2' and c.kind=='APPROACH_DEPOSIT')
    assert all(kind!='DOOR_TRANSIT' for kind,_,_ in calls)
    assert any(kind=='DEPOSIT_NAV' and command==deposit for kind,_,command in calls)
    assert not any(s.token for s in mission.coordinator.states.values())


@pytest.mark.parametrize('kind', ['PICKUP_NAV','DEPOSIT_NAV'])
@pytest.mark.parametrize('fault', ['distance','stop','canceled'])
@pytest.mark.parametrize('robot', ['go2','ranger'])
def test_target_acceptance_rejects_bad_terminal_geometry_or_stop(kind,fault,robot):
    mission=private_mission();adapter=mission.adapters[robot]
    advance=adapter.advance
    terminal=[False]
    def inject(now):
        finishing=[v for v in adapter.active.values() if now>=v[0] and v[2].goal_kind==kind]
        if fault=='canceled':
            for v in finishing:v[2].status='CANCELED'
        advance(now)
        if finishing:
            terminal[0]=True
            if fault=='distance':
                obj=adapter.detections[('A' if robot=='go2' else 'B') if kind=='PICKUP_NAV' else 'C']
                adapter.pose=Pose2D(obj.x-1.1,obj.y)
    adapter.advance=inject
    health=adapter.get_health
    adapter.get_health=lambda:replace(health(),stopped=False) if fault=='stop' and terminal[0] else health()
    assert mission.start().accepted
    mission.run(400)
    state=mission.machines[robot].state()
    assert terminal[0]
    assert not (state.ready if kind=='PICKUP_NAV' else state.deposited)


@pytest.mark.parametrize('field,value', [('standoff_distance',None),('approach_distance_margin',0),
    ('arrival_config_files',[]),('scan_angles',[])])
def test_live_local_gates_validate_parameters_used_by_target_planning(field,value):
    from multi_agent_nav.config import load_config,live_gates
    config=load_config();config['robot_go2'][field]=value
    errors=live_gates(config)
    if field=='standoff_distance':
        from multi_agent_nav.go2_exploration import exploration_gates
        assert not any('standoff_distance' in error for error in errors)
        config['robot_go2']['approach_on_detection']=True
        assert any('standoff_distance' in error for error in exploration_gates(config))
        return
    expected={'standoff_distance':'standoff_distance','approach_distance_margin':'approach_distance_margin',
        'arrival_config_files':'arrival contract','scan_angles':'scan angles'}
    assert any(expected[field] in error for error in errors)


@pytest.mark.parametrize('field', ['resources','exit_points'])
def test_malformed_local_geometry_is_reported_as_gate_failure(field):
    from multi_agent_nav.config import load_config,live_gates
    config=load_config();config['scene'][field]=None
    assert live_gates(config)


@pytest.mark.parametrize('kind,target', [('APPROACH_PICKUP','A'),('SEARCH_VIEWPOINT','C'),
    ('APPROACH_DEPOSIT','C'),('EXIT','C')])
@pytest.mark.parametrize('position_valid', [False,True])
def test_private_command_cannot_bypass_local_execution_with_coordinates(kind,target,position_valid):
    from multi_agent_nav.models import Command,validate_command
    mission=private_mission();assert mission.start().accepted
    mission.run(2)
    context=mission.machines['go2'].context
    command=Command(task_id=context.task_id,recipient_id='go2',common_frame_id='',alignment_id='',
        kind=kind,target_id=target,observation_id='own-C',position_valid=position_valid,
        object_position=Pose2D(10,0))
    assert not validate_command(command,context).accepted


@pytest.mark.parametrize('distance', [.90,1.00])
def test_confirmed_current_target_navigation_within_radius_completes(distance):
    mission=private_mission();adapter=mission.adapters['go2'];advance=adapter.advance
    def settled(now):
        finishing=[v[2].goal_kind for v in adapter.active.values() if now>=v[0]
            and v[2].goal_kind in ('PICKUP_NAV','DEPOSIT_NAV')]
        advance(now)
        for kind in finishing:
            target=adapter.detections['A' if kind=='PICKUP_NAV' else 'C']
            adapter.pose=Pose2D(target.x-distance,target.y)
    adapter.advance=settled
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert mission.machines['go2'].state().ready and mission.machines['go2'].state().deposited


def test_legacy_basket_region_cannot_block_search_or_ready_barrier():
    mission=private_mission()
    mission.config['scene']['resources']['basket']['polygon']=[[-.5,-5],[.5,-5],[.5,5],[-.5,5]]
    mission=MockMission(mission.config)
    mission.config['scene']['viewpoints']={'go2':[dict(id='g',pose=[2,-2,0])],
        'ranger':[dict(id='r',pose=[-2,-2,0])]}
    assert mission.start().accepted
    mission.run(120)
    assert mission.coordinator.phase=='DONE'
    assert not any(s.token for s in mission.coordinator.states.values())
    assert any(c.kind=='APPROACH_DEPOSIT' for _,c in mission.executed)


def test_distinct_local_map_coordinates_need_no_shared_transform():
    from copy import deepcopy
    from multi_agent_nav.mock import make_machine
    mission=private_mission()
    ranger_config=deepcopy(mission.config)
    def shifted(point):return [point[0]+100.,point[1]+200.,point[2]]
    for point in ranger_config['scene']['viewpoints']['ranger']:point['pose']=shifted(point['pose'])
    ranger_config['scene']['exit_points']['ranger']=shifted(ranger_config['scene']['exit_points']['ranger'])
    for region in ranger_config['scene']['resources'].values():
        region['polygon']=[[x+100.,y+200.] for x,y in region['polygon']]
    adapter=mission.adapters['ranger']
    adapter.pose=Pose2D(*shifted([adapter.pose.x,adapter.pose.y,adapter.pose.yaw]))
    adapter.detections={t:Pose2D(*shifted([p.x,p.y,p.yaw])) for t,p in adapter.detections.items()}
    machine,frame=make_machine(ranger_config,'ranger',True)
    mission.machines['ranger']=machine;mission.frames['ranger']=frame
    mission.runners['ranger'].machine=machine;mission.runners['ranger'].config=ranger_config
    machine.local_execution_enabled=True
    machine.tick(0,adapter.get_health())
    mission.coordinator.ingest_state(machine.state(),0)
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert all(goal.x>90 and goal.y>190 for _,goal,_ in adapter.calls)
