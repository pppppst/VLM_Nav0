"""Independent C completion, without protected areas or permit protocol."""
from dataclasses import replace
from pathlib import Path
from concurrent.futures import Future

from multi_agent_nav.mock import MockMission,mock_config
from multi_agent_nav.models import Pose2D,Command,validate_command


def independent_mission():
    config=mock_config();config['scene']['share_c_coordinates']=False
    config['scene']['resources']={};config['scene']['exit_points']={}
    return MockMission(config)


def test_empty_resource_and_exit_configuration_finishes_without_permits():
    mission=independent_mission();assert mission.start().accepted
    mission.run(300)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert all(s.ready and s.deposited and s.stopped for s in mission.coordinator.states.values())
    assert not any(c.kind=='EXIT' or c.token or c.resource_id for _,c in mission.executed)
    assert all(not s.token and not s.requested_resource and not s.exited for s in mission.coordinator.states.values())


def test_peer_search_does_not_delay_own_c_approach():
    mission=independent_mission();adapter=mission.adapters['ranger']
    del adapter.detections['C'];scan=adapter.start_scan
    pending=Future();work=[]
    def delayed(descriptions,scan_id,**options):
        if mission.coordinator.phase in ('SEARCH_C','DEPOSIT'):
            work.append((descriptions,scan_id,options));return pending
        return scan(descriptions,scan_id,**options)
    adapter.start_scan=delayed
    assert mission.start().accepted
    mission.run(120)
    assert work and mission.machines['go2'].state().deposited
    assert not mission.machines['ranger'].state().deposited
    assert mission.coordinator.phase!='DONE'
    adapter.detections['C']=Pose2D(9,1)
    args=work[0];pending.set_result(scan(args[0],args[1],**args[2]).result())
    mission.run(120)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason


def test_deposit_commands_can_be_outstanding_for_both_robots():
    mission=independent_mission();mission.adapters['go2'].delay=1.;mission.adapters['ranger'].delay=1.
    assert mission.start().accepted
    together=False
    for _ in range(200):
        mission.step()
        active=mission.coordinator.commands
        together|=all(r in active and active[r].kind=='APPROACH_DEPOSIT' for r in ('go2','ranger'))
    assert together and mission.coordinator.phase=='DONE'


def test_stale_polygons_cannot_reintroduce_resource_restrictions():
    mission=independent_mission()
    mission.config['scene']['resources']={'basket':{'polygon':[[-20,-20],[20,-20],[20,20],[-20,20]]},
        'door':{'polygon':[[-10,-10],[10,-10],[10,10],[-10,10]]}}
    mission=MockMission(mission.config)
    assert mission.start().accepted
    mission.run(300)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert all(kind!='DOOR_TRANSIT' for a in mission.adapters.values() for kind,_,_ in a.calls)


def test_no_gate_requires_resource_geometry_or_stopping_envelope_margin():
    from multi_agent_nav.config import load_config,live_gates
    cfg=load_config();cfg['scene'].pop('resources');cfg['scene'].pop('exit_points')
    cfg['robot_go2'].pop('stop_margin')
    errors=live_gates(cfg)
    assert not any('polygon' in e or 'exit' in e or 'stop margin' in e for e in errors)
    assert any('vision_validated' in e for e in errors)


def test_obsolete_exit_and_grant_commands_are_rejected():
    mission=independent_mission();assert mission.start().accepted;mission.run(2)
    context=mission.machines['go2'].context
    for action,kind in (('EXECUTE','EXIT'),('GRANT_RESOURCE','RELEASE')):
        command=Command(task_id=context.task_id,common_frame_id='',alignment_id='',recipient_id='go2',
            action=action,kind=kind)
        assert not validate_command(command,context).accepted
