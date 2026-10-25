"""RViz event display with synthetic messages; no ROS graph or hardware."""
from dataclasses import asdict
from types import SimpleNamespace as S

import pytest
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from multi_agent_nav.frame_adapter import LocalFrame
from multi_agent_nav.models import Observation, Pose2D


def display_fixture(frame=None):
    from multi_agent_nav.exploration_display import ExplorationDisplay
    messages={}
    def publisher(kind, topic, qos):
        assert qos.depth==1
        messages[topic.rsplit('/',1)[-1]]=[]
        return S(publish=messages[topic.rsplit('/',1)[-1]].append)
    node=S(create_publisher=publisher,get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=7))))
    return ExplorationDisplay(node,frame or LocalFrame()),messages


def test_only_current_candidates_visible_labels_and_selected_heading():
    display,messages=display_fixture()
    points=[dict(id=f'long/furniture/{i}',furniture_id='long furniture name',pose=[float(i),2.,.5],display_id=f'P{i}') for i in range(1,4)]
    display.record(dict(state='CANDIDATE_GEOMETRY',candidates=points,task_id='one'))
    assert len(messages['generated_candidates'][-1].markers)==1
    display.record(dict(state='CANDIDATE_REQUEST',candidates=points[:2],records={}))
    display.record(dict(state='CANDIDATE_SELECTED',viewpoint_id=points[1]['id']))
    candidates=messages['generated_candidates'][-1].markers[1:]
    assert [m.text for m in candidates if m.type==Marker.TEXT_VIEW_FACING]==['P1','P2']
    assert all(m.color.b>m.color.r+.5 for m in candidates)
    assert all(.12<=m.scale.z<=.16 for m in candidates)
    selected=messages['selected_candidate'][-1].markers[1:]
    assert selected[1].text=='VLM P2' and selected[0].scale.x==.4
    assert selected[0].color.r>selected[0].color.b+.5
    assert selected[0].pose.position.x==2. and selected[0].pose.orientation.z!=0.
    display.record(dict(state='CANDIDATE_REQUEST',candidates=points[2:],records={}))
    assert len(messages['selected_candidate'][-1].markers)==1
    assert messages['generated_candidates'][-1].markers[2].text=='P3'


def test_located_targets_use_map_transform_and_unknown_positions_are_not_fabricated():
    display,messages=display_fixture(S(to_local=lambda p:Pose2D(p.x-10.,p.y+1.,0.)))
    valid=Observation(target_id='A',status='DETECTED',position_valid=True,common_position=Pose2D(12.,3.))
    invalid=Observation(target_id='B',status='DETECTED',position_valid=False)
    nonfinite=Observation(target_id='C',status='DETECTED',position_valid=True,common_position=Pose2D(float('nan'),1.))
    display.record(dict(state='INITIAL_CAMERA_RESULT',observations=list(map(asdict,[valid,invalid,nonfinite]))))
    markers=messages['targets'][-1].markers[1:]
    assert len(markers)==2 and markers[1].text=='A'
    assert markers[0].color.g>markers[0].color.r+.5
    assert markers[0].scale.x==.18
    assert (markers[0].pose.position.x,markers[0].pose.position.y)==(2.,4.)
    assert markers[0].header.frame_id=='map' and markers[0].header.stamp.sec==7


def test_path_is_chosen_plan_only_and_new_task_clears_stale_display():
    display,messages=display_fixture()
    display.record(dict(state='DISPLAY_RESET',task_id='one'))
    display.record(dict(state='PATH_PLANNED',path=[asdict(Pose2D(1.,2.)),asdict(Pose2D(3.,4.))]))
    path=messages['planned_path'][-1]
    assert path.header.frame_id=='map' and path.header.stamp.sec==7
    assert [(p.pose.position.x,p.pose.position.y) for p in path.poses]==[(1.,2.),(3.,4.)]
    display.record(dict(state='CANDIDATE_REQUEST',candidates=[],records={}))
    assert not messages['planned_path'][-1].poses
    display.record(dict(state='PATH_PLANNED',path=[asdict(Pose2D(1.,2.))]))
    display.record(dict(state='DISPLAY_RESET',task_id='two'))
    assert not messages['planned_path'][-1].poses
    assert all(len(messages[t][-1].markers)==1 for t in ('targets','generated_candidates','selected_candidate'))


def test_rviz_has_small_target_and_path_displays():
    from pathlib import Path
    import yaml
    root=Path(__file__).resolve().parents[4]
    displays=yaml.safe_load((root/'config/go2_exploration.rviz').read_text())['Visualization Manager']['Displays']
    by_topic={d.get('Topic',{}).get('Value'):d for d in displays}
    assert '/vlm_nav/go2_exploration/targets' in by_topic
    path=by_topic['/vlm_nav/go2_exploration/planned_path']
    assert path['Class']=='rviz_default_plugins/Path' and path['Line Width']==.04
    assert path['Topic']['Durability Policy']=='Transient Local'


@pytest.mark.parametrize('event',[dict(state='STOPPING'),dict(state='STOP_UNCONFIRMED'),dict(event='SESSION_END')])
def test_stop_clears_active_markers_and_path_but_keeps_detected_targets(event):
    display,messages=display_fixture()
    observation=Observation(status='DETECTED',position_valid=True,common_position=Pose2D(2.,3.))
    display.record(dict(state='INITIAL_CAMERA_RESULT',observations=[asdict(observation)]))
    point=dict(id='p',display_id='P1',pose=[1.,2.,0.])
    display.record(dict(state='CANDIDATE_REQUEST',candidates=[point],records={}))
    display.record(dict(state='CANDIDATE_SELECTED',viewpoint_id='p'))
    display.record(dict(state='PATH_PLANNED',path=[asdict(Pose2D(1.,2.))]))
    display.record(event)
    assert all(len(messages[t][-1].markers)==1 for t in ('generated_candidates','selected_candidate'))
    assert not messages['planned_path'][-1].poses
    assert len(messages['targets'][-1].markers)==3


def test_display_publish_failure_does_not_raise_into_navigation():
    display,_=display_fixture();warnings=[]
    display.node.get_logger=lambda:S(warning=lambda *args,**kwargs:warnings.append(args))
    def failed_publish(message):raise RuntimeError('synthetic transport failure')
    display.publishers['planned_path'].publish=failed_publish
    display.record(dict(state='STOPPING'))
    assert warnings
