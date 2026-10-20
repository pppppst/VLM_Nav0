"""Unlabelled-map candidates and the shared executor, without ROS or provider calls."""
from pathlib import Path
import math

import numpy as np
from PIL import Image
import pytest
import yaml

from test_generated_candidates import generated_config,run_generated
from multi_agent_nav.generated_candidates import GeneratedCandidates,circle_free


def no_labels(config):
    path=Path(config['robot_go2']['generated_candidates']['semantic_yaml'])
    path.unlink()
    return path


@pytest.mark.parametrize('mode',['missing','unset','empty'])
def test_no_labels_still_generate_stable_free_candidates(generated_config,mode):
    cfg=generated_config;path=Path(cfg['robot_go2']['generated_candidates']['semantic_yaml'])
    if mode=='empty':
        doc=yaml.safe_load(path.read_text());doc['furniture']=[];path.write_text(yaml.safe_dump(doc))
    else:
        path.unlink()
        if mode=='unset':del cfg['robot_go2']['generated_candidates']['semantic_yaml']
    catalog=GeneratedCandidates(cfg)
    assert catalog.points and not catalog.furniture
    assert catalog.points==GeneratedCandidates(cfg).points
    assert all(p['id'].startswith('free/') and p['furniture_id']=='' for p in catalog.points)
    for point in catalog.points:
        x,y=catalog.map_xy(*point['pose'][:2])
        assert circle_free(catalog.raw,x,y,catalog.radius,catalog.source['resolution'],lambda a:a>=253)
    assert path.exists()==(mode=='empty')


def test_furniture_points_are_kept_alongside_free_space(generated_config):
    catalog=GeneratedCandidates(generated_config)
    furniture=[p for p in catalog.points if p['furniture_id']]
    free=[p for p in catalog.points if not p['furniture_id']]
    assert len(furniture)==4 and free
    assert len({p['id'] for p in catalog.points})==len(catalog.points)
    gap=max(1.,2*catalog.radius)/2
    assert all(math.hypot(p['pose'][0]-f['pose'][0],p['pose'][1]-f['pose'][1])>=gap
               for p in free for f in furniture)
    assert all(math.hypot(max(abs(p['pose'][0])-1.,0.),max(abs(p['pose'][1])-1.,0.))>catalog.radius for p in free)


def test_optional_asset_creation_deletion_and_corruption_are_detected(generated_config):
    path=no_labels(generated_config);catalog=GeneratedCandidates(generated_config)
    assert catalog.unchanged()
    path.write_text('invalid: [')
    assert not catalog.unchanged()
    with pytest.raises((ValueError,yaml.YAMLError)):GeneratedCandidates(generated_config)
    path.unlink();assert catalog.unchanged()


@pytest.mark.parametrize('cell',[0,128])
def test_obstacle_or_unknown_under_rotation_disk_rejects_free_candidate(generated_config,cell):
    no_labels(generated_config)
    catalog=GeneratedCandidates(generated_config)
    first=catalog.points[0]
    image=Path(catalog.source['image']);raw=np.array(Image.open(image))
    raw[raw.shape[0]-1-5,8]=cell
    Image.fromarray(raw).save(image)
    from multi_agent_nav.config import map_file_info
    info=map_file_info(generated_config['robot_go2']['map_path'])
    generated_config['robot_go2']['map_digest']=info['map_digest']
    generated_config['alignment']['robots']['go2']['map_version']='sha256:'+info['map_digest']
    after=GeneratedCandidates(generated_config)
    assert first['id'] not in {p['id'] for p in after.points}


def test_rotated_map_and_ranger_geometry_use_own_coordinates(generated_config):
    from copy import deepcopy
    cfg=generated_config;no_labels(cfg)
    map_path=Path(cfg['robot_go2']['map_path']);doc=yaml.safe_load(map_path.read_text())
    doc['origin']=[100.,200.,math.pi/2];map_path.write_text(yaml.safe_dump(doc))
    from multi_agent_nav.config import map_file_info
    info=map_file_info(map_path)
    cfg['robot_ranger']=deepcopy(cfg['robot_go2'])
    cfg['robot_ranger'].update(map_digest=info['map_digest'],footprint=[[-.8,-.4],[.8,-.4],[.8,.4],[-.8,.4]])
    cfg['alignment']['robots']['ranger']=dict(local_map_id='ranger',map_version='sha256:'+info['map_digest'])
    catalog=GeneratedCandidates(cfg,'ranger')
    assert catalog.points
    for point in catalog.points:
        assert point['pose'][0]<100 and point['pose'][1]>200
        assert point['pose'][2]==pytest.approx(math.pi/2)
        x,y=catalog.map_xy(*point['pose'][:2])
        assert circle_free(catalog.raw,x,y,catalog.radius,.1,lambda a:a>=253)


def test_no_label_single_execution_orders_nearby_points_before_map_prefix(generated_config,tmp_path):
    no_labels(generated_config)
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'found')
    assert outcome=='TARGET_FOUND' and len(requests)==1
    candidates=requests[0]['candidates']
    assert len(candidates)==12
    distances=[math.hypot(*p['pose'][:2]) for p in candidates]
    assert distances==sorted(distances)
    assert all(not p['furniture_id'] for p in candidates)
    assert set(requests[0]['images']) and adapter.scans==1


def test_no_label_dual_fake_keeps_private_c_and_independent_completion(generated_config,tmp_path):
    from test_local_tasks import generated_mission
    mission=generated_mission(generated_config,tmp_path)
    for robot in ('go2','ranger'):
        Path(mission.config[f'robot_{robot}']['generated_candidates']['semantic_yaml']).unlink()
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    assert all(s.ready and s.deposited and s.stopped for s in mission.coordinator.states.values())
    assert [r for r,c in mission.executed if c.kind=='APPROACH_DEPOSIT']==['go2','ranger']
    assert all(not c.position_valid for _,c in mission.executed)
    assert not any(s.token for s in mission.coordinator.states.values())


@pytest.mark.parametrize('asset',['semantic','image','yaml'])
def test_asset_deletion_invalidates_existing_catalog(generated_config,asset):
    catalog=GeneratedCandidates(generated_config)
    path={'semantic':catalog.semantic_path,'image':Path(catalog.source['image']),'yaml':catalog.map_path}[asset]
    path.unlink()
    assert not catalog.unchanged()


def test_unlabelled_dual_images_keep_same_markers_and_map_bytes(generated_config,tmp_path):
    from multi_agent_nav.generated_candidates import render_selection
    from multi_agent_nav.models import Pose2D
    no_labels(generated_config);catalog=GeneratedCandidates(generated_config)
    original=Path(catalog.source['image']).read_bytes()
    images=render_selection(catalog,[dict(catalog.points[0],display_id='P1')],{},Pose2D(),tmp_path/'images')
    assert len(images)==2
    assert np.array_equal(np.asarray(Image.open(images[0])),np.asarray(Image.open(images[1])))
    assert Path(catalog.source['image']).read_bytes()==original


def test_no_label_unexecutable_candidates_never_call_vlm(generated_config,tmp_path):
    no_labels(generated_config)
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'blocked')
    assert outcome=='WAITING_VIEWPOINTS' and not requests and not adapter.calls


def test_existing_bad_label_is_still_a_gate_failure(generated_config):
    from multi_agent_nav.go2_exploration import exploration_gates
    path=Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
    path.write_text('invalid: [')
    assert any('generated candidates invalid' in e for e in exploration_gates(generated_config))
