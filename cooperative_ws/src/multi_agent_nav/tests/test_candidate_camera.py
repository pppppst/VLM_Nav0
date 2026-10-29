"""Candidate RGB projections reuse an existing frame; no ROS graph or model calls."""
import base64
import json
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
from PIL import Image
import pytest

from test_generated_candidates import generated_config, run_generated


def camera_frame():
    # Camera at map (1,2,1), looking +X; optical +X is map -Y, +Y is map -Z.
    rgb=np.full((240,320,3),(70,90,110),dtype=np.uint8)
    rgb.flags.writeable=False
    return S(rgb=rgb,intrinsics=(100.,100.,160.,120.),
        transform_matrix=np.array([[0.,0.,1.,1.],[-1.,0.,0.,2.],[0.,-1.,0.,1.],[0.,0.,0.,1.]]),
        raw_depth_snapshot=None,sequence=17,stamp=S(sec=123,nanosec=456),frame_id='color_optical')


def candidate(label,x,y):
    return dict(display_id=label,pose=[x,y,0.])


def test_rgb_projection_uses_inverse_image_time_transform_and_keeps_original(tmp_path):
    from multi_agent_nav.generated_candidates import render_candidate_camera
    frame=camera_frame();original=frame.rgb.copy()
    points=[candidate('P1',5.,1.),candidate('P2',5.,3.),candidate('P3',-1.,2.),
            candidate('P4',3.,-10.),candidate('P5',1.,2.),candidate('P6',float('nan'),2.)]
    path=render_candidate_camera(frame,points,tmp_path)
    meta=json.loads((tmp_path/'camera_projection.json').read_text())
    assert meta['projections']==[
        dict(candidate_id='P1',pixel=[185,145]),dict(candidate_id='P2',pixel=[135,145])]
    assert meta['unprojected_candidate_ids']==['P3','P4','P5','P6']
    assert meta['image_sequence']==17 and meta['image_stamp_ns']==123000000456
    assert meta['frame_id']=='color_optical'
    assert meta['projection_plane_z']==0. and meta['occlusion_checked'] is False
    assert meta['transform_matrix']==frame.transform_matrix.tolist()
    assert np.array_equal(np.asarray(Image.open(tmp_path/'camera_rgb.png')),original)
    marked=np.asarray(Image.open(path))
    assert np.array_equal(frame.rgb,original)
    assert tuple(marked[145,185])==(70,90,110)  # hollow marker retains the scene
    assert not np.array_equal(marked[135:155,175:195],original[135:155,175:195])
    assert np.array_equal(marked[10:30,10:30],original[10:30,10:30])


def test_no_in_view_candidates_still_produces_rgb_without_removing_map_choices(tmp_path):
    from multi_agent_nav.generated_candidates import render_candidate_camera
    path=render_candidate_camera(camera_frame(),[candidate('P1',-1.,2.)],tmp_path)
    assert Image.open(path).size[0]==320
    meta=json.loads((tmp_path/'camera_projection.json').read_text())
    assert meta['projections']==[] and meta['unprojected_candidate_ids']==['P1']


@pytest.mark.parametrize('fault',['intrinsics','transform','distortion'])
def test_invalid_projection_geometry_is_not_silently_drawn(tmp_path,fault):
    from multi_agent_nav.generated_candidates import render_candidate_camera
    frame=camera_frame()
    if fault=='intrinsics':frame.intrinsics=(0.,100.,160.,120.)
    elif fault=='transform':frame.transform_matrix[0,0]=float('nan')
    else:
        frame.raw_depth_snapshot=(frame.rgb,None,dict(color_info=dict(width=320,height=240,
            k=[100.,0.,160.,0.,100.,120.,0.,0.,1.],d=[.1,0.,0.,0.,0.])))
    with pytest.raises(ValueError):render_candidate_camera(frame,[candidate('P1',5.,1.)],tmp_path)


@pytest.mark.parametrize('ready',[True,False])
def test_adapter_reads_existing_snapshot_without_creating_subscriptions(ready):
    from multi_agent_nav.adapters.go2 import Go2Adapter
    frame=camera_frame();adapter=Go2Adapter.__new__(Go2Adapter)
    def forbidden(*args,**kwargs):raise AssertionError('must reuse the existing camera subscriber')
    adapter.node=S(latest_snapshot=frame,vlm_input_ready=lambda:ready,create_subscription=forbidden)
    if ready:assert adapter.selection_camera_snapshot() is frame
    else:
        with pytest.raises(RuntimeError,match='camera'):adapter.selection_camera_snapshot()


def test_generated_selection_uses_only_maps_without_reading_camera(generated_config,tmp_path,monkeypatch):
    from test_go2_exploration import LocalFixture
    def forbidden(self):raise AssertionError('selection must not read the camera')
    monkeypatch.setattr(LocalFixture,'selection_camera_snapshot',forbidden,raising=False)
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    assert outcome=='EXHAUSTED' and requests
    for request in requests:
        assert len(request['images'])==2
        directory=Path(request['evidence_directory'])
        assert request['images']==[str(directory/name) for name in ('occupancy.png','semantic.png')]
        for name in ('camera.png','camera_rgb.png','camera_projection.json'):
            assert not (directory/name).exists()
    assert any(e['state']=='CANDIDATE_SELECTED' for e in events)


@pytest.mark.parametrize('camera_exists',[True,False])
def test_candidate_request_uploads_only_maps_even_with_legacy_camera_path(monkeypatch,tmp_path,camera_exists):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    paths=[tmp_path/name for name in ('occupancy.png','semantic.png','camera.png')]
    for i,path in enumerate(paths):Image.new('RGB',(20,20),(i*30,50,80)).save(path)
    if not camera_exists:paths[2].unlink()
    calls=[]
    def completion(self,messages,max_tokens):
        calls.append(messages)
        return S(choices=[S(message=S(content='{"candidate_id":"P2","reason":"outside current view"}'))])
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',completion)
    client=ABCClient.__new__(ABCClient);client.client=S(api_key='secret')
    reply=client.choose_candidate(dict(images=list(map(str,paths)),task_id='task',decision_id='decision',
        main_target='cup',history=[],candidates=[dict(display_id='P1',pose=[1,2,0]),
        dict(display_id='P2',pose=[-1,2,0])]))
    content=calls[0][1]['content'];context=json.loads(content[0]['text'])
    assert reply['candidate_id']=='P2' and context['allowed_candidate_ids']==['P1','P2']
    assert context['images']==['occupancy map','exploration map with optional furniture annotations']
    assert len(content)==3
    for item,path in zip(content[1:],paths[:2]):
        assert base64.b64decode(item['image_url']['url'].split(',',1)[1])==path.read_bytes()
    assert 'RGB' not in context['legend'] and 'RGB' not in calls[0][0]['content']
    assert 'pose' not in content[0]['text']
