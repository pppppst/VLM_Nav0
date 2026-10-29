"""Generated geometry, image contract and the existing serial executor, without ROS."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
from PIL import Image
import pytest
import yaml


@pytest.mark.parametrize('stage',['geometry','unchanged','image','model'])
@pytest.mark.parametrize('cancel',[False,True])
def test_expensive_preparation_keeps_heartbeats_and_cancel_responsive(generated_config,tmp_path,monkeypatch,stage,cancel):
    import threading
    import time
    from test_go2_exploration import LocalFixture
    from test_startup_handshake import activate,peer,sync
    import multi_agent_nav.go2_exploration as exploration
    from multi_agent_nav.adapters.go2 import AsyncVision
    from multi_agent_nav.frame_adapter import LocalFrame
    config,agent,health=activate()
    generated_config['robot_go2']['api_timeout']=30.
    adapter=LocalFixture()
    adapter.vision=AsyncVision(10.)
    adapter.selection_evidence_dir=tmp_path/'selection'
    adapter.client=S(choose_candidate=lambda snapshot:dict(candidate_id='P1',reason='offline'))
    started=threading.Event();release=threading.Event();main_thread=threading.get_ident()
    def blocking(function):
        def work(*args,**kwargs):
            assert threading.get_ident()!=main_thread,'expensive work ran on heartbeat path'
            started.set()
            assert release.wait(2.),'test worker was not released'
            return function(*args,**kwargs)
        return work
    if stage=='geometry':monkeypatch.setattr(exploration,'GeneratedCandidates',blocking(exploration.GeneratedCandidates))
    elif stage=='unchanged':monkeypatch.setattr(exploration.GeneratedCandidates,'unchanged',blocking(exploration.GeneratedCandidates.unchanged))
    elif stage=='image':monkeypatch.setattr(exploration,'render_selection',blocking(exploration.render_selection))
    else:adapter.client.choose_candidate=blocking(adapter.client.choose_candidate)
    now=[.2];events=[]
    steps=exploration.explore_steps(adapter,generated_config,LocalFrame(),600.,'target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:now[0])
    try:
        for _ in range(200):
            adapter.vision.poll();next(steps)
            if started.is_set():break
            time.sleep(.001)
        assert started.is_set()
        heartbeats=[]
        for tick in range(1,21):
            now[0]=.2+tick*.5
            agent.accept(sync(phase='SEARCH_AB'),now[0])
            agent.receive_peer(peer(config,task_id='task',run_state='PLANNING'),now[0])
            agent.tick(now[0],health)
            heartbeats.append(agent.state())
            adapter.vision.poll();next(steps)
        assert len(heartbeats)==20 and not agent.paused and not adapter.calls
        if cancel:
            agent.stop('explicit test cancel');agent.tick(now[0],health)
            steps.close();adapter.vision.cancel()
            release.set()
            adapter.vision.executor.shutdown(wait=True)
            adapter.vision.poll()
            assert agent.paused and not adapter.calls
        else:
            release.set()
            for _ in range(300):
                adapter.vision.poll();next(steps)
                if any(e.get('state')=='CANDIDATE_SELECTED' for e in events):break
                time.sleep(.001)
            assert any(e.get('state')=='CANDIDATE_SELECTED' for e in events)
    finally:
        release.set();steps.close();adapter.vision.close();adapter.vision.executor.shutdown(wait=True)


@pytest.fixture
def generated_config(tmp_path):
    from multi_agent_nav.config import map_file_info
    Image.new('L', (100, 100), 254).save(tmp_path/'map.pgm')
    source = dict(image='map.pgm', resolution=.1, origin=[-5., -5., 0.],
                  negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(source))
    info = map_file_info(tmp_path/'map.yaml')
    semantic = dict(schema_version=1, source_robot='go2', frame_id='map', units='meters',
                    source_map=dict(yaml=info['yaml_path'], image=info['image_path'],
                                    yaml_sha256=info['yaml_sha256'], image_sha256=info['image_sha256']),
                    furniture=[dict(id='table', name='desk', category='table',
                                    polygon=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])])
    (tmp_path/'semantic.yaml').write_text(yaml.safe_dump(semantic))
    return dict(alignment=dict(robots=dict(go2=dict(local_map_id='fixture', map_version='sha256:'+info['map_digest']))),
                scene=dict(resources={}, targets=dict(A='target', B='box', C='basket')),
                robot_go2=dict(exploration_strategy='generated_candidates_vlm',
                               map_path=str(tmp_path/'map.yaml'), map_digest=info['map_digest'],
                               footprint=[[.38,.18],[.38,-.18],[-.38,-.18],[-.38,.18]],
                               footprint_padding=0., stop_margin=.1, scan_angles=[0.], api_timeout=1.,
                               generated_candidates=dict(semantic_yaml=str(tmp_path/'semantic.yaml'), observation_distance=1.)))


def test_each_edge_faces_furniture_for_both_windings(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    for reversed_order in (False, True):
        path=Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
        if reversed_order:
            doc=yaml.safe_load(path.read_text());doc['furniture'][0]['polygon'].reverse()
            path.write_text(yaml.safe_dump(doc))
        catalog=GeneratedCandidates(generated_config)
        furniture=[p for p in catalog.points if p['furniture_id']]
        positions={tuple(p['pose'][:2]) for p in furniture}
        assert positions=={(0.,-2.),(2.,0.),(0.,2.),(-2.,0.)}
        for point in furniture:
            x,y,yaw=point['pose']
            assert x*math.cos(yaw)+y*math.sin(yaw)<-1.99
        assert len({p['id'] for p in furniture})==4


def test_static_filter_rejects_unknown_occupied_bounds_and_scan_corner(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    image=np.full((100,100),254,dtype=np.uint8)
    image[69,50]=128  # unknown at bottom candidate with this map's .25/.65 thresholds
    image[49,70]=0    # occupied at right candidate
    image[29,54]=0    # intersects rotation envelope of top candidate, beyond narrow parked body
    Image.fromarray(image).save(Path(generated_config['robot_go2']['map_path']).parent/'map.pgm')
    from multi_agent_nav.config import map_file_info
    info=map_file_info(generated_config['robot_go2']['map_path'])
    generated_config['robot_go2']['map_digest']=info['map_digest']
    generated_config['alignment']['robots']['go2']['map_version']='sha256:'+info['map_digest']
    path=Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
    doc=yaml.safe_load(path.read_text())
    doc['source_map'].update(yaml_sha256=info['yaml_sha256'],image_sha256=info['image_sha256'])
    path.write_text(yaml.safe_dump(doc))
    catalog=GeneratedCandidates(generated_config)
    assert [p['pose'][:2] for p in catalog.points if p['furniture_id']]==[[-2.,0.]]
    generated_config['robot_go2']['generated_candidates']['observation_distance']=10.
    assert not any(p['furniture_id'] for p in GeneratedCandidates(generated_config).points)


@pytest.mark.parametrize('fault',['distance','polygon','digest'])
def test_generated_input_validation(generated_config,fault):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    options=generated_config['robot_go2']['generated_candidates']
    if fault=='distance':options['observation_distance']=float('nan')
    else:
        path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
        if fault=='polygon':doc['furniture'][0]['polygon']=[[0,0],[1,1],[0,1],[1,0]]
        else:doc['source_map']['image_sha256']='wrong'
        path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError):GeneratedCandidates(generated_config)


def test_dual_images_have_identical_marks_and_do_not_write_map(generated_config,tmp_path):
    from multi_agent_nav.generated_candidates import GeneratedCandidates, render_selection
    from multi_agent_nav.models import Pose2D
    catalog=GeneratedCandidates(generated_config)
    original=Path(catalog.source['image']).read_bytes()
    point=dict(catalog.points[0], display_id='P1')
    images=render_selection(catalog,[point],{catalog.points[1]['id']:dict(status='observed')},
                            Pose2D(-3.,-3.,math.pi/2),tmp_path/'render')
    arrays=[np.array(Image.open(path).convert('RGB')) for path in images]
    assert arrays[0].shape==arrays[1].shape
    for color in ((30,100,230),(240,140,20),(125,125,125)):
        masks=[np.all(a==color,axis=-1) for a in arrays]
        assert masks[0].any() and np.array_equal(*masks)
    for a in arrays:
        assert tuple(a[209,150])==(240,140,20)  # bottom edge candidate (0,-2)
        assert tuple(a[149,210])==(125,125,125)  # completed right edge (2,0)
        assert tuple(a[239,60])==(30,100,230)  # robot (-3,-3)
    assert Path(catalog.source['image']).read_bytes()==original


def run_generated(config,tmp_path,mode='normal',initial_camera_check=False,camera_snapshot=None,**execution_options):
    from test_go2_exploration import LocalFixture
    from multi_agent_nav.adapters.fake import completed
    from multi_agent_nav.models import Pose2D, Observation, ApproachResult
    from multi_agent_nav.frame_adapter import FrameAdapter
    from multi_agent_nav.go2_exploration import explore
    from concurrent.futures import Future
    adapter=LocalFixture();events=[];requests=[];now=[0.]
    if camera_snapshot is not None:adapter.selection_camera_snapshot=lambda:camera_snapshot
    adapter.selection_evidence_dir=tmp_path/'requests'
    adapter.observation_candidate_free=lambda goal: mode!='blocked' and not (mode=='arrived_blocked' and adapter.calls)
    def choose(snapshot):
        requests.append(snapshot)
        return dict(candidate_id='unknown' if mode=='invalid' else 'P1',reason='unobserved side')
    adapter.client=S(choose_candidate=choose)
    pending=Future()
    def submit(fn):
        if mode in ('cancel','late') and events and events[-1].get('state')=='CANDIDATE_REQUEST':
            pending.work=fn;return pending
        return completed(fn())
    adapter.vision=S(submit=submit)
    def scan(descriptions,scan_id,*,stop_on_target=None,current_view_only=False,task_id=''):
        adapter.scans+=1
        return completed([Observation(target_id=t,view_index=i,
                           status='API_ERROR' if mode=='api' or mode=='partial_detection' and t!='A' else
                               ('DETECTED' if mode in ('found','partial_detection') and t=='A' else 'NOT_DETECTED'),
                           position_valid=mode in ('found','partial_detection') and t=='A',
                           common_position=Pose2D(5.,2.)) for t in descriptions
                          for i in range(1 if mode in ('found','partial_detection') else len(config['robot_go2']['scan_angles']))])
    adapter.start_scan=scan
    adapter.select_approach=lambda *args,**kwargs:completed(ApproachResult(Pose2D(4.5,2.)))
    def spin():
        now[0]+=.05;adapter.advance(now[0])
        if hasattr(pending,'work') and not pending.done():
            if mode=='cancel':adapter.selection_epoch=1
            if mode=='late':now[0]=2.
            pending.set_result(pending.work())
    def stop():adapter.stop_motion('stop');adapter.advance(now[0]+1);return True
    try:
        outcome=explore(adapter,spin,config,FrameAdapter('fixture','1','lab_ref','',[0,0,0]),
                        600.,'target',events.append,lambda:[],stop,lambda:None,clock=lambda:now[0],
                        initial_camera_check=initial_camera_check,**execution_options)
    except Exception:
        if mode in ('invalid','late','cancel'):assert not adapter.calls
        if mode=='api':assert len(adapter.calls)==1 and adapter.scans==1
        raise
    return adapter,events,requests,outcome


def test_fake_select_observe_next_and_current_id_mapping(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    points=GeneratedCandidates(generated_config).points
    assert outcome=='EXHAUSTED' and adapter.scans==len(points) and len(requests)==len(points)
    assert {(c[1].x,c[1].y) for c in adapter.calls}=={tuple(p['pose'][:2]) for p in points}
    assert len({r['decision_id'] for r in requests})==len(points)
    assert [Path(r['evidence_directory']).name for r in requests]==[f'{i:04d}' for i in range(1,len(requests)+1)]
    assert all(Path(r['images'][0]).parent==Path(r['evidence_directory']) for r in requests)
    assert all(r['decision_id']!=Path(r['evidence_directory']).name for r in requests)
    assert requests[0]['task_id']==next(e['task_id'] for e in events if e['state']=='NAVIGATING')
    assert requests[1]['candidates'][0]['display_id']=='P1'
    assert requests[1]['candidates'][0]['id']!=requests[0]['candidates'][0]['id']
    assert requests[1]['history'][0]['candidate_id']=='P1'
    assert requests[1]['history'][0]['result']=='NOT_DETECTED'
    assert not any(e['state']=='READY' for e in events)


@pytest.mark.parametrize('mode',['invalid','late','cancel'])
def test_bad_decision_cannot_move(generated_config,tmp_path,mode):
    with pytest.raises((ValueError,RuntimeError)):
        run_generated(generated_config,tmp_path,mode)


def test_no_executable_candidate_never_calls_model(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'blocked')
    assert outcome=='WAITING_VIEWPOINTS' and not requests and not adapter.calls


def test_scan_rechecks_real_stopped_pose_before_rotating(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'arrived_blocked')
    assert outcome=='WAITING_VIEWPOINTS' and len(adapter.calls)==1 and adapter.scans==0
    assert not any(e.get('observation_status')=='observed' for e in events)


def test_image_api_failure_is_not_observed(generated_config,tmp_path):
    from multi_agent_nav.go2_exploration import ExplorationStopped
    with pytest.raises(ExplorationStopped,match='image/API/validation error'):
        run_generated(generated_config,tmp_path,'api')


@pytest.mark.parametrize('angles', [[0.], [0.,1.,2.,3.,4.,5.,6.,7.]])
def test_generated_detection_uses_original_approach(generated_config,tmp_path,angles):
    generated_config['robot_go2']['scan_angles']=angles
    generated_config['robot_go2'].update(approach_on_detection=True,standoff_distance=.9,approach_distance_margin=.03,
        arrival_config_files=[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')])
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'found')
    assert outcome=='TARGET_APPROACHED' and len(requests)==1
    assert [e['goal_kind'] for e in events if e['state']=='NAVIGATING']==['VIEWPOINT_NAV','PICKUP_NAV']
    if len(angles)>1:
        assert not any(e.get('observation_status') in ('observed','observation_failed') for e in events)
        assert next(e for e in events if e['state']=='SCAN_RESULT')['scan_complete'] is False


def test_partial_api_error_keeps_valid_target_without_completing_observation(generated_config,tmp_path):
    generated_config['robot_go2'].update(approach_on_detection=True,standoff_distance=.9,approach_distance_margin=.03,
        arrival_config_files=[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')])
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'partial_detection')
    assert outcome=='TARGET_APPROACHED' and len(requests)==1
    rows=[e for e in events if e['state']=='OBSERVATION_RECORD']
    assert rows[-1]['observation_status']=='observation_failed'
    assert not any(e.get('observation_status')=='observed' for e in rows)


def test_candidate_api_sends_only_two_images_ids_and_history(monkeypatch,tmp_path):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    client=ABCClient.__new__(ABCClient);client.client=S(api_key='secret')
    images=[tmp_path/'occupancy.png',tmp_path/'semantic.png']
    for path in images:path.write_bytes(b'png')
    captured=[]
    def completion(self,messages,max_tokens):
        captured.extend(messages)
        return S(choices=[S(message=S(content='{"candidate_id":"P1","reason":"side"}'))])
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',completion)
    reply=client.choose_candidate(dict(images=list(map(str,images)),task_id='task',decision_id='decision',
        main_target='target',candidates=[dict(display_id='P1',id='internal',pose=[1.,2.,3.])],
        furniture=[dict(id='table',polygon=[[1.,2.],[3.,4.]])],history=[],records={'internal':{'pose':[1,2,3]}}))
    assert reply['candidate_id']=='P1'
    assert len([c for c in captured[1]['content'] if c['type']=='image_url'])==2
    text=captured[1]['content'][0]['text']
    assert 'internal' not in text and 'pose' not in text and 'polygon' not in text
    assert json.loads(text)['allowed_candidate_ids']==['P1']
    assert 'optional furniture' in json.loads(text)['images'][1]
    assert '家具标注可以为空' in captured[0]['content']


@pytest.mark.parametrize('raw',[
    '{"candidate_id":"P13","reason":"x"}',
    '{"candidate_id":"P1","reason":"x","x":1}',
    '{"candidate_id":"P1","candidate_id":"P2","reason":"x"}',
    '{"candidate_id":["P1"],"reason":"x"}', 'null', 'bad',
])
def test_candidate_parser_rejects_extra_fields_duplicate_and_foreign_id(raw):
    from multi_agent_nav.generated_candidates import parse_candidate_selection
    with pytest.raises(ValueError):parse_candidate_selection(raw,['P1','P2'])


@pytest.mark.parametrize('reason,expected',[
    ('视野开阔','视野开阔'),
    ('12345678901234567890','12345678901234567890'),
    ('123456789012345678901','12345678901234567890'),
    ('一二三四五六七八九十一二三四五六七八九十尾','一二三四五六七八九十一二三四五六七八九十'),
    ('  P1视野开阔\n','P1视野开阔'),
    ('',''),
])
def test_candidate_reason_is_limited_without_changing_selected_id(reason,expected):
    from multi_agent_nav.generated_candidates import parse_candidate_selection
    reply=parse_candidate_selection(json.dumps(dict(candidate_id='P1',reason=reason)),['P1'])
    assert reply==dict(candidate_id='P1',reason=expected)


@pytest.mark.parametrize('cost',[253,254,255])
def test_global_rotation_space_rejects_side_obstacle_and_stale_grid(cost):
    from test_field_preparation import adapter,costmap
    from multi_agent_nav.models import Pose2D
    a=adapter();a.on_costmap(costmap())
    grid=costmap();grid.header.frame_id='map'
    a.on_costmap(grid,global_goal=True)
    assert a.observation_candidate_free(Pose2D(3.,0.))  # outside local coverage is irrelevant
    grid.data[43*80+70]=cost  # y=.35, inside scan disk but outside parked body y=.18
    assert a.costmap_candidate_free(Pose2D(3.,0.),global_goal=True)
    assert a.observation_candidate_free(Pose2D(3.,0.)) == (cost==253)
    grid.data[43*80+70]=0
    assert not a.observation_candidate_free(Pose2D(3.8,0.))
    a.node.get_clock=lambda:S(now=lambda:S(nanoseconds=12_000_000_000))
    assert not a.observation_candidate_free(Pose2D(3.,0.))


def test_repeated_geometry_disabled_furniture_and_map_change(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    options=generated_config['robot_go2']['generated_candidates']
    path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
    doc['furniture'].append(dict(doc['furniture'][0],id='duplicate_geometry'))
    doc['furniture'].append(dict(doc['furniture'][0],id='disabled',enabled=False))
    path.write_text(yaml.safe_dump(doc))
    catalog=GeneratedCandidates(generated_config)
    assert len([p for p in catalog.points if p['furniture_id']])==4 and catalog.unchanged()
    assert len({p['id'] for p in catalog.points})==len(catalog.points)
    path.write_text(path.read_text()+'\n')
    assert not catalog.unchanged()


def test_cap_twelve_and_no_reselect_of_observed(generated_config,tmp_path):
    options=generated_config['robot_go2']['generated_candidates']
    path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
    doc['furniture']=[]
    for i,(x,y) in enumerate([(-2,-2),(2,-2),(-2,2),(2,2)]):
        doc['furniture'].append(dict(id=f't{i}',category='table',
            polygon=[[x-.4,y-.4],[x+.4,y-.4],[x+.4,y+.4],[x-.4,y+.4]]))
    path.write_text(yaml.safe_dump(doc))
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    assert outcome=='EXHAUSTED' and adapter.scans==len(GeneratedCandidates(generated_config).points)
    assert len(requests[0]['candidates'])==12
    seen=set()
    for request in requests:
        assert len(request['candidates'])<=12
        assert not seen.intersection(p['id'] for p in request['candidates'])
        seen.add(request['candidates'][0]['id'])


def test_generated_strategy_requires_real_abc_descriptions(generated_config):
    from multi_agent_nav.go2_exploration import exploration_gates
    generated_config['scene']['targets']['B']='现场填写B'
    assert 'generated scans require measured A/B/C target descriptions' in exploration_gates(generated_config)


def test_offline_generated_entry_does_not_initialize_ros(monkeypatch,capsys,generated_config):
    from multi_agent_nav import go2_check
    monkeypatch.setattr(go2_check,'load_config',lambda *_:generated_config)
    monkeypatch.setattr(go2_check,'live_gates',lambda *_:[])  # fixture has no peer/alignment certification
    monkeypatch.setattr(go2_check,'run_ros',lambda *_:pytest.fail('offline initialized ROS'))
    assert go2_check.main(['--offline','--strategy','generated_candidates_vlm'])==0
    report=json.loads(capsys.readouterr().out)
    assert sum(bool(p['furniture_id']) for p in report['semantic_search']['points'])==4
    assert any(not p['furniture_id'] for p in report['semantic_search']['points']) and not report['motion_sent']


@pytest.mark.parametrize('fault', ['footprint', 'stopped', 'terminal'])
def test_motion_prerequisites_rechecked_after_final_map_worker(generated_config,tmp_path,monkeypatch,fault):
    from dataclasses import replace
    from test_go2_exploration import LocalFixture
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    from multi_agent_nav.go2_exploration import ExplorationStopped
    created=[]
    initialize=LocalFixture.__init__
    def initialize_and_capture(adapter):
        initialize(adapter);created.append(adapter)
    monkeypatch.setattr(LocalFixture,'__init__',initialize_and_capture)
    unchanged=GeneratedCandidates.unchanged
    checks=[0]
    def change_during_last_check(geometry):
        checks[0]+=1
        if checks[0]==3:
            adapter=created[0]
            if fault=='footprint':
                adapter.costmap_candidate_free=lambda *args,**kwargs:False
                adapter.observation_candidate_free=lambda *args,**kwargs:False
            else:
                get_health=adapter.get_health
                field='stopped' if fault=='stopped' else 'motion_terminal'
                adapter.get_health=lambda:replace(get_health(),**{field:False})
        return unchanged(geometry)
    monkeypatch.setattr(GeneratedCandidates,'unchanged',change_during_last_check)
    try:
        run_generated(generated_config,tmp_path)
    except ExplorationStopped:
        pass
    assert checks[0]>=3 and not created[0].calls


@pytest.mark.parametrize('cancel', [False, True])
def test_new_task_yields_until_previous_worker_finishes(generated_config,tmp_path,cancel):
    import threading
    import time
    from multi_agent_nav.adapters.go2 import AsyncVision
    from multi_agent_nav.frame_adapter import LocalFrame
    from multi_agent_nav.go2_exploration import explore_steps
    from test_go2_exploration import LocalFixture
    adapter=LocalFixture();adapter.vision=AsyncVision(10.)
    adapter.selection_evidence_dir=tmp_path/'selection'
    adapter.client=S(choose_candidate=lambda snapshot:dict(candidate_id='P1',reason='offline'))
    release=threading.Event();started=threading.Event()
    def old_work():
        started.set();assert release.wait(2.)
        return 'old-task-result'
    previous=adapter.vision.submit(old_work)
    assert started.wait(1.)
    adapter.vision.cancel()
    assert previous.cancelled()
    events=[]
    steps=explore_steps(adapter,generated_config,LocalFrame(),600.,'target',events.append,
        lambda:[],lambda:True,lambda:None,clock=lambda:0.,task_id='new-task')
    try:
        for _ in range(20):
            adapter.vision.poll();next(steps)
        assert not adapter.calls and not events
        if cancel:
            steps.close()
        release.set();adapter.vision.job.result(timeout=2.)
        adapter.vision.poll()
        if not cancel:
            for _ in range(300):
                adapter.vision.poll();next(steps)
                if any(e.get('state')=='CANDIDATE_SELECTED' for e in events):break
                time.sleep(.001)
            assert any(e.get('state')=='CANDIDATE_SELECTED' and e['task_id']=='new-task' for e in events)
        else:
            assert not adapter.calls and not events
    finally:
        release.set();steps.close();adapter.vision.close();adapter.vision.executor.shutdown(wait=True)


@pytest.mark.parametrize('block', [0, 254, 255, 'furniture'])
@pytest.mark.parametrize('distance,edge,far', [(2.,-1.,-.99),(2.5,-.5,-.49)])
def test_coverage_uses_configured_range_and_preserves_occluded_points(generated_config, block, distance, edge, far):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    from multi_agent_nav.models import Pose2D
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = distance
    catalog = GeneratedCandidates(generated_config)
    catalog.raw[:] = 0
    catalog.furniture = []
    catalog.points = [dict(id=pid, pose=pose) for pid, pose in (
        ('near', [-2., -3., 0.]), ('edge', [edge, -3., 0.]),
        ('far', [far, -3., 0.]), ('behind', [-3., -1.5, 0.]))]
    if block == 'furniture':
        catalog.furniture = [dict(polygon=[[-3.2,-2.4],[-2.8,-2.4],[-2.8,-2.2],[-3.2,-2.2]])]
    else:
        catalog.raw[27:29, 19:22] = block
    covered = catalog.covered_points(Pose2D(-3., -3.), [-i*math.pi/4 for i in range(1,9)])
    assert set(covered) == ({'near', 'edge', 'behind'} if block == 0 else {'near', 'edge'})
    assert not catalog.covered_points(Pose2D(-3., -3.), [0.])


@pytest.mark.parametrize('value', [-1., float('nan'), True])
def test_coverage_range_requires_finite_nonnegative_distance(generated_config, value):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = value
    with pytest.raises(ValueError, match='coverage_range'):
        GeneratedCandidates(generated_config)


def test_complete_scan_excludes_nearby_candidates_from_future_decisions(generated_config, tmp_path):
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = 2.
    generated_config['robot_go2']['scan_angles'] = [-i*math.pi/4 for i in range(1,9)]
    adapter, events, requests, outcome = run_generated(generated_config, tmp_path)
    assert outcome == 'EXHAUSTED'
    covered = {pid for e in events if e['state'] == 'OBSERVATION_RECORD'
               for pid, row in e['records'].items() if row['status'] == 'covered'}
    assert covered
    for request in requests:
        assert all(request['records'][p['id']]['status'] != 'covered'
                   for p in request['candidates'])
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    assert adapter.scans < len(GeneratedCandidates(generated_config).points)


@pytest.mark.parametrize('mode', ['found', 'partial_detection'])
def test_early_detection_does_not_cover_unscanned_nearby_points(generated_config, tmp_path, mode):
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = 2.
    generated_config['robot_go2']['scan_angles'] = [-i*math.pi/4 for i in range(1,9)]
    _, events, _, outcome = run_generated(generated_config, tmp_path, mode)
    assert outcome == 'TARGET_FOUND'
    assert not any(row['status'] == 'covered' for e in events if e['state'] == 'OBSERVATION_RECORD'
                   for row in e['records'].values())


def test_coverage_is_disabled_without_go2_setting(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    from multi_agent_nav.models import Pose2D
    angles = [-i*math.pi/4 for i in range(1,9)]
    assert not GeneratedCandidates(generated_config).covered_points(Pose2D(-3., -3.), angles)
    robot = generated_config['robot_go2']
    robot['generated_candidates']['coverage_range'] = 2.
    generated_config['robot_ranger'] = robot
    generated_config['alignment']['robots']['ranger'] = generated_config['alignment']['robots']['go2']
    robot['generated_candidates']['semantic_yaml'] = ''
    assert not GeneratedCandidates(generated_config, 'ranger').covered_points(Pose2D(-3., -3.), angles)


def test_coverage_keeps_point_when_ray_grazes_unknown_cell(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    from multi_agent_nav.models import Pose2D
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = 2.
    catalog = GeneratedCandidates(generated_config)
    catalog.raw[:] = 0
    catalog.furniture = []
    catalog.points = [dict(id='behind', pose=[-3.,-1.5,0.])]
    catalog.raw[27,19] = 255  # Ray x=-3 lies on this cell's right boundary.
    assert not catalog.covered_points(Pose2D(-3.,-3.), [-i*math.pi/4 for i in range(1,9)])


def test_coverage_records_actual_stop_instead_of_commanded_goal(generated_config, tmp_path, monkeypatch):
    from dataclasses import replace
    from multi_agent_nav.models import Pose2D
    from test_go2_exploration import LocalFixture
    get_health = LocalFixture.get_health
    def offset_health(adapter):
        health = get_health(adapter)
        if adapter.calls and health.stopped:
            health = replace(health, pose=Pose2D(health.pose.x+.3,health.pose.y,health.pose.yaw))
        return health
    monkeypatch.setattr(LocalFixture, 'get_health', offset_health)
    generated_config['robot_go2']['generated_candidates']['coverage_range'] = 2.
    generated_config['robot_go2']['scan_angles'] = [-i*math.pi/4 for i in range(1,9)]
    adapter, events, _, _ = run_generated(generated_config, tmp_path)
    observed = next(row for e in events if e['state'] == 'OBSERVATION_RECORD'
                    for row in e['records'].values() if row['status'] == 'observed')
    assert observed['scan_pose']['x'] == pytest.approx(adapter.calls[0][1].x+.3)
