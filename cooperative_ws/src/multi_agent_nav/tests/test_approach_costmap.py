"""Approach footprint safety with a bounded local window; no ROS initialization."""
import math
from concurrent.futures import Future

import pytest

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.models import ApproachResult, Observation, PathResult, Pose2D
from test_field_preparation import adapter, costmap, stamp


def maps():
    a = adapter()
    local = costmap()
    local.metadata.size_x = local.metadata.size_y = 30
    local.metadata.origin.position.x = local.metadata.origin.position.y = -1.5
    local.data = [0] * 900
    global_grid = costmap()
    global_grid.header.frame_id = 'map'
    a.on_costmap(local)
    a.on_costmap(global_grid, global_goal=True)
    return a, local, global_grid


def test_far_approach_enters_planning_with_all_eight_original_candidates():
    a, _, _ = maps()
    planned, events = [], []
    a.record_event = events.append
    def plan(goal):
        planned.append(goal)
        return completed(PathResult(True, [Pose2D(), goal]))
    a.compute_path = plan
    target = Pose2D(.744896, 2.911190)
    result = a.select_approach(target, [], standoff_distance=.53).result()
    assert result.goal is not None and planned == [result.goal]
    checks = [e for e in events if e['event'] == 'APPROACH_FOOTPRINT_CHECK']
    assert len({tuple(e['candidate_pose']) for e in checks}) == 8
    assert all(e['global_check']['status'] == 'free' for e in checks)
    assert all(e['local_check']['status'] == 'coverage_insufficient' for e in checks)
    assert all(e['accepted'] for e in checks)
    assert math.hypot(result.goal.x-target.x, result.goal.y-target.y) == pytest.approx(.53)


@pytest.mark.parametrize('value', [253, 254, 255])
def test_global_footprint_distinguishes_inscribed_from_lethal_and_unknown(value):
    a, _, global_grid = maps()
    # Center stays free; only lethal/unknown body cells veto under the revised contract.
    global_grid.data[40*80+58] = value
    a.on_costmap(global_grid, global_goal=True)
    assert a.approach_candidate_free(Pose2D(1.6, 0.)) == (value==253)
    assert a.costmap_candidate_check(Pose2D(1.6, 0.), global_goal=True)['status'] == ('free' if value==253 else 'collision')


@pytest.mark.parametrize('value', [253, 254, 255])
def test_partial_local_footprint_keeps_collision_and_unknown_veto(value):
    a, local, _ = maps()
    # Candidate center is beyond x=1.5; its rear body still overlaps this cell.
    local.data[15*30+29] = value
    a.on_costmap(local)
    goal = Pose2D(1.6, 0.)
    assert a.costmap_candidate_free(goal, global_goal=True)
    assert a.approach_candidate_free(goal) == (value==253)
    assert a.costmap_candidate_check(goal)['status'] == ('coverage_insufficient' if value==253 else 'collision')


@pytest.mark.parametrize('x,status', [(0., 'free'), (1.6, 'coverage_insufficient'), (3., 'coverage_insufficient')])
def test_local_coverage_is_explicit_and_legacy_parking_stays_strict(x, status):
    a, _, _ = maps()
    goal = Pose2D(x, 0.)
    assert a.approach_candidate_free(goal)
    assert a.costmap_candidate_check(goal)['status'] == status
    assert a.costmap_candidate_free(goal) == (status == 'free')


def test_global_out_of_bounds_cannot_be_treated_as_local_coverage():
    a, _, _ = maps()
    goal = Pose2D(4., 0.)
    assert not a.approach_candidate_free(goal)
    assert a.costmap_candidate_check(goal, global_goal=True)['status'] == 'out_of_bounds'


@pytest.mark.parametrize('global_goal', [False, True], ids=['local', 'global'])
@pytest.mark.parametrize('fault', ['missing', 'payload', 'stamp', 'receipt', 'tf'])
def test_bad_map_or_tf_is_an_explicit_failure_not_coverage(global_goal, fault):
    a, local, global_grid = maps()
    grid = global_grid if global_goal else local
    if fault == 'missing':
        if global_goal:a.global_costmap = None
        else:a.costmap = None
    elif fault == 'receipt':
        if global_goal:a.global_costmap_at -= 2.
        else:a.costmap_at -= 2.
    else:
        if fault == 'payload':grid.data = []
        if fault == 'stamp':grid.header.stamp = stamp(1.)
        if fault == 'tf':
            grid.header.frame_id = 'missing_frame'
            a.node.tf_buffer.lookup_transform = lambda *args: (_ for _ in ()).throw(RuntimeError('missing TF'))
            (local if global_goal else global_grid).header.frame_id = 'map'
        a.on_costmap(grid, global_goal=global_goal)
    check = a.costmap_candidate_check(Pose2D(3., 0.), global_goal=global_goal)
    assert check['status'] == 'unavailable'
    with pytest.raises(RuntimeError, match=('global' if global_goal else 'local') + ' costmap unavailable'):
        a.approach_candidate_free(Pose2D(3., 0.))
    planned = []
    a.compute_path = lambda goal: planned.append(goal)
    result = a.select_approach(Pose2D(3., 0.), [], standoff_distance=.53).result()
    assert result.goal is None and 'costmap unavailable' in result.reason and not planned


def test_partial_check_transforms_pose_and_rotated_grid_before_testing_body():
    from types import SimpleNamespace as S
    a, local, _ = maps()
    local.metadata.origin.position.x = 1.5
    local.metadata.origin.orientation.z = math.sin(math.pi/4)
    local.metadata.origin.orientation.w = math.cos(math.pi/4)
    requested = []
    def transform(target, source, when):
        requested.append((target, source, when.nanoseconds))
        return S(transform=S(translation=S(x=0., y=0.),
            rotation=S(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))))
    a.node.tf_buffer.lookup_transform = transform
    a.on_costmap(local)
    goal = Pose2D(1.6, 0.)
    assert a.approach_candidate_free(goal)
    assert ('odom', 'map', 10_000_000_000) in requested
    local.data[15*30+29] = 254
    a.on_costmap(local)
    assert not a.approach_candidate_free(goal)


def test_global_complete_footprint_uses_its_frame_and_rotated_origin():
    from types import SimpleNamespace as S
    a, local, _ = maps()
    local.header.frame_id = 'map'
    global_grid = costmap(yaw=math.pi/2)
    global_grid.header.frame_id = 'world'
    global_grid.metadata.origin.position.x = 4.
    requested = []
    def transform(target, source, when):
        requested.append((target, source, when.nanoseconds))
        return S(transform=S(translation=S(x=1., y=2.),
            rotation=S(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))))
    a.node.tf_buffer.lookup_transform = transform
    a.on_costmap(global_grid, global_goal=True)
    goal = Pose2D(1.6, 0.)
    assert a.approach_candidate_free(goal)
    global_grid.data[30*80+78] = 254
    a.on_costmap(global_grid, global_goal=True)
    assert not a.approach_candidate_free(goal)
    assert ('world', 'map', 10_000_000_000) in requested


@pytest.mark.parametrize('goal', [Pose2D(float('nan'), 0.), Pose2D(0., float('inf')), Pose2D(0., 0., float('nan'))])
def test_invalid_pose_is_reported_without_nonfinite_json(goal):
    import json
    a, _, _ = maps()
    a.record_event = lambda event: json.dumps(event, allow_nan=False)
    with pytest.raises(RuntimeError, match='invalid candidate pose'):
        a.approach_candidate_free(goal)


def test_planning_failure_is_logged_separately_from_coverage():
    a, _, _ = maps()
    events = []
    a.record_event = events.append
    a.compute_path = lambda goal: completed(PathResult(False, reason='fixture planner failure'))
    assert a.select_approach(Pose2D(.75, 2.91), [], standoff_distance=.53).result().goal is None
    failures = [e for e in events if e['event'] == 'APPROACH_PATH_CHECK']
    assert len(failures) == 8
    assert all(not e['reachable'] and e['reason'] == 'fixture planner failure' for e in failures)


@pytest.mark.parametrize('global_goal', [False, True], ids=['local', 'global'])
def test_collision_appearing_during_planning_still_prevents_selection(global_goal):
    a, local, global_grid = maps()
    pending = Future()
    a.compute_path = lambda goal: pending
    result = a.select_approach(Pose2D(), [], standoff_distance=.53)
    assert not result.done()
    grid = global_grid if global_goal else local
    grid.data = [254] * len(grid.data)
    a.on_costmap(grid, global_goal=global_goal)
    pending.set_result(PathResult(True, [Pose2D()]))
    assert result.result().goal is None


@pytest.mark.parametrize('global_goal', [False, True], ids=['local', 'global'])
def test_map_loss_during_planning_remains_an_explicit_input_failure(global_goal):
    a, _, _ = maps()
    pending = Future()
    a.compute_path = lambda goal: pending
    result = a.select_approach(Pose2D(), [], standoff_distance=.53)
    assert not result.done()
    if global_goal:a.global_costmap = None
    else:a.costmap = None
    pending.set_result(PathResult(True, [Pose2D()]))
    assert result.result().goal is None
    assert ('global' if global_goal else 'local') + ' costmap unavailable' in result.result().reason


@pytest.mark.parametrize('global_goal', [False, True], ids=['local', 'global'])
@pytest.mark.parametrize('fault', ['missing', 'tf'])
def test_last_planner_failure_does_not_hide_input_loss(global_goal, fault):
    import numpy as np
    a, local, global_grid = maps()
    excluded = [Pose2D(.53*math.cos(angle), .53*math.sin(angle),
        math.atan2(math.sin(angle+math.pi), math.cos(angle+math.pi)))
        for angle in np.linspace(-math.pi, math.pi, 8, endpoint=False)[1:]]
    pending = Future()
    a.compute_path = lambda goal: pending
    result = a.select_approach(Pose2D(), excluded, standoff_distance=.53)
    assert not result.done()
    if fault == 'missing':
        if global_goal:a.global_costmap = None
        else:a.costmap = None
    else:
        grid = global_grid if global_goal else local
        grid.header.frame_id = 'missing_frame'
        (local if global_goal else global_grid).header.frame_id = 'map'
        def missing_tf(*args):raise RuntimeError('missing TF')
        a.node.tf_buffer.lookup_transform = missing_tf
    pending.set_result(PathResult(False, reason='fixture planner failure'))
    assert ('global' if global_goal else 'local') + ' costmap unavailable' in result.result().reason


def test_exploration_rechecks_far_requested_goal_and_actual_endpoint_without_local_veto():
    from test_go2_exploration import LocalFixture, run_case
    a, _, global_grid = maps()
    global_grid.metadata.size_x = global_grid.metadata.size_y = 120
    global_grid.metadata.origin.position.x = global_grid.metadata.origin.position.y = -6.
    global_grid.data = [0] * 14400
    a.on_costmap(global_grid, global_goal=True)
    fixture = LocalFixture()
    fixture.costmap_candidate_free = a.costmap_candidate_free
    fixture.approach_candidate_free = lambda goal: a.approach_candidate_free(goal)
    fixture.start_scan = lambda *args, **kwargs: completed([Observation(target_id='A', status='DETECTED',
        position_valid=True, common_position=Pose2D(5., 2.))])
    fixture.select_approach = lambda *args, **kwargs: completed(ApproachResult(Pose2D(4.5, 2.)))
    events = []
    assert run_case(fixture, events, resources={}, approach=True) == 'TARGET_APPROACHED'
    assert [c[0] for c in fixture.calls] == ['VIEWPOINT_NAV', 'PICKUP_NAV']
    assert any(e['state'] == 'ARRIVAL_VERIFIED' for e in events)


@pytest.mark.parametrize('global_goal', [False, True])
def test_body_edge_inscribed_cost_is_not_a_second_inflation(global_goal):
    a, local, grid = maps()
    if global_goal:
        grid.data[40*80+58] = 253
    else:
        local.data[15*30+29] = 253
    check = a.costmap_candidate_check(Pose2D(1.6, 0.), global_goal)
    assert check['status'] == ('free' if global_goal else 'coverage_insufficient')
    assert a.approach_candidate_free(Pose2D(1.6, 0.))


@pytest.mark.parametrize('value', [253, 254, 255])
def test_center_cost_rejected_for_body_and_rotation(value):
    a, local, grid = maps()
    local.data[15*30+15] = grid.data[40*80+40] = value
    assert a.costmap_candidate_check(Pose2D())['status'] == 'collision'
    assert a.costmap_candidate_check(Pose2D(), True)['status'] == 'collision'
    assert not a.observation_candidate_free(Pose2D())


@pytest.mark.parametrize('offset', [1, 4], ids=['disk_interior', 'intersecting_edge'])
@pytest.mark.parametrize('value', [253, 254, 255])
def test_rotation_disk_retains_full_area_check_without_double_inflation(offset, value):
    a, _, grid = maps()
    grid.data[(40+offset)*80+40] = value
    assert a.observation_candidate_free(Pose2D()) == (value == 253)


@pytest.mark.parametrize('pose', [Pose2D(float('nan'), 0.), Pose2D(0., float('inf'))])
def test_invalid_rotation_pose_cannot_pass(pose):
    a, _, _ = maps()
    assert not a.observation_candidate_free(pose)
