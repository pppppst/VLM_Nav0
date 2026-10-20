from concurrent.futures import Future

import pytest

from test_go2_exploration import LocalFixture, run_case
from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.models import ApproachResult, Observation, Pose2D


@pytest.mark.parametrize('radius,budget,outcome', [
    (.81, .53, 'APPROACH_DISTANCE_FAILED'), (1., .72, 'TARGET_APPROACHED')])
def test_entry_radius_controls_budget_and_actual_stopped_distance(radius, budget, outcome):
    adapter = LocalFixture()
    events, radii = [], []
    adapter.start_scan = lambda *args, **kwargs: completed([Observation(target_id='A',
        status='DETECTED', position_valid=True, common_position=Pose2D(5., 2.))])
    def select(obj, excluded, *, standoff_distance, path_accept):
        radii.append(standoff_distance)
        return completed(ApproachResult(Pose2D(5.-standoff_distance, 2.)))
    adapter.select_approach = select
    advance = adapter.advance
    def overshoot(now):
        finishing = any(now >= value[0] and value[2].goal_kind == 'PICKUP_NAV'
                        for value in adapter.active.values())
        advance(now)
        if finishing:
            adapter.pose = Pose2D(4.1, 2.)
    adapter.advance = overshoot
    assert run_case(adapter, events, resources={}, approach=True, success_radius=radius,
                    require_navigation_success=True) == outcome
    assert radii[0] == pytest.approx(budget)
    verified = [e for e in events if e['state'] == 'ARRIVAL_VERIFIED']
    assert verified[-1]['measured_distance'] == pytest.approx(.90)


def test_pending_plan_yields_without_sending_motion():
    from multi_agent_nav.go2_exploration import explore_steps
    from multi_agent_nav.frame_adapter import FrameAdapter
    adapter = LocalFixture()
    pending = Future()
    adapter.compute_path = lambda goal: pending
    config = {'scene': {'resources': {}, 'viewpoints': {'go2': [dict(id='p', pose=[2.,0.,0.])]}},
              'robot_go2': {'stop_margin': .1}}
    steps = explore_steps(adapter, config, FrameAdapter('m','v','lab_ref','a',[0,0,0]),
        10., 'target', lambda event: None, lambda: [], lambda: True, lambda: None, clock=lambda:0.)
    next(steps)
    next(steps)
    assert not pending.done() and not adapter.calls
    steps.close()
