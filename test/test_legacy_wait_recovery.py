"""Legacy Go2 recovery without ROS initialization, model calls or hardware."""
from concurrent.futures import Future
from types import SimpleNamespace as S

import pytest

pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from vlm_nav.vlm_navigator import APPROACHING, DISARMED, FAILED, SCANNING, SENSOR_WAITING, VLMNavigator


@pytest.fixture
def legacy(monkeypatch):
    now = [10.]
    monkeypatch.setattr('vlm_nav.vlm_navigator.time.monotonic', lambda: now[0])
    n = VLMNavigator.__new__(VLMNavigator)
    n.p = S(wait_recovery=True, require_external_safety_gates=True, external_gate_timeout=2.,
        rgbd_wait_timeout=2., tf_failure_timeout=3., sensor_failure_timeout=30., sensor_recovery_frames=3,
        arrival_stationary_samples=3, arrival_odom_max_age=.5, arrival_linear_speed_tolerance=.03,
        arrival_angular_speed_tolerance=.05, easy_case_mode=True, easy_case_timeout=120.,
        target_confirmation_timeout=20., goal_timeout=120., raw_depth_mode=False, scan_steps=4)
    n.observation_only = False
    n.state = SCANNING
    n.enabled = True
    n.get_parameter = lambda name: S(value=n.enabled if name == 'enabled' else getattr(n.p, name, False))
    def set_parameters(parameters):
        n.enabled = parameters[0].value
        n.state = DISARMED
        return [S(successful=True)]
    n.set_parameters = set_parameters
    n.get_logger = lambda: S(info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None)
    n.set_state = lambda value: setattr(n, 'state', value)
    n.publish_easy_event = lambda *a, **k: None
    n.publish_stop = lambda: None
    n.task_epoch = n.sequence = 1
    n.goal_token = n.plan_token = 0
    n.goal_handle = n.plan_handle = None
    n.goal_pending = n.plan_pending = False
    n.goal_kind = n.plan_kind = None
    n.goal_pose = n.active_motion_origin = None
    n.plan_candidates = []
    n.sensor_wait_started = 0.
    n.sensor_wait_reason = 'none'
    n.sensor_recovery_count = 3
    n.last_valid_rgbd_received = n.last_camera_tf_success = n.last_robot_tf_success = 10.
    n.nav_ready = n.control_armed = n.vlm_api_ready = True
    n.external_gate_receipts = dict(nav_ready=10., control_armed=10.)
    n.latest_snapshot = None
    n.target_reference_position = None
    n.target_tracker = S(reset=lambda: None)
    n.target_confirmation_started = 0.
    n.easy_started = n.enabled_since = 5.
    n.easy_alignment_complete = True
    n.session_origin = None
    n.frontier_generation = 0
    n.scan_headings = [0., 1., 2., 3.]
    n.scan_index = 2
    n.scan_observations = ['complete-0', 'complete-1']
    n.scan_observation_headings = [0., 1.]
    n.scan_waiting_for_vlm = True
    n.scan_settle_until = 0.
    n.scan_retry_count = 0
    n.prepare_stationary_observation = lambda: None
    n.robot_pose = lambda: (0., 0., 0.)
    n.arrival_last_odom_received = 10.
    n.arrival_stationary_count = 3
    n.arrival_linear_speed = n.arrival_angular_speed = 0.
    n.worker = None
    n.grid = n.map_message = object()
    n.initial_costmap_clear_pending = 0
    n.initial_costmap_clear_errors = []
    n.initial_costmap_clear_completed_at = 0.
    n.api_failures = n.rejected_results = 0
    n.last_failure_reason = 'none'
    n.recovery_requests = set()
    n.recovery_results = set()
    n.recovery_hard_fault = ''
    n.control_recovery = S(closed=False, reason='', step=lambda ready: None, cancel=lambda: None)
    return n, now


def refresh(n, now):
    n.last_valid_rgbd_received = n.last_camera_tf_success = n.last_robot_tf_success = now
    n.arrival_last_odom_received = now
    n.external_gate_receipts = dict(nav_ready=now, control_armed=now)
    n.sensor_recovery_count = 3
    n.arrival_stationary_count = 3


def test_go2_external_gate_loss_waits_without_writing_manual_enable(legacy):
    n, _ = legacy
    n.set_parameters = lambda values: pytest.fail('transient gate loss must preserve enabled')
    n.on_external_gate('nav_ready', S(data=False))
    assert n.state == SENSOR_WAITING and n.enabled


def test_go2_sensor_wait_is_unbounded_and_easy_budget_excludes_pause(legacy):
    n, now = legacy
    n.enter_sensor_wait('camera unavailable')
    for value in (50., 700., 1400.):
        now[0] = value
        refresh(n, value)
        n.nav_ready = False
        n.safety_tick()
        assert n.state == SENSOR_WAITING
    n.nav_ready = True
    refresh(n, now[0])
    assert n.recover_from_sensor_wait()
    assert n.easy_started == 1395.
    assert n.state == SCANNING and n.scan_index == 2
    assert n.scan_observations == ['complete-0', 'complete-1']


def test_go2_robot_tf_loss_enters_wait_instead_of_failed(legacy):
    n, now = legacy
    n.robot_pose = lambda: None
    n.last_robot_tf_success = 1.
    n.safety_tick()
    assert n.state == SENSOR_WAITING and n.enabled


@pytest.mark.parametrize('blocked', ['manual', 'moving', 'native_pending', 'external', 'hard_fault'])
def test_resume_requires_original_health_stop_native_terminal_and_authority(legacy, blocked):
    n, _ = legacy
    n.enter_sensor_wait('transient')
    refresh(n, 10.)
    if blocked == 'manual': n.enabled = False
    elif blocked == 'moving': n.arrival_stationary_count = 0
    elif blocked == 'native_pending': n.recovery_results.add(Future())
    elif blocked == 'external': n.nav_ready = False
    else: n.recovery_hard_fault = 'native action identity unknown'
    assert not n.recover_from_sensor_wait()
    assert n.state == SENSOR_WAITING


def test_recovery_preserves_confirmed_target_and_completed_alignment(legacy):
    n, now = legacy
    n.state = APPROACHING
    n.target_reference_position = (2., 1., .5)
    n.enter_sensor_wait('TF unavailable')
    now[0] = 1300.
    refresh(n, now[0])
    assert n.recover_from_sensor_wait()
    assert n.state == APPROACHING and n.target_reference_position == (2., 1., .5)
    assert n.easy_alignment_complete


@pytest.mark.parametrize('reason,waiting', [
    ('Spin action server unavailable', True),
    ('ComputePathToPose action server unavailable', True),
    ('fixture unknown program error', False),
    ('Cannot obtain approach goal result: lost identity', False),
])
def test_only_known_temporary_failure_is_soft(legacy, reason, waiting):
    n, _ = legacy
    n.fail_safe(reason)
    assert n.state == (SENSOR_WAITING if waiting else FAILED)


@pytest.mark.parametrize('kind', ['goal', 'plan'])
def test_late_native_accept_is_cancelled_and_only_terminal_result_unblocks_recovery(legacy, kind):
    n, _ = legacy
    accepted, terminal = Future(), Future()
    cancelled = []
    handle = S(accepted=True, cancel_goal_async=lambda: cancelled.append(True), get_result_async=lambda: terminal)
    n.recovery_requests.add(accepted)
    n.enter_sensor_wait('nav unavailable')
    accepted.set_result(handle)
    if kind == 'goal':n.on_goal_response(accepted, 0, 'scan')
    else:n.on_plan_response(accepted, 0)
    refresh(n, 10.)
    assert cancelled and n.recovery_results == {terminal}
    assert not n.recovery_ready()
    terminal.set_result(S(status=GoalStatus.STATUS_CANCELED))
    assert n.recovery_ready()


@pytest.mark.parametrize('kind', ['goal', 'plan'])
def test_native_response_exception_never_infers_goal_was_not_sent(legacy, kind):
    n, _ = legacy
    accepted = Future()
    n.recovery_requests.add(accepted)
    n.enter_sensor_wait('nav unavailable')
    accepted.set_exception(RuntimeError('connection lost after send'))
    if kind == 'goal':n.on_goal_response(accepted, 0, 'scan')
    else:n.on_plan_response(accepted, 0)
    assert n.recovery_hard_fault
    refresh(n, 10.)
    assert not n.recover_from_sensor_wait()


def test_api_error_keeps_enabled_and_schedules_a_real_preflight_retry(legacy):
    n, _ = legacy
    n.p.api_failure_limit = 3
    n.api_failures = 2
    n.set_vlm_api_ready(False, 'API unavailable')
    assert n.enabled and n.state == SENSOR_WAITING
    assert n.api_preflight_pending and not n.vlm_api_ready


def test_observation_only_cannot_enable_this_nodes_recovery_controller(legacy):
    n, _ = legacy
    n.observation_only = True
    assert not n.wait_recovery_enabled()


def test_waiting_bridge_recovery_excludes_only_control_armed_and_requires_real_stop(legacy):
    n, _ = legacy
    ready = []
    n.control_recovery.step = ready.append
    n.control_armed = False
    n.enter_sensor_wait('control lost')
    refresh(n, 10.)
    n.safety_tick()
    assert ready == [True] and n.state == SENSOR_WAITING
    n.arrival_stationary_count = 0
    n.safety_tick()
    assert ready == [True, False]


def test_status_file_has_wait_reason_and_diagnostic_io_cannot_break_wait(legacy):
    n, _ = legacy
    values = []
    n.experiment_status = S(update=lambda **kw: values.append(kw))
    n.enter_sensor_wait('camera unavailable')
    n.publish_recovery_status(10.)
    assert values[-1]['state'] == SENSOR_WAITING
    assert values[-1]['reason_class'] == 'dependency_wait'
    assert values[-1]['reason'] == 'camera unavailable'
    n.experiment_status.update = lambda **kw: (_ for _ in ()).throw(OSError('readonly diagnostics'))
    n.publish_recovery_status(10.)
    assert n.state == SENSOR_WAITING


def test_rejected_mode_parameter_does_not_recreate_cancelled_recovery_client(legacy):
    from rclpy.parameter import Parameter
    n, _ = legacy
    n.control_recovery.closed = True
    n.destroy_subscription = lambda _: pytest.fail('rejected parameters must have no recovery side effects')
    result = n.on_parameters_changed([Parameter('easy_case_mode', value=True)])
    assert not result.successful


def test_missing_map_wait_does_not_resume_until_map_returns(legacy):
    n, _ = legacy
    n.grid = n.map_message = None
    n.navigation_tick()
    assert n.state == SENSOR_WAITING
    refresh(n, 10.)
    assert not n.recover_from_sensor_wait()
    n.grid = n.map_message = object()
    assert n.recover_from_sensor_wait()


def test_initial_costmap_wait_keeps_driving_existing_clear_until_refreshed(legacy):
    n, _ = legacy
    n.grid = n.map_message = object()
    n.initial_costmap_clear_required = True
    ready = [False]
    calls = []
    n.ensure_initial_costmap_clear_done = lambda: calls.append(True) or ready[0]
    n.navigation_tick()
    assert n.state == SENSOR_WAITING
    refresh(n, 10.)
    assert not n.recover_from_sensor_wait()
    ready[0] = True
    assert n.recover_from_sensor_wait()
    assert len(calls) >= 3


@pytest.mark.parametrize('kind', ['navigation', 'spin', 'plan'])
def test_send_exception_is_reported_as_nonresumable_identity_gap(legacy, kind):
    from geometry_msgs.msg import PoseStamped
    n, _ = legacy
    def unavailable(goal):raise RuntimeError('lost connection during native send')
    client = S(wait_for_server=lambda **kw: True, send_goal_async=unavailable)
    n.make_pose_stamped = lambda *args: PoseStamped()
    n.navigator = n.spinner = n.path_planner = client
    n.p.spin_time_allowance = 20.
    n.plan_attempt_count = 0
    if kind == 'navigation':n.send_navigation_goal(1., 0., 0., 'approach')
    elif kind == 'spin':n.send_spin(.1)
    else:
        n.plan_candidates = [(1., 0., 0.)]
        n.request_next_plan(0)
    assert n.recovery_hard_fault and n.state == FAILED


def test_manual_parameter_disable_cancels_recovery_and_never_auto_enables(legacy):
    from rclpy.parameter import Parameter
    n, _ = legacy
    cancelled = []
    n.control_recovery.cancel = lambda: cancelled.append(True)
    n.initial_costmap_clear_token = 0
    n.reset_task = n.set_state
    n.publish_vlm_enabled = lambda: None
    n.enter_sensor_wait('temporary')
    assert n.on_parameters_changed([Parameter('enabled', value=False)]).successful
    n.enabled = False
    refresh(n, 10.)
    n.safety_tick()
    assert cancelled == [True] and n.state == DISARMED
    assert not n.recover_from_sensor_wait()


def test_bridge_hard_diagnostic_cannot_resume_even_before_armed_bool_updates(legacy):
    n, _ = legacy
    n.control_recovery.closed = True
    n.control_recovery.reason = ''
    n.control_recovery.hard_fault = 'bridge hard fault: FOREIGN_CONTROL'
    n.enter_sensor_wait('control lost')
    refresh(n, 10.)
    assert not n.recover_from_sensor_wait()
    n.safety_tick()
    assert n.recovery_hard_fault == 'bridge hard fault: FOREIGN_CONTROL'


def test_lost_costmap_service_reply_is_bounded_and_late_reply_cannot_finish_new_query(legacy):
    n, _ = legacy
    n.p.initial_costmap_refresh_timeout = 3.
    n.initial_costmap_clear_required = n.initial_costmap_clear_requested = True
    n.initial_costmap_clear_pending = 1
    n.initial_costmap_clear_token = 7
    n.initial_costmap_clear_query_started = 1.
    pending = [Future(), Future()]
    n.initial_costmap_clear_futures = pending.copy()
    for future in pending:future.add_done_callback(lambda done: n.on_initial_costmap_clear_result(done, 'global', 7))
    assert not n.ensure_initial_costmap_clear_done()
    assert n.state == SENSOR_WAITING
    assert n.initial_costmap_clear_token == 8 and n.initial_costmap_clear_pending == 0
    assert all(future.cancelled() for future in pending)
    n.on_initial_costmap_clear_result(pending[0], 'global', 7)
    assert n.initial_costmap_clear_pending == 0


def test_accepted_handle_with_missing_result_rpc_is_still_cancelled_and_never_reports_terminal(legacy):
    n, _ = legacy
    cancelled = []
    def broken_result():raise RuntimeError('result service unavailable')
    handle = S(accepted=True, get_result_async=broken_result, cancel_goal_async=lambda: cancelled.append(True))
    accepted = Future();accepted.set_result(handle)
    n.recovery_requests.add(accepted)
    n.on_goal_response(accepted, 0, 'scan')
    assert cancelled and n.recovery_hard_fault
    records = []
    n.experiment_status = S(update=lambda **kw: records.append(kw))
    n.publish_recovery_status(10.)
    assert records[-1]['health']['motion_terminal'] is False


def test_malformed_plan_result_cannot_be_retried_as_unreachable_candidate(legacy):
    n, _ = legacy
    n.current_plan_pose = None
    n.plan_kind = 'approach'
    future = Future();future.set_exception(TypeError('bad action payload'))
    n.on_plan_result(future, 0)
    assert n.state == FAILED and n.recovery_hard_fault


def test_client_initialization_program_error_is_not_an_api_retry(legacy, monkeypatch):
    n, _ = legacy
    n.p.vlm_timeout = 20.
    n.p.image_detail = 'high'
    n.p.jpeg_quality = 85
    n.p.api_failure_limit = 3
    def broken(**kwargs):raise TypeError('program bug')
    monkeypatch.setattr('vlm_nav.vlm_navigator.OpenAICompatibleVLMClient', broken)
    assert not n.ensure_worker()
    assert n.state == FAILED


def test_shutdown_cancels_motion_before_bounded_control_finish_and_reports_missing_ack(legacy):
    n, _ = legacy
    calls = []
    n._timers = [S(cancel=lambda: calls.append('timer_cancel'))]
    n.cancel_motion = lambda **kw: calls.append('motion_stop')
    n.control_recovery.finish_cancel = lambda spin, timeout: calls.append(('finish', timeout)) or False
    updates = []
    n.experiment_status = S(update=lambda **kw: updates.append(kw))
    assert not n.finish_legacy_control(lambda: None, 5.)
    assert calls == ['timer_cancel', 'motion_stop', ('finish', 5.)]
    assert n.recovery_shutdown
    assert updates[-1]['state'] == 'STOP_UNCONFIRMED'
    assert updates[-1]['result'] == 'NON_RESUMABLE'


def test_confirmation_budget_keeps_elapsed_time_across_pause(legacy):
    from vlm_nav.vlm_navigator import TARGET_CONFIRMING
    n, now = legacy
    n.state = TARGET_CONFIRMING
    n.target_confirmation_started = 5.
    n.enter_sensor_wait('temporary camera failure')
    now[0] = 1400.
    refresh(n, now[0])
    assert n.recover_from_sensor_wait()
    assert n.state == TARGET_CONFIRMING
    assert n.target_confirmation_started == 1395.
