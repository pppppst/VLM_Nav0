"""Small ROS contract tests; skipped when the ROS environment is not sourced."""

from collections import deque
import importlib.util
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
import numpy as np

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros import TransformException

from vlm_nav.vlm_navigator import (
    APPROACHING,
    APPROACH_STOPPING,
    API_ERROR,
    DISARMED,
    FAILED,
    SCANNING,
    SEARCHING,
    SENSOR_WAITING,
    TARGET_ALIGNING,
    TARGET_CONFIRMING,
    TARGET_REOBSERVING,
    VLMNavigator,
)
from vlm_nav.models import FrameSnapshot, Pixel, VLMResult, WorkerResult


RAW_DEPTH_LIBRARY = Path("/usr/local/lib/librealsense2.so.2.53.1")
RAW_DEPTH_PROBE_SPEC = importlib.util.spec_from_file_location(
    "navigation_gate_raw_depth_probe",
    Path(__file__).parents[1] / "scripts/raw_depth_probe.py",
)
RAW_DEPTH_PROBE = importlib.util.module_from_spec(RAW_DEPTH_PROBE_SPEC)
RAW_DEPTH_PROBE_SPEC.loader.exec_module(RAW_DEPTH_PROBE)


def test_raw_vlm_subscribes_to_images_and_dual_info_with_sensor_qos():
    os.environ["ROS_LOG_DIR"] = "/tmp/go2_roslog"
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "raw_depth_mode:=true",
            "-p", "rgb_topic:=/camera/camera/color/image_raw",
            "-p", "depth_topic:=/camera/camera/depth/image_rect_raw",
            "-p", "camera_info_topic:=/camera/camera/color/camera_info",
            "-p", "depth_camera_info_topic:=/camera/camera/depth/camera_info",
        ]
    )
    node = VLMNavigator()
    try:
        topics = {
            "/camera/camera/color/image_raw",
            "/camera/camera/depth/image_rect_raw",
            "/camera/camera/color/camera_info",
            "/camera/camera/depth/camera_info",
        }
        subscriptions = [
            item for item in node.subscriptions if item.topic_name in topics
        ]
        assert {item.topic_name for item in subscriptions} == topics
        assert all(
            item.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            and item.qos_profile.durability == DurabilityPolicy.VOLATILE
            for item in subscriptions
        )
        assert not any(
            item.topic_name == "/vlm_nav/vlm_api_ready"
            for item in node.subscriptions
        )
        assert any(
            item.topic_name == "/vlm_nav/vlm_api_ready"
            for item in node.publishers
        )
        assert any(
            item.topic_name == "/vlm_nav/camera_preview"
            and item.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            for item in node.publishers
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def path_pose(x, y):
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=float(x), y=float(y)))
    )


class RecordingMarkerPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_camera_preview_reuses_rgb_subscription_at_one_hz():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(camera_preview_rate=1.0)
    node.last_camera_preview = 0.0
    node.camera_preview_pub = RecordingMarkerPublisher()
    message = object()

    node.publish_camera_preview(message, now=10.0)
    node.publish_camera_preview(message, now=10.5)
    node.publish_camera_preview(message, now=11.0)

    assert node.camera_preview_pub.messages == [message, message]


def test_publish_debug_emits_annotated_target_image():
    node = VLMNavigator.__new__(VLMNavigator)
    node.debug_pub = RecordingMarkerPublisher()
    snapshot = SimpleNamespace(
        rgb=np.zeros((160, 240, 3), dtype=np.uint8),
        stamp=Time(),
        frame_id="camera_color_optical_frame",
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(180, 40),
        evidence_pixel=Pixel(35, 50),
    )

    node.publish_debug(snapshot, result)

    assert len(node.debug_pub.messages) == 1
    message = node.debug_pub.messages[0]
    assert (message.width, message.height, message.encoding) == (240, 160, "rgb8")
    assert any(message.data)


@pytest.mark.skipif(
    not RAW_DEPTH_LIBRARY.exists(), reason="Official librealsense library required"
)
def test_grounding_maps_color_pixel_before_reading_raw_depth():
    """Catches a regression to raw_depth[v_color, u_color]."""
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(
        depth_neighborhood_radius=0,
        min_depth=0.2,
        max_depth=5.0,
        min_depth_samples=1,
        max_depth_deviation=0.2,
        max_ground_height=0.35,
    )
    node.raw_depth_geometry = RAW_DEPTH_PROBE.Geometry(str(RAW_DEPTH_LIBRARY))
    info = {
        "width": 64,
        "height": 48,
        "k": [100.0, 0.0, 32.0, 0.0, 100.0, 24.0, 0.0, 0.0, 1.0],
        "d": [0.0] * 5,
    }
    depth_to_color = np.eye(4)
    depth_to_color[0, 3] = -0.02
    raw = np.zeros((48, 64), dtype=np.uint16)
    raw[24, 30] = 3000  # Wrong direct color-pixel lookup.
    raw[24, 32] = 1000  # Correct SDK-mapped depth pixel.
    metadata = {
        "color_info": info,
        "depth_info": info,
        "rgb_stamp_ns": 10,
        "depth_stamp_ns": 10,
        "depth_to_color": depth_to_color.tolist(),
        "color_to_depth": np.linalg.inv(depth_to_color).tolist(),
        "T_map_depth_optical": np.eye(4).tolist(),
    }
    snapshot = SimpleNamespace(
        rgb=np.zeros((48, 64, 3), dtype=np.uint8),
        depth_m=raw.astype(np.float32) * 0.001,
        intrinsics=(100.0, 100.0, 32.0, 24.0),
        transform_matrix=np.eye(4),
        raw_depth_snapshot=(
            np.zeros((48, 64, 3), dtype=np.uint8),
            raw,
            metadata,
        ),
        depth_scale=0.001,
    )

    point, reason = node.ground_pixel_with_reason(
        snapshot, Pixel(30, 24), require_ground=False
    )

    assert reason == "ok"
    assert point == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)


def test_raw_depth_missing_pixel_remains_recoverable():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(min_depth=0.2, max_depth=5.0, max_ground_height=0.35)
    node.raw_depth_geometry = SimpleNamespace(
        project=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            ValueError("SDK did not find a depth pixel")
        )
    )
    snapshot = SimpleNamespace(
        raw_depth_snapshot=object(), depth_scale=0.001
    )

    point, reason = node.ground_pixel_with_reason(
        snapshot, Pixel(30, 24), require_ground=False
    )

    assert point is None
    assert reason.startswith("insufficient_valid_depth_samples:")


def test_input_and_autonomy_readiness_are_separate_fail_closed_gates():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(
        rgbd_wait_timeout=2.0,
        tf_failure_timeout=3.0,
        external_gate_timeout=2.0,
    )
    now = time.monotonic()
    node.rgbd_sync_status = {"healthy": True, "rate": 0.9}
    node.latest_snapshot = SimpleNamespace(
        captured_monotonic=now, raw_depth_snapshot=object()
    )
    node.camera_extrinsic_matrix = np.eye(4)
    node.last_camera_tf_success = now
    node.depth_units_verified = False
    node.vlm_api_ready = True

    assert node.vlm_input_ready(now) is True
    assert node.vlm_autonomy_ready(now) is False

    node.depth_units_verified = True
    assert node.vlm_autonomy_ready(now) is True
    node.vlm_api_ready = False
    assert node.vlm_autonomy_ready(now) is False


def test_real_vlm_results_own_api_ready_state():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(require_external_safety_gates=False)
    published = []
    node.vlm_api_ready = False
    node.vlm_api_ready_pub = SimpleNamespace(
        publish=lambda message: published.append(message.data)
    )
    snapshot = SimpleNamespace(request_kind="target")
    result = VLMResult(
        target_visible=False,
        object_match=False,
        qualifier_match=False,
        relation_match=False,
        confidence=0.0,
        target_pixel=None,
        evidence_pixel=None,
    )

    assert node.update_vlm_api_ready(
        WorkerResult(snapshot, result, 0.2)
    ) is True
    assert node.update_vlm_api_ready(
        WorkerResult(snapshot, None, 4.0, "TimeoutError: timed out")
    ) is False

    assert published == [True, False]
    assert node.vlm_api_status_reason == "TimeoutError: timed out"


def test_first_snapshot_submits_one_real_api_preflight_while_disabled():
    node = VLMNavigator.__new__(VLMNavigator)
    node.api_preflight_pending = True
    node.state = "DISARMED"
    node.latest_snapshot = FrameSnapshot(
        sequence=1,
        task_epoch=0,
        target_description="chair",
        captured_monotonic=time.monotonic(),
        stamp=Time(),
        frame_id="camera",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 1.0, 1.0),
        transform_matrix=np.eye(4),
    )
    submitted = []
    node.worker = SimpleNamespace(submit=submitted.append)
    node.ensure_worker = lambda: True
    node.get_parameter = lambda name: SimpleNamespace(value="chair")
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)

    node.sample_latest_frame()
    node.sample_latest_frame()

    assert len(submitted) == 1
    assert submitted[0].request_kind == "api_preflight"
    assert node.api_preflight_pending is False


def test_raw_depth_autonomy_enable_requires_verified_report_not_a_bool_parameter():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(raw_depth_mode=True)
    values = {
        "enabled": False,
        "require_camera_calibration": False,
        "camera_extrinsic_calibrated": True,
        "require_external_safety_gates": False,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    node.depth_units_verified = False
    node.depth_units_error = "raw-depth unit report profile does not match runtime"
    node.vlm_input_ready = lambda: True

    result = node.on_parameters_changed([Parameter("enabled", value=True)])

    assert result.successful is False
    assert "unit report" in result.reason


def test_successful_target_probe_path_dispatches_without_map_reclassification():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 7
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0, path_horizon_segments=16)
    node.plan_kind = "target_probe"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.marker_pub = None
    node.classify_standoff_point = lambda *_args, **_kwargs: pytest.fail(
        "ComputePathToPose is authoritative for target probes"
    )
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(poses=[path_pose(0.0, 0.0), path_pose(1.0, 0.0)])
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=7)

    assert dispatched == [(1.0, 0.0, 0.0, "target_probe")]
    assert node.plan_pending is False


def test_easy_case_confirmation_enters_target_alignment():
    node = VLMNavigator.__new__(VLMNavigator)
    node.target_tracker = SimpleNamespace(
        update=lambda _point: (2.0, 0.0, 0.5)
    )
    node.target_reference_position = None
    node.p = SimpleNamespace(easy_case_mode=True)
    states = []
    node.cancel_motion = lambda publish_stop: None
    node.set_state = states.append
    node.publish_grounded_markers = lambda *_args: None

    node.confirm_target((2.0, 0.0, 0.5))

    assert states == [TARGET_ALIGNING]
    assert node.easy_alignment_complete is False
    assert node.target_reference_position == (2.0, 0.0, 0.5)


@pytest.mark.parametrize(
    ("refreshed_target", "expected_cancels"),
    [
        ((2.34, 0.0, 0.5), []),
        ((2.36, 0.0, 0.5), [True]),
    ],
)
def test_visible_target_refresh_replans_only_after_confirmed_goal_drift(
    refreshed_target, expected_cancels
):
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.target_reference_position = (2.0, 0.0, 0.5)
    node.goal_pose = (2.0, 0.0, 0.0)
    node.goal_kind = "approach"
    node.goal_handle = object()
    node.goal_pending = False
    node.plan_pending = False
    node.plan_kind = None
    node.current_plan_pose = None
    node.p = SimpleNamespace(confirmation_radius=0.35)
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    cancelled = []
    node.cancel_motion = lambda publish_stop: cancelled.append(publish_stop)

    node.set_confirmed_target_reference_position(refreshed_target)

    assert node.target_reference_position == refreshed_target
    assert cancelled == expected_cancels


def test_initial_arm_costmap_clear_blocks_scan_until_services_complete():
    callbacks = []

    class Future:
        def add_done_callback(self, callback):
            callbacks.append(callback)

        def result(self):
            return SimpleNamespace()

    class Client:
        srv_name = "/local_costmap/clear_entirely_local_costmap"

        def __init__(self):
            self.requests = []

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return Future()

    node = VLMNavigator.__new__(VLMNavigator)
    client = Client()
    node.p = SimpleNamespace(clear_costmap_on_arm=True)
    node.costmap_clear_clients = [("local", client)]
    node.initial_costmap_clear_required = False
    node.initial_costmap_clear_token = 0
    node.behavior_costmap_revision = 4
    node.last_behavior_costmap_received = 0.0
    stopped = []
    failed = []
    node.publish_stop = lambda: stopped.append(True)
    node.fail_safe = failed.append
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    node.require_initial_costmap_clear()

    assert node.ensure_initial_costmap_clear_done() is False
    assert len(client.requests) == 1
    assert node.initial_costmap_clear_pending == 1
    assert stopped
    assert not failed

    callbacks[0](Future())

    assert node.initial_costmap_clear_required is True
    assert node.last_initial_costmap_clear_status == "waiting_behavior_refresh"
    node.on_behavior_costmap(object())

    assert node.ensure_initial_costmap_clear_done() is True
    assert node.initial_costmap_clear_required is False
    assert node.last_initial_costmap_clear_status == "cleared_and_refreshed"


def test_initial_arm_scan_spin_waits_for_costmap_clear_gate():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.grid = np.zeros((1, 1), dtype=np.int16)
    node.map_message = object()
    node.session_origin = (0.0, 0.0)
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    node.target_reference_position = None
    node.scan_waiting_for_vlm = False
    node.scan_settle_until = 0.0
    node.scan_index = 0
    node.scan_headings = [0.5]
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.ensure_initial_costmap_clear_done = lambda: False
    spins = []
    node.send_spin = lambda relative, kind="scan": spins.append((relative, kind))

    node.navigation_tick()

    assert spins == []


def test_initial_arm_scan_fails_if_behavior_costmap_never_refreshes():
    node = VLMNavigator.__new__(VLMNavigator)
    node.initial_costmap_clear_required = True
    node.initial_costmap_clear_requested = True
    node.initial_costmap_clear_pending = 0
    node.initial_costmap_clear_completed_at = time.monotonic() - 1.0
    node.initial_costmap_refresh_deadline = time.monotonic() - 0.1
    node.initial_costmap_clear_baseline_revision = 3
    node.behavior_costmap_revision = 3
    node.last_behavior_costmap_received = 0.0
    stopped = []
    failed = []
    node.publish_stop = lambda: stopped.append(True)
    node.fail_safe = failed.append

    assert node.ensure_initial_costmap_clear_done() is False
    assert failed == [
        "Behavior costmap did not refresh after Nav2 costmap clear; "
        "refusing initial ARM scan"
    ]
    assert stopped
    assert (
        node.last_initial_costmap_clear_status
        == "failed:behavior_costmap_refresh_timeout"
    )


def test_first_grounded_scan_detection_stops_before_three_frame_confirmation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.target_tracker = SimpleNamespace(update=lambda _point: None)
    node.frontier_generation = 0
    node.frontier_request = object()
    node.frontier_request_pending = True
    cancelled = []
    states = []
    node.cancel_motion = lambda publish_stop: cancelled.append(publish_stop)
    node.set_state = states.append

    node.confirm_target((2.0, 0.0, 0.5))

    assert cancelled == [True]
    assert states == [TARGET_CONFIRMING]
    assert node.frontier_request is None
    assert node.frontier_request_pending is False


def test_successful_scan_spin_waits_for_stationary_vlm_frame():
    node = VLMNavigator.__new__(VLMNavigator)
    node.goal_token = 4
    node.goal_pose = None
    node.goal_handle = object()
    node.goal_pending = False
    node.goal_kind = "scan"
    node.active_motion_origin = None
    node.rolling_goal_is_final = True
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.sequence = 42
    node.p = SimpleNamespace(scan_settle_time=0.30)
    stopped = []
    node.publish_stop = lambda: stopped.append(True)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    started = time.monotonic()
    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped), token=4, kind="scan"
    )

    assert stopped
    assert node.scan_index == 0
    assert node.scan_capture_after_sequence == 42
    assert node.scan_settle_until >= started + 0.25
    assert node.scan_waiting_for_vlm is False
    assert node.scan_request_sequence == -1


def test_stationary_scan_submits_one_new_frame_and_holds_heading():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.scan_waiting_for_vlm = False
    node.scan_settle_until = time.monotonic() - 0.1
    node.scan_capture_after_sequence = 7
    node.scan_request_sequence = -1
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.last_submitted_sequence = -1
    snapshot = SimpleNamespace(
        sequence=8,
        captured_monotonic=time.monotonic(),
    )
    node.latest_snapshot = snapshot
    submitted = []
    node.worker = SimpleNamespace(submit=submitted.append)
    node.ensure_worker = lambda: True
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    node.sample_latest_frame()
    node.sample_latest_frame()

    assert submitted == [snapshot]
    assert node.scan_waiting_for_vlm is True
    assert node.scan_request_sequence == 8
    assert node.scan_index == 0


def test_begin_scan_observes_initial_heading_before_first_rotation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(scan_steps=8, scan_settle_time=0.30)
    node.sequence = 42
    canceled = []
    stopped = []
    states = []
    node.cancel_motion = lambda publish_stop=False: canceled.append(publish_stop)
    node.publish_stop = lambda: stopped.append(True)
    node.set_state = states.append

    started = time.monotonic()
    node.begin_scan((1.0, 2.0, 0.25))

    assert canceled == [True]
    assert stopped == [True]
    assert states == [SCANNING]
    assert node.scan_headings[0] == pytest.approx(0.25)
    assert node.scan_headings[1] == pytest.approx(0.25 + math.pi / 4.0)
    assert node.scan_headings[-1] == pytest.approx(0.25 + 7.0 * math.pi / 4.0)
    assert node.scan_index == 0
    assert node.scan_capture_after_sequence == 42
    assert node.scan_settle_until >= started + 0.25
    assert node.scan_waiting_for_vlm is False


def test_no_target_scan_result_advances_only_after_result_handling():
    node = VLMNavigator.__new__(VLMNavigator)
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.scan_observations = []
    node.scan_observation_headings = []
    node.scan_waiting_for_vlm = True
    node.scan_request_sequence = 9
    node.scan_settle_until = time.monotonic()
    node.scan_capture_after_sequence = 8
    node.scan_retry_count = 2
    node.sequence = 10
    snapshot = SimpleNamespace(rgb=np.zeros((2, 2, 3), dtype=np.uint8))

    node.advance_stationary_scan_view(snapshot)

    assert node.scan_index == 1
    assert len(node.scan_observations) == 1
    assert node.scan_observation_headings == [0.5]
    assert node.scan_waiting_for_vlm is False
    assert node.scan_settle_until == 0.0
    assert node.scan_retry_count == 0


def depth_recovery_snapshot():
    camera_to_map = np.eye(4)
    camera_to_map[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )
    return SimpleNamespace(
        intrinsics=(500.0, 500.0, 320.0, 240.0),
        transform_matrix=camera_to_map,
    )


def depth_recovery_node():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.task_epoch = 4
    node.p = SimpleNamespace(
        depth_reobserve_angle_deg=10.0,
        depth_reobserve_attempt_limit=3,
        target_probe_distance=2.0,
        target_probe_attempt_limit=3,
        max_travel_radius=3.0,
        target_probe_min_distance=0.3,
    )
    node.depth_reobserve_attempts = 0
    node.target_probe_attempts = 0
    node.last_depth_recovery_action = "none"
    node.scan_waiting_for_vlm = True
    node.scan_request_sequence = 8
    node.scan_settle_until = 1.0
    node.target_reference_position = None
    node.target_tracker = SimpleNamespace(reset=lambda: None)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.ground_pixel = lambda *_args, **_kwargs: None
    node.cancel_motion = lambda publish_stop: None
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    node.set_state = lambda state: setattr(node, "state", state)
    return node


def test_unreliable_target_depth_rotates_toward_vlm_pixel():
    node = depth_recovery_node()
    rotations = []
    node.send_spin = lambda angle, kind: rotations.append((angle, kind))
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(420, 240),
        evidence_pixel=Pixel(420, 240),
    )

    recovered = node.recover_target_depth(
        depth_recovery_snapshot(),
        result,
        "insufficient_valid_depth_samples:0/8",
    )

    assert recovered is True
    assert node.state == TARGET_REOBSERVING
    assert node.depth_reobserve_attempts == 1
    assert node.task_epoch == 5
    assert rotations[0][1] == "depth_reobserve"
    assert math.degrees(rotations[0][0]) == pytest.approx(-10.0)


def test_out_of_range_target_uses_nav2_validated_directional_probe():
    node = depth_recovery_node()
    planned = []
    node.plan_then_navigate = (
        lambda candidates, kind: planned.append((candidates, kind))
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(320, 240),
        evidence_pixel=Pixel(320, 240),
    )

    recovered = node.recover_target_depth(
        depth_recovery_snapshot(),
        result,
        "depth_out_of_range:median_m=6.800,limit_m=6.000",
    )

    assert recovered is True
    assert node.state == TARGET_REOBSERVING
    assert node.target_probe_attempts == 1
    assert node.task_epoch == 5
    assert planned[0][1] == "target_probe"
    assert planned[0][0][0] == pytest.approx((2.0, 0.0, 0.0))


def test_target_probe_candidates_use_only_the_target_ray_without_map_heuristics():
    node = depth_recovery_node()
    node.snap_free = lambda _point: pytest.fail(
        "target probes must not snap to occupancy-grid neighbors"
    )
    node.classify_standoff_point = lambda *_args, **_kwargs: pytest.fail(
        "target probes must not use footprint or N-cell map classification"
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(320, 240),
        evidence_pixel=Pixel(320, 240),
    )

    candidates = node.build_target_probe_candidates(
        depth_recovery_snapshot(), result, (0.0, 0.0, 0.0)
    )

    assert candidates == pytest.approx(
        [(2.0, 0.0, 0.0), (1.5, 0.0, 0.0), (1.0, 0.0, 0.0)]
    )


def test_confirmed_target_never_reenters_target_probe_recovery():
    node = depth_recovery_node()
    node.target_reference_position = (4.0, 0.0, 0.5)
    node.send_spin = lambda *_args, **_kwargs: pytest.fail(
        "a confirmed target must continue to standoff"
    )
    node.plan_then_navigate = lambda *_args, **_kwargs: pytest.fail(
        "a confirmed target must not dispatch a target probe"
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(320, 240),
        evidence_pixel=Pixel(320, 240),
    )

    recovered = node.recover_target_depth(
        depth_recovery_snapshot(),
        result,
        "depth_out_of_range:median_m=6.800,limit_m=6.000",
    )

    assert recovered is False
    assert node.target_reference_position == (4.0, 0.0, 0.5)


def test_target_probe_dispatch_refuses_an_already_confirmed_target():
    node = depth_recovery_node()
    node.target_reference_position = (4.0, 0.0, 0.5)
    node.plan_then_navigate = lambda *_args, **_kwargs: pytest.fail(
        "a confirmed target must not dispatch a target probe"
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(320, 240),
        evidence_pixel=Pixel(320, 240),
    )

    started = node.start_target_probe(
        depth_recovery_snapshot(), result, "depth_out_of_range"
    )

    assert started is False
    assert node.target_reference_position == (4.0, 0.0, 0.5)


def test_target_probe_arrival_waits_for_fresh_stationary_observation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_REOBSERVING
    node.goal_token = 6
    node.goal_pose = (2.0, 0.0, 0.0)
    node.goal_handle = object()
    node.goal_pending = False
    node.goal_kind = "target_probe"
    node.active_motion_origin = (0.0, 0.0)
    node.rolling_goal_is_final = True
    node.sequence = 21
    node.target_probe_attempts = 1
    node.depth_reobserve_attempts = 2
    node.scan_retry_count = 1
    node.p = SimpleNamespace(scan_settle_time=0.30)
    node.publish_stop = lambda: None
    node.set_state = lambda state: setattr(node, "state", state)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    started = time.monotonic()
    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped),
        token=6,
        kind="target_probe",
    )

    assert node.state == TARGET_REOBSERVING
    assert node.depth_reobserve_attempts == 0
    assert node.scan_capture_after_sequence == 21
    assert node.scan_settle_until >= started + 0.25
    assert node.scan_waiting_for_vlm is False


def test_easy_case_straight_plan_is_dispatched_without_radius_clipping():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 8
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.19, 0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        easy_case_max_path_deviation=0.10,
        path_horizon_segments=16,
    )
    node.plan_kind = "easy_approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[
                    path_pose(0.0, 0.0),
                    path_pose(0.6, 0.03),
                    path_pose(1.19, 0.0),
                ]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=8)

    assert dispatched == [(1.19, 0.0, 0.0, "easy_approach")]
    assert node.easy_path_deviation == pytest.approx(0.03)
    assert node.last_plan_radius_clipped is False


def test_easy_case_detouring_plan_is_rejected():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 9
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.19, 0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        easy_case_max_path_deviation=0.10,
    )
    node.plan_kind = "easy_approach"
    rejected = []
    node.reject_current_plan = lambda reason, token: rejected.append(
        (reason, token)
    )
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[
                    path_pose(0.0, 0.0),
                    path_pose(0.6, 0.25),
                    path_pose(1.19, 0.0),
                ]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=9)

    assert rejected and rejected[0][1] == 9
    assert "not sufficiently straight" in rejected[0][0]


def test_approach_dispatches_the_exact_nav2_validated_candidate_without_truncation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 2
    node.plan_attempt_count = 1
    node.current_plan_pose = (4.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0, path_horizon_segments=16)
    node.plan_kind = "approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warn=lambda *_args: None,
    )
    node.classify_standoff_point = lambda *_args, **_kwargs: pytest.fail(
        "approach must not consult the occupancy-map standoff helper"
    )
    dispatched = []
    rejected = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    node.reject_current_plan = lambda reason, token: rejected.append((reason, token))
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(poses=[path_pose(0.0, 0.0), path_pose(4.0, 0.0)])
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=2)

    assert not rejected
    assert dispatched == [(4.0, 0.0, 0.0, "approach")]
    assert node.last_plan_radius_clipped is False
    assert node.rolling_goal_is_final is True
    assert node.last_plan_commit_length == pytest.approx(4.0)


def test_compute_path_and_navigate_use_the_same_approach_goal():
    candidate = (1.25, -0.40, 0.75)
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 12
    node.plan_kind = "approach"
    node.plan_candidates = [candidate]
    node.plan_attempt_count = 0
    node.last_plan_candidate = None
    compute_goals = []

    class Future:
        def add_done_callback(self, callback):
            self.callback = callback

    node.path_planner = SimpleNamespace(
        send_goal_async=lambda goal: compute_goals.append(goal) or Future()
    )
    def make_pose(x, y, yaw):
        message = PoseStamped()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    node.make_pose_stamped = make_pose
    node.request_next_plan(12)

    node.p = SimpleNamespace(max_travel_radius=3.0)
    node.plan_pending = True
    node.plan_handle = object()
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warn=lambda *_args: None,
    )
    navigate_goals = []
    node.send_navigation_goal = lambda *goal: navigate_goals.append(goal)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[path_pose(0.0, 0.0), path_pose(candidate[0], candidate[1])]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=12)

    compute_pose = compute_goals[0].goal.pose
    compute_yaw = math.atan2(
        2.0 * compute_pose.orientation.w * compute_pose.orientation.z,
        1.0 - 2.0 * compute_pose.orientation.z**2,
    )
    assert (
        compute_pose.position.x,
        compute_pose.position.y,
        compute_yaw,
    ) == pytest.approx(candidate)
    assert navigate_goals == [(*candidate, "approach")]


def arrival_node(target_distance=0.80):
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.goal_token = 4
    node.plan_token = 3
    node.goal_handle = None
    node.goal_pending = False
    node.goal_kind = "approach"
    node.goal_pose = (2.0, 0.0, 0.0)
    node.plan_pending = False
    node.plan_handle = None
    node.plan_candidates = []
    node.plan_kind = None
    node.active_motion_origin = (0.0, 0.0)
    node.target_reference_position = (target_distance, 0.0, 0.5)
    node.arrival_stop_started = 0.0
    node.arrival_cancel_requested_at = 0.0
    node.arrival_cancel_acknowledged = False
    node.arrival_action_terminal = False
    node.arrival_action_terminal_at = 0.0
    node.arrival_stationary_count = 0
    node.arrival_last_odom_received = 0.0
    node.arrival_linear_speed = math.inf
    node.arrival_angular_speed = math.inf
    node.arrival_retry_count = 0
    node.p = SimpleNamespace(
        target_success_radius=0.81,
        approach_cancel_radius=0.89,
        arrival_linear_speed_tolerance=0.03,
        arrival_angular_speed_tolerance=0.05,
        arrival_stationary_samples=3,
        arrival_odom_max_age=0.5,
        arrival_cancel_timeout=3.0,
        arrival_stop_timeout=5.0,
        arrival_retry_limit=1,
        publish_stop_command=False,
    )
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warn=lambda *_args: None,
        error=lambda *_args: None,
    )
    node.publish_stop = lambda: None
    node.publish_easy_event = lambda *_args, **_kwargs: None
    node.set_state = lambda state: setattr(node, "state", state)
    node.fail_safe = lambda reason: (
        setattr(node, "last_failure_reason", reason),
        setattr(node, "state", FAILED),
    )
    return node


def stationary_odom():
    message = Odometry()
    message.twist.twist.linear.x = 0.02
    message.twist.twist.angular.z = 0.04
    return message


def test_distance_contract_requests_cancel_without_immediate_success():
    node = arrival_node()

    class CancelFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    cancel_future = CancelFuture()
    node.goal_handle = SimpleNamespace(
        cancel_goal_async=lambda: cancel_future
    )

    node.begin_approach_stop("distance_contract_reached")

    assert node.state == APPROACH_STOPPING
    assert node.arrival_cancel_requested_at > 0.0
    assert node.arrival_action_terminal is False
    assert hasattr(cancel_future, "callback")


def test_distance_contract_invalidates_a_pending_plan_before_success_checks():
    node = arrival_node()
    node.plan_pending = True
    original_token = node.plan_token

    node.begin_approach_stop("distance_contract_reached")

    assert node.plan_token == original_token + 1
    assert node.plan_pending is False
    assert node.arrival_action_terminal is True
    assert node.state == APPROACH_STOPPING


@pytest.mark.parametrize("easy_case_mode", [False, True])
def test_approach_plans_only_the_target_reference_position(easy_case_mode):
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.grid = np.zeros((2, 2), dtype=np.int8)
    node.map_message = object()
    node.session_origin = (0.0, 0.0)
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    node.target_reference_position = (2.0, -0.5, 0.4)
    node.p = SimpleNamespace(
        easy_case_mode=easy_case_mode,
        target_success_radius=0.81,
        approach_cancel_radius=0.89,
    )
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node.robot_pose = lambda: (0.0, 0.0, 0.35)
    node.publish_easy_case_markers = lambda _pose: None
    node.publish_easy_event = lambda *_args, **_kwargs: None
    planned = []
    node.plan_then_navigate = lambda candidates, kind: planned.append(
        (candidates, kind)
    )

    node.navigation_tick()

    expected_kind = "easy_approach" if easy_case_mode else "approach"
    assert planned == [([(2.0, -0.5, 0.35)], expected_kind)]


def test_approach_requests_stop_at_the_089_cancel_boundary():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.grid = np.zeros((2, 2), dtype=np.int8)
    node.map_message = object()
    node.session_origin = (0.0, 0.0)
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    node.target_reference_position = (0.85, 0.0, 0.4)
    node.p = SimpleNamespace(
        easy_case_mode=False,
        target_success_radius=0.81,
        approach_cancel_radius=0.89,
    )
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    reasons = []
    node.begin_approach_stop = reasons.append
    node.plan_then_navigate = lambda *_args: pytest.fail(
        "the goal must be canceled before entering the success radius"
    )

    node.navigation_tick()

    assert reasons == ["distance_contract_reached"]


def test_cancel_result_and_three_stationary_odom_frames_are_required_for_success():
    node = arrival_node()
    node.state = APPROACH_STOPPING
    node.arrival_stop_started = time.monotonic()
    node.goal_handle = object()
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_CANCELED)

    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped), token=4, kind="approach"
    )

    assert node.state == APPROACH_STOPPING
    assert node.arrival_action_terminal is True
    node.on_arrival_odom(stationary_odom())
    node.on_arrival_odom(stationary_odom())
    node.advance_approach_stop(time.monotonic())
    assert node.state == APPROACH_STOPPING
    node.on_arrival_odom(stationary_odom())
    node.advance_approach_stop(time.monotonic())
    assert node.state == "SUCCEEDED"


def test_nav2_success_still_enters_stop_confirmation():
    node = arrival_node()
    node.goal_handle = object()
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped), token=4, kind="approach"
    )

    assert node.state == APPROACH_STOPPING
    assert node.arrival_action_terminal is True


def test_moving_odom_resets_the_consecutive_stationary_count():
    node = arrival_node()
    node.state = APPROACH_STOPPING
    node.on_arrival_odom(stationary_odom())
    node.on_arrival_odom(stationary_odom())
    moving = stationary_odom()
    moving.twist.twist.linear.x = 0.04

    node.on_arrival_odom(moving)

    assert node.arrival_stationary_count == 0


def test_pending_action_that_never_accepts_hits_cancel_timeout():
    node = arrival_node()
    node.goal_pending = True
    node.begin_approach_stop("distance_contract_reached")

    node.advance_approach_stop(
        node.arrival_stop_started + node.p.arrival_cancel_timeout + 0.01
    )

    assert node.state == FAILED
    assert "terminal" in node.last_failure_reason


def test_stale_odom_cannot_complete_stop_confirmation():
    node = arrival_node()
    node.begin_approach_stop("nav2_action_succeeded", action_terminal=True)
    node.arrival_stationary_count = 3
    node.arrival_last_odom_received = node.arrival_stop_started

    node.advance_approach_stop(
        node.arrival_stop_started + node.p.arrival_stop_timeout + 0.01
    )

    assert node.state == FAILED
    assert "fresh odometry" in node.last_failure_reason


def test_motion_stop_timeout_starts_after_the_action_becomes_terminal():
    node = arrival_node()
    node.state = APPROACH_STOPPING
    node.arrival_stop_started = time.monotonic() - 2.9
    node.arrival_action_terminal = True
    node.arrival_action_terminal_at = time.monotonic()

    node.advance_approach_stop(
        node.arrival_action_terminal_at + node.p.arrival_stop_timeout - 0.01
    )
    assert node.state == APPROACH_STOPPING

    node.advance_approach_stop(
        node.arrival_action_terminal_at + node.p.arrival_stop_timeout + 0.01
    )
    assert node.state == FAILED


def test_stopped_outside_contract_replans_once_then_fails():
    node = arrival_node(target_distance=0.82)
    node.state = APPROACH_STOPPING
    node.arrival_stop_started = time.monotonic()
    node.arrival_action_terminal = True
    node.arrival_stationary_count = 3
    node.arrival_last_odom_received = time.monotonic()

    node.advance_approach_stop(time.monotonic())

    assert node.state == APPROACHING
    assert node.arrival_retry_count == 1

    node.begin_approach_stop("nav2_action_succeeded", action_terminal=True)
    node.arrival_stationary_count = 3
    node.arrival_last_odom_received = time.monotonic()
    node.advance_approach_stop(time.monotonic())

    assert node.state == FAILED
    assert "outside" in node.last_failure_reason


def test_retry_inside_cancel_boundary_is_allowed_to_drive_to_success_radius():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.grid = np.zeros((2, 2), dtype=np.int8)
    node.map_message = object()
    node.session_origin = (0.0, 0.0)
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    node.target_reference_position = (0.85, 0.0, 0.4)
    node.arrival_retry_count = 1
    node.p = SimpleNamespace(
        easy_case_mode=False,
        target_success_radius=0.81,
        approach_cancel_radius=0.89,
    )
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    planned = []
    node.plan_then_navigate = lambda candidates, kind: planned.append(
        (candidates, kind)
    )
    node.begin_approach_stop = lambda *_args: pytest.fail(
        "the one allowed retry must be able to close the remaining gap"
    )

    node.navigation_tick()

    assert planned == [([(0.85, 0.0, 0.0)], "approach")]


def test_planner_action_failure_is_distinct_from_radius_rejection():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 5
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0)
    node.plan_kind = "approach"
    dispatched = []
    rejected = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    node.reject_current_plan = lambda reason, token: rejected.append((reason, token))
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(path=SimpleNamespace(poses=[])),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=5)

    assert not dispatched
    assert rejected and rejected[0][1] == 5
    assert "reason=planner_action_status_6" in rejected[0][0]


def test_empty_approach_path_rejects_candidate_and_tries_the_next_one():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 6
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.plan_kind = "approach"
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        blocked_goal_seconds=15.0,
    )
    requested = []
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    node.blocked_goals = {}
    node.plan_handle = object()
    node.request_next_plan = requested.append
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(path=SimpleNamespace(poses=[])),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=6)

    assert requested == [6]
    assert node.last_plan_rejection_reason == "empty_path"


def test_long_frontier_path_commits_only_first_half_for_reassessment():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 3
    node.current_plan_pose = (2.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        path_horizon_segments=16,
        path_execute_segments=8,
        frontier_full_commit_distance=1.0,
    )
    node.plan_kind = "frontier"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.frontier_generation = 5
    node.frontier_selected_id = 2
    node.publish_rolling_path = lambda *_args: None
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[path_pose(0.0, 0.0), path_pose(2.0, 0.0)]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=3)

    assert dispatched[0][0] == pytest.approx(1.0)
    assert dispatched[0][1] == pytest.approx(0.0)
    assert dispatched[0][3] == "frontier"
    assert node.frontier_goal_is_final is False
    assert len(node.frontier_path_samples) == 17


def test_short_frontier_path_commits_to_actual_frontier():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 4
    node.current_plan_pose = (0.8, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        path_horizon_segments=16,
        path_execute_segments=8,
        frontier_full_commit_distance=1.0,
    )
    node.plan_kind = "frontier"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.frontier_generation = 5
    node.frontier_selected_id = 1
    node.publish_rolling_path = lambda *_args: None
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[path_pose(0.0, 0.0), path_pose(0.8, 0.0)]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=4)

    assert dispatched == [(0.8, 0.0, 0.0, "frontier")]
    assert node.frontier_goal_is_final is True


def test_api_failure_does_not_overwrite_failed_state():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = FAILED
    node.api_failures = 3
    node.rejected_results = 0
    node.last_api_error = "none"
    node.p = SimpleNamespace(api_failure_limit=3)
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    node.cancel_motion = lambda publish_stop: pytest.fail(
        "FAILED state must not be replaced by API_ERROR"
    )
    node.set_state = lambda state: pytest.fail(
        "FAILED state must not be replaced by API_ERROR"
    )

    node.record_api_failure("timeout")

    assert node.state == FAILED
    assert node.last_api_error == "timeout"
    assert node.api_failures == 4


def test_target_api_failure_disarms_only_after_three_consecutive_failures():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.api_failures = 0
    node.rejected_results = 0
    node.last_api_error = "none"
    node.last_api_latency = 0.0
    node.last_result_age = 0.0
    node.vlm_api_ready = True
    node.vlm_api_status_reason = "valid target response"
    node.vlm_api_last_update = 0.0
    node.vlm_api_ready_pub = SimpleNamespace(publish=lambda _message: None)
    node.p = SimpleNamespace(
        api_failure_limit=3,
        require_external_safety_gates=True,
    )
    enabled = {"value": True}
    node.get_parameter = lambda name: SimpleNamespace(value=enabled["value"])

    def set_parameters(parameters):
        assert len(parameters) == 1
        assert parameters[0].name == "enabled"
        assert parameters[0].value is False
        enabled["value"] = False
        node.state = DISARMED
        return [SimpleNamespace(successful=True)]

    node.set_parameters = set_parameters
    node.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        warn=lambda _message: None,
    )
    node.is_current_scan_result = lambda _snapshot: False
    node.is_current_depth_reobserve_result = lambda _snapshot: False
    node.log_worker_result = lambda *_args: None
    node.cancel_motion = lambda publish_stop: pytest.fail(
        "parameter disable owns the fail-safe reset"
    )
    node.set_state = lambda state: pytest.fail(
        f"API failure threshold must disable the task, not enter {state}"
    )
    snapshot = SimpleNamespace(
        request_kind="target",
        captured_monotonic=time.monotonic(),
    )

    for expected_failures in (1, 2):
        node.handle_worker_result(
            WorkerResult(snapshot, None, 0.2, "JSONDecodeError: truncated")
        )
        assert node.api_failures == expected_failures
        assert enabled["value"] is True
        assert node.state == APPROACHING

    node.handle_worker_result(
        WorkerResult(snapshot, None, 0.2, "JSONDecodeError: truncated")
    )

    assert node.api_failures == 3
    assert enabled["value"] is False
    assert node.state == DISARMED
    assert node.vlm_api_ready is False


def test_vlm_result_is_published_and_forwarded_to_image_recorder():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_CONFIRMING
    recorded = []
    published = []
    node.image_recorder = SimpleNamespace(
        record=lambda *args: recorded.append(args) or ("image.jpg",),
        last_error="none",
    )
    node.publish_vlm_output = published.append
    snapshot = SimpleNamespace(
        sequence=12,
        task_epoch=3,
        target_description="chair",
        stamp=SimpleNamespace(sec=123, nanosec=456),
        frame_id="camera_color_optical_frame",
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.92,
        target_pixel=Pixel(640, 320),
        evidence_pixel=Pixel(700, 250),
    )
    completed = WorkerResult(
        snapshot,
        result,
        0.75,
        raw_response=(
            '{"target_visible":true,"object_match":true,'
            '"qualifier_match":true,"relation_match":true,'
            '"confidence":0.92,'
            '"target_pixel":{"u":640,"v":320},'
            '"evidence_pixel":{"u":700,"v":250}}'
        ),
    )

    node.log_worker_result(
        completed, age=0.8, state_before=SEARCHING, disposition="accepted_target"
    )

    record = published[0]
    assert record["event"] == "vlm_response"
    assert record["navigation_state_before"] == SEARCHING
    assert record["navigation_state_after"] == TARGET_CONFIRMING
    assert record["disposition"] == "accepted_target"
    assert record["accepted"] is True
    assert record["parsed_result"]["target_pixel"] == {"u": 640, "v": 320}
    assert record["parsed_result"]["object_match"] is True
    assert record["parsed_result"]["qualifier_match"] is True
    assert record["parsed_result"]["relation_match"] is True
    assert record["parsed_result"]["evidence_pixel"] == {"u": 700, "v": 250}
    assert "waypoints" not in record["parsed_result"]
    assert record["raw_response"].startswith('{"target_visible":true')
    assert recorded[0][0] is snapshot
    assert recorded[0][1] is result
    assert recorded[0][2] == "accepted_target"


def semantic_gate_node():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SEARCHING
    node.task_epoch = 3
    node.last_api_latency = 0.0
    node.last_result_age = 0.0
    node.last_vlm_confidence = 0.0
    node.api_failures = 0
    node.accepted_results = 0
    node.rejected_results = 0
    node.target_reference_position = None
    node.p = SimpleNamespace(max_result_age=10.0, confidence_threshold=0.60)
    node.get_parameter = lambda name: SimpleNamespace(
        value=True if name == "enabled" else "放了可乐的椅子"
    )
    node.is_current_scan_result = lambda snapshot: False
    node.is_current_depth_reobserve_result = lambda snapshot: False
    node.publish_debug = lambda *_args: None
    node.log_dispositions = []
    node.log_worker_result = (
        lambda _completed, _age, _state, disposition:
        node.log_dispositions.append(disposition)
    )
    node.target_tracker = SimpleNamespace(reset=lambda: None)
    node.clear_vlm_grounding_markers = lambda: None
    return node


def semantic_gate_completed(result):
    snapshot = SimpleNamespace(
        captured_monotonic=time.monotonic(),
        request_kind="target",
        task_epoch=3,
        target_description="放了可乐的椅子",
    )
    return WorkerResult(snapshot=snapshot, result=result, latency_s=0.2)


def test_partial_semantic_match_never_enters_target_grounding():
    node = semantic_gate_node()
    node.ground_pixel_with_reason = lambda *_args, **_kwargs: pytest.fail(
        "partial semantic matches must not be projected"
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=False,
        relation_match=False,
        confidence=0.95,
        target_pixel=Pixel(640, 320),
        evidence_pixel=Pixel(700, 250),
    )

    node.handle_worker_result(semantic_gate_completed(result))

    assert node.log_dispositions == ["accepted_no_target"]


def test_missing_semantic_evidence_never_enters_target_grounding():
    node = semantic_gate_node()
    node.ground_pixel_with_reason = lambda *_args, **_kwargs: pytest.fail(
        "a complete semantic gate requires evidence_pixel"
    )
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.95,
        target_pixel=Pixel(640, 320),
        evidence_pixel=None,
    )

    node.handle_worker_result(semantic_gate_completed(result))

    assert node.log_dispositions == ["accepted_no_target"]


def test_no_target_observation_keeps_confirmed_goal_and_is_not_api_failure():
    node = semantic_gate_node()
    node.state = APPROACHING
    node.api_failures = 2
    node.target_reference_position = (2.0, -0.5, 0.4)
    node.target_tracker = SimpleNamespace(
        reset=lambda: pytest.fail("a temporary no-target view must keep tracking")
    )
    node.clear_vlm_grounding_markers = lambda: pytest.fail(
        "a temporary no-target view must keep the confirmed target"
    )
    node.cancel_motion = lambda **_kwargs: pytest.fail(
        "the existing Nav2 approach must continue"
    )
    result = VLMResult(
        target_visible=False,
        object_match=False,
        qualifier_match=False,
        relation_match=False,
        confidence=0.0,
        target_pixel=None,
        evidence_pixel=None,
    )

    node.handle_worker_result(semantic_gate_completed(result))

    assert node.api_failures == 0
    assert node.state == APPROACHING
    assert node.target_reference_position == (2.0, -0.5, 0.4)
    assert node.log_dispositions == ["accepted_no_target"]


def test_complete_match_projects_target_pixel_but_not_evidence_pixel():
    node = semantic_gate_node()
    projected = []
    node.ground_pixel_with_reason = (
        lambda _snapshot, pixel, require_ground:
        projected.append((pixel, require_ground)) or ((1.0, 2.0, 0.5), "ok")
    )
    node.clear_target_depth_recovery = lambda: None
    node.confirm_target = lambda target: None
    node.easy_case_enabled = lambda: True
    node.publish_grounded_markers = lambda *_args: None
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.90,
        target_pixel=Pixel(640, 320),
        evidence_pixel=Pixel(700, 250),
    )

    node.handle_worker_result(semantic_gate_completed(result))

    assert projected == [(Pixel(640, 320), False)]
    assert node.log_dispositions == ["accepted_target_easy_case"]


def test_grounded_markers_include_only_target_and_label_without_deleteall():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(global_frame="map", target_description="chair")
    node.last_vlm_confidence = 0.91
    published = []
    node.marker_pub = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    node.publish_grounded_markers(target=(2.0, 0.5, 0.1))

    markers = published[0].markers
    assert not any(item.action == Marker.DELETEALL for item in markers)
    namespaces = {item.ns for item in markers if item.action == Marker.ADD}
    assert namespaces == {"vlm_target", "vlm_target_label"}


@pytest.mark.parametrize("terminal_state", [SENSOR_WAITING, FAILED])
def test_terminal_failure_does_not_submit_more_images(terminal_state):
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = terminal_state
    node.latest_snapshot = object()
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: pytest.fail(
        "FAILED state must not start or submit API work"
    )

    node.sample_latest_frame()


def test_api_error_can_still_retry_for_automatic_recovery():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = API_ERROR
    node.latest_snapshot = SimpleNamespace(sequence=9)
    node.last_submitted_sequence = 8
    submitted = []
    node.worker = SimpleNamespace(submit=lambda snapshot: submitted.append(snapshot))
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: True

    node.sample_latest_frame()

    assert submitted == [node.latest_snapshot]


def test_frontier_retry_refreshes_attempt_timestamp():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = API_ERROR
    snapshot = FrameSnapshot(
        sequence=9,
        task_epoch=1,
        target_description="chair",
        captured_monotonic=1.0,
        stamp=Time(),
        frame_id="camera",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.zeros((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 1.0, 1.0),
        transform_matrix=np.eye(4),
        request_kind="frontier",
    )
    node.frontier_request = snapshot
    node.frontier_request_pending = True
    submitted = []
    node.worker = SimpleNamespace(submit=submitted.append)
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: True

    node.sample_latest_frame()

    assert len(submitted) == 1
    assert submitted[0].captured_monotonic > 1.0
    assert node.frontier_request is submitted[0]
    assert node.frontier_request_pending is False


def safety_test_node():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_CONFIRMING
    node.p = SimpleNamespace(
        rgbd_wait_timeout=2.0,
        sensor_failure_timeout=30.0,
        tf_failure_timeout=3.0,
        max_travel_radius=3.0,
        approach_cancel_radius=0.89,
        target_confirmation_timeout=20.0,
        goal_timeout=120.0,
    )
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.session_origin = None
    node.target_reference_position = None
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    now = time.monotonic()
    node.last_valid_rgbd_received = now
    node.last_camera_tf_success = now
    node.last_robot_tf_success = now
    node.sensor_wait_started = 0.0
    node.sensor_wait_reason = "none"
    node.target_confirmation_started = time.monotonic()
    return node


def test_approach_does_not_expire_when_target_is_no_longer_observed():
    node = safety_test_node()
    node.state = APPROACHING
    node.target_reference_position = (2.0, 0.0, 0.5)
    node.target_seen_time = time.monotonic() - 3600.0
    node.last_robot_tf_success = time.monotonic()
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == []
    assert node.state == APPROACHING


def test_safety_tick_refreshes_robot_tf_while_target_confirming():
    node = safety_test_node()
    node.last_valid_rgbd_received = time.monotonic()
    node.last_camera_tf_success = time.monotonic()
    node.last_robot_tf_success = 0.0
    node.robot_pose = lambda: (
        setattr(node, "last_robot_tf_success", time.monotonic())
        or (0.0, 0.0, 0.0)
    )
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == []


def test_safety_tick_fails_with_confirmation_progress_instead_of_hanging():
    """Catch removal of the bounded TARGET_CONFIRMING watchdog."""
    node = safety_test_node()
    node.target_confirmation_started = time.monotonic() - 21.0
    node.target_tracker = SimpleNamespace(progress=2, required_frames=3, reset_count=7)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == [
        "Target confirmation did not converge within 20.0s "
        "(progress=2/3, spatial_resets=7)"
    ]


def test_safety_tick_pauses_for_camera_tf_without_failing():
    node = safety_test_node()
    node.last_valid_rgbd_received = time.monotonic()
    node.last_camera_tf_success = 0.0
    node.last_robot_tf_success = 0.0
    node.robot_pose = lambda: (
        setattr(node, "last_robot_tf_success", time.monotonic())
        or (0.0, 0.0, 0.0)
    )
    pauses = []
    node.enter_sensor_wait = pauses.append
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == []
    assert pauses == [
        "Valid RGB-D is arriving but image-time camera TF is unavailable"
    ]


def test_safety_tick_distinguishes_missing_valid_rgbd():
    node = safety_test_node()
    node.last_valid_rgbd_received = 0.0
    node.last_camera_tf_success = time.monotonic()
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    pauses = []
    node.enter_sensor_wait = pauses.append
    node.fail_safe = lambda reason: pytest.fail(reason)

    node.safety_tick()

    assert pauses == [
        "No valid aligned RGB-D received within safety timeout"
    ]


def test_sensor_wait_hard_timeout_is_the_only_sensor_failure():
    node = safety_test_node()
    node.state = SENSOR_WAITING
    now = time.monotonic()
    node.last_valid_rgbd_received = now
    node.last_camera_tf_success = now
    node.sensor_wait_started = now - 31.0
    node.sensor_wait_reason = "No valid aligned RGB-D received within safety timeout"
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == [
        "Sensor unavailable beyond hard timeout: "
        "No valid aligned RGB-D received within safety timeout"
    ]


def test_rgbd_metadata_mismatch_is_rejected_before_image_conversion():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.p = SimpleNamespace(camera_frame="camera_color_optical_frame")
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.camera_info = SimpleNamespace(width=1280, height=720)
    node.last_rgbd_pair_received = 0.0
    node.last_valid_rgbd_received = 0.0
    node.invalid_rgbd_frames = 0
    node.sensor_recovery_count = 2
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    node.image_to_rgb = lambda message: pytest.fail(
        "metadata mismatch must be rejected before RGB conversion"
    )
    node.image_to_depth_m = lambda message: pytest.fail(
        "metadata mismatch must be rejected before depth conversion"
    )
    rgb = SimpleNamespace(
        width=1280,
        height=720,
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )
    depth = SimpleNamespace(
        width=640,
        height=480,
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )

    node.on_rgbd(rgb, depth)

    assert node.last_rgbd_pair_received > 0.0
    assert node.last_valid_rgbd_received == 0.0
    assert node.invalid_rgbd_frames == 1
    assert node.sensor_recovery_count == 0


def test_raw_rgbd_snapshot_caches_static_extrinsic_and_uses_depth_stamp_tf():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.p = SimpleNamespace(
        global_frame="map",
        camera_frame="camera_color_optical_frame",
        vlm_sample_rate=5.0,
        image_tf_queue_size=4,
        image_tf_wait_timeout=5.0,
        sensor_recovery_frames=3,
        raw_depth_mode=True,
    )
    node.get_parameter = lambda name: SimpleNamespace(
        value="chair" if name == "target_description" else False
    )
    color_info = SimpleNamespace(
        width=2,
        height=2,
        k=[100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0],
        d=[0.0] * 5,
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )
    depth_info = SimpleNamespace(
        width=2,
        height=2,
        k=[90.0, 0.0, 1.0, 0.0, 90.0, 1.0, 0.0, 0.0, 1.0],
        d=[0.0] * 5,
        header=SimpleNamespace(frame_id="camera_depth_optical_frame"),
    )
    node.camera_infos = {"color": color_info, "depth": depth_info}
    node.last_rgbd_pair_received = 0.0
    node.last_valid_rgbd_received = 0.0
    node.invalid_rgbd_frames = 0
    node.sensor_recovery_count = 0
    node.pending_rgbd_frames = deque()
    node.last_rgbd_queued = 0.0
    node.image_tf_queue_drops = 0
    node.image_tf_failures = 0
    node.last_image_tf_error = "none"
    node.sequence = 0
    node.task_epoch = 7
    node.latest_snapshot = None
    node.last_camera_tf_success = 0.0
    node.camera_extrinsic_matrix = None
    node.depth_scale = 0.001
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    requested = []

    def lookup(target, source, requested_time, timeout):
        requested.append((target, source, requested_time.nanoseconds))
        translation = SimpleNamespace(x=0.02 if target.startswith("camera") else 1.0,
                                      y=0.0, z=0.0)
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=translation,
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    def image(stamp_ns, frame, encoding, data, step):
        return SimpleNamespace(
            width=2,
            height=2,
            step=step,
            encoding=encoding,
            is_bigendian=0,
            data=data,
            header=SimpleNamespace(
                frame_id=frame,
                stamp=Time(sec=0, nanosec=stamp_ns),
            ),
        )

    rgb = image(10, "camera_color_optical_frame", "rgb8", bytes(12), 6)
    depth = image(
        20,
        "camera_depth_optical_frame",
        "16UC1",
        np.full((2, 2), 1000, dtype=np.uint16).tobytes(),
        4,
    )

    node.on_rgbd(rgb, depth)
    first = node.latest_snapshot
    node.last_rgbd_queued = 0.0
    newer_depth = image(
        20,
        "camera_depth_optical_frame",
        "16UC1",
        np.full((2, 2), 2000, dtype=np.uint16).tobytes(),
        4,
    )
    node.on_rgbd(rgb, newer_depth)

    assert first is not None
    assert first.raw_depth_snapshot[1][0, 0] == 1000
    assert node.latest_snapshot.raw_depth_snapshot[1][0, 0] == 2000
    assert first.rgb.flags.writeable is False
    assert first.raw_depth_snapshot[1].flags.writeable is False
    with pytest.raises(TypeError):
        first.raw_depth_snapshot[2]["depth_stamp_ns"] = 999
    assert requested.count(
        ("camera_color_optical_frame", "camera_depth_optical_frame", 0)
    ) == 1
    assert requested.count(("map", "camera_depth_optical_frame", 20)) == 2


def test_delayed_image_tf_is_retried_at_the_exact_original_stamp():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.p = SimpleNamespace(
        global_frame="map",
        camera_frame="camera_color_optical_frame",
        image_tf_wait_timeout=5.0,
        sensor_recovery_frames=3,
    )
    stamp = Time(sec=123, nanosec=456)
    frame = SimpleNamespace(
        received_monotonic=time.monotonic(),
        stamp=stamp,
        frame_id="camera_color_optical_frame",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 1.0, 1.0),
    )
    node.pending_rgbd_frames = deque([frame])
    node.image_tf_queue_drops = 0
    node.image_tf_failures = 0
    node.last_image_tf_error = "none"
    node.sensor_recovery_count = 0
    node.sequence = 0
    node.task_epoch = 7
    node.latest_snapshot = None
    node.last_camera_tf_success = 0.0
    node.get_parameter = lambda name: SimpleNamespace(
        value="chair" if name == "target_description" else True
    )
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda *args, **kwargs: warnings.append((args, kwargs))
    )
    requested = []

    def unavailable(_target, _source, requested_time, timeout):
        requested.append(requested_time.nanoseconds)
        raise TransformException("Lookup would require extrapolation into the future")

    node.tf_buffer = SimpleNamespace(lookup_transform=unavailable)
    node.resolve_pending_rgbd()

    assert len(node.pending_rgbd_frames) == 1
    assert node.latest_snapshot is None
    assert node.image_tf_failures == 0
    assert node.last_image_tf_error == "none"
    assert not warnings

    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    node.tf_buffer.lookup_transform = (
        lambda _target, _source, requested_time, timeout: (
            requested.append(requested_time.nanoseconds) or transform
        )
    )
    node.resolve_pending_rgbd()

    expected_stamp_ns = 123_000_000_456
    assert requested == [expected_stamp_ns, expected_stamp_ns]
    assert not node.pending_rgbd_frames
    assert node.latest_snapshot.stamp is stamp
    assert node.latest_snapshot.captured_monotonic == frame.received_monotonic
    assert node.sequence == 1
    assert node.last_image_tf_error == "none"


def test_map_update_does_not_cancel_nav2_validated_approach():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.target_reference_position = (2.0, 0.0, 0.5)
    node.map_revision = 3
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    cancelled = []
    node.cancel_motion = lambda publish_stop: cancelled.append(publish_stop)
    message = SimpleNamespace(
        info=SimpleNamespace(width=2, height=2),
        data=[0, 0, 0, 100],
    )

    node.on_map(message)

    assert node.map_revision == 4
    assert cancelled == []


def test_compact_diagnostics_keep_only_fault_isolation_fields():
    now = time.monotonic()
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.sequence = 42
    node.last_camera_tf_success = now - 0.25
    node.last_robot_tf_success = now - 0.10
    node.last_image_tf_error = "none"
    node.scan_headings = [0.0, 1.57, 3.14]
    node.scan_index = 1
    node.scan_waiting_for_vlm = True
    node.scan_retry_count = 1
    node.scan_settle_until = 0.0
    node.goal_kind = "scan"
    node.last_failure_reason = "none"
    node.last_result_age = 0.4
    node.last_api_latency = 1.2
    node.last_vlm_disposition = "target_rejected"
    node.target_reference_position = None
    node.last_target_grounding_error = "invalid_depth"
    node.plan_pending = False
    node.plan_kind = None
    node.goal_pending = False
    node.goal_handle = None
    node.last_plan_rejection_reason = "none"
    node.last_plan_kind = "none"
    node.last_plan_status = -1
    node.sensor_wait_reason = "none"
    node.last_api_error = "none"
    node.rgbd_sync_status = {
        "healthy": False, "matched": 8, "total": 10, "rate": 0.8
    }
    node.depth_units_verified = False
    node.depth_units_error = "unit report not loaded"

    values = node.compact_diagnostic_values(now, worker_busy=False)

    assert list(values) == [
        "state",
        "sequence",
        "camera_status",
        "robot_tf_age_s",
        "scan_status",
        "vlm_status",
        "target_status",
        "navigation_status",
        "approach_status",
        "rgbd_sync",
        "depth_units",
        "vlm_api_ready",
        "sensor_wait_reason",
        "last_api_error",
        "last_failure_reason",
    ]
    assert values["sequence"] == 42
    assert values["camera_status"] == "ok; ready_age_s=0.25"
    assert values["scan_status"] == (
        "waiting_vlm; heading=2/3; retry=1"
    )
    assert values["target_status"] == "rejected; invalid_depth"
    assert values["rgbd_sync"] == "degraded; matched=8/10; rate=0.800"
    assert values["depth_units"] == "blocked; unit report not loaded"


def test_compact_diagnostics_expose_target_confirmation_resets():
    """Catch diagnostics that collapse a non-converging tracker to target_found."""
    now = time.monotonic()
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_CONFIRMING
    node.sequence = 8
    node.last_camera_tf_success = now
    node.last_robot_tf_success = now
    node.last_image_tf_error = "none"
    node.scan_headings = []
    node.scan_index = 0
    node.scan_waiting_for_vlm = False
    node.scan_retry_count = 0
    node.scan_settle_until = 0.0
    node.goal_kind = None
    node.last_failure_reason = "none"
    node.last_result_age = 0.2
    node.last_api_latency = 1.0
    node.last_vlm_disposition = "accepted_target"
    node.target_reference_position = None
    node.last_target_grounding_error = "none"
    node.target_confirmation_started = now - 6.25
    node.target_tracker = SimpleNamespace(
        progress=2,
        required_frames=3,
        reset_count=4,
        last_jump_distance=0.62,
    )
    node.plan_pending = False
    node.plan_kind = None
    node.goal_pending = False
    node.goal_handle = None
    node.last_plan_rejection_reason = "none"
    node.last_plan_kind = "none"
    node.last_plan_status = -1
    node.sensor_wait_reason = "none"
    node.last_api_error = "none"

    values = node.compact_diagnostic_values(now, worker_busy=False)

    assert values["target_status"] == (
        "confirming; progress=2/3; age_s=6.25; "
        "spatial_resets=4; last_jump_m=0.62"
    )
