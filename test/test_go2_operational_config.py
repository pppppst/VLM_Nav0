from pathlib import Path
import importlib.util
import os
from types import SimpleNamespace
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

import yaml

from vlm_nav.go2_sensor_preflight import load_validated_config


ROOT = Path(__file__).resolve().parents[1]


def test_common_environment_includes_xt16_driver_workspace():
    source = (ROOT / "scripts/common.sh").read_text()
    assert "/home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash" in source
    assert 'vlm_nav_setup="${vlm_nav_workspace_dir}/go2_ws/install/setup.bash"' in source


def test_readiness_waiter_declares_string_array_parameters():
    from vlm_nav.go2_readiness_waiter import ReadinessWaiter

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/go2_roslog")
    rclpy.init(args=[])
    node = ReadinessWaiter()
    try:
        assert node.get_parameter("required_topics").type_.name == "STRING_ARRAY"
        assert node.get_parameter("required_transforms").type_.name == "STRING_ARRAY"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_go2_camera_preflight_uses_sensor_data_qos_for_all_four_streams():
    from vlm_nav.go2_camera_preflight import Go2CameraPreflight

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/go2_roslog")
    rclpy.init(args=[])
    node = Go2CameraPreflight(duration=0.1, minimum_samples=1, sync_slop_s=0.05)
    try:
        camera_topics = {
            "/camera/camera/color/image_raw",
            "/camera/camera/depth/image_rect_raw",
            "/camera/camera/color/camera_info",
            "/camera/camera/depth/camera_info",
        }
        subscriptions = {
            subscription.topic_name: subscription.qos_profile
            for subscription in node.subscriptions
            if subscription.topic_name in camera_topics
        }
        assert set(subscriptions) == camera_topics
        assert all(
            profile.reliability == ReliabilityPolicy.BEST_EFFORT
            and profile.durability == DurabilityPolicy.VOLATILE
            for profile in subscriptions.values()
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_network_environment_is_modern_and_fails_closed():
    script = (ROOT / "scripts/go2_network_env.sh").read_text()
    cyclone = (ROOT / "config/cyclonedds_go2.xml").read_text()

    assert "GO2_NET_IFACE" in script
    assert "matches[@]" in script
    assert "more than one" in script
    assert "ROS_DOMAIN_ID=0" in script
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in script
    assert "ROS_LOCALHOST_ONLY=0" in script
    assert "<Interfaces>" in cyclone
    assert "<NetworkInterface" in cyclone
    assert "NetworkInterfaceAddress" not in cyclone


def test_cyclonedds_large_cloud_receive_buffer_and_runtime_guard():
    script = (ROOT / "scripts/go2_network_env.sh").read_text()
    cyclone = ET.parse(ROOT / "config/cyclonedds_go2.xml").getroot()
    namespace = {"c": "https://cdds.io/config"}
    internal = cyclone.findall(".//c:Internal", namespace)

    assert len(internal) == 1
    socket_buffer = internal[0].find("c:SocketReceiveBufferSize", namespace)
    assert socket_buffer is not None
    assert socket_buffer.attrib == {"min": "16MiB"}
    assert "go2_required_rmem=16777216" in script
    assert "sysctl -n net.core.rmem_max" in script
    assert "go2_actual_rmem < go2_required_rmem" in script


def test_qos_check_covers_all_cross_distro_sensor_topics():
    source = (ROOT / "scripts/check_go2_qos.sh").read_text()
    for topic in (
        "/utlidar/cloud",
        "/utlidar/imu",
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/image_rect_raw",
        "/camera/camera/color/camera_info",
        "/camera/camera/depth/camera_info",
    ):
        assert topic in source
    assert "/camera/camera/aligned_depth_to_color/image_raw" not in source
    for policy in ("Reliability", "Durability", "History", "Depth"):
        assert policy in source
    assert "ros2 topic info -v" in source
    assert "--once" in source
    assert "expected_reliability" in source
    assert "expected_durability" in source
    assert "required_samples=3" in source
    assert 'for (( sample=1; sample<=required_samples; sample++ ))' in source


def test_reviewed_gate1_config_is_formally_loadable_and_keeps_clock_offset_diagnostic_only():
    config = load_validated_config(ROOT / "config/go2_preflight.yaml")

    assert config["clock"]["measurement_method"] == (
        "multi_frame_ros_header_offset_diagnostic"
    )
    assert "arm_threshold_s" not in config["clock"]
    assert "threshold_metric" not in config["clock"]
    assert config["lidar_timing"]["validated"] is True
    point_semantics = config["lidar_timing"]["point_time_semantics"]
    assert point_semantics["validated"] is True
    assert point_semantics["unit"] == "seconds"
    assert "cloud stamp" in point_semantics["header_stamp_reference"]
    assert config["lidar_timing"]["timestamp_unit"] == 0
    assert (
        config["lidar_timing"]["lidar_imu_measurement_alignment"]["validated"]
        is True
    )
    alignment = config["lidar_timing"]["lidar_imu_measurement_alignment"]
    assert "all-point nearest-IMU" in alignment["method"]
    assert "dynamic" in alignment["method"]
    assert "retry4" in alignment["evidence"]
    assert "yaw" in alignment["evidence"]
    assert "roll" in alignment["evidence"]
    assert "pitch" in alignment["evidence"]
    assert config["lidar_timing"]["scan_line"] == 18
    assert config["lidar_timing"]["scan_rate_hz"] == 16
    assert config["lidar_timing"]["scan_rate_tolerance_hz"] == 1.0
    assert config["lidar_timing"]["point_time_span_min_s"] == 0.055
    assert config["lidar_timing"]["point_time_span_max_s"] == 0.071
    assert config["lidar_timing"]["lidar_imu_offset_jitter_max_s"] == 0.011
    assert config["imu_stationary_thresholds"]["validated"] is True
    sensor_source = (ROOT / "vlm_nav/go2_sensor_preflight.py").read_text()
    assert '"minimum_samples"' in sensor_source


def test_xt16_preflight_uses_stationary_thresholds_and_runtime_checks_integrity_only():
    config = yaml.safe_load(
        (ROOT / "config/go2_preflight_xt16.yaml").read_text(encoding="utf-8")
    )
    timing = config["lidar_timing"]
    assert timing["topic"] == "/lidar_points"
    assert timing["imu_topic"] == "/body_imu"
    assert timing["scan_line"] == 16
    assert timing["scan_rate_hz"] == 10
    assert timing["timestamp_unit"] == 0
    assert timing["point_time_span_min_s"] == 0.05
    assert timing["point_time_span_max_s"] == 0.15
    assert config["converter_health"]["overwrite_ratio_max"] == 0.08
    assert config["imu_stationary_thresholds"]["validated"] is True
    assert config["imu_stationary_thresholds"]["gravity_min_mps2"] == 9.0
    assert "imu_runtime_safety_thresholds" not in config

    preflight_source = (ROOT / "vlm_nav/go2_sensor_preflight.py").read_text()
    safety_source = (ROOT / "vlm_nav/go2_safety_supervisor.py").read_text()
    assert 'timing.get("topic", "/utlidar/cloud")' in preflight_source
    assert "raw_lidar_topic" not in safety_source
    assert '"/cloud_registered_base"' not in safety_source
    assert 'declare_parameter("lidar_frame", "utlidar_lidar")' in safety_source
    assert 'declare_parameter("runtime_safety_validated", False)' not in safety_source
    assert "self.runtime_safety_validated" not in safety_source
    assert "data_ok=finite" in safety_source
    assert 'imu_thresholds["acceleration_abs_max_mps2"]' not in (
        ROOT / "launch/go2_system.launch.py"
    ).read_text()
    assert 'imu_thresholds["gyro_abs_max_radps"]' not in (
        ROOT / "launch/go2_system.launch.py"
    ).read_text()


def test_xt16_formal_launch_defaults_are_explicit_and_downstream_topics_stable():
    launch = (ROOT / "launch/go2_system.launch.py").read_text()
    system = (ROOT / "launch/system.launch.py").read_text()
    for source in (launch, system):
        assert "go2_calibration_xt16.yaml" in source
        assert "spark_fast_lio_go2_xt16.yaml" in source
    assert "/vlm_nav/obstacle_cloud" in launch
    assert "pointcloud_to_laserscan" in launch
    assert "slam_toolbox" in launch
    assert '"wait_go2_fastlio"' in launch
    assert 'topics=("/odometry", "/cloud_registered_base")' in launch
    assert "minimal_cloud_probe" not in launch


def test_go2_nav_launch_forces_real_costmaps_to_wall_time():
    launch = (ROOT / "launch/go2_navigation.launch.py").read_text(encoding="utf-8")
    assert "for server in lifecycle_nodes" in launch
    assert 'for costmap in ("local_costmap", "global_costmap")' in launch
    assert '"use_sim_time"\n        ] = False' in launch


def test_point_time_validation_records_static_and_operator_direction_passes():
    report = (
        ROOT / "docs/validation/2026-08-25_go2_point_time.md"
    ).read_text(encoding="utf-8")

    for contract in (
        "header.stamp + point.time",
        "3,353,118",
        "P99",
        "1.884 ms",
        "静止数值对齐：PASS",
        "动态旋转方向一致性：PASS",
        "yaw",
        "roll",
        "pitch",
        "不参与 timing gate",
    ):
        assert contract in report


def test_go2_runtime_never_uses_robot_odom_as_main_tf_source():
    runtime_files = [
        ROOT / "launch/go2_system.launch.py",
        ROOT / "config/spark_fast_lio_go2.yaml",
        ROOT / "config/robot_go2.yaml",
        ROOT / "config/nav2_go2.yaml",
    ]
    text = "\n".join(path.read_text() for path in runtime_files)

    assert "/utlidar/robot_odom" not in text


def test_sensor_preflight_checks_point_time_imu_and_multiframe_clock():
    source = (ROOT / "vlm_nav/go2_sensor_preflight.py").read_text()
    timing_source = (ROOT / "vlm_nav/go2_preflight.py").read_text()

    for contract in (
        "/utlidar/cloud",
        "/utlidar/imu",
        "point_time_field",
        'required = {"x", "y", "z", self.point_time_field, "ring"}',
        "analyze_cloud_points",
        "ImuHealthAccumulator",
        "TimestampAccumulator",
        "lidar_imu_offset_jitter_s",
        "ReliabilityPolicy.RELIABLE",
        "DurabilityPolicy.VOLATILE",
    ):
        assert contract in source
    for statistic in ("minimum_offset_s", "median_offset_s", "jitter_s"):
        assert statistic in timing_source
    assert "ring_single_value_diagnostic" in source
    assert "ring_single_value_diagnostic\"]" not in source.split(
        'report["healthy"]', 1
    )[-1]


def test_sensor_preflight_shuts_down_outside_ros_callback():
    source = (ROOT / "vlm_nav/go2_sensor_preflight.py").read_text()
    callback = source.split("def check_complete", 1)[1].split(
        "def build_report", 1
    )[0]

    assert "rclpy.shutdown()" not in callback
    assert "self.done = True" in callback
    assert "while rclpy.ok() and not node.done" in source


def test_stage_waiter_requires_received_messages_not_only_graph_presence():
    source = (ROOT / "vlm_nav/go2_readiness_waiter.py").read_text()

    assert "received_counts" in source
    assert "get_topic_names_and_types" in source
    assert "required_message_count" in source
    callback = source.split("def check", 1)[1].split("def main", 1)[0]
    assert "rclpy.shutdown()" not in callback
    assert "while rclpy.ok() and not node.done" in source


class _Timer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


def _waiter_state(*, cloud_count, timeout=30.0, stable_samples=3):
    return SimpleNamespace(
        required_topics=["/cloud_registered_base"],
        required_transforms=[],
        required_message_count=3,
        received_counts={"/cloud_registered_base": cloud_count},
        get_topic_names_and_types=lambda: [
            ("/cloud_registered_base", ["sensor_msgs/msg/PointCloud2"])
        ],
        _ensure_subscription=lambda _topic, _types: None,
        tf_buffer=None,
        started=time.monotonic(),
        timeout=timeout,
        stable_samples=stable_samples,
        consecutive_ready=0,
        exit_code=1,
        done=False,
        timer=_Timer(),
        get_logger=lambda: _Logger(),
    )


def test_waiter_does_not_succeed_before_required_message_count():
    from vlm_nav.go2_readiness_waiter import ReadinessWaiter

    waiter = _waiter_state(cloud_count=2, stable_samples=1)
    ReadinessWaiter.check(waiter)

    assert waiter.done is False
    assert waiter.exit_code == 1


def test_waiter_success_and_timeout_are_fail_closed():
    from vlm_nav.go2_readiness_waiter import ReadinessWaiter

    ready = _waiter_state(cloud_count=3)
    for _ in range(3):
        ReadinessWaiter.check(ready)
    assert ready.done is True
    assert ready.exit_code == 0
    assert ready.timer.cancelled is True

    timed_out = _waiter_state(cloud_count=0, timeout=0.1)
    timed_out.started = time.monotonic() - 1.0
    ReadinessWaiter.check(timed_out)
    assert timed_out.done is True
    assert timed_out.exit_code != 0
    assert timed_out.timer.cancelled is True


def test_staged_transition_only_advances_after_successful_waiter_exit():
    from launch.actions import EmitEvent, ExecuteProcess
    from launch.events import Shutdown

    spec = importlib.util.spec_from_file_location(
        "go2_system_launch", ROOT / "launch/go2_system.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    next_stage = ["next-stage"]
    handler = module._advance_after(ExecuteProcess(cmd=["true"]), next_stage, "test")
    callback = handler.event_handler.__dict__["_OnActionEventBase__on_event"]

    assert callback(SimpleNamespace(returncode=0), None) is next_stage
    failure = callback(SimpleNamespace(returncode=1), None)
    assert len(failure) == 1
    assert isinstance(failure[0], EmitEvent)
    assert isinstance(failure[0].event, Shutdown)


def test_fastlio_waiter_has_formal_lifecycle_and_progress_logging():
    from vlm_nav.go2_readiness_waiter import ReadinessWaiter

    source = (ROOT / "vlm_nav/go2_readiness_waiter.py").read_text()
    assert "FASTLIO_BARE_NODE" not in source
    assert "fastlio_diagnostics" not in source
    assert "rclpy.spin(node)" not in source
    assert "rclpy.spin_once(node, timeout_sec=0.2)" in source

    logger = _Logger()
    waiter = SimpleNamespace(
        received_counts={"/cloud_registered_base": 0},
        required_message_count=3,
        started=time.monotonic(),
        get_logger=lambda: logger,
    )
    for _ in range(3):
        ReadinessWaiter._on_message(waiter, "/cloud_registered_base")
    assert any("cloud readiness 1/3" in message for message in logger.messages)
    assert any("cloud readiness 2/3" in message for message in logger.messages)
    assert any("cloud readiness 3/3" in message for message in logger.messages)
    assert any("cloud readiness reached 3/3 after" in message for message in logger.messages)
