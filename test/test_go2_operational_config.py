from pathlib import Path
import os
import rclpy

import yaml

from vlm_nav.go2_sensor_preflight import load_validated_config


ROOT = Path(__file__).resolve().parents[1]


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


def test_qos_check_covers_all_cross_distro_sensor_topics():
    source = (ROOT / "scripts/check_go2_qos.sh").read_text()
    for topic in (
        "/utlidar/cloud",
        "/utlidar/imu",
        "/camera/camera/color/image_raw",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/color/camera_info",
    ):
        assert topic in source
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
        '"x", "y", "z", "time", "ring"',
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

    assert "get_message" in source
    assert "received_counts" in source
    assert "get_topic_names_and_types" in source
    assert "required_message_count" in source
    callback = source.split("def check", 1)[1].split("def main", 1)[0]
    assert "rclpy.shutdown()" not in callback
    assert "while rclpy.ok() and not node.done" in source
