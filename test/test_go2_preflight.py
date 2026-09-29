import math
import time

import pytest
import yaml

import vlm_nav.go2_preflight as go2_preflight
from vlm_nav.go2_preflight import (
    CameraHealthAccumulator,
    FormalPreflightError,
    CalibrationError,
    InterfaceSelectionError,
    ImuHealthAccumulator,
    RawImuAccumulator,
    TimestampAccumulator,
    analyze_cloud_points,
    nearest_stream_offsets,
    require_calibrated,
    select_go2_interface,
    validate_formal_preflight_report,
)
from vlm_nav.go2_safety_supervisor import StreamHealth
from vlm_nav.go2_sensor_preflight import Go2SensorPreflight, load_validated_config


def test_network_interface_auto_selection_fails_closed():
    with pytest.raises(InterfaceSelectionError, match="no interface"):
        select_go2_interface({"lo": ["127.0.0.1"]})

    assert select_go2_interface(
        {"eno1": ["192.168.123.222"], "lo": ["127.0.0.1"]}
    ) == "eno1"

    with pytest.raises(InterfaceSelectionError, match="multiple interfaces"):
        select_go2_interface(
            {"eno1": ["192.168.123.222"], "usb0": ["192.168.123.99"]}
        )


def test_explicit_interface_override_must_exist_and_have_go2_subnet():
    interfaces = {"eno1": ["192.168.123.222"], "wlan0": ["10.0.0.2"]}

    assert select_go2_interface(interfaces, override="eno1") == "eno1"
    with pytest.raises(InterfaceSelectionError, match="does not exist"):
        select_go2_interface(interfaces, override="missing")
    with pytest.raises(InterfaceSelectionError, match="192.168.123"):
        select_go2_interface(interfaces, override="wlan0")


def test_timestamp_statistics_detect_backward_and_separate_offset_from_jitter():
    stats = TimestampAccumulator()
    stats.add(header_stamp=10.000, local_receive_time=10.020)
    stats.add(header_stamp=10.010, local_receive_time=10.032)
    stats.add(header_stamp=10.020, local_receive_time=10.041)

    result = stats.summary()
    assert result["minimum_offset_s"] == pytest.approx(0.020)
    assert result["median_offset_s"] == pytest.approx(0.021)
    assert result["jitter_s"] == pytest.approx(0.002)
    assert result["timestamp_backward_count"] == 0

    stats.add(header_stamp=9.0, local_receive_time=10.050)
    assert stats.summary()["timestamp_backward_count"] == 1


def test_runtime_stream_freshness_does_not_gate_on_header_to_wall_offset():
    stream = StreamHealth()
    stream.observe(
        time.time() + 3600.0,
        data_ok=True,
    )

    assert stream.healthy(stream.last_receipt + 0.01, maximum_age=0.5) is True


def test_runtime_stream_still_rejects_timestamp_backward():
    stream = StreamHealth()
    stream.observe(time.time() + 10.0, data_ok=True)
    stream.observe(time.time() + 9.0, data_ok=True)

    assert stream.healthy(stream.last_receipt + 0.01, maximum_age=0.5) is False


def test_formal_timing_requires_point_semantics_and_measurement_alignment(tmp_path):
    config = {
        "lidar_timing": {
            "validated": False,
            "point_time_semantics": {
                "validated": False,
                "header_stamp_reference": "TBD / requires measurement",
                "point_time_reference": "TBD / requires measurement",
                "unit": "TBD / requires measurement",
            },
            "lidar_imu_measurement_alignment": {
                "validated": False,
                "method": "TBD / requires measurement",
                "evidence": "TBD / requires measurement",
            },
        },
        "imu_stationary_thresholds": {"validated": True},
    }
    path = tmp_path / "go2_preflight.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match="point_time_semantics"):
        load_validated_config(path)

    config["lidar_timing"]["point_time_semantics"] = {
        "validated": True,
        "header_stamp_reference": "scan start",
        "point_time_reference": "offset from header stamp",
        "unit": "seconds",
    }
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match="lidar_imu_measurement_alignment"):
        load_validated_config(path)


def test_formal_sensor_health_ignores_absolute_header_to_wall_offset():
    preflight = object.__new__(Go2SensorPreflight)
    preflight.config = {
        "clock": {
            "measurement_method": "multi_frame_ros_header_offset_diagnostic",
            "threshold_metric": "minimum_offset_s",
            "arm_threshold_s": 1.0e-6,
            "minimum_samples": 2,
        }
    }
    preflight.diagnostic_only = False
    preflight.expected_scan_rate = 2.0
    preflight.scan_rate_tolerance = 0.01
    preflight.point_time_span_min = 0.49
    preflight.point_time_span_max = 0.51
    preflight.lidar_imu_jitter_max = 0.01
    preflight.lidar_timestamps = TimestampAccumulator()
    preflight.imu_timestamps = TimestampAccumulator()
    for header_stamp, lidar_receive, imu_receive in (
        (100.0, 1100.0, -900.0),
        (100.5, 1100.5, -899.5),
    ):
        preflight.lidar_timestamps.add(
            header_stamp=header_stamp, local_receive_time=lidar_receive
        )
        preflight.imu_timestamps.add(
            header_stamp=header_stamp + 0.001, local_receive_time=imu_receive
        )
    preflight.cloud_reports = [
        {
            "healthy": True,
            "point_time_span_s": 0.5,
            "ring_single_value_diagnostic": True,
        },
        {
            "healthy": True,
            "point_time_span_s": 0.5,
            "ring_single_value_diagnostic": True,
        },
    ]
    preflight.cloud_header_stamps = [100.0, 100.5]
    preflight.imu_header_stamps = [100.001, 100.501]
    preflight.cloud_point_time_frames = [
        (100.0, [0.0, 0.25, 0.5]),
        (100.5, [0.0]),
    ]
    preflight.cloud_receipts = [1.0, 1.5]
    preflight.cloud_errors = []
    preflight.raw_imu = RawImuAccumulator()
    preflight.raw_imu.add(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    preflight.imu_health = ImuHealthAccumulator(
        gravity_min=8.0,
        gravity_max=11.0,
        stationary_gyro_max=0.2,
        acceleration_abs_max=30.0,
        gyro_abs_max=5.0,
        jump_max=5.0,
    )
    preflight.imu_health.add(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

    report = preflight.build_report()

    assert report["lidar_clock"]["median_offset_s"] == pytest.approx(1000.0)
    assert report["imu_clock"]["median_offset_s"] == pytest.approx(-1000.001)
    assert report["point_time_alignment"]["point_count"] == 4
    assert report["point_time_alignment"]["matched_count"] == 3
    assert report["healthy"] is True


def test_imu_health_rejects_nonfinite_extreme_and_unreasonable_stationary_data():
    health = ImuHealthAccumulator(
        gravity_min=8.0,
        gravity_max=11.0,
        stationary_gyro_max=0.2,
        acceleration_abs_max=30.0,
        gyro_abs_max=5.0,
        jump_max=5.0,
    )
    health.add(0.0, 0.0, 9.81, 0.01, -0.01, 0.02)
    assert health.summary()["healthy"] is True

    health.add(math.nan, 0.0, 9.81, 0.0, 0.0, 0.0)
    assert health.summary()["nonfinite_count"] == 1
    assert health.summary()["healthy"] is False

    extreme = ImuHealthAccumulator(
        gravity_min=8.0,
        gravity_max=11.0,
        stationary_gyro_max=0.2,
        acceleration_abs_max=30.0,
        gyro_abs_max=5.0,
        jump_max=5.0,
    )
    extreme.add(0.0, 0.0, 1000.0, 0.0, 0.0, 0.0)
    assert extreme.summary()["extreme_count"] == 1
    assert extreme.summary()["healthy"] is False


def test_calibration_gate_never_treats_tbd_or_identity_as_calibrated():
    with pytest.raises(CalibrationError, match="lidar_imu_extrinsic"):
        require_calibrated(
            "lidar_imu_extrinsic",
            {
                "calibrated": False,
                "translation": "TBD / requires measurement",
                "rotation_matrix": "TBD / requires measurement",
            },
        )


def test_cloud_health_uses_finite_xyz_and_point_time_not_ring_as_blocker():
    result = analyze_cloud_points(
        [
            (1.0, 2.0, 3.0, 0.000, 1),
            (1.1, 2.1, 3.1, 0.031, 1),
            (1.2, 2.2, 3.2, 0.063, 1),
        ]
    )

    assert result["healthy"] is True
    assert result["finite_xyz"] is True
    assert result["point_time_varies"] is True
    assert result["point_time_span_s"] == pytest.approx(0.063)
    assert result["ring_distribution"] == {1: 3}
    assert result["ring_single_value_diagnostic"] is True


def test_cloud_health_rejects_nonfinite_xyz_and_constant_point_time():
    result = analyze_cloud_points(
        [(1.0, 2.0, math.inf, 0.0, 1), (1.0, 2.0, 3.0, 0.0, 2)]
    )

    assert result["healthy"] is False
    assert result["finite_xyz"] is False
    assert result["point_time_varies"] is False


def test_raw_imu_diagnostics_report_values_without_inventing_thresholds():
    raw = RawImuAccumulator()
    raw.add(0.0, 0.0, 9.81, 0.01, -0.02, 0.03)
    raw.add(0.1, 0.0, 9.80, 0.02, -0.01, 0.02)

    report = raw.summary()
    assert report["sample_count"] == 2
    assert report["nonfinite_count"] == 0
    assert report["acceleration_norm_mps2"]["median"] == pytest.approx(9.805255, rel=1e-4)
    assert report["gyro_norm_radps"]["maximum"] > 0.0
    assert report["maximum_acceleration_jump_mps2"] > 0.0
    assert "healthy" not in report


def test_lidar_imu_offset_ignores_uncovered_collection_window_edges():
    offsets = nearest_stream_offsets(
        reference_stamps=[9.9, 10.005, 10.015, 10.1],
        high_rate_stamps=[10.0, 10.01, 10.02],
    )

    assert [abs(value) for value in offsets] == pytest.approx([0.005, 0.005])

    with pytest.raises(CalibrationError, match="identity"):
        require_calibrated(
            "camera_extrinsic",
            {
                "calibrated": True,
                "xyz": [0.0, 0.0, 0.0],
                "rpy": [0.0, 0.0, 0.0],
                "source": "measured fixture",
            },
        )

def test_formal_preflight_report_must_be_healthy_and_fresh():
    report = {"healthy": True, "formal_preflight": "PASS", "generated_at_unix_s": 100.0}
    validate_formal_preflight_report(report, now_unix_s=105.0, max_age_s=10.0)

    with pytest.raises(FormalPreflightError, match="healthy"):
        validate_formal_preflight_report(
            {**report, "healthy": False}, now_unix_s=105.0, max_age_s=10.0
        )
    with pytest.raises(FormalPreflightError, match="stale"):
        validate_formal_preflight_report(report, now_unix_s=120.0, max_age_s=10.0)
    with pytest.raises(FormalPreflightError, match="diagnostic"):
        validate_formal_preflight_report(
            {**report, "formal_preflight": "NOT_EVALUATED: diagnostic"},
            now_unix_s=105.0,
            max_age_s=10.0,
        )


def test_depth_units_report_is_the_only_autonomy_scale_authority():
    report = {
        "verified": True,
        "passed": True,
        "profile": {
            "width": 640,
            "height": 480,
            "fps": 15,
            "enable_sync": True,
            "align_depth": False,
        },
        "depth_topic": "/camera/camera/depth/image_rect_raw",
        "device_scale": 0.0010000000474974513,
        "meters_per_unit": 0.001,
        "raw_value": 1000,
        "measured_z_m": 0.98,
        "converted_m": 1.0,
        "raw_times_device_scale_m": 1.0000000474974513,
        "absolute_error_m": 0.02,
        "allowed_error_m": 0.05,
        "snapshot_sha256": "abc123",
    }

    assert go2_preflight.validate_depth_units_report(
        report,
        expected_width=640,
        expected_height=480,
        expected_fps=15,
        expected_depth_topic="/camera/camera/depth/image_rect_raw",
        expected_device_scale=0.0010000000474974513,
    ) == pytest.approx(0.001)

    for broken in (
        {**report, "verified": False},
        {**report, "profile": {**report["profile"], "fps": 30}},
        {**report, "absolute_error_m": 0.06},
        {**report, "device_scale": 0.002},
        {**report, "meters_per_unit": 0.002},
        {**report, "converted_m": 2.0},
    ):
        with pytest.raises(CalibrationError):
            go2_preflight.validate_depth_units_report(
                broken,
                expected_width=640,
                expected_height=480,
                expected_fps=15,
                expected_depth_topic="/camera/camera/depth/image_rect_raw",
                expected_device_scale=0.0010000000474974513,
            )


def test_rgbd_sync_health_requires_a_successful_short_window_not_one_pair():
    rgb = [100.0 + index / 15.0 for index in range(10)]
    mostly_matched_depth = [stamp + 0.01 for stamp in rgb[:9]] + [200.0]
    one_lucky_depth = [rgb[0] + 0.01] + [200.0 + index for index in range(9)]

    healthy = go2_preflight.rgbd_sync_health(
        rgb, mostly_matched_depth, slop_s=0.05, minimum_samples=10, minimum_rate=0.9
    )
    unhealthy = go2_preflight.rgbd_sync_health(
        rgb, one_lucky_depth, slop_s=0.05, minimum_samples=10, minimum_rate=0.9
    )

    assert healthy == {"healthy": True, "matched": 9, "total": 10, "rate": 0.9}
    assert unhealthy == {"healthy": False, "matched": 1, "total": 10, "rate": 0.1}


def test_raw_camera_health_matches_each_image_to_its_own_camera_info():
    health = CameraHealthAccumulator(minimum_samples=1, sync_slop_s=0.05)
    health.add_rgb(
        stamp=1.0, local_receive_time=1.01, width=640, height=480,
        encoding="rgb8", step=1920, data_size=921600,
    )
    health.add_depth(
        stamp=1.01, local_receive_time=1.02, width=848, height=480,
        encoding="16UC1", step=1696, data_size=814080,
    )
    health.add_color_camera_info(
        stamp=1.0, local_receive_time=1.01, width=640, height=480,
        frame_id="camera_color_optical_frame",
        k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
    )
    health.add_depth_camera_info(
        stamp=1.01, local_receive_time=1.02, width=848, height=480,
        frame_id="camera_depth_optical_frame",
        k=[420.0, 0.0, 424.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0],
    )

    report = health.summary()

    assert report["healthy"] is True
    assert report["profiles_match_camera_info"] is True
    assert report["color_camera_info_count"] == 1
    assert report["depth_camera_info_count"] == 1


def test_formal_raw_camera_health_enforces_rate_duplicates_and_sync_rate():
    health = CameraHealthAccumulator(
        minimum_samples=10,
        sync_slop_s=0.05,
        minimum_sync_rate=0.9,
        minimum_rate_hz=14.0,
        maximum_rate_hz=16.0,
        maximum_duplicate_rate=0.01,
    )
    intrinsics = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    for index in range(16):
        stamp = 100.0 + index / 15.0
        health.add_rgb(
            stamp=stamp, local_receive_time=stamp + 0.08,
            width=640, height=480, encoding="rgb8", step=1920,
            data_size=921600,
        )
        health.add_depth(
            stamp=stamp + 0.01, local_receive_time=stamp + 0.09,
            width=640, height=480, encoding="16UC1", step=1280,
            data_size=614400,
        )
        health.add_color_camera_info(
            stamp=stamp, local_receive_time=stamp + 0.08,
            width=640, height=480, frame_id="camera_color_optical_frame",
            k=intrinsics,
        )
        health.add_depth_camera_info(
            stamp=stamp + 0.01, local_receive_time=stamp + 0.09,
            width=640, height=480, frame_id="camera_depth_optical_frame",
            k=intrinsics,
        )

    report = health.summary()

    assert report["healthy"] is True
    assert report["rgb_stream"]["unique_hz"] == pytest.approx(15.0)
    assert report["depth_stream"]["unique_hz"] == pytest.approx(15.0)
    assert report["rgb_stream"]["duplicate_rate"] == 0.0
    assert report["sync_window"]["rate"] == 1.0


def test_raw_camera_preflight_rejects_float_depth_encoding():
    health = CameraHealthAccumulator(minimum_samples=1, sync_slop_s=0.05)
    intrinsics = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    health.add_rgb(
        stamp=1.0, local_receive_time=1.01, width=2, height=2,
        encoding="rgb8", step=6, data_size=12,
    )
    health.add_depth(
        stamp=1.0, local_receive_time=1.01, width=2, height=2,
        encoding="32FC1", step=8, data_size=16,
    )
    health.add_color_camera_info(
        stamp=1.0, local_receive_time=1.01, width=2, height=2,
        frame_id="color", k=intrinsics,
    )
    health.add_depth_camera_info(
        stamp=1.0, local_receive_time=1.01, width=2, height=2,
        frame_id="depth", k=intrinsics,
    )

    report = health.summary()

    assert report["healthy"] is False
    assert report["depth_metadata_healthy"] is False
def test_camera_health_requires_raw_rgbd_dual_info_and_monotonic_timestamps():
    health = CameraHealthAccumulator(minimum_samples=3, sync_slop_s=0.05)
    for index in range(3):
        stamp = 100.0 + index * 0.033
        health.add_rgb(
            stamp=stamp,
            local_receive_time=stamp + 0.01,
            width=640,
            height=480,
            encoding="rgb8",
            step=1920,
            data_size=921600,
        )
        health.add_depth(
            stamp=stamp + 0.005,
            local_receive_time=stamp + 0.015,
            width=640,
            height=480,
            encoding="16UC1",
            step=1280,
            data_size=614400,
        )
        health.add_color_camera_info(
            stamp=stamp,
            local_receive_time=stamp + 0.01,
            width=640,
            height=480,
            frame_id="camera_color_optical_frame",
            k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
        )
        health.add_depth_camera_info(
            stamp=stamp + 0.005,
            local_receive_time=stamp + 0.015,
            width=640,
            height=480,
            frame_id="camera_depth_optical_frame",
            k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
        )

    report = health.summary()
    assert report["healthy"] is True
    assert report["rgb_count"] == 3
    assert report["depth_count"] == 3
    assert report["camera_info_count"] == 3
    assert report["depth_camera_info_count"] == 3
    assert report["rgb_depth_offset_s"]["maximum_absolute"] == pytest.approx(0.005)


def test_camera_health_rejects_raw_unaligned_dimensions_and_backward_stamp():
    health = CameraHealthAccumulator(minimum_samples=1, sync_slop_s=0.05)
    health.add_rgb(
        stamp=2.0, local_receive_time=2.0, width=640, height=480,
        encoding="rgb8", step=1920, data_size=921600,
    )
    health.add_rgb(
        stamp=1.0, local_receive_time=2.1, width=640, height=480,
        encoding="rgb8", step=1920, data_size=921600,
    )
    health.add_depth(
        stamp=2.0, local_receive_time=2.0, width=848, height=480,
        encoding="16UC1", step=1696, data_size=814080,
    )
    health.add_camera_info(
        stamp=2.0, local_receive_time=2.0, width=640, height=480,
        frame_id="camera_color_optical_frame",
        k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
    )

    report = health.summary()
    assert report["healthy"] is False
    assert report["rgb_clock"]["timestamp_backward_count"] == 1
    assert report["dimensions_aligned"] is False
