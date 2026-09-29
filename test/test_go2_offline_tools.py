import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_reconstructed_point_alignment_matches_every_covered_point():
    assert importlib.util.find_spec("vlm_nav.go2_point_time") is not None
    from vlm_nav.go2_point_time import analyze_reconstructed_point_alignment

    result = analyze_reconstructed_point_alignment(
        [(10.0, [0.0, 0.004, 0.009]), (10.010, [0.0, 0.005])],
        [9.999, 10.003, 10.007, 10.011, 10.015],
    )

    assert result["point_count"] == 5
    assert result["matched_count"] == 5
    assert result["outside_imu_coverage_count"] == 0
    assert result["absolute_offset_s"]["median"] == pytest.approx(0.001)
    assert result["absolute_offset_s"]["maximum"] == pytest.approx(0.002)


def test_candidate_generator_records_observed_envelope_without_validating_gate():
    assert importlib.util.find_spec("vlm_nav.go2_evidence") is not None
    from vlm_nav.go2_evidence import derive_candidate_config

    point_report = {
        "counts": {"cloud_frames": 900, "imu_messages": 15000},
        "rates_and_intervals": {"cloud_rate_from_header_hz": 15.4},
        "point_time_raw": {
            "minimum_s": {"minimum": 0.0},
            "maximum_s": {"minimum": 0.060, "median": 0.063, "maximum": 0.066},
        },
        "lidar_imu_alignment": {
            "all_reconstructed_points": {
                "absolute_offset_s": {"p99": 0.0019, "maximum": 0.0048},
                "matched_count": 3_300_000,
            },
            "header_nearest_imu_minus_header_s": {"range": 0.0077},
        },
    }
    raw_reports = [
        {
            "lidar_hz": 15.37,
            "lidar_imu_offset_jitter_s": 0.0068,
            "point_time_span_s": {"minimum": 0.061, "maximum": 0.066},
            "imu_raw": {
                "acceleration_norm_mps2": {"minimum": 10.1, "maximum": 10.6},
                "gyro_norm_radps": {"maximum": 0.023},
                "maximum_acceleration_abs_mps2": 10.1,
                "maximum_gyro_abs_radps": 0.021,
                "maximum_acceleration_jump_mps2": 0.49,
            },
        }
    ]

    result = derive_candidate_config(point_report, raw_reports)

    assert result["candidate_only"] is True
    assert result["validated"] is False
    assert result["lidar_timing"]["scan_line"] == 18
    assert result["lidar_timing"]["scan_rate_hz"] == 16
    assert result["observed"]["cloud_rate_hz"] == pytest.approx([15.37, 15.4])
    assert result["observed"]["point_time_span_s"] == {
        "minimum": 0.060,
        "median": 0.063,
        "maximum": 0.066,
    }
    assert result["lidar_timing"]["point_time_span_min_s"] < 0.060
    assert result["lidar_timing"]["point_time_span_max_s"] > 0.066
    assert result["imu_stationary_thresholds"]["gravity_min_mps2"] < 10.1
    assert result["imu_stationary_thresholds"]["gravity_max_mps2"] > 10.6


def test_readonly_collection_dry_run_never_contains_motion_topics(tmp_path):
    script = ROOT / "scripts/collect_go2_readonly.sh"
    assert script.exists()

    completed = subprocess.run(
        [
            "bash",
            str(script),
            "--dry-run",
            "--output-dir",
            str(tmp_path / "evidence"),
            "--static-duration",
            "30",
            "--dynamic-duration",
            "45",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    output = completed.stdout
    assert "/utlidar/cloud /utlidar/imu" in output
    assert "static_lidar_imu" in output
    assert "dynamic_lidar_imu" in output
    assert "/camera/camera/depth/image_rect_raw" in output
    assert "/camera/camera/aligned_depth_to_color/image_raw" not in output
    assert "/api/sport/request" not in output
    assert "/cmd_vel" not in output


def test_preflight_script_is_readonly_and_cleans_up_children():
    script = ROOT / "scripts/run_go2_preflight.sh"
    subprocess.run(["bash", "-n", str(script)], check=True)
    source = script.read_text()

    assert "trap cleanup EXIT" in source
    assert source.count("setsid ros2 run") == 2
    assert 'kill -KILL -- "-${pid}"' in source
    assert 'kill -0 -- "-$1"' in source
    assert " +  " not in source
    assert "source /opt/ros/humble/setup.bash" not in source
    assert 'source "${script_dir}/common.sh"' in source
    assert "bridge_dry_run" not in source
    assert "/api/sport/request" not in source
    assert "/cmd_vel" not in source
    assert "--duration=30" in source
    assert "within 600 seconds" in source

    launch = (ROOT / "launch/go2_system.launch.py").read_text()
    supervisor = (ROOT / "vlm_nav/go2_safety_supervisor.py").read_text()
    assert '"sensor_preflight_max_age_s", default_value="600.0"' in launch
    assert '"formal_report_max_age_s", 600.0' in supervisor


def test_sport_publisher_inspector_is_readonly_and_requires_disarmed():
    script = ROOT / "scripts/inspect_go2_sport_publishers.sh"
    subprocess.run(["bash", "-n", str(script)], check=True)
    source = script.read_text()

    assert "go2_bridge_state" in source
    assert "DISARMED" in source
    assert "ros2 topic info /api/sport/request -v" in source
    assert "ss -uapne" in source
    assert "lsof -nP -iUDP" in source
    assert "<Verbosity>finest</Verbosity>" in source
    assert "ros2 topic pub" not in source
    assert "ros2 service call" not in source


def test_manual_nav2_scripts_keep_arm_and_emergency_stop_fail_closed():
    manual = ROOT / "scripts/manualnav2.sh"
    stop = ROOT / "scripts/stopnav2.sh"
    for script in (manual, stop):
        subprocess.run(["bash", "-n", str(script)], check=True)

    source = manual.read_text()
    assert "run_go2_preflight.sh" in source
    assert "target_stage:=nav2" in source
    assert "start_bridge:=false" in source
    assert "record_go2_nav2_manual.sh" in source
    assert "inspect_go2_sport_publishers.sh" in source
    assert "/twist_to_go2_sport_bridge/arm" in source
    assert "/navigate_to_pose/_action/cancel_goal" in source
    assert "stopnav2.sh" in source
    assert "trap cleanup EXIT" in source
    assert 'kill -KILL -- "-${pid}"' in source
    assert 'kill -0 -- "-$1"' in source
    assert "wait_for_nav2" in source
    assert "confirm START_NAV" in source
    assert source.index("wait_for_nav2") < source.index("confirm START_NAV")
    assert "wait_for_preflight_inputs" in source
    assert "confirm RETRY_PREFLIGHT" in source
    assert source.index("wait_for_preflight_inputs") < source.index(
        'echo "Reusing a fresh formal preflight report, or running the 30-second preflight..."'
    )

    stop_source = stop.read_text()
    assert "/twist_to_go2_sport_bridge/disarm" in stop_source
    assert "/cmd_vel" not in stop_source
    assert "/api/sport" not in stop_source

    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "scripts/manualnav2.sh" in cmake
    assert "scripts/stopnav2.sh" in cmake


def test_footprint_sweep_debug_is_readonly_and_uses_go2_rectangle():
    script = ROOT / "scripts/go2_footprint_sweep_debug.py"
    spec = importlib.util.spec_from_file_location("go2_footprint_sweep_debug", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.yaw_offsets_degrees() == list(range(-90, 91, 5))
    expected = [
        (0.38, 0.18),
        (0.38, -0.18),
        (-0.38, -0.18),
        (-0.38, 0.18),
        (0.38, 0.18),
    ]
    for point, (expected_x, expected_y) in zip(module.rectangle_points(), expected):
        assert point.x == pytest.approx(expected_x)
        assert point.y == pytest.approx(expected_y)
    source = script.read_text()
    assert 'MarkerArray, "/vlm_nav/go2_footprint_sweep"' in source
    assert '"map", "base_link", Time()' in source
    for forbidden in ("cmd_vel", "/api/sport", "create_client", "set_parameters"):
        assert forbidden not in source


def test_gate5_pulse_arms_before_nonzero_and_is_bounded():
    source = (ROOT / "vlm_nav/go2_gate5_pulse.py").read_text()

    assert 'default=0.20' in source
    assert 'args.speed <= 0.40' in source
    assert 'default=0.50' in source
    assert '"--settle-duration"' in source
    assert 'args.settle_duration <= 5.0' in source
    assert 'default=2.0' in source
    assert 'args.duration <= 2.0' in source
    assert 'args.duration <= 10.0' in source
    assert '"linear": (' in source
    assert '"yaw": (' in source
    assert '"counterclockwise", 0.0, args.yaw_rate' in source
    assert '"clockwise", 0.0, -args.yaw_rate' in source
    assert "linear/yaw sequences require --execute-extended" in source
    assert source.index("response = _call(node, arm)") < source.index(
        '"forward", args.speed, 0.0'
    )
    assert "publisher.publish(Twist())" in source
    assert "response = _call(node, disarm)" in source
    assert "ros2\", \"bag\", \"record" in source


def test_gate5_movetest_reuses_fail_closed_pulse_tool():
    script = ROOT / "scripts/movetest.sh"
    subprocess.run(["bash", "-n", str(script)], check=True)
    source = script.read_text()

    assert '"--execute"' in source
    assert "require_disarmed_ready" in source
    assert "--speed 0.40" in source
    assert "--duration 6" in source
    assert "--yaw-rate 0.50" in source
    assert "--settle-duration 3" in source
    assert source.count("ros2 run vlm_nav go2_gate5_pulse") == 2


def test_candidate_cli_writes_yaml_and_keeps_validated_false(tmp_path):
    assert importlib.util.find_spec("vlm_nav.go2_evidence") is not None
    from vlm_nav.go2_evidence import main

    point = tmp_path / "point.json"
    raw = tmp_path / "raw.json"
    output = tmp_path / "candidates.yaml"
    point.write_text(
        json.dumps(
            {
                "counts": {"cloud_frames": 1, "imu_messages": 2},
                "rates_and_intervals": {"cloud_rate_from_header_hz": 15.4},
                "point_time_raw": {
                    "maximum_s": {
                        "minimum": 0.061,
                        "median": 0.063,
                        "maximum": 0.066,
                    }
                },
                "lidar_imu_alignment": {
                    "all_reconstructed_points": {
                        "absolute_offset_s": {"p99": 0.002, "maximum": 0.005},
                        "matched_count": 100,
                    },
                    "header_nearest_imu_minus_header_s": {"range": 0.008},
                },
            }
        ),
        encoding="utf-8",
    )
    raw.write_text(
        json.dumps(
            {
                "lidar_hz": 15.3,
                "lidar_imu_offset_jitter_s": 0.007,
                "point_time_span_s": {"minimum": 0.060, "maximum": 0.067},
                "imu_raw": {
                    "acceleration_norm_mps2": {"minimum": 10.0, "maximum": 10.7},
                    "gyro_norm_radps": {"maximum": 0.025},
                    "maximum_acceleration_abs_mps2": 10.2,
                    "maximum_gyro_abs_radps": 0.022,
                    "maximum_acceleration_jump_mps2": 0.5,
                },
            }
        ),
        encoding="utf-8",
    )

    assert main(["--point-report", str(point), "--raw-report", str(raw), "--output", str(output)]) == 0
    text = output.read_text(encoding="utf-8")
    assert "candidate_only: true" in text
    assert "validated: false" in text
