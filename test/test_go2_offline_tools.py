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
    assert "/camera/camera/aligned_depth_to_color/image_raw" in output
    assert "/api/sport/request" not in output
    assert "/cmd_vel" not in output


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
