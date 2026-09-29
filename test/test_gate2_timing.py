import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/measure_gate2_timing.py"
SPEC = importlib.util.spec_from_file_location("gate2_timing", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_stream_separates_middleware_receive_from_callback_execution():
    stream = MODULE.Stream()

    stream.add(10_000_000_000, received_ns=12_000_000_000, execute_ns=12_250_000_000)
    report = stream.report()

    assert report["middleware_receive_minus_header_stamp_sec"]["median"] == 2.0
    assert report["callback_execute_minus_header_stamp_sec"]["median"] == 2.25
    assert report["callback_queue_delay_sec"]["median"] == 0.25


def test_pose_drift_reports_translation_and_wrapped_yaw():
    drift = MODULE.pose_drift(
        [(0.0, 0.0, 0.0, 3.13), (0.03, 0.04, 0.0, -3.13)]
    )

    assert drift["translation_m"] == 0.05
    assert 1.3 < drift["yaw_deg"] < 1.4


def test_stats_includes_nearest_rank_p95():
    assert MODULE.stats(list(range(1, 21)))["p95"] == 19
