import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/measure_hesai_fastlio_converter.py"
SPEC = importlib.util.spec_from_file_location("hesai_converter_measurement", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_empty_report_fails_converter_gate():
    report = {
        "input": {"count": 0, "rollback": 0},
        "output": {"count": 0, "rollback": 0},
        "output_point_count": {"count": 0, "min": None},
        "output_schema_ok": False,
        "output_rings_ok": False,
        "output_finite_time_ok": False,
        "output_time_span_sec": {"min": None, "max": None},
        "header_stamp_matches_input": False,
    }

    checks, passed = MODULE.check_report(report)

    assert not passed
    assert checks["output_nonempty"] is False

