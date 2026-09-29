"""Exercise the installed official geometry on a known one-metre plane."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location(
    "raw_depth_probe", Path(__file__).parents[1] / "scripts/raw_depth_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
LIBRARY = "/usr/local/lib/librealsense2.so.2.53.1"


def snapshot():
    info = {"width": 64, "height": 48, "k": [100., 0., 32., 0., 100., 24., 0., 0., 1.], "d": [0.] * 5}
    # Color camera sits 2 cm right of depth camera: color u=30 maps to depth u=32 at z=1.
    dc = np.eye(4)
    dc[0, 3] = -.02
    md = np.eye(4)
    md[:3, 3] = [1., 2., 3.]
    meta = {"color_info": info, "depth_info": info, "rgb_stamp_ns": 10, "depth_stamp_ns": 10,
            "depth_to_color": dc.tolist(), "color_to_depth": np.linalg.inv(dc).tolist(),
            "T_map_depth_optical": md.tolist()}
    return np.zeros((48, 64, 3), np.uint8), np.full((48, 64), 1000, np.uint16), meta


@pytest.mark.skipif(not Path(LIBRARY).exists(), reason="Official librealsense library required")
def test_official_mapping_deprojection_and_saved_tf():
    sdk = probe.Geometry(LIBRARY)
    original = snapshot()
    for u in (20., 30., 40.):
        result = sdk.project(original, [u, 24.], .001, .2, 5.)
        assert result["depth_pixel"] == [int(u + 2), 24]
        assert np.allclose(result["depth_optical_point"], [(u - 30) / 100., 0., 1.], atol=1e-5)
        assert np.allclose(result["map_point"], [1 + (u - 30) / 100., 2., 4.], atol=1e-5)
    newer = snapshot()
    newer[1][:] = 2000
    newer[2]["T_map_depth_optical"][0][3] = 99
    assert sdk.project(original, [30, 24], .001, .2, 5.)["map_point"] == pytest.approx([1, 2, 4])


@pytest.mark.skipif(not Path(LIBRARY).exists(), reason="Official librealsense library required")
def test_official_mapping_accepts_immutable_camera_info_sequences():
    rgb, raw, meta = snapshot()
    for name in ("color_info", "depth_info"):
        meta[name]["k"] = tuple(meta[name]["k"])
        meta[name]["d"] = tuple(meta[name]["d"])

    result = probe.Geometry(LIBRARY).project((rgb, raw, meta), [30, 24], .001, .2, 5.)

    assert result["depth_pixel"] == [32, 24]
    assert result["depth_m"] == pytest.approx(1.0)


def test_npz_snapshot_is_independent_and_readonly(tmp_path):
    import json
    rgb, raw, meta = snapshot()
    path = tmp_path / "snapshot.npz"
    np.savez(path, rgb=rgb, raw_depth=raw, metadata=json.dumps(meta))
    saved = probe.load_snapshot(path)
    before = probe.fingerprint(saved)
    raw[:] = 999
    meta["T_map_depth_optical"][0][3] = 99
    assert probe.fingerprint(saved) == before
    with pytest.raises(ValueError):
        saved[1][0, 0] = 5


def test_past_tf_extrapolation_is_irrecoverable():
    assert probe.past_extrapolation("Lookup would require extrapolation into the past")
    assert not probe.past_extrapolation("Lookup would require extrapolation into the future")


def test_units_cli_writes_a_profile_bound_verification_report(tmp_path):
    import json
    import subprocess
    import sys
    rgb, raw, meta = snapshot()
    path = tmp_path / "units_snapshot.npz"
    np.savez(path, rgb=rgb, raw_depth=raw, metadata=json.dumps(meta))
    report_path = tmp_path / "units.json"

    subprocess.run(
        [
            sys.executable,
            str(Path(probe.__file__)),
            "units",
            str(path),
            "--pixel", "32", "24",
            "--distance-m", "1.0",
            "--device-scale", "0.0010000000474974513",
            "--meters-per-unit", "0.001",
            "--fps", "15",
            "--output", str(report_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    report = json.loads(report_path.read_text())
    assert report["verified"] is True
    assert report["profile"] == {
        "width": 64,
        "height": 48,
        "fps": 15,
        "enable_sync": True,
        "align_depth": False,
    }
    assert report["depth_topic"] == "/camera/camera/depth/image_rect_raw"
    assert report["absolute_error_m"] == pytest.approx(0.0)
    assert report["allowed_error_m"] == pytest.approx(0.05)


@pytest.mark.skipif(not Path(LIBRARY).exists(), reason="Official librealsense library required")
def test_cli_five_second_delay_uses_saved_depth_and_tf(tmp_path):
    import json
    import subprocess
    import sys
    rgb, raw, meta = snapshot()
    path = tmp_path / "synthetic_snapshot.npz"
    np.savez(path, rgb=rgb, raw_depth=raw, metadata=json.dumps(meta))
    report = tmp_path / "synthetic_units.json"
    report.write_text(json.dumps({
        "verified": True, "passed": True, "meters_per_unit": .001
    }))
    result = subprocess.run([
        sys.executable, str(Path(probe.__file__)), "project", str(path),
        "--units-report", str(report), "--pixel", "30", "24", "--delay", "5",
    ], check=True, capture_output=True, text=True)
    output = json.loads(result.stdout)
    assert output["delayed_snapshot_unchanged"] is True
    assert output["depth_stamp_ns"] == 10
    assert output["map_point"] == pytest.approx([1., 2., 4.])


@pytest.mark.skipif(not Path(LIBRARY).exists(), reason="Official librealsense library required")
def test_project_rejects_an_unverified_units_report(tmp_path):
    import json
    import subprocess
    import sys
    rgb, raw, meta = snapshot()
    path = tmp_path / "snapshot.npz"
    np.savez(path, rgb=rgb, raw_depth=raw, metadata=json.dumps(meta))
    report = tmp_path / "units.json"
    report.write_text(json.dumps({
        "verified": False, "passed": True, "meters_per_unit": .001
    }))

    result = subprocess.run([
        sys.executable, str(Path(probe.__file__)), "project", str(path),
        "--units-report", str(report), "--pixel", "30", "24",
    ], capture_output=True, text=True)

    assert result.returncode != 0
    assert "has not passed" in result.stderr
