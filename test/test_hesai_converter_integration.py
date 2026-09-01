from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "Co-NavGPT2/start_ros_single_nav_hesai_fastlio.sh"
LAUNCH = ROOT / "go2_ws/src/hesai_fastlio_converter/launch/hesai_fastlio_converter.launch.py"


def test_hesai_fastlio_entrypoint_uses_cpp_converter():
    script = SCRIPT.read_text()
    launch = LAUNCH.read_text()

    assert "hesai_fastlio_converter_node" in script
    assert "/lidar_points_fastlio" in script
    assert "tools/hesai_to_fastlio.py" not in script
    assert "hesai_fastlio_converter_node" in launch


def test_hesai_fastlio_script_keeps_spark_stage_explicit():
    script = SCRIPT.read_text()
    assert "spark_lio_mapping" in script or "spark_fast_lio_go2_xt16.launch.yaml" in script
