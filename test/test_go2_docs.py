from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_dependencies_are_pinned_with_recorded_verification_reasons():
    manifest = yaml.safe_load((ROOT / "go2_dependencies.repos").read_text())
    repositories = manifest["repositories"]

    assert repositories["unitree_ros2"]["version"] == (
        "0dfa8f2e444713c52c96c7b70c433b2609879a31"
    )
    assert repositories["spark-fast-lio"]["version"] == (
        "17b36d293a14df37d57e1751a337a32e2f164692"
    )
    docs = (ROOT / "docs/GO2_PORTING.md").read_text()
    for reason in (
        "Humble verified",
        "Go2 interface verified",
        "message definition verified",
        "Sport API verified",
        "tested with current VLM_Nav integration",
        "不得自动跟随 upstream main",
    ):
        assert reason in docs


def test_plan_b_tf_ownership_and_minimal_spark_patch_are_documented():
    docs = (ROOT / "docs/GO2_PORTING.md").read_text()

    assert "SPARK 只发布 `/odometry`，不广播 `odom → base_link`" in docs
    assert "planar adapter 独占 `odom → base_footprint`" in docs
    assert "planar adapter 独占 `base_footprint → base_link`" in docs
    assert "`publish_tf`（默认 `true`）" in docs
    assert "禁止创建平行 frame" in docs


def test_fast_track_checklist_has_six_gates_and_preserves_safety_metrics():
    docs = (ROOT / "docs/GO2_PORTING.md").read_text()

    for number in range(1, 7):
        assert f"Gate {number}" in docs
    for contract in (
        "SYSTEM_READY",
        "NAV_READY",
        "CONTROL_ARMED",
        "VLM_ENABLED",
        "平移漂移 < 5 cm",
        "航向漂移 < 2°",
        "SIGKILL",
        "主机掉电",
        "网络物理断开",
    ):
        assert contract in docs


def test_runtime_and_readme_have_no_previous_account_hardcoding():
    checked = [ROOT / "README.md", *list((ROOT / "scripts").glob("*.sh"))]

    assert all("/home/isee-cdh" not in path.read_text() for path in checked)
    common = (ROOT / "scripts/common.sh").read_text()
    for ranger_overlay in ("ros2_ws", "rs515", "agilex_ws", "install_fastlio"):
        assert ranger_overlay in common


def test_api_environment_is_pinned_without_simulation_or_local_model_packages():
    requirements = (ROOT / "requirements.txt").read_text()

    assert "openai==2.32.0" in requirements
    for excluded in ("habitat", "ultralytics", "torch", "transformers"):
        assert excluded not in requirements.lower()
