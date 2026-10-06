import json
import os
from pathlib import Path
import shutil
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "scripts/go2_run_evidence.py"
RECORDER = ROOT / "scripts/record_go2_nav2_manual.sh"


def run_tool(*args, env=None):
    return subprocess.run(
        ["python3", str(TOOL), *map(str, args)],
        env=env,
        text=True,
        capture_output=True,
    )


def make_fake_install(tmp_path):
    vlm_share = tmp_path / "install/vlm_nav"
    nav2_share = tmp_path / "install/nav2_bringup"
    for relative in (
        "launch/go2_system.launch.py",
        "launch/go2_navigation.launch.py",
        "launch/go2_vlm.launch.py",
        "config/go2_calibration_xt16.yaml",
        "config/spark_fast_lio_go2_xt16.yaml",
        "config/go2_preflight_xt16.yaml",
        "config/robot_go2.yaml",
        "config/go2_laserscan.yaml",
        "config/go2_amcl.yaml",
        "config/nav2_go2.yaml",
        "config/nav2_go2_static_map.yaml",
    ):
        path = vlm_share / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"installed: {relative}\n")
    nav2_params = nav2_share / "params/nav2_params.yaml"
    nav2_params.parent.mkdir(parents=True)
    nav2_params.write_text("installed: nav2 defaults\n")
    return vlm_share, nav2_share


def test_init_snapshots_relative_map_and_installed_configs(tmp_path):
    run_dir = tmp_path / "run"
    map_dir = tmp_path / "maps"
    map_dir.mkdir()
    image = map_dir / "lab.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\0")
    map_yaml = map_dir / "lab.yaml"
    map_yaml.write_text("image: lab.pgm\nresolution: 0.05\n")
    vlm_share, nav2_share = make_fake_install(tmp_path)

    completed = run_tool(
        "init",
        "--run-dir", run_dir,
        "--map-yaml", map_yaml,
        "--source-dir", ROOT,
        "--install-share", vlm_share,
        "--nav2-share", nav2_share,
        "--entry", ROOT / "scripts/VLMNav-go.sh",
        "--argument", "红色灭火器",
        "--argument", f"map:={map_yaml}",
        "--localization-mode", "amcl",
        "--amcl-scan-topic", "/scan",
        "--vlm-artifact-root", tmp_path / "VLM_feedback",
    )

    assert completed.returncode == 0, completed.stderr
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["localization"]["amcl_scan_topic"] == "/scan"
    assert manifest["map"]["image"]["original"] == str(image.resolve())
    assert len(manifest["map"]["image"]["sha256"]) == 64
    assert (run_dir / manifest["map"]["yaml"]["snapshot"]).read_text() == map_yaml.read_text()
    assert (run_dir / manifest["map"]["image"]["snapshot"]).read_bytes() == image.read_bytes()
    assert any(
        item["original"].endswith("nav2_bringup/params/nav2_params.yaml")
        for item in manifest["installed_configs"]
    )
    assert manifest["source"]["branch"] == "VLM_nav_go"
    assert manifest["source"]["commit"]
    assert manifest["package_share"] == str(vlm_share.resolve())


def test_finalize_links_only_new_arm_directory(tmp_path):
    run_dir = tmp_path / "run"
    map_dir = tmp_path / "maps"
    map_dir.mkdir()
    (map_dir / "lab.pgm").write_bytes(b"map")
    map_yaml = map_dir / "lab.yaml"
    map_yaml.write_text("image: lab.pgm\n")
    artifact_root = tmp_path / "VLM_feedback"
    old = artifact_root / "arm_old"
    old.mkdir(parents=True)
    vlm_share, nav2_share = make_fake_install(tmp_path)
    initialized = run_tool(
        "init",
        "--run-dir", run_dir,
        "--map-yaml", map_yaml,
        "--source-dir", ROOT,
        "--install-share", vlm_share,
        "--nav2-share", nav2_share,
        "--entry", "scripts/VLMNav-go.sh",
        "--localization-mode", "amcl",
        "--amcl-scan-topic", "/scan",
        "--vlm-artifact-root", artifact_root,
    )
    assert initialized.returncode == 0, initialized.stderr
    new = artifact_root / "arm_20261002_120000_000001"
    new.mkdir()

    completed = run_tool(
        "finalize", "--run-dir", run_dir, "--vlm-artifact-root", artifact_root
    )

    assert completed.returncode == 0, completed.stderr
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["vlm_artifacts"]["status"] == "linked"
    assert manifest["vlm_artifacts"]["session_directory"] == str(new.resolve())
    assert manifest["status"] == "finished"
    assert manifest["ended_at"]


def test_snapshot_core_times_out_and_fails_closed_for_amcl(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"$*\" == 'param dump /amcl' ]]; then sleep 2; fi\n"
        "if [[ \"$*\" == 'param dump /map_server' ]]; then echo 'map_server: {}'; exit 0; fi\n"
        "echo \"failed: $*\" >&2\n"
        "exit 9\n"
    )
    ros2.chmod(0o755)
    env = {**os.environ, "PATH": f"{fake_bin}:{os.environ['PATH']}"}

    completed = run_tool(
        "snapshot-core",
        "--output-dir", tmp_path / "params",
        "--localization-mode", "amcl",
        "--timeout", "0.1",
        env=env,
    )

    assert completed.returncode == 1
    assert "timed out" in (tmp_path / "params/amcl.error.txt").read_text()
    assert (tmp_path / "params/map_server.yaml").is_file()
    assert (tmp_path / "params/lio_mapping.error.txt").is_file()


def test_snapshot_vlm_queries_only_whitelisted_parameters(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    calls = tmp_path / "calls"
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        "#!/usr/bin/env bash\n"
        "echo \"$*\" >> \"$CALLS\"\n"
        "case \"$*\" in\n"
        "  *vlm_image_record_path) echo '~/unitree_ros2/log/VLM_feedback' ;;\n"
        "  *) echo 'map' ;;\n"
        "esac\n"
    )
    ros2.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CALLS": str(calls),
    }

    completed = run_tool(
        "snapshot-vlm", "--output-dir", tmp_path / "params", "--timeout", "1",
        env=env,
    )

    assert completed.returncode == 0, completed.stderr
    params = yaml.safe_load((tmp_path / "params/vlm_nav.yaml").read_text())
    saved = params["/vlm_nav"]["ros__parameters"]
    assert set(saved) == {
        "global_frame", "base_frame", "camera_frame", "map_topic",
        "arrival_odom_topic", "nav_ready_topic", "require_external_safety_gates",
        "external_gate_timeout", "target_description", "max_result_age",
        "confirm_frames", "confirmation_radius", "target_confirmation_timeout",
        "target_probe_distance", "target_probe_attempt_limit",
        "target_probe_min_distance", "target_success_radius",
    }
    recorded_calls = calls.read_text()
    assert "API_KEY" not in recorded_calls
    assert "token" not in recorded_calls.lower()
    assert (tmp_path / "params/vlm_artifact_root.txt").read_text().strip().endswith(
        "unitree_ros2/log/VLM_feedback"
    )


def test_snapshot_vlm_reports_query_failures(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text("#!/usr/bin/env bash\necho 'node unavailable' >&2\nexit 9\n")
    ros2.chmod(0o755)
    env = {**os.environ, "PATH": f"{fake_bin}:{os.environ['PATH']}"}

    completed = run_tool(
        "snapshot-vlm", "--output-dir", tmp_path / "params", "--timeout", "1",
        env=env,
    )

    assert completed.returncode == 1
    assert "global_frame: ros2 param get exited 9" in (
        tmp_path / "params/vlm_nav.error.txt"
    ).read_text()
    assert (tmp_path / "params/vlm_artifact_root.error.txt").is_file()


def test_recorder_uses_run_directory_qos_and_deduplicates_scan(tmp_path):
    scripts = tmp_path / "scripts"
    config = tmp_path / "config"
    scripts.mkdir()
    config.mkdir()
    shutil.copy2(RECORDER, scripts / RECORDER.name)
    (scripts / "common.sh").write_text("")
    (scripts / "go2_network_env.sh").write_text("")
    qos_source = ROOT / "config/go2_nav2_rosbag_qos.yaml"
    shutil.copy2(qos_source, config / qos_source.name)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    calls = tmp_path / "calls"
    ros2 = fake_bin / "ros2"
    ros2.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" > \"$CALLS\"\n")
    ros2.chmod(0o755)
    run_dir = tmp_path / "run"
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CALLS": str(calls),
    }

    completed = subprocess.run(
        ["bash", str(scripts / RECORDER.name), str(run_dir), "/scan"],
        env=env,
        text=True,
        capture_output=True,
    )

    assert completed.returncode == 0, completed.stderr
    arguments = calls.read_text().split()
    assert arguments.count("/scan") == 1
    for topic in (
        "/tf", "/tf_static", "/map", "/amcl_pose", "/particle_cloud",
        "/initialpose", "/vlm_nav/diagnostics", "/vlm_nav/state",
        "/vlm_nav/vlm_enabled", "/vlm_nav/markers", "/vlm_nav/output_text",
    ):
        assert topic in arguments
    assert not any("image_raw" in item or "points" in item for item in arguments)
    qos_path = run_dir / "rosbag/qos_overrides.yaml"
    assert f"--qos-profile-overrides-path {qos_path}" in calls.read_text()
    qos = yaml.safe_load(qos_path.read_text())
    for topic in ("/map", "/tf_static"):
        assert qos[topic]["durability"] == "transient_local"
        assert qos[topic]["reliability"] == "reliable"


def test_manual_workflow_orders_record_snapshot_stop_and_cleanup():
    source = (ROOT / "scripts/manualnav2.sh").read_text()

    recorder = source.index('start_child bag "${script_dir}/record_go2_nav2_manual.sh"')
    system = source.index("start_child nav2 ros2 launch vlm_nav go2_system.launch.py")
    ready = source.index("\nwait_for_nav2\n")
    initial_snapshot = source.index("snapshot_core_parameters start true")
    arm_prompt = source.index('confirm ARM "')
    vlm_snapshot = source.index("snapshot_vlm_parameters start")
    enable_prompt = source.index('confirm ENABLE_VLM "')
    stop = source.index('"${script_dir}/stopgo.sh"', source.index("cleanup()"))
    end_snapshot = source.index("finish_evidence", source.index("cleanup()"))
    terminate = source.index('kill -INT -- "-${pid}"', source.index("cleanup()"))

    assert recorder < system
    assert ready < initial_snapshot < arm_prompt
    assert vlm_snapshot < enable_prompt
    assert stop < end_snapshot < terminate
    assert "VLM_NAV_RUN_ROOT:-/home/isee-pst/unitree_ros2/log/localization_runs" in source
    assert 'require_child "${bag_index}"' in source
    confirm_body = source[source.index("confirm() {"):source.index("publisher_count() {")]
    assert "read -r -t 1" in confirm_body
    assert "require_active_children" in confirm_body
    assert source.count("require_evidence\nconfirm ARM") == 1
    assert source.count("require_evidence\n  confirm ENABLE_VLM") == 1


def test_finalize_can_mark_a_completed_manifest_aborted(tmp_path):
    run_dir = tmp_path / "run"
    map_dir = tmp_path / "maps"
    map_dir.mkdir()
    (map_dir / "lab.pgm").write_bytes(b"map")
    map_yaml = map_dir / "lab.yaml"
    map_yaml.write_text("image: lab.pgm\n")
    artifact_root = tmp_path / "VLM_feedback"
    vlm_share, nav2_share = make_fake_install(tmp_path)
    initialized = run_tool(
        "init",
        "--run-dir", run_dir,
        "--map-yaml", map_yaml,
        "--source-dir", ROOT,
        "--install-share", vlm_share,
        "--nav2-share", nav2_share,
        "--entry", "scripts/VLMNav-go.sh",
        "--localization-mode", "amcl",
        "--amcl-scan-topic", "/scan",
        "--vlm-artifact-root", artifact_root,
    )
    assert initialized.returncode == 0, initialized.stderr
    assert run_tool(
        "finalize", "--run-dir", run_dir, "--vlm-artifact-root", artifact_root
    ).returncode == 0

    completed = run_tool(
        "finalize", "--run-dir", run_dir, "--vlm-artifact-root", artifact_root,
        "--status", "aborted",
    )

    assert completed.returncode == 0, completed.stderr
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["status"] == "aborted"
    assert manifest["vlm_artifacts"]["status"] == "not_found"
