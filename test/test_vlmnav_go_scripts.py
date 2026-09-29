import json
import os
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]


def make_fake_command(directory, name, body):
    path = directory / name
    path.write_text(f"#!/usr/bin/env bash\nset -eu\n{body}\n")
    path.chmod(0o755)


def copied_scripts(tmp_path):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    for name in ("VLMNav-go.sh", "stopgo.sh"):
        shutil.copy2(ROOT / "scripts" / name, scripts / name)
    (scripts / "common.sh").write_text("")
    (scripts / "go2_network_env.sh").write_text("")
    return scripts


def test_preflight_reuses_fresh_formal_report_without_starting_sensors(tmp_path):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_go2_preflight.sh", scripts)
    (scripts / "common.sh").write_text("")
    (scripts / "go2_network_env.sh").write_text("")
    os.symlink(ROOT / "vlm_nav", tmp_path / "vlm_nav")

    report = tmp_path / "preflight.json"
    report.write_text(
        json.dumps(
            {
                "healthy": True,
                "formal_preflight": "PASS",
                "generated_at_unix_s": time.time(),
            }
        )
    )
    calls = tmp_path / "calls"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    make_fake_command(fake_bin, "ros2", 'echo "$*" >> "$CALLS"; exit 99')
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CALLS": str(calls),
        "GO2_PREFLIGHT_REPORT": str(report),
        "PYTHONPATH": str(tmp_path),
    }

    completed = subprocess.run(
        ["bash", str(scripts / "run_go2_preflight.sh")],
        env=env,
        text=True,
        capture_output=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "reusing fresh formal preflight report" in completed.stdout
    assert not calls.exists()


def test_vlm_start_delegates_to_existing_launcher_without_camera_preflight(tmp_path):
    scripts = copied_scripts(tmp_path)
    calls = tmp_path / "calls"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    make_fake_command(fake_bin, "ping", 'echo "ping $*" >> "$CALLS"')
    make_fake_command(fake_bin, "ssh", 'echo yes')
    make_fake_command(
        fake_bin,
        "gnome-terminal",
        'echo "gnome-terminal $*" >> "$CALLS"',
    )
    make_fake_command(
        fake_bin,
        "ros2",
        '''echo "ros2 $*" >> "$CALLS"
case "$*" in
  "topic info /camera/camera/color/image_raw") echo "Publisher count: 1" ;;
  "topic info /camera/camera/depth/image_rect_raw") echo "Publisher count: 1" ;;
  *"param get /camera/camera depth_module.depth_profile"*) echo "String value is: 640x480x15" ;;
  *"param get /camera/camera rgb_camera.color_profile"*) echo "String value is: 640x480x15" ;;
  *"param get /camera/camera enable_sync"*) echo "Boolean value is: True" ;;
  *"param get /camera/camera align_depth.enable"*) echo "Boolean value is: False" ;;
esac''',
    )
    make_fake_command(
        scripts,
        "manualnav2.sh",
        'printf "%s|%s\\n" "$VLM_NAV_MODE" "$VLM_TARGET_DESCRIPTION"',
    )
    report = tmp_path / "units.json"
    report.write_text("{}")
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CALLS": str(calls),
        "VLM_DEPTH_UNITS_REPORT": str(report),
        "DISPLAY": ":0",
        "DASHSCOPE_API_KEY": "test-key",
        "DASHSCOPE_BASE_URL": "https://example.invalid",
    }

    completed = subprocess.run(
        ["bash", str(scripts / "VLMNav-go.sh"), "红色灭火器"],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert completed.stdout.rstrip().endswith("true|红色灭火器")
    recorded_calls = calls.read_text()
    assert "go2_camera_preflight" not in recorded_calls
    assert "gnome-terminal" in recorded_calls
    assert "/cmd_vel_bridge" in recorded_calls
    assert "/vlm_nav/diagnostics" in recorded_calls


def test_stopgo_disables_then_cancels_then_disarms(tmp_path):
    scripts = copied_scripts(tmp_path)
    calls = tmp_path / "calls"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    make_fake_command(
        fake_bin,
        "ros2",
        '''echo "$*" >> "$CALLS"
case "$*" in
  "node list") printf '/vlm_nav\n/twist_to_go2_sport_bridge\n' ;;
  *"param set /vlm_nav enabled false"*) echo "Set parameter successful" ;;
  *"twist_to_go2_sport_bridge/disarm"*) echo "success=True" ;;
  *"topic echo /vlm_nav/go2_bridge_state"*) echo DISARMED ;;
  *"topic echo /vlm_nav/control_armed"*) echo False ;;
esac''',
    )
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CALLS": str(calls),
    }

    subprocess.run(
        ["bash", str(scripts / "stopgo.sh")],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    actions = [
        line
        for line in calls.read_text().splitlines()
        if "param set" in line or "cancel_goal" in line or "/disarm" in line
    ]
    assert "param set /vlm_nav enabled false" in actions[0]
    assert "/navigate_to_pose/_action/cancel_goal" in actions[1]
    assert "/twist_to_go2_sport_bridge/disarm" in actions[2]
