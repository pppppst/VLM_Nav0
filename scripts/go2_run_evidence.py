#!/usr/bin/env python3
"""Create and update one self-contained Go2 localization evidence run."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


CORE_NODES = (
    "/amcl",
    "/map_server",
    "/lio_mapping",
    "/obstacle_cloud_filter",
    "/go2_cloud_to_scan",
    "/controller_server",
    "/planner_server",
    "/behavior_server",
    "/velocity_smoother",
    "/go2_nav_supervisor",
)
VLM_PARAMETERS = (
    "global_frame",
    "base_frame",
    "camera_frame",
    "map_topic",
    "arrival_odom_topic",
    "nav_ready_topic",
    "require_external_safety_gates",
    "external_gate_timeout",
    "target_description",
    "max_result_age",
    "confirm_frames",
    "confirmation_radius",
    "target_confirmation_timeout",
    "target_probe_distance",
    "target_probe_attempt_limit",
    "target_probe_min_distance",
    "target_success_radius",
)
VLM_NAV_FILES = (
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
)


def now():
    return datetime.now().astimezone()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(source, *arguments):
    completed = subprocess.run(
        ["git", "-C", str(source), *arguments],
        text=True,
        capture_output=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "unknown"


def copy_record(source, destination, run_dir):
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return {
        "original": str(source.absolute()),
        "resolved": str(source.resolve()),
        "snapshot": str(destination.relative_to(run_dir)),
        "sha256": sha256(destination),
    }


def write_manifest(run_dir, manifest):
    destination = run_dir / "manifest.json"
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def artifact_directories(root):
    if not root.is_dir():
        return []
    return sorted(str(path.resolve()) for path in root.glob("arm_*") if path.is_dir())


def init_run(args):
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    for child in ("logs", "rosbag", "parameters", "map", "installed_config"):
        (run_dir / child).mkdir()

    map_record = {"status": "not_applicable"}
    if args.map_yaml is not None:
        map_yaml = args.map_yaml.expanduser().resolve()
        data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        image_value = data.get("image") if isinstance(data, dict) else None
        if not isinstance(image_value, str) or not image_value.strip():
            raise ValueError(f"map YAML has no image path: {map_yaml}")
        image = Path(image_value).expanduser()
        if not image.is_absolute():
            image = map_yaml.parent / image
        image = image.resolve()
        map_record = {
            "status": "snapshotted",
            "yaml": copy_record(map_yaml, run_dir / "map/map.yaml", run_dir),
            "image": copy_record(
                image, run_dir / f"map/map_image{''.join(image.suffixes)}", run_dir
            ),
            "yaml_image_value": image_value,
        }

    installed = []
    for relative in VLM_NAV_FILES:
        installed.append(
            copy_record(
                args.install_share / relative,
                run_dir / "installed_config/vlm_nav" / relative,
                run_dir,
            )
        )
    installed.append(
        copy_record(
            args.nav2_share / "params/nav2_params.yaml",
            run_dir / "installed_config/nav2_bringup/params/nav2_params.yaml",
            run_dir,
        )
    )

    started = now()
    artifact_root = args.vlm_artifact_root.expanduser().resolve()
    status = git(args.source_dir, "status", "--porcelain")
    manifest = {
        "schema_version": 1,
        "status": "running",
        "started_at": started.isoformat(),
        "timezone": started.strftime("%Z%z"),
        "entry": str(args.entry),
        "arguments": args.argument,
        "localization": {
            "mode": args.localization_mode,
            "amcl_scan_topic": args.amcl_scan_topic,
        },
        "ros": {
            name: os.environ.get(name, "")
            for name in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID")
        },
        "package_share": str(args.install_share.absolute()),
        "package_share_resolved": str(args.install_share.resolve()),
        "source": {
            "path": str(args.source_dir.resolve()),
            "branch": git(args.source_dir, "branch", "--show-current"),
            "commit": git(args.source_dir, "rev-parse", "HEAD"),
            "dirty": bool(status and status != "unknown"),
        },
        "map": map_record,
        "installed_configs": installed,
        "vlm_artifacts": {
            "root": str(artifact_root),
            "status": "pending",
            "preexisting_directories": artifact_directories(artifact_root),
        },
    }
    write_manifest(run_dir, manifest)
    print(run_dir)


def command_error(path, message, stderr=""):
    detail = message
    if stderr.strip():
        detail += f"\nstderr: {stderr.strip()}"
    path.write_text(detail + "\n", encoding="utf-8")


def ros2(arguments, timeout):
    try:
        return subprocess.run(
            ["ros2", *arguments],
            text=True,
            capture_output=True,
            timeout=timeout,
        ), None
    except subprocess.TimeoutExpired as error:
        return None, f"timed out after {timeout:g} seconds"


def snapshot_core(args):
    args.output_dir.mkdir(parents=True, exist_ok=True)
    critical_failed = False
    for node in CORE_NODES:
        name = node.removeprefix("/")
        destination = args.output_dir / f"{name}.yaml"
        error_path = args.output_dir / f"{name}.error.txt"
        completed, timeout_error = ros2(["param", "dump", node], args.timeout)
        if timeout_error:
            command_error(error_path, timeout_error)
        elif completed.returncode != 0 or not completed.stdout.strip():
            command_error(
                error_path,
                f"ros2 param dump exited {completed.returncode}",
                completed.stderr,
            )
        else:
            destination.write_text(completed.stdout, encoding="utf-8")
            error_path.unlink(missing_ok=True)
            continue
        destination.unlink(missing_ok=True)
        if args.localization_mode == "amcl" and node in ("/amcl", "/map_server"):
            critical_failed = True
    return 1 if critical_failed else 0


def parameter_value(name, timeout):
    completed, timeout_error = ros2(
        ["param", "get", "--hide-type", "/vlm_nav", name], timeout
    )
    if timeout_error:
        return None, timeout_error
    if completed.returncode != 0 or not completed.stdout.strip():
        return None, (
            f"ros2 param get exited {completed.returncode}: {completed.stderr.strip()}"
        )
    text = completed.stdout.strip()
    try:
        return yaml.safe_load(text), None
    except yaml.YAMLError:
        return text, None


def snapshot_vlm(args):
    args.output_dir.mkdir(parents=True, exist_ok=True)
    saved = {}
    errors = []
    for name in VLM_PARAMETERS:
        value, error = parameter_value(name, args.timeout)
        if error:
            errors.append(f"{name}: {error}")
        else:
            saved[name] = value
    (args.output_dir / "vlm_nav.yaml").write_text(
        yaml.safe_dump(
            {"/vlm_nav": {"ros__parameters": saved}},
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    error_path = args.output_dir / "vlm_nav.error.txt"
    if errors:
        error_path.write_text("\n".join(errors) + "\n", encoding="utf-8")
    else:
        error_path.unlink(missing_ok=True)
    artifact_root, error = parameter_value("vlm_image_record_path", args.timeout)
    if error:
        (args.output_dir / "vlm_artifact_root.error.txt").write_text(
            error + "\n", encoding="utf-8"
        )
    else:
        root = Path(str(artifact_root)).expanduser().resolve()
        (args.output_dir / "vlm_artifact_root.txt").write_text(
            str(root) + "\n", encoding="utf-8"
        )
    return 1 if errors or error else 0


def finalize(args):
    run_dir = args.run_dir.resolve()
    manifest_path = run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    root = args.vlm_artifact_root.expanduser().resolve()
    previous = manifest["vlm_artifacts"]
    current = artifact_directories(root)
    if "preexisting_directories" not in previous and str(root) == previous.get("root"):
        association = previous
    elif str(root) == previous.get("root"):
        candidates = sorted(set(current) - set(previous["preexisting_directories"]))
        association = {"root": str(root), "candidate_directories": candidates}
    else:
        started = datetime.fromisoformat(manifest["started_at"]).timestamp()
        candidates = [path for path in current if Path(path).stat().st_mtime >= started]
        association = {"root": str(root), "candidate_directories": candidates}
    if "status" not in association or association["status"] == "pending":
        if len(candidates) == 1:
            association.update(status="linked", session_directory=candidates[0])
        elif candidates:
            association["status"] = "ambiguous"
        else:
            association["status"] = "not_found"
    manifest["vlm_artifacts"] = association
    manifest["status"] = args.status
    manifest["ended_at"] = now().isoformat()
    write_manifest(run_dir, manifest)
    print(association["status"])


def parser():
    result = argparse.ArgumentParser()
    commands = result.add_subparsers(dest="command", required=True)
    init = commands.add_parser("init")
    init.add_argument("--run-dir", type=Path, required=True)
    init.add_argument("--map-yaml", type=Path)
    init.add_argument("--source-dir", type=Path, required=True)
    init.add_argument("--install-share", type=Path, required=True)
    init.add_argument("--nav2-share", type=Path, required=True)
    init.add_argument("--entry", required=True)
    init.add_argument("--argument", action="append", default=[])
    init.add_argument("--localization-mode", required=True)
    init.add_argument("--amcl-scan-topic", required=True)
    init.add_argument("--vlm-artifact-root", type=Path, required=True)
    init.set_defaults(function=init_run)

    core = commands.add_parser("snapshot-core")
    core.add_argument("--output-dir", type=Path, required=True)
    core.add_argument("--localization-mode", required=True)
    core.add_argument("--timeout", type=float, default=8.0)
    core.set_defaults(function=snapshot_core)

    vlm = commands.add_parser("snapshot-vlm")
    vlm.add_argument("--output-dir", type=Path, required=True)
    vlm.add_argument("--timeout", type=float, default=5.0)
    vlm.set_defaults(function=snapshot_vlm)

    finish = commands.add_parser("finalize")
    finish.add_argument("--run-dir", type=Path, required=True)
    finish.add_argument("--vlm-artifact-root", type=Path, required=True)
    finish.add_argument("--status", choices=("finished", "aborted"), default="finished")
    finish.set_defaults(function=finalize)
    return result


def main():
    args = parser().parse_args()
    try:
        result = args.function(args)
    except (OSError, ValueError, yaml.YAMLError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return result or 0


if __name__ == "__main__":
    raise SystemExit(main())
