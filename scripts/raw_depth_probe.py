#!/usr/bin/env python3
"""Standalone Go2 raw RGB-D capture and single-pixel projection. Never commands motion."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from vlm_nav.geometry import RealSenseGeometry as Geometry


def load_snapshot(path):
    with np.load(path, allow_pickle=False) as data:
        rgb, raw = data["rgb"].copy(), data["raw_depth"].copy()
        meta = json.loads(str(data["metadata"]))
    rgb.flags.writeable = raw.flags.writeable = False
    return rgb, raw, meta


def fingerprint(snapshot):
    rgb, raw, meta = snapshot
    return hashlib.sha256(rgb.tobytes() + raw.tobytes() + json.dumps(meta, sort_keys=True).encode()).hexdigest()


def past_extrapolation(error):
    return "extrapolation into the past" in str(error).lower()


def rate(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node("raw_depth_rate_probe")
    stamps = []
    node.create_subscription(Image, args.topic, lambda m: stamps.append(
        m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec), qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        elapsed = time.monotonic() - started
        node.destroy_node()
        rclpy.shutdown()
    unique = len(set(stamps))
    duplicates = len(stamps) - unique
    rollback = sum(b < a for a, b in zip(stamps, stamps[1:]))
    hz = unique / elapsed
    result = {"topic": args.topic, "duration_s": elapsed, "callbacks": len(stamps),
              "unique_frames": unique, "unique_hz": hz, "duplicate": duplicates,
              "rollback": rollback, "passed": 14 <= hz <= 16 and
              duplicates / max(1, len(stamps)) < .01 and rollback == 0}
    print(json.dumps(result))
    raise SystemExit(0 if result["passed"] else 1)


def capture(args):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_ros import Buffer, TransformListener, TransformException
    from vlm_nav.geometry import transform_matrix
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init()
    node = rclpy.create_node("raw_depth_snapshot_probe")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    queues = {"color": deque(maxlen=10), "depth": deque(maxlen=10)}
    infos = {}

    def stamp(m):
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

    def enqueue(name, message):
        if all(stamp(old) != stamp(message) for old in queues[name]):
            queues[name].append(message)

    def matrix(target, source, timestamp):
        t = buffer.lookup_transform(target, source, timestamp, timeout=Duration(seconds=0.)).transform
        return transform_matrix([t.translation.x, t.translation.y, t.translation.z],
                                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).tolist()

    for name, suffix in (("color", "image_raw"), ("depth", "image_rect_raw")):
        node.create_subscription(Image, f"/camera/camera/{name}/{suffix}",
                                 lambda m, n=name: enqueue(n, m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f"/camera/camera/{name}/camera_info",
                                 lambda m, n=name: infos.__setitem__(n, m), 10)
    deadline = time.monotonic() + args.timeout
    pending = None
    error = "Waiting for synchronized RGB/depth and CameraInfo"
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if pending is None and len(infos) == 2:
                pairs = [(abs(stamp(c) - stamp(d)), c, d) for c in queues["color"] for d in queues["depth"]]
                if pairs:
                    delta, c, d = min(pairs, key=lambda item: item[0])
                    if delta <= 50_000_000:
                        pending = c, d, dict(infos)
            if pending is None:
                continue
            c, d, info = pending
            if any(m.header.frame_id != info[n].header.frame_id or
                   (m.width, m.height) != (info[n].width, info[n].height)
                   for n, m in (("color", c), ("depth", d))):
                raise ValueError("CameraInfo does not match image frame/profile")
            try:
                timestamp = Time.from_msg(d.header.stamp)
                dc = matrix(c.header.frame_id, d.header.frame_id, timestamp)
                md = None if args.camera_only else matrix("map", d.header.frame_id, timestamp)
            except TransformException as exc:
                error = str(exc)
                if past_extrapolation(exc):
                    pending = None
                    queues["color"].clear()
                    queues["depth"].clear()
                continue
            if c.encoding not in ("rgb8", "bgr8") or d.encoding != "16UC1":
                raise ValueError(f"Expected RGB8/BGR8 and 16UC1, got {c.encoding}, {d.encoding}")
            rgb = np.ndarray((c.height, c.width, 3), dtype=np.uint8, buffer=bytes(c.data),
                             strides=(c.step, 3, 1)).copy()
            if c.encoding == "bgr8":
                rgb = rgb[:, :, ::-1].copy()
            raw = np.ndarray((d.height, d.width), dtype=">u2" if d.is_bigendian else "<u2",
                             buffer=bytes(d.data), strides=(d.step, 2)).astype(np.uint16)
            meta = {"rgb_stamp_ns": stamp(c), "depth_stamp_ns": stamp(d),
                    "color_info": message_to_ordereddict(info["color"]),
                    "depth_info": message_to_ordereddict(info["depth"]),
                    "depth_to_color": dc, "color_to_depth": np.linalg.inv(dc).tolist(),
                    "T_map_depth_optical": md}
            with open(args.output, "xb") as output:
                np.savez_compressed(output, rgb=rgb, raw_depth=raw, metadata=json.dumps(meta))
            import cv2
            cv2.imwrite(str(Path(args.output).with_suffix(".png")), rgb[:, :, ::-1])
            print(json.dumps({"snapshot": args.output, "delta_ms": abs(stamp(c)-stamp(d))/1e6,
                              "center_raw": int(raw[d.height//2, d.width//2]), "map_tf_saved": md is not None}))
            return
        raise RuntimeError(f"Capture timed out: {error}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    rates = sub.add_parser("rate")
    rates.add_argument("--topic", required=True)
    rates.add_argument("--duration", type=float, default=60.)
    cap = sub.add_parser("capture")
    cap.add_argument("--output", required=True)
    cap.add_argument("--timeout", type=float, default=15)
    cap.add_argument("--camera-only", action="store_true", help="Unit/optical diagnostics only; no fake map TF")
    units = sub.add_parser("units")
    units.add_argument("snapshot")
    units.add_argument("--pixel", type=int, nargs=2, required=True, help="DEPTH pixel on measured plane")
    units.add_argument("--distance-m", type=float, required=True, help="Measured optical-axis plane distance")
    units.add_argument("--device-scale", type=float, required=True)
    units.add_argument("--meters-per-unit", type=float, required=True, help="Candidate ROS scale to validate, never inferred by fitting")
    units.add_argument("--fps", type=int, required=True, help="Measured fixed camera profile frame rate")
    units.add_argument("--output", required=True)
    proj = sub.add_parser("project")
    proj.add_argument("snapshot")
    choice = proj.add_mutually_exclusive_group(required=True)
    choice.add_argument("--pixel", type=float, nargs=2)
    choice.add_argument("--target", help="One real VLM request using this snapshot")
    proj.add_argument("--units-report", required=True)
    proj.add_argument("--library", default="/usr/local/lib/librealsense2.so.2.53.1")
    proj.add_argument("--color-model", type=int)
    proj.add_argument("--depth-model", type=int)
    proj.add_argument("--min-depth", type=float, default=.2)
    proj.add_argument("--max-depth", type=float, default=5.)
    proj.add_argument("--delay", type=float, default=0.)
    args = parser.parse_args()
    if args.command == "rate":
        if args.duration <= 0 or not np.isfinite(args.duration):
            parser.error("duration must be positive and finite")
        rate(args)
        return
    if args.command == "capture":
        capture(args)
        return
    snapshot = load_snapshot(args.snapshot)
    if args.command == "units":
        x, y = args.pixel
        raw = snapshot[1]
        if not (0 <= x < raw.shape[1] and 0 <= y < raw.shape[0]):
            raise ValueError("Depth pixel outside image")
        if not np.isfinite([args.distance_m, args.device_scale, args.meters_per_unit]).all() or min(args.distance_m, args.device_scale, args.meters_per_unit) <= 0 or args.fps <= 0:
            raise ValueError("Distances/scales must be positive and finite")
        value = int(raw[y, x])
        meters = value * args.meters_per_unit
        absolute_error = abs(meters - args.distance_m)
        allowed_error = max(.05, args.distance_m * .05)
        passed = value > 0 and absolute_error <= allowed_error
        report = {"verified": passed, "passed": passed,
                  "profile": {"width": int(raw.shape[1]), "height": int(raw.shape[0]),
                              "fps": args.fps, "enable_sync": True, "align_depth": False},
                  "depth_topic": "/camera/camera/depth/image_rect_raw",
                  "raw_value": value, "measured_z_m": args.distance_m,
                  "meters_per_unit": args.meters_per_unit, "converted_m": meters,
                  "device_scale": args.device_scale, "raw_times_device_scale_m": value * args.device_scale,
                  "absolute_error_m": absolute_error, "allowed_error_m": allowed_error,
                  "snapshot_sha256": fingerprint(snapshot)}
        with open(args.output, "x") as output:
            json.dump(report, output, indent=2)
        print(json.dumps(report))
        raise SystemExit(0 if passed else 1)
    report = json.loads(Path(args.units_report).read_text())
    if report.get("verified") is not True or report.get("passed") is not True:
        raise ValueError("Unit measurement has not passed")
    if args.target and snapshot[2]["T_map_depth_optical"] is None:
        raise ValueError("Real VLM map validation requires a snapshot with capture-time map TF")
    sdk = Geometry(args.library)
    def project(pixel):
        return sdk.project(snapshot, pixel, report["meters_per_unit"], args.min_depth,
                           args.max_depth, args.color_model, args.depth_model)
    before_hash = fingerprint(snapshot)
    baseline = project(args.pixel) if args.pixel is not None else None
    if args.delay < 0 or not np.isfinite(args.delay):
        raise ValueError("Delay must be nonnegative and finite")
    time.sleep(args.delay)
    pixel = args.pixel
    if args.target:
        from vlm_nav.vlm_client import OpenAICompatibleVLMClient
        result = OpenAICompatibleVLMClient(timeout_s=8., image_detail="low", jpeg_quality=85).infer(snapshot[0], args.target)
        if not result.target_visible or result.target_pixel is None:
            raise ValueError("VLM did not return a visible target pixel")
        pixel = [result.target_pixel.u, result.target_pixel.v]
    result = project(pixel)
    if fingerprint(snapshot) != before_hash or (baseline is not None and result != baseline):
        raise RuntimeError("Snapshot changed during delay")
    result.update(snapshot_sha256=before_hash, delayed_snapshot_unchanged=True, delay_s=args.delay)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
