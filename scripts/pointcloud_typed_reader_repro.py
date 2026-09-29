#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main() -> None:
    rclpy.init()
    reader_id = os.environ.get("REPRO_READER_ID", "1")
    node = Node(f"pointcloud_typed_reader_{reader_id}")
    started = time.monotonic()
    count = 0
    first_receipt = None
    last_receipt = None
    ages = []

    def on_cloud(message: PointCloud2) -> None:
        nonlocal count, first_receipt, last_receipt
        now = time.monotonic()
        count += 1
        first_receipt = now if first_receipt is None else first_receipt
        last_receipt = now
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        ages.append(time.time() - stamp)
        if count == 1:
            print(
                "REPRO_FIRST "
                f"after_s={now - started:.6f} age_s={ages[-1]:.6f} "
                f"width={message.width} height={message.height} "
                f"point_step={message.point_step} row_step={message.row_step} "
                f"data_len={len(message.data)}",
                flush=True,
            )

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        PointCloud2, "/cloud_registered_base", on_cloud, qos
    )
    print(
        f"REPRO_START pid={os.getpid()} reader_id={reader_id} "
        f"rmw={rclpy.utilities.get_rmw_implementation_identifier()}",
        flush=True,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        callback_hz = (
            (count - 1) / (last_receipt - first_receipt)
            if count > 1 and last_receipt > first_receipt
            else 0.0
        )
        age_mean = sum(ages) / len(ages) if ages else float("nan")
        first_after = first_receipt - started if first_receipt is not None else -1.0
        print(
            "REPRO_FINAL "
            f"count={count} elapsed_s={elapsed:.6f} callback_hz={callback_hz:.6f} "
            f"first_after_s={first_after:.6f} age_mean_s={age_mean:.6f}",
            flush=True,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
