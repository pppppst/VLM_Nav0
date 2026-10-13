"""Offline regression: no DDS participant or hardware, one constructor check."""
import importlib.util
import os
from pathlib import Path
import tempfile
from unittest.mock import patch


def check():
    source = Path(os.environ.get("GO2_DIAGNOSTIC_MODULE", Path(__file__).resolve().parents[1] / "scripts/run_ab.py"))
    spec = importlib.util.spec_from_file_location("go2_diagnostic_entry", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class GraphOnlyNode:
        def __init__(self):
            self.topics = []

        def create_subscription(self, message_type, topic, callback, qos, **kwargs):
            self.topics.append(topic)
            return object()

        def destroy_node(self):
            pass

    with tempfile.TemporaryDirectory() as directory:
        node = GraphOnlyNode()
        with patch.object(module.rclpy, "create_node", return_value=node):
            monitor = module.Monitor(Path(directory), "A")
        try:
            assert set(node.topics) == {"/odometry", "/lowstate"}, node.topics
        finally:
            monitor.close()
    with tempfile.TemporaryDirectory() as directory:
        node = GraphOnlyNode()
        with patch.object(module.rclpy, "create_node", return_value=node):
            monitor = module.Monitor(Path(directory), "A", full_cloud_monitoring=True)
        try:
            assert set(node.topics) == {"/odometry", "/lowstate", "/lidar_points", "/lidar_points_fastlio"}, node.topics
        finally:
            monitor.close()
    print("default guard and explicit full-cloud opt-in passed; no ROS init")


if __name__ == "__main__":
    check()
