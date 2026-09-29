import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node("minimal_cloud_subscriber")
    callback_count = 0

    def on_cloud(_message: PointCloud2) -> None:
        nonlocal callback_count
        callback_count += 1
        node.get_logger().info(f"LAUNCHED_PROBE callback_count={callback_count}")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        PointCloud2, "/cloud_registered_base", on_cloud, qos
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
