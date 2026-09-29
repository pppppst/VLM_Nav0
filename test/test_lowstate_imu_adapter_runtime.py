import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


def test_lowstate_imu_adapter_publishes_body_imu(tmp_path, monkeypatch):
    executable = (
        Path(get_package_prefix("vlm_nav"))
        / "lib"
        / "vlm_nav"
        / "lowstate_imu_adapter"
    )
    ros_log_dir = tmp_path / "ros_logs"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    suffix = f"p{os.getpid()}"
    lowstate_topic = f"/vlm_nav/test/{suffix}/lowstate"
    body_imu_topic = f"/vlm_nav/test/{suffix}/body_imu"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-r",
            f"/lowstate:={lowstate_topic}",
            "-r",
            f"/body_imu:={body_imu_topic}",
        ],
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    probe = rclpy.create_node(f"lowstate_imu_adapter_probe_{os.getpid()}")
    lowstate_pub = probe.create_publisher(
        LowState,
        lowstate_topic,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
    )
    received = []
    body_imu_sub = probe.create_subscription(
        Imu,
        body_imu_topic,
        lambda message: received.append(message),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )

    try:
        deadline = time.monotonic() + 5.0
        while probe.count_subscribers(lowstate_topic) < 1 and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert probe.count_subscribers(lowstate_topic) == 1

        message = LowState()
        message.imu_state.quaternion = [0.9, 0.1, 0.2, 0.3]
        message.imu_state.gyroscope = [1.0, 2.0, 3.0]
        message.imu_state.accelerometer = [4.0, 5.0, 6.0]
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            lowstate_pub.publish(message)
            rclpy.spin_once(probe, timeout_sec=0.05)

        assert received
        output = received[-1]
        assert output.header.frame_id == "body_imu"
        assert output.header.stamp.sec > 0
        assert (
            output.orientation.w,
            output.orientation.x,
            output.orientation.y,
            output.orientation.z,
        ) == pytest.approx((
            0.9,
            0.1,
            0.2,
            0.3,
        ))
        assert (
            output.angular_velocity.x,
            output.angular_velocity.y,
            output.angular_velocity.z,
        ) == pytest.approx((
            1.0,
            2.0,
            3.0,
        ))
        assert (
            output.linear_acceleration.x,
            output.linear_acceleration.y,
            output.linear_acceleration.z,
        ) == pytest.approx((4.0, 5.0, 6.0))
    finally:
        probe.destroy_subscription(body_imu_sub)
        probe.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5.0)
