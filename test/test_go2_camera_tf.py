"""Isolated ROS tests: no sensors, Nav2, Sport or hardware commands."""
import time
import json
import os
import signal
import subprocess
import uuid
import pty
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def graph(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '31')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    def wait(condition, timeout=6.0):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
            if condition():
                return
        assert condition(), 'ROS condition did not converge'

    yield add, wait
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


class GuardProcess(Node):
    """Test harness for the real C++ executable; receives no hardware topics."""
    def __init__(self):
        from ament_index_python.packages import get_package_prefix
        from vlm_nav.go2_camera_tf import camera_tf_parameters
        name = 'go2_camera_tf_' + uuid.uuid4().hex[:8]
        super().__init__('probe_' + name)
        default = Path(get_package_prefix('vlm_nav')) / 'lib/vlm_nav/go2_camera_tf'
        binary = Path(os.environ.get('GO2_CAMERA_TF_EXECUTABLE', default))
        assert binary.exists(), f'Build the camera TF target first: {binary}'
        arguments = [str(binary), '--ros-args', '-r', f'__node:={name}']
        for key, value in camera_tf_parameters(str(ROOT / 'config/go2_calibration_xt16.yaml')).items():
            arguments.extend(['-p', f'{key}:={json.dumps(value)}'])
        self.process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        os.set_blocking(self.process.stdout.fileno(), False)
        self.logs = b''

    @property
    def state(self):
        try:
            self.logs += os.read(self.process.stdout.fileno(), 65536)
        except BlockingIOError:
            pass
        if self.process.poll() is not None:
            assert self.process.returncode == 1, self.logs.decode()
            assert b'duplicate or inconsistent camera mount TF sources' in self.logs, self.logs.decode()
            return 'fault'
        if self.logs.rfind(b'Reusing existing') > self.logs.rfind(b'Publishing configured'):
            return 'reusing'
        return 'publishing' if b'Publishing configured' in self.logs else 'waiting'

    def destroy_node(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        assert self.process.returncode in (0, 1), self.logs.decode()
        self.process.stdout.close()
        return super().destroy_node()


def guard(add):
    return add(GuardProcess())


def external(add, name, x=0.32715, parent='base_link'):
    node = add(Node(name))
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = parent
    tf.child_frame_id = 'camera_link'
    tf.transform.translation.x = x
    tf.transform.translation.y = -0.00003
    tf.transform.translation.z = 0.04297
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    return node, broadcaster


def publishers(node):
    return node.get_publishers_info_by_topic('/tf_static')


def test_two_entries_publish_one_configured_transform(graph):
    add, wait = graph
    first, second = guard(add), guard(add)
    probe = add(Node('camera_tf_probe'))
    received = []
    from rclpy.qos import QoSProfile, DurabilityPolicy
    probe.create_subscription(TFMessage, '/tf_static', received.append,
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(lambda: received and {first.state, second.state} == {'publishing', 'reusing'})
    assert len(publishers(probe)) == 1
    tf = received[-1].transforms[0]
    assert (tf.header.frame_id, tf.child_frame_id) == ('base_link', 'camera_link')
    assert [tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z] == [0.32715, -0.00003, 0.04297]
    assert [tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w] == [0.0, 0.0, 0.0, 1.0]


def test_existing_driver_or_urdf_transform_is_reused(graph):
    add, wait = graph
    external(add, 'existing_camera_mount')
    node = guard(add)
    wait(lambda: node.state == 'reusing')
    assert len(publishers(node)) == 1


def test_late_external_transform_withdraws_owned_publisher(graph):
    add, wait = graph
    node = guard(add)
    wait(lambda: node.state == 'publishing')
    external(add, 'late_camera_mount')
    wait(lambda: node.state == 'reusing' and len(publishers(node)) == 1)
    assert publishers(node)[0].node_name == 'late_camera_mount'


@pytest.mark.parametrize('x,parent', [(0.9, 'base_link'), (0.32715, 'other_base')])
def test_conflicting_mount_fails_without_a_second_publisher(graph, x, parent):
    add, wait = graph
    external(add, 'wrong_camera_mount', x=x, parent=parent)
    node = guard(add)
    wait(lambda: node.state == 'fault')
    wait(lambda: len(publishers(node)) == 1)


def test_two_existing_sources_are_reported_as_a_fault(graph):
    add, wait = graph
    external(add, 'existing_mount_a')
    external(add, 'existing_mount_b')
    node = guard(add)
    wait(lambda: node.state == 'fault')
    wait(lambda: len(publishers(node)) == 2)


def test_optical_frame_cannot_be_used_as_the_mount(graph, tmp_path):
    from vlm_nav.go2_camera_tf import camera_tf_parameters
    config = yaml.safe_load((ROOT / 'config/go2_calibration.yaml').read_text())
    config['camera_extrinsic']['child_frame'] = 'camera_color_optical_frame'
    path = tmp_path / 'wrong-frame.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='base_link.*camera_link'):
        camera_tf_parameters(str(path))


def test_cooperative_observer_receives_adopted_calibration_without_motion(graph):
    from multi_agent_nav.config import load_config
    from multi_agent_nav.adapters.go2 import make_observer
    add, _ = graph
    config = load_config(ROOT / 'cooperative_ws/src/multi_agent_nav/config')
    node = add(make_observer(config['robot_go2'], read_only=True))
    assert node.get_parameter('camera_extrinsic_calibrated').value is True
    assert node.p.camera_frame == 'camera_color_optical_frame'
    assert node.p.raw_depth_mode is True and node.p.enabled is False
    assert node.p.publish_stop_command is False


def test_old_script_rejects_overlay_without_camera_owner(tmp_path):
    script = tmp_path / 'scripts'
    script.mkdir()
    (script / 'manualnav2.sh').write_bytes((ROOT / 'scripts/manualnav2.sh').read_bytes())
    # Replace only environment discovery; a missing owner must stop before network/hardware setup.
    (script / 'common.sh').write_text('ros2() { echo /nonexistent/old-overlay; }\n')
    master, slave = pty.openpty()
    try:
        result = subprocess.run(['bash', str(script / 'manualnav2.sh')], stdin=slave,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5,
                                env=dict(os.environ, VLM_NAV_RUN_ROOT=str(tmp_path / 'runs')))
    finally:
        os.close(master)
        os.close(slave)
    assert result.returncode != 0
    assert 'Go2 camera TF owner missing from active overlay' in result.stderr
