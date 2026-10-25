"""Actual Nav2 costmaps, mask servers and synthetic inputs on localhost Domain31."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
MASK = ROOT/'maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261022_29ab5c38/keepout_mask.yaml'


def test_native_keepout_and_dynamic_obstacle_clearing(tmp_path, monkeypatch):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage
    from vlm_nav.go2_keepout import KeepoutMask

    for key, value in dict(ROS_DOMAIN_ID='31', ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(tmp_path)).items():
        monkeypatch.setenv(key, value)
    for key in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(key, raising=False)
    path = ROOT/'cooperative_ws/src/multi_agent_nav/launch/go2_costmap_acceptance.launch.py'
    spec = importlib.util.spec_from_file_location('keepout_native_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    real = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda p:
        str(ROOT) if p == 'vlm_nav' else str(path.parents[1]) if p == 'multi_agent_nav' else real(p))
    context = LaunchContext()
    context.launch_configurations['keepout_mask'] = str(MASK)
    actions = module.build(context)
    parameter_file = actions[0]._Node__parameters[0].evaluate(context)
    native = Path(get_package_prefix('multi_agent_nav'))/'lib/multi_agent_nav/multi_agent_costmap_acceptance'
    assert native.exists()
    commands = [[str(native), '--ros-args', '--params-file', str(parameter_file)]]
    for action in actions[1:]:
        values = evaluate_parameters(context, action._Node__parameters)[0]
        params = tmp_path/(action._Node__node_name+'.yaml')
        import yaml
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}))
        package = 'nav2_lifecycle_manager' if action.node_executable == 'lifecycle_manager' else 'nav2_map_server'
        binary = Path(get_package_prefix(package))/'lib'/package/action.node_executable
        commands.append([str(binary), '--ros-args', '-r', '__node:='+action._Node__node_name,
                         '--params-file', str(params)])

    mask = KeepoutMask(MASK)
    rclpy.init(args=[])
    node = rclpy.create_node('keepout_synthetic_probe', enable_rosout=False)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, '/map', durable)
    tf_pub = node.create_publisher(TFMessage, '/tf_static', durable)
    from sensor_msgs.msg import PointCloud2
    cloud_pub = node.create_publisher(PointCloud2, '/vlm_nav/obstacle_cloud', 10)
    latest = {}
    for name in ('local_costmap', 'global_costmap'):
        node.create_subscription(Costmap, f'/go2_costmap_acceptance/{name}/costmap_raw',
            lambda m, name=name: latest.update({name: m}), durable)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = mask.resolution
    grid.info.height, grid.info.width = mask.blocked.shape
    grid.info.origin.position.x, grid.info.origin.position.y, _ = mask.origin
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*mask.blocked.size  # Prove outside costs originate from the filter.
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z'))]
    processes, streams, forced = [], [], []

    def sample(message, x, y):
        if message.header.frame_id == 'odom':
            x, y = x-6.8, y+13.5
        m = message.metadata
        ix, iy = int((x-m.origin.position.x)/m.resolution), int((y-m.origin.position.y)/m.resolution)
        if not (0 <= ix < m.size_x and 0 <= iy < m.size_y): return None
        return message.data[iy*m.size_x+ix]

    def spin_until(predicate, obstacle_x, seconds=20):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            grid.header.stamp = stamp
            grid_pub.publish(grid)
            transforms = []
            for parent, child, x, y in [('map', 'odom', 6.8, -13.5), ('odom', 'base_link', 0., 0.)]:
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id, t.header.stamp = parent, child, stamp
                t.transform.translation.x, t.transform.translation.y = x, y
                t.transform.rotation.w = 1.
                transforms.append(t)
            tf_pub.publish(TFMessage(transforms=transforms))
            # A narrow fan clears both grids despite their different cell origins.
            offsets = [0.] if obstacle_x == .8 else [i*.025 for i in range(-6, 7)]
            cloud_pub.publish(create_cloud(Header(stamp=stamp, frame_id='base_link'), fields,
                [(obstacle_x, .125*obstacle_x+offset, 0.) for offset in offsets]))
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate(): return True
            if any(p.poll() is not None for p in processes): return False
        return bool(predicate())

    try:
        for index, command in enumerate(commands):
            stream = (tmp_path/f'native_{index}.log').open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, env=os.environ.copy(), stdout=stream, stderr=subprocess.STDOUT))
        assert spin_until(lambda: len(latest) == 2 and mask.applied_to(latest['global_costmap'])
            and sample(latest['local_costmap'], 7.6, -12.5) == 254
            and all(sample(m, 7.6, -13.4) >= 253 for m in latest.values()), .8)
        assert spin_until(lambda: all(sample(m, 7.6, -13.4) == 0 for m in latest.values())
            and mask.applied_to(latest['global_costmap'])
            and sample(latest['local_costmap'], 7.6, -12.5) == 254, 3.)
        for x in (4.435, 5.435, 9.435):
            assert sample(latest['global_costmap'], x, -13.275) < 253
        assert sample(latest['global_costmap'], 7.435, -15.275) == 254
        print('Native global/local keepout, P1/P5/P10 inside, P3 blocked, dynamic marking/clearing: PASS')
    finally:
        import numpy as np
        for name, message in latest.items():
            m = message.metadata
            details = dict(frame=message.header.frame_id, size=[m.size_y,m.size_x],
                origin=[m.origin.position.x,m.origin.position.y], resolution=m.resolution,
                outside_sample=sample(message,7.6,-12.5), obstacle_sample=sample(message,7.6,-13.4))
            if name == 'global_costmap' and (m.size_y,m.size_x) == mask.blocked.shape:
                data = np.asarray(message.data).reshape(mask.blocked.shape)
                mismatch = np.argwhere(mask.blocked & (data != 254))
                details.update(applied=mask.applied_to(message), wrong_outside_cells=len(mismatch),
                    mismatch_examples=mismatch[:8].tolist(), outside_costs=np.unique(data[mask.blocked]).tolist())
            print(name, details)
        for p in processes:
            if p.poll() is None: p.send_signal(signal.SIGINT)
        for p in processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                forced.append(p.pid)
                p.kill()
                p.wait()
        for stream in streams: stream.close()
        for p in tmp_path.glob('native_*.log'): print(p.name+': '+p.read_text()[-2000:])
        node.destroy_node()
        rclpy.shutdown()
    assert not forced and all(p.returncode == 0 for p in processes)
