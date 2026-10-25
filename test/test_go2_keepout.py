"""Approved mask, launch wiring and fail-closed readiness; no robot inputs."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
MASK = ROOT/'maps/lab_rescan_20261018_155656_0pa6gm/keepout_boundary_20261022_29ab5c38/keepout_mask.yaml'


def costmap(mask, value=254):
    import time
    from nav2_msgs.msg import Costmap
    message = Costmap()
    message.header.frame_id = 'map'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 10**9)
    m = message.metadata
    m.size_y, m.size_x = mask.blocked.shape
    m.resolution = mask.resolution
    m.origin.position.x, m.origin.position.y, _ = mask.origin
    m.origin.orientation.w = 1.
    message.data = np.where(mask.blocked, value, 0).astype('uint8').ravel().tolist()
    return message


def test_approved_mask_preserves_internal_unknown_and_forbids_outside():
    from vlm_nav.go2_keepout import KeepoutMask
    mask = KeepoutMask(MASK)
    assert mask.blocked.shape == (578, 472)
    assert mask.origin == [-8.09, -20.8, 0]
    assert mask.blocked.sum() == 230319
    assert mask.applied_to(costmap(mask))
    message = costmap(mask)
    message.data[int(np.flatnonzero(mask.blocked)[0])] = 0
    assert not mask.applied_to(message)


@pytest.mark.parametrize('bad', ['missing', 'unknown', 'inscribed', 'frame', 'origin', 'resolution', 'size', 'payload'])
def test_unapplied_or_misaligned_mask_never_reports_ready(bad):
    from vlm_nav.go2_keepout import KeepoutMask
    mask = KeepoutMask(MASK)
    message = costmap(mask, 255 if bad == 'unknown' else 253 if bad == 'inscribed' else 254)
    if bad == 'missing': message = None
    if bad == 'frame': message.header.frame_id = 'odom'
    if bad == 'origin': message.metadata.origin.position.x += .05
    if bad == 'resolution': message.metadata.resolution = .1
    if bad == 'size': message.metadata.size_x -= 1
    if bad == 'payload': message.data = []
    assert not mask.applied_to(message)


def load_launch(path):
    spec = importlib.util.spec_from_file_location('keepout_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('enabled', [False, True])
def test_navigation_launch_wires_keepout_only_when_requested(monkeypatch, enabled):
    from launch import LaunchContext
    module = load_launch(ROOT/'launch/go2_navigation.launch.py')
    real = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda p: str(ROOT) if p == 'vlm_nav' else real(p))
    context = LaunchContext()
    context.launch_configurations.update(localization_mode='amcl', overrides_file=str(ROOT/'config/nav2_go2.yaml'),
        amcl_scan_topic='/scan', costmap_overrides_file='', keepout_mask=str(MASK) if enabled else '')
    actions = module.configure_go2_navigation(context)
    params = yaml.safe_load(actions[0]._Node__parameters[0].evaluate(context).read_text())
    servers = [n for n in actions if getattr(n, 'node_executable', '') in ('map_server', 'costmap_filter_info_server')]
    assert len(servers) == (2 if enabled else 0)
    for name in ('local_costmap', 'global_costmap'):
        p = params[name][name]['ros__parameters']
        assert ('go2_keepout_filter' in p.get('filters', [])) is enabled
        assert p['plugins'] == ['static_layer', 'inflation_layer']
        if enabled:
            assert p['go2_keepout_filter']['plugin'] == 'nav2_costmap_2d::KeepoutFilter'
            assert p['go2_keepout_filter']['filter_info_topic'] == '/vlm_nav/keepout_filter_info'
    manager = next(n for n in actions if getattr(n, '_Node__node_name', '') == 'lifecycle_manager_navigation')
    manager_params = manager._Node__parameters[0]
    # Normalized launch parameters use tuple keys and substitutions.
    from launch_ros.utilities import evaluate_parameters
    values = evaluate_parameters(context, [manager_params])[0]
    assert ('go2_keepout_mask_server' in values['node_names']) is enabled
    if enabled:
        assert tuple(values['node_names'][:2]) == ('go2_keepout_mask_server', 'go2_keepout_filter_info_server')


def test_requested_missing_mask_fails_before_nodes_are_created():
    from vlm_nav.go2_keepout import configure_keepout
    with pytest.raises((OSError, ValueError)):
        configure_keepout({}, '/missing/keepout_mask.yaml')


def test_readonly_and_scan_adapters_subscribe_global_map_when_keepout_required():
    from multi_agent_nav.adapters.go2 import Go2Adapter
    topics = []
    observer = SimpleNamespace(create_subscription=lambda kind, topic, *args: topics.append(topic))
    adapter = Go2Adapter(observer, dict(api_timeout=1., keepout_mask=str(MASK),
        goal_costmap_topic='/global_costmap/costmap_raw'), None, read_only=True)
    try:
        assert '/global_costmap/costmap_raw' in topics
    finally:
        adapter.vision.close()


def test_static_acceptance_can_load_same_filter_without_motion_servers(monkeypatch):
    from launch import LaunchContext
    coop = ROOT/'cooperative_ws/src/multi_agent_nav'
    module = load_launch(coop/'launch/go2_costmap_acceptance.launch.py')
    real = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory',
        lambda p: str(coop) if p == 'multi_agent_nav' else str(ROOT) if p == 'vlm_nav' else real(p))
    context = LaunchContext()
    context.launch_configurations['keepout_mask'] = str(MASK)
    actions = module.build(context)
    assert {n.node_executable for n in actions} == {'multi_agent_costmap_acceptance', 'lifecycle_manager',
        'map_server', 'costmap_filter_info_server'}
    params = yaml.safe_load(actions[0]._Node__parameters[0].evaluate(context).read_text())
    assert all(p['ros__parameters']['filters'] == ['go2_keepout_filter'] for p in params.values())


def test_supervisor_requires_applied_fresh_mask_and_active_servers(monkeypatch):
    import time
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from unittest.mock import Mock
    from vlm_nav.go2_nav_supervisor import Go2NavSupervisor
    from vlm_nav.go2_keepout import KEEP_OUT_NODES
    monkeypatch.setenv('ROS_DOMAIN_ID', '31')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    rclpy.init(args=['--ros-args', '-p', 'localization_mode:=amcl', '-p', f'keepout_mask:={MASK}'])
    node = Go2NavSupervisor()
    try:
        node._request_lifecycle_states = lambda: None
        node.count_publishers = lambda _: 1
        published = []
        node._publish = published.append
        now = time.monotonic()
        node.system_ready = True
        node.system_ready_receipt = now
        node.receipts.update(dict.fromkeys(('obstacle', 'scan', 'costmap', 'map'), now))
        node.map_valid = True
        node.lifecycle_active = dict.fromkeys(node.NAV2_NODES, True)
        node.lifecycle_receipts = dict.fromkeys(node.NAV2_NODES, now)
        node.tf_buffer = Mock()
        node.tf_buffer.can_transform.return_value = True
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        node.tf_buffer.lookup_transform.return_value = transform
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.
        node._on_amcl_pose(pose)
        node._tick()
        assert not published[-1]
        node._on_keepout_costmap(costmap(node.keepout_mask, 0))
        node._tick()
        assert not published[-1]
        node._on_keepout_costmap(costmap(node.keepout_mask))
        node._tick()
        assert published[-1]
        old_message = costmap(node.keepout_mask)
        old_message.header.stamp.sec -= 10
        node._on_keepout_costmap(old_message)
        node._tick()
        assert not published[-1]
        delayed_message = costmap(node.keepout_mask)
        delayed_message.header.stamp.sec -= 1
        node._on_keepout_costmap(delayed_message)
        node.receipts['keepout_costmap'] -= 1.2  # Transit age plus time without a new sample.
        node._tick()
        assert not published[-1]
        node._on_keepout_costmap(costmap(node.keepout_mask))
        node.receipts['keepout_costmap'] -= 10
        node._tick()
        assert not published[-1]
        node._on_keepout_costmap(costmap(node.keepout_mask))
        for name in KEEP_OUT_NODES:
            assert name in node.lifecycle_clients
            node.lifecycle_active[name] = False
            node._tick()
            assert not published[-1]
            node.lifecycle_active[name] = True
    finally:
        node.destroy_node()
        rclpy.shutdown()
