import copy
import math
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from show_semantic_viewpoints import load_overlay, make_markers

CATALOG = ROOT / 'maps/lab_rear120_20261001_IwQjMX/observation_points_go2.yaml'


def test_unbound_disabled_points_are_visible_with_exact_native_pose():
    from visualization_msgs.msg import Marker
    semantic, catalog = load_overlay(CATALOG)
    for point in catalog['viewpoints']:
        point.update(furniture_id=None, enabled=False)
    markers = make_markers(semantic, catalog).markers
    arrows = [m for m in markers if m.type == Marker.ARROW and m.action == Marker.ADD]
    assert len(arrows) == 2
    for arrow, point in zip(arrows, catalog['viewpoints']):
        pose = point['pose']
        assert arrow.header.frame_id == 'map'
        assert arrow.pose.position.x == pose['x']
        assert arrow.pose.position.y == pose['y']
        assert arrow.pose.orientation.z == pytest.approx(math.sin(pose['yaw']/2))
    assert sum('UNBOUND' in m.text and 'disabled' in m.text for m in markers) == 2
    assert markers[0].action == Marker.DELETEALL
    outlines = [m for m in markers if m.type == Marker.LINE_STRIP]
    assert len(outlines) == 2 and all(m.points[0] == m.points[-1] for m in outlines)


def test_bound_point_has_link_without_modifying_input():
    from visualization_msgs.msg import Marker
    semantic, catalog = load_overlay(CATALOG)
    for point in catalog['viewpoints']:
        point.update(furniture_id=None, enabled=False)
    catalog['viewpoints'][0]['furniture_id'] = 'table_01'
    before = copy.deepcopy(catalog)
    markers = make_markers(semantic, catalog).markers
    assert any(catalog['viewpoints'][0]['viewpoint_id'] + ' -> table_01' in m.text for m in markers)
    assert len([m for m in markers if m.type == Marker.LINE_STRIP and len(m.points) == 2]) == 1
    assert catalog == before


@pytest.mark.parametrize('fault', ['map', 'binding', 'pose', 'enabled', 'digest', 'duplicate'])
def test_rejects_bad_overlay_input(tmp_path, fault):
    catalog = yaml.safe_load(CATALOG.read_text())
    catalog['semantic_yaml'] = str(CATALOG.parent/'semantic_map.yaml')
    if fault == 'map':catalog['map_version'] = 'wrong'
    if fault == 'binding':catalog['viewpoints'][0]['furniture_id'] = 'missing'
    if fault == 'pose':catalog['viewpoints'][0]['pose']['yaw'] = float('nan')
    if fault == 'enabled':catalog['viewpoints'][0].update(enabled=True, furniture_id=None)
    if fault == 'digest':catalog['semantic_yaml_sha256'] = 'wrong'
    if fault == 'duplicate':catalog['viewpoints'][1]['viewpoint_id'] = catalog['viewpoints'][0]['viewpoint_id']
    path = tmp_path/'points.yaml';path.write_text(yaml.safe_dump(catalog))
    with pytest.raises(ValueError):load_overlay(path)


def test_rviz_uses_same_marker_topic_and_no_navigation_tool():
    config = yaml.safe_load((ROOT/'config/go2_semantic_annotation.rviz').read_text())['Visualization Manager']
    display = next(d for d in config['Displays'] if d['Class'].endswith('/MarkerArray'))
    assert display['Topic']['Value'] == '/go2_point_annotation/semantic_markers'
    assert display['Topic']['Durability Policy'] == 'Transient Local'
    assert not any('Goal' in t['Class'] or 'InitialPose' in t['Class'] for t in config['Tools'])
