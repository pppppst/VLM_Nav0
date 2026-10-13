#!/usr/bin/env python3
"""Read-only RViz furniture and viewpoint markers in the native Go2 map frame."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml
from render_semantic_map import finite, load_map, sha256, validate_polygon


def load_overlay(path):
    path = Path(path).resolve()
    catalog = yaml.safe_load(path.read_text())
    if (catalog['schema_version'] != 1 or catalog['robot_id'] != 'go2'
            or catalog['frame_id'] != 'map' or not catalog.get('map_id')):
        raise ValueError('Go2 map-frame catalog required')
    semantic_path = path.parent / catalog['semantic_yaml']
    if sha256(semantic_path) != catalog['semantic_yaml_sha256']:
        raise ValueError('semantic YAML digest mismatch')
    semantic = yaml.safe_load(semantic_path.read_text())
    if (semantic['schema_version'] != 1 or semantic['source_robot'] != 'go2'
            or semantic['frame_id'] != 'map' or semantic['units'] != 'meters'):
        raise ValueError('Go2 map-frame semantic map required')
    source = load_map(semantic_path.parent / semantic['source_map']['yaml'])
    if source != semantic['source_map']:
        raise ValueError('source map metadata/digest mismatch')
    digest = hashlib.sha256((source['yaml_sha256'] + source['image_sha256']).encode()).hexdigest()
    if catalog['map_version'] != 'sha256:' + digest:
        raise ValueError('catalog map version mismatch')
    furniture = {}
    for item in semantic['furniture']:
        fid = item['id']
        if not isinstance(fid, str) or not fid or fid in furniture:
            raise ValueError('invalid/duplicate furniture ID')
        validate_polygon(item['polygon'], source)
        furniture[fid] = item
    seen = set()
    for point in catalog['viewpoints']:
        pid, fid = point['viewpoint_id'], point.get('furniture_id')
        if not isinstance(pid, str) or not pid or pid in seen:
            raise ValueError('invalid/duplicate viewpoint ID')
        seen.add(pid)
        if type(point['enabled']) is not bool or (fid is not None and fid not in furniture):
            raise ValueError('invalid binding/enabled flag: ' + pid)
        if point['enabled'] and fid is None:
            raise ValueError('enabled viewpoint must be bound: ' + pid)
        pose = point['pose']
        if not all(finite(pose[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('finite pose required for visualization: ' + pid)
    return semantic, catalog


def make_markers(semantic, catalog):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    output = MarkerArray()
    clear = Marker();clear.action = Marker.DELETEALL;clear.header.frame_id = 'map'
    output.markers.append(clear)
    colors = [(30/255,110/255,230/255), (230/255,95/255,25/255), (.1,.6,.3)]
    centers, palette = {}, {}

    def marker(kind, color, x=0., y=0., z=.08):
        item = Marker()
        item.header.frame_id = 'map'
        item.ns = 'semantic_viewpoints'
        item.id = len(output.markers)
        item.type = kind;item.action = Marker.ADD
        item.pose.position.x = float(x);item.pose.position.y = float(y);item.pose.position.z = z
        item.pose.orientation.w = 1.
        item.color.r, item.color.g, item.color.b = color
        item.color.a = 1.
        output.markers.append(item)
        return item

    for index, furniture in enumerate(semantic['furniture']):
        fid, polygon = furniture['id'], furniture['polygon']
        color = palette[fid] = colors[index % len(colors)]
        center = centers[fid] = [sum(p[i] for p in polygon)/len(polygon) for i in (0,1)]
        outline = marker(Marker.LINE_STRIP, color)
        outline.scale.x = .045
        outline.points = [Point(x=float(x), y=float(y), z=0.) for x,y in polygon + polygon[:1]]
        label = marker(Marker.TEXT_VIEW_FACING, color, *center, z=.2)
        label.scale.z = .25;label.text = fid
    for point in catalog['viewpoints']:
        pose, fid = point['pose'], point.get('furniture_id')
        x,y,yaw = (pose[k] for k in ('x','y','yaw'))
        color = palette[fid] if fid else (.4,.4,.4)
        arrow = marker(Marker.ARROW, color, x,y)
        arrow.pose.orientation.z = math.sin(yaw/2);arrow.pose.orientation.w = math.cos(yaw/2)
        arrow.scale.x = .7;arrow.scale.y = .12;arrow.scale.z = .16
        # Put each label above its point in map XY so top-down labels do not cover arrows.
        label = marker(Marker.TEXT_VIEW_FACING, color, x,y+.45, z=.2)
        label.scale.z = .16
        label.text = (f"{point['viewpoint_id']} -> {fid or 'UNBOUND'}\n"
                      f"{'enabled' if point['enabled'] else 'disabled'} | yaw={yaw:.2f}")
        if fid:
            link = marker(Marker.LINE_STRIP, color)
            link.scale.x = .02;link.color.a = .55
            link.points = [Point(x=float(x),y=float(y),z=0.),
                           Point(x=float(centers[fid][0]),y=float(centers[fid][1]),z=0.)]
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observation-yaml', required=True, type=Path)
    parser.add_argument('--check', action='store_true', help='validate and print poses without initializing ROS')
    args = parser.parse_args()
    semantic, catalog = load_overlay(args.observation_yaml)
    if args.check:
        print(json.dumps(dict(map_id=catalog['map_id'], viewpoints=catalog['viewpoints']), ensure_ascii=False, indent=2))
        return
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from visualization_msgs.msg import MarkerArray
    rclpy.init()
    node = rclpy.create_node('go2_semantic_viewpoints')
    publisher = node.create_publisher(MarkerArray, '/go2_point_annotation/semantic_markers',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    markers = make_markers(semantic, catalog)
    publisher.publish(markers)
    node.create_timer(1., lambda: publisher.publish(markers))
    print('家具/观察点标记已发布；UNBOUND 表示未绑定。修改 YAML 后重启本程序；Ctrl+C 退出。', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
