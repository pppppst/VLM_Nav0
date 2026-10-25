"""Native Go2 KeepoutFilter wiring and evidence that the global mask is applied."""
from pathlib import Path
import math

import cv2
import numpy as np
import yaml

KEEP_OUT_NODES = ('go2_keepout_mask_server', 'go2_keepout_filter_info_server')
MASK_TOPIC = '/vlm_nav/keepout_mask'
INFO_TOPIC = '/vlm_nav/keepout_filter_info'


class KeepoutMask:
    def __init__(self, path):
        self.path = Path(path).resolve()
        data = yaml.safe_load(self.path.read_text())
        self.resolution = data['resolution']
        self.origin = data['origin']
        if (data.get('mode', 'trinary') != 'trinary' or data['negate'] != 0
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin)
                or self.origin[2] != 0
                or not 0 < data['free_thresh'] < data['occupied_thresh'] < 1):
            raise ValueError('finite, unrotated binary trinary keepout mask required')
        pixels = cv2.imread(str(self.path.parent/data['image']), cv2.IMREAD_GRAYSCALE)
        if pixels is None or set(np.unique(pixels)) != {0, 255}:
            raise ValueError('keepout mask must contain black forbidden and white unrestricted cells')
        self.blocked = np.flipud(pixels == 0)

    def applied_to(self, message):
        if message is None or message.header.frame_id != 'map':
            return False
        m = message.metadata
        q = m.origin.orientation
        if ((m.size_y, m.size_x) != self.blocked.shape
                or len(message.data) != self.blocked.size
                or not math.isclose(m.resolution, self.resolution, rel_tol=1e-6)
                or any(not math.isclose(a, b, abs_tol=1e-6) for a, b in zip(
                    (m.origin.position.x, m.origin.position.y, m.origin.position.z),
                    (self.origin[0], self.origin[1], 0.)))
                or any(not math.isfinite(v) or abs(v) > 1e-6 for v in (q.x, q.y, q.z))
                or not math.isclose(abs(q.w), 1., abs_tol=1e-6)):
            return False
        costs = np.asarray(message.data).reshape(self.blocked.shape)
        return bool(np.all(costs[self.blocked] == 254))


def configure_keepout(params, mask_yaml):
    if not mask_yaml:
        return []
    mask = KeepoutMask(mask_yaml)  # Fail before creating any navigation process.
    from launch_ros.actions import Node
    for name in ('local_costmap', 'global_costmap'):
        p = params[name][name]['ros__parameters']
        filters = p.setdefault('filters', [])
        if 'go2_keepout_filter' not in filters:
            filters.append('go2_keepout_filter')
        p['go2_keepout_filter'] = dict(plugin='nav2_costmap_2d::KeepoutFilter',
                                      enabled=True, filter_info_topic=INFO_TOPIC)
    return [
        Node(package='nav2_map_server', executable='map_server', name=KEEP_OUT_NODES[0],
             output='screen', parameters=[dict(use_sim_time=False, yaml_filename=str(mask.path),
                                              topic_name=MASK_TOPIC, frame_id='map')]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name=KEEP_OUT_NODES[1], output='screen', parameters=[dict(use_sim_time=False,
                 type=0, filter_info_topic=INFO_TOPIC, mask_topic=MASK_TOPIC, base=0., multiplier=1.)]),
    ]
