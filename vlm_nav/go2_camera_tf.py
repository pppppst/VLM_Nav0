"""Build the single-owner camera TF node from the selected calibration file."""
import math
import uuid
import yaml
from vlm_nav.go2_preflight import require_calibrated


def camera_tf_parameters(calibration_path):
    with open(calibration_path, encoding='utf-8') as stream:
        camera = yaml.safe_load(stream)['camera_extrinsic']
    require_calibrated('camera_extrinsic', camera)
    if (camera['parent_frame'], camera['child_frame']) != ('base_link', 'camera_link'):
        raise ValueError('Go2 camera mount must be base_link -> camera_link')
    for field in ('xyz', 'rpy'):
        if len(camera[field]) != 3 or not all(math.isfinite(v) for v in camera[field]):
            raise ValueError('camera xyz/rpy must contain three finite values')
    return {key: camera[key] for key in ('parent_frame', 'child_frame', 'xyz', 'rpy')}


def camera_tf_action(calibration_path):
    from launch_ros.actions import Node
    return Node(package='vlm_nav', executable='go2_camera_tf', output='screen',
                name='go2_camera_tf_' + uuid.uuid4().hex[:8],
                parameters=[camera_tf_parameters(calibration_path)])
