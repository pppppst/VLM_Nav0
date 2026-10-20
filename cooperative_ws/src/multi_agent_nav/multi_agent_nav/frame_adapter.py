import math
from .models import Pose2D

def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


class LocalFrame:
    """Keep adapter positions in this robot's map; no common-map transform exists."""
    def to_local(self, pose):
        if not pose.valid:
            raise ValueError('nonfinite local pose')
        return pose

    to_common = to_local

    def polygon_to_local(self, vertices):
        return [(p.x, p.y) for p in (self.to_local(Pose2D(*v)) for v in vertices)]

class FrameAdapter:
    def __init__(self, local_map_id, map_version, common_frame_id, alignment_id, transform):
        if transform is None or len(transform) != 3 or not all(math.isfinite(v) for v in transform):
            raise ValueError('missing/nonfinite calibrated transform')
        self.ids = (local_map_id, map_version, common_frame_id, alignment_id)
        self.tx, self.ty, self.yaw = transform
        self.c, self.s = math.cos(self.yaw), math.sin(self.yaw)

    def validate_ids(self, local_map_id, map_version, common_frame_id, alignment_id):
        if (local_map_id, map_version, common_frame_id, alignment_id) != self.ids:
            raise ValueError('map/frame/alignment identity mismatch')

    def to_common(self, pose):
        if not pose.valid:
            raise ValueError('nonfinite pose')
        return Pose2D(self.c*pose.x-self.s*pose.y+self.tx,
                      self.s*pose.x+self.c*pose.y+self.ty, wrap(pose.yaw+self.yaw))

    def to_local(self, pose):
        if not pose.valid:
            raise ValueError('nonfinite pose')
        x, y = pose.x-self.tx, pose.y-self.ty
        return Pose2D(self.c*x+self.s*y, -self.s*x+self.c*y, wrap(pose.yaw-self.yaw))

    def polygon_to_local(self, vertices):
        return [(p.x, p.y) for p in (self.to_local(Pose2D(*v)) for v in vertices)]
