"""Local lifecycle from runner at 1acae1d; task/peer validity stays with callers."""
import time

from vlm_nav.arrival import target_distance


def local_target_accepted(health, target, success_radius):
    distance = target_distance((health.pose.x, health.pose.y),
        (target.x, target.y) if target else None)
    return (health.nav_ready and health.localization_valid and health.stopped
            and health.motion_terminal and distance <= success_radius)


class LocalApproachExecution:
    def __init__(self, adapter, target, task_id, command_id, kind, parameters, record,
                 clock=time.monotonic):
        from .single_robot_approach import SingleRobotApproach
        self.adapter = adapter
        self.approach = SingleRobotApproach(adapter, target, task_id, command_id,
            kind, parameters, record, clock)

    def run(self, check, on_tick=lambda: None):
        from .single_robot_approach import SUCCEEDED, FAILED
        approach = self.approach
        self.adapter.approach = approach
        try:
            while approach.state not in (SUCCEEDED, FAILED):
                check()
                approach.tick()
                on_tick()
                yield
            check()
            if approach.state == FAILED:
                if getattr(approach, 'distance_rejected', False):return 'APPROACH_DISTANCE_FAILED'
                return 'APPROACH_FAILED: ' + approach.reason
            return 'TARGET_APPROACHED'
        finally:
            if approach.state != SUCCEEDED:approach.close()
            self.adapter.approach = None

    def accepted(self):
        approach = self.approach
        approach.emit('TARGET_APPROACHED',
            measured_distance=approach.target_reference_distance(approach.robot_pose()),
            arrival_retry=approach.arrival_retry_count)
