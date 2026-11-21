"""Keep manual Nav2 sessions alive; cancel a failed goal, never replay it."""
import os
from pathlib import Path
import time
from types import SimpleNamespace


class ActionStop:
    def __init__(self):
        self.active = set()
        self.finished = set()
        self.acknowledged = False

    def observe(self, message):
        for value in message.status_list:
            key = bytes(value.goal_info.goal_id.uuid)
            if value.status in (1, 2, 3) and key not in self.finished:self.active.add(key)
            elif value.status in (4, 5, 6):
                self.active.discard(key)
                self.finished.add(key)

    def begin(self):
        self.acknowledged = False

    def cancelled(self, response):
        self.acknowledged = response.return_code == 0
        self.active.update(bytes(g.goal_id.uuid) for g in response.goals_canceling
                           if bytes(g.goal_id.uuid) not in self.finished)

    @property
    def terminal(self):
        return self.acknowledged and not self.active


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    import yaml
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from ament_index_python.packages import get_package_share_directory
    from multi_agent_nav.adapters.go2 import Go2Adapter
    from multi_agent_nav.experiment_status import ExperimentStatus
    from .go2_control_recovery import ControlRecovery

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('go2_manual_recovery')
    recovery = ControlRecovery(node)
    cfg_path = Path(get_package_share_directory('multi_agent_nav'))/'config/robot_go2.yaml'
    config = yaml.safe_load(cfg_path.read_text())
    observer = SimpleNamespace(node=node,config=config,odom_at=0.,odom_stamp=None,stationary_since=None)
    stop = ActionStop()
    ready = [False, 0.]
    cancel = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
    publisher = node.create_publisher(Twist, '/cmd_vel_bridge', 10)
    latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry,config.get('odom_topic','/odometry'),
        lambda m:Go2Adapter.on_odometry(observer,m),qos_profile_sensor_data)
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',stop.observe,latched)
    node.create_subscription(Bool,'/vlm_nav/nav_ready',lambda m:ready.__setitem__(slice(None),[m.data,time.monotonic()]),latched)
    path = Path(os.environ['GO2_EXPERIMENT_STATUS_FILE'])
    status = ExperimentStatus(path.parent)
    status.path = path
    pending = None
    sent_at = 0.
    waiting = False
    was_ready = False
    last_status_at = 0.
    cancel_file = path.with_suffix('.cancel')
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            if cancel_file.exists():
                recovery.cancel()
                cancel_file.unlink()
            recovery.poll()
            snapshot = recovery.snapshot
            healthy = ready[0] and time.monotonic()-ready[1] <= recovery.max_age
            if healthy:was_ready = True
            if (recovery.hard_fault or snapshot.get('state') == 'FAULT' or (was_ready and not healthy)) and not waiting:
                stop.begin()
                publisher.publish(Twist())
                waiting = True
            if not waiting:
                if time.monotonic()-last_status_at >= .5:
                    status.update(state='RUNNING' if healthy and snapshot.get('state')=='ARMED' else 'WAITING',
                        phase='MANUAL_NAVIGATION',automatic=False,
                        reason='人工取消后本轮自动恢复已撤销' if recovery.closed else '手动导航待命，目标由RViz指定',
                        reason_class='MANUAL_REQUIRED' if recovery.closed else '',
                        health=dict(nav_ready=healthy,control_armed=snapshot.get('state')=='ARMED'))
                    last_status_at=time.monotonic()
                continue
            if pending is not None and time.monotonic()-sent_at > recovery.max_age:
                pending = None
            if pending is not None and pending.done():
                try:stop.cancelled(pending.result())
                except Exception:pass
                pending = None
            if not stop.acknowledged and pending is None and cancel.service_is_ready():
                pending = cancel.call_async(CancelGoal.Request())
                sent_at = time.monotonic()
            stopped = Go2Adapter.is_stopped(observer)
            recovery.step(stopped and stop.terminal and healthy)
            automatic = not recovery.closed and snapshot.get('recovery_allowed')=='true'
            status.update(state='WAITING',phase='MANUAL_NAVIGATION',
                reason=recovery.hard_fault or recovery.reason or ('等待 Nav2/TF 输入' if not healthy else '等待停车和动作终态'),
                reason_class='WAIT_INPUT' if automatic else 'MANUAL_REQUIRED',
                automatic=automatic,
                health=dict(stopped=stopped,motion_terminal=stop.terminal,nav_ready=healthy))
            if snapshot.get('state')=='ARMED' and stopped and stop.terminal and healthy and not recovery.closed:
                waiting=False
                status.update(state='RUNNING',reason='已恢复待命；请手动重新发送目标',automatic=False)
    except KeyboardInterrupt:
        pass
    finally:
        try:confirmed=recovery.finish_cancel(lambda:rclpy.spin_once(node,timeout_sec=.05),5.)
        except Exception:confirmed=False
        if not confirmed:
            status.update(state='STOP_UNCONFIRMED',reason='恢复授权取消未确认，请人工核对Sport状态',
                          reason_class='NON_RESUMABLE',automatic=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':main()
