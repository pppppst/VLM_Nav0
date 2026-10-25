#!/usr/bin/env python3
"""Display Go2 and coordinator reports without controlling the task."""
from datetime import datetime
import json
import os
import time


class StatusDisplay:
    def __init__(self, lost_timeout):
        self.lost_timeout = lost_timeout
        self.latest = {}
        self.received_at = {}
        self.errors = {}
        self.last_text = None
        self.last_output_at = None

    def receive_go2(self, message, now):
        if message.robot_id != 'go2' or message.schema_version != 1 or message.simulation:
            self.errors['go2'] = 'Go2消息身份不符：要求 robot_id=go2、schema_version=1、simulation=false'
            return
        self.latest['go2'] = {name: getattr(message, name) for name in (
            'task_id', 'run_state', 'reason', 'localization_valid', 'nav_ready', 'stopped')}
        self.received_at['go2'] = now
        self.errors.pop('go2', None)

    def receive_coordinator(self, raw, now):
        try:
            message = json.loads(raw)
        except (ValueError, TypeError):
            message = None
        if not isinstance(message, dict) or not all(
                isinstance(message.get(name), str) for name in ('task_id', 'phase', 'reason')):
            self.errors['coordinator'] = '协调器消息格式错误：需要字符串 task_id、phase、reason'
            return
        self.latest['coordinator'] = {name: message[name] for name in ('task_id', 'phase', 'reason')}
        self.received_at['coordinator'] = now
        self.errors.pop('coordinator', None)

    def render(self, now):
        agent = self.latest.get('go2')
        coordinator = self.latest.get('coordinator')
        expired = {source: source in self.received_at and now-self.received_at[source] > self.lost_timeout
                   for source in ('go2', 'coordinator')}
        suffix = {source: '（最后报告）' if expired[source] else '' for source in expired}
        flag = lambda name: '未知' if agent is None else '是' if agent[name] else '否'
        lines = [
            'Go2 当前状态',
            'Go2：'+(agent['run_state']+suffix['go2'] if agent else '等待消息'),
            '原因：'+(agent['reason'] or '无' if agent else '等待消息'),
            '任务阶段：'+(coordinator['phase']+suffix['coordinator'] if coordinator else '等待消息'),
            '协调器原因：'+(coordinator['reason'] or '无' if coordinator else '等待消息'),
            f'定位有效：{flag("localization_valid")}    导航就绪：{flag("nav_ready")}    停稳报告：{flag("stopped")}',
            'Go2任务ID：'+(agent['task_id'] or '无任务' if agent else '等待消息'),
            '协调器任务ID：'+(coordinator['task_id'] or '无任务' if coordinator else '等待消息'),
        ]
        for source, label in (('go2', 'Go2'), ('coordinator', '协调器')):
            freshness = ('等待消息' if source not in self.received_at else
                         '消息已过期（仅为最后报告）' if expired[source] else '新鲜')
            lines.append(f'{label}消息：{freshness}')
            if source in self.errors:
                lines.append(self.errors[source])
        if agent and coordinator and agent['task_id'] != coordinator['task_id']:
            lines.append('任务ID不一致：请分别审查两端报告，勿合并判断。')
        return '\n'.join(lines)

    def emit(self, now):
        text = self.render(now)
        if text != self.last_text or self.last_output_at is None or now-self.last_output_at >= 5.:
            print(f'[{datetime.now():%Y-%m-%d %H:%M:%S}] {text}\n', flush=True)
            self.last_text, self.last_output_at = text, now


def main():
    from multi_agent_nav.config import load_config
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from multi_agent_task_msgs.msg import AgentState
    from std_msgs.msg import String

    config = load_config(os.environ['COOP_CONFIG'])
    display = StatusDisplay(config['scene'].get('lost_timeout', 3.))
    print('本窗口只读；停稳为消息报告，不代表实体停稳或DISARM。Ctrl+C仅结束显示。', flush=True)
    node = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node(f'go2_status_watch_{os.getpid()}',
                                enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        def receive_go2(message):
            now = time.monotonic()
            display.receive_go2(message, now)
            display.emit(now)

        def receive_coordinator(message):
            now = time.monotonic()
            display.receive_coordinator(message.data, now)
            display.emit(now)

        node.create_subscription(AgentState, '/multi_agent/go2/state', receive_go2, qos)
        node.create_subscription(String, '/multi_agent/coordinator/status', receive_coordinator, qos)
        node.create_timer(.5, lambda: display.emit(time.monotonic()))
        display.emit(time.monotonic())
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
