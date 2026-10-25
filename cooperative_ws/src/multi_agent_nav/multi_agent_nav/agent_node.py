import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from multi_agent_task_msgs.msg import AgentState,Observation,Command
from .node_common import settings,qos
from .mock import make_machine
from .adapters.fake import FakeAdapter
from .runner import AgentRunner
from .ros_codec import to_ros,from_ros
from .models import Pose2D

class AgentNode(Node):
    def __init__(self):
        super().__init__('multi_agent_task_agent')
        self.declare_parameter('robot_id','go2')
        robot=self.get_parameter('robot_id').value
        if robot not in ('go2','ranger'):raise ValueError('unknown robot')
        config,simulation,prefix,gates=settings(self)
        self.observer=None
        self.blocked=bool(gates)
        self.machine=self.adapter=self.runner=None
        self.state_pub=self.create_publisher(AgentState,f'{prefix}/{robot}/state',qos())
        self.observation_pub=self.create_publisher(Observation,f'{prefix}/{robot}/observation',qos(True))
        if simulation:
            machine,frame=make_machine(config,robot,True)
            detections={'C':Pose2D(9,1)}
            if robot=='ranger':detections.update(A=Pose2D(-3,-3),B=Pose2D(3,-3))
            self.adapter=FakeAdapter(robot,detections,delay=.5)
            self.adapter.pose=frame.to_local(Pose2D(-3,-3) if robot=='go2' else Pose2D(3,-3))
            self.machine=machine
        elif not gates:
            if robot!='go2':raise ValueError('Ranger live adapter must be installed on Ranger PC')
            from .adapters.go2 import ABCClient,Go2Adapter,make_observer
            machine,frame=make_machine(config,robot,False)
            self.observer=make_observer(config['robot_go2'])
            self.adapter=Go2Adapter(self.observer,config['robot_go2'],frame,global_goal_checks=True)
            if not machine.context.share_c_coordinates:self.adapter.client=ABCClient(config['robot_go2'])
            self.machine=machine
        else:
            self.get_logger().warning('Live adapter not activated: '+'; '.join(gates))
        if self.machine:
            on_event=None
            if robot=='go2' and not simulation:
                from .exploration_display import ExplorationDisplay
                self.exploration_display=ExplorationDisplay(self.observer,self.machine.frame)
                self.adapter.display_path=lambda path:self.exploration_display.publish('planned_path',path)
                on_event=self.exploration_display.record
            self.runner=AgentRunner(self.machine,self.adapter,config['scene']['targets'],self.publish_observation,config=config,success_radius=1.0,on_event=on_event)
            if not self.machine.context.share_c_coordinates:
                from .go2_check import other_owners
                self.runner.owners=lambda:[name for name in other_owners(self.observer) if name not in
                    ('multi_agent_coordinator','multi_agent_go2_agent','multi_agent_task_agent','multi_agent_go2_rgbd')]
        self.robot,self.config,self.simulation=robot,config,simulation
        self.get_logger().info(f'Task prefix: {prefix}; robot={robot}; simulation={simulation}')
        self.create_subscription(Command,f'{prefix}/go2/command',self.command,qos())
        peer='ranger' if robot=='go2' else 'go2'
        self.create_subscription(AgentState,f'{prefix}/{peer}/state',lambda m:self.peer(m,peer),qos())
        self.create_subscription(Observation,f'{prefix}/{peer}/observation',lambda m:self.peer_observation(m,peer),qos(True))
        self.last_observations={}
        self.create_timer(.05,self.tick)
        self.create_timer(1/config['scene'].get('heartbeat_hz',2.0),self.publish_state)

    def command(self,message):
        if not self.machine:return
        reply=self.machine.accept(from_ros(message),time.monotonic())
        if message.recipient_id in (self.robot,'*') and not reply.accepted:
            self.get_logger().warning('command rejected: '+reply.reason,throttle_duration_sec=2)
        self.publish_state()

    def peer(self,message,source):
        if not self.machine:return
        if not self.machine.context.share_c_coordinates:
            self.machine.receive_peer(from_ros(message),time.monotonic())
            return
        if message.robot_id!=source or message.simulation!=self.simulation:return
        entry=self.config['alignment']['robots'][source]
        if message.schema_version!=1 or (message.local_map_id,message.map_version,message.common_frame_id,message.alignment_id)!=(
            entry['local_map_id'],entry['map_version'],'lab_ref',self.config['alignment']['alignment_id']):return
        if message.task_id!=self.machine.context.task_id:return
        self.machine.peer_received=time.monotonic()
        if not message.localization_valid or not message.pose_valid:self.machine.stop('peer localization invalid')

    def peer_observation(self,message,source):
        if message.robot_id==source and self.machine and message.task_id==self.machine.context.task_id and message.simulation==self.simulation:
            if message.target_id=='C' and not self.machine.context.share_c_coordinates:return
            self.last_observations[message.observation_id]=from_ros(message)
            if len(self.last_observations)>1000:self.last_observations.pop(next(iter(self.last_observations)))

    def publish_observation(self,observation):
        self.observation_pub.publish(to_ros(observation))

    def publish_state(self):
        if self.machine:
            state=self.machine.state()
        else:
            from .models import AgentState as State
            entry=self.config['alignment']['robots'][self.robot]
            state=State(robot_id=self.robot,local_map_id=entry['local_map_id'],map_version=entry['map_version'],
                alignment_id=self.config['alignment']['alignment_id'],simulation=False,run_state='PAUSED',
                reason='live configuration gates closed')
        state.stamp=self.get_clock().now().nanoseconds*1e-9
        self.state_pub.publish(to_ros(state))

    def tick(self):
        if self.runner:
            before=self.machine.state()
            self.runner.tick(time.monotonic())
            if before!=self.machine.state():self.publish_state()

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=AgentNode()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    if node.observer:executor.add_node(node.observer)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if node.machine and rclpy.ok():
            node.machine.stop('agent shutdown')
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                node.runner.tick(time.monotonic())
                node.publish_state()
                if node.machine.state().run_state=='PAUSED':break
                executor.spin_once(timeout_sec=.05)
        if node.adapter and hasattr(node.adapter,'close'):node.adapter.close()
        executor.shutdown()
        if node.observer:node.observer.destroy_node()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
