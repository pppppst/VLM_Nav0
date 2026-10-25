"""Local RViz display of accepted exploration events; no navigation commands."""
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .go2_exploration import generated_exploration_markers
from .models import Observation, Pose2D


class ExplorationDisplay:
    def __init__(self, node, frame):
        self.node,self.frame=node,frame
        self.task_id=None
        self.display={}
        self.targets={}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.publishers={topic:node.create_publisher(Path if topic=='planned_path' else MarkerArray,
            '/vlm_nav/go2_exploration/'+topic,qos)
            for topic in ('generated_candidates','selected_candidate','targets','planned_path')}
        self.record(dict(state='DISPLAY_RESET'))

    def publish(self, topic, message):
        message=deepcopy(message)
        stamp=self.node.get_clock().now().to_msg()
        for item in message.markers if isinstance(message,MarkerArray) else [message,*message.poses]:
            item.header.frame_id='map';item.header.stamp=stamp
        try:self.publishers[topic].publish(message)
        except Exception as error:
            # Display transport must not interrupt navigation cancellation or zero velocity.
            self.node.get_logger().warning('RViz publish failed: '+str(error),throttle_duration_sec=5.)

    def record(self, event):
        state=event.get('state')
        if state=='DISPLAY_RESET':
            if event.get('task_id')!=self.task_id or self.task_id is None:
                self.display.clear();self.targets.clear()
                self.task_id=event.get('task_id')
                clear=Marker();clear.action=Marker.DELETEALL
                self.publish('targets',MarkerArray(markers=[clear]))
            self.display.update(points=[],records={},selectable={},selected=None)
            self.publish('planned_path',Path())
        elif state in ('STOPPING','STOP_UNCONFIRMED') or event.get('event')=='SESSION_END':
            self.display.update(selectable={},selected=None)
            self.publish('planned_path',Path())
        elif state=='CANDIDATE_GEOMETRY':
            self.display.update(points=event['candidates'],records={},selectable={},selected=None)
        elif state=='CANDIDATE_REQUEST':
            self.display.update(points=event['candidates'],selectable={p['id']:p['display_id'] for p in event['candidates']},
                                records=event['records'],selected=None)
            self.publish('planned_path',Path())
        elif state=='CANDIDATE_SELECTED':self.display['selected']=event['viewpoint_id']
        elif state=='OBSERVATION_RECORD':self.display['records']=event['records']
        elif state=='PATH_PLANNED':
            path=Path()
            for point in event['path']:
                pose=PoseStamped();pose.pose.position.x=point['x'];pose.pose.position.y=point['y']
                pose.pose.orientation.w=1.;path.poses.append(pose)
            self.publish('planned_path',path)
            return
        elif state in ('INITIAL_CAMERA_RESULT','SCAN_RESULT'):
            for value in event['observations']:
                observation=Observation(**dict(value,common_position=Pose2D(**value['common_position'])))
                if observation.navigable:
                    point=self.frame.to_local(observation.common_position)
                    if point.valid:self.targets[observation.target_id]=point
            markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
            for index,(target,point) in enumerate(sorted(self.targets.items())):
                for offset,kind in enumerate((Marker.SPHERE,Marker.TEXT_VIEW_FACING)):
                    marker=Marker();marker.ns='detected_targets';marker.id=index*2+offset
                    marker.type=kind;marker.action=Marker.ADD
                    marker.pose.position.x=point.x;marker.pose.position.y=point.y
                    marker.pose.position.z=.12 if offset==0 else .28;marker.pose.orientation.w=1.
                    marker.scale.x=marker.scale.y=marker.scale.z=.18 if offset==0 else .2
                    marker.color.r=.15;marker.color.g=1.;marker.color.b=.25;marker.color.a=1.
                    marker.text=target;markers.markers.append(marker)
            self.publish('targets',markers)
            return
        else:return
        for topic,markers in zip(('generated_candidates','selected_candidate'),generated_exploration_markers(self.display)):
            self.publish(topic,markers)
