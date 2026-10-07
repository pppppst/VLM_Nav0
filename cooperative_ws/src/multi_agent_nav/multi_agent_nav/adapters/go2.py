"""Thin local adapter: no coordinator logic, no hardware startup or auto ARM."""
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import time
import uuid
import os
import numpy as np
from vlm_nav.vlm_client import OpenAICompatibleVLMClient, validate_vlm_result, build_prompt
from vlm_nav.geometry import normalize_angle
from ..models import *

def make_observer(config,read_only=False,node_name=None):
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    base=yaml.safe_load((Path(get_package_share_directory('vlm_nav'))/'config/robot_go2.yaml').read_text())['vlm_nav']['ros__parameters']
    base.update(enabled=False,camera_extrinsic_calibrated=config.get('camera_calibrated',False),
        publish_stop_command=not read_only,
        behavior_costmap_topic=config.get('costmap_topic','/local_costmap/costmap_raw'),
        arrival_odom_topic=config.get('odom_topic','/odometry'))
    return VLMNavigator(observation_only=True,node_name=node_name,
        parameter_overrides=[Parameter(k,value=v) for k,v in base.items()])

class ABCClient(OpenAICompatibleVLMClient):
    def __init__(self, config):
        # Explicit custom environment name is supported without logging key contents.
        from openai import OpenAI
        import httpx
        api_key = os.getenv(config.get('api_key_env','DASHSCOPE_API_KEY')) or os.getenv('OPENAI_API_KEY')
        endpoint = config.get('endpoint') or os.getenv('DASHSCOPE_BASE_URL') or os.getenv('OPENAI_BASE_URL')
        if not endpoint and os.getenv('DASHSCOPE_WORKSPACE_ID'):
            endpoint=f"https://{os.getenv('DASHSCOPE_WORKSPACE_ID')}.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
        model=config.get('model') or os.getenv('DASHSCOPE_MODEL') or os.getenv('OPENAI_MODEL')
        if not api_key or not endpoint or not model:
            raise ValueError('vision key/endpoint/model configuration missing')
        self.model=model
        self._is_qwen='qwen' in model.lower() or 'aliyuncs' in endpoint
        self._structured_output_supported=not self._is_qwen
        self.image_detail='high'
        self.jpeg_quality=85
        self.last_raw_response=None
        self.client=OpenAI(api_key=api_key,base_url=endpoint,timeout=config['api_timeout'],max_retries=0,
                           http_client=httpx.Client(timeout=config['api_timeout'],trust_env=False))
        self.mode=config.get('vision_mode','abc')

    @staticmethod
    def parse(payload,width,height,coordinate_mode='pixels'):
        if not isinstance(payload,dict) or set(payload)!=set(TARGETS):
            raise ValueError('ABC results required')
        output={}
        for target,item in payload.items():
            if not isinstance(item,dict) or set(item)!= {'status','grounding'}:
                raise ValueError('invalid target structure')
            status=item['status']
            if status not in ('DETECTED','NOT_DETECTED','UNCERTAIN'):
                raise ValueError('invalid recognition status')
            result=validate_vlm_result(item['grounding'],width,height,coordinate_mode)
            if (status=='DETECTED')!=result.target_visible:
                raise ValueError('status/grounding mismatch')
            output[target]=(status,result)
        return output

    def infer_abc(self,rgb,descriptions):
        if self.mode=='individual':
            return {t:(('DETECTED' if (r:=self.infer(rgb,descriptions[t])).target_visible else 'NOT_DETECTED'),r) for t in TARGETS}
        height,width=rgb.shape[:2]
        prompt='Check all three targets on this SAME image. Return only an object with exactly A, B, C keys. Each value has status (DETECTED/NOT_DETECTED/UNCERTAIN) and grounding (the documented fields below). Uncertain uses target_visible=false.\n'
        for target in TARGETS:
            prompt+=f'{target}: '+build_prompt(descriptions[target],width,height,normalized_1000=self._is_qwen)+'\n'
        response=self._completion([{'role':'user','content':[{'type':'text','text':prompt},self._image_content(rgb)]}],1200)
        content=response.choices[0].message.content
        if not content: raise ValueError('empty ABC result')
        return self.parse(json.loads(content),width,height,'normalized_1000' if self._is_qwen else 'pixels')

def observations_from_results(results,snapshot,project,frame,scan_id,view_index):
    output=[]
    for target in TARGETS:
        status,result=results.get(target,('API_ERROR',None))
        position,reason=None,''
        if status=='DETECTED' and result and result.target_pixel is not None:
            point,reason=project(snapshot,result.target_pixel,False)
            if point is not None and all(math.isfinite(v) for v in point):
                position=frame.to_common(Pose2D(float(point[0]),float(point[1])))
        stamp=snapshot.stamp.sec+snapshot.stamp.nanosec*1e-9
        output.append(Observation(target_id=target,observation_id=f'{scan_id}/{view_index}/{target}',
            scan_id=scan_id,view_index=view_index,status=status,confidence=result.confidence if result else 0.0,
            common_position=position or Pose2D(),position_valid=position is not None,image_stamp=stamp,reason=reason))
    return output

class AsyncVision:
    """One in-flight job, deadline observed by the ROS timer; late work never queues more."""
    def __init__(self,timeout):
        self.timeout=timeout
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='cooperative-vision')
        self.job=self.reply=None
        self.deadline=0

    @property
    def available(self):
        return self.job is None

    def submit(self,function):
        if not self.available: raise RuntimeError('vision busy')
        self.reply=Future()
        self.job=self.executor.submit(function)
        self.deadline=time.monotonic()+self.timeout
        return self.reply

    def poll(self):
        if self.job is None: return
        if not self.reply.done():
            if time.monotonic()>self.deadline:
                self.reply.set_exception(TimeoutError('vision timeout'))
            elif self.job.done():
                try: self.reply.set_result(self.job.result())
                except Exception:
                    # Do not leak provider exceptions (which can contain credentials) into logs.
                    self.reply.set_exception(RuntimeError('vision request/validation failed'))
        if self.job.done(): self.job=None

    def cancel(self):
        if self.reply and not self.reply.done(): self.reply.cancel()

    def close(self):
        self.cancel()
        self.executor.shutdown(wait=False,cancel_futures=True)

class Go2Adapter:
    simulation=False
    def __init__(self,observer,config,frame,read_only=False):
        from nav_msgs.msg import Odometry,Path
        from nav2_msgs.msg import Costmap
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
        self.node,self.config,self.frame=observer,config,frame
        self.read_only=read_only
        self.results=deque()
        self.motions={}
        self.last_action_uuid=''
        self.plans={}
        self.path=[]
        self.path_at=0.0
        self.odom_at=0.0
        self.stationary_since=None
        self.costmap=None
        self.costmap_at=0.0
        self.motion_fault=False
        self.scan=None
        self.client=None
        self.vision=AsyncVision(config['api_timeout'])
        self.node.create_subscription(Odometry,config.get('odom_topic','/odometry'),self.on_odometry,qos_profile_sensor_data)
        self.node.create_subscription(Costmap,config.get('costmap_topic','/local_costmap/costmap_raw'),
            self.on_costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if config.get('path_topic'):
            self.node.create_subscription(Path,config['path_topic'],self.on_path_update,10)

    def on_odometry(self,message):
        now=time.monotonic()
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        ros_now=self.node.get_clock().now().nanoseconds*1e-9
        if abs(ros_now-stamp)>self.config['odom_max_age']:
            self.stationary_since=None
            self.odom_at=0.0
            return
        if self.odom_at and now-self.odom_at>self.config['odom_max_age']:
            self.stationary_since=None
        self.odom_at=now
        v=message.twist.twist
        linear=math.sqrt(sum(getattr(v.linear,k,0.)**2 for k in ('x','y','z')))
        angular=math.sqrt(sum(getattr(v.angular,k,0.)**2 for k in ('x','y','z')))
        if linear<=self.config.get('stationary_linear_speed',self.config.get('stationary_speed',.02)) and angular<=self.config.get('stationary_angular_speed',self.config.get('stationary_speed',.02)):
            if self.stationary_since is None:self.stationary_since=now
        else:self.stationary_since=None

    def on_costmap(self,message):
        self.costmap=None
        meta=message.metadata
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        if not message.header.frame_id or not math.isfinite(meta.resolution) or meta.resolution<=0 or meta.size_x*meta.size_y<=0 or len(message.data)!=meta.size_x*meta.size_y:
            return
        if abs(now-stamp)>self.config.get('costmap_max_age',1.):return
        q=meta.origin.orientation
        if not all(math.isfinite(v) for v in (meta.origin.position.x,meta.origin.position.y,q.x,q.y,q.z,q.w)) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.)>.01:
            return
        self.costmap=message
        self.costmap_at=time.monotonic()

    def costmap_transform(self):
        """Map pose → costmap frame, at the payload stamp; no odom-as-map shortcut."""
        from rclpy.time import Time
        from vlm_nav.vlm_navigator import quaternion_yaw
        message=self.costmap
        if message is None or time.monotonic()-self.costmap_at>self.config.get('costmap_max_age',1.):return None
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if abs(self.node.get_clock().now().nanoseconds*1e-9-stamp)>self.config.get('costmap_max_age',1.):return None
        try:
            if message.header.frame_id=='map':return Pose2D()
            tf=self.node.tf_buffer.lookup_transform(message.header.frame_id,'map',Time.from_msg(message.header.stamp)).transform
            pose=Pose2D(tf.translation.x,tf.translation.y,quaternion_yaw(tf.rotation))
            return pose if pose.valid else None
        except Exception:return None

    def costmap_candidate_free(self,candidate):
        from vlm_nav.vlm_navigator import quaternion_yaw
        transform=self.costmap_transform()
        if transform is None:return False
        c,s=math.cos(transform.yaw),math.sin(transform.yaw)
        x=c*candidate.x-s*candidate.y+transform.x
        y=s*candidate.x+c*candidate.y+transform.y
        meta=self.costmap.metadata
        yaw=quaternion_yaw(meta.origin.orientation)
        c,s=math.cos(yaw),math.sin(yaw)
        dx,dy=x-meta.origin.position.x,y-meta.origin.position.y
        x,y=c*dx+s*dy,-s*dx+c*dy
        local_yaw=normalize_angle(candidate.yaw+transform.yaw-yaw)
        footprint=self.config['footprint']
        # Conservative centered rectangle also covers asymmetric footprints.
        length,width=2*max(abs(p[0]) for p in footprint),2*max(abs(p[1]) for p in footprint)
        c,s=math.cos(local_yaw),math.sin(local_yaw)
        if any(not (0<=x+c*px-s*py<meta.size_x*meta.resolution and 0<=y+s*px+c*py<meta.size_y*meta.resolution)
            for px in (-length/2,length/2) for py in (-width/2,width/2)):return False
        raw=np.asarray(self.costmap.data,dtype=np.int16).reshape(meta.size_y,meta.size_x)
        # Nav2 253/254 are inscribed/lethal and 255 unknown. Unknown body cells veto too.
        # Test the actual pose, including any cell intersecting the body rectangle;
        # do not snap it to a cell center. Projection of a half-cell is conservative.
        gy,gx=np.indices(raw.shape)
        dx,dy=(gx+.5)*meta.resolution-x,(gy+.5)*meta.resolution-y
        half_cell=meta.resolution*(abs(c)+abs(s))/2
        body=(np.abs(c*dx+s*dy)<=length/2+half_cell) & (np.abs(-s*dx+c*dy)<=width/2+half_cell)
        return bool(body.any()) and not np.any(body & (raw>=self.config.get('costmap_blocked_cost',253)))

    def on_path_update(self,message):
        from vlm_nav.vlm_navigator import quaternion_yaw
        if message.header.frame_id!='map':
            self.path=[]
            return
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        ros_now=self.node.get_clock().now().nanoseconds*1e-9
        if abs(ros_now-stamp)>self.config.get('path_max_age',1.0) or stamp<getattr(self,'path_after_stamp',0.):
            self.path=[]
            return
        self.path=[Pose2D(p.pose.position.x,p.pose.position.y,quaternion_yaw(p.pose.orientation)) for p in message.poses]
        if not all(p.valid for p in self.path):self.path=[]
        self.path_at=time.monotonic()

    def get_health(self):
        from rclpy.time import Time
        from vlm_nav.vlm_navigator import quaternion_yaw
        now=time.monotonic()
        pose=Pose2D()
        valid=False
        try:
            tf=self.node.tf_buffer.lookup_transform('map','base_link',Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            valid=abs(self.node.get_clock().now().nanoseconds*1e-9-stamp)<self.config.get('pose_max_age',1.0)
            pose=Pose2D(tf.transform.translation.x,tf.transform.translation.y,quaternion_yaw(tf.transform.rotation))
        except Exception:
            valid=False
        stopped=now-self.odom_at<=self.config['odom_max_age'] and self.stationary_since is not None and now-self.stationary_since>=self.config['stationary_duration']
        gates=self.node.nav_ready and self.node.control_armed and all(now-v<=self.node.p.external_gate_timeout for v in self.node.external_gate_receipts.values())
        ready=gates and self.node.navigator.server_is_ready() and self.node.path_planner.server_is_ready() and self.node.spinner.server_is_ready() and not self.motion_fault
        ready=ready and self.node.depth_units_verified and self.node.vlm_input_ready(now)
        ready=ready and self.costmap_transform() is not None
        path_valid=bool(self.path) and now-self.path_at<=self.config.get('path_max_age',1.0) and self.config.get('path_source_verified',False)
        return Health(pose,valid,ready,stopped,list(self.path),path_valid,self.config['footprint'],not self.motions,
                      'path source/health pending' if not path_valid else '')

    def compute_path(self,local_goal):
        from nav2_msgs.action import ComputePathToPose
        future=Future()
        if not self.node.path_planner.server_is_ready():
            future.set_result(PathResult(False,reason='planner unavailable'))
            return future
        goal=ComputePathToPose.Goal()
        goal.goal=self.node.make_pose_stamped(local_goal.x,local_goal.y,local_goal.yaw)
        request=self.node.path_planner.send_goal_async(goal)
        self.plans[id(future)] = None
        def response(done):
            try:
                handle=done.result()
                if not handle.accepted: raise ValueError('path rejected')
                self.plans[id(future)]=handle
                if future.cancelled():handle.cancel_goal_async()
                def result(completed):
                    self.plans.pop(id(future),None)
                    if future.done():return
                    try:
                        value=completed.result()
                        if value.status!=4 or not value.result.path.poses:raise ValueError('no local path')
                        from vlm_nav.vlm_navigator import quaternion_yaw
                        path=[Pose2D(p.pose.position.x,p.pose.position.y,quaternion_yaw(p.pose.orientation)) for p in value.result.path.poses]
                        future.set_result(PathResult(True,path))
                    except Exception:future.set_result(PathResult(False,reason='local path failed'))
                handle.get_result_async().add_done_callback(result)
            except Exception:
                self.plans.pop(id(future),None)
                if not future.done():future.set_result(PathResult(False,reason='path request failed'))
        request.add_done_callback(response)
        return future

    def select_approach(self,local_object,excluded_candidates):
        output=Future()
        if self.costmap_transform() is None:
            output.set_result(ApproachResult(reason='local map/costmap unavailable'))
            return output
        candidates=[]
        distance=self.config['standoff_distance']
        for angle in np.linspace(-math.pi,math.pi,8,endpoint=False):
            candidate=Pose2D(local_object.x+distance*math.cos(angle),local_object.y+distance*math.sin(angle),normalize_angle(angle+math.pi))
            if candidate in excluded_candidates:continue
            if self.costmap_candidate_free(candidate):candidates.append(candidate)
        health=self.get_health()
        candidates.sort(key=lambda p:math.hypot(p.x-health.pose.x,p.y-health.pose.y))
        def attempt():
            if output.done():return
            if not candidates:
                output.set_result(ApproachResult(reason='no locally reachable candidate'))
                return
            candidate=candidates.pop(0)
            if not self.costmap_candidate_free(candidate):attempt();return
            future=self.compute_path(candidate)
            def done(f):
                if output.done():return
                if f.result().reachable and self.costmap_candidate_free(candidate):output.set_result(ApproachResult(candidate))
                else:attempt()
            future.add_done_callback(done)
        attempt()
        return output

    def _motion(self,client,goal,task_id,command_id,kind):
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot send motion goals')
        goal_id=uuid.uuid4().hex
        motion={'handle':None,'cancel':False,'reply':Future(),'task':task_id,'command':command_id,'kind':kind,'since':time.monotonic()}
        self.motions[goal_id]=motion
        def finish(status):
            self.motions.pop(goal_id,None)
            if kind!='SCAN_SPIN':self.results.append(GoalResult(task_id,command_id,goal_id,kind,status))
            if not motion['reply'].done():motion['reply'].set_result(status)
        def accepted(done):
            try:
                handle=done.result()
                if not handle.accepted:finish('ABORTED');return
                motion['handle']=handle
                self.last_action_uuid=bytes(handle.goal_id.uuid).hex()
                if motion['cancel']:handle.cancel_goal_async()
                def completed(done):
                    try:finish({4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(done.result().status,'ABORTED'))
                    except Exception:self.motion_fault=True;finish('ABORTED')
                handle.get_result_async().add_done_callback(completed)
            except Exception:
                self.motion_fault=True
                finish('ABORTED')
        try:client.send_goal_async(goal).add_done_callback(accepted)
        except Exception:self.motion_fault=True;finish('ABORTED')
        return goal_id,motion['reply']

    def start_navigation(self,local_goal,task_id,command_id,goal_kind):
        from nav2_msgs.action import NavigateToPose
        if self.motions or self.scan:raise RuntimeError('motion not serial')
        goal=NavigateToPose.Goal()
        goal.pose=self.node.make_pose_stamped(local_goal.x,local_goal.y,local_goal.yaw)
        self.path=[]  # a preflight path is NOT an executing/replanned path
        self.path_after_stamp=self.node.get_clock().now().nanoseconds*1e-9
        return self._motion(self.node.navigator,goal,task_id,command_id,goal_kind)[0]

    def cancel_navigation(self,goal_id):
        future=Future()
        motion=self.motions.get(goal_id)
        if not motion:future.set_result(True);return future
        motion['cancel']=True
        motion['since']=time.monotonic()
        if motion['handle']:
            try:return motion['handle'].cancel_goal_async()
            except Exception:self.motion_fault=True
        future.set_result(False)  # acceptance pending; health still motion_terminal=false
        return future

    def stop_motion(self,reason):
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot publish stopping commands')
        from geometry_msgs.msg import Twist
        self.vision.cancel()
        if self.scan:
            self.scan['reply'].cancel()
            self.scan=None
        for goal_id in list(self.motions):self.cancel_navigation(goal_id)
        for handle in self.plans.values():
            if handle:handle.cancel_goal_async()
        self.node.stop_pub.publish(Twist())
        future=Future();future.set_result(True)
        return future

    def start_scan(self,target_descriptions,scan_id):
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot start Spin/scan')
        if self.scan or self.motions:raise RuntimeError('motion not serial')
        reply=Future()
        self.scan={'reply':reply,'id':scan_id,'targets':target_descriptions,'index':0,'phase':'SPIN',
                   'initial':self.get_health().pose.yaw,'pending':None,'observations':[],'after':0,'deadline':time.monotonic()+self.config.get('scan_timeout',120.0)}
        return reply

    def tick(self):
        from nav2_msgs.action import Spin
        from rclpy.duration import Duration
        self.vision.poll()
        now=time.monotonic()
        for motion in self.motions.values():
            if motion['cancel'] and now-motion['since']>self.config.get('cancel_timeout',5.0):
                self.motion_fault=True  # keep handle occupied; no replacement movement
        scan=self.scan
        if not scan:return
        if not self.get_health().nav_ready or not self.get_health().localization_valid:
            self.stop_motion('scan health lost')
            return
        if now>scan['deadline']:
            scan['reply'].set_exception(TimeoutError('scan timeout'))
            self.stop_motion('scan timeout')
            return
        phase=scan['phase']
        if phase=='SPIN':
            if not self.get_health().stopped:return
            headings=self.config['scan_angles']
            if scan['index']>=len(headings):
                scan['reply'].set_result(scan['observations']);self.scan=None;return
            yaw=self.get_health().pose.yaw
            relative=normalize_angle(scan['initial']+headings[scan['index']]-yaw)
            if abs(relative)<.01:
                scan['phase']='FRAME';scan['after']=now
            else:
                goal=Spin.Goal();goal.target_yaw=relative;goal.time_allowance=Duration(seconds=30.0).to_msg()
                _,scan['pending']=self._motion(self.node.spinner,goal,'',scan['id'],'SCAN_SPIN')
                scan['phase']='SPIN_WAIT'
        elif phase=='SPIN_WAIT' and scan['pending'].done() and self.get_health().stopped:
            if scan['pending'].result()!='SUCCEEDED':
                scan['reply'].set_exception(RuntimeError('scan spin failed'));self.scan=None;return
            scan['phase']='FRAME';scan['after']=now
        elif phase=='FRAME':
            snapshot=self.node.latest_snapshot
            if not self.node.vlm_input_ready(now) or snapshot.captured_monotonic<=scan['after'] or not self.vision.available:return
            def infer():
                if self.client is None:self.client=ABCClient(self.config)
                return self.client.infer_abc(snapshot.rgb,scan['targets'])
            scan['snapshot']=snapshot
            scan['pending']=self.vision.submit(infer)
            scan['phase']='INFER'
        elif phase=='INFER' and scan['pending'].done():
            try:results=scan['pending'].result()
            except Exception:results={}
            scan['observations']+=observations_from_results(results,scan['snapshot'],self.node.ground_pixel_with_reason,
                                                          self.frame,scan['id'],scan['index'])
            scan['index']+=1;scan['phase']='SPIN'

    def close(self,stop=True):
        if stop and not getattr(self,'read_only',False):self.stop_motion('shutdown')
        self.vision.close()
