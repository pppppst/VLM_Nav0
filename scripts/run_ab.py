#!/usr/bin/env python3
"""One recorder A/B diagnostic; full-cloud monitoring is intrusive and opt-in."""
import argparse,csv,hashlib,json,math,os,pathlib,re,shutil,signal,subprocess,sys,time
import numpy as np
import rclpy,yaml
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from rclpy.serialization import deserialize_message
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState

ROOT=pathlib.Path(__file__).resolve().parent
SESSION=ROOT.parent
WORK=pathlib.Path('/home/isee-pst/unitree_ros2/VLM_Nav')
GO2=pathlib.Path('/home/isee-pst/unitree_ros2/go2_ws')
HESAI=pathlib.Path('/home/isee-pst/Documents/liang/hesai_xt16_ws')
DRIVER=HESAI/'install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node'
CONVERTER=GO2/'install/hesai_fastlio_converter/lib/hesai_fastlio_converter/hesai_fastlio_converter_node'
LIO=GO2/'install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping'
VLM=WORK/'build/go2-coop/install/vlm_nav/lib/vlm_nav'
TOPICS=['/tf','/cloud_registered_base','/lidar_points','/odometry','/cloud_registered_lidar','/tf_static','/body_imu','/parameter_events','/lidar_points_fastlio','/lowstate','/rosout']
DTYPE=np.dtype({'names':['timestamp'],'formats':['<f8'],'offsets':[18],'itemsize':26})

def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def write_json(p,x):pathlib.Path(p).write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def diagnostic_stop(previous,origin,current):
    if not np.isfinite(current).all():return 'nonfinite LIO position'
    step=float(np.linalg.norm(current-previous)) if previous is not None else 0.0
    displacement=float(np.linalg.norm(current-origin)) if origin is not None else 0.0
    if step>0.5:return f'LIO single step {step:.6f}m >0.5m'
    if displacement>1.0:return f'LIO displacement {displacement:.6f}m >1.0m (stationary)'
    return None

def system_sample():
    cpu=[int(x) for x in pathlib.Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    mem={x.split(':',1)[0]:int(x.split()[1]) for x in pathlib.Path('/proc/meminfo').read_text().splitlines() if x.startswith(('MemTotal:','MemAvailable:','SwapFree:','Dirty:','Writeback:'))}
    disks={x.split()[2]:[int(v) for v in x.split()[3:]] for x in pathlib.Path('/proc/diskstats').read_text().splitlines() if x.split()[2] in ['nvme0n1','nvme0n1p6']}
    return {'wall_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns(),'cpu_ticks':cpu,'loadavg':os.getloadavg(),'memory_kib':mem,'diskstats':disks}

def endpoint(x):
    q=x.qos_profile
    return {'node_name':x.node_name,'node_namespace':x.node_namespace,'topic_type':x.topic_type,'gid':bytes(x.endpoint_gid).hex(),'qos':{k:{'value':int(getattr(q,k)),'name':str(getattr(q,k))} for k in ['history','reliability','durability','liveliness']},'depth':q.depth,'deadline_ns':q.deadline.nanoseconds,'lifespan_ns':q.lifespan.nanoseconds,'liveliness_lease_ns':q.liveliness_lease_duration.nanoseconds}

def graph(node):
    return {'wall_ns':time.time_ns(),'nodes':node.get_node_names_and_namespaces(),'topics':{t:{'publishers':[endpoint(x) for x in node.get_publishers_info_by_topic(t)],'subscriptions':[endpoint(x) for x in node.get_subscriptions_info_by_topic(t)]} for t in TOPICS}}

class Monitor:
    def __init__(self,path,label,*,full_cloud_monitoring=False):
        self.path=path;self.node=rclpy.create_node('go2_record_ab_monitor_'+label.lower())
        self.rawlog=(path/'raw-spans.jsonl').open('x');self.poselog=(path/'lio-poses.jsonl').open('x')
        self.raw_count=0;self.converted_count=0;self.lowstate_count=0;self.last_lowstate=0;self.first_raw=None
        self.stop=None;self.origin=None;self.previous=None;self.max_step=0.;self.max_displacement=0.;self.odom_count=0
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        # Live control-PC evidence: these full-cloud subscriptions perturb publishing.
        # Prefer offline point-by-point analysis; retain small safety inputs by default.
        if full_cloud_monitoring:
            self.node.create_subscription(PointCloud2,'/lidar_points',self.raw,qos,raw=True)
            self.node.create_subscription(PointCloud2,'/lidar_points_fastlio',self.converted,qos,raw=True)
        self.node.create_subscription(Odometry,'/odometry',self.odom,qos)
        self.node.create_subscription(LowState,'/lowstate',self.lowstate,qos,raw=True)
    def lowstate(self,data):self.lowstate_count+=1;self.last_lowstate=time.monotonic()
    def converted(self,data):self.converted_count+=1
    def raw(self,data):
        received=time.time_ns();cloud=deserialize_message(data,PointCloud2)
        fields={f.name:(f.offset,f.datatype) for f in cloud.fields}
        if cloud.point_step!=26 or cloud.is_bigendian or fields.get('timestamp')!=(18,8):
            self.stop='unexpected raw timestamp layout';return
        t=np.frombuffer(cloud.data,dtype=DTYPE)['timestamp'];finite=t[np.isfinite(t)]
        lo=float(finite.min()) if len(finite) else None;hi=float(finite.max()) if len(finite) else None
        row={'index':self.raw_count,'receipt_ns':received,'header_ns':cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec,'frame_id':cloud.header.frame_id,'points':cloud.width*cloud.height,'timestamp_nonfinite':len(t)-len(finite),'timestamp_min_s':lo,'timestamp_max_s':hi,'span_s':hi-lo if lo is not None else None}
        self.rawlog.write(json.dumps(row)+'\n');self.raw_count+=1
        if self.first_raw is None:self.first_raw=row
    def odom(self,msg):
        p=msg.pose.pose.position;current=np.array([p.x,p.y,p.z],dtype=float)
        step=float(np.linalg.norm(current-self.previous)) if self.previous is not None else 0.
        disp=float(np.linalg.norm(current-self.origin)) if self.origin is not None else 0.
        reason=diagnostic_stop(self.previous,self.origin,current)
        self.poselog.write(json.dumps({'index':self.odom_count,'receipt_ns':time.time_ns(),'header_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'frame':msg.header.frame_id,'xyz':current.tolist(),'step_m':step,'displacement_m':disp,'stop_reason':reason})+'\n')
        self.odom_count+=1;self.max_step=max(self.max_step,step);self.max_displacement=max(self.max_displacement,disp)
        if self.origin is None:self.origin=current.copy()
        self.previous=current
        if reason:self.stop=reason
    def spin(self,timeout=0.02):rclpy.spin_once(self.node,timeout_sec=timeout)
    def close(self):self.rawlog.close();self.poselog.close();self.node.destroy_node()

def launch_owned(path,name,args,processes,base_env,extra=None):
    env=base_env.copy();env['ROS_LOG_DIR']=str(path/'ros')
    if extra:env.update(extra)
    log=(path/(name+'.log')).open('xb')
    p=subprocess.Popen([str(x) for x in args],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append({'name':name,'pid':p.pid,'command':[str(x) for x in args],'process':p,'log':log})
    return p

def stop_owned(processes):
    for item in reversed(processes):
        p=item['process']
        if p.poll() is None:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
    result=[]
    for item in reversed(processes):
        p=item['process'];forced=False
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:
            forced=True
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            p.wait(timeout=5)
        item['log'].close();result.append({'name':item['name'],'pid':p.pid,'returncode':p.returncode,'SIGTERM_fallback':forced})
    return result

def summary(path,manifest):
    raw=[json.loads(x) for x in (path/'raw-spans.jsonl').open()]
    spans=[x['span_s'] for x in raw if x['span_s'] is not None]
    timings=list(csv.DictReader((path/'publish.csv').open())) if (path/'publish.csv').exists() else []
    window_start=manifest.get('lio_window_start_wall_ns',0);window_end=manifest.get('window_end_wall_ns',time.time_ns())
    in_window=[x for x in timings if window_start<=int(x['wall_start_ns'])<=window_end]
    def stat(a):
        return {'count':len(a),'min':float(np.min(a)),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a))} if a else {'count':0}
    diagnostics=[]
    for line in (path/'converter.log').read_text(errors='replace').splitlines():
        if 'received=' in line:diagnostics.append({'line':line,'counters':{k:int(v) for k,v in re.findall(r'(received|published|incomplete|ring|nonfinite|timespan|out_of_order|overwritten|errors)=(\d+)',line)}})
    samples=[json.loads(x) for x in (path/'system.jsonl').open()];load=[];io=[]
    for a,b in zip(samples,samples[1:]):
        dt=(b['monotonic_ns']-a['monotonic_ns'])/1e9;delta=np.array(b['cpu_ticks'])-np.array(a['cpu_ticks']);total=int(delta.sum())
        if total>0:load.append(100*(1-(delta[3]+delta[4])/total))
        if 'nvme0n1' in a['diskstats'] and 'nvme0n1' in b['diskstats']:
            d=np.array(b['diskstats']['nvme0n1'])-np.array(a['diskstats']['nvme0n1'])
            io.append({'read_mib_s':float(d[2]*512/dt/2**20),'write_mib_s':float(d[6]*512/dt/2**20),'busy_percent':float(d[9]/(dt*1000)*100),'read_requests':int(d[0]),'write_requests':int(d[4]),'read_latency_ms':float(d[3]/d[0]) if d[0] else None,'write_latency_ms':float(d[7]/d[4]) if d[4] else None})
    result={'raw_observed':len(raw),'raw_span_s':stat(spans),'raw_over_0_15s':sum(s>0.15 for s in spans),'raw_outside_existing_span_contract':sum(s<0.05 or s>0.15 for s in spans),'publish_rcl_ms_in_lio_window':stat([int(x['duration_ns'])/1e6 for x in in_window]),'publish_total_rows':len(timings),'publish_nonzero_return_codes':sum(int(x['return_code'])!=0 for x in timings),'converted_observed':manifest['monitor']['converted_count'],'converter_last_diagnostics':diagnostics[-1] if diagnostics else None,'converter_diagnostics_limitation':'Existing5s timer; last counters may omit final<5s. Monitor BEST_EFFORT counts are observed counts, not a loss-free total.','lio_pose':{'observed':manifest['monitor']['odom_count'],'max_step_m':manifest['monitor']['max_step'],'max_displacement_m':manifest['monitor']['max_displacement']},'whole_machine_cpu_busy_percent':stat(load),'memory_available_min_mib':min(s['memory_kib']['MemAvailable'] for s in samples)/1024 if samples else None,'disk_nvme0n1':{'write_mib_s':stat([x['write_mib_s'] for x in io]),'read_mib_s':stat([x['read_mib_s'] for x in io]),'busy_percent':stat([x['busy_percent'] for x in io]),'write_latency_ms':stat([x['write_latency_ms'] for x in io if x['write_latency_ms'] is not None])},'measurement_limit':'rcl_publish includes RMW publication, excludes ToRosMsg packing; exact C++ expression time is not measured.','status':manifest['status'],'stop_reason':manifest.get('stop_reason')}
    if not manifest.get('full_cloud_monitoring',False):
        for key in ['raw_observed','raw_span_s','raw_over_0_15s','raw_outside_existing_span_contract','converted_observed']:
            result[key]=None
        result['raw_statistics_note']='NOT_MEASURED: full-cloud monitoring disabled; use unchanged converter diagnostics, prefer offline point analysis.'
    write_json(path/'summary.json',result);return result

def run_group(label,*,full_cloud_monitoring=False):
    path=ROOT/label;path.mkdir();processes=[];monitor=None;syslog=(path/'system.jsonl').open('x')
    manifest={'label':label,'full_cloud_monitoring':full_cloud_monitoring,'recorder_enabled':label=='B','planned_lio_seconds':120,'attempt':1,'environment':{k:os.getenv(k) for k in ['ROS_DOMAIN_ID','RMW_IMPLEMENTATION','CYCLONEDDS_URI','GO2_NET_IFACE','AMENT_PREFIX_PATH']},'rmw_actual':get_rmw_implementation_identifier(),'historical_recorder_qos':'未知','scope_difference':'Old safety supervisor omitted because formal preflight PASS is stale; no PASS reused or fabricated. Same omission in A and B.'}
    base_env=os.environ.copy()
    try:
        monitor=Monitor(path,label,full_cloud_monitoring=full_cloud_monitoring)
        deadline=time.monotonic()+6
        while monitor.lowstate_count<5 and time.monotonic()<deadline:monitor.spin()
        assert monitor.lowstate_count>=5 and time.monotonic()-monitor.last_lowstate<1,'live lowstate missing; no core launched'
        assert monitor.node.count_publishers('/lidar_points')==monitor.node.count_publishers('/odometry')==0,'existing driver or LIO publisher'
        write_json(path/'graph-before.json',graph(monitor.node))
        (path/'processes-before.txt').write_text(subprocess.check_output(['ps','-eo','pid,ppid,comm,args'],text=True))
        syslog.write(json.dumps(system_sample())+'\n');syslog.flush()
        if label=='B':
            cmd=['/opt/ros/humble/bin/ros2','bag','record','-o',str(path/'bag'),*TOPICS]
            recorder=launch_owned(path,'recorder',cmd,processes,base_env)
            manifest['recorder_command']=cmd;manifest['recorder_pid']=recorder.pid
            deadline=time.monotonic()+6
            while not any(n=='rosbag2_recorder' for n,ns in monitor.node.get_node_names_and_namespaces()) and time.monotonic()<deadline:
                assert recorder.poll() is None,'recorder startup failed';monitor.spin()
            assert recorder.poll() is None,'recorder exited'
        calibration=yaml.safe_load((SESSION/'go2_calibration_xt16.yaml').read_text())
        for name,key in [('base-lidar','base_lidar_extrinsic'),('base-imu','base_imu_extrinsic')]:
            tr=calibration[key];args=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher']
            for flag,value in zip(['--x','--y','--z','--roll','--pitch','--yaw'],tr['xyz']+tr['rpy']):args.extend([flag,str(value)])
            args+=['--frame-id',tr['parent_frame'],'--child-frame-id',tr['child_frame'],'--ros-args','-r','__node:='+('go2_base_to_lidar' if key=='base_lidar_extrinsic' else 'go2_base_to_body_imu')]
            launch_owned(path,name,args,processes,base_env)
        driver=launch_owned(path,'driver',[DRIVER,'--ros-args','-r','__node:=hesai_ros_driver_node','--params-file',ROOT/'driver-params.yaml'],processes,base_env,{'LD_PRELOAD':str(ROOT/'publish_clock.so'),'GO2_PUBLISH_TIMING_PATH':str(path/'publish.csv')})
        launch_owned(path,'converter',[CONVERTER,'--ros-args','-r','__node:=hesai_fastlio_converter','--params-file',ROOT/'converter-params.yaml'],processes,base_env)
        launch_owned(path,'imu',[VLM/'lowstate_imu_adapter','--ros-args','-r','__node:=lowstate_imu_adapter'],processes,base_env)
        waiter=launch_owned(path,'base-waiter',[VLM/'go2_readiness_waiter','--ros-args','-r','__node:=wait_go2_base_lidar_tf','--params-file',ROOT/'base-waiter.yaml'],processes,base_env)
        deadline=time.monotonic()+12
        while waiter.poll() is None and time.monotonic()<deadline:
            assert driver.poll() is None,'driver exited during readiness';monitor.spin()
        assert waiter.poll()==0,'base TF readiness failed'
        manifest['lio_window_start_wall_ns']=time.time_ns();manifest['lio_window_start_monotonic_ns']=time.monotonic_ns()
        lio=launch_owned(path,'lio',[LIO,'--ros-args','-r','__node:=lio_mapping','--params-file',SESSION/'spark_fast_lio_go2_xt16.yaml','-r','lidar:=/lidar_points_fastlio','-r','imu:=/body_imu'],processes,base_env)
        manifest['lio_pid']=lio.pid
        launch_owned(path,'lio-waiter',[VLM/'go2_readiness_waiter','--ros-args','-r','__node:=wait_go2_fastlio','--params-file',ROOT/'lio-waiter.yaml'],processes,base_env)
        print(label+': FRESH LIO STARTED PID '+str(lio.pid)+'; max120s; recorder='+str(label=='B'),flush=True)
        deadline=time.monotonic()+120;last_system=0;last_graph=0;graph_index=0
        while time.monotonic()<deadline:
            monitor.spin();now=time.monotonic()
            if monitor.stop:
                manifest['stop_reason']=monitor.stop;manifest['status']='early_stop_divergence' if monitor.stop.startswith(('LIO ','nonfinite')) else 'early_stop_input_error';break
            for item in processes:
                if item['name'] in ['driver','converter','imu','lio','recorder'] and item['process'].poll() is not None:
                    raise RuntimeError(item['name']+' exited unexpectedly')
            if now-last_system>=1:
                syslog.write(json.dumps(system_sample())+'\n');syslog.flush();last_system=now
            if now-last_graph>=5:
                write_json(path/f'graph-{graph_index:03d}.json',graph(monitor.node));graph_index+=1;last_graph=now
                (path/'processes-running.txt').write_text(subprocess.check_output(['ps','-eo','pid,ppid,comm,args'],text=True))
                manifest['driver_maps_rmw_and_timing']=[x for x in pathlib.Path('/proc/'+str(driver.pid)+'/maps').read_text().splitlines() if 'librmw' in x or 'publish_clock' in x]
            if graph_index and graph_index%4==0 and now-last_graph<0.03:
                print(label+': raw='+str(monitor.raw_count)+' odom='+str(monitor.odom_count)+' max_step='+str(round(monitor.max_step,4)),flush=True)
        else:manifest['status']='completed_120s';manifest['stop_reason']=None
        manifest['window_end_wall_ns']=time.time_ns();manifest['window_elapsed_s']=(time.monotonic_ns()-manifest['lio_window_start_monotonic_ns'])/1e9
        manifest['monitor']={k:getattr(monitor,k) for k in ['raw_count','converted_count','lowstate_count','odom_count','max_step','max_displacement','first_raw']}
        write_json(path/'graph-final.json',graph(monitor.node))
        print(label+': STOP '+str(manifest.get('stop_reason') or '120s complete'),flush=True)
    except BaseException as e:
        manifest['status']='failed';manifest['error']=repr(e);raise
    finally:
        manifest['process_commands']=[{k:i[k] for k in ['name','pid','command']} for i in processes]
        manifest['process_exit']=stop_owned(processes)
        if monitor is not None:
            manifest['monitor']={k:getattr(monitor,k) for k in ['raw_count','converted_count','lowstate_count','odom_count','max_step','max_displacement','first_raw']};monitor.close()
        syslog.close()
        timeseries=pathlib.Path('/tmp/spark_fast_lio_timeseries.jsonl')
        if 'lio_pid' in manifest and timeseries.exists():(path/'lio-timeseries.jsonl').write_bytes(timeseries.read_bytes())
        write_json(path/'manifest.json',manifest)
    result=summary(path,manifest)
    assert result['publish_total_rows']>0,'no measured rcl_publish rows; do not add another run'
    if label=='B':
        graphs=sorted(path.glob('graph-*.json'));actual={}
        for p in graphs:
            g=json.loads(p.read_text());subs=g['topics']['/lidar_points']['subscriptions'];rec=[s for s in subs if s['node_name']=='rosbag2_recorder']
            if rec:actual={'graph_file':str(p),'raw_recorder_endpoints':rec,'rmw_actual':manifest['rmw_actual'],'recorder_command':manifest['recorder_command'],'recorder_pid':manifest['recorder_pid']};break
        write_json(path/'recorder-qos.json',actual if actual else {'actual_endpoint_qos':'未知','reason':'No recorder endpoint recorded; no extra run'})
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir',type=pathlib.Path,help='Existing session with calibration, LIO params and original build manifest')
    parser.add_argument('--reference-dir',type=pathlib.Path,help='Existing diagnostic params and publish_clock.so; unchanged inputs')
    parser.add_argument('--output-dir',type=pathlib.Path,help='New evidence directory; never overwrite a prior run')
    parser.add_argument('--full-cloud-monitoring',action='store_true',help='INTRUSIVE: explicitly receive both complete clouds; known to perturb live publishing/localization')
    parser.add_argument('--self-check',action='store_true',help='Offline guard check; no ROS initialization or processes')
    args=parser.parse_args()
    if args.self_check:
        assert diagnostic_stop(None,None,np.array([0.,0.,0.])) is None
        assert diagnostic_stop(np.zeros(3),np.zeros(3),np.array([0.6,0.,0.])).startswith('LIO single step')
        assert diagnostic_stop(np.array([.9,0.,0.]),np.zeros(3),np.array([1.1,0.,0.])).startswith('LIO displacement')
        assert diagnostic_stop(np.zeros(3),np.zeros(3),np.array([float('nan'),0.,0.]))=='nonfinite LIO position'
        assert sum(system_sample()['cpu_ticks'])>0
        print('lightweight monitor stop-policy and system parser self-check passed; no ROS init');raise SystemExit(0)
    if not all([args.session_dir,args.reference_dir,args.output_dir]):
        parser.error('--session-dir, --reference-dir and --output-dir required for a hardware run')
    SESSION=args.session_dir.resolve();ROOT=args.output_dir.resolve()
    if ROOT.exists():parser.error('output directory already exists; no rerun or overwrite')
    reference=args.reference_dir.resolve()
    files=['driver-params.yaml','converter-params.yaml','base-waiter.yaml','lio-waiter.yaml','publish_clock.so']
    if not all((reference/name).is_file() for name in files):parser.error('reference files missing')
    ROOT.mkdir(parents=True)
    for name in files:shutil.copy2(reference/name,ROOT/name)
    write_json(ROOT/'entry.json',{'entry':str(pathlib.Path(__file__).resolve()),'reference':str(reference),'full_cloud_monitoring':args.full_cloud_monitoring,'safety':'diagnostic Odometry/LowState guard; not formal SYSTEM_READY or live acceptance'})

    assert os.environ.get('ROS_DOMAIN_ID')=='0' and os.environ.get('GO2_NET_IFACE')=='enp4s0'
    assert not os.getenv('LD_PRELOAD'),'unexpected existing preload'
    old_build=json.loads((SESSION/'patch-validation.wfw0zqo0/build-manifest.json').read_text())
    component=GO2/'install/spark_fast_lio/lib/libspark_lio_component.so'
    assert sha(LIO)==old_build['live_executable_sha256'] and sha(component)==old_build['live_component_sha256'],'LIO binary changed; do not test a patched version'
    header=GO2/'src/spark-fast-lio/spark_fast_lio/include/imu_processing.hpp'
    assert sha(header)==old_build['source_header_sha256'],'LIO source changed'
    ps=subprocess.check_output(['ps','-eo','args'],text=True)
    names=['hesai_ros_driver_node','hesai_fastlio_converter_node','spark_lio_mapping','lowstate_imu_adapter','nav2_costmap_2d','domain_bridge']
    assert not any(line.split() and pathlib.Path(line.split()[0]).name in names for line in ps.splitlines()),'existing live stack; not stopping unrelated processes'
    original_tmp=pathlib.Path('/tmp/spark_fast_lio_timeseries.jsonl')
    if original_tmp.exists():(ROOT/'preserved-preexisting-timeseries.jsonl').write_bytes(original_tmp.read_bytes())
    original_files=[HESAI/'src/HesaiLidar_ROS_2.0/config/config.yaml',SESSION/'spark_fast_lio_go2_xt16.yaml',header,LIO,component,CONVERTER]
    before={str(p):sha(p) for p in original_files}
    write_json(ROOT/'experiment.json',{'status':'running','ordered_groups':['A','B'],'maximum_seconds_each':120,'early_stop_thresholds':{'single_step_m':0.5,'displacement_m':1.0},'threshold_purpose':'diagnostic stationary stop, not acceptance flags','files_before':before,'historical_qos_unknown_preserved':True})
    rclpy.init()
    try:
        results={}
        for label in ['A','B']:results[label]=run_group(label,full_cloud_monitoring=args.full_cloud_monitoring)
        write_json(ROOT/'results.json',results)
    finally:
        if rclpy.ok():rclpy.shutdown()
        after={str(p):sha(p) for p in original_files};write_json(ROOT/'files-after.json',{'files_after':after,'all_originals_unchanged':before==after})
    print('EXACTLY ONE A/B FINISHED; NO RETRIES; '+str(ROOT),flush=True)
