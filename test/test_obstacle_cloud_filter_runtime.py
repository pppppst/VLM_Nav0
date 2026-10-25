import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def test_filter_stays_alive_while_waiting_for_cloud_transform(tmp_path, monkeypatch):
    executable = (
        Path(get_package_prefix("vlm_nav"))
        / "lib"
        / "vlm_nav"
        / "obstacle_cloud_filter"
    )
    topic = f"/vlm_nav/test/filter_input_{os.getpid()}"
    ros_log_dir = tmp_path / "ros_logs"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    environment = os.environ.copy()
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-r",
            "__node:=obstacle_cloud_filter_runtime_test",
            "-p",
            f"input_topic:={topic}",
            "-p",
            "output_topic:=/vlm_nav/test/filter_output",
            "-p",
            "target_frame:=base_link",
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    publisher_node = rclpy.create_node("obstacle_cloud_filter_runtime_test_publisher")
    publisher = publisher_node.create_publisher(
        PointCloud2, topic, qos_profile_sensor_data
    )
    try:
        discovery_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            if process.poll() is not None or time.monotonic() >= discovery_deadline:
                output = process.communicate(timeout=1.0)[0]
                raise AssertionError(
                    "filter subscriber was not discovered before process exit/timeout:\n"
                    + output
                )
            rclpy.spin_once(publisher_node, timeout_sec=0.05)

        cloud = PointCloud2()
        cloud.header.frame_id = "body"
        cloud.header.stamp = publisher_node.get_clock().now().to_msg()
        publisher.publish(cloud)

        survival_deadline = time.monotonic() + 1.0
        while process.poll() is None and time.monotonic() < survival_deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.05)

        if process.poll() is not None:
            output = process.communicate(timeout=1.0)[0]
            raise AssertionError(
                f"filter exited after receiving a cloud (code {process.returncode}):\n"
                + output
            )
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5.0)


@pytest.mark.parametrize("delayed_tf", [False, True], ids=["static_tf", "delayed_dynamic_tf"])
def test_temporal_confirmation_uses_map_cells_and_keeps_clearing_live(tmp_path, monkeypatch, delayed_tf):
    """Domain31 only: actual filter, map/TF, and timestamped synthetic clouds."""
    import struct
    from nav_msgs.msg import OccupancyGrid
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from rclpy.qos import QoSProfile, DurabilityPolicy

    for key, value in {'ROS_DOMAIN_ID':'31','ROS_LOCALHOST_ONLY':'1',
                       'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_LOG_DIR':str(tmp_path)}.items():
        monkeypatch.setenv(key,value)
    monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    executable=Path(get_package_prefix('vlm_nav'))/'lib/vlm_nav/obstacle_cloud_filter'
    topic=f'/temporal_test_{os.getpid()}'
    log=(tmp_path/'temporal_filter.log').open('w')
    process=subprocess.Popen([str(executable),'--ros-args','-r','__node:=temporal_filter_test',
        '-p',f'input_topic:={topic}/in','-p',f'output_topic:={topic}/out',
        '-p','temporal_confirmation_enabled:=true','-p','temporal_confirmation_period:=0.8',
        '-p','temporal_confirmation_cycles:=3'],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('temporal_filter_probe')
    durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps=node.create_publisher(OccupancyGrid,'/map',durable)
    transforms=node.create_publisher(TFMessage,'/tf' if delayed_tf else '/tf_static',durable)
    publisher=node.create_publisher(PointCloud2,topic+'/in',qos_profile_sensor_data)
    raw={};confirmed={};diagnostics=[]
    node.create_subscription(DiagnosticArray,'/diagnostics',lambda m:diagnostics.extend(m.status),10)
    key=lambda c: c.header.stamp.sec*1000000000+c.header.stamp.nanosec
    node.create_subscription(PointCloud2,topic+'/out',lambda c:raw.setdefault(key(c),c),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,topic+'/out_confirmed',lambda c:confirmed.update({key(c):c}),qos_profile_sensor_data)
    fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,n in enumerate(('x','y','z','intensity'))]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def deliver(ns,x=1.01,late_transform=None,expect_map=False):
        stamp=Time(sec=ns//1000000000,nanosec=ns%1000000000)
        c=point_cloud2.create_cloud(Header(stamp=stamp,frame_id='base_link'),fields,[(x,.01,.5,17.)])
        publisher.publish(c)
        if late_transform is not None:
            raw_deadline=time.monotonic()+1.
            while ns not in raw and time.monotonic()<raw_deadline:spin(.02)
            assert ns in raw, 'Raw clearing must publish without waiting for map TF'
            extra=point_cloud2.create_cloud(Header(stamp=Time(sec=(ns+1)//1000000000,nanosec=(ns+1)%1000000000),frame_id='base_link'),fields,[(x,.01,.5,17.)])
            publisher.publish(extra)
            raw_deadline=time.monotonic()+.25
            while ns+1 not in raw and time.monotonic()<raw_deadline:spin(.02)
            assert ns+1 in raw, 'Waiting for map TF must not block subsequent clearing clouds'
            late_transform.header.stamp=extra.header.stamp
            transforms.publish(TFMessage(transforms=[late_transform]))
        end=time.monotonic()+2.
        while time.monotonic()<end:
            spin(.05)
            if ns in raw and ns in confirmed and (not expect_map or confirmed[ns].header.frame_id=='map'):
                return raw[ns],confirmed[ns]
            assert process.poll() is None,'filter exited'
        raise AssertionError('Missing raw/confirmed output at stamp '+str(ns))
    try:
        deadline=time.monotonic()+4
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:spin(.05)
        assert publisher.get_subscription_count()>0
        spin(.3)
        # No map: clearing continues but no obstacle is confirmed.
        original,filtered=deliver(10_000_000_000)
        assert original.width==1 and filtered.width==0
        if not delayed_tf:
            spin(1.1)
            relevant=[d for d in diagnostics if 'Obstacle cloud preprocessing' in d.name]
            assert relevant and relevant[-1].level==DiagnosticStatus.WARN
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05
        m.info.width=m.info.height=200;m.info.origin.position.x=m.info.origin.position.y=-5.
        m.info.origin.orientation.w=1.;m.data=[0]*40000;maps.publish(m);spin(.15)
        # Map without map->base TF also must not bypass confirmation.
        original,filtered=deliver(10_100_000_000)
        assert original.width==1 and filtered.width==0
        # Let this deliberately untransformable cloud expire before the new sequence.
        spin(.85)
        if delayed_tf:
            initial=TransformStamped();initial.header.frame_id='map';initial.child_frame_id='base_link'
            initial.header.stamp=Time(sec=10,nanosec=900000000);initial.transform.rotation.w=1.
            transforms.publish(TFMessage(transforms=[initial]));spin(.15)
        for cycle in range(4):
            ns=11_000_000_000+cycle*800_000_000
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link'
            t.transform.rotation.w=1.;t.transform.translation.x=cycle*.1
            t.header.stamp=Time(sec=ns//1000000000,nanosec=ns%1000000000)
            if not delayed_tf:transforms.publish(TFMessage(transforms=[t]));spin(.15)
            original,filtered=deliver(ns,1.01-cycle*.1,t if delayed_tf else None,expect_map=True)
            assert original.width==1
            assert filtered.width==(1 if cycle==3 else 0)
            if cycle==3:
                assert filtered.header.frame_id=='map'
                assert abs(struct.unpack_from('<f',filtered.data,0)[0]-1.01)<1e-5
                assert struct.unpack_from('<f',filtered.data,12)[0]==17.
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()
        print((tmp_path/'temporal_filter.log').read_text()[-2000:])
    assert process.returncode==0
