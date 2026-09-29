# `/cloud_registered_base` subscriber delta result

This is a diagnostic result only. No QoS, executor, probe, waiter, converter, or
SPARK estimator fix was applied.

## Common setup

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source ../go2_ws/install/setup.bash
source install/setup.bash
source scripts/go2_network_env.sh
```

The probe was the original flushed working probe, unchanged:

```bash
sleep 3
python3 /tmp/minimal_cloud_subscriber.py
```

Its SHA-256 was
`077ec116f53fdc166094b783462c4646c101f47d151a8950b855c39e4722d663`.

## Minimal failing runtime

```bash
export ROS_LOG_DIR=/tmp/cloud_delta_staged_core_roslog
ros2 launch vlm_nav cloud_runtime_delta_staged.launch.py \
  with_bridge_safety:=false with_fastlio_waiter:=false
```

Live ROS nodes while the probe was running:

```text
/go2_base_to_lidar
/hesai_fastlio_converter
/hesai_ros_driver_node
/hesai_ros_driver_node
/lio_mapping
/minimal_cloud_subscriber
/transform_listener_impl_5a019d1d44b0
```

The duplicate Hesai graph name came from the one Hesai driver process; there
was only one driver OS process.

Observed result:

| Metric | Value |
|---|---:|
| cloud publishers / typed subscribers | 1 / 1 |
| probe callback count / elapsed | 0 / 18.251 s |
| probe callback rate | 0 Hz |
| raw `ros2 topic hz` cloud rate | about 4.02 Hz |
| `/odometry` rate | about 9.08 Hz |
| converter received / published / overwritten | 248 / 226 / 21 |

One resource snapshot (CPU %, RSS KiB) was: static TF 0.8/15988, driver
15.6/120440, converter 50.1/30592, SPARK 20.8/174040, probe 5.0/82252.

## Minimal working runtime

The only launch argument changed was `with_fastlio_waiter`:

```bash
export ROS_LOG_DIR=/tmp/cloud_delta_waiter_only_roslog
ros2 launch vlm_nav cloud_runtime_delta_staged.launch.py \
  with_bridge_safety:=false with_fastlio_waiter:=true
```

Live ROS nodes while the probe was running:

```text
/go2_base_to_lidar
/hesai_fastlio_converter
/hesai_ros_driver_node
/hesai_ros_driver_node
/lio_mapping
/minimal_cloud_subscriber
/transform_listener_impl_62e0e1a71440
/wait_go2_fastlio
```

Observed result:

| Metric | Value |
|---|---:|
| cloud publishers / typed subscribers | 1 / 2 |
| probe callback count / elapsed | 72 / 18.247 s |
| probe callback rate | about 3.95 Hz |
| waiter callback count at the same time | 73 |
| raw `ros2 topic hz` cloud rate | about 4.08 Hz |
| `/odometry` rate | about 4.09 Hz |
| converter received / published / overwritten | 248 / 236 / 10 |

One resource snapshot (CPU %, RSS KiB) was: static TF 1.3/16088, driver
16.3/119028, converter 51.1/31028, SPARK 19.2/182440, waiter 6.6/59028,
probe 9.0/56236.

## Repeated boundary with bridge/safety present

Keeping bridge and safety present reproduced the same boundary:

| `with_fastlio_waiter` | Probe result | Raw cloud | Odom | Converter recv/pub/overwrite |
|---|---:|---:|---:|---:|
| false | 0 / 18.248 s | 4.04 Hz | 10.00 Hz | 248 / 247 / 0 |
| true | 71 / 18.252 s = 3.89 Hz | 4.03 Hz | 4.02 Hz | 248 / 246 / 0 |

## Delta boundary

The minimal observed difference is one additional typed
`sensor_msgs/msg/PointCloud2` subscription process, `/wait_go2_fastlio`, using
BEST_EFFORT / VOLATILE / KEEP_LAST depth 1. With only the original typed probe,
the probe received zero callbacks. With the second typed subscriber present,
both typed subscribers received about 4 Hz.

This establishes the boundary but does not yet establish the underlying DDS or
deserialization cause. `ros2 topic hz` was only auxiliary evidence because the
Humble implementation uses `raw=True` unless a filter expression is supplied.

The historical manual-working run at 12:11 also had `/wait_go2_fastlio` alive in
the staged runtime, so it was process-standalone but not subscriber-standalone.
