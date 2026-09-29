# CycloneDDS large typed PointCloud2 reproducer

Date: 2026-09-02. This is diagnosis only. No production QoS, SPARK, waiter,
launch, DDS transport, or buffer setting was changed.

## Versions

```text
ROS_DISTRO=humble
Linux 6.8.0-138-generic x86_64
ros-humble-cyclonedds             0.10.5-2jammy.20260226.013234
ros-humble-rmw-cyclonedds-cpp     1.3.4-1jammy.20260718.002719
ros-humble-fastrtps               2.6.12-1jammy.20260723.233918
ros-humble-rmw-fastrtps-cpp       6.2.10-1jammy.20260724.002510
```

CycloneDDS used `config/cyclonedds_go2.xml`: the selected interface was
`enp4s0`, multicast was enabled, and no tracing section was configured.

## Fixed input and frame size

The exact live SPARK CDR stream was captured without a typed reader:

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source ../go2_ws/install/setup.bash
source install/setup.bash
source scripts/go2_network_env.sh
export ROS_LOG_DIR=/tmp/dds_capture_roslog

ros2 launch vlm_nav cloud_runtime_delta_staged.launch.py \
  with_bridge_safety:=false with_fastlio_waiter:=false

# In another shell, for 22 seconds:
ros2 bag record -o /tmp/cloud_registered_base_repro_20260902 \
  /cloud_registered_base
```

Bag identity:

```text
DB3 SHA-256: 95077f4678fe667cadc5e615a93024458da6e2ed068d24cb853565baf6479f12
metadata SHA-256: c40cae22204357ee1dec2a00ac46a5ac07c168be77c26ecff5b1184b5fec7c16
207 frames over 21.678342819 s
```

Five CDR samples were successfully deserialized offline with
`rclpy.serialization.deserialize_message`. Typical frame properties were:

```text
width:                 59,860-59,886 points in the sampled frames
height:                1
point_step:            48 bytes
row_step:              about 2,873,000 bytes
len(msg.data):         equal to row_step
serialized frame:      mean 2,873,498 bytes
serialized range:      2,870,756-2,876,084 bytes
frame_size_MB:         2.873498 MB
publish_Hz:            9.502571 Hz
approx_MB_per_sec:     27.305622 MB/s
```

The earlier approximately 4 Hz from `ros2 topic hz` was not the publisher
rate: Humble's command uses a raw subscription by default and only took about
4 Hz under this load. Bag timestamps/counts show the offered rate was about
9.5 Hz.

## Minimal reader

The only custom process in the matrix was
`scripts/pointcloud_typed_reader_repro.py` (SHA-256
`21c4c6eb15d71829ffdcb8c351f1aa9d26b3995f4cb22f1a270aa4388decab88`).
It has one typed `sensor_msgs/msg/PointCloud2` subscription using
BEST_EFFORT / VOLATILE / KEEP_LAST depth 1. It records count, callback rate,
first-callback time, header age, dimensions, and data length. It does not
publish or create other ROS entities.

All matrix runs used the same bag player, bag, topic, machine, payloads,
timing, and reader QoS. The bag player's offered endpoint was RELIABLE /
VOLATILE and remained unchanged across RMW runs. Its reported history depth
was 10, so this independent reproducer does not depend on SPARK's live writer
depth of 1.

## Commands

Common setup:

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source ../go2_ws/install/setup.bash
source install/setup.bash
source scripts/go2_network_env.sh
export ROS2CLI_NO_DAEMON=1
```

For CycloneDDS:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=/tmp/dds_matrix_cyclone_roslog
```

For Fast DDS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/dds_matrix_fastdds_roslog
```

One-reader run:

```bash
REPRO_READER_ID=1 timeout --signal=INT 28 \
  python3 scripts/pointcloud_typed_reader_repro.py > /tmp/reader1.out 2>&1 &
reader1=$!
sleep 2
ros2 bag play /tmp/cloud_registered_base_repro_20260902
wait "$reader1" || true
```

Two-reader run:

```bash
REPRO_READER_ID=1 timeout --signal=INT 28 \
  python3 scripts/pointcloud_typed_reader_repro.py > /tmp/reader1.out 2>&1 &
reader1=$!
REPRO_READER_ID=2 timeout --signal=INT 28 \
  python3 scripts/pointcloud_typed_reader_repro.py > /tmp/reader2.out 2>&1 &
reader2=$!
sleep 2
ros2 bag play /tmp/cloud_registered_base_repro_20260902
wait "$reader1" || true
wait "$reader2" || true
```

During each run this command recorded the graph without adding a reader:

```bash
ros2 topic info -v /cloud_registered_base
```

## Results

| RMW | Typed readers | Reader results | First callback | Pub/sub count | Repeatability |
|---|---:|---|---|---:|---|
| CycloneDDS | 1 | 0 / 27.870 s, 0 Hz | none | 1 / 1 | Reproduced in the independent bag run and the live minimal core |
| CycloneDDS | 2 | 102 each, 4.077/4.078 Hz | 2.869 s after reader start, about 0.87 s after player start | 1 / 2 | Reproduces the earlier live core boundary |
| Fast DDS | 1 | 203, 9.539 Hz | 3.095 s after reader start, about 1.10 s after player start | 1 / 1 | Repeated: 203, 9.539 Hz, first callback at 3.088 s |
| Fast DDS | 2 | Not run | Not run | Not run | Stopped after two stable boundaries, as required |

There was no delayed spontaneous recovery. Cyclone with one reader stayed at
zero for the full run. Cyclone with two readers and Fast DDS with one reader
started callbacks immediately after discovery and continued until playback
ended.

Bag replay preserves the original header timestamps. Consequently the measured
header ages (roughly 247 seconds for Cyclone and 303/364 seconds for the Fast
DDS runs) are expected stale-bag ages, not transport latency measurements.

Auxiliary CPU/RSS snapshots did not indicate exhaustion:

```text
Cyclone, 1 reader: reader 4.4% / 83,204 KiB; player 29.0% / 638,292 KiB
Cyclone, 2 readers: readers 5.4% / 55,916 KiB and 5.2% / 55,736 KiB;
                    player 27.6% / 643,800 KiB
Fast DDS, 1 reader: reader 6.2% / 62,444 KiB; player 28.3% / 657,796 KiB
```

## CycloneDDS verbosity A/B

Not run. The independent Cyclone 1-vs-2-reader boundary and the repeated
Cyclone-vs-Fast-DDS one-reader boundary were both stable. The requested stop
condition therefore applied before changing tracing verbosity. No CycloneDDS
transport or logging parameter was changed.

## Supported conclusion

The failure is independently reproducible without VLM_Nav staged orchestration,
SPARK, Hesai, TF, bridge, safety, or readiness logic. For this approximately
2.87 MB, approximately 9.5 Hz CDR stream:

```text
rmw_cyclonedds_cpp + one typed reader  -> 0 callbacks
rmw_cyclonedds_cpp + two typed readers -> both about 4.08 Hz
rmw_fastrtps_cpp + one typed reader    -> about 9.54 Hz
```

The same stored CDR deserializes offline, and the same typed Python reader and
payload work under Fast DDS. The current evidence therefore localizes the
problem to the CycloneDDS/rmw_cyclonedds_cpp large-sample typed-reader delivery
path, under this machine and Cyclone interface configuration, rather than to
the application callback or PointCloud2 payload in general.

## Not established

This does not identify the exact faulty layer or mechanism inside CycloneDDS,
`rmw_cyclonedds_cpp`, the kernel/network path, or their interaction. It does
not establish a message-size threshold, prove that tracing changes behavior,
or show whether newer/older CycloneDDS versions behave differently. It also
does not justify changing production RMW, adding a second reader, or changing
buffers/QoS. Those would be separate experiments or workarounds.
