# CycloneDDS socket receive-buffer A/B

Date: 2026-09-02. This was a temporary diagnostic experiment. It did not
change VLM_Nav, SPARK, the waiter, launch structure, PointCloud2 content,
reader QoS, the production RMW, or any permanent sysctl configuration.

## Fixed reproducer

Both runs used exactly the same inputs and processes:

```text
RMW:                 rmw_cyclonedds_cpp
publisher:           rosbag2_player
topic:               /cloud_registered_base
publisher count:     1
typed reader count:  1
reader QoS:          BEST_EFFORT / VOLATILE / KEEP_LAST depth 1
bag:                 /tmp/cloud_registered_base_repro_20260902
DB3 SHA-256:         95077f4678fe667cadc5e615a93024458da6e2ed068d24cb853565baf6479f12
messages:            207
bag duration:        21.678342819 s
offered rate:        9.502571 Hz
mean frame size:     2.873498 MB
offered throughput:  27.305622 MB/s
reader:              scripts/pointcloud_typed_reader_repro.py
```

The reader was started two seconds before bag playback and allowed to run for
28 seconds. `ros2 topic info -v` confirmed one publisher and one subscriber in
both runs.

## A: baseline

Environment and kernel state:

```text
net.core.rmem_default = 212992
net.core.rmem_max     = 212992
CYCLONEDDS_URI=file:///home/isee-pst/unitree_ros2/VLM_Nav/scripts/../config/cyclonedds_go2.xml
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The production XML had SHA-256
`32254549af2b3c573381388564de072c48098174edf04a1821705b391160537e`
and contained no `<Internal>` or `<SocketReceiveBufferSize>` element. It had no
git diff before or after the experiment.

Command:

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source VLM_Nav/install/setup.bash
source VLM_Nav/scripts/go2_network_env.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=/tmp/dds_rmem_baseline_roslog
export ROS2CLI_NO_DAEMON=1

REPRO_READER_ID=1 timeout --signal=INT 28 \
  python3 VLM_Nav/scripts/pointcloud_typed_reader_repro.py \
  > /tmp/dds_rmem_baseline_reader.out 2>&1 &
reader=$!
sleep 2
ros2 bag play /tmp/cloud_registered_base_repro_20260902 \
  > /tmp/dds_rmem_baseline_player.out 2>&1 &
player=$!
sleep 2
ros2 topic info -v /cloud_registered_base
wait "$player" || true
wait "$reader" || true
```

Result:

```text
publisher/subscriber count: 1 / 1
callback_count:             0
reader elapsed:             27.865371 s
callback Hz:                0
first callback:             none
bag playback/offered Hz:    9.502571
```

Auxiliary snapshot: reader CPU 4.0%, RSS 83,768 KiB; player CPU 20.6%, RSS
638,364 KiB.

## B: 16 MiB receive buffer

Only the receive-buffer combination was changed. No other sysctl or CycloneDDS
setting was changed.

Temporary kernel settings:

```bash
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.rmem_default=16777216
```

Both `sysctl` and `/proc/sys/net/core/rmem_*` returned `16777216` before the
run.

The independent experiment file was
`/tmp/cyclonedds_large_cloud_16m.xml`, SHA-256
`cb035872e7edd16ae50b2ad2fac58b789104b708a56b4a51739bd63a09380536`.
Its only semantic difference from the production XML was:

```xml
<Internal>
  <SocketReceiveBufferSize min="16MiB"/>
</Internal>
```

Command:

```bash
cd /home/isee-pst/unitree_ros2
source /opt/ros/humble/setup.bash
source go2_ws/install/setup.bash
source VLM_Nav/install/setup.bash
source VLM_Nav/scripts/go2_network_env.sh
export CYCLONEDDS_URI=file:///tmp/cyclonedds_large_cloud_16m.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=/tmp/dds_rmem_16m_roslog
export ROS2CLI_NO_DAEMON=1

REPRO_READER_ID=1 timeout --signal=INT 28 \
  python3 VLM_Nav/scripts/pointcloud_typed_reader_repro.py \
  > /tmp/dds_rmem_16m_reader.out 2>&1 &
reader=$!
sleep 2
ros2 bag play /tmp/cloud_registered_base_repro_20260902 \
  > /tmp/dds_rmem_16m_player.out 2>&1 &
player=$!
sleep 2
ros2 topic info -v /cloud_registered_base
ss -u -a -m -p
wait "$player" || true
wait "$reader" || true
```

`ss -u -a -m -p` showed `rb16777216` on the Cyclone sockets belonging to the
reader and bag player. No Cyclone configuration parsing warning or error was
present in either process output.

Result:

```text
publisher/subscriber count: 1 / 1
callback_count:             207 (all bag messages)
reader elapsed:             27.870360 s
callback Hz:                9.678301 Hz
first callback after reader start: 2.650156 s
first callback after player start: about 0.650 s
bag playback/offered Hz:    9.502571
```

The callback rate uses the interval between the first and last received
callbacks, while the offered rate uses the full recorded bag interval; this
accounts for the small numerical difference. The decisive observation is that
all 207 messages were delivered.

Auxiliary snapshot: reader CPU 7.4%, RSS 57,320 KiB; player CPU 29.3%, RSS
638,304 KiB.

## A/B result

| Case | Linux rmem default/max | Cyclone socket request | Readers | Result |
|---|---:|---|---:|---:|
| Baseline | 212,992 / 212,992 | not configured | 1 | 0 / 27.865 s, 0 Hz |
| 16 MiB | 16,777,216 / 16,777,216 | `min="16MiB"` | 1 | 207/207, 9.678 Hz |

The single controlled receive-buffer change moved the same CycloneDDS typed
reader from zero callbacks to delivery of every message at approximately the
publisher rate. This strongly supports insufficient receive socket buffering
as the direct trigger for the approximately 2.87 MB at 9.5 Hz large-sample
delivery failure in this environment.

No 4/8/16 MiB threshold matrix was run. The 16 MiB case met the explicit
success and stop criterion, and a threshold experiment requires separate
confirmation.

## What is not established

This experiment changed the Linux receive-buffer limits and Cyclone's requested
socket receive buffer together, as the test specification required. It does not
separately establish which one alone is sufficient. It also does not identify
the exact faulty code or mechanism inside CycloneDDS, `rmw_cyclonedds_cpp`, UDP
fragment reception/reassembly, or the Linux socket path. It does not explain
why a second typed reader changed delivery behavior.

The result is not authorization to change the production RMW, add a second
reader, change production QoS, or make a permanent sysctl/XML change.

## Restoration

The temporary kernel values were restored with:

```bash
sudo sysctl -w net.core.rmem_default=212992
sudo sysctl -w net.core.rmem_max=212992
```

Final verification:

```text
net.core.rmem_default = 212992
net.core.rmem_max     = 212992
CYCLONEDDS_URI=file:///home/isee-pst/unitree_ros2/VLM_Nav/scripts/../config/cyclonedds_go2.xml
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
production XML git diff: empty
production XML SHA-256: 32254549af2b3c573381388564de072c48098174edf04a1821705b391160537e
active reproducer/player processes: none
```
