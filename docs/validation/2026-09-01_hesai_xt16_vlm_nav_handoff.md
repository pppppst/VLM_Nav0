# Hesai XT16 VLM_Nav handoff

## Runtime chain

```text
/lidar_points
  -> hesai_fastlio_converter
/lidar_points_fastlio + /utlidar/imu
  -> SPARK FAST-LIO
/odometry + /cloud_registered_base
  -> obstacle_cloud_filter
/vlm_nav/obstacle_cloud
  -> pointcloud_to_laserscan:/scan
  -> SLAM Toolbox:/map
  -> Nav2/VLM_Nav
```

The formal Go2 launch starts `hesai_ros_driver_node` and the C++ converter before
SPARK readiness. Use `target_stage:=fastlio`, `obstacle`, `scan`, `slam`, or `nav2`
to stop the staged bringup at a read-only gate. Do not start the Sport bridge in a
hardware verification shell and do not send `/cmd_vel_bridge` commands.

## Host receive-buffer prerequisite

The approximately 2.87 MB `/cloud_registered_base` frames require
`net.core.rmem_max >= 16777216`. The host installs
`/etc/sysctl.d/99-cyclonedds-go2.conf` with only that limit; the production
CycloneDDS XML requests `<SocketReceiveBufferSize min="16MiB"/>`.
`scripts/go2_network_env.sh` fails closed before ROS startup if the maximum is
smaller. `net.core.rmem_default` remains at the OS default because the max-only
configuration passed the single-reader smoke test. See
`2026-09-02_cyclonedds_receive_buffer_fix.md` for evidence and recovery details.

## Frames and extrinsics

`base_link -> hesai_lidar` is the ROS installation TF:

```text
xyz = [0.171, 0, 0.0908]
rpy = [0, 0, 1.570796]
```

SPARK's internal LiDAR-to-Unitree-IMU transform is separate:

```text
T = [0.086231, 0.014655, -0.170388]
R = [ 0, 0.965512, -0.260358,
      1, 0,        0,
      0, -0.260358, -0.965512 ]
```

SPARK is the sole owner of `odom -> base_link`. `fastlio_odom_adapter` must not
be launched in the Go2 path.

## Downstream contract

SPARK's `/cloud_registered_base` is the current IMU-deskewed scan transformed
into `base_link`, not a world/map cloud. The obstacle filter consumes it, applies
height/self-crop/voxel filtering, and publishes `/vlm_nav/obstacle_cloud`. Nav2
costmaps consume that filtered cloud with `sensor_frame: base_link`.

## Verification status

The isolated XT16 + `/utlidar/imu` + SPARK run passed input, IMU initialization,
base TF, odometry, registered cloud, and matching/convergence checks. Full
obstacle/scan/SLAM/Nav2 readiness must be re-run after this formal launch
integration. Any failed stage is a contract blocker and must not be bypassed.

As of 2026-09-02, the CycloneDDS receive-buffer blocker is **CLOSED** and the
formal launch again schedules `/wait_go2_fastlio` after SPARK with no
`minimal_cloud_probe`. The formal waiter lifecycle is restored: the fixed-bag
smoke reached cloud 3/3 after 1.158 s, logged readiness success, and exited 0;
an unmet 0.5 s contract exited 1. Tests also verify that only exit 0 advances a
stage and a non-zero exit emits shutdown.

The converter input QoS blocker is now **CLOSED**: the formal launch supplies
best-effort input and reliable output. The clean staged regression reached
cloud readiness 3/3 after 0.898 s, the waiter exited 0, and the obstacle stage
started 2.5 ms later. FAST-LIO, odometry, and TF remained at approximately
10 Hz; odom/TF rollback and duplicate counts were zero and their stamps matched.
Bridge remained dry-run and DISARMED. Gate 3 is **PASS**. See
`2026-09-02_go2_gate3_final_regression.md`.
