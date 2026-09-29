# Go2 body-IMU extrinsic / Gate 2 check (2026-09-03)

## Configuration applied

The XT16 profile now uses the Unitree Go2 body IMU adapter:

* `base_link -> body_imu`: `[-0.02557, 0, 0.04232]`, identity rotation.
* `body_imu -> hesai_lidar`: `[0.1710, 0, 0.0908]`, identity rotation.
* FAST-LIO receives the inverse LiDAR-to-IMU transform
  `[-0.1710, 0, -0.0908]` and subscribes to `/body_imu`.
* The composed ROS `base_link -> hesai_lidar` translation is
  `[0.14543, 0, 0.13312]`.

`lowstate_imu_adapter` maps `/lowstate.imu_state` to `sensor_msgs/Imu` on
`/body_imu`. Because `LowState` has no ROS header, it emits a monotonic host
anchored timestamp.

## Gate 2 evidence

The clean, uniquely remapped adapter sample (60 s, one publisher and one
typed subscriber) produced 29,975 samples at 499.58 Hz. Header timestamps had
zero backward steps. Acceleration norm statistics were:

```text
mean 9.59769   std 0.03037   P0.1 9.51193   P1 9.52737
min  9.50319   max 9.69458
```

The old 9.6 m/s² lower bound is therefore not a validated body-IMU threshold
(52.6% of these samples are below it). It remains fail-closed in
`go2_preflight_xt16.yaml`; no threshold was changed based on this run.

The Hesai terminal was closed during this check, so a simultaneous live
Hesai→converter→FAST-LIO Gate 2 run was not possible. Gate 2 is **BLOCKED for
threshold/runtime re-validation**, while the transform/config and monotonic
adapter checks are complete.

## Verification

```text
colcon build --packages-select vlm_nav --symlink-install: PASS
36 focused Python tests: PASS
```

