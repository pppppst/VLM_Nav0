# Go2 Gate 3 final regression

Date: 2026-09-02.

## Formal baseline recovery

Git history (`f648add`, `4a61f58`) confirms that the formal waiter uses dynamic
message-type resolution, default callback groups, a TF listener, a 0.2 s timer,
single-node `rclpy.spin_once`, three required messages, and a non-zero default
exit code. MultiThreadedExecutor, static type tables, TF skipping, environment
and identity logging, heartbeat callbacks, and unconditional fastlio spins were
uncommitted delta-debug changes and were removed.

The formal launch schedules SPARK and `/wait_go2_fastlio` after the base-TF
gate. It contains no `minimal_cloud_probe`, debug TimerAction, or second typed
readiness reader. Its process-exit handler advances only on return code zero;
otherwise it emits shutdown.

## Tests and lifecycle smoke

```text
related pytest:                    51 passed
vlm_nav build:                     PASS
missing topic, timeout 0.5 s:      exit 1
fixed bag cloud readiness:         1/3 -> 2/3 -> 3/3
time to 3/3:                       1.158 s
fixed bag readiness result:        PASS, exit 0
exit 0 transition test:            next stage returned
exit 1 transition test:            Shutdown returned; next stage absent
```

The bag smoke used `/tmp/cloud_registered_base_repro_20260902`, CycloneDDS,
one publisher, and the sole formal waiter subscriber. No callback-frequency
estimate was derived from the first three frames.

## Formal preflight

Report: `/tmp/go2_gate3_final_preflight_60s.json`.

```text
formal_preflight:                  PASS
LiDAR samples / Hz:                600 / 10.0017
LiDAR timestamp rollback:          0
IMU samples / approximate Hz:      14911 / 248.5
IMU timestamp rollback:            0
IMU nonfinite/extreme/range:        0 / 0 / 0
point-time matched/outside:         38351360 / 48640
```

## Converter input QoS correction

The first staged attempt stopped because the formal converter input parameter
was `reliable`, while the deployed converter contract requires:

```text
input_reliability:  best_effort
output_reliability: reliable
```

The Hesai publisher was measured as RELIABLE, VOLATILE, KEEP_LAST depth 10.
The formal launch was changed only at the converter input-reliability field;
all topics, durability, history, processing, and reliable output remained
unchanged. Runtime parameters confirmed best-effort input and reliable output.

## Clean staged result

The final run used `target_stage:=obstacle`, bridge dry-run, and DISARMED state.
No PointCloud2 measurement subscriber was added during the clean run.

```text
converter:                         running, approximately 10 Hz
converter initial report:          received 46 / published 44 / errors 0
FAST-LIO:                          approximately 10 Hz
effective points:                  approximately 240-280, no sustained failure
cloud readiness:                   1/3 -> 2/3 -> 3/3
time to cloud 3/3:                 0.898 s
FAST-LIO waiter:                   readiness PASS, clean exit 0
next-stage latency:                2.5 ms
next stage:                        obstacle_cloud_filter started
obstacle waiter:                   clean exit 0
odometry:                          301 samples / 9.9999 Hz
odometry rollback / duplicate:     0 / 0
odom->base_link TF:                300 samples / 9.9996 Hz
TF rollback / duplicate:           0 / 0
odom/TF stamp median / P95:         0 / 0 s
static drift over 30 s:             0.00375 m / 0.0106 degrees yaw
bridge:                            dry-run, DISARMED
real motion output:                none
```

The single unmatched odom/TF nearest sample was at the collector window
boundary; every paired sample had an identical stamp.

```text
CycloneDDS receive-buffer blocker: CLOSED
converter input QoS blocker:       CLOSED
FAST-LIO waiter lifecycle:         PASS
formal staged transition:          PASS
Gate 3:                            PASS
bridge:                            dry-run, DISARMED
real motion output:                none
```
