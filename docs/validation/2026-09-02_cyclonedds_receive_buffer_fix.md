# Formal CycloneDDS receive-buffer mitigation

Date: 2026-09-02.

## Root cause and evidence

`/cloud_registered_base` is approximately 2.873 MB per frame at approximately
9.5 Hz (27.3 MB/s). With CycloneDDS 0.10.5 and `rmw_cyclonedds_cpp` 1.3.4, the
212,992-byte Linux receive-buffer condition produced zero callbacks for one
typed PointCloud2 reader. A temporary 16 MiB A/B delivered 207/207 frames at
9.678 Hz. The detailed controlled experiment is in
`2026-09-02_cyclonedds_socket_receive_buffer_ab.md`.

No SPARK estimator, waiter QoS/executor, staged scheduling, or default RMW
change was required to restore the large-message typed path.

## Permanent configuration

Production `config/cyclonedds_go2.xml` preserves the existing interface,
multicast, and discovery settings and adds only:

```xml
<Internal>
  <SocketReceiveBufferSize min="16MiB"/>
</Internal>
```

The host file `/etc/sysctl.d/99-cyclonedds-go2.conf` contains only:

```text
net.core.rmem_max=16777216
```

Active values after `sudo sysctl --system`:

```text
net.core.rmem_max     = 16777216
net.core.rmem_default = 212992
```

`scripts/go2_network_env.sh` now exits before selecting the Go2 DDS environment
when `net.core.rmem_max` is unavailable, non-numeric, or below 16 MiB. The
production RMW remains `rmw_cyclonedds_cpp`.

## Max-only smoke test

The fixed bag `/tmp/cloud_registered_base_repro_20260902` and the unchanged
single typed reader `scripts/pointcloud_typed_reader_repro.py` were run with the
production XML and one publisher/one subscriber.

```text
messages:                 207 / 207
typed callback rate:      9.677914 Hz
first callback:           about 0.606 s after player start
publisher/subscriber:     1 / 1
Cyclone parse warnings:   none
net.core.rmem_max:        16777216
net.core.rmem_default:    212992
socket receive buffer:    rb33554432 reported by Linux ss
```

Linux accounts twice the requested `SO_RCVBUF` size, so `rb33554432` confirms
that Cyclone's 16 MiB request was accepted. The max-only configuration is
sufficient; `rmem_default` was not raised.

The actual `wait_go2_fastlio` executable was also tested as the only typed
reader against the same bag. It received 204 frames at approximately 9.54 Hz;
the three initial messages were missed during discovery.

## Formal 60-second preflight

Report: `/tmp/go2_rmem_formal_preflight_60s.json`.

```text
formal_preflight:                 PASS
healthy:                          true
LiDAR samples / Hz:               600 / 10.005
LiDAR timestamp rollback:         0
IMU samples / approximate Hz:     14908 / 248.5
IMU timestamp rollback:           0
IMU nonfinite/extreme/range:       0 / 0 / 0
point-time matched points:         38373760
converter/driver packet loss:      none observed
```

The Hesai firetime-file warning remains the previously documented
non-blocking driver warning.

## Read-only live FAST-LIO run

The existing formal command was run with `target_stage:=fastlio`, the fresh
preflight report, bridge dry-run, and DISARMED state. No motion goal or real
Sport request was sent.

```text
single live typed cloud reader:    771 callbacks over 79.860 s, 9.642 Hz
converter received/published:      798 / 796
converter overwritten/errors:      0 / 0
SPARK diagnostic frames:           804 (last frame 839)
effective points median/max:        229 / 251
sustained No Effective Points:      none
odometry samples / Hz (45 s):       451 / 10.002
odometry rollback/duplicate:        0 / 0
odom->base_link TF samples / Hz:    450 / 10.002
TF rollback/duplicate:              0 / 0
odom/TF nearest stamp P95:          0 s
static drift over 45 s:             0.00415 m / 0.0388 degrees yaw
SYSTEM_READY:                       false -> true
bridge:                             dry-run and DISARMED
```

The one odometry sample without an exact nearest TF match was at the collector
window boundary; 95% and the median stamp difference were zero. Startup had
one LiDAR loopback clear and transient `No point` warnings; the subsequent run
maintained normal effective-point counts and had no `No Effective Points`
warning.

## Tests and build

```text
new XML/fail-fast tests:            2 passed
Gate 3 related tests:               51 passed
colcon build --packages-select vlm_nav --symlink-install: PASS
```

The former four delta-debug structure assertions were replaced with formal
behavior checks for message threshold, timeout, exit status, and fail-closed
stage transition.

## Gate 3 integration status

The formal `go2_system.launch.py` diagnostic substitution was removed on
2026-09-02: it again creates `/wait_go2_fastlio`, schedules it with SPARK after
the base-TF gate, and no longer references `minimal_cloud_probe`. The package
build passes and the launch contract test confirms that only the formal waiter
is present.

The waiter delta-debug branch was removed after comparison with git history.
The restored waiter uses its original dynamic subscription, TF listener,
timer/check, single-node `spin_once`, and exit-code lifecycle. A fixed-bag smoke
reached cloud 3/3 after 1.158 s, reported readiness success, and exited 0. A
missing-topic smoke timed out and exited 1; launch behavior tests prove that a
non-zero exit emits shutdown instead of scheduling the next stage.

The subsequent bounded fix changed only the formal converter input reliability
to best-effort and retained reliable output. In the clean staged regression the
converter, FAST-LIO, waiter, and obstacle transition all completed their Gate 3
contracts.

Current status:

```text
CycloneDDS receive-buffer: CLOSED
formal launch probe cleanup: PASS
formal waiter lifecycle:    PASS (bag and timeout smoke)
formal staged integration:  PASS
Gate 3:                     PASS
```
