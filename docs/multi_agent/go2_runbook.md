# Go2 cooperative tasks

Repository: `/home/isee-pst/unitree_ros2/VLM_Nav`, branch `go2-cooperative-task-layer`.
ROS: Humble. New sources: `cooperative_ws/src` (root `vlm_nav` remains a separate package).
Task semantics: Go2=A, Ranger=B; both fresh READY before C; same C observation;
Go2 deposit **and whole-body exit** before Ranger. No physical arm.

Field preparation and independent Go2 checks: [go2_field_checklist.md](go2_field_checklist.md).
The independent entry defaults to read-only and does not unlock cooperative tasks.

## Build and offline validation

```bash
cd /home/isee-pst/unitree_ros2/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon list --base-paths "$PWD" "$PWD/cooperative_ws/src"
colcon --log-base /tmp/go2-coop/log build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base /tmp/go2-coop/build --install-base /tmp/go2-coop/install \
  --packages-up-to multi_agent_nav \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCOOPERATIVE_RUNTIME_PYTHON=/home/isee-pst/venv/co-nav-real/bin/python
bash scripts/test_cooperative_no_hardware.sh
```

System Python3.10 generates/installs messages. Task executables and their ament
pytest use the existing CPython3.10 `co-nav-real` environment (httpx/vision SDK).
This avoids its setuptools/packaging build incompatibility without changing it.
The built executable shebang must point at the configured runtime interpreter.
Prepend repository PYTHONPATH without removing the ROS-generated entries.
Do not source the legacy `unitree_ros2/install` after the cooperative overlay:
its older Sport binary lacks the response subscription expected by the baseline.
Build, test, source and result all use `/tmp/go2-coop`; rebuilding is required
after that directory is removed. No system upgrade is needed.

## Local dual-fake ROS example

```bash
source /opt/ros/humble/setup.bash
source /tmp/go2-coop/install/setup.bash
export ROS_LOG_DIR=/tmp/go2-coop/ros-logs
ros2 launch multi_agent_nav cooperative_mock.launch.py
```

The launch fixes Domain31, localhost only and a unique prefix
`/multi_agent/mock/run_<id>`, printed in node/service names and logs. It starts
three task nodes, no sensor/action clients, velocities, bridge or real adapters.
No motion before explicit start. In a second terminal use that actual prefix:

```bash
source /opt/ros/humble/setup.bash
source /tmp/go2-coop/install/setup.bash
export ROS_LOG_DIR=/tmp/go2-coop/ros-logs
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
ros2 run multi_agent_nav multi_agent_start_task start --scenario synthetic --prefix /multi_agent/mock/run_<id>
ros2 run multi_agent_nav multi_agent_start_task pause --prefix /multi_agent/mock/run_<id>
```

The automated ROS smoke test discovers its own prefix, starts the task and
asserts DONE and both final states; it needs no manual commands or ROS daemon.

## Live preparation only — not executed this round

Keep the existing Domain0 sensor/localization/Nav2 stack and its validated
startup scripts. `manualnav2.sh localization_mode:=amcl map:=<actual.yaml>` is
the existing manual stack; `VLMNav-go.sh` additionally starts the autonomous
single-target loop and must not run concurrently with cooperative task ownership.
Do not run either script as an offline test: they contain hardware probes/ARM.

The task launch below starts only coordinator + Go2 task agent, no Ranger,
fake peer, bridge, low-level stack or ARM. It defaults to IDLE; incomplete
configuration constructs no live adapter and StartTask rejects with reasons.

```bash
source /opt/ros/humble/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /tmp/go2-coop/install/setup.bash
export ROS_LOG_DIR=/tmp/go2-coop/ros-logs
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_NET_IFACE=enp4s0
export CYCLONEDDS_URI=file:///tmp/go2-coop/install/vlm_nav/share/vlm_nav/config/cyclonedds_go2.xml
ros2 launch multi_agent_nav go2_cooperative.launch.py config_dir:=<reviewed-live-config-directory>
```

In a second terminal, repeat the same source/export commands above. Field
authorization, open gates and both real peers are required before invoking:

```bash
ros2 run multi_agent_nav multi_agent_start_task start --scenario lab
ros2 run multi_agent_nav multi_agent_start_task pause
```

The launch fixes Domain0/CycloneDDS/enp4s0 using the existing XML; its DDS
configuration does not move any validated low-level process to Domain31.

Required live data/acceptance:

- Actual independent map identities/versions and map paths; lab_ref equals Go2
  map, Ranger transform from measured correspondences and an independent check.
  alignment.report must retain source/common/held-out points, thresholds and
  identities; startup recomputes the fit and checks the configured transform.
  map_digest binds each PC's YAML/image content; only local files are read on each PC.
- Measured scene viewpoints, wait/exit points, doorway and C protection polygons,
  and doorway transit exits for each robot. Regions must cover selected approach
  and exit paths. Live templates have no invented coordinates.
- Go2 configured fresh executing-path topic and verified association with actual
  NavigateToPose replans. A ComputePathToPose precheck or guessed `/plan` topic is
  insufficient. This interface is currently an explicit live gate.
- Measured stopping margin, verified camera geometry/depth units, ABC inference
  samples, full-body dynamic obstacle detection and actual motion-direction coverage.
  Configure path_source_verified/camera_calibrated/vision_validated/
  peer_obstacle_verified only with field evidence. Existing AMCL input is unchanged.
  costmap_source_verified additionally requires valid fresh full raw payload, TF and
  active dynamic layers. The existing AMCL static-map overlay removes dynamic layers;
  it must receive separate field review before cooperative obstacle acceptance.
- Sole task goal ownership, existing NAV_READY/CONTROL_ARMED gates, both real agents
  fresh/located/stopped, identical protocol and alignment version. No auto ARM.

`ros2 run multi_agent_nav multi_agent_calibration references.yaml` prints a candidate report only; it
does not publish TF, write active configuration or hot-swap alignment.

The first executing-path publication may arrive after NavigateToPose starts.
`robot_go2.path_start_timeout` (default 1 s) bounds the wait, allowed only while
fresh odometry confirms physical stop and the stationary stopping envelope is
clear/permitted. Movement before the first valid path, timeout, or any later
path loss cancels and pauses. Preflight paths never satisfy this monitor.

Pause invalidates pending motion results immediately; state must then confirm
action termination and physical stop. A C EXIT keeps the original deposit permit
identity/token; loss/failure cannot release it. New tasks reject held permits.
An uncertain occupied region requires field clearance and stopped peer reconciliation;
do not clear occupancy just because a process restarted or a timeout elapsed.
Once a new task is accepted, SYNC for every retired task is rejected; a delayed
old SYNC cannot reset command history and revive canceled execution.

For an authorized live shutdown, pause first and verify stopped agent states.
The existing `scripts/stopgo.sh` provides the established single-robot cancellation
and DISARM procedure; do not call it during mock tests. SIGINT cleanup attempts
cancellation/stopping; it is not proof of hardware stop. After stopping task nodes,
the unchanged single-robot stack is the rollback path.
