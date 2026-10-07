# Local implementation delivery — 2026-10-07

Actual repository `/home/isee-pst/unitree_ros2/VLM_Nav`; original branch
`VLM_nav_go`, base `32eab6244ec4b483807f8ed68e8667f5495874bb`.
Implementation branch `go2-cooperative-task-layer`, code through `d5c9680`;
the following documentation commit contains this delivery record. No merge/push.
The two original root input plans remain untouched and untracked; tracked copies
are in `docs/superpowers/plans`. No applicable AGENTS.md was found.

Tasks 0–8 are complete for the requested **local implementation** scope:
interface evidence, independent messages/configuration, SE(2)/offline calibration,
serial agent/Fake, coordinator/RulePlanner, resource and region supervision,
Go2 adapter, installed ROS mock/gated live launch, and this handoff.
C permit identity spans deposit and EXIT, including the original deposit
parent_command_id/token; Ranger receives C only after Go2 whole-body exit.
READY/deposit are Nav2-success simulation semantics, with no physical arm.

## Final checks (installed overlay, after review fixes)

| Check | Actual result | Evidence |
|---|---|---|
| Baseline Python, all `test/` | **273 passed**, 8.81 s | [baseline-final.log](evidence/baseline-final.log) |
| Baseline C++, 3 CTest executables | **21 gtest cases passed** (2 obstacle filter + 17 bridge state + 2 speed generator) | [baseline-cpp.log](evidence/baseline-cpp.log) |
| New cooperative pytest | **49 passed**, 10.93 s, no skips | [cooperative.log](evidence/cooperative.log) |
| colcon build, root + 2 new packages | **3 packages finished**, final incremental build | [build.log](evidence/build.log) |
| ament cooperative tests | **49 pytest + 1 parser**, no failures | [colcon-test.log](evidence/colcon-test.log), [test-result.log](evidence/test-result.log) |
| colcon result aggregate | **73 entries**, 0 errors/failures/skips | 49 new pytest + 1 parser + 2 CTest wrapper entries + 21 old gtest; excludes baseline Python |
| Both launch descriptions | Parsed with `--show-args`; live launch not run | [launch-and-packages.log](evidence/launch-and-packages.log) |
| Package discovery | vlm_nav, multi_agent_nav, multi_agent_task_msgs, each once | Same log |
| Git diff whitespace | `git diff --check`, exit 0 | Checked before commits |

Baseline and new results are separate; ament reruns the same 49 pytest cases,
so it does not create another 49 distinct cases. Historical “52 tests” was an
earlier partial baseline, not this checkout's full baseline. Before implementation,
all 273 Python cases also passed on the original branch after environment fixes:
[baseline-verified.log](evidence/baseline-verified.log).

## Reproduction

Full source/build commands are in [go2_runbook.md](go2_runbook.md). These were
executed, using Humble and the existing go2_ws dependency overlay:

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

The script sources `/tmp/go2-coop/install`, retains ROS PYTHONPATH, selects the
newly built Sport binary, fixes localhost Domain31/FastDDS, and runs baseline
Python, baseline C++, new pytest, selected colcon tests and test-result. Runtime
tests of existing low-level nodes use dry_run/remapped mock inputs; they do not
exercise hardware. Build generation uses system Python3.10; installed task
entrypoints/tests use same-ABI co-nav-real Python3.10. No ROS, SDK, system package
or existing environment was upgraded. `/tmp` outputs may be removed; small
success logs and gtest result XML are retained beside this document.

Environment repairs addressed overwritten ROS PYTHONPATH, a stale legacy Sport
binary selected by overlay order, and venv setuptools/packaging incompatibility.
The existing evidence test's literal branch assertion was changed to verify the
actual Git branch so it remains valid on a work branch. Default VLMNavigator
behavior remains unchanged; its opt-in passive observer reuses RGB-D/TF geometry.

## Coverage and independent review

Pure logic checks task/result fencing, duplicate commands, READY barrier, fixed
assignments and same C, nonzero synthetic Ranger transform, independent fit
validation, door-before-C transit, original C permit across EXIT, FIFO/occupancy
freeze, whole-body exit, unreachable approaches, API error/timeout and peer loss.
Adapter checks cover ABC parsing, invalid depth, nonblocking inference, cancellation
before action acceptance, stale odometry and passive observation-only mode.
Generated message round trips preserve resources and identities.

The ROS smoke test starts three installed task executables on isolated localhost
Domain31 with a unique mock prefix, explicitly starts the mission and observes
DONE with both agents deposited/exited. Fake node endpoints have no sensors,
velocity publishers or real action clients. Native domain_bridge parser validation
checks all five one-way routes per PC, including effective reversed directions,
without ROS initialization or a bridge instance. These are not cross-PC tests.

Independent read-only whole-branch review found two Important issues, both
reproduced by new failing tests before correction in `d5c9680`:

- Retired-task SYNC could reset sequence history and revive canceled execution.
  Retired task IDs are now fenced before adopting any different task.
- First executing-path publication could arrive after the next supervision tick.
  Startup now allows a bounded wait only with confirmed physical stop and safe
  stationary envelope. Movement before the first path, expiry, or later path
  loss still stops. A preflight ComputePath never counts as an executing path.

The final 49-case run includes these regressions. No additional confirmed
Critical/Important issue was reported in that review; physical behavior and real
path-source association were outside its hardware-free scope.

## Explicitly unexecuted field acceptance

No Ranger access/changes, cross-PC/domain communication, bridge startup, live
cooperative launch, real camera/SDK inference, true Nav2 motion, ARM or hardware
stopping was performed. This record does not reuse prior single-robot acceptance
as evidence for cooperative behavior.

Live templates remain closed: actual independent map IDs/paths/versions;
measured Ranger→lab_ref references and independent checks; real viewpoints,
wait/exit points and door/C polygons; verified executing/replanned path interface;
measured stopping margin; camera/depth/ABC and peer-body obstacle validation.
There was no verified executing-path source in the existing stack, so its topic
is deliberately null rather than guessed.

Ranger hardware adapter must be implemented and validated on Ranger PC using
[ranger_handoff.md](ranger_handoff.md). Subsequently validate new message routing,
Domain0 NIC isolation and no recirculation, both-body dynamic obstacle coverage,
cancel/physical stop, door contention and full footprint exits, then the ordered
same-C two-robot mission. Coordinator/Go2 remain Domain0=enp4s0; pending shared
Domain40 uses wlo1. Domain31 is mock only and has no bridge route.
