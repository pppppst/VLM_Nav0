# Ranger PC handoff (no Ranger connection performed)

Transfer the commits on `go2-cooperative-task-layer` through the final commit
listed in the delivery/test record. Keep `ranger_go2_codex_plan.md` and the
implementation plan beside the code. Use identical `multi_agent_task_msgs` v1
and shared `multi_agent_nav`; keep old `multi_agent_msgs` and
`/multi_agent/agents/{go2,ranger}/state` consumers unchanged.

## Public protocol

Five reliable, volatile topics:

| Topic | Type | depth |
|---|---|---|
| /multi_agent/go2/state, /multi_agent/ranger/state | multi_agent_task_msgs/msg/AgentState | 10 |
| /multi_agent/go2/observation, /multi_agent/ranger/observation | multi_agent_task_msgs/msg/Observation | 50 |
| /multi_agent/go2/command | multi_agent_task_msgs/msg/Command | 10 |

Full fields are in generated interfaces and `models.py`; all poses on these
topics are common lab_ref, with local_map_id/map_version/alignment_id explicitly
identifying the source. Image/stamp fields are trace-only; heartbeat expiry
uses receiver monotonic time. `simulation=false` is mandatory for live peers.

AgentState persists READY/deposited, current command identity and terminal reply,
resource request/occupancy/exit. Coordinator SYNC contains ack_sequence and phase.
Command actions: EXECUTE, SYNC, PAUSE, CANCEL, GRANT_RESOURCE. EXECUTE sequence
is per robot/task and idempotent. CANCEL targets cancel_command_id. Resource
GRANT binds parent_command_id; kind=RELEASE acknowledges a validated exit.
EXIT retains the **deposit command** parent identity and token, although it is
a new EXECUTE command. Resource grant must never replace the active main task.

Observation emits A/B/C individually per view, with DETECTED, NOT_DETECTED,
UNCERTAIN or API_ERROR; invalid depth retains DETECTED but position_valid=false.
Object common positions are not transferable parking points: select a local
approach using Ranger map/costmap and ComputePath before navigating.

## Ranger-specific work

Implement `adapters/ranger.py` against `adapters/base.py` and wire only the Ranger
live branch of AgentNode on that PC. This round supplies no guessed Ranger SDK,
camera topics, footprint, map path or hardware adapter.

Adapter operations: get_health; asynchronous compute_path and select_approach;
start_navigation returning goal_id; cancel_navigation; start_scan; stop_motion.
Use the same Future/result-queue contract as Go2/Fake and AgentRunner. Results
must bind task_id/command_id/goal_id/goal_kind. Health reports local map pose,
fresh localization/path/footprint, stationary confirmation and action terminal.
Scan checks all ABC, serializes Spin and navigation, and keeps inference off
the ROS heartbeat thread. Do not copy Go2 sensor-height/footprint assumptions.

Deploy Ranger agent on Ranger **Domain0**; no coordinator on Ranger. Build/source
Ranger's existing ROS overlay, generated interfaces and the common task package
using a locally verified CPython ABI/runtime. Go2's absolute venv path is not a
Ranger deployment requirement; configure COOPERATIVE_RUNTIME_PYTHON locally.
The common package's vlm_nav dependency presently carries Go2's vision utilities;
keep the existing shared vlm_nav package available, without starting its Go2 nodes.

Both PCs use separate occupancy maps. Fit common_from_ranger from ≥3 separated
non-collinear fixed references plus held-out references; set actual map versions
and the same alignment_id. Rebuilding a map invalidates the previous alignment.
No task-time calibration changes; missing/invalid configuration blocks live start.
Each robot configuration now includes `map_digest`: SHA256 of concatenated YAML
and referenced image SHA256 hex strings. Use `map_file_info` or the offline Go2
check entry to calculate it; verify each PC's own actual files and active map locally.
The Go2 PC does not need a copied Ranger map file. Remote map identity and digest
must still be filled consistently; unknown values remain unset.

## Pending communication setup

Use `config/bridge_ranger_pending.yaml` on Ranger and
`config/bridge_go2_pending.yaml` on Go2. Ranger exports only its state/observation
0→40 and imports Go2 state/observation/command 40→0. Go2 does the complementary
half. Both local domains are 0; robot identity comes from PC/topic source.
Keep Go2 bridge XML Domain0=enp4s0, Domain40=wlo1; supply Ranger's actual local NIC
isolation separately. Never enable bidirectional task routes or bridge Domain31.

Before activation, install/source custom interfaces in both bridge environments.
No connection, SSH, packets, bridge startup or Ranger code changes were performed
from the Go2 implementation session.

## Subsequent field acceptance

Validate Ranger local projection, approach/cancel/scan and executing-path monitor;
verify map alignment in both rooms/door; verify new task message routes/QoS,
Domain0 isolation and no recirculation; verify peer-body marking/clearing and
stopping; test door FIFO and whole-body exit, then the same-C full ordered mission.
These are separate from synthetic tests and prior single-robot acceptance.
