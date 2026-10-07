# Implementation ledger — go2_codex_implementation_plan.md

Task 0: repository, branch, ROS and old consumers verified; original scripts preserved.
Ruling: use an in-place work branch rather than a worktree — existing launch scripts rely on absolute paths.
Pre-flight: DTOs/codec feed agent+coordinator; adapter outputs bind task/command/goal; resource feedback binds original permit command.
Ruling: EXIT command carries parent_command_id of the APPROACH_DEPOSIT permit owner and the same resource token; no regrant between deposit and exit.
Ruling: legacy install must precede go2_ws in test source order — stale legacy Sport binary shadows current code otherwise. No existing overlay is rebuilt in place.
Baseline: 273 passed in 8.87s on original VLM_nav_go, isolated Domain31. Evidence /tmp/go2-coop/evidence/baseline-verified.log.
Ruling: source deduplication did not reorder AMENT_PREFIX_PATH; explicitly prepend go2_ws/install/vlm_nav for baseline. Cooperative install will be prepended after build.
Task 1: protocol/config tests RED (missing package) → GREEN 2 passed. Codec round-trip deferred until generated ROS interfaces are built in Task 7.
Task 2: SE(2), held-out rigid fit, degeneracy/reflection tests RED (missing module) → GREEN; total 8 passed. No field transform generated.
Task 3: agent/fake tests RED → GREEN; current goal identity, duplicates, internal goals, pause/stationary confirmation and fake cancel covered. Region integration follows Task 5.
Task 4: READY barrier, early C, original permit EXIT identity, order, old READY/mixed mode/clock and restart tests RED → GREEN; 18 passed total.
Ruling: a twice-failed bound C approach pauses for a new task instead of silently selecting another basket. Keeps C identity stable; field operator may need to reposition before restarting.
Task 5: FIFO/token/entry/exit/freeze and swept footprint/replan guard tests RED → GREEN; total 22 passed.
Ruling: swept path uses conservative circumcircle envelope, with a ponytail comment; live stopping margin is mandatory and unset until measured. May restrict narrow clearances, never silently shrinks footprint.
Task 6: Go2 adapter ABC parsing, invalid depth, timeout/cancel, pending action cancellation and passive observer tests passed (5). Actual motion and camera not executed.
Ruling: add opt-in observation_only mode to VLMNavigator, reusing existing raw-depth synchronization/projection without its autonomy timers; existing default entry behavior unchanged.
Task 7: installed interfaces/nodes, dual-fake ROS DONE, bridge parser and launch-description checks passed. 47 cooperative pytest cases; ament contains 47 pytest + 1 parser + 2 CTest wrapper entries (50 reported), no skip/error/failure. Installed executables use co-nav-real.
Ruling: use system Python3.10 for ament/rosidl build and same-ABI co-nav-real only for entrypoints/tests — avoids setuptools/packaging conflict without upgrading either environment.
Safety regressions: resource restart fencing, phase-before-deposit, door-before-basket, delayed odometry and missing footprint all reproduced before fixes and passed afterward.
Task 8: completed within the authorized local scope. Final baseline 273 Python cases
and 21 gtest cases (3 CTest executables) passed. New cooperative pytest 49 cases,
including a three-process isolated ROS mission, and native parser-only route check passed.
colcon test-result reports 73 entries: 49 pytest + 1 parser + 2 wrapper + 21 baseline gtest;
it excludes the separately recorded 273 baseline Python cases and is not 73 distinct new cases.
Independent whole-branch review found two Important defects; both were reproduced RED
and fixed GREEN: retired-task SYNC revival and executing-path startup latency.
Ruling: first-path waiting is bounded by robot path_start_timeout, only while physically
stopped with a permitted stationary stopping envelope; movement or later path loss pauses.
Code fix commit d5c9680; final evidence, commands and unexecuted field acceptance
are in test_results.md, go2_runbook.md and ranger_handoff.md.
Original user input files, old scripts, legacy messages/routes and existing overlays remain preserved.
No Ranger access, bridge activation, live task launch or real motion was performed.

Field preparation (2026-10-07), baseline branch head 1ba2589: added independent
Go2 check entry (default passive, offline without ROS); map YAML/image digests and
per-PC file checks; real raw Costmap footprint/TF/stamp filtering; separate linear
and angular stationary thresholds; stricter finite field data and unchanged closed
cooperative gates. Existing AMCL overlay drops dynamic layers; executing path
exporter remains unverified. Neither issue was hidden with synthetic field values.
Review regressions fixed unsnapped footprint coverage, pre-send revalidation and
sole independent motion ownership; cleanup errors preserve stop reconciliation.
Final rebuilt evidence: 273 original Python + 21 gtest baseline; 78 cooperative
pytest (49 existing + 29 field preparation); parser-only 1; colcon 102 entries,
zero errors/failures/skips. Separate new evidence and unexecuted field checklist:
field_preparation_results.md and go2_field_checklist.md. No Ranger, bridge,
live Domain0 participant or hardware motion was used.
