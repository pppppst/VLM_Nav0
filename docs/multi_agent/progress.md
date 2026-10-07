# Implementation ledger — go2_codex_implementation_plan.md

Task 0: repository, branch, ROS and old consumers verified; original scripts preserved.
Ruling: use an in-place work branch rather than a worktree — existing launch scripts rely on absolute paths.
Pre-flight: DTOs/codec feed agent+coordinator; adapter outputs bind task/command/goal; resource feedback binds original permit command.
Ruling: EXIT command carries parent_command_id of the APPROACH_DEPOSIT permit owner and the same resource token; no regrant between deposit and exit.
Ruling: legacy install must precede go2_ws in test source order — stale legacy Sport binary shadows current code otherwise. No existing overlay is rebuilt in place.
Baseline: 273 passed in 8.87s on original VLM_nav_go, isolated Domain31. Evidence /tmp/go2-coop/evidence/baseline-verified.log.
Ruling: source deduplication did not reorder AMENT_PREFIX_PATH; explicitly prepend go2_ws/install/vlm_nav for baseline. Cooperative install will be prepended after build.
Task 1: protocol/config tests RED (missing package) → GREEN 2 passed. Codec round-trip deferred until generated ROS interfaces are built in Task 7.
