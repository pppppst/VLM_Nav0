#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "${script_dir}/common.sh" ]]; then
  script_dir="$(ros2 pkg prefix --share vlm_nav)/scripts"
fi
unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"
export ROS_LOG_DIR=/tmp/stopgo_roslog
mkdir -p "${ROS_LOG_DIR}"

nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
if grep -Fxq /vlm_nav <<<"${nodes}"; then
  timeout 5 ros2 param set /vlm_nav enabled false
fi

timeout 5 ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}" \
  >/dev/null 2>&1 || true

if ! grep -Fxq /twist_to_go2_sport_bridge <<<"${nodes}"; then
  echo "Bridge未运行；无需DISARM。"
  exit 0
fi
timeout 5 ros2 service call /twist_to_go2_sport_bridge/disarm \
  std_srvs/srv/Trigger "{}"

for _ in {1..10}; do
  bridge_state="$(timeout 2 ros2 topic echo /vlm_nav/go2_bridge_state \
    --once --field data 2>/dev/null || true)"
  control_armed="$(timeout 2 ros2 topic echo /vlm_nav/control_armed \
    --once --field data 2>/dev/null || true)"
  if [[ "${bridge_state}" == *DISARMED* && "${control_armed}" == *False* ]]; then
    echo "VLM已禁用，Nav2目标已取消，bridge已DISARM。"
    exit 0
  fi
  sleep 0.2
done
echo "ERROR: DISARM state could not be verified." >&2
exit 1
