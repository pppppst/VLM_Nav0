#!/usr/bin/env bash
set -eo pipefail
set +u

if [[ "${1:-}" != "--execute" || $# -ne 1 ]]; then
  echo "Usage: $0 --execute" >&2
  echo "REAL MOTION: forward/backward 0.40 m/s, then CCW/CW 0.50 rad/s." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"
export ROS_LOG_DIR=/tmp/go2_gate5_movetest_roslog
mkdir -p "$ROS_LOG_DIR"

topic_value() {
  timeout 5 ros2 topic echo "$1" --once --field data 2>/dev/null || true
}

require_disarmed_ready() {
  local ready state armed traffic
  ready="$(topic_value /vlm_nav/system_ready)"
  state="$(topic_value /vlm_nav/go2_bridge_state)"
  armed="$(topic_value /vlm_nav/control_armed)"
  if [[ "$ready" != *True* || "$state" != *DISARMED* || "$armed" != *False* ]]; then
    echo "ERROR: require SYSTEM_READY=True, bridge=DISARMED, control_armed=False" >&2
    echo "observed: ready=${ready:-missing} state=${state:-missing} armed=${armed:-missing}" >&2
    echo "movetest.sh is not a bringup script; use manualnav2.sh for the complete Nav2 workflow." >&2
    exit 1
  fi
  traffic="$(timeout 2 ros2 topic echo /api/sport/request \
    unitree_api/msg/Request --qos-reliability best_effort 2>/dev/null || true)"
  if [[ -n "$traffic" ]]; then
    echo "ERROR: external Sport traffic observed during the 2 second quiet window" >&2
    exit 1
  fi
}

confirm() {
  local expected="$1" prompt="$2" answer
  read -r -p "$prompt Type ${expected} to continue: " answer
  if [[ "$answer" != "$expected" ]]; then
    echo "Cancelled; no motion command sent."
    exit 1
  fi
}

require_disarmed_ready
confirm MOVE "REAL MOTION: forward 0.40 m/s for 6 s, stop 1 s, backward 6 s."
ros2 run vlm_nav go2_gate5_pulse \
  --sequence linear \
  --speed 0.40 \
  --duration 6 \
  --execute-extended

require_disarmed_ready
confirm ROTATE "REAL MOTION: CCW 0.50 rad/s for 10 s, stop 3 s, CW 10 s."
ros2 run vlm_nav go2_gate5_pulse \
  --sequence yaw \
  --yaw-rate 0.50 \
  --duration 10 \
  --settle-duration 3 \
  --execute-extended

require_disarmed_ready
echo "COMPLETE: Gate 5 movement test finished; bridge is DISARMED."
