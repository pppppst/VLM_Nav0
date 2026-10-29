#!/usr/bin/env bash
# Source in each Go2 control terminal; only loads environment, never starts ROS nodes.
if [[ -z "${BASH_VERSION:-}" || "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 Bash 的 source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash" >&2
  return 1 2>/dev/null || exit 1
fi

_covlm_load_environment() {
  export REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  export GO2_COOP_OUTPUT="${GO2_COOP_OUTPUT:-/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-coop}"
  export COOP_SETUP_FILE="${COOP_SETUP_FILE:-$REPO/build/go2-local-scan-20261029/overlay/install/local_setup.bash}"
  [[ "$COOP_SETUP_FILE" == /* ]] || { echo 'COOP_SETUP_FILE 必须是绝对路径' >&2; return 1; }
  export GO2_EXPLORE_CONFIG="${GO2_CONFIG:-$REPO/cooperative_ws/src/multi_agent_nav/config}"
  local setup
  local setups=(
    /opt/ros/humble/setup.bash
    /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
    /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
    "$GO2_COOP_OUTPUT/install/local_setup.bash"
    "$COOP_SETUP_FILE"
    "$REPO/scripts/go2_network_env.sh"
  )
  for setup in "${setups[@]}"; do
    if [[ ! -r "$setup" ]]; then
      echo "ERROR: environment file missing: $setup" >&2
      return 1
    fi
  done
  unset PYTHONHOME PYTHONPATH
  export GO2_NET_IFACE=enp4s0
  for setup in "${setups[@]}"; do
    source "$setup" || return 1
  done
  export VLM_NAV_EXTRA_SETUP_FILES="$GO2_COOP_OUTPUT/install/local_setup.bash:$COOP_SETUP_FILE"
  export ROS_LOG_DIR="$REPO/build/go2-local-scan-20261029/ros-logs"
  mkdir -p "$ROS_LOG_DIR" || return 1
  cd "$REPO" || return 1
}

if _covlm_load_environment; then
  unset -f _covlm_load_environment
else
  unset -f _covlm_load_environment
  return 1
fi
