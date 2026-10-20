#!/usr/bin/env bash
# Source in each Go2 control terminal; only loads environment, never starts ROS nodes.
if [[ -z "${BASH_VERSION:-}" || "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 Bash 的 source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash" >&2
  return 1 2>/dev/null || exit 1
fi

_covlm_load_environment() {
  export REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  export GO2_COOP_OUTPUT="$REPO/build/go2-coop"
  export GO2_EXPLORE_OUTPUT="$REPO/build/go2-approach-coverage-20261018"
  export GO2_GENERATED_OUTPUT="$REPO/build/go2-generated-candidates-20261013"
  export GO2_RGBD_OUTPUT="$REPO/build/go2-rgbd-pair-health-20261013"
  export GO2_BRIDGE_OUTPUT="$REPO/build/go2-manual-takeover-20261015"
  export GO2_CAMERA_OUTPUT="$REPO/build/go2-camera-tf"
  export GO2_FILTER_OUTPUT="$REPO/build/go2-intensity-default-20261012"
  export GO2_EXPLORE_CONFIG="$REPO/cooperative_ws/src/multi_agent_nav/config"
  local setup
  local setups=(
    /opt/ros/humble/setup.bash
    /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
    /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
    "$GO2_COOP_OUTPUT/install/local_setup.bash"
    "$GO2_CAMERA_OUTPUT/install/local_setup.bash"
    "$GO2_FILTER_OUTPUT/install/local_setup.bash"
    "$GO2_GENERATED_OUTPUT/install/local_setup.bash"
    "$GO2_EXPLORE_OUTPUT/install/local_setup.bash"
    "$GO2_RGBD_OUTPUT/install/local_setup.bash"
    "$GO2_BRIDGE_OUTPUT/install/local_setup.bash"
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
  export VLM_NAV_EXTRA_SETUP_FILES="$GO2_CAMERA_OUTPUT/install/local_setup.bash:$GO2_FILTER_OUTPUT/install/local_setup.bash:$GO2_GENERATED_OUTPUT/install/local_setup.bash:$GO2_EXPLORE_OUTPUT/install/local_setup.bash:$GO2_RGBD_OUTPUT/install/local_setup.bash:$GO2_BRIDGE_OUTPUT/install/local_setup.bash"
  export ROS_LOG_DIR="$GO2_EXPLORE_OUTPUT/ros-logs"
  mkdir -p "$ROS_LOG_DIR" || return 1
  cd "$REPO" || return 1
}

if _covlm_load_environment; then
  unset -f _covlm_load_environment
else
  unset -f _covlm_load_environment
  return 1
fi
