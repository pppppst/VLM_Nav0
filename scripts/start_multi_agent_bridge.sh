#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

require_address() {
  local interface="$1" prefix="$2"
  if ! ip link show dev "$interface" >/dev/null 2>&1; then
    echo "ERROR: required interface ${interface} does not exist" >&2
    exit 1
  fi
  if ! ip -o -4 address show dev "$interface" 2>/dev/null \
    | awk -v prefix="$prefix" '$4 ~ ("^" prefix "[0-9]+/") {found=1} END {exit !found}'; then
    echo "ERROR: ${interface} has no ${prefix}x IPv4 address" >&2
    exit 1
  fi
}

require_address enp4s0 '192\.168\.123\.'
require_address wlo1 '192\.168\.1\.'

if command -v iw >/dev/null 2>&1; then
  if power_save="$(iw dev wlo1 get power_save 2>&1)"; then
    if grep -qi 'Power save: on' <<<"$power_save"; then
      echo "WARNING: wlo1 Wi-Fi power saving is enabled; Domain 40 communication may be unstable" >&2
    fi
  else
    echo "WARNING: unable to check wlo1 Wi-Fi power saving: ${power_save}" >&2
  fi
else
  echo "WARNING: iw is unavailable; cannot check wlo1 Wi-Fi power saving" >&2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/go2_ws/install/setup.bash"
source "${workspace_dir}/install/setup.bash"
set -u

unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
unset GO2_NET_IFACE ROS_DISCOVERY_SERVER ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${script_dir}/../config/cyclonedds_go2_multi_agent_bridge.xml"

exec ros2 run domain_bridge domain_bridge "${script_dir}/../config/multi_agent_bridge.yaml"
