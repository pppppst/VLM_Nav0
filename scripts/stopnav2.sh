#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "${script_dir}/common.sh" ]]; then
  script_dir="$(ros2 pkg prefix --share vlm_nav)/scripts"
fi
unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"
export ROS_LOG_DIR=/tmp/stopnav2_roslog
mkdir -p "${ROS_LOG_DIR}"

timeout 5 ros2 service call /twist_to_go2_sport_bridge/disarm \
  std_srvs/srv/Trigger "{}"
