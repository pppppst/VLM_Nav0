#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/go2_ws/install/setup.bash"
source "${workspace_dir}/install/setup.bash"
source "${script_dir}/go2_network_env.sh"
set -u

exec ros2 run multi_agent_comm go2_agent_node
