#!/usr/bin/env bash
set -euo pipefail

if (( $# != 2 )); then
  echo "Usage: $0 <run-directory> <amcl-scan-topic>" >&2
  exit 2
fi
run_dir="$1"
amcl_scan_topic="$2"
if [[ -z "${amcl_scan_topic}" || "${amcl_scan_topic}" != /* ]]; then
  echo "ERROR: amcl_scan_topic must be an absolute ROS topic" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"

export ROS_LOG_DIR="${run_dir}/logs/rosbag_ros"
output="${run_dir}/rosbag/go2_nav2"
qos_source="${script_dir}/../config/go2_nav2_rosbag_qos.yaml"
qos_overrides="${run_dir}/rosbag/qos_overrides.yaml"
mkdir -p "${ROS_LOG_DIR}" "${run_dir}/rosbag"
cp "${qos_source}" "${qos_overrides}"
echo "Recording Nav2 evidence to: ${output}"
echo "Press Ctrl+C only after DISARM to stop recording."

topics=(
  /cmd_vel_nav \
  /cmd_vel_bridge \
  /api/sport/request \
  /api/sport/response \
  /odometry \
  /sportmodestate \
  /clicked_point \
  /goal_pose \
  /plan \
  /local_plan \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /navigate_to_pose/_action/status \
  /vlm_nav/system_ready \
  /vlm_nav/nav_ready \
  /vlm_nav/go2_bridge_state \
  /vlm_nav/control_armed \
  /tf \
  /tf_static \
  /scan \
  /map \
  /amcl_pose \
  /particle_cloud \
  /initialpose \
  /vlm_nav/diagnostics \
  /vlm_nav/state \
  /vlm_nav/vlm_enabled \
  /vlm_nav/markers \
  /vlm_nav/output_text
)
if [[ "${amcl_scan_topic}" != /scan ]]; then
  topics+=("${amcl_scan_topic}")
fi

exec ros2 bag record --output "${output}" \
  --qos-profile-overrides-path "${qos_overrides}" \
  "${topics[@]}"
