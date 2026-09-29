#!/usr/bin/env bash
set -eo pipefail
set +u

unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/go2_ws/install/setup.bash
source /home/isee-pst/unitree_ros2/VLM_Nav/scripts/go2_network_env.sh

export ROS_LOG_DIR=/tmp/go2_nav2_manual_roslog
mkdir -p "$ROS_LOG_DIR"
output="/tmp/go2_nav2_manual_$(date +%Y%m%d_%H%M%S)_${BASHPID}"
echo "Recording Nav2 evidence to: ${output}"
echo "Press Ctrl+C only after DISARM to stop recording."

exec ros2 bag record --output "$output" \
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
  /vlm_nav/control_armed
