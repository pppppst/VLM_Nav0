#!/usr/bin/env bash
# Run only the camera publisher on the Go2 upper computer; keep this terminal open.
set -euo pipefail
exec ssh -tt -o ConnectTimeout=5 unitree@192.168.123.18 \
  'bash -lc "source /home/unitree/unitree_ros2/setup.sh &&
    source /home/unitree/ros2_ws/install/setup.bash &&
    export ROS_LOG_DIR=/tmp/realsense_roslog &&
    mkdir -p /tmp/realsense_roslog &&
    exec ros2 launch realsense2_camera rs_launch.py \
      depth_module.depth_profile:=640x480x15 \
      rgb_camera.color_profile:=640x480x15 \
      enable_sync:=true \
      align_depth.enable:=false"'
