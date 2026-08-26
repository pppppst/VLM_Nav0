#!/usr/bin/env bash
set -euo pipefail

dry_run=false
output_dir="${PWD}/go2_readonly_evidence"
static_duration=30
dynamic_duration=45

usage() {
  echo "Usage: $0 [--dry-run] [--output-dir DIR] [--static-duration SEC] [--dynamic-duration SEC]"
}

while (($#)); do
  case "$1" in
    --dry-run) dry_run=true; shift ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --static-duration) static_duration="$2"; shift 2 ;;
    --dynamic-duration) dynamic_duration="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for duration in "$static_duration" "$dynamic_duration"; do
  [[ "$duration" =~ ^[1-9][0-9]*$ ]] || {
    echo "Durations must be positive integer seconds" >&2
    exit 2
  }
done

print_command() {
  printf 'DRY-RUN:'
  printf ' %s' "$@"
  printf '\n'
}

capture_text() {
  local destination="$1"
  shift
  if $dry_run; then
    print_command "$@" ">" "$destination"
  else
    "$@" >"$destination" 2>&1 || true
  fi
}

record_lidar_imu() {
  local duration="$1"
  local destination="$2"
  if $dry_run; then
    print_command timeout --signal=INT "$duration" ros2 bag record -o "$destination" /utlidar/cloud /utlidar/imu
  else
    timeout --signal=INT "$duration" ros2 bag record -o "$destination" /utlidar/cloud /utlidar/imu || true
  fi
}

if $dry_run; then
  print_command mkdir -p "$output_dir"
else
  command -v ros2 >/dev/null
  mkdir -p "$output_dir"
fi

capture_text "$output_dir/time_status.txt" timedatectl status
capture_text "$output_dir/lidar_qos.txt" ros2 topic info -v /utlidar/cloud
capture_text "$output_dir/imu_qos.txt" ros2 topic info -v /utlidar/imu
capture_text "$output_dir/camera_color_qos.txt" ros2 topic info -v /camera/camera/color/image_raw
capture_text "$output_dir/camera_depth_qos.txt" ros2 topic info -v /camera/camera/aligned_depth_to_color/image_raw
capture_text "$output_dir/camera_info_qos.txt" ros2 topic info -v /camera/camera/color/camera_info
capture_text "$output_dir/base_lidar_tf.txt" timeout 5 ros2 run tf2_ros tf2_echo base_link utlidar_lidar
capture_text "$output_dir/base_camera_tf.txt" timeout 5 ros2 run tf2_ros tf2_echo base_link camera_link

if ! $dry_run; then
  echo "Keep the Go2 stationary and keep every control bridge DISARMED."
  read -r -p "Press Enter to record the stationary LiDAR/IMU bag... "
fi
record_lidar_imu "$static_duration" "$output_dir/static_lidar_imu"

if ! $dry_run; then
  echo "Keep the bridge DISARMED. An operator may now manually yaw/rock the robot."
  read -r -p "Press Enter when the operator is ready for the dynamic bag... "
fi
record_lidar_imu "$dynamic_duration" "$output_dir/dynamic_lidar_imu"

if $dry_run; then
  print_command sha256sum "$output_dir"/static_lidar_imu/*.db3 "$output_dir"/dynamic_lidar_imu/*.db3
else
  find "$output_dir" -name '*.db3' -type f -print0 | sort -z | xargs -0 -r sha256sum >"$output_dir/sha256sums.txt"
  echo "Read-only evidence saved to $output_dir"
fi
