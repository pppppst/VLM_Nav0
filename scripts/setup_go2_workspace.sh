#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${project_dir}/.." && pwd)"
go2_build_dir="${workspace_dir}/go2_ws"
manifest="${project_dir}/go2_dependencies.repos"

required_apt_packages=(
  ros-humble-rosidl-generator-dds-idl
  ros-humble-diagnostic-updater
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-slam-toolbox
  ros-humble-pointcloud-to-laserscan
  ros-humble-realsense2-camera
)
missing_apt_packages=()
for package in "${required_apt_packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null \
    | grep -q 'install ok installed'; then
    missing_apt_packages+=("$package")
  fi
done
if (( ${#missing_apt_packages[@]} > 0 )); then
  echo "ERROR: install the missing system dependencies in an interactive terminal:" >&2
  printf '  sudo apt-get install -y' >&2
  printf ' %q' "${missing_apt_packages[@]}" >&2
  printf '\n' >&2
  exit 1
fi

command -v vcs >/dev/null || {
  echo "ERROR: vcs is required (python3-vcstool)" >&2
  exit 1
}
mkdir -p "${go2_build_dir}/src"
vcs import --recursive "${go2_build_dir}/src" <"${manifest}"

unitree_expected="0dfa8f2e444713c52c96c7b70c433b2609879a31"
spark_expected="17b36d293a14df37d57e1751a337a32e2f164692"
[[ "$(git -C "${go2_build_dir}/src/unitree_ros2" rev-parse HEAD)" == "$unitree_expected" ]]
[[ "$(git -C "${go2_build_dir}/src/spark-fast-lio" rev-parse HEAD)" == "$spark_expected" ]]

set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "${go2_build_dir}/log" build \
  --base-paths "${project_dir}" "${go2_build_dir}/src" \
  --build-base "${go2_build_dir}/build" \
  --install-base "${go2_build_dir}/install" \
  --symlink-install \
  --packages-up-to vlm_nav
