#!/usr/bin/env bash

# Shared ROS environment for both Ranger and Go2 profiles.
# Preserve the established Ranger overlay order relative to the current
# account.  VLM_NAV_RANGER_HOME is an explicit escape hatch for deployments
# whose Ranger workspaces live under a different home directory.
set +u

source /opt/ros/humble/setup.bash

common_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
vlm_nav_source_dir="$(cd "${common_script_dir}/.." && pwd)"
vlm_nav_workspace_dir="$(cd "${vlm_nav_source_dir}/.." && pwd)"
ranger_home="${VLM_NAV_RANGER_HOME:-${HOME}}"
setup_files=(
  "${ranger_home}/ros2_ws/install/setup.bash"
  "${ranger_home}/rs515/ros2_ws/install/setup.bash"
  "${ranger_home}/agilex_ws/install/setup.bash"
  "${ranger_home}/ws/install_fastlio/setup.bash"
  "${vlm_nav_workspace_dir}/install/setup.bash"
  "${vlm_nav_source_dir}/install/setup.bash"
)
if [[ -n "${VLM_NAV_EXTRA_SETUP_FILES:-}" ]]; then
  IFS=':' read -r -a extra_setup_files <<<"${VLM_NAV_EXTRA_SETUP_FILES}"
  setup_files+=("${extra_setup_files[@]}")
fi

for setup_file in "${setup_files[@]}"; do
  if [[ -f "$setup_file" ]]; then
    source "$setup_file"
  fi
done

export RCUTILS_COLORIZED_OUTPUT=1
unset common_script_dir vlm_nav_source_dir vlm_nav_workspace_dir ranger_home setup_files setup_file
set -u
