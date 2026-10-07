#!/usr/bin/env bash
set -euo pipefail
coop_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
coop_output="${GO2_COOP_OUTPUT:-/tmp/go2-coop}"
coop_python="${GO2_COOP_PYTHON:-/home/isee-pst/venv/co-nav-real/bin/python}"
set +u
source /opt/ros/humble/setup.bash
source "${coop_repo}/../go2_ws/install/setup.bash"
source "${coop_output}/install/setup.bash"
set -u
export AMENT_PREFIX_PATH="${coop_output}/install/vlm_nav:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${coop_repo}:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOG_DIR="${coop_output}/evidence/ros-tests"
unset CYCLONEDDS_URI
mkdir -p "${coop_output}/evidence"
cd "${coop_repo}"
"${coop_python}" -m pytest -q test | tee "${coop_output}/evidence/baseline-final.log"
ctest --test-dir "${coop_output}/build/vlm_nav" \
  -R '^(test_obstacle_cloud_filter|test_go2_bridge_state|test_componentwise_min_speed)$' \
  --output-on-failure | tee "${coop_output}/evidence/baseline-cpp.log"
"${coop_python}" -m pytest -q cooperative_ws/src/multi_agent_nav/tests | tee "${coop_output}/evidence/cooperative.log"
colcon --log-base "${coop_output}/log" test --base-paths "${coop_repo}" "${coop_repo}/cooperative_ws/src" \
  --build-base "${coop_output}/build" --install-base "${coop_output}/install" \
  --packages-select multi_agent_task_msgs multi_agent_nav --return-code-on-test-failure \
  | tee "${coop_output}/evidence/colcon-test.log"
colcon test-result --test-result-base "${coop_output}/build" --all --verbose \
  | tee "${coop_output}/evidence/test-result.log"
