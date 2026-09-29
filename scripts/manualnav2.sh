#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "${script_dir}/common.sh" ]]; then
  project_dir="$(ros2 pkg prefix --share vlm_nav)"
  script_dir="${project_dir}/scripts"
else
  project_dir="$(cd "${script_dir}/.." && pwd)"
fi
session_dir="$(mktemp -d /tmp/manualnav2.XXXXXX)"
children=()
child_names=()
bridge_started=false
vlm_mode="${VLM_NAV_MODE:-false}"
vlm_index=-1

group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

cleanup() {
  local pid attempt alive=false
  set +e
  if [[ "${bridge_started}" == true ]]; then
    "${script_dir}/stopnav2.sh" >/dev/null 2>&1
  fi
  for pid in "${children[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null
  done
  for attempt in {1..100}; do
    alive=false
    for pid in "${children[@]}"; do
      group_alive "${pid}" && alive=true
    done
    [[ "${alive}" == false ]] && break
    sleep 0.1
  done
  for pid in "${children[@]}"; do
    if group_alive "${pid}"; then
      kill -TERM -- "-${pid}" 2>/dev/null
    fi
  done
  for attempt in {1..20}; do
    alive=false
    for pid in "${children[@]}"; do
      group_alive "${pid}" && alive=true
    done
    [[ "${alive}" == false ]] && break
    sleep 0.1
  done
  for pid in "${children[@]}"; do
    if group_alive "${pid}"; then
      kill -KILL -- "-${pid}" 2>/dev/null
    fi
    wait "${pid}" 2>/dev/null
  done
  echo "Session logs: ${session_dir}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ ! -t 0 ]]; then
  echo "ERROR: manualnav2.sh requires an interactive terminal." >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "ERROR: RViz requires DISPLAY or WAYLAND_DISPLAY." >&2
  exit 1
fi
exec 9>/tmp/manualnav2.lock
if ! flock -n 9; then
  echo "ERROR: another manualnav2.sh session is running." >&2
  exit 1
fi

start_child() {
  local name="$1"
  shift
  setsid "$@" </dev/null >"${session_dir}/${name}.log" 2>&1 &
  children+=("$!")
  child_names+=("${name}")
  echo "Started ${name}; log: ${session_dir}/${name}.log"
}

require_child() {
  local index="$1"
  if ! kill -0 "${children[index]}" 2>/dev/null; then
    echo "ERROR: ${child_names[index]} exited; log follows:" >&2
    tail -40 "${session_dir}/${child_names[index]}.log" >&2
    exit 1
  fi
}

wait_for_value() {
  local topic="$1" expected="$2" value attempt
  for attempt in {1..120}; do
    value="$(timeout 2 ros2 topic echo "${topic}" --once --field data 2>/dev/null || true)"
    if [[ "${value}" == *"${expected}"* ]]; then
      return
    fi
    require_child 0
    [[ "${bridge_started}" == true ]] && require_child 3
    (( vlm_index >= 0 )) && require_child "${vlm_index}"
    sleep 0.5
  done
  echo "ERROR: timed out waiting for ${topic}=${expected}" >&2
  exit 1
}

wait_for_nav2() {
  local controller planner navigator action attempt
  for attempt in {1..120}; do
    controller="$(timeout 2 ros2 lifecycle get /controller_server 2>/dev/null || true)"
    planner="$(timeout 2 ros2 lifecycle get /planner_server 2>/dev/null || true)"
    navigator="$(timeout 2 ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    action="$(timeout 2 ros2 action info /navigate_to_pose 2>/dev/null || true)"
    if [[ "${controller}" == *"active [3]"* &&
          "${planner}" == *"active [3]"* &&
          "${navigator}" == *"active [3]"* &&
          "${action}" == *"Action servers: 1"* ]]; then
      echo "Nav2 is ACTIVE and /navigate_to_pose is ready."
      return
    fi
    require_child 0
    require_child 1
    sleep 0.5
  done
  echo "ERROR: Nav2 did not become ready; log: ${session_dir}/nav2.log" >&2
  exit 1
}

confirm() {
  local expected="$1" prompt="$2" answer
  read -r -p "${prompt} Type ${expected}: " answer
  if [[ "${answer}" != "${expected}" ]]; then
    echo "Cancelled; bridge remains DISARMED." >&2
    exit 1
  fi
}

publisher_count() {
  local info
  info="$(ros2 topic info "$1" 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"${info}"
}

wait_for_preflight_inputs() {
  local lidar imu
  while true; do
    lidar="$(publisher_count /lidar_points)"
    imu="$(publisher_count /body_imu)"
    if (( lidar == 0 && imu == 0 )); then
      return
    fi
    echo "Preflight requires exclusive sensor ownership; found /lidar_points=${lidar}, /body_imu=${imu}." >&2
    ros2 topic info -v /lidar_points 2>/dev/null || true
    ros2 topic info -v /body_imu 2>/dev/null || true
    confirm RETRY_PREFLIGHT "Stop the existing sensor process in its original terminal, then"
  done
}

unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"
export PYTHONPATH="${project_dir}:${PYTHONPATH:-}"
export ROS_LOG_DIR="${session_dir}/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

wait_for_preflight_inputs
echo "Reusing a fresh formal preflight report, or running the 30-second preflight..."
"${script_dir}/run_go2_preflight.sh"

start_child nav2 ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:=/tmp/go2_costmap_preflight.json \
  target_stage:=nav2 bridge_dry_run:=false start_bridge:=false
start_child rviz "${script_dir}/start_rviz.sh" \
  "${project_dir}/config/go2_costmaps.rviz"
start_child bag "${script_dir}/record_go2_nav2_manual.sh"

wait_for_nav2

confirm START_NAV "确认 RViz 地图和 footprint 已显示，已用遥控器将 Go2 移到起点、机器人已静止且遥控输入已停止。"

traffic="${session_dir}/sport_requests_before_bridge.txt"
timeout 2 ros2 topic echo /api/sport/request unitree_api/msg/Request \
  --qos-reliability best_effort >"${traffic}" 2>/dev/null || true
if [[ -s "${traffic}" ]]; then
  echo "ERROR: Sport traffic observed; bridge was not started. Evidence: ${traffic}" >&2
  exit 1
fi

start_child bridge ros2 run vlm_nav twist_to_go2_sport_bridge --ros-args \
  --params-file "${project_dir}/config/go2_bridge.yaml" -p dry_run:=false
bridge_started=true
wait_for_value /vlm_nav/go2_bridge_state DISARMED
wait_for_value /vlm_nav/system_ready True
wait_for_value /vlm_nav/nav_ready True
wait_for_value /vlm_nav/control_armed False
"${script_dir}/inspect_go2_sport_publishers.sh" 2

if [[ "${vlm_mode}" == true ]]; then
  target_description="${VLM_TARGET_DESCRIPTION:-}"
  if [[ -z "${target_description}" ]]; then
    echo "ERROR: VLM_TARGET_DESCRIPTION is required in VLM mode." >&2
    exit 1
  fi
  start_child camera_tf ros2 run tf2_ros static_transform_publisher \
    --x 0.32715 --y -0.00003 --z 0.04297 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id camera_link
  start_child vlm ros2 launch vlm_nav go2_vlm.launch.py \
    enabled:=false target_description:="${target_description}"
  vlm_index=$((${#children[@]} - 1))
  wait_for_value /vlm_nav/input_ready True
  wait_for_value /vlm_nav/vlm_api_ready True
  wait_for_value /vlm_nav/autonomy_ready True
  if [[ "$(timeout 5 ros2 param get /vlm_nav easy_case_mode 2>/dev/null || true)" != *"False"* ]]; then
    echo "ERROR: easy_case_mode must remain false for formal VLM navigation." >&2
    exit 1
  fi
  echo "VLM input/API/autonomy gates are ready; VLM remains disabled."
fi

confirm ARM "Confirm the physical emergency stop is ready, the area is clear, and no other Sport controller is active."
arm_result="$(timeout 10 ros2 service call /twist_to_go2_sport_bridge/arm std_srvs/srv/Trigger "{}")"
echo "${arm_result}"
if [[ "${arm_result}" != *"success=True"* || "${arm_result}" != *"message='ARMED'"* ]]; then
  echo "ERROR: ARM failed." >&2
  exit 1
fi
wait_for_value /vlm_nav/go2_bridge_state ARMED
wait_for_value /vlm_nav/control_armed True

if [[ "${vlm_mode}" == true ]]; then
  wait_for_value /vlm_nav/system_ready True
  wait_for_value /vlm_nav/nav_ready True
  wait_for_value /vlm_nav/vlm_api_ready True
  wait_for_value /vlm_nav/input_ready True
  wait_for_value /vlm_nav/autonomy_ready True
  confirm ENABLE_VLM "Confirm the RViz camera preview, maps, path area, and target description are correct."
  enable_result="$(timeout 10 ros2 param set /vlm_nav enabled true)"
  echo "${enable_result}"
  if [[ "${enable_result}" != *"Set parameter successful"* ]]; then
    echo "ERROR: VLM enable failed." >&2
    exit 1
  fi
  wait_for_value /vlm_nav/vlm_enabled True
  echo "VLM navigation enabled for target: ${target_description}"
  echo "Stop from another terminal: ${script_dir}/stopgo.sh"
  while true; do
    bridge_state="$(timeout 2 ros2 topic echo /vlm_nav/go2_bridge_state --once --field data 2>/dev/null || true)"
    if [[ "${bridge_state}" == *"DISARMED"* ]]; then
      bridge_started=false
      break
    fi
    require_child 0
    require_child 3
    require_child "${vlm_index}"
    sleep 0.5
  done
  exit 0
fi

echo "ARMED: use RViz 2D Goal Pose for repeated tests."
echo "Enter CANCEL to cancel the active goal; enter DISARM after all tests."
echo "Emergency stop from another terminal: ${script_dir}/stopnav2.sh"
while read -r -p "manualnav2> " command; do
  case "${command}" in
    CANCEL)
      ros2 service call /navigate_to_pose/_action/cancel_goal \
        action_msgs/srv/CancelGoal \
        "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
      echo "Wait for CANCELED and a complete stop before selecting the next goal."
      ;;
    DISARM)
      "${script_dir}/stopnav2.sh"
      wait_for_value /vlm_nav/go2_bridge_state DISARMED
      wait_for_value /vlm_nav/control_armed False
      bridge_started=false
      break
      ;;
    *) echo "Enter CANCEL or DISARM." ;;
  esac
done
