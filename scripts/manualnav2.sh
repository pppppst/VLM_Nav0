#!/usr/bin/env bash
set -euo pipefail

localization_mode=slam
map_path=""
amcl_scan_topic=/scan_amcl
for argument in "$@"; do
  case "${argument}" in
    localization_mode:=*) localization_mode="${argument#*:=}" ;;
    map:=*) map_path="${argument#*:=}" ;;
    amcl_scan_topic:=*) amcl_scan_topic="${argument#*:=}" ;;
    *) echo "ERROR: unsupported argument: ${argument}" >&2; exit 2 ;;
  esac
done
if [[ "${localization_mode}" != slam && "${localization_mode}" != amcl ]]; then
  echo "ERROR: localization_mode must be slam or amcl" >&2
  exit 2
fi
if [[ "${localization_mode}" == amcl && ! -f "${map_path}" ]]; then
  echo "ERROR: AMCL localization requires a valid saved map YAML" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "${script_dir}/common.sh" ]]; then
  project_dir="$(ros2 pkg prefix --share vlm_nav)"
  script_dir="${project_dir}/scripts"
else
  project_dir="$(cd "${script_dir}/.." && pwd)"
fi
run_root="${VLM_NAV_RUN_ROOT:-/home/isee-pst/unitree_ros2/log/localization_runs}"
run_dir="${run_root}/$(date +%Y%m%d_%H%M%S_%N)_${BASHPID}"
session_dir="${run_dir}/logs"
evidence_tool="${script_dir}/go2_run_evidence.py"
children=()
child_names=()
bridge_started=false
bootstrap_pid=""
vlm_mode="${VLM_NAV_MODE:-false}"
vlm_index=-1
nav2_index=-1
rviz_index=-1
bag_index=-1
bridge_index=-1
evidence_initialized=false
evidence_finished=false
vlm_artifact_root="${VLM_NAV_ARTIFACT_ROOT:-${HOME}/unitree_ros2/log/VLM_feedback}"

group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

stop_bootstrap() {
  local attempt
  [[ -z "${bootstrap_pid}" ]] && return
  kill -INT -- "-${bootstrap_pid}" 2>/dev/null || true
  for attempt in {1..20}; do
    group_alive "${bootstrap_pid}" || break
    sleep 0.1
  done
  group_alive "${bootstrap_pid}" && kill -TERM -- "-${bootstrap_pid}" 2>/dev/null || true
  wait "${bootstrap_pid}" 2>/dev/null || true
  bootstrap_pid=""
}

cleanup() {
  local status=$? pid attempt alive=false bag_failed=false safe_to_snapshot=true
  trap - EXIT INT TERM
  set +e
  if (( bag_index >= 0 )) && ! kill -0 "${children[bag_index]}" 2>/dev/null; then
    echo "ERROR: evidence collection failed: rosbag recorder exited unexpectedly; see ${session_dir}/bag.log" >&2
    bag_failed=true
  fi
  if [[ "${bridge_started}" == true ]]; then
    if ! "${script_dir}/stopgo.sh"; then
      echo "ERROR: failed to verify VLM disable, goal cancellation, and bridge DISARM." >&2
      status=1
      safe_to_snapshot=false
    else
      bridge_started=false
    fi
  fi
  if [[ "${evidence_initialized}" == true && "${evidence_finished}" == false && \
        "${safe_to_snapshot}" == true ]]; then
    finish_evidence aborted
  elif [[ "${safe_to_snapshot}" == false ]]; then
    echo "WARNING: skipping end parameter snapshot because DISARM was not verified." >&2
  fi
  if (( bag_index >= 0 )) && ! kill -0 "${children[bag_index]}" 2>/dev/null && \
      [[ "${bag_failed}" == false ]]; then
    echo "ERROR: evidence collection failed: rosbag recorder exited before cleanup; see ${session_dir}/bag.log" >&2
    bag_failed=true
  fi
  stop_bootstrap
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
  [[ "${bag_failed}" == true ]] && evidence_finished=false
  if [[ "${evidence_initialized}" == true && "${evidence_finished}" == false ]]; then
    finalize_evidence_manifest aborted
  fi
  echo "Experiment directory: ${run_dir}"
  [[ "${bag_failed}" == true ]] && status=1
  exit "${status}"
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
    if (( index == bag_index )); then
      echo "ERROR: evidence collection failed: rosbag recorder exited unexpectedly." >&2
    fi
    echo "ERROR: ${child_names[index]} exited; log follows:" >&2
    tail -40 "${session_dir}/${child_names[index]}.log" >&2
    exit 1
  fi
}

require_evidence() {
  (( bag_index >= 0 )) && require_child "${bag_index}"
}

require_active_children() {
  (( nav2_index >= 0 )) && require_child "${nav2_index}"
  [[ "${bridge_started}" == true ]] && require_child "${bridge_index}"
  (( vlm_index >= 0 )) && require_child "${vlm_index}"
  require_evidence
}

wait_for_bag_ready() {
  local attempt
  for attempt in {1..100}; do
    grep -Fq "Listening for topics" "${session_dir}/bag.log" 2>/dev/null && return
    require_evidence
    sleep 0.1
  done
  echo "ERROR: evidence collection failed: rosbag recorder did not become ready." >&2
  exit 1
}

snapshot_core_parameters() {
  local phase="$1" critical="$2" per_query_timeout=5 overall_timeout=65
  if [[ "${phase}" == end ]]; then
    per_query_timeout=2
    overall_timeout=25
  fi
  if ! timeout "${overall_timeout}" python3 "${evidence_tool}" snapshot-core \
      --output-dir "${run_dir}/parameters/${phase}/core" \
      --localization-mode "${localization_mode}" --timeout "${per_query_timeout}"; then
    if [[ "${critical}" == true ]]; then
      echo "ERROR: AMCL/map_server runtime parameter snapshot failed before ARM; see ${run_dir}/parameters/${phase}/core" >&2
      return 1
    fi
    echo "WARNING: end core parameter snapshot was incomplete; cleanup will continue." >&2
  fi
}

snapshot_vlm_parameters() {
  local phase="$1" root_file
  if ! timeout 20 python3 "${evidence_tool}" snapshot-vlm \
      --output-dir "${run_dir}/parameters/${phase}/vlm" --timeout 2; then
    echo "WARNING: VLM parameter snapshot was incomplete; see ${run_dir}/parameters/${phase}/vlm" >&2
  fi
  root_file="${run_dir}/parameters/${phase}/vlm/vlm_artifact_root.txt"
  if [[ -s "${root_file}" ]]; then
    vlm_artifact_root="$(<"${root_file}")"
  fi
}

capture_topic_qos() {
  local phase="$1" topic output
  output="${session_dir}/topic_qos_${phase}.txt"
  shift
  : >"${output}"
  for topic in "$@"; do
    printf '=== %s ===\n' "${topic}" >>"${output}"
    timeout 3 ros2 topic info -v "${topic}" >>"${output}" 2>&1 || \
      echo "unavailable at ${phase} snapshot" >>"${output}"
  done
}

finalize_evidence_manifest() {
  local run_status="${1:-finished}" association
  if ! association="$(timeout 10 python3 "${evidence_tool}" finalize \
      --run-dir "${run_dir}" --vlm-artifact-root "${vlm_artifact_root}" \
      --status "${run_status}" 2>&1)"; then
    echo "WARNING: manifest finalization failed: ${association}" >&2
    return 1
  fi
  if [[ "${association}" == linked ]]; then
    echo "VLM artifact directory linked in manifest."
  elif [[ -n "${association}" ]]; then
    echo "VLM artifact association: ${association}; see manifest.json."
  fi
  evidence_finished=true
}

finish_evidence() {
  local run_status="${1:-finished}"
  [[ "${evidence_finished}" == true ]] && return
  if (( nav2_index >= 0 )) && kill -0 "${children[nav2_index]}" 2>/dev/null; then
    snapshot_core_parameters end false
  fi
  if [[ "${vlm_mode}" == true ]] && (( vlm_index >= 0 )) && \
      kill -0 "${children[vlm_index]}" 2>/dev/null; then
    snapshot_vlm_parameters end
  fi
  finalize_evidence_manifest "${run_status}"
}

wait_for_value() {
  local topic="$1" expected="$2" value attempt
  for attempt in {1..120}; do
    value="$(timeout 2 ros2 topic echo "${topic}" --once --field data 2>/dev/null || true)"
    if [[ "${value}" == *"${expected}"* ]]; then
      return
    fi
    require_active_children
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
    require_child "${nav2_index}"
    require_child "${rviz_index}"
    require_evidence
    sleep 0.5
  done
  echo "ERROR: Nav2 did not become ready; log: ${session_dir}/nav2.log" >&2
  exit 1
}

confirm() {
  local expected="$1" prompt="$2" answer read_status
  printf '%s Type %s: ' "${prompt}" "${expected}"
  while true; do
    if read -r -t 1 answer; then
      break
    else
      read_status=$?
    fi
    if (( read_status == 1 )); then
      echo "Cancelled; input closed." >&2
      exit 1
    fi
    require_active_children
  done
  require_active_children
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
install_share="$(timeout 5 ros2 pkg prefix --share vlm_nav)"
nav2_share="$(timeout 5 ros2 pkg prefix --share nav2_bringup)"
source_dir="${VLM_NAV_SOURCE_DIR:-${project_dir}}"
if [[ ! -d "${source_dir}/.git" ]]; then
  source_dir=/home/isee-pst/unitree_ros2/VLM_Nav
fi
init_evidence=(
  python3 "${evidence_tool}" init
  --run-dir "${run_dir}"
  --source-dir "${source_dir}"
  --install-share "${install_share}"
  --nav2-share "${nav2_share}"
  --entry "${VLM_NAV_ENTRY:-${script_dir}/manualnav2.sh}"
  --argument "localization_mode:=${localization_mode}"
  --argument "map:=${map_path}"
  --argument "amcl_scan_topic:=${amcl_scan_topic}"
  --localization-mode "${localization_mode}"
  --amcl-scan-topic "${amcl_scan_topic}"
  --vlm-artifact-root "${vlm_artifact_root}"
)
if [[ -n "${map_path}" ]]; then
  init_evidence+=(--map-yaml "${map_path}")
fi
if [[ "${vlm_mode}" == true && -n "${VLM_TARGET_DESCRIPTION:-}" ]]; then
  init_evidence+=(--argument "target_description:=${VLM_TARGET_DESCRIPTION}")
fi
"${init_evidence[@]}"
evidence_initialized=true
echo "Experiment directory: ${run_dir}"
export ROS_LOG_DIR="${session_dir}/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

wait_for_preflight_inputs
echo "Reusing a fresh formal preflight report, or running the 30-second preflight..."
"${script_dir}/run_go2_preflight.sh"

start_child bag "${script_dir}/record_go2_nav2_manual.sh" \
  "${run_dir}" "${amcl_scan_topic}"
bag_index=$((${#children[@]} - 1))
wait_for_bag_ready

start_child nav2 ros2 launch vlm_nav go2_system.launch.py \
  sensor_preflight_report:=/tmp/go2_costmap_preflight.json \
  target_stage:=nav2 bridge_dry_run:=false start_bridge:=false \
  localization_mode:="${localization_mode}" map:="${map_path}" \
  amcl_scan_topic:="${amcl_scan_topic}"
nav2_index=$((${#children[@]} - 1))
if [[ "${localization_mode}" == amcl ]]; then
  setsid ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id amcl_bootstrap \
    </dev/null >"${session_dir}/amcl_bootstrap.log" 2>&1 &
  bootstrap_pid=$!
  echo "AMCL initialization: use RViz 2D Pose Estimate within 10 minutes."
fi
start_child rviz "${script_dir}/start_rviz.sh" \
  "${project_dir}/config/go2_costmaps.rviz"
rviz_index=$((${#children[@]} - 1))

wait_for_nav2
stop_bootstrap
snapshot_core_parameters start true
capture_topic_qos nav2 \
  /tf /tf_static /scan "${amcl_scan_topic}" /map /amcl_pose /particle_cloud /initialpose

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
bridge_index=$((${#children[@]} - 1))
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
    enabled:=false target_description:="${target_description}" \
    localization_mode:="${localization_mode}"
  vlm_index=$((${#children[@]} - 1))
  wait_for_value /vlm_nav/input_ready True
  wait_for_value /vlm_nav/vlm_api_ready True
  wait_for_value /vlm_nav/autonomy_ready True
  if [[ "$(timeout 5 ros2 param get /vlm_nav easy_case_mode 2>/dev/null || true)" != *"False"* ]]; then
    echo "ERROR: easy_case_mode must remain false for formal VLM navigation." >&2
    exit 1
  fi
  snapshot_vlm_parameters start
  capture_topic_qos vlm \
    /vlm_nav/diagnostics /vlm_nav/state /vlm_nav/vlm_enabled \
    /vlm_nav/markers /vlm_nav/output_text
  echo "VLM input/API/autonomy gates are ready; VLM remains disabled."
fi

require_evidence
confirm ARM "Confirm the physical emergency stop is ready, the area is clear, and no other Sport controller is active."
require_evidence
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
  require_evidence
  confirm ENABLE_VLM "Confirm the RViz camera preview, maps, path area, and target description are correct."
  require_evidence
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
      "${script_dir}/stopgo.sh"
      bridge_started=false
      break
    fi
    require_child "${nav2_index}"
    require_child "${bridge_index}"
    require_child "${vlm_index}"
    require_evidence
    sleep 0.5
  done
  finish_evidence
  exit 0
fi

echo "ARMED: use RViz 2D Goal Pose for repeated tests."
echo "Enter CANCEL to cancel the active goal; enter DISARM after all tests."
echo "Emergency stop from another terminal: ${script_dir}/stopnav2.sh"
printf 'manualnav2> '
while true; do
  if read -r -t 1 command; then
    :
  else
    read_status=$?
    if (( read_status == 1 )); then
      break
    fi
    require_active_children
    continue
  fi
  case "${command}" in
    CANCEL)
      ros2 service call /navigate_to_pose/_action/cancel_goal \
        action_msgs/srv/CancelGoal \
        "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
      echo "Wait for CANCELED and a complete stop before selecting the next goal."
      ;;
    DISARM)
      "${script_dir}/stopgo.sh"
      wait_for_value /vlm_nav/go2_bridge_state DISARMED
      wait_for_value /vlm_nav/control_armed False
      bridge_started=false
      break
      ;;
    *) echo "Enter CANCEL or DISARM." ;;
  esac
  printf 'manualnav2> '
done
if [[ "${bridge_started}" == true ]]; then
  echo "ERROR: input ended before DISARM; stopping before the end snapshot." >&2
  exit 1
fi
finish_evidence
