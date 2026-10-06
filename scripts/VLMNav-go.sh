#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "${script_dir}/common.sh" ]]; then
  script_dir="$(ros2 pkg prefix --share vlm_nav)/scripts"
fi

if (( $# != 2 )) || [[ -z "$1" || "$2" != map:=* ]]; then
  echo "用法：$0 \"目标描述\" map:=<saved_map.yaml>" >&2
  exit 2
fi
target_description="$1"
map_path="${2#map:=}"
if [[ ! -f "${map_path}" ]]; then
  echo "ERROR: VLM AMCL navigation requires a valid saved map YAML" >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "ERROR: RViz requires DISPLAY or WAYLAND_DISPLAY." >&2
  exit 1
fi

unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  venv_activate="${VLM_NAV_VENV_ACTIVATE:-/home/isee-pst/venv/co-nav-real/bin/activate}"
  if [[ ! -r "${venv_activate}" ]]; then
    echo "ERROR: Python environment not active and ${venv_activate} is unavailable." >&2
    exit 1
  fi
  source "${venv_activate}"
fi
source "${script_dir}/go2_network_env.sh"
export ROS_LOG_DIR=/tmp/VLMNav-go_roslog
mkdir -p "${ROS_LOG_DIR}"

ping -I "${GO2_NET_IFACE:-enp4s0}" -c 1 -W 2 192.168.123.18 >/dev/null
if [[ "$(ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.18 \
  'timedatectl show -p NTPSynchronized --value' 2>/dev/null)" != yes ]]; then
  echo "ERROR: Go upper computer NTP is not synchronized." >&2
  exit 1
fi

publisher_count() {
  timeout 5 ros2 topic info "$1" 2>/dev/null \
    | awk '/Publisher count:/ {print $3; exit}'
}

camera_param_is() {
  timeout 5 ros2 param get /camera/camera "$1" 2>/dev/null \
    | grep -Fqx "$2"
}

if [[ "$(publisher_count /camera/camera/color/image_raw)" != 1 ||
      "$(publisher_count /camera/camera/depth/image_rect_raw)" != 1 ]] ||
   ! camera_param_is depth_module.depth_profile 'String value is: 640x480x15' ||
   ! camera_param_is rgb_camera.color_profile 'String value is: 640x480x15' ||
   ! camera_param_is enable_sync 'Boolean value is: True' ||
   ! camera_param_is align_depth.enable 'Boolean value is: False'; then
  echo "ERROR: D435 is absent, duplicated, or has the wrong raw RGB-D parameters." >&2
  exit 1
fi
echo "D435进程已启动，不重复启动；跳过相机 preflight。"

units_report="${VLM_DEPTH_UNITS_REPORT:-${HOME}/.config/vlm_nav/raw_depth_units_verified.json}"
if [[ ! -r "${units_report}" ]]; then
  echo "ERROR: raw-depth unit report is missing: ${units_report}" >&2
  exit 1
fi
if [[ -z "${DASHSCOPE_API_KEY:-}" && -z "${OPENAI_API_KEY:-}" ]]; then
  echo "ERROR: DASHSCOPE_API_KEY (or OPENAI_API_KEY) is not set." >&2
  exit 1
fi
if [[ -n "${DASHSCOPE_API_KEY:-}" && -z "${DASHSCOPE_BASE_URL:-}" &&
      -z "${DASHSCOPE_WORKSPACE_ID:-}" ]]; then
  echo "ERROR: DASHSCOPE_BASE_URL or DASHSCOPE_WORKSPACE_ID is required." >&2
  exit 1
fi
if timeout 4 ros2 node list 2>/dev/null \
  | grep -Eq '^/(vlm_nav|twist_to_go2_sport_bridge)$'; then
  echo "ERROR: an old VLM/bridge stack is still running; stop it first." >&2
  exit 1
fi
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "ERROR: gnome-terminal is required for the VLMNav monitor." >&2
  exit 1
fi

speed_monitor="bash -lc 'until timeout 2 ros2 topic type /cmd_vel_bridge >/dev/null 2>&1; do sleep 1; done; exec ros2 topic echo /cmd_vel_bridge'"
diagnostics_monitor="bash -lc 'until timeout 2 ros2 topic type /vlm_nav/diagnostics >/dev/null 2>&1; do sleep 1; done; exec ros2 topic echo /vlm_nav/diagnostics'"
gnome-terminal \
  --window --title="Go2 Velocity" --command="${speed_monitor}" \
  --tab --title="VLM Diagnostics" --command="${diagnostics_monitor}"

unset ALL_PROXY all_proxy
export VLM_NAV_MODE=true
export VLM_TARGET_DESCRIPTION="${target_description}"
export VLM_NAV_ENTRY="$(readlink -f "${BASH_SOURCE[0]}")"
exec "${script_dir}/manualnav2.sh" \
  localization_mode:=amcl map:="${map_path}" amcl_scan_topic:=/scan
