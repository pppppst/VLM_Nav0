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

status_dir="$(mktemp -d /tmp/go2-vlm-entry.XXXXXX)"
if [[ -f "${script_dir}/go2_experiment_status.sh" ]]; then
  source "${script_dir}/go2_experiment_status.sh"
  go2_status_init "${script_dir}/.." "$status_dir"
else
  go2_status() { :; }
fi
echo "启动日志：${status_dir}/entry.log"
exec 3>&1 4>&2
export GO2_ENTRY_STDOUT_OPEN=1
exec >"${status_dir}/entry.log" 2>&1

unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
if declare -F go2_load_overlay >/dev/null; then go2_load_overlay; fi
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  venv_activate="${VLM_NAV_VENV_ACTIVATE:-/home/isee-pst/venv/co-nav-real/bin/activate}"
  if [[ ! -r "${venv_activate}" ]]; then
    echo "ERROR: Python environment not active and ${venv_activate} is unavailable." >&2
    exit 1
  fi
  source "${venv_activate}"
fi
until source "${script_dir}/go2_network_env.sh"; do
  go2_status WAITING '等待Go2网卡和DDS环境；请修复原配置' MANUAL_REQUIRED
  sleep 1
done
export ROS_LOG_DIR=/tmp/VLMNav-go_roslog
mkdir -p "${ROS_LOG_DIR}"

until ping -I "${GO2_NET_IFACE:-enp4s0}" -c 1 -W 2 192.168.123.18 >/dev/null; do
  go2_status WAITING '等待Go2网络连接' WAIT_INPUT
  sleep 1
done
while [[ "$(ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.18 \
  'timedatectl show -p NTPSynchronized --value' 2>/dev/null)" != yes ]]; do
  go2_status WAITING '等待Go2时钟同步或SSH连接恢复' WAIT_INPUT
  sleep 1
done

publisher_count() {
  timeout 5 ros2 topic info "$1" 2>/dev/null \
    | awk '/Publisher count:/ {print $3; exit}'
}

camera_param_is() {
  timeout 5 ros2 param get /camera/camera "$1" 2>/dev/null \
    | grep -Fqx "$2"
}

while [[ "$(publisher_count /camera/camera/color/image_raw)" != 1 ||
      "$(publisher_count /camera/camera/depth/image_rect_raw)" != 1 ]] ||
   ! camera_param_is depth_module.depth_profile 'String value is: 640x480x15' ||
   ! camera_param_is rgb_camera.color_profile 'String value is: 640x480x15' ||
   ! camera_param_is enable_sync 'Boolean value is: True' ||
   ! camera_param_is align_depth.enable 'Boolean value is: False'; do
  go2_status WAITING '等待D435唯一发布者和原RGB-D参数恢复' WAIT_INPUT
  sleep 1
done
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

unset ALL_PROXY all_proxy
export VLM_NAV_MODE=true
export VLM_TARGET_DESCRIPTION="${target_description}"
export VLM_NAV_ENTRY="$(readlink -f "${BASH_SOURCE[0]}")"
exec "${script_dir}/manualnav2.sh" \
  localization_mode:=amcl map:="${map_path}" amcl_scan_topic:=/scan
