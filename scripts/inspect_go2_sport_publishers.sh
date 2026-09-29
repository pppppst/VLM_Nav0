#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"

duration="${1:-10}"
if [[ ! "${duration}" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: $0 [positive-duration-seconds]" >&2
  exit 2
fi

output="/tmp/go2_sport_publishers_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "${output}"
export ROS_LOG_DIR="${output}/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

state="$(timeout 5 ros2 topic echo /vlm_nav/go2_bridge_state --once --field data 2>/dev/null || true)"
if [[ "${state}" != *DISARMED* ]]; then
  echo "ERROR: bridge must be running and DISARMED; observed: ${state:-no state}" >&2
  exit 1
fi

ros2 topic info /api/sport/request -v >"${output}/topic_info.txt"
ss -uapne >"${output}/ss_udp.txt" 2>&1 || true
if command -v lsof >/dev/null 2>&1; then
  lsof -nP -iUDP >"${output}/lsof_udp.txt" 2>&1 || true
fi
ps -eo pid,ppid,user,lstart,args --forest >"${output}/processes.txt"
timeout 5 systemctl --user list-units --type=service --all \
  >"${output}/systemd_user.txt" 2>&1 || true
timeout 5 systemctl list-units --type=service --all \
  >"${output}/systemd_system.txt" 2>&1 || true
if command -v tmux >/dev/null 2>&1; then
  tmux list-sessions >"${output}/tmux.txt" 2>&1 || true
fi

trace_xml="${output}/cyclonedds_trace.xml"
sed "s#</Domain>#    <Tracing><Verbosity>finest</Verbosity><OutputFile>${output}/cyclonedds.\${CYCLONEDDS_PID}.log</OutputFile></Tracing>\n  </Domain>#" \
  "${script_dir}/../config/cyclonedds_go2.xml" >"${trace_xml}"

CYCLONEDDS_URI="file://${trace_xml}" timeout "${duration}" \
  ros2 topic echo /api/sport/request unitree_api/msg/Request \
  --qos-reliability best_effort >"${output}/sport_requests.txt" 2>"${output}/echo_stderr.txt" || true

ss -uapne >"${output}/ss_udp_after.txt" 2>&1 || true
echo "PASS: read-only Sport publisher evidence captured: ${output}"
echo "Use topic_info.txt + bridge foreign-request GID logs + CycloneDDS GUID/locator trace to map source IP/port, then ss/lsof to local PID."
