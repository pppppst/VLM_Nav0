#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
report="${GO2_PREFLIGHT_REPORT:-/tmp/go2_costmap_preflight.json}"
runtime_dir="$(mktemp -d /tmp/go2_preflight.XXXXXX)"
children=()
if [[ -f "${script_dir}/go2_experiment_status.sh" ]]; then
  source "${script_dir}/go2_experiment_status.sh"
  go2_status_init "${project_dir}" "${runtime_dir}"
else
  go2_status() { :; }
fi
echo "预检日志：${runtime_dir}/preflight.log"
exec >"${runtime_dir}/preflight.log" 2>&1

group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

cleanup() {
  local pid attempt
  set +e
  for pid in "${children[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null
    for attempt in {1..50}; do
      group_alive "$pid" || break
      sleep 0.1
    done
    if group_alive "$pid"; then
      kill -TERM -- "-${pid}" 2>/dev/null
      for attempt in {1..20}; do
        group_alive "$pid" || break
        sleep 0.1
      done
    fi
    if group_alive "$pid"; then
      kill -KILL -- "-${pid}" 2>/dev/null
    fi
    wait "$pid" 2>/dev/null
  done
  set -e
}
trap cleanup EXIT

unset PYTHONHOME PYTHONPATH
source "${script_dir}/common.sh"
if declare -F go2_load_overlay >/dev/null; then go2_load_overlay; fi
until source "${script_dir}/go2_network_env.sh"; do
  go2_status WAITING '等待Go2网卡/DDS环境，请修复原配置' MANUAL_REQUIRED
  sleep 1
done

if ! /usr/bin/python3 -c 'import sys; from pathlib import Path; from vlm_nav.go2_sensor_preflight import load_validated_config; load_validated_config(Path(sys.argv[1]))' "${project_dir}/config/go2_preflight_xt16.yaml"; then
  go2_status FAILED '静态预检配置或标定无效；修复后重新启动' NON_RESUMABLE sensor "${runtime_dir}/preflight.log"
  exit 1
fi

if [[ -r "${report}" ]] && /usr/bin/python3 -c '
import json, sys, time
from vlm_nav.go2_preflight import validate_formal_preflight_report
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
validate_formal_preflight_report(report, now_unix_s=time.time(), max_age_s=600.0)
' "${report}" 2>/dev/null; then
  echo "PASS: reusing fresh formal preflight report: ${report}"
  exit 0
fi

publisher_count() {
  local info
  info="$(ros2 topic info "$1" 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$info"
}

wait_for_publisher() {
  local topic="$1" pid="$2" log="$3" attempt
  while true; do
    if (( $(publisher_count "$topic") == 1 )); then
      return
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      go2_status WAITING "${topic} 的组件退出，请在原终端人工恢复" MANUAL_REQUIRED "${topic}" "${log}"
    else
      go2_status WAITING "等待 ${topic} 唯一发布者" WAIT_INPUT "${topic}" "${log}"
    fi
    sleep 1
  done
}

for topic in /lidar_points /body_imu; do
  while (( $(publisher_count "$topic") != 0 )); do
    go2_status WAITING "${topic} 已有发布者，请在原终端结束旧预检组件" MANUAL_REQUIRED
    sleep 1
  done
done

echo "Starting temporary XT16 driver and Go2 body-IMU adapter..."
setsid ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:="/home/isee-pst/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml" \
  >"${runtime_dir}/hesai.log" 2>&1 &
children+=("$!")

setsid ros2 run vlm_nav lowstate_imu_adapter \
  >"${runtime_dir}/body_imu.log" 2>&1 &
children+=("$!")

wait_for_publisher /lidar_points "${children[0]}" "${runtime_dir}/hesai.log"
wait_for_publisher /body_imu "${children[1]}" "${runtime_dir}/body_imu.log"

echo "Both sensor topics are ready. Keep Go2 stationary for 30 seconds."
attempt=0
while true; do
  attempt=$((attempt+1))
  sample="${runtime_dir}/sample-${attempt}.json"
  if /usr/bin/python3 "${script_dir}/go2_sensor_preflight" \
  --config="${project_dir}/config/go2_preflight_xt16.yaml" \
  --duration=30 \
  --output="$sample"; then
    cp "$sample" "$report"
    break
  fi
  if [[ ! -r "$sample" ]]; then
    go2_status FAILED '预检未产生报告：程序或证据写入异常，无法可信续接' NON_RESUMABLE sensor "${runtime_dir}/preflight.log"
    exit 1
  fi
  go2_status WAITING '传感器预检未通过，等待修复后重新采样；不重启驱动' WAIT_INPUT sensor "${runtime_dir}/preflight.log"
  sleep 1
done

cleanup
trap - EXIT

/usr/bin/python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("healthy:", r["healthy"], "cloud_count:", r["cloud_count"], "imu_count:", r["imu"]["sample_count"])' "$report"
echo "PASS: temporary sensor processes stopped; report: ${report}"
echo "Start go2_system.launch.py within 600 seconds."
echo "Temporary logs: ${runtime_dir}"
