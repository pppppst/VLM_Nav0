#!/usr/bin/env bash
# Read-only evidence collection. Never starts a stack, arms, cancels or sends goals.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="/tmp/go2-field/evidence/readonly-$(date +%Y%m%d_%H%M%S)"
config_dir=/tmp/go2-field/config
duration=15
phase=inspect
dry_run=false
while (($#)); do
  case "$1" in
    --output-dir) output="$2"; shift 2 ;;
    --config-dir) config_dir="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --phase) phase="$2"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) echo 'Usage: collect_go2_cooperative_static.sh [--config-dir DIR] [--output-dir DIR] [--duration SEC] [--phase inspect|baseline|mark|clear] [--dry-run]'; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ "$duration" =~ ^[1-9][0-9]*$ ]] && (( duration <= 60 )) || { echo 'duration must be 1..60 seconds' >&2; exit 2; }
[[ "$phase" =~ ^(inspect|baseline|mark|clear)$ ]] || { echo 'invalid phase' >&2; exit 2; }
capture() {
  local label="$1"; shift
  if $dry_run; then printf 'READ-ONLY %s:' "$label"; printf ' %q' "$@"; printf '\n'; return; fi
  local status=0
  timeout --kill-after=2 5 "$@" >"$output/$label.log" 2>&1 || status=$?
  printf '%s\t%s\n' "$label" "$status" >>"$output/status.tsv"
  echo "$label: exit $status (collection only)"
}
if ! $dry_run; then
  [[ ! -e "$output" ]] || { echo 'Output exists; choose a fresh directory to preserve evidence.' >&2; exit 2; }
  mkdir -p "$(dirname "$output")"
  mkdir "$output"
  printf 'collection_only; no field acceptance flags changed\n' >"$output/classification.txt"
  capture interface ip -br link show dev enp4s0
  capture address ip -br -4 address show dev enp4s0
  export GO2_NET_IFACE=enp4s0
  if ! source "$repo/scripts/go2_network_env.sh" >"$output/network.log" 2>&1; then
    printf 'network\t1\n' >>"$output/status.tsv"
    echo "Local enp4s0 preflight failed; no ROS participant initialized. See $output/network.log" >&2
    exit 1
  fi
  set +u
  source /opt/ros/humble/setup.bash
  source "$repo/../go2_ws/install/setup.bash"
  source "${GO2_COOP_OUTPUT:-/tmp/go2-coop}/install/setup.bash"
  set -u
  export ROS_LOG_DIR="$output/ros"
fi
capture nodes ros2 node list --no-daemon
for node in map_server amcl local_costmap/local_costmap global_costmap/global_costmap \
  go2_costmap_acceptance/local_costmap/local_costmap go2_costmap_acceptance/global_costmap/global_costmap; do
  capture "${node//\//_}" ros2 param dump "/$node"
done
capture map ros2 param get /map_server yaml_filename
capture topics ros2 topic list -t --no-daemon
capture cloud_qos ros2 topic info /vlm_nav/obstacle_cloud -v --no-daemon
if $dry_run; then
  capture adapter ros2 run multi_agent_nav multi_agent_go2_check --config-dir "$config_dir" --timeout "$duration"
else
  status=0
  timeout --kill-after=2 "$((duration+10))" ros2 run multi_agent_nav multi_agent_go2_check \
    --config-dir "$config_dir" --timeout "$duration" >"$output/adapter.json" 2>"$output/adapter.stderr" || status=$?
  printf 'adapter\t%s\n' "$status" >>"$output/status.tsv"
fi
if [[ "$phase" != inspect ]]; then
  if ! $dry_run; then
    python3 - "$output/adapter.json" <<'PY'
import json,sys
health=json.load(open(sys.argv[1]))['health']
if not (health['stopped'] and health['localization_valid']):
    raise SystemExit('Fresh localization and physical stationary confirmation required for static bag')
PY
  fi
  topics=(/tf /tf_static /map /odometry /clicked_point /vlm_nav/obstacle_cloud
    /go2_costmap_acceptance/local_costmap/costmap_raw /go2_costmap_acceptance/global_costmap/costmap_raw
    /local_costmap/costmap_raw /global_costmap/costmap_raw)
  if $dry_run; then
    capture bag timeout --signal=INT --kill-after=5 "$duration" ros2 bag record --output "$output/$phase" "${topics[@]}"
  else
    status=0
    timeout --signal=INT --kill-after=5 "$duration" ros2 bag record --output "$output/$phase" \
      "${topics[@]}" >"$output/bag.log" 2>&1 || status=$?
    printf 'bag\t%s\n' "$status" >>"$output/status.tsv"  # timeout 124 is expected; inspect bag info separately.
  fi
fi
echo 'COLLECTION_ONLY: evaluate evidence before any verified flag; keep Go2 stationary.'
