#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"
source "${script_dir}/go2_network_env.sh"

topics=(
  /utlidar/cloud
  /utlidar/imu
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/color/camera_info
)

declare -A expected_reliability expected_durability
expected_reliability[/utlidar/cloud]=RELIABLE
expected_reliability[/utlidar/imu]=RELIABLE
expected_reliability[/camera/camera/color/image_raw]=RELIABLE
expected_reliability[/camera/camera/aligned_depth_to_color/image_raw]=RELIABLE
expected_reliability[/camera/camera/color/camera_info]=RELIABLE
expected_durability[/utlidar/cloud]=VOLATILE
expected_durability[/utlidar/imu]=VOLATILE
expected_durability[/camera/camera/color/image_raw]=VOLATILE
expected_durability[/camera/camera/aligned_depth_to_color/image_raw]=VOLATILE
expected_durability[/camera/camera/color/camera_info]=VOLATILE

failed=0
required_samples=3
for topic in "${topics[@]}"; do
  echo "=== ${topic} ==="
  if ! output="$(timeout 8 ros2 topic info -v "$topic" 2>&1)"; then
    echo "$output"
    failed=1
    continue
  fi
  echo "$output"
  if ! grep -q 'Publisher count: [1-9]' <<<"$output"; then
    echo "ERROR: ${topic} has no publisher" >&2
    failed=1
  fi
  publisher_qos="$(awk '
    /Endpoint type: PUBLISHER/ {capture=1; next}
    capture {print}
    capture && /Liveliness lease duration:/ {exit}
  ' <<<"$output")"
  for policy in Reliability Durability History Depth; do
    if ! grep -qi "$policy" <<<"$publisher_qos"; then
      echo "ERROR: ${topic} does not expose QoS ${policy}; compatibility is unproven" >&2
      failed=1
    fi
  done
  if [[ "${expected_reliability[$topic]}" != TBD ]] &&
    ! grep -Eq "Reliability: (RMW_QOS_POLICY_RELIABILITY_)?${expected_reliability[$topic]}" <<<"$publisher_qos"; then
    echo "ERROR: ${topic} publisher reliability differs from verified profile (${expected_reliability[$topic]})" >&2
    failed=1
  fi
  if [[ "${expected_durability[$topic]}" != TBD ]] &&
    ! grep -Eq "Durability: (RMW_QOS_POLICY_DURABILITY_)?${expected_durability[$topic]}" <<<"$publisher_qos"; then
    echo "ERROR: ${topic} publisher durability differs from verified profile (${expected_durability[$topic]})" >&2
    failed=1
  fi
  reliability_ok=0
  durability_ok=0
  history_depth_ok=0
  grep -Eq 'Reliability: (RMW_QOS_POLICY_RELIABILITY_)?(RELIABLE|BEST_EFFORT)' <<<"$publisher_qos" && reliability_ok=1
  grep -Eq 'Durability: (RMW_QOS_POLICY_DURABILITY_)?(VOLATILE|TRANSIENT_LOCAL)' <<<"$publisher_qos" && durability_ok=1
  if grep -Eq 'History \(Depth\): KEEP_LAST \([1-9][0-9]*\)' <<<"$publisher_qos" ||
    { grep -Eq 'History \(Store Policy\): (RMW_QOS_POLICY_HISTORY_)?KEEP_LAST' <<<"$publisher_qos" &&
      grep -Eq 'Depth: [1-9][0-9]*' <<<"$publisher_qos"; }; then
    history_depth_ok=1
  fi
  if (( reliability_ok == 0 || durability_ok == 0 || history_depth_ok == 0 )); then
    echo "ERROR: ${topic} publisher history/depth is not usable KEEP_LAST" >&2
    failed=1
  fi
  echo "MEASURED publisher QoS for ${topic}:"
  grep -E 'Reliability:|Durability:|History \((Store Policy|Depth)\):|Depth:' <<<"$publisher_qos"

  # A BEST_EFFORT/VOLATILE subscriber is compatible with both verified sensor
  # reliability profiles. Seeing the graph or one lucky sample is insufficient.
  received_samples=0
  for (( sample=1; sample<=required_samples; sample++ )); do
    if ! timeout 8 ros2 topic echo "$topic" --once \
      --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1; then
      echo "ERROR: ${topic} received only $((sample - 1))/${required_samples} required samples" >&2
      failed=1
      break
    fi
    received_samples=$sample
  done
  echo "MEASURED compatible reception for ${topic}: ${received_samples}/${required_samples} samples"
done

if (( failed != 0 )); then
  echo "Go2 cross-distro QoS preflight: FAIL" >&2
  exit 1
fi
echo "Go2 cross-distro QoS endpoint inspection: PASS"
