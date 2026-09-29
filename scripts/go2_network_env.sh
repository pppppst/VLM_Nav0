#!/usr/bin/env bash
# Source this file before any local Humble process that communicates with Go2.

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "ERROR: go2_network_env.sh requires bash" >&2
  return 1 2>/dev/null || exit 1
fi

go2_required_rmem=16777216
go2_actual_rmem="$(sysctl -n net.core.rmem_max 2>/dev/null)"
if [[ ! "$go2_actual_rmem" =~ ^[0-9]+$ ]] || (( go2_actual_rmem < go2_required_rmem )); then
  echo "ERROR: net.core.rmem_max=${go2_actual_rmem:-unavailable}, require >=${go2_required_rmem} for CycloneDDS large PointCloud2" >&2
  return 1 2>/dev/null || exit 1
fi
unset go2_required_rmem go2_actual_rmem

go2_has_subnet_address() {
  local interface="$1"
  ip -o -4 address show dev "$interface" 2>/dev/null \
    | awk '$4 ~ /^192\.168\.123\./ {found=1} END {exit !found}'
}

if [[ -n "${GO2_NET_IFACE:-}" ]]; then
  if ! ip link show dev "$GO2_NET_IFACE" >/dev/null 2>&1; then
    echo "ERROR: GO2_NET_IFACE=${GO2_NET_IFACE} does not exist" >&2
    return 1 2>/dev/null || exit 1
  fi
  if ! go2_has_subnet_address "$GO2_NET_IFACE"; then
    echo "ERROR: GO2_NET_IFACE=${GO2_NET_IFACE} has no 192.168.123.x address" >&2
    return 1 2>/dev/null || exit 1
  fi
else
  mapfile -t matches < <(
    ip -o -4 address show \
      | awk '$4 ~ /^192\.168\.123\./ {print $2}' \
      | sort -u
  )
  if (( ${#matches[@]} == 0 )); then
    echo "ERROR: no interface has a 192.168.123.x address" >&2
    return 1 2>/dev/null || exit 1
  fi
  if (( ${#matches[@]} > 1 )); then
    echo "ERROR: more than one interface matches 192.168.123.x; set GO2_NET_IFACE" >&2
    printf '  %s\n' "${matches[@]}" >&2
    return 1 2>/dev/null || exit 1
  fi
  export GO2_NET_IFACE="${matches[0]}"
fi

go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${go2_script_dir}/../config/cyclonedds_go2.xml"
unset go2_script_dir

echo "Go2 DDS interface: ${GO2_NET_IFACE}" >&2
