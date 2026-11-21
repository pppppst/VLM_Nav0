#!/usr/bin/env bash
# Six joint angles to zero; uses the existing Piper diagnostics and control lock.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action=zero
if [[ "${1:-}" == diagnostics ]]; then
  action=diagnostics
  shift
fi
exec bash "$script_dir/piper_vision_grasp.sh" "$action" "$@"
