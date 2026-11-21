#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == diagnostics ]]; then
  if [[ $# -gt 2 ]]; then
    echo 'Usage: bash piper_gripper_debug.sh diagnostics [LOG_PATH]' >&2
    exit 2
  fi
  exec tail -n 40 -F -- "${2:-$HOME/.local/state/go2-piper-gripper/current.log}"
fi
exec /home/unitree/piper-venv/bin/python -I "$script_dir/go2_piper_gripper.py" "$@"
