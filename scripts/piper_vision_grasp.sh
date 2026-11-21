#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$script_dir/.." && pwd)"
piper_python="${PIPER_VISION_GRASP_PYTHON:-/usr/bin/python3}"
if [[ "${1:-}" == "serve-graspnet" || "${1:-}" == "visualize" ]]; then
  piper_python="${PIPER_GRASPNET_PYTHON:-$HOME/unitree_ros2/VLM_Nav/build/piper-graspnet-20261117/model-venv/bin/python}"
fi
exec "$piper_python" -I "$root/tools/piper_vision_grasp/main.py" "$@"
