#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
  --grasp|--place|--piper-help)
    framework="$repo/../SenseNova-RoboRSI"
    [[ -d "$framework" ]] || framework="$HOME/unitree_ros2/SenseNova-RoboRSI"
    if [[ "$1" == --piper-help ]]; then set -- --help; fi
    exec "$framework/.venv/bin/python" -I "$repo/tools/go2_agent/piper.py" "$@"
    ;;
esac
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo "PiPER 工具：--grasp 目标 [--then-place] | --place 抓取编号；详见 --piper-help"
  exec "$repo/scripts/start_go2_vlm.sh" "$@"
fi
exec "$repo/scripts/start_go2_vlm.sh" --agent-framework "$@"
