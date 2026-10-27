#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec /usr/bin/python3 "$repo/scripts/start_go2_vlm.py" "$@"
fi
# ROS setup files are not compatible with nounset.
source "$repo/CoVLM_env.bash"
/home/isee-pst/venv/co-nav-real/bin/python "$repo/scripts/start_go2_vlm.py" --check-install
exec /usr/bin/python3 "$repo/scripts/start_go2_vlm.py" "$@"
