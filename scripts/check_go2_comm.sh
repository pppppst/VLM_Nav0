#!/usr/bin/env bash
# Default preflight is file/configuration inspection; bridge/agent/check are explicit actions.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for arg in "$@"; do
  if [[ "$arg" == --help || "$arg" == -h ]]; then
    exec /usr/bin/python3 "$script_dir/check_multi_agent_comm.py" --side go2 "$@"
  fi
done
export COMM_ENV_FILE="${COMM_ENV_FILE:-$script_dir/../CoVLM_env.bash}"
set +u
source "$COMM_ENV_FILE" || exit $?
set -u
exec "${COOPERATIVE_RUNTIME_PYTHON:-/usr/bin/python3}" "$script_dir/check_multi_agent_comm.py" --side go2 "$@"
