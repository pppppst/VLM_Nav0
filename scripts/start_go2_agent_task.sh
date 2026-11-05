#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec "$repo/scripts/start_go2_vlm.sh" "$@"
fi
exec "$repo/scripts/start_go2_vlm.sh" --agent-framework "$@"
