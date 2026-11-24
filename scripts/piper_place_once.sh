#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == --help ]]; then
    echo '用法：bash piper_place_once.sh [抓取运行编号] [--check]；默认取最近一次抓取。'
    exit 0
fi
args=()
for arg in "$@"; do
    [[ "$arg" == --check || "$arg" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || exit 2
    args+=("$arg")
done
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
stage=$(mktemp -d)
trap 'rm -f "$stage/agent_release.py"; rmdir "$stage"' EXIT
# Extract the actual primitive. Importing primitive3 wholesale constructs and enables a second CAN driver.
python3 - "$HOME/unitree_ros2/piper/agent/primitive3.py" "$stage/agent_release.py" <<'PY'
import ast,sys
from pathlib import Path
source=Path(sys.argv[1]).read_text()
fn,=[n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='release_gripper']
Path(sys.argv[2]).write_text('from __future__ import annotations\nimport time\nHAND_GRIPPED_OBJECTS = {}\n'+ast.get_source_segment(source,fn)+'\n')
PY
remote=$(ssh -T -o BatchMode=yes unitree-go 'mktemp -d /tmp/piper-place.XXXXXXXX')
[[ "$remote" =~ ^/tmp/piper-place\.[a-zA-Z0-9]+$ ]]
scp -q -o BatchMode=yes "$stage/agent_release.py" "$root/tools/piper_vision_grasp/place_once.py" "unitree-go:$remote/"
# Arguments above are limited to plain run IDs and the read-only check flag.
worker="~/piper-handeye-20261112/.handeye-venv/bin/python -I -u $remote/place_once.py"
check=$(ssh -T -o BatchMode=yes unitree-go "$worker --check ${args[*]}")
for arg in "${args[@]}"; do
    if [[ "$arg" == --check ]]; then echo "$check"; exit 0; fi
done
grasp_id=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["source"])' <<< "$check")
[[ "$grasp_id" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]
if command -v gnome-terminal >/dev/null && [[ -n ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
    gnome-terminal --window --title='PiPER · 放置诊断' -- ssh -T -o BatchMode=yes unitree-go \
        "tail -n 12 -F ~/piper-place-runs/$grasp_id/events.jsonl" >/dev/null 2>&1 || true
fi
ssh -T -o BatchMode=yes unitree-go "exec $worker $grasp_id"
