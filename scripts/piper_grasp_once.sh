#!/usr/bin/env bash
# Run on the development desktop in the normal VLM_Nav model environment.
set -euo pipefail
usage() { echo '用法：bash piper_grasp_once.sh "抓取目标描述"'; }
if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
if (( $# != 1 )) || [[ ! $1 =~ [^[:space:]] ]]; then usage >&2; exit 2; fi
target=$1
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
launcher="$script_dir/piper_vision_grasp.sh"
export PIPER_VISION_GRASP_PYTHON="${PIPER_VISION_GRASP_PYTHON:-$HOME/unitree_ros2/VLM_Nav/build/piper-vision-semantic-20261114/model-venv/bin/python}"
work=$(mktemp -d /tmp/piper-grasp-once.XXXXXXXX)
run_id="${PIPER_AGENT_GRASP_ID:-grasp-${work##*.}}"
[[ "$run_id" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || exit 2
remote_root=/home/unitree/piper-r4-grasp-evidence
socket="$work/ssh"
model_pid=
cleanup() {
    if [[ -S $socket ]]; then ssh -S "$socket" -O exit unitree-go >/dev/null 2>&1 || true; fi
    if [[ -n $model_pid ]]; then kill "$model_pid" 2>/dev/null || true; wait "$model_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
printf '模型诊断：%s/model/events.jsonl\n启动日志：%s/startup.log\n机载结果：%s/%s/result.json\n' "$work" "$work" "$remote_root" "$run_id"
bash "$launcher" serve --output-root "$work" --run-id model >"$work/startup.log" 2>&1 &
model_pid=$!
until grep -Eq '"state":[[:space:]]*"MODEL_READY"' "$work/model/events.jsonl" 2>/dev/null; do
    if ! kill -0 "$model_pid" 2>/dev/null; then exit 1; fi
    sleep 0.3
done
ssh -fNT -M -S "$socket" -o BatchMode=yes -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=15 -o ServerAliveCountMax=2 \
    -R 127.0.0.1:18765:127.0.0.1:18765 unitree-go 2>>"$work/startup.log"
if command -v gnome-terminal >/dev/null && [[ -n ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
    gnome-terminal --window --title='PiPER · 抓取诊断' -- ssh -T -o BatchMode=yes unitree-go \
        "tail -n 20 -F '$remote_root/$run_id/events.jsonl'" >>"$work/startup.log" 2>&1 || true
fi
bash "$launcher" desktop-run --output-root "$remote_root" --run-id "$run_id" \
    --color '以目标描述为准' --description="$target"
