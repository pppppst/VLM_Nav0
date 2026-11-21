#!/usr/bin/env bash
# Shared, file-only diagnostics; no ROS subscriptions or motion commands.
go2_status_init() {
  local root="$1" directory="$2"
  GO2_STATUS_TOOL="${root}/cooperative_ws/src/multi_agent_nav/multi_agent_nav/experiment_status.py"
  if [[ ! -f "$GO2_STATUS_TOOL" ]]; then
    GO2_STATUS_TOOL="$(python3 -c 'import multi_agent_nav.experiment_status as m; print(m.__file__)' 2>/dev/null || true)"
  fi
  export GO2_EXPERIMENT_STATUS_FILE="${GO2_EXPERIMENT_STATUS_FILE:-${directory}/status.json}"
  go2_status WAITING '等待启动检查' WAIT_INPUT
  if [[ "${GO2_EXPERIMENT_STATUS_WINDOW:-}" != "$GO2_EXPERIMENT_STATUS_FILE" ]]; then
    gnome-terminal --window --title='Go2 实验状态' -- python3 "$GO2_STATUS_TOOL" \
      --watch "$GO2_EXPERIMENT_STATUS_FILE" >/dev/null 2>&1 || true
    export GO2_EXPERIMENT_STATUS_WINDOW="$GO2_EXPERIMENT_STATUS_FILE"
  fi
}

go2_status() {
  [[ -f "${GO2_STATUS_TOOL:-}" ]] || return 0
  python3 "$GO2_STATUS_TOOL" --write "$GO2_EXPERIMENT_STATUS_FILE" \
    --state "$1" --reason "$2" --reason-class "${3:-WAIT_INPUT}" \
    --component "${4:-}" --log-path "${5:-}" >/dev/null 2>&1 || true
}

go2_prompt_sound() {
  paplay /usr/share/sounds/freedesktop/stereo/dialog-warning.oga >/dev/null 2>&1 &
}

go2_entry_environment() {
  local root="$1" environment="$2" directory
  shift 2
  directory="$(mktemp -d /tmp/go2-entry.XXXXXX)"
  go2_status_init "$root" "$directory"
  echo "环境检查日志：${directory}/environment.log"
  until source "$environment" "$@" >>"${directory}/environment.log" 2>&1; do
    go2_status WAITING '环境未就绪；请检查网卡/安装路径及日志，Ctrl+C可退出' MANUAL_REQUIRED environment "${directory}/environment.log"
    sleep 1
  done
}

go2_load_overlay() {
  local had_nounset=false
  [[ "$-" == *u* ]] && had_nounset=true
  export COOP_SETUP_FILE="${COOP_SETUP_FILE:-/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-codex-compat-20261111/overlay/install/local_setup.bash}"
  while [[ ! -r "$COOP_SETUP_FILE" ]]; do
    go2_status WAITING '当前Go2安装不存在，请修复安装路径' MANUAL_REQUIRED environment "$COOP_SETUP_FILE"
    sleep 1
  done
  set +u
  source "$COOP_SETUP_FILE" || return 1
  if "$had_nounset"; then set -u; fi
  export VLM_NAV_EXTRA_SETUP_FILES="${VLM_NAV_EXTRA_SETUP_FILES:+$VLM_NAV_EXTRA_SETUP_FILES:}$COOP_SETUP_FILE"
}
