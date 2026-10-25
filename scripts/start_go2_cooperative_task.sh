#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'HELP'
用法：bash scripts/start_go2_cooperative_task.sh [prepare|start|pause|disarm|restart]
  prepare 默认：live预检→本机bridge→人工确认ARM→真实agent→两端WAITING_PEER/STANDBY核对→coordinator/watch
  start   再次预检，人工确认START并重查两端状态后显式开始一轮
  pause   立即调用原暂停入口，不受live gates阻塞；仍须确认实际停稳
  disarm  人工确认两端停稳/原生动作结束后，解除Go2 Sport使能
  restart 第二轮：暂停→引导原窗口退出旧任务组件→DISARM→NAV_READY/输入检查→prepare→一次start
          保留健康硬件；需人工确认CLOSED、STOPPED、ARM、START，不自动结束进程或操作Ranger
当前live gates须为空，跳过PAUSED专用检查；不跳过bridge或agent。
prepare/start保留人工确认；无需确认词的完整启动用start_go2_cooperative.sh，二者不要叠加start。
prepare自动打开“Go2 当前状态”窗口，显示状态、完整原因、任务阶段及消息新鲜度；关闭只结束显示。
准备等待允许Ranger晚启动；SYNCED须同时匹配run_state和reply，READY不是启动握手。
先完成Go2硬件准备；Ranger由操作者按原文档在本机启动，不自动SSH或重启旧节点。
HELP
  exit 0
fi
(( $# <= 1 )) || { echo '只接受一个动作；使用 --help。' >&2; exit 2; }
action="${1:-prepare}"
case "$action" in prepare|start|pause|disarm|restart) ;; *) echo '未知动作；使用 --help。' >&2; exit 2;; esac
source "$repo/Cooperative_env.bash" go2 "${COOP_CONFIG:-}"
exec /usr/bin/python3 "$repo/scripts/start_go2_cooperative_task.py" "$action"
