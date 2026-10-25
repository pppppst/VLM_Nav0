#!/usr/bin/env bash
# Full Go2 pipeline; invoking this entry authorizes its automatic ARM and one task start.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'HELP'
用法：bash scripts/start_go2_cooperative.sh
在Go2控制电脑的图形桌面交互式Bash终端执行，任意目录均可；各组件窗口保持运行。
各自准备：live预检→相机→传感器预检→System→RViz→DISARMED Sport→NAV_READY/输入检查。
等待对方：任务bridge→ARM→agent，等待两端各3条新鲜健康WAITING_PEER/STANDBY。
本轮握手：coordinator/watch→仅一次start→SYNCING→双方run_state/reply均为SYNCED。
watch自动打开“Go2 当前状态”窗口，显示状态、完整原因、任务阶段及消息新鲜度；过期值标为最后报告。
执行：coordinator先同步SEARCH_AB再派工；READY仍只表示A/B严格到达验收。
不要求输入CAMERA、START、READY、CURRENT或ARM确认词；运行本入口会自动ARM并发起真实任务。
Ranger可同时或稍后按原流程准备硬件、bridge及真实agent；本入口持续等待，不远程操作Ranger。
需Go2控制电脑图形桌面。未初始化的真实位姿仍在RViz设置，SSH未免密时仍需相机凭据。
准备采样无限等待，每5秒报告原因；600秒整轮预算从start受理开始，执行期通信门槛3秒。
RViz默认显示：蓝色P编号候选点、橙色VLM选点箭头、绿色ABC目标、青色导航路径。
已打开的RViz需用 File → Open Config 加载 config/go2_exploration.rviz 才会更新显示配置。
实际安装由Cooperative_env.bash选定并导出；COOP_SETUP_FILE/COOP_CONFIG可显式覆盖。
条件不满足、重复任务节点或FAULT时停止，不猜坐标、不改验证标志、不重试start或强停旧进程。
暂停沿用：bash scripts/start_go2_cooperative_task.sh pause
退出本入口或watch不会暂停任务或DISARM；失败后先核对原窗口，不叠加运行完整入口。
完整步骤与关闭顺序：docs/双机协同启动与验收.md
HELP
  exit 0
fi
(( $# == 0 )) || { echo '不接受参数；使用 --help。' >&2; exit 2; }
source "$repo/Cooperative_env.bash" go2 "${COOP_CONFIG:-}"
exec /usr/bin/python3 "$repo/scripts/start_go2_cooperative_task.py" run
