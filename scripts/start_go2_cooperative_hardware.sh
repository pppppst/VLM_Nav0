#!/usr/bin/env bash
# Prepare Go2 hardware only; task communication starts separately.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
  cat <<'HELP'
用法：bash scripts/start_go2_cooperative_hardware.sh
按双机文档准备 Go2：相机→传感器预检→底层→RViz/人工定位→DISARMED Sport→输入检查。
在控制电脑桌面的交互式 Bash 终端运行；组件各开终端并保持运行。
不 ARM、不请求模型、不启动通信桥/agent/coordinator或任务；配置沿用 COOP_CONFIG。
HELP
  exit 0
fi
(( $# == 0 )) || { echo '不接受参数；使用 --help。' >&2; exit 2; }
source "$repo/Cooperative_env.bash" go2 "${COOP_CONFIG:-}"
exec /usr/bin/python3 "$repo/scripts/start_go2_vlm.py" --cooperative-hardware
