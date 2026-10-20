#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec /usr/bin/python3 "$repo/scripts/start_go2_vlm.py" "$@"
fi
# ROS setup files are not compatible with nounset.
source "$repo/CoVLM_env.bash"
/home/isee-pst/venv/co-nav-real/bin/python - <<'PY'
import os
from pathlib import Path

try:
    import multi_agent_nav.adapters.go2 as go2
    import multi_agent_nav.go2_exploration as exploration

    prefix = (Path(os.environ['GO2_EXPLORE_OUTPUT']) / 'install/multi_agent_nav').resolve()
    for module in (go2, exploration):
        actual = Path(module.__file__).resolve()
        if not actual.is_relative_to(prefix):
            raise RuntimeError(f'{module.__name__}: {actual}; 预期位于 {prefix}')
        print(f'{module.__name__}={actual}', flush=True)
    if not hasattr(go2.Go2Adapter, 'approach_candidate_free'):
        raise RuntimeError('当前 adapter 缺少全局/局部接近候选检查')
except (ImportError, KeyError, RuntimeError) as error:
    raise SystemExit(f'安装校验失败，未启动实验：{error}')
PY
exec /usr/bin/python3 "$repo/scripts/start_go2_vlm.py" "$@"
