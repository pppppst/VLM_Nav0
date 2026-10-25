"""Exercise overlay selection with stub setup files, never sourcing hardware env."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize('override', [False, True])
def test_go2_environment_selects_target_plan_overlay_and_preserves_override(tmp_path, override):
    root = Path(__file__).resolve().parents[1]
    shutil.copy2(root/'Cooperative_env.bash', tmp_path/'Cooperative_env.bash')
    config = tmp_path/'config'
    config.mkdir()
    for name in ('cyclonedds_go2.xml','cyclonedds_go2_multi_agent_bridge.xml','bridge_go2_pending.yaml'):
        (config/name).write_text('offline fixture')
    (tmp_path/'CoVLM_env.bash').write_text('export GO2_CONFIG='+str(config)+'\n')
    setups = {
        'build/go2-independent-c-20261020/install/local_setup.bash':'old',
        'build/go2-rviz-markers-20261022/install/local_setup.bash':'old-rviz',
        'build/go2-target-plan-20261025/overlay/install/local_setup.bash':'target-plan',
        'custom/install/local_setup.bash':'override',
    }
    for path, marker in setups.items():
        target = tmp_path/path
        target.parent.mkdir(parents=True)
        target.write_text('export LOADED_COOPERATIVE_OVERLAY='+marker+'\n')
    environment = dict(os.environ, COOP_SETUP_FILE=str(tmp_path/'custom/install/local_setup.bash') if override else '')
    result = subprocess.run(['bash','--noprofile','--norc','-c',
        'source "$1" go2 || exit $?\nprintf "%s\\n" "$LOADED_COOPERATIVE_OVERLAY"\nbash -c \'printf "%s\\n" "$COOP_SETUP_FILE"\'',
        'offline-env-test',str(tmp_path/'Cooperative_env.bash')],
        env=environment, capture_output=True, text=True, check=True)
    lines = result.stdout.splitlines()
    assert lines[0] == ('override' if override else 'target-plan')
    assert lines[1] == str(tmp_path/('custom/install/local_setup.bash' if override else
        'build/go2-target-plan-20261025/overlay/install/local_setup.bash'))
