"""Exercise overlay selection with stub setup files, never sourcing hardware env."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize('override', [False, True])
def test_go2_environment_selects_single_robot_parity_overlay_and_preserves_override(tmp_path, override):
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
        'build/go2-local-scan-20261029/overlay/install/local_setup.bash':'single-parity',
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
    assert lines[0] == ('override' if override else 'single-parity')
    assert lines[1] == str(tmp_path/('custom/install/local_setup.bash' if override else
        'build/go2-local-scan-20261029/overlay/install/local_setup.bash'))


@pytest.mark.parametrize('override',[False,True])
def test_standalone_and_child_load_one_pair_without_legacy_overlays(tmp_path,override):
    root=Path(__file__).resolve().parents[1]
    source=(root/'CoVLM_env.bash').read_text()
    # Stub hardware/ROS setup only; execute the actual selection and child reload.
    for absolute in ('/opt/ros/humble/setup.bash',
        '/home/isee-pst/Documents/liang/hesai_xt16_ws/install/setup.bash',
        '/home/isee-pst/unitree_ros2/go2_ws/install/setup.bash'):
        stub=tmp_path/(str(len(absolute))+Path(absolute).parent.parent.name+'.bash')
        stub.write_text(':\n');source=source.replace(absolute,str(stub))
    (tmp_path/'CoVLM_env.bash').write_text(source)
    (tmp_path/'scripts').mkdir()
    (tmp_path/'scripts/go2_network_env.sh').write_text(':\n')
    msg=tmp_path/'messages/install/local_setup.bash';msg.parent.mkdir(parents=True);msg.write_text(':\n')
    default=tmp_path/'build/go2-local-scan-20261029/overlay/install/local_setup.bash'
    custom=tmp_path/'custom/install/local_setup.bash'
    for setup in (default,custom):
        setup.parent.mkdir(parents=True)
        setup.write_text('export LOADED_PAIR="${BASH_SOURCE[0]}"\n')
    env=dict(os.environ,GO2_COOP_OUTPUT=str(msg.parents[1]),COOP_SETUP_FILE=str(custom) if override else '')
    result=subprocess.run(['bash','--noprofile','--norc','-c',
        'source "$1/CoVLM_env.bash" || exit; printf "%s\\n" "$LOADED_PAIR"; '
        'bash -c \'source "$REPO/CoVLM_env.bash" || exit; printf "%s\\n" "$LOADED_PAIR" "$COOP_SETUP_FILE"\'',
        'env-test',str(tmp_path)],env=env,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert result.stdout.splitlines()==[str(custom if override else default)]*3
