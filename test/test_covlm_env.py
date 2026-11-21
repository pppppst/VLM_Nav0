import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('network_ok', [True, False])
def test_common_environment_sources_real_overlays_and_checks_network(tmp_path, network_ok):
    # Only replace network probes: this test must not require the robot's cable.
    for name, body in {
        'sysctl': 'echo 16777216',
        'ip': ('echo "2: enp4s0 inet 192.168.123.222/24"' if network_ok else 'exit 1'),
    }.items():
        command = tmp_path / name
        command.write_text('#!/bin/bash\n' + body + '\n')
        command.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path) + ':' + os.environ['PATH'],
               PYTHONHOME='/stale-python-home', PYTHONPATH='/stale-overlay',
               ROS_DOMAIN_ID='31', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    result = subprocess.run([
        'bash', '--noprofile', '--norc', '-c',
        'source "$1/CoVLM_env.bash" || exit $?\n'
        '[[ "$PWD" == "$1" && "$REPO" == "$1" ]] || exit 91\n'
        '[[ "$ROS_DOMAIN_ID" == 0 && "$ROS_LOCALHOST_ONLY" == 0 ]] || exit 92\n'
        '[[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && "$GO2_NET_IFACE" == enp4s0 ]] || exit 93\n'
        '[[ -z "${PYTHONHOME:-}" && "$PYTHONPATH" != *stale-overlay* ]] || exit 94\n'
        '[[ "$VLM_NAV_EXTRA_SETUP_FILES" == "$GO2_COOP_OUTPUT/install/local_setup.bash:$COOP_SETUP_FILE" ]] || exit 95\n'
        '[[ -d "$ROS_LOG_DIR" ]] || exit 96\n'
        '[[ "$COOP_SETUP_FILE" == "$REPO/build/go2-codex-compat-20261111/overlay/install/local_setup.bash" ]] || exit 97\n'
        'cd /tmp || exit 98\n'
        '/home/isee-pst/venv/co-nav-real/bin/python -c "import multi_agent_nav.adapters.go2 as g; import multi_agent_nav.go2_exploration as e; import vlm_nav.vlm_client as v; '
        'assert \'go2-codex-compat-20261111/overlay/install/\' in g.__file__; '
        'assert \'go2-codex-compat-20261111/overlay/install/\' in e.__file__; '
        'assert hasattr(g.Go2Adapter, \'approach_candidate_free\'); '
        'assert \'go2-codex-compat-20261111/overlay/install/\' in v.__file__"\n',
        'covlm-test', str(ROOT),
    ], cwd=tmp_path, env=env, capture_output=True, text=True)
    if network_ok:
        assert result.returncode == 0, result.stderr
    else:
        assert result.returncode == 1
        assert 'does not exist' in result.stderr


def test_environment_requires_source():
    result = subprocess.run(['bash', str(ROOT / 'CoVLM_env.bash')], capture_output=True, text=True)
    assert result.returncode == 1
    assert 'source' in result.stderr
