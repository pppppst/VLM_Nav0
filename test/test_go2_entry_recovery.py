"""Exercise shell wait functions with fake ROS/process commands, never hardware."""
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def function(path,name):
    text=Path(path).read_text()
    return name+'() {'+text.split(name+'() {',1)[1].split('\n}',1)[0]+'\n}\n'


def run_shell(body,tmp_path):
    return subprocess.run(['bash','-c','set -euo pipefail\n'+body],cwd=tmp_path,
                          capture_output=True,text=True,timeout=5)


def test_nav_ready_wait_has_no_total_attempt_limit(tmp_path):
    source=os.getenv('GO2_MANUAL_TEST_SOURCE',str(ROOT/'scripts/manualnav2.sh'))
    body=function(source,'wait_for_value')+'''
go2_status() { :; }
require_active_children() { :; }
sleep() { :; }
timeout() {
  count=$(cat counter); count=$((count+1)); echo "$count" >counter
  if (( count > 125 )); then echo True; else echo False; fi
}
echo 0 >counter
wait_for_value /vlm_nav/nav_ready True
test "$(cat counter)" = 126
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==0,result.stderr


def test_missing_nav2_child_waits_for_live_replacement_without_restarting(tmp_path):
    source=os.getenv('GO2_MANUAL_TEST_SOURCE',str(ROOT/'scripts/manualnav2.sh'))
    body=function(source,'require_child')+'''
children=(999); child_names=(nav2); bag_index=-1; session_dir=.; arm_confirmed=true
kill() { return 1; }
sleep() { :; }
go2_status() { echo "$*" >>status; }
timeout() { echo checked >>queries; test "$(wc -l <queries)" -ge 2; }
require_child 0
test "$(wc -l <queries)" = 2
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==0,result.stderr
    assert '人工恢复' in (tmp_path/'status').read_text()


def test_replacement_before_arm_uses_original_stage_checks(tmp_path):
    body=function(ROOT/'scripts/manualnav2.sh','require_child')+'''
children=(999 998); child_names=(nav2 bridge); bag_index=-1; session_dir=.; arm_confirmed=false
kill() { return 1; }
sleep() { :; }
go2_status() { :; }
nav2_available() { return 0; }
timeout() { [[ "$*" == *go2_bridge_state* && "$*" == *DISARMED* ]]; }
require_child 0
require_child 1
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==0,result.stderr


def test_lost_recovery_process_is_explicitly_non_resumable(tmp_path):
    body=function(ROOT/'scripts/manualnav2.sh','require_child')+'''
children=(999); child_names=(recovery); bag_index=-1; session_dir=.
kill() { return 1; }
go2_status() { echo "$*"; }
require_child 0
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==1
    assert 'NON_RESUMABLE' in result.stdout


def test_go2_overlay_is_loaded_last_and_propagated_to_children(tmp_path):
    setup=tmp_path/'local_setup.bash'
    setup.write_text('export ACTIVE_PACKAGE=current\n')
    body=function(ROOT/'scripts/go2_experiment_status.sh','go2_load_overlay')+'''
COOP_SETUP_FILE="$PWD/local_setup.bash"
VLM_NAV_EXTRA_SETUP_FILES=/old/setup.bash
ACTIVE_PACKAGE=old
go2_load_overlay
test "$ACTIVE_PACKAGE" = current
test "$VLM_NAV_EXTRA_SETUP_FILES" = "/old/setup.bash:$PWD/local_setup.bash"
[[ "$-" == *u* ]]
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==0,result.stderr


def test_preflight_dead_producer_waits_for_manual_replacement(tmp_path):
    source=os.getenv('GO2_PREFLIGHT_TEST_SOURCE',str(ROOT/'scripts/run_go2_preflight.sh'))
    body=function(source,'wait_for_publisher')+'''
kill() { return 1; }
sleep() { :; }
go2_status() { :; }
publisher_count() {
 count=$(cat counter); count=$((count+1)); echo "$count" >counter
 if (( count > 2 )); then echo 1; else echo 0; fi
}
echo 0 >counter
wait_for_publisher /body_imu 999 missing.log
test "$(cat counter)" = 3
'''
    result=run_shell(body,tmp_path)
    assert result.returncode==0,result.stderr
