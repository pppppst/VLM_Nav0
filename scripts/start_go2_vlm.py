#!/usr/bin/env python3
"""Go2 单机启动编排；真实输入检查、人工定位及运动确认保持原流程。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time


def parse_report(output):
    for match in reversed(list(re.finditer(r'^\{', output, re.M))):
        try:
            report, _ = json.JSONDecoder().raw_decode(output[match.start():])
            return report
        except ValueError:
            continue
    raise RuntimeError('未取得 JSON 检查报告，请检查本次日志')


def require_inputs(report):
    failed = [key for key in ('active_map_matches', 'costmap_valid', 'rgbd_ready',
                              'depth_units_verified') if report.get(key) is not True]
    failed += [key for key in ('localization_valid', 'stopped')
               if report.get('health', {}).get(key) is not True]
    window = report.get('health_window', {})
    if window.get('non_motion_inputs_stable') is not True:
        failed.append('non_motion_inputs_stable')
    transitions = window.get('transitions', [])
    if not transitions or transitions[-1].get('other_goal_owners') != []:
        failed.append('no_other_goal_owner')
    if failed:
        raise RuntimeError('现场输入检查未通过：' + ', '.join(failed))


def parse_state(output):
    match = re.search(r"^['\"]?(DISARMED|ARMED|FAULT)(?=['\":\s]|$)", output, re.M)
    if not match:
        raise RuntimeError('无法读取 Sport 状态，不继续 ARM')
    return match[1]


def require_service_success(output):
    if not re.search(r'\bsuccess\s*[=:]\s*(?:True|true)\b', output):
        raise RuntimeError('Sport 服务未返回 success=true；停止后续操作')


class Launcher:
    def __init__(self, repo, prepare_only, target):
        self.repo = repo
        self.config = repo / 'cooperative_ws/src/multi_agent_nav/config'
        self.prepare_only = prepare_only
        self.target = target
        self.logs = None

    def command(self, args, timeout=20, check=True):
        process = subprocess.Popen([str(x) for x in args], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, start_new_session=True)
        try:
            output, _ = process.communicate(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            previous = {sig: signal.signal(sig, signal.SIG_IGN)
                        for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                # These commands never own motion; include ros2 run's observer descendant.
                for sig, grace in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        pass
                    try:
                        process.communicate(timeout=grace)
                        break
                    except subprocess.TimeoutExpired:
                        continue
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
            raise
        result = subprocess.CompletedProcess(args, process.returncode, output)
        if check and result.returncode:
            print(result.stdout, flush=True)
            raise RuntimeError('命令失败：' + ' '.join(str(x) for x in args[:5]))
        return result

    def ros(self, *args, **kwargs):
        return self.command(['ros2', *args], **kwargs)

    def confirm(self, word, message):
        if input(f'{message}\n输入 {word} 继续，其他输入退出：').strip() != word:
            raise RuntimeError('用户取消，停止后续步骤')

    def window(self, name, args):
        # Positional arguments preserve paths and targets without shell interpolation.
        shell = ('source "$1/CoVLM_env.bash" || exit; shift; "$@"; status=$?; '
                 'echo "进程结束 exit=$status，请查看上方日志"; '
                 'read -r -p "按 Enter 关闭此终端……" _; exit "$status"')
        self.command(['gnome-terminal', '--window', f'--title=Go2 {name}', '--',
                      'bash', '-c', shell, 'go2-vlm-terminal', self.repo, *args])
        print(f'已打开 Go2 {name} 终端；运行期间保持打开。', flush=True)

    def publishers(self, topic):
        result = self.ros('topic', 'info', topic, check=False, timeout=10)
        match = re.search(r'^Publisher count:\s*(\d+)\s*$', result.stdout, re.M)
        if result.returncode == 0 and match:
            return int(match[1])
        if 'Unknown topic' in result.stdout:
            return 0
        raise RuntimeError(f'无法核对 {topic} 发布者：{result.stdout.strip()}')

    def camera(self):
        topics = ['/camera/camera/color/image_raw', '/camera/camera/depth/image_rect_raw']
        counts = [self.publishers(t) for t in topics]
        if counts == [1, 1]:
            print('复用已有唯一相机发布者；稍后由 inspect 核对真实 RGB-D。', flush=True)
            return
        if counts != [0, 0]:
            raise RuntimeError(f'相机发布者异常 {counts}；不叠加驱动，请在原终端排障')
        self.confirm('CAMERA', '没有相机发布者。确认旧相机进程/USB已释放，允许 SSH 启动相机。')
        self.window('Camera SSH', ['bash', self.repo / 'scripts/go2_camera_ssh.sh'])
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            counts = [self.publishers(t) for t in topics]
            if counts == [1, 1]:
                return
            if any(n > 1 for n in counts):
                break
            time.sleep(1)
        raise RuntimeError('相机未就绪；检查 Camera SSH 终端，不自动重启')

    def nodes(self):
        return self.ros('node', 'list', '--no-daemon').stdout.splitlines()

    def check_owners(self):
        owners = {'/vlm_nav', '/multi_agent_coordinator', '/multi_agent_go2_agent',
                  '/multi_agent_task_agent', '/multi_agent_go2_rgbd',
                  '/multi_agent_go2_check_motion', '/multi_agent_go2_check_readonly'}
        active = owners.intersection(self.nodes())
        if active:
            raise RuntimeError('先结束已有上层任务/观察器：' + ', '.join(sorted(active)))

    def check(self, *args):
        return ['ros2', 'run', 'multi_agent_nav', 'multi_agent_go2_check',
                '--config-dir', self.config, '--strategy', 'generated_candidates_vlm', *args]

    def inspect(self):
        result = self.command(self.check('--action', 'inspect', '--require-ready', '--timeout', '10'),
                              timeout=45)
        (self.logs / 'inspect.log').write_text(result.stdout)
        print(result.stdout, flush=True)
        require_inputs(parse_report(result.stdout))

    def state(self):
        return parse_state(self.ros('topic', 'echo', '/vlm_nav/go2_bridge_state',
            'std_msgs/msg/String', '--once', '--field', 'data', '--no-daemon',
            '--qos-reliability', 'reliable', '--qos-durability', 'transient_local', timeout=10).stdout)

    def service(self, action):
        result = self.ros('service', 'call', f'/twist_to_go2_sport_bridge/{action}',
                          'std_srvs/srv/Trigger', '{}', timeout=10)
        print(result.stdout, flush=True)
        require_service_success(result.stdout)

    def start(self):
        self.check_owners()
        for package, output in [('vlm_nav', 'GO2_BRIDGE_OUTPUT'),
                                ('multi_agent_nav', 'GO2_EXPLORE_OUTPUT')]:
            expected = Path(os.environ[output]) / 'install' / package
            actual = Path(self.ros('pkg', 'prefix', package).stdout.strip())
            if actual != expected:
                raise RuntimeError(f'{package} 安装不匹配：{actual}；应为 {expected}')
        self.command(self.check('--offline', '--require-ready'))
        self.camera()
        counts = [self.publishers(t) for t in ['/lidar_points', '/body_imu', '/odometry']]
        if counts == [0, 0, 0]:
            if any(n in self.nodes() for n in ['/amcl', '/map_server', '/controller_server', '/lio_mapping']):
                raise RuntimeError('已有不完整底层；请在原终端排障，不重复启动')
            if self.publishers('/lowstate') != 1:
                raise RuntimeError('/lowstate 必须有唯一真实发布者')
            self.confirm('START', '冷启动：确认 Go2 按原流程站稳、预检期间保持静止、急停人员在场。')
            report = self.logs / 'preflight.json'
            env = dict(os.environ, GO2_PREFLIGHT_REPORT=str(report))
            subprocess.run(['bash', str(self.repo / 'scripts/run_go2_preflight.sh')], env=env, check=True)
            self.window('System', ['ros2', 'launch', 'vlm_nav', 'go2_system.launch.py',
                f'sensor_preflight_report:={report}',
                f'sensor_preflight_config:={self.repo}/config/go2_preflight_xt16.yaml',
                f'calibration_file:={self.repo}/config/go2_calibration_xt16.yaml',
                f'spark_config_file:={self.repo}/config/spark_fast_lio_go2_xt16.yaml',
                'target_stage:=nav2', 'localization_mode:=amcl',
                f'map:={self.repo}/maps/lab_rescan_20261018_155656_0pa6gm/lab.yaml',
                'amcl_scan_topic:=/scan', 'start_bridge:=false',
                f'costmap_overrides_file:={self.config}/nav2_go2_cooperative_costmap.yaml'])
        elif counts != [1, 1, 1]:
            raise RuntimeError(f'底层不完整或发布者重复 {counts}；请在原终端排障')
        else:
            print('复用已有底层；后续检查地图、定位和 Nav2。', flush=True)
        if '/rviz2' not in self.nodes():
            self.window('RViz', ['rviz2', '-d', self.repo / 'config/go2_exploration.rviz'])
        self.confirm('READY', '在 RViz 等待地图；仅 AMCL 未初始化时，用 2D Pose Estimate 设置真实位姿。\n'
                     '已有有效定位不要重置。定位完成后继续。')
        self.ros('topic', 'echo', '/vlm_nav/nav_ready', 'std_msgs/msg/Bool', '--once',
                 '--filter', 'm.data', '--no-daemon', '--qos-reliability', 'reliable',
                 '--qos-durability', 'transient_local', timeout=120)
        for node, key, expected in [
            ('/global_costmap/global_costmap', 'inflation_layer.inflation_radius', .45),
            ('/local_costmap/local_costmap', 'inflation_layer.inflation_radius', .45),
            ('/controller_server', 'general_goal_checker.xy_goal_tolerance', .25),
            ('/planner_server', 'GridBased.tolerance', .5)]:
            text = self.ros('param', 'get', node, key, timeout=10).stdout
            match = re.search(r'Double value is:\s*(\S+)', text)
            if not match or float(match[1]) != expected:
                raise RuntimeError(f'{node} {key} 应为 {expected}，实际：{text.strip()}')
        bridges = self.nodes().count('/twist_to_go2_sport_bridge')
        if bridges > 1:
            raise RuntimeError('Sport 转换节点重复，停止后续操作')
        if bridges:
            self.confirm('CURRENT', '复用已有 Sport 转换：确认它来自当前 manual-takeover 安装；无法确认则退出并在原终端处理。')
        else:
            self.window('Sport', ['ros2', 'run', 'vlm_nav', 'twist_to_go2_sport_bridge',
                '--ros-args', '--params-file', self.repo / 'config/go2_bridge.yaml', '-p', 'dry_run:=false'])
            time.sleep(2)
        print('Sport 状态：' + self.state(), flush=True)
        self.inspect()

    def explore(self):
        env = os.environ
        if not ((env.get('DASHSCOPE_API_KEY') or env.get('OPENAI_API_KEY'))
                and (env.get('DASHSCOPE_BASE_URL') or env.get('OPENAI_BASE_URL') or env.get('DASHSCOPE_WORKSPACE_ID'))
                and (env.get('DASHSCOPE_MODEL') or env.get('OPENAI_MODEL'))):
            raise RuntimeError('缺少 API key / endpoint / model 环境变量；不打印或写入密钥')
        self.confirm('ARM', f'将执行一次最长300秒的探索并接近“{self.target}”。\n'
                     '确认已有运动授权、机器人停稳、急停人员在场、其他目标 owner 已退出，遥控静默至少2秒。')
        self.check_owners()
        self.inspect()
        state = self.state()
        if state == 'FAULT':
            self.confirm('RESET', '先消除故障并确认静止；确认允许 reset_fault。')
            self.service('reset_fault')
            if self.state() != 'DISARMED':
                raise RuntimeError('reset 后未进入 DISARMED')
            state = 'DISARMED'
        if state == 'DISARMED':
            self.service('arm')
        if self.state() != 'ARMED':
            raise RuntimeError('未确认 ARMED，不发送探索请求')
        log = self.logs / 'explore.log'
        print(f'探索日志：{log}；Ctrl+C 请求取消并等待停车。', flush=True)
        with log.open('w') as stream:
            process = subprocess.Popen([str(x) for x in self.check('--action', 'explore', '--allow-motion',
                '--target', self.target, '--approach-target', '--timeout', '300')],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            try:
                for line in process.stdout:
                    stream.write(line)
                    stream.flush()
                    print(line, end='', flush=True)
                process.wait()
            except KeyboardInterrupt:
                # Forward one interrupt so repeated Ctrl+C cannot interrupt the child's stop handler.
                previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
                try:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    tail, _ = process.communicate(timeout=30)
                except subprocess.TimeoutExpired:
                    raise RuntimeError('取消未完成；先现场急停，不终止底层或删除运动锁')
                finally:
                    signal.signal(signal.SIGINT, previous)
                stream.write(tail)
                print(tail, end='', flush=True)
        report = parse_report(log.read_text())
        if report.get('stop_confirmed') is not True:
            raise RuntimeError('未确认动作终态及实际停车；先现场急停，按原流程排障，不自动 DISARM 或重启')
        self.service('disarm')
        if self.state() != 'DISARMED':
            raise RuntimeError('DISARM 未确认，请按原流程处理')
        if (process.returncode or report.get('result') != 'TARGET_APPROACHED'
                or report.get('evidence_write_failed') is not False
                or report.get('vlm_evidence_pending') is not False):
            raise RuntimeError('本次探索未完整成功；已确认停车并 DISARM，查看实验 result.json 和日志')
        print('TARGET_APPROACHED；停车已确认，Sport 已 DISARMED。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='灭火器', help='A目标描述，默认灭火器')
    parser.add_argument('--prepare-only', action='store_true', help='仅启动并检查；不 ARM、不探索')
    args = parser.parse_args()
    if not args.target.strip():
        parser.error('目标描述不能为空')
    repo = Path(__file__).resolve().parents[1]
    if os.environ.get('REPO') != str(repo):
        parser.error('请通过 scripts/start_go2_vlm.sh 加载当前环境')
    if not sys.stdin.isatty() or not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
        parser.error('需要控制电脑桌面的交互式 Bash 终端及 RViz 显示环境')
    if not shutil.which('gnome-terminal'):
        parser.error('需要已有 gnome-terminal；未自动安装依赖')
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    with open('/tmp/go2-single-vlm-start.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('已有一键启动入口运行')
        app = Launcher(repo, args.prepare_only, args.target)
        root = repo / 'build/go2-field'
        root.mkdir(parents=True, exist_ok=True)
        app.logs = Path(tempfile.mkdtemp(prefix='single-vlm.', dir=root))
        print(f'本次检查日志：{app.logs}', flush=True)
        try:
            app.start()
            if not args.prepare_only:
                app.explore()
            return 0
        except (RuntimeError, OSError, ValueError, subprocess.SubprocessError, EOFError, KeyboardInterrupt) as error:
            print(f'停止：{error or "用户中断"}。不自动重试、重启或清理运动锁。', file=sys.stderr)
            print('若已经 ARM/运动而未确认 DISARM，请先确认停车，必要时现场急停，再按原指引处理。', file=sys.stderr)
            return 1
        finally:
            print('已有及本次打开的相机/底层/RViz/Sport终端保留。结束整套流程前确认停稳和 DISARM，\n'
                  '然后依次关闭本次 Sport、RViz、System，最后 Camera SSH；复用的进程由原终端管理。', flush=True)


if __name__ == '__main__':
    sys.exit(main())
