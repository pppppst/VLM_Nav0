#!/usr/bin/env python3
"""Go2 cooperative startup with staged controls and an automatic full-run entry."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

from start_go2_vlm import Launcher, parse_report


def idle_issues(message, robot, identity):
    expected = dict(schema_version=1, robot_id=robot, simulation=False,
        ready=False, deposited=False, exited=False, localization_valid=True, nav_ready=True,
        stopped=True, common_frame_id='', alignment_id='', task_id='', command_id='',
        goal_id='', reason='', sequence=0, reply='', pose_valid=False, resource_id='', token='',
        permit_command_id='', resource_status='', requested_resource='', requested_command_id='',
        local_map_id=identity.get('local_map_id'), map_version=identity.get('map_version'))
    issues = [f'{name}={getattr(message,name)!r}' for name,value in expected.items()
              if getattr(message,name) != value]
    if message.run_state not in ('WAITING_PEER','STANDBY'):
        issues.append(f'run_state={message.run_state!r}')
    if any(getattr(message.common_pose, field) != 0. for field in ('x','y','theta')):
        issues.append('common_pose must be zero in private mode')
    if not identity.get('local_map_id') or not identity.get('map_version'):
        issues.append('configured map identity missing')
    if not math.isfinite(message.stamp) or message.stamp <= 0:
        issues.append('invalid stamp')
    return issues


def wait_idle(config, bridge_name, timeout=None):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from multi_agent_task_msgs.msg import AgentState
    from check_multi_agent_comm import publisher_issues
    identities = config['alignment']['robots']
    lost = config['scene']['lost_timeout']
    latest, counts = {}, dict(go2=0, ranger=0)
    node = None
    try:
        rclpy.init(args=[])
        node = rclpy.create_node(f'go2_live_start_check_{os.getpid()}',
                                enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        def receive(message, robot):
            now = time.monotonic()
            previous = latest.get(robot)
            if idle_issues(message, robot, identities[robot]):
                counts[robot] = 0
            elif previous and now-previous[1] <= lost and message.stamp > previous[0].stamp:
                counts[robot] += 1
            else:
                counts[robot] = 1
            latest[robot] = (message, now)
        for robot in counts:
            node.create_subscription(AgentState, f'/multi_agent/{robot}/state',
                lambda message,robot=robot:receive(message,robot), qos)
        deadline = time.monotonic()+timeout if timeout is not None else None
        next_report = time.monotonic()
        errors = []
        while deadline is None or time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            errors = []
            for robot in counts:
                sample = latest.get(robot)
                if not sample:
                    errors.append(f'{robot}: no state received')
                    continue
                errors += [f'{robot}: {e}' for e in idle_issues(sample[0],robot,identities[robot])]
                if counts[robot] < 3 or time.monotonic()-sample[1] > lost:
                    errors.append(f'{robot}: need 3 fresh advancing WAITING_PEER/STANDBY states')
                expected = 'multi_agent_go2_agent' if robot == 'go2' else bridge_name
                errors += [f'{robot}: {e}' for e in publisher_issues(
                    node.get_publishers_info_by_topic(f'/multi_agent/{robot}/state'),
                    expected, bridge=robot=='ranger')]
            if not errors:
                return {robot:sample[0] for robot,sample in latest.items()}
            if time.monotonic() >= next_report:
                print('等待真实待命（Ctrl+C 可退出）：'+'; '.join(errors),flush=True)
                next_report = time.monotonic()+5
        raise RuntimeError('真实待命未就绪：'+'; '.join(errors))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class TaskLauncher(Launcher):
    def cooperative(self, action):
        result = self.command(['bash',self.repo/'scripts/cooperative.sh','go2',action], timeout=30)
        (self.logs/(action+'.log')).write_text(result.stdout)
        return result

    def preflight(self):
        report = parse_report(self.cooperative('preflight').stdout)
        if report['live_gates'] or not report['api_environment_complete']:
            raise RuntimeError('live预检未通过；不启动真实组件，不修改验证标志')
        print(f"场景={report['scenario_id']} 目标={report['targets']}", flush=True)
        print('live_gates=[]：跳过 PAUSED 专用采样，保留真实任务 bridge/agent。', flush=True)
        return report

    def component(self, action, name):
        def matches():
            return [node for node in self.nodes() if node=='/'+name or node.startswith('/'+name+'_')]
        if matches():
            raise RuntimeError(f'已有 {name}，请核对原窗口；不重复启动或自动结束旧进程')
        self.window(action, ['bash',self.repo/'scripts/cooperative.sh','go2',action])
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            nodes = matches()
            if len(nodes) == 1:
                return
            if len(nodes) > 1:
                raise RuntimeError(f'{name} 重复')
            time.sleep(.2)
        raise RuntimeError(f'{name} 未出现；查看对应组件窗口')

    def idle(self, report):
        from multi_agent_nav.config import load_config
        states = wait_idle(load_config(self.config), report['bridge'])
        fields = ('robot_id','stamp','local_map_id','map_version','run_state','localization_valid','nav_ready','stopped')
        summary = {robot:{field:getattr(message,field) for field in fields} for robot,message in states.items()}
        (self.logs/'idle.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2), flush=True)

    def prepare(self):
        report = self.preflight()
        self.check_owners()
        self.component('bridge',report['bridge'])
        self.confirm('ARM','确认Go2硬件准备已通过、本机停稳、无其他目标拥有者、急停可用，\n'
                     '允许使能Go2 Sport；本机随后独立准备并等待Ranger，此步不开始任务。')
        state = self.state()
        if state == 'FAULT':
            raise RuntimeError('Sport 为 FAULT；按原流程处理，不自动 reset')
        self.ros('topic','echo','/vlm_nav/nav_ready','std_msgs/msg/Bool','--once','--filter','m.data',
                 '--no-daemon','--qos-reliability','reliable','--qos-durability','transient_local',timeout=10)
        if state == 'DISARMED':
            self.service('arm')
        if self.state() != 'ARMED':
            raise RuntimeError('Sport 未确认 ARMED，不启动真实agent')
        self.component('agent','multi_agent_go2_agent')
        self.idle(report)
        self.component('coordinator','multi_agent_coordinator')
        self.window('当前状态',['bash',self.repo/'scripts/cooperative.sh','go2','watch'])
        print('COOPERATIVE_READY：两端已独立准备并通过新鲜状态核对；任务尚未开始。',flush=True)

    def start_task(self):
        report = self.preflight()
        count = self.nodes().count('/multi_agent_coordinator')
        if count != 1:
            raise RuntimeError(f'coordinator数量={count}，要求1；核对prepare的coordinator窗口，不重复启动已有组件')
        state = self.state()
        if state != 'ARMED':
            raise RuntimeError(f'Sport状态={state}，要求ARMED；回查prepare及Sport窗口，start不自动ARM或reset')
        self.confirm('START','确认两端配置/目标一致，Ranger端peer也收到Go2新鲜WAITING_PEER/STANDBY，实体及目标已就位，\n'
                     '具备本轮运动授权且急停可用；将开始一次真实双机任务。')
        self.idle(report)
        print(self.cooperative('start').stdout,flush=True)

    def pause_task(self):
        print(self.cooperative('pause').stdout,flush=True)
        print('暂停请求已受理；仍须从两端state/原诊断确认未完成任务PAUSED（全局已完成保持IDLE/SUCCEEDED）、实际停稳及原生动作终止。',flush=True)

    def disarm_task(self):
        self.confirm('STOPPED','先按原文暂停并确认两机实际停稳、原生动作终止，正常结束agent/coordinator/bridge；现在仅解除Go2 Sport使能。')
        self.service('disarm')
        if self.state() != 'DISARMED':
            raise RuntimeError('未确认 DISARMED，请按原流程处理')
        print('Sport=DISARMED',flush=True)

    def restart(self):
        self.pause_task()
        self.confirm('CLOSED','先确认未完成任务两端 PAUSED（全局已完成保持 IDLE/SUCCEEDED）、实体停稳及原生导航/恢复动作终止，再在原窗口按顺序 Ctrl+C：\n'
                     '两端 agents → Go2 coordinator → 两端任务 bridges。等待各进程正常结束。\n'
                     '保留 Go2 Camera SSH、System、RViz、Sport 和 Ranger 健康底层；本脚本不操作 Ranger。\n'
                     '旧任务组件退出完成后继续；Ranger 随后由操作者在本机检查并重启任务 bridge/agent。')
        self.disarm_task()
        self.ros('topic','echo','/vlm_nav/nav_ready','std_msgs/msg/Bool','--once','--filter','m.data',
                 '--no-daemon','--qos-reliability','reliable','--qos-durability','transient_local',timeout=120)
        self.inspect()
        self.prepare()
        self.start_task()


class AutomaticLauncher(TaskLauncher):
    def confirm(self, word, message):
        if word not in ('CAMERA','START','READY','CURRENT','ARM'):
            raise RuntimeError(f'自动入口不处理 {word}，请按原流程处理')
        if word == 'CURRENT':
            for key,value in [('dry_run','False'),('require_system_ready','True'),
                              ('fault_on_foreign_sport_request','False')]:
                output = self.ros('param','get','/twist_to_go2_sport_bridge',key,timeout=10).stdout.strip()
                if output != 'Boolean value is: '+value:
                    raise RuntimeError(f'已有Sport参数 {key} 不符合当前profile：{output}')
        if word == 'READY':
            print('等待真实定位/Nav2就绪；未定位时请在RViz设置实际初始位姿，无需输入READY。',flush=True)
        else:
            print(f'自动继续阶段 {word}；后续实际就绪检查仍须通过。',flush=True)

    def run(self):
        print('完整自动流程：检查通过后会ARM并开始一轮真实任务，无需输入确认词。',flush=True)
        self.preflight()
        Launcher.start(self)
        self.prepare()
        self.start_task()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','start','pause','disarm','restart','run'),default='prepare',nargs='?')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    if os.getenv('REPO') != str(repo) or os.getenv('COOP_SIDE') != 'go2' or not os.getenv('COOP_CONFIG'):
        parser.error('请通过 scripts/start_go2_cooperative_task.sh 加载协同环境')
    if args.action != 'pause' and not sys.stdin.isatty():
        parser.error('该动作需要控制电脑的交互式终端')
    if args.action in ('prepare','restart','run') and (not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')) or not shutil.which('gnome-terminal')):
        parser.error('prepare/restart/run 需要图形桌面及已有 gnome-terminal')
    cls = AutomaticLauncher if args.action == 'run' else TaskLauncher
    app = cls(repo,True,'',cooperative_hardware=True)
    root = repo/'build/go2-field'
    root.mkdir(parents=True,exist_ok=True)
    app.logs = Path(tempfile.mkdtemp(prefix='cooperative-task.',dir=root))
    print(f'本次日志：{app.logs}',flush=True)
    try:
        with open('/tmp/go2-cooperative-task-prepare.lock','a') as lock:
            if args.action in ('prepare','restart','run'):
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if args.action == 'run':
                with open('/tmp/go2-single-vlm-start.lock','a') as hardware_lock:
                    fcntl.flock(hardware_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    app.run()
            else:
                {'prepare':app.prepare,'start':app.start_task,'pause':app.pause_task,'disarm':app.disarm_task,
                 'restart':app.restart}[args.action]()
        return 0
    except (RuntimeError,OSError,ValueError,KeyError,subprocess.SubprocessError,EOFError,KeyboardInterrupt) as error:
        print(f'停止：{error}。不自动重试、结束旧进程或DISARM。',file=sys.stderr)
        print('组件窗口保留；已ARM或start结果不明时，按原文暂停并确认停稳，不盲目重复start。',file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
