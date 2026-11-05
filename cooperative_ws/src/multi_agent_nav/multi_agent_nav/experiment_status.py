"""Best-effort experiment status file and independent, read-only terminal viewer."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


HEALTH = {'nav_ready': '导航就绪', 'localization_valid': '定位有效',
          'stopped': '已停稳', 'motion_terminal': '运动动作已结束',
          'rgbd_ready': 'RGB-D就绪', 'control_armed': '控制已解锁',
          'costmap_valid': '代价地图有效'}
FIELDS = ('state', 'phase', 'reason', 'run_elapsed_s', 'pause_elapsed_s',
          'paused_total_s', 'run_remaining_s', 'health', 'result', 'stop_confirmed')
FINAL = ('FINISHED', 'FAILED', 'CANCELLED', 'STOP_UNCONFIRMED')


class ExperimentStatus:
    def __init__(self, directory: Path, clock=time.monotonic):
        self.path = Path(directory).resolve() / 'status.json'
        self.clock = clock
        self.snapshot = {}

    def update(self, **fields):
        temporary = None
        try:
            snapshot = dict(self.snapshot, **{key: fields[key] for key in FIELDS if key in fields})
            if isinstance(snapshot.get('health'), dict):
                snapshot['health'] = {key: value for key, value in snapshot['health'].items()
                                      if key in HEALTH and isinstance(value, bool)}
            snapshot['updated_monotonic'] = self.clock()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix='.status-', delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(snapshot, stream, ensure_ascii=False, allow_nan=False)
            os.replace(temporary, self.path)
            self.snapshot = snapshot
            return True
        except Exception:
            # Diagnostics must never interrupt navigation or the stop sequence.
            return False
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass

    def launch(self):
        try:
            subprocess.Popen(['gnome-terminal', '--window', '--title=Go2 实验状态', '--',
                              sys.executable, str(Path(__file__).resolve()), '--watch', str(self.path)],
                             shell=False, start_new_session=True,
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:
            return False


def _text(value):
    return ''.join(c for c in value if c.isprintable())[:240] if isinstance(value, str) else '未知'


def _number(value):
    try:
        return float(value) if type(value) in (int, float) and math.isfinite(value) else None
    except OverflowError:
        return None


def _boolean(value):
    return '是' if value is True else '否' if value is False else '未知'


def format_status(snapshot, now):
    snapshot = snapshot if isinstance(snapshot, dict) else {}
    state = snapshot.get('state')
    updated = _number(snapshot.get('updated_monotonic'))
    age = max(0., now - updated) if updated is not None else float('inf')
    stale = age > 3. and state not in FINAL
    delta = 0. if stale else age
    lines = ['Go2 实验状态（只读；关闭窗口不影响实验）',
             '状态：' + _text(state), '阶段：' + _text(snapshot.get('phase')),
             '等待原因：' + _text(snapshot.get('reason'))]
    if stale:
        lines.append('状态已过期：计时冻结，等待执行器更新')
    for key, label in (('run_elapsed_s', '运行时间'), ('pause_elapsed_s', '本次暂停'),
                       ('paused_total_s', '累计暂停'), ('run_remaining_s', '剩余运行')):
        value = _number(snapshot.get(key))
        if value is not None:
            if state == 'RUNNING' and key in ('run_elapsed_s', 'run_remaining_s'):
                value += delta if key == 'run_elapsed_s' else -delta
            elif state == 'WAITING' and key in ('pause_elapsed_s', 'paused_total_s'):
                value += delta
        lines.append(label + '：' + (f'{max(0., value):.1f} 秒' if value is not None else '未知'))
    health = snapshot.get('health')
    health = health if isinstance(health, dict) else {}
    lines.extend(label + '：' + _boolean(health.get(key)) for key, label in HEALTH.items())
    lines.extend(('最终结果：' + _text(snapshot.get('result')),
                  '停车确认：' + _boolean(snapshot.get('stop_confirmed'))))
    return '\n'.join(lines)


def watch(path):
    snapshot = {}
    try:
        while True:
            try:
                value = json.loads(Path(path).read_text(encoding='utf-8'))
                if isinstance(value, dict):
                    snapshot = value
            except (OSError, ValueError):
                pass
            print('\033[2J\033[H' + (format_status(snapshot, time.monotonic()) if snapshot
                                    else '等待状态文件……'), flush=True)
            time.sleep(.5)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', required=True, type=Path)
    watch(parser.parse_args().watch)
