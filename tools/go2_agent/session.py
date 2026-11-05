"""One search per durable session; original executor owns all robot behavior."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time


def write_json(path, data):
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[2]), fields[19]  # PGID and Linux process start ticks.
    except (OSError, ValueError, IndexError):
        return None


class SearchSession:
    def __init__(self, spec, *, popen=subprocess.Popen, identity=process_identity,
                 killpg=os.killpg, clock=time.monotonic, sleep=time.sleep):
        self.target = spec['target']
        if not isinstance(self.target, str) or not self.target.strip() or '\0' in self.target:
            raise ValueError('nonempty target without NUL required')
        self.directory = Path(spec['directory']).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.output_dir = self.directory/'search'
        self.task_id = spec['session_id'] + '/find-target'
        self.argv = list(spec['argv'])
        if (not all(isinstance(x, str) for x in self.argv)
                or self.argv.count('--target') != 1
                or self.argv[self.argv.index('--target') + 1:] == []
                or self.argv[self.argv.index('--target') + 1] != self.target
                or '--output-dir' in self.argv):
            raise ValueError('host argv must contain exact target and no output override')
        self.env = dict(spec['ros_env'])
        self._popen, self._identity, self._killpg = popen, identity, killpg
        self._clock, self._sleep = clock, sleep
        self._lock = threading.RLock()
        self._process = self._stream = None
        self._event_offset = 0
        self._event_error = False
        self._end = None
        self._closed = False
        self._cancel_deadline = None
        self._manifest = self.directory/'manifest.json'
        self.data = dict(session_id=spec['session_id'], task_id=self.task_id, target=self.target,
                         state='IDLE', consumed=False, active_process=False, pid=None, pgid=None,
                         process_birth=None, started_at=None, cancel_requested_at=None,
                         finished_at=None, signal_sent=False, original_phase=None,
                         original_run_id=None, original_task_id=None, original_command_id=None,
                         exit_code=None, result=None, stop_confirmed=None, stop_error=None,
                         evidence_write_failed=None, vlm_evidence_pending=None,
                         evidence_paths={'events': str(self.output_dir/'events.jsonl'),
                                         'report': str(self.output_dir/'result.json'),
                                         'log': str(self.directory/'search.log')},
                         original_report=None, report_error=None, host_cleanup='PENDING',
                         framework_error=None, execution={'argv': self.argv + ['--output-dir', str(self.output_dir)],
                             'environment': {key: self.env[key] for key in
                                 ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'COOP_SETUP_FILE', 'GO2_CONFIG')
                                 if key in self.env}})
        if self._manifest.exists():
            self._restore()

    @property
    def started_at(self):
        return self.data['started_at']

    def _restore(self):
        old = json.loads(self._manifest.read_text())
        for name in ('session_id', 'task_id', 'target'):
            if old[name] != self.data[name]:
                raise ValueError('session identity mismatch')
        self.data = old
        if old['consumed'] and old['active_process']:
            self.data['state'] = 'STOP_UNCONFIRMED'
            self.data['report_error'] = 'previous manager unavailable; no recovery or signal attempted'

    def _save(self):
        try:
            write_json(self._manifest, self.data)
        except OSError:
            # Evidence I/O must never prevent the owned process from receiving SIGINT.
            self.data['framework_error'] = 'PERSISTENCE_ERROR'

    def _snapshot(self):
        return json.loads(json.dumps(self.data))

    def start(self, target):
        with self._lock:
            if target != self.target:
                raise ValueError('target must exactly match the host target')
            if self._closed:
                return self._snapshot()
            if self.data['consumed']:
                return self.status(self.task_id)
            with (self.directory/'manager.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                if self._manifest.exists():
                    self._restore()
                    if self.data['consumed']:
                        return self._snapshot()
                if self.output_dir.exists():
                    raise ValueError('search output already exists; refusing stale evidence')
                # Persist consumption before spawn, including a possibly interrupted spawn.
                self.data.update(consumed=True, state='STARTING', active_process=True,
                                 started_at=time.time())
                write_json(self._manifest, self.data)
                try:
                    self._stream = (self.directory/'search.log').open('x')
                    self._process = self._popen(self.argv + ['--output-dir', str(self.output_dir)],
                        env=self.env, stdin=subprocess.DEVNULL, stdout=self._stream,
                        stderr=subprocess.STDOUT, shell=False, start_new_session=True)
                except Exception as error:
                    self.data.update(state='FAILED', active_process=False,
                                     framework_error='spawn: ' + type(error).__name__)
                    if self._stream:
                        self._stream.close()
                else:
                    identity = self._identity(self._process.pid)
                    self.data.update(pid=self._process.pid, pgid=identity[0] if identity else None,
                                     process_birth=identity[1] if identity else None, state='RUNNING')
                self._save()
                return self._snapshot()

    def _read_events(self):
        try:
            with (self.output_dir/'events.jsonl').open('rb') as stream:
                stream.seek(self._event_offset)
                for line in stream:
                    if not line.endswith(b'\n'):
                        break
                    self._event_offset += len(line)
                    try:
                        row = json.loads(line)
                        if (row.get('target') != self.target or not row.get('run_id')
                                or row.get('wall_time', 0) < self.started_at
                                or (self.data['original_run_id'] is not None
                                    and self.data['original_run_id'] != row['run_id'])):
                            raise ValueError('foreign event')
                        self.data['original_run_id'] = row['run_id']
                        for source, dest in [('state', 'original_phase'), ('task_id', 'original_task_id'),
                                             ('command_id', 'original_command_id')]:
                            if source in row:
                                self.data[dest] = row[source]
                        if row.get('event') == 'SESSION_END':
                            self._end = row
                    except (ValueError, TypeError, AttributeError):
                        self._event_error = True
        except FileNotFoundError:
            pass
        except OSError:
            self._event_error = True

    def status(self, task_id):
        with self._lock:
            if task_id != self.task_id:
                raise ValueError('unknown task_id')
            if self._process is None or self.data['finished_at'] is not None:
                return self._snapshot()
            self._read_events()
            code = self._process.poll()
            self.data['active_process'] = code is None
            if code is None:
                if self._cancel_deadline is not None and self._clock() >= self._cancel_deadline:
                    self.data.update(state='STOP_UNCONFIRMED', report_error='cancel wait expired; original lock retained')
                self._save()
                return self._snapshot()
            # Exit can race the first read; drain the now-complete final event file.
            self._read_events()
            self.data.update(exit_code=code, finished_at=time.time())
            if self._stream:
                self._stream.close()
            try:
                report = json.loads((self.output_dir/'result.json').read_text())
                if not isinstance(report, dict):
                    raise ValueError('report is not an object')
                self.data['original_report'] = report
                for key in ('result', 'stop_confirmed', 'stop_error', 'evidence_write_failed', 'vlm_evidence_pending'):
                    self.data[key] = report.get(key)
                if (self._event_error or self._end is None
                        or report.get('mode') != 'local_go2_explore'
                        or report.get('vlm_evidence_root') != str(self.output_dir/'vlm')
                        or self._end.get('result') != report.get('result')
                        or self._end.get('stop_confirmed') is not report.get('stop_confirmed')):
                    raise ValueError('report source or final event mismatch')
            except (OSError, ValueError, TypeError) as error:
                self.data.update(state='STOP_UNCONFIRMED', report_error=type(error).__name__)
            else:
                if report.get('stop_confirmed') is not True:
                    state = 'STOP_UNCONFIRMED'
                elif self.data['cancel_requested_at'] is not None:
                    state = 'CANCELED'
                elif (code == 0 and report.get('result') == 'TARGET_APPROACHED'
                      and report.get('evidence_write_failed') is False
                      and report.get('vlm_evidence_pending') is False):
                    state = 'SUCCEEDED'
                else:
                    state = 'FAILED'
                self.data['state'] = state
            self._save()
            return self._snapshot()

    def cancel(self, task_id):
        with self._lock:
            self.status(task_id)
            if not self.data['consumed']:
                self._closed = True
                self.data.update(consumed=True, state='CANCELED', cancel_requested_at=time.time())
                self._save()
                return self._snapshot()
            if self._process is None or not self.data['active_process'] or self.data['cancel_requested_at'] is not None:
                return self._snapshot()
            self._cancel_deadline = self._clock() + 30
            self.data.update(state='CANCEL_REQUESTED', cancel_requested_at=time.time())
            self._save()
            actual = self._identity(self._process.pid)
            expected = (self.data['pgid'], self.data['process_birth'])
            if actual is None or actual != expected or actual[0] != self._process.pid:
                self.data.update(state='STOP_UNCONFIRMED', report_error='process identity unconfirmed')
            elif self._process.poll() is None:
                try:
                    self._killpg(actual[0], signal.SIGINT)
                    self.data['signal_sent'] = True
                except ProcessLookupError:
                    pass
                except OSError:
                    self.data.update(state='STOP_UNCONFIRMED', report_error='SIGINT delivery failed')
            self._save()
            return self._snapshot()

    def close(self):
        with self._lock:
            self._closed = True
            if self.data['consumed']:
                self.cancel(self.task_id)
        deadline = self._cancel_deadline if self._cancel_deadline is not None else self._clock() + 30
        while self._process is not None and self._process.poll() is None and self._clock() < deadline:
            self._sleep(min(.1, max(0, deadline - self._clock())))
        with self._lock:
            self.status(self.task_id)
            if self.data['active_process']:
                self.data.update(state='STOP_UNCONFIRMED', report_error='cancel wait expired; original lock retained')
            elif not self.data['consumed']:
                self.data.update(state='FAILED', framework_error=self.data['framework_error'] or 'AGENT_NO_EXECUTION')
            self._save()
            return self._snapshot()
