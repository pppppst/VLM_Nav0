"""Offline contract: fake search only, no ROS or model."""
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from tools.go2_agent.session import SearchSession


class FakeProcess:
    pid = 987654321
    returncode = None

    def poll(self):
        return self.returncode


def make_session(tmp_path, **kwargs):
    process = FakeProcess()
    calls = []
    target = ' 中文  "灭火器"; $(touch SHOULD_NOT_EXIST) '
    spec = dict(session_id='session-1', target=target, directory=str(tmp_path),
                argv=['/original/ros2', 'run', 'package', 'check', '--target', target],
                ros_env={'PATH': '/original/ros/bin', 'API_KEY': 'secret-never-persist'})
    def spawn(*args, **options):
        calls.append((args, options))
        return process
    session = SearchSession(spec, popen=spawn, identity=lambda pid: (pid, 'birth'), **kwargs)
    return session, process, calls, spec


def finish(session, process, *, code=0, **changes):
    out = session.output_dir
    out.mkdir(exist_ok=True)
    report = dict(mode='local_go2_explore', vlm_evidence_root=str(out/'vlm'),
                  result='TARGET_APPROACHED', stop_confirmed=True,
                  evidence_write_failed=False, vlm_evidence_pending=False,
                  native_status='CANCELED')
    report.update(changes)
    event = dict(run_id='original-run', task_id='original-task', command_id='original-command',
                 sequence=1, wall_time=session.started_at, target=session.target,
                 event='SESSION_END', state='STOPPED', result=report.get('result'),
                 stop_confirmed=report.get('stop_confirmed'))
    (out/'events.jsonl').write_text(json.dumps(event)+'\n')
    (out/'result.json').write_text(json.dumps(report))
    process.returncode = code
    return report


def test_concurrent_start_exact_argv_and_no_replay(tmp_path):
    s, p, calls, spec = make_session(tmp_path)
    with ThreadPoolExecutor(8) as pool:
        results = list(pool.map(s.start, [s.target]*24))
    assert len(calls) == 1
    assert len({r['task_id'] for r in results}) == 1
    args, options = calls[0]
    assert args[0] == spec['argv'] + ['--output-dir', str(s.output_dir)]
    assert options['shell'] is False and options['start_new_session'] is True
    assert options['env'] == spec['ros_env']
    with pytest.raises(ValueError): s.start(s.target.strip())
    finish(s, p)
    assert s.status(s.task_id)['state'] == 'SUCCEEDED'
    assert s.start(s.target)['state'] == 'SUCCEEDED'
    restored = SearchSession(spec, popen=lambda *a, **k: pytest.fail('replay'))
    assert restored.start(s.target)['state'] == 'SUCCEEDED'
    assert 'secret-never-persist' not in (tmp_path/'manifest.json').read_text()


@pytest.mark.parametrize('change,expected', [
    ({'result': 'FAILED'}, 'FAILED'), ({'stop_confirmed': False}, 'STOP_UNCONFIRMED'),
    ({'stop_confirmed': None}, 'STOP_UNCONFIRMED'),
    ({'evidence_write_failed': True}, 'FAILED'), ({'evidence_write_failed': None}, 'FAILED'),
    ({'vlm_evidence_pending': True}, 'FAILED'), ({'vlm_evidence_pending': None}, 'FAILED'),
    ({'code': 1}, 'FAILED'), ({}, 'SUCCEEDED'),
])
def test_success_requires_all_original_facts(tmp_path, change, expected):
    s, p, calls, _ = make_session(tmp_path)
    s.start(s.target); finish(s, p, **change)
    snap = s.status(s.task_id)
    assert snap['state'] == expected
    assert snap['original_run_id'] == 'original-run'
    assert snap['original_task_id'] == 'original-task'
    assert snap['original_command_id'] == 'original-command'
    assert snap['host_cleanup'] == 'PENDING'
    s.start(s.target)
    assert len(calls) == 1


def test_partial_files_do_not_finish_running_process(tmp_path):
    s, p, _, _ = make_session(tmp_path); s.start(s.target)
    s.output_dir.mkdir()
    (s.output_dir/'events.jsonl').write_text('{"run_id":')
    (s.output_dir/'result.json').write_text('{')
    assert s.status(s.task_id)['state'] == 'RUNNING'
    assert s.status(s.task_id)['original_run_id'] is None
    finish(s, p)
    assert s.status(s.task_id)['state'] == 'SUCCEEDED'


@pytest.mark.parametrize('corruption', ['missing', 'json', 'target', 'old-time', 'root', 'run', 'missing-end'])
def test_missing_corrupt_or_foreign_report_is_not_trusted(tmp_path, corruption):
    s, p, _, _ = make_session(tmp_path); s.start(s.target);finish(s, p)
    rp, ep = s.output_dir/'result.json', s.output_dir/'events.jsonl'
    if corruption == 'missing': rp.unlink()
    elif corruption == 'json': rp.write_text('{')
    elif corruption == 'root':
        data=json.loads(rp.read_text());data['vlm_evidence_root']='/old-session/vlm';rp.write_text(json.dumps(data))
    else:
        row=json.loads(ep.read_text())
        if corruption=='target':row['target']='different'
        if corruption=='old-time':row['wall_time']=0
        if corruption=='missing-end':row['event']='START'
        if corruption=='run':ep.write_text(json.dumps(dict(row,run_id='different',event='START'))+'\n'+json.dumps(dict(row,sequence=2))+'\n')
        else:ep.write_text(json.dumps(row)+'\n')
    assert s.status(s.task_id)['state'] == 'STOP_UNCONFIRMED'


def test_spawn_failure_consumes_session(tmp_path):
    s, _, _, spec = make_session(tmp_path)
    def fail(*a, **k): raise OSError('unavailable')
    s._popen=fail
    assert s.start(s.target)['state']=='FAILED'
    restored=SearchSession(spec,popen=lambda *a,**k:pytest.fail('replay'))
    assert restored.start(s.target)['state']=='FAILED'


def test_reloaded_active_session_never_signals_or_restarts(tmp_path):
    s, _, _, spec=make_session(tmp_path);s.start(s.target)
    restored=SearchSession(spec,popen=lambda *a,**k:pytest.fail('replay'))
    assert restored.start(s.target)['state']=='STOP_UNCONFIRMED'
    assert restored.close()['state']=='STOP_UNCONFIRMED'


def test_final_events_written_during_exit_poll_are_read(tmp_path):
    s,p,_,_=make_session(tmp_path);s.start(s.target)
    def poll():
        finish(s,p)
        return 0
    p.poll=poll
    assert s.status(s.task_id)['state']=='SUCCEEDED'


def test_cancel_before_start_consumes_without_spawning(tmp_path):
    s,_,calls,spec=make_session(tmp_path)
    assert s.cancel(s.task_id)['state']=='CANCELED'
    assert s.start(s.target)['state']=='CANCELED'
    restored=SearchSession(spec,popen=lambda *a,**k:pytest.fail('canceled replay'))
    assert restored.start(s.target)['state']=='CANCELED'
    assert not calls
