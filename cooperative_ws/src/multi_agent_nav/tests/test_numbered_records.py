"""Sequential evidence directories preserve existing artifacts and task identities."""
from concurrent.futures import ThreadPoolExecutor
import json

import pytest


def test_numbered_directories_continue_after_restart_and_leave_legacy_data(tmp_path):
    from multi_agent_nav.experiment_records import numbered_directory
    legacy=tmp_path/'deadbeef_view_00';legacy.mkdir();(legacy/'response.txt').write_text('keep')
    first=numbered_directory(tmp_path)
    assert first.name=='0001'
    (first/'response.txt').write_text('first')
    (tmp_path/'0003').write_text('occupied filename')
    second=numbered_directory(tmp_path)
    assert second.name=='0004'
    assert (first/'response.txt').read_text()=='first'
    assert (legacy/'response.txt').read_text()=='keep'
    assert (tmp_path/'0003').read_text()=='occupied filename'


def test_concurrent_directory_allocation_never_reuses_a_number(tmp_path):
    from multi_agent_nav.experiment_records import numbered_directory
    with ThreadPoolExecutor(max_workers=8) as workers:
        directories=list(workers.map(lambda _:numbered_directory(tmp_path),range(16)))
    assert {p.name for p in directories}=={f'{i:04d}' for i in range(1,17)}
    assert all(p.is_dir() for p in directories)


def test_numbered_directory_does_not_follow_occupied_child_symlink(tmp_path):
    from multi_agent_nav.experiment_records import numbered_directory
    external=tmp_path/'outside';external.mkdir()
    root=tmp_path/'records';root.mkdir();(root/'0001').symlink_to(external,target_is_directory=True)
    assert numbered_directory(root).name=='0002'
    assert list(external.iterdir())==[]


def test_scan_records_use_sequential_directories_but_keep_scan_identity(tmp_path):
    from multi_agent_nav.scan_evidence import ScanEvidence
    records=[ScanEvidence(tmp_path,dict(scan_id='task/scan/hash',view_index=0,attempt=i)) for i in range(2)]
    for evidence in records:evidence.begin()
    assert [e.directory.name for e in records]==['0001','0002']
    for i,evidence in enumerate(records):
        metadata=json.loads((evidence.directory/'metadata.json').read_text())
        assert metadata['scan_id']=='task/scan/hash' and metadata['attempt']==i


def test_numbered_scan_directory_failure_is_reported_without_touching_existing_file(tmp_path):
    from multi_agent_nav.scan_evidence import ScanEvidence
    root=tmp_path/'file';root.write_text('keep')
    evidence=ScanEvidence(root,dict(scan_id='scan',view_index=0))
    with pytest.raises(OSError,match='directory creation failed'):evidence.begin()
    assert evidence.write_failed and root.read_text()=='keep'
