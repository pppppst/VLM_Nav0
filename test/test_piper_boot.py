import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('piper_boot', ROOT / 'scripts/go2_piper_boot.py')
boot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boot)


def arm(enabled=False):
    status = NS(ctrl_mode=0, arm_status=0, teach_status=0, err_code=0)
    motors = {}
    for i in range(1, 7):
        flags = dict.fromkeys(boot.FAULTS, False)
        flags['driver_enable_status'] = enabled
        motors['motor_' + str(i)] = NS(foc_status=NS(**flags))
    a = NS(GetArmStatus=lambda: NS(arm_status=status),
           GetArmLowSpdInfoMsgs=lambda: NS(**motors), calls=[])
    a.EnableArm = a.calls.append
    return a, status, motors


def seen(now=10):
    return {can_id: now for can_id in boot.FEEDBACK_IDS}


def test_requires_every_joint_fresh_not_only_aggregate_timestamp():
    a, _, _ = arm(True)
    stamps = seen()
    stamps[0x263] = 1
    assert boot.assess(a, stamps, 10)[0] == 'WAIT_FEEDBACK'
    del stamps[0x263]
    assert boot.assess(a, stamps, 10)[0] == 'WAIT_FEEDBACK'
    assert a.calls == []


@pytest.mark.parametrize('field', boot.FAULTS)
def test_each_motor_fault_waits_without_sending(field):
    a, _, motors = arm()
    setattr(motors['motor_2'].foc_status, field, True)
    assert boot.step(a, seen(), 10, readonly=False)[0] == 'WAIT_FAULT'
    assert a.calls == []


@pytest.mark.parametrize('field,value', [('err_code', 1), ('arm_status', 1), ('teach_status', 1), ('ctrl_mode', 2)])
def test_arm_fault_or_other_control_waits(field, value):
    a, status, _ = arm()
    setattr(status, field, value)
    assert boot.step(a, seen(), 10, readonly=False)[0].startswith('WAIT_')
    assert a.calls == []


def test_enable_only_when_healthy_and_confirm_with_new_feedback():
    a, _, motors = arm()
    stamps = seen()
    assert boot.step(a, stamps, 10, readonly=False)[0] == 'ENABLING'
    assert a.calls == [7]
    assert stamps == {}
    for motor in motors.values():
        motor.foc_status.driver_enable_status = True
    assert boot.step(a, stamps, 10.1, readonly=False)[0] == 'WAIT_FEEDBACK'
    assert boot.step(a, seen(11), 11, readonly=False)[0] == 'READY'
    assert a.calls == [7]


def test_inspection_never_enables_and_ready_never_reenables():
    a, _, _ = arm()
    assert boot.step(a, seen(), 10, readonly=True)[0] == 'DISABLED'
    assert a.calls == []
    a, _, _ = arm(True)
    assert boot.step(a, seen(), 10, readonly=False)[0] == 'READY'
    assert a.calls == []


def test_fault_recovery_can_proceed_without_clearing_errors():
    a, status, _ = arm()
    status.err_code = 1
    assert boot.step(a, seen(), 10, readonly=False)[0] == 'WAIT_FAULT'
    status.err_code = 0
    assert boot.step(a, seen(), 10, readonly=False)[0] == 'ENABLING'
    assert a.calls == [7]


def test_discover_matches_usb_identity_not_interface_name(tmp_path):
    usb = tmp_path / 'usb'
    usb.mkdir()
    for name, value in [('idVendor', '1d50'), ('idProduct', '606f'), ('serial', 'correct')]:
        (usb / name).write_text(value)
    (usb / 'interface').mkdir()
    net = tmp_path / 'net'
    (net / 'can9').mkdir(parents=True)
    (net / 'can9' / 'device').symlink_to(usb / 'interface')
    assert boot.discover('correct', net) == 'can9'
    with pytest.raises(RuntimeError, match='USB'):
        boot.discover('wrong', net)


def test_can_init_only_configures_down_device():
    commands = []
    link = {'flags': [], 'linkinfo': {'info_data': {'state': 'STOPPED'}}}
    def run(args, **kw):
        commands.append(args)
        return NS(stdout=json.dumps([link]))
    boot.prepare_can('can0', run=run)
    assert commands[1:] == [
        ['ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate', '1000000'],
        ['ip', 'link', 'set', 'can0', 'up'],
    ]
    commands.clear()
    link['flags'] = ['UP']
    link['linkinfo']['info_data'] = {'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 1000000}}
    boot.prepare_can('can0', run=run)
    assert len(commands) == 1
    link['linkinfo']['info_data']['bittiming']['bitrate'] = 500000
    with pytest.raises(RuntimeError, match='bitrate'):
        boot.prepare_can('can0', run=run)
    assert all('set' not in cmd for cmd in commands)


def test_inspect_does_not_bring_can_up():
    commands = []
    def run(args, **kw):
        commands.append(args)
        return NS(stdout=json.dumps([{'flags': []}]))
    with pytest.raises(RuntimeError, match='DOWN'):
        boot.prepare_can('can0', readonly=True, run=run)
    assert len(commands) == 1


def test_bus_off_is_reported_without_resetting_bus():
    commands = []
    def run(args, **kw):
        commands.append(args)
        return NS(stdout=json.dumps([{'flags': ['UP'], 'linkinfo': {'info_data': {
            'state': 'BUS-OFF', 'bittiming': {'bitrate': 1000000}}}}]))
    with pytest.raises(RuntimeError, match='BUS-OFF'):
        boot.prepare_can('can0', run=run)
    assert len(commands) == 1


def simulated_session(monkeypatch, malformed=False, queued_before_enable=False):
    a, _, motors = arm()
    clock = [0.0]
    received = [0]
    sent_at = [None]
    def enable(number):
        a.calls.append(number)
        sent_at[0] = clock[0]
    a.EnableArm = enable
    a.connections = []
    a.disconnections = []
    a.ConnectPort = lambda **kw: a.connections.append(kw)
    a.DisconnectPort = lambda: a.disconnections.append(True)
    def parse(msg):
        if 0x261 <= msg.arbitration_id <= 0x266 and a.calls:
            motors['motor_' + str(msg.arbitration_id - 0x260)].foc_status.driver_enable_status = True
    a.ParseCANFrame = parse
    class Bus:
        def __init__(self, **kw):
            assert len(kw['can_filters']) == 7
        def __enter__(self):
            return self
        def __exit__(self, *exc):
            pass
        def recv(self, timeout):
            clock[0] += 0.05
            can_id = boot.FEEDBACK_IDS[received[0] % 7]
            received[0] += 1
            stamp = (sent_at[0] - 0.01 if queued_before_enable and sent_at[0] is not None else clock[0])
            return NS(arbitration_id=can_id, timestamp=1000 + stamp,
                      is_error_frame=False, is_remote_frame=False, is_extended_id=False,
                      is_fd=False, dlc=7 if malformed else 8)
    monkeypatch.setitem(sys.modules, 'can', NS(Bus=Bus))
    monkeypatch.setitem(sys.modules, 'piper_sdk', NS(C_PiperInterface_V2=lambda port: a))
    monkeypatch.setattr(boot, 'time', NS(monotonic=lambda: clock[0], time=lambda: 1000 + clock[0]))
    return a


def test_receive_loop_readonly_timeout_then_reconnect_and_confirm(monkeypatch):
    a = simulated_session(monkeypatch)
    assert boot.session('can0', True)[0] == 'DISABLED'
    assert a.calls == []
    assert boot.session('can0', False)[0] == 'READY'
    assert a.calls == [7]
    assert a.connections == [dict(piper_init=False, start_thread=False)] * 2
    assert len(a.disconnections) == 2


def test_malformed_feedback_never_confirms_or_sends(monkeypatch):
    a = simulated_session(monkeypatch, malformed=True)
    assert boot.session('can0', False)[0] == 'WAIT_FEEDBACK'
    assert a.calls == []
    assert len(a.disconnections) == 1


def test_buffered_precommand_frames_cannot_confirm_enable(monkeypatch):
    a = simulated_session(monkeypatch, queued_before_enable=True)
    assert boot.session('can0', False)[0] == 'WAIT_FEEDBACK'
    assert a.calls == [7]


def test_main_waits_for_device_then_exits_after_ready(monkeypatch, tmp_path):
    config = tmp_path / 'piper.json'
    config.write_text('{"usb_serial": "abc"}')
    monkeypatch.setattr(sys, 'argv', ['boot', '--config', str(config)])
    calls = []
    def discover(serial):
        calls.append(serial)
        if len(calls) == 1:
            raise RuntimeError('USB not connected')
        return 'can9'
    monkeypatch.setattr(boot, 'discover', discover)
    monkeypatch.setattr(boot, 'prepare_can', lambda port, readonly: None)
    monkeypatch.setattr(boot, 'session', lambda port, ro: ('READY', 'enabled'))
    sleeps = []
    monkeypatch.setattr(boot, 'time', NS(sleep=sleeps.append))
    assert boot.main() == 0
    assert calls == ['abc', 'abc']
    assert sleeps == [3]


def test_main_inspect_exits_without_retry_or_mutation(monkeypatch, tmp_path):
    config = tmp_path / 'piper.json'
    config.write_text('{"usb_serial": "abc"}')
    monkeypatch.setattr(sys, 'argv', ['boot', '--config', str(config), '--inspect'])
    monkeypatch.setattr(boot, 'discover', lambda serial: 'can0')
    def check(port, readonly):
        assert readonly
        raise RuntimeError('DOWN')
    monkeypatch.setattr(boot, 'prepare_can', check)
    monkeypatch.setattr(boot, 'time', NS())
    assert boot.main() == 2
