import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gripper', ROOT / 'scripts/go2_piper_gripper.py')
gripper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gripper)


def arm():
    flags = {name: False for name in gripper.GRIPPER_FAULTS}
    flags.update(driver_enable_status=True, homing_status=True)
    grip = NS(grippers_angle=20000, grippers_effort=100, foc_status=NS(**flags))
    status = NS(ctrl_mode=1, arm_status=0, err_code=0, teach_status=0)
    motors = NS(**{'motor_'+str(i): NS(foc_status=NS(**dict(
        {name: False for name in gripper.boot.FAULTS}, driver_enable_status=True))) for i in range(1, 7)})
    cache = NS(gripper_state=grip, time_stamp=0.)
    a = NS(GetArmStatus=lambda: NS(arm_status=status), GetArmLowSpdInfoMsgs=lambda: motors,
           GetArmGripperMsgs=lambda: cache,
           GetGripperTeachingPendantParamFeedback=lambda: NS(
               arm_gripper_teaching_param_feedback=NS(max_range_config=70)), calls=[])
    a.GripperCtrl = lambda *args: a.calls.append(args)
    return a, grip, status


def stamps(now=10):
    return {can_id: now for can_id in gripper.FEEDBACK_IDS}


@pytest.mark.parametrize('width,effort', [(float('nan'), .2), (float('inf'), .2),
    (-1,.2), (101,.2), (30,float('nan')), (30,0), (30,5.1)])
def test_invalid_commands_rejected_before_hardware(width, effort):
    with pytest.raises(ValueError): gripper.validate_command(width, effort)


def test_units_and_firmware_limit():
    assert gripper.validate_command(34.125, .2) == (34125, 200)
    a, _, _ = arm()
    command = gripper.Command(80, .2)
    result = command.step(a, stamps(), 10)
    assert result['state'] == 'WAIT_RANGE'
    assert a.calls == []


@pytest.mark.parametrize('can_id', gripper.FEEDBACK_IDS)
def test_missing_or_old_feedback_cannot_send(can_id):
    a, _, _ = arm()
    command = gripper.Command(30, .2)
    seen = stamps(); seen[can_id] = 8
    assert command.step(a, seen, 10)['state'] == 'WAIT_FEEDBACK'
    assert a.calls == []


@pytest.mark.parametrize('fault', gripper.GRIPPER_FAULTS)
def test_gripper_fault_blocks_send_until_recovery(fault):
    a, grip, _ = arm(); command = gripper.Command(30, .2)
    setattr(grip.foc_status, fault, True)
    assert command.step(a, stamps(), 10)['state'] == 'WAIT_GRIPPER'
    assert a.calls == []
    setattr(grip.foc_status, fault, False)
    assert command.step(a, stamps(), 10)['state'] == 'COMMAND_ATTEMPTED'
    assert a.calls == [(30000, 200, 1, 0)]


def test_set_sends_only_once_and_requires_feedback_after_attempt():
    a, grip, _ = arm(); command = gripper.Command(30, .2)
    seen = stamps()
    assert command.step(a, seen, 10)['state'] == 'COMMAND_ATTEMPTED'
    assert 0x2A8 not in seen
    assert command.step(a, seen, 10)['state'] == 'WAIT_FEEDBACK'
    assert command.step(a, stamps(11), 11)['state'] == 'OBSERVING'
    grip.grippers_angle = 30000
    assert command.step(a, stamps(12), 12)['state'] == 'AT_TARGET'
    assert a.calls == [(30000, 200, 1, 0)]


def test_send_exception_latches_attempt_and_never_replays():
    a, _, _ = arm(); command = gripper.Command(30, .2)
    def uncertain(*args):
        a.calls.append(args)
        raise OSError('transport failed after possible write')
    a.GripperCtrl = uncertain
    with pytest.raises(OSError): command.step(a, stamps(), 10)
    other, _, _ = arm()
    assert command.step(other, stamps(11), 11)['command_attempted'] is True
    assert other.calls == []
    assert len(a.calls) == 1


def test_disabled_arm_waits_without_enabling_joints():
    a, _, _ = arm()
    a.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status = False
    assert gripper.Command(30,.2).step(a,stamps(),10)['state'] == 'DISABLED'
    assert a.calls == []


def test_inspect_never_sends_even_when_disabled():
    a, grip, _ = arm(); grip.foc_status.driver_enable_status = False
    command = gripper.Command()
    result = command.step(a, {i:10 for i in gripper.boot.FEEDBACK_IDS+(0x2A8,)},10)
    assert result['state'] == 'OBSERVING'
    assert result['gripper_enabled'] is False
    assert result['width_mm'] == 20
    assert a.calls == []


@pytest.mark.parametrize('change', [dict(timestamp=9), dict(timestamp=8.9),
    dict(dlc=7), dict(is_error_frame=True), dict(is_remote_frame=True),
    dict(is_extended_id=True), dict(is_fd=True), dict(arbitration_id=0x159)])
def test_old_queued_or_malformed_frames_are_rejected(change):
    message = dict(timestamp=10, dlc=8, is_error_frame=False, is_remote_frame=False,
        is_extended_id=False, is_fd=False, arbitration_id=0x2A8)
    message.update(change)
    assert not gripper.accept_frame(NS(**message), 9.5, 10)


def test_fresh_frame_accepted():
    message = NS(timestamp=10, dlc=8, is_error_frame=False, is_remote_frame=False,
        is_extended_id=False, is_fd=False, arbitration_id=0x2A8)
    assert gripper.accept_frame(message, 9.5, 10.1)


def simulated_session(monkeypatch, command, queued=False, cleanup_error=False, filtered_gripper=False):
    import sys
    clock = NS(now=0.)
    monkeypatch.setattr(gripper, 'time', NS(monotonic=lambda: clock.now,
        time=lambda: 1900000000 + clock.now))
    a, grip, _ = arm()
    events, queries, connections, disconnected = [], [], [], []
    a.ConnectPort = lambda **kw: connections.append(kw)
    def disconnect():
        disconnected.append(True)
        if cleanup_error: raise OSError('disconnect failed')
    a.DisconnectPort = disconnect
    a.ArmParamEnquiryAndConfig = lambda **kw: queries.append(kw)
    def parse(message):
        if message.arbitration_id == 0x2A8 and not filtered_gripper:
            grip.grippers_angle = 30000 if a.calls else grip.grippers_angle
            a.GetArmGripperMsgs().time_stamp = message.timestamp
    a.ParseCANFrame = parse
    class Bus:
        def __init__(self, **kwargs): self.count = 0
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def recv(self, timeout):
            self.count += 1; clock.now += .03
            if self.count > 140: raise KeyboardInterrupt
            stamp = 1900000000 + clock.now
            if queued and a.calls: stamp = 1899999999
            return NS(timestamp=stamp, arbitration_id=gripper.FEEDBACK_IDS[self.count % len(gripper.FEEDBACK_IDS)],
                dlc=8, is_error_frame=False, is_remote_frame=False, is_extended_id=False, is_fd=False)
    monkeypatch.setitem(sys.modules, 'can', NS(Bus=Bus))
    monkeypatch.setitem(sys.modules, 'piper_sdk', NS(C_PiperInterface_V2=lambda interface: a))
    monkeypatch.setattr(gripper, 'report', lambda **fields: events.append(fields))
    return a, events, queries, connections, disconnected


def test_readonly_session_sends_nothing_and_closes_on_interrupt(monkeypatch):
    command = gripper.Command()
    a, events, queries, connections, disconnected = simulated_session(monkeypatch, command)
    with pytest.raises(KeyboardInterrupt): gripper.session('can0', command)
    assert a.calls == queries == []
    assert any(event['state'] == 'OBSERVING' for event in events)
    assert connections == [dict(can_init=True, piper_init=False, start_thread=False)]
    assert disconnected == [True]


def test_set_session_observes_new_target_without_repeating_or_releasing(monkeypatch):
    command = gripper.Command(30,.2)
    a, events, queries, _, disconnected = simulated_session(monkeypatch, command)
    with pytest.raises(KeyboardInterrupt): gripper.session('can0', command)
    assert a.calls == [(30000,200,1,0)]
    assert queries and all(q == dict(param_enquiry=4) for q in queries)
    assert any(event['state'] == 'AT_TARGET' for event in events)
    assert all(event['grasp_confirmed'] is False for event in events)
    assert disconnected == [True]


def test_precommand_queued_frames_cannot_confirm_new_target(monkeypatch):
    command = gripper.Command(30,.2)
    a, events, _, _, _ = simulated_session(monkeypatch, command, queued=True)
    with pytest.raises(KeyboardInterrupt): gripper.session('can0', command)
    assert a.calls == [(30000,200,1,0)]
    assert not any(event['state'] == 'AT_TARGET' for event in events)
    assert events[-1]['state'] == 'WAIT_FEEDBACK'


def test_sdk_rejected_gripper_frames_never_refresh_cached_feedback(monkeypatch):
    # SDK0.6.2 discards abnormal widths without changing its cached value or timestamp.
    command = gripper.Command(30,.2)
    a, events, _, _, _ = simulated_session(monkeypatch, command, filtered_gripper=True)
    a.GetArmGripperMsgs().time_stamp = 1899999999.
    with pytest.raises(KeyboardInterrupt): gripper.session('can0', command)
    assert a.calls == []
    assert events[-1]['state'] == 'WAIT_FEEDBACK'


def test_disconnect_failure_must_not_swallow_user_interrupt(monkeypatch):
    command = gripper.Command(30,.2)
    a, _, _, _, _ = simulated_session(monkeypatch, command, cleanup_error=True)
    with pytest.raises(KeyboardInterrupt): gripper.session('can0', command)
    assert len(a.calls) == 1


def test_reconnect_keeps_uncertain_command_latched(monkeypatch, tmp_path):
    config = tmp_path/'piper.json'; config.write_text('{"usb_serial":"abc"}')
    command = gripper.Command(30,.2)
    a, _, _ = arm(); calls = []
    def session(interface, same_command):
        calls.append(interface)
        same_command.step(a,stamps(),10)
        if len(calls) == 1: raise OSError('connection lost after possible send')
        raise KeyboardInterrupt
    monkeypatch.setattr(gripper, 'session', session)
    monkeypatch.setattr(gripper.boot, 'discover', lambda serial: 'can9')
    monkeypatch.setattr(gripper.boot, 'prepare_can', lambda interface,readonly: None)
    monkeypatch.setattr(gripper.time, 'sleep', lambda seconds: None)
    with pytest.raises(KeyboardInterrupt): gripper.run(config,command,tmp_path)
    assert calls == ['can9','can9']
    assert a.calls == [(30000,200,1,0)]
