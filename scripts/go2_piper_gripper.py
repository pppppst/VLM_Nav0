#!/usr/bin/env python3
"""Observe Piper or attempt one explicit gripper command; never move arm joints."""
import argparse
import contextlib
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

spec = importlib.util.spec_from_file_location('piper_boot', Path(__file__).with_name('go2_piper_boot.py'))
boot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boot)
FEEDBACK_IDS = boot.FEEDBACK_IDS + (0x2A8, 0x47E)
GRIPPER_FAULTS = ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                  'driver_overheating', 'sensor_status', 'driver_error_status')


def validate_command(width, effort):
    if not math.isfinite(width) or not 0 <= width <= 100:
        raise ValueError('width-mm must be finite and within 0..100; firmware range is also checked')
    if not math.isfinite(effort) or not 0 < effort <= 5 or round(effort * 1000) < 1:
        raise ValueError('effort-nm must be finite and within 0.001..5; this is torque, not force')
    return round(width * 1000), round(effort * 1000)


def accept_frame(msg, not_before, wall_now):
    return (msg is not None and not msg.is_error_frame and not msg.is_remote_frame
            and not msg.is_extended_id and not msg.is_fd and msg.dlc == 8
            and msg.arbitration_id in FEEDBACK_IDS and msg.timestamp >= not_before
            and 0 <= wall_now - msg.timestamp <= 1.0)


class Command:
    def __init__(self, width=None, effort=None):
        self.values = None if width is None else validate_command(width, effort)
        self.attempted = False

    def step(self, arm, seen, now):
        result = dict(command_attempted=self.attempted, grasp_confirmed=False)
        required = boot.FEEDBACK_IDS + (0x2A8,)
        if self.values is not None and not self.attempted:
            required += (0x47E,)
        missing = [hex(i) for i in required if i not in seen or not 0 <= now - seen[i] <= 1.0]
        if missing:
            return dict(result, state='WAIT_FEEDBACK', reason_class='WAIT_INPUT', reason=','.join(missing))
        grip = arm.GetArmGripperMsgs().gripper_state
        result.update(width_mm=grip.grippers_angle / 1000., torque_nm=grip.grippers_effort / 1000.,
                      gripper_enabled=bool(grip.foc_status.driver_enable_status))
        state, reason = boot.assess(arm, seen, now)
        if state != 'READY':
            return dict(result, state=state, reason_class='WAIT_ARM', reason=reason)
        faults = [name for name in GRIPPER_FAULTS if getattr(grip.foc_status, name)]
        if faults:
            return dict(result, state='WAIT_GRIPPER', reason_class='WAIT_FAULT', reason=','.join(faults))
        if self.values is None:
            return dict(result, state='OBSERVING', reason_class='NONE', reason='read_only')
        width, effort = self.values
        result.update(target_mm=width / 1000., requested_torque_nm=effort / 1000.)
        if not self.attempted:
            maximum = arm.GetGripperTeachingPendantParamFeedback().arm_gripper_teaching_param_feedback.max_range_config
            if maximum not in (70, 100) or width > maximum * 1000:
                return dict(result, state='WAIT_RANGE', reason_class='WAIT_COMMAND',
                            reason='firmware_max_mm={}; cancel and choose a valid command'.format(maximum))
            # Mark before calling: transport exceptions must never replay a possible write.
            self.attempted = True
            seen.clear()
            arm.GripperCtrl(width, effort, 0x01, 0x00)
            return dict(result, command_attempted=True, state='COMMAND_ATTEMPTED',
                        reason_class='WAIT_FEEDBACK', reason='one_call_only; awaiting new feedback')
        at_target = grip.foc_status.driver_enable_status and abs(grip.grippers_angle - width) <= 1000
        return dict(result, state='AT_TARGET' if at_target else 'OBSERVING', reason_class='NONE',
                    reason='position_feedback_only; not proof of grasp or command acknowledgement')


def report(**fields):
    print(json.dumps(dict(time=time.time(), **fields), ensure_ascii=False), flush=True)


def session(interface, command):
    import can
    from piper_sdk import C_PiperInterface_V2
    arm = C_PiperInterface_V2(interface)
    try:
        arm.ConnectPort(can_init=True, piper_init=False, start_thread=False)
        with can.Bus(interface='socketcan', channel=interface,
                     can_filters=[dict(can_id=i, can_mask=0x7FF, extended=False) for i in FEEDBACK_IDS]) as bus:
            seen = {}
            not_before = time.time()
            next_report = next_query = 0.
            while True:
                now = time.monotonic()
                if command.values is not None and not command.attempted and now >= next_query:
                    arm.ArmParamEnquiryAndConfig(param_enquiry=0x04)
                    next_query = now + 2.
                msg = bus.recv(timeout=.1)
                if accept_frame(msg, not_before, time.time()):
                    old_stamp = arm.GetArmGripperMsgs().time_stamp
                    arm.ParseCANFrame(msg)
                    if msg.arbitration_id == 0x2A8 and arm.GetArmGripperMsgs().time_stamp == old_stamp:
                        seen.pop(0x2A8, None)  # SDK may reject a frame while retaining old healthy feedback.
                    else:
                        seen[msg.arbitration_id] = time.monotonic()
                if time.monotonic() >= next_report:
                    attempted = command.attempted
                    result = command.step(arm, seen, time.monotonic())
                    if command.attempted and not attempted:
                        not_before = time.time()
                    report(**result)
                    next_report = time.monotonic() + .5
    finally:
        try:
            arm.DisconnectPort()
        except Exception as error:
            report(state='CLEANUP_ERROR', reason_class='WAIT_INPUT', reason=str(error))


def run(config_path, command, state_dir):
    while True:
        try:
            config = json.loads(config_path.read_text())
            serial = config['usb_serial']
            if not isinstance(serial, str) or not serial or not serial.isalnum():
                raise ValueError('invalid USB serial')
            break
        except (OSError, ValueError, KeyError) as error:
            report(state='WAIT_CONFIG', reason_class='WAIT_INPUT', reason=str(error))
            time.sleep(1)
    with contextlib.ExitStack() as stack:
        if command.values is not None:
            lock = stack.enter_context(open(state_dir / ('control-' + serial + '.lock'), 'a'))
            while True:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    report(state='WAIT_BUSY', reason_class='WAIT_COMMAND', reason='another gripper command is active')
                    time.sleep(1)
        while True:
            try:
                interface = boot.discover(serial)
                boot.prepare_can(interface, readonly=True)
                session(interface, command)
            except Exception as error:
                report(state='WAIT_IO', reason_class='WAIT_INPUT', reason=str(error),
                       command_attempted=command.attempted, automatic_resend=False)
                time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('/opt/piper-boot/piper.json'))
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('inspect', help='read-only monitoring; Ctrl+C exits')
    setter = sub.add_parser('set', help='one explicit gripper command then monitor; Ctrl+C does not stop/release')
    setter.add_argument('--width-mm', type=float, required=True)
    setter.add_argument('--effort-nm', type=float, required=True, help='torque limit in N.m, not fingertip force')
    args = parser.parse_args()
    try:
        command = Command(args.width_mm, args.effort_nm) if args.action == 'set' else Command()
    except ValueError as error:
        parser.error(str(error))
    state_dir = Path.home() / '.local/state/go2-piper-gripper'
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, path = tempfile.mkstemp(prefix=args.action + '-', suffix='.log', dir=str(state_dir))
    link = Path(path + '.link')
    link.symlink_to(path)
    link.replace(state_dir / 'current.log')
    print('诊断日志：' + path, flush=True)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    with os.fdopen(fd, 'w') as stream, contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
        try:
            if os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY'):
                try:
                    subprocess.Popen(['gnome-terminal', '--window', '--title=Piper gripper diagnostics', '--',
                        'bash', str(Path(__file__).with_name('piper_gripper_debug.sh')), 'diagnostics', path],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError as error:
                    report(state='DIAGNOSTICS', reason_class='DISPLAY_UNAVAILABLE', reason=str(error))
            report(state='STARTING', reason_class='NONE', reason=args.action,
                   requested_raw=command.values, automatic_resend=False)
            run(args.config, command, state_dir)
        except KeyboardInterrupt:
            report(state='EXITED', reason_class='USER_EXIT', command_attempted=command.attempted,
                   reason='monitor closed; no stop, release, disable, zero or reset command sent')
            return 130


if __name__ == '__main__':
    raise SystemExit(main())
