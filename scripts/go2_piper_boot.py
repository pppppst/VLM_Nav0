#!/usr/bin/env python3
"""Initialize the identified Piper CAN device and confirm joint enable once per start."""

import argparse
import json
from pathlib import Path
import subprocess
import time


FEEDBACK_IDS = (0x2A1,) + tuple(range(0x261, 0x267))
FAULTS = (
    'voltage_too_low', 'motor_overheating', 'driver_overcurrent',
    'driver_overheating', 'collision_status', 'driver_error_status', 'stall_status',
)


def discover(serial, net=Path('/sys/class/net')):
    matches = []
    for interface in net.iterdir():
        device = (interface / 'device').resolve()
        for parent in (device,) + tuple(device.parents):
            try:
                identity = [(parent / name).read_text().strip()
                            for name in ('idVendor', 'idProduct', 'serial')]
            except OSError:
                continue
            if identity == ['1d50', '606f', serial]:
                matches.append(interface.name)
                break
    if len(matches) != 1:
        raise RuntimeError('USB identity match count={} expected=1'.format(len(matches)))
    return matches[0]


def prepare_can(interface, readonly=False, run=subprocess.run):
    result = run(['ip', '-j', '-details', 'link', 'show', interface],
                 check=True, capture_output=True, text=True, timeout=5)
    link = json.loads(result.stdout)[0]
    info = link.get('linkinfo', {}).get('info_data', {})
    if 'UP' not in link['flags']:
        if readonly:
            raise RuntimeError('CAN interface is DOWN')
        run(['ip', 'link', 'set', interface, 'type', 'can', 'bitrate', '1000000'],
            check=True, capture_output=True, text=True, timeout=5)
        run(['ip', 'link', 'set', interface, 'up'],
            check=True, capture_output=True, text=True, timeout=5)
        return
    if info.get('bittiming', {}).get('bitrate') != 1000000:
        raise RuntimeError('active CAN bitrate mismatch; existing interface left unchanged')
    if info.get('state') != 'ERROR-ACTIVE':
        raise RuntimeError('CAN state={}'.format(info.get('state')))


def assess(arm, seen, now):
    missing = [hex(i) for i in FEEDBACK_IDS if i not in seen or now - seen[i] > 1.0]
    if missing:
        return 'WAIT_FEEDBACK', 'stale_or_missing=' + ','.join(missing)
    status = arm.GetArmStatus().arm_status
    if status.err_code or status.arm_status:
        return 'WAIT_FAULT', 'arm_status={} err_code={}'.format(status.arm_status, status.err_code)
    if status.teach_status or status.ctrl_mode not in (0, 1):
        return 'WAIT_CONTROL', 'teach_status={} ctrl_mode={}'.format(status.teach_status, status.ctrl_mode)
    low = arm.GetArmLowSpdInfoMsgs()
    enabled = []
    faults = []
    for i in range(1, 7):
        motor = getattr(low, 'motor_' + str(i))
        enabled.append(bool(motor.foc_status.driver_enable_status))
        faults.extend('joint{}:{}'.format(i, f) for f in FAULTS if getattr(motor.foc_status, f))
    if faults:
        return 'WAIT_FAULT', ','.join(faults)
    return ('READY' if all(enabled) else 'DISABLED'), 'enabled={}'.format(enabled)


def step(arm, seen, now, readonly):
    state, reason = assess(arm, seen, now)
    if state == 'DISABLED' and not readonly:
        arm.EnableArm(7)
        seen.clear()  # Confirmation must include fresh feedback from all six joints after the command.
        return 'ENABLING', 'joint_enable_sent; awaiting_feedback'
    return state, reason


def report(state, reason):
    category = ('NONE' if state == 'READY' else
                'WAIT_INPUT' if state in ('WAIT_CAN', 'WAIT_FEEDBACK', 'ENABLING') else
                'MANUAL_REQUIRED')
    print(json.dumps(dict(state=state, reason_class=category, reason=reason), ensure_ascii=False), flush=True)


def session(interface, readonly):
    import can
    from piper_sdk import C_PiperInterface_V2

    seen = {}
    last = ('WAIT_FEEDBACK', 'waiting_for_arm')
    # Own the receive loop so each motor has an independent monotonic freshness timestamp.
    arm = C_PiperInterface_V2(interface)
    arm.ConnectPort(piper_init=False, start_thread=False)
    try:
        with can.Bus(interface='socketcan', channel=interface,
                     can_filters=[dict(can_id=i, can_mask=0x7FF, extended=False)
                                  for i in FEEDBACK_IDS]) as bus:
            deadline = time.monotonic() + (3.0 if readonly else 10.0)
            next_check = 0.0
            not_before = time.time()
            previous = None
            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.1)
                now = time.monotonic()
                if (msg is not None and not msg.is_error_frame and not msg.is_remote_frame
                        and not msg.is_extended_id and not msg.is_fd and msg.dlc == 8
                        and msg.arbitration_id in FEEDBACK_IDS
                        and msg.timestamp >= not_before
                        and 0 <= time.time() - msg.timestamp <= 1.0):
                    arm.ParseCANFrame(msg)
                    seen[msg.arbitration_id] = now
                if now >= next_check:
                    last = step(arm, seen, now, readonly)
                    if last[0] == 'ENABLING':
                        not_before = time.time()
                    if last != previous:
                        report(*last)
                        previous = last
                    if last[0] == 'READY':
                        return last
                    next_check = now + 1.0
            return last
    finally:
        arm.DisconnectPort()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--inspect', action='store_true', help='read only; no CAN configuration or enable')
    args = parser.parse_args()
    while True:
        state = 'WAIT_CONFIG'
        try:
            config = json.loads(Path(args.config).read_text())
            serial = config['usb_serial']
            if not isinstance(serial, str) or not serial or not serial.isalnum():
                raise ValueError('invalid USB serial')
            state = 'WAIT_CAN'
            interface = discover(serial)
            prepare_can(interface, readonly=args.inspect)
            state = 'WAIT_FEEDBACK'
            state, reason = session(interface, args.inspect)
            if state == 'READY':
                return 0
            report(state, reason)
        except Exception as exc:
            detail = getattr(exc, 'stderr', None) or str(exc)
            report(state, '{}: {}'.format(type(exc).__name__, detail.strip()))
        if args.inspect:
            return 2
        time.sleep(3)


if __name__ == '__main__':
    raise SystemExit(main())
