"""Passive Piper feedback and durable one-attempt stages; no import-time device I/O."""
import fcntl
import hashlib
import importlib.util
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location('vision_piper_boot', ROOT/'scripts/go2_piper_boot.py')
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)
JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)
POSE_IDS = (0x2A2, 0x2A3, 0x2A4)
FEEDBACK_IDS = common.FEEDBACK_IDS + JOINT_IDS + POSE_IDS + (0x2A8, 0x473, 0x47E)
REQUIRED_IDS = common.FEEDBACK_IDS + JOINT_IDS + POSE_IDS + (0x2A8,)
GRIPPER_FAULTS = ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                  'driver_overheating', 'sensor_status', 'driver_error_status')

# Feedback migrated from standalone/piper_control.py SHA256 44e692a9f2397918558ac2c49b7d77f0af4882ae658d957bbeb445edb18a1d47
class Feedback:
    def __init__(self):
        self.seen = {}
        self.limits = {}
        self.max_width = None

    def missing(self, ids, now):
        return [hex(i) for i in ids if not 0 <= now - self.seen.get(i, -math.inf) <= 1.0]

    def accept(self, arm, msg, not_before, wall_now, now):
        if (msg is None or msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id
                or msg.is_fd or msg.dlc != 8 or msg.arbitration_id not in FEEDBACK_IDS
                or msg.timestamp < not_before or not 0 <= wall_now - msg.timestamp <= 1.0):
            return False
        can_id = msg.arbitration_id
        # SDK can filter a frame while retaining old joint, pose or gripper values.
        getter = (arm.GetArmJointMsgs if can_id in JOINT_IDS else
                  arm.GetArmEndPoseMsgs if can_id in POSE_IDS else
                  arm.GetArmGripperMsgs if can_id == 0x2A8 else None)
        previous_stamp = getter().time_stamp if getter else None
        arm.ParseCANFrame(msg)
        if getter and getter().time_stamp == previous_stamp:
            self.seen.pop(can_id, None)
            return False
        if can_id == 0x473:
            motor = msg.data[0]
            if motor not in range(1, 7):
                return False
            high = int.from_bytes(msg.data[1:3], 'big', signed=True)
            low = int.from_bytes(msg.data[3:5], 'big', signed=True)
            if low >= high or high == 32767 or low == 32767:
                self.limits.pop(motor, None)
                return False
            self.limits[motor] = (low * 100, high * 100, now)
        if can_id == 0x47E:
            maximum = arm.GetGripperTeachingPendantParamFeedback().arm_gripper_teaching_param_feedback.max_range_config
            self.max_width = (maximum * 1000, now) if maximum in (70, 100) else None
        self.seen[can_id] = now
        return True

    def snapshot(self, arm, now):
        result = dict(joints_deg=None, sdk_pose_mm_xyz=None, sdk_pose_euler_deg_xyz=None,
                      gripper_width_mm=None, gripper_torque_nm=None, gripper_enabled=None,
                      missing_feedback=self.missing(common.FEEDBACK_IDS + JOINT_IDS + POSE_IDS + (0x2A8,), now))
        if not self.missing(JOINT_IDS, now):
            joints = arm.GetArmJointMsgs().joint_state
            result['joints_deg'] = [getattr(joints, 'joint_'+str(i)) / 1000. for i in range(1, 7)]
        if not self.missing(POSE_IDS, now):
            pose = arm.GetArmEndPoseMsgs().end_pose
            result['sdk_pose_mm_xyz'] = [getattr(pose, n) / 1000. for n in ('X_axis', 'Y_axis', 'Z_axis')]
            result['sdk_pose_euler_deg_xyz'] = [getattr(pose, n) / 1000. for n in ('RX_axis', 'RY_axis', 'RZ_axis')]
        if not self.missing((0x2A8,), now):
            grip = arm.GetArmGripperMsgs().gripper_state
            result.update(gripper_width_mm=grip.grippers_angle / 1000.,
                          gripper_torque_nm=grip.grippers_effort / 1000.,
                          gripper_enabled=bool(grip.foc_status.driver_enable_status))
        if not self.missing(common.FEEDBACK_IDS, now):
            state = arm.GetArmStatus().arm_status
            result.update(ctrl_mode=state.ctrl_mode, arm_status=state.arm_status,
                          err_code=state.err_code, motion_status=state.motion_status,
                          joints_enabled=[bool(getattr(arm.GetArmLowSpdInfoMsgs(), 'motor_'+str(i)).foc_status.driver_enable_status)
                                          for i in range(1, 7)])
        return result



def dependencies():
    for package, expected in [('piper-sdk','0.6.2'),('python-can','4.5.0')]:
        if version(package) != expected: raise RuntimeError(package+' requires '+expected)
    import can
    from piper_sdk import C_PiperInterface_V2
    return can, C_PiperInterface_V2


class PiperSession:
    def __init__(self, config):
        self.config = config
        self.readonly = config.get('readonly', True)
        self.arm = self.bus = self.lock = None
        self.feedback = Feedback()
        self.attempts = {}
        self.error = None
        self.uncertain = False
        self.not_before = 0.
        self.query_kind = None
        self.next_query = 0.

    def __enter__(self):
        return self

    def _connect(self):
        if self.lock is None:
            directory = Path.home()/'.local/state/go2-piper-gripper'
            directory.mkdir(parents=True, exist_ok=True)
            lock = (directory/('control-'+self.config['usb_serial']+'.lock')).open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BaseException:
                lock.close()
                raise
            self.lock = lock
        can, factory = dependencies()
        interface = common.discover(self.config['usb_serial'])
        common.prepare_can(interface, readonly=True)
        self.arm = factory(interface)
        self.arm.ConnectPort(can_init=True, piper_init=False, start_thread=False)
        self.bus = can.Bus(interface='socketcan', channel=interface,
                           can_filters=[dict(can_id=i,can_mask=0x7FF,extended=False) for i in FEEDBACK_IDS])
        self.feedback = Feedback()
        self.not_before = time.time()

    def _disconnect(self):
        bus, arm = self.bus, self.arm
        self.bus = self.arm = None
        self.feedback = Feedback()
        try:
            if bus: bus.shutdown()
        finally:
            if arm: arm.DisconnectPort()

    def __exit__(self, *_):
        try:
            self._disconnect()
        finally:
            if self.lock: self.lock.close(); self.lock = None

    def poll(self):
        try:
            if self.arm is None: self._connect()
            if not self.readonly and self.query_kind and time.monotonic() >= self.next_query:
                if self.query_kind == 'joint': self.arm.SearchAllMotorMaxAngleSpd()
                else: self.arm.ArmParamEnquiryAndConfig(param_enquiry=0x04)
                self.next_query = time.monotonic()+.5
            deadline = time.monotonic()+.12
            while time.monotonic() < deadline:
                message = self.bus.recv(timeout=.02)
                self.feedback.accept(self.arm,message,self.not_before,time.time(),time.monotonic())
            self.error = None
        except Exception as error:
            self.error = str(error)
            try: self._disconnect()
            except Exception: pass

    def snapshot(self):
        self.poll()
        if self.error or self.arm is None:
            return dict(state='WAIT_IO',reason=self.error or 'device unavailable',missing_feedback=['connection'],
                        command_attempted=bool(self.attempts),sample_time=0.)
        now = time.monotonic()
        result = self.feedback.snapshot(self.arm,now)
        state, reason = common.assess(self.arm,self.feedback.seen,now)
        if result['missing_feedback']: state,reason='WAIT_FEEDBACK','missing/old frame'
        elif any(getattr(self.arm.GetArmGripperMsgs().gripper_state.foc_status,f) for f in GRIPPER_FAULTS):
            state,reason='WAIT_GRIPPER','gripper fault'
        if self.uncertain: state,reason='COMMAND_UNCERTAIN','possible partial write; no stage replay'
        result.update(state=state,reason=reason,command_attempted=bool(self.attempts),
                      sample_time=min((self.feedback.seen.get(i,0.) for i in REQUIRED_IDS),default=0.),
                      time=time.time(),teach_status=self.arm.GetArmStatus().arm_status.teach_status)
        return result

    def _attempt(self, stage_id, kind, values, speed=None):
        if self.readonly: return dict(state='READ_ONLY',command_attempted=False)
        if not stage_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in stage_id):
            raise ValueError('invalid stage id')
        receipt = Path(self.config['run_dir'])/'attempts'/(stage_id+'.json')
        if self.uncertain or (receipt.exists() and stage_id not in self.attempts):
            self.uncertain = True
            return dict(state='COMMAND_UNCERTAIN',command_attempted=True,reason='durable previous attempt; cannot replay')
        if stage_id in self.attempts:
            return dict(state='COMMAND_ATTEMPTED',command_attempted=True,reason='already attempted; monitor only')
        measured = self.snapshot()
        if measured['state'] != 'READY': return measured
        now = time.monotonic()
        if kind == 'joint':
            if (len(values)!=6 or not all(math.isfinite(v) for v in values)
                    or type(speed) is not int or not 0 <= speed <= 100):
                raise ValueError('six finite angles and integer speed in 0..100 required')
            target = [round(v*1000) for v in values]
            if any(i not in self.feedback.limits or not 0 <= now-self.feedback.limits[i][2] <= 1 for i in range(1,7)):
                self.query_kind='joint'; return dict(state='WAIT_LIMITS',command_attempted=False)
            if any(not self.feedback.limits[i][0] <= target[i-1] <= self.feedback.limits[i][1] for i in range(1,7)):
                return dict(state='WAIT_RANGE',command_attempted=False,reason='device joint limits')
        else:
            width, effort = values
            if measured['ctrl_mode']==0:
                speed=self.config['motion']['speed_percent']
                if type(speed) is not int or not 0 <= speed <= 100:
                    raise ValueError('integer speed in 0..100 required for CAN mode')
            grip=self.config['gripper']
            expected=grip.get('open_effort_nm',grip['effort_nm']) if stage_id=='open' else grip['effort_nm']
            if not all(math.isfinite(v) for v in values) or not 0 <= width <=100 or effort != expected:
                raise ValueError('invalid gripper target/experiment effort')
            target = [round(width*1000),round(effort*1000)]
            maximum=self.feedback.max_width
            if maximum is None or not 0 <= now-maximum[1] <=1:
                self.query_kind='gripper'; return dict(state='WAIT_LIMITS',command_attempted=False)
            if target[0]>maximum[0]:return dict(state='WAIT_RANGE',command_attempted=False,reason='device gripper range')
        receipt.parent.mkdir(parents=True,exist_ok=True)
        data=dict(run_id=self.config['run_id'],stage_id=stage_id,kind=kind,target=target,speed_percent=speed,
                  time=time.time(),status='ATTEMPT_RECORDED_NOT_ACKNOWLEDGED')
        try:
            # fsync file and directory before the first possible SDK write, including mode selection.
            fd=os.open(str(receipt.parent.parent),os.O_RDONLY)
            try:os.fsync(fd)
            finally:os.close(fd)
            with receipt.open('x') as stream:
                json.dump(data,stream);stream.flush();os.fsync(stream.fileno())
            fd=os.open(str(receipt.parent),os.O_RDONLY)
            try:os.fsync(fd)
            finally:os.close(fd)
        except Exception:
            self.uncertain=True
            return dict(state='COMMAND_UNCERTAIN',command_attempted=receipt.exists(),reason='receipt persistence failed')
        self.attempts[stage_id]=data
        self.uncertain=True
        self.query_kind=None
        self.feedback.seen.clear()
        try:
            if kind=='joint' or measured['ctrl_mode']==0:
                self.arm.MotionCtrl_2(1,1,speed,0)
            if kind=='joint':
                self.arm.JointCtrl(*target)
            else:self.arm.GripperCtrl(*target,1,0)
            self.not_before=time.time()
            self.uncertain=False
            return dict(state='COMMAND_ATTEMPTED',command_attempted=True,reason='awaiting new feedback')
        except Exception as error:
            return dict(state='COMMAND_UNCERTAIN',command_attempted=True,reason=str(error))

    def attempt_joint(self, stage_id, angles_deg, speed_percent):
        return self._attempt(stage_id,'joint',angles_deg,speed_percent)

    def attempt_gripper(self, stage_id, width_mm, effort_nm):
        return self._attempt(stage_id,'gripper',[width_mm,effort_nm])
