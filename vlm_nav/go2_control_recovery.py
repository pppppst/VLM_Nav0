"""Nonblocking, generation-fenced recovery of an already armed Go2 session."""
import time


class ControlRecovery:
    def __init__(self, node, *, service_type=None, clock=time.monotonic, max_age=2.):
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        if service_type is None:
            from multi_agent_task_msgs.srv import RecoverGo2Control
            service_type = RecoverGo2Control
        self.Request = service_type.Request
        self.clock, self.max_age = clock, max_age
        self.client = node.create_client(service_type, '/twist_to_go2_sport_bridge/recover_control')
        self.subscription = node.create_subscription(DiagnosticArray, '/vlm_nav/go2_bridge_diagnostics',
            self.observe, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.instance = None
        self.epoch = None
        self.claimed = None
        self.pending = None
        self.sent_at = None
        self.request_epoch = None
        self.prepare_epoch = None
        self.cancel_epoch = None
        self.cancel_pending = None
        self.cancel_sent_at = None
        self.snapshot = {}
        self.received = None
        self.closed = False
        self.reason = ''
        self.hard_fault = ''
        self.ever_armed = False

    def observe(self, message):
        for status in message.status:
            if status.name != 'go2_bridge': continue
            fields = {item.key: item.value for item in status.values}
            try:
                instance, epoch = fields['instance_id'], int(fields['epoch'])
                if not instance or epoch < 0: return
            except (KeyError, ValueError, TypeError):
                return
            if self.instance is not None and instance != self.instance:
                self.closed = True
                self.hard_fault = self.reason = 'bridge restarted; explicit ARM required'
                return
            if self.epoch is not None and epoch < self.epoch: return
            self.instance, self.epoch = instance, epoch
            self.snapshot, self.received = fields, self.clock()
            code = fields.get('fault_code')
            if code in ('INVALID_COMMAND', 'FOREIGN_CONTROL', 'OWNERSHIP_CONFLICT', 'UNKNOWN'):
                self.hard_fault = self.hard_fault or 'bridge hard fault: ' + code
                self.closed = True
            elif (self.ever_armed and fields.get('state')=='DISARMED'
                  and fields.get('recovery_pending')!='true'):
                self.hard_fault = self.hard_fault or 'bridge manually disarmed; task resume revoked'
                self.closed = True
            if fields.get('state')=='ARMED':self.ever_armed = True

    def _send(self, phase, epoch=None):
        if self.pending is not None or not self.client.service_is_ready(): return
        epoch = self.epoch if epoch is None else epoch
        request = self.Request(instance_id=self.instance, expected_epoch=epoch, phase=phase)
        try:
            self.request_epoch, self.sent_at = epoch, self.clock()
            if phase == self.Request.PREPARE:self.prepare_epoch = epoch
            self.pending = self.client.call_async(request)
            self.pending.add_done_callback(lambda future: self._done(future, phase, epoch))
        except (RuntimeError, OSError) as error:
            self.pending = None
            self.reason = 'recovery service unavailable: ' + type(error).__name__

    def _done(self, future, phase, epoch):
        if future is not self.pending:return
        self.pending = None
        try:
            response = future.result()
            if response.instance_id != self.instance: return
            self.epoch = max(self.epoch, response.epoch)
            self.reason = response.reason
            if response.accepted and phase == self.Request.PREPARE and response.epoch == epoch:
                self.claimed = epoch
            if phase in (self.Request.RESUME, self.Request.CANCEL) and response.accepted:
                self.claimed = None
        except Exception as error:
            # A failed RPC never grants authority; retain the claim for diagnostics/retry.
            self.reason = 'recovery response unavailable: ' + type(error).__name__

    def _cancel(self):
        if self.cancel_pending is not None or not self.client.service_is_ready():return
        try:
            request=self.Request(instance_id=self.instance, expected_epoch=self.cancel_epoch, phase=self.Request.CANCEL)
            self.cancel_sent_at=self.clock()
            self.cancel_pending=self.client.call_async(request)
            self.cancel_pending.add_done_callback(self._cancel_done)
        except (RuntimeError,OSError):
            self.cancel_pending=None

    def _cancel_done(self,future):
        if future is not self.cancel_pending:return
        self.cancel_pending=None
        try:
            response=future.result()
            if response.instance_id==self.instance and (response.accepted or response.epoch>self.cancel_epoch):
                self.cancel_epoch=None
        except Exception:
            self.reason='recovery cancellation response unavailable'

    def poll(self):
        if self.pending is not None and self.clock() - self.sent_at > self.max_age:
            # Late callbacks cannot mutate a newer request; the server fences retries.
            self.pending = None
            self.reason = 'recovery RPC timed out; waiting for current bridge state'
        if self.cancel_pending is not None and self.clock()-self.cancel_sent_at>self.max_age:
            self.cancel_pending=None
        if self.closed and self.cancel_epoch is not None:self._cancel()

    def step(self, ready_to_resume):
        self.poll()
        if self.closed or self.pending is not None or self.received is None: return
        if self.clock() - self.received > self.max_age:
            self.reason = 'waiting for fresh bridge recovery diagnostics'
            return
        if self.snapshot.get('recovery_allowed') != 'true': return
        if self.snapshot.get('recovery_pending') == 'true':
            if self.prepare_epoch == self.epoch:self.claimed = self.epoch
            if self.claimed == self.epoch and ready_to_resume:
                self._send(self.Request.RESUME)
        elif self.snapshot.get('state') == 'FAULT':
            self._send(self.Request.PREPARE)

    def cancel(self):
        if self.closed:return
        self.closed = True
        # Keep the source epoch: CANCEL also revokes its direct RESUME successor.
        self.cancel_epoch = self.prepare_epoch if self.prepare_epoch is not None else self.epoch
        if (self.snapshot.get('state')=='FAULT' and self.snapshot.get('recovery_allowed')=='true'
                and self.cancel_epoch is not None and self.epoch>self.cancel_epoch):
            self.cancel_epoch=self.epoch
        if self.cancel_epoch is not None:
            self._cancel()

    def finish_cancel(self,spin,timeout):
        self.cancel()
        deadline=self.clock()+timeout
        while self.cancel_epoch is not None and self.clock()<deadline:
            self.poll()
            spin()
        return self.cancel_epoch is None
