"""Single-robot recovery: stop first, freeze the run clock, resume on live health."""
from dataclasses import replace
import time

from .models import motion_path_error


class ExperimentRecovery:
    def __init__(self, adapter, owners, record, clock=time.monotonic):
        self.adapter, self.owners, self.record = adapter, owners, record
        self.wall_clock = clock
        self.started = clock()
        self.paused_at = None
        self.paused_total = 0.
        self.reason = ''
        self.recovering = False
        self.closed = False

    def clock(self):
        return (self.paused_at if self.paused_at is not None else self.wall_clock()) - self.paused_total

    def timing(self, deadline):
        current = 0. if self.paused_at is None else self.wall_clock() - self.paused_at
        return dict(run_elapsed_s=max(0., self.clock() - self.started),
                    pause_elapsed_s=current, paused_total_s=self.paused_total + current,
                    run_remaining_s=max(0., deadline - self.clock()))

    def gates(self, stationary=True):
        health = self.adapter.get_health()
        gates = []
        if self.owners(): gates.append('other_goal_owner')
        if not health.localization_valid: gates.append('localization_valid')
        if not health.nav_ready:
            checks = getattr(self.adapter, 'last_health_details', {}).get('checks', {})
            gates.extend(k for k, ok in checks.items() if not ok and k != 'rgbd_ready')
            if not gates: gates.append('nav_ready')
        if self.adapter.costmap_transform() is None: gates.append('local_costmap')
        if self.adapter.costmap_transform(global_goal=True) is None: gates.append('global_goal_costmap')
        if stationary and not health.stopped: gates.append('stopped')
        if stationary and not health.motion_terminal: gates.append('motion_terminal')
        error = motion_path_error(replace(health, path_valid=True), [health.pose])
        if error: gates.append(error)
        return list(dict.fromkeys(gates))

    def pause(self, reason):
        if self.paused_at is None:
            self.paused_at = self.wall_clock()
        if self.reason != reason:
            self.reason = reason
            self.record(dict(state='WAITING', event='RECOVERY_WAIT', reason=reason))

    def resume(self):
        if self.paused_at is not None:
            elapsed = self.wall_clock() - self.paused_at
            self.paused_total += elapsed
            self.paused_at = None
            self.record(dict(state='RUNNING', event='RECOVERED', reason=self.reason,
                             pause_elapsed_s=elapsed, paused_total_s=self.paused_total))
        self.reason = ''

    def observe_camera(self):
        if self.recovering or self.closed: return
        health = self.adapter.get_health()
        waiting = getattr(self.adapter, 'perception_wait_allowed', lambda h: False)(health)
        if waiting and self.paused_at is None:self.pause('rgbd_ready / image_time_camera_tf')
        elif not waiting and self.reason=='rgbd_ready / image_time_camera_tf':self.resume()

    def wait(self, reason, probe=None):
        self.recovering = True
        if self.paused_at is None:self.paused_at = self.wall_clock()
        # Revoke callback authority before cancelling; late action replies remain owned.
        self.adapter.stop_motion('experiment recovery: ' + reason)
        self.pause(reason)
        try:
            while True:
                yield
                failed = self.gates()
                if not getattr(getattr(self.adapter,'vision',None),'available',True):failed.append('vision_worker')
                if probe is not None: failed.extend(probe())
                if not failed:
                    self.resume()
                    self.recovering = False
                    return
                self.pause(', '.join(dict.fromkeys(failed)))
        finally:
            # Closing this generator is cancellation, never permission to resume motion.
            if self.recovering: self.closed = True
