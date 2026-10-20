from dataclasses import replace
from .models import *

NAV_KIND = {'SEARCH_VIEWPOINT':'VIEWPOINT_NAV', 'APPROACH_PICKUP':'PICKUP_NAV',
            'WAIT':'WAIT_NAV', 'APPROACH_DEPOSIT':'DEPOSIT_NAV'}

class AgentMachine:
    def __init__(self, context, frame, lost_timeout=3.0, path_start_timeout=1.0):
        import math
        if not math.isfinite(path_start_timeout) or path_start_timeout <= 0:
            raise ValueError('path_start_timeout must be finite and positive')
        if not isinstance(context.share_c_coordinates, bool):
            raise ValueError('share_c_coordinates must be boolean')
        self.context, self.frame, self.lost_timeout = context, frame, lost_timeout
        self.path_start_timeout = path_start_timeout
        self.retired_tasks = set()
        self.navigation_started_at = 0.0
        self.path_acquired = False
        self._state = AgentState(robot_id=context.robot_id, task_id=context.task_id,
            local_map_id=context.local_map_id, map_version=context.map_version,
            common_frame_id=context.common_frame_id, alignment_id=context.alignment_id,
            simulation=context.simulation)
        self.command = None
        self.effects = []
        self.goal_kind = ''
        self.sync_received = None
        self.task_started = None
        self.phase = 'IDLE'
        self.peer_received = None
        self.last_sequence = 0
        self.paused = False
        self.retry = 0
        self.excluded = []
        self.selected_goal = None
        self.health = Health()
        self.local_c_observations = {}
        self.local_execution_enabled = False
        self.local_motion = None
        self.local_target = None
        self.local_approach_succeeded = False
        self.success_radius = 1.0

    def state(self):
        return replace(self._state) if self.context.share_c_coordinates else replace(
            self._state, common_pose=Pose2D(), pose_valid=False)

    def take_effects(self):
        effects, self.effects = self.effects, []
        return effects

    def _plan(self):
        if self.local_motion is not None:
            self.effects.append(Effect('PLAN',self.local_motion['goal']))
            return
        if self.local_execution_enabled and not self.command.position_valid and self.command.kind in (
            'APPROACH_PICKUP','SEARCH_VIEWPOINT','APPROACH_DEPOSIT'):
            self.effects.append(Effect('LOCAL_EXECUTION'))
            return
        if self.command.kind.startswith('APPROACH'):
            self.effects.append(Effect('APPROACH', (self.frame.to_local(self.command.object_position), list(self.excluded))))
        else:
            self.selected_goal = self.frame.to_local(self.command.common_goal)
            self.effects.append(Effect('PLAN', self.selected_goal))

    def accept(self, command, now):
        # SYNC alone establishes a task; EXECUTE can never silently replace it.
        sync = command.action in ('SYNC', 'SYNC_LOCAL_C')
        if sync and command.task_id != self.context.task_id:
            if command.task_id in self.retired_tasks:
                return CommandReply(False, 'retired task')
            if self._state.run_state not in ('IDLE', 'PAUSED') or not self._state.stopped or not self.health.motion_terminal:
                return CommandReply(False, 'not stopped for new task')
            candidate = replace(self.context, task_id=command.task_id)
            reply = validate_command(command, candidate)
            if not reply.accepted:
                return reply
            if self.context.task_id:
                self.retired_tasks.add(self.context.task_id)
            self.context = candidate
            self._state = AgentState(robot_id=candidate.robot_id, task_id=candidate.task_id,
                local_map_id=candidate.local_map_id, map_version=candidate.map_version,
                common_frame_id=candidate.common_frame_id, alignment_id=candidate.alignment_id,
                simulation=candidate.simulation, common_pose=self._state.common_pose,
                pose_valid=self._state.pose_valid, localization_valid=self._state.localization_valid,
                nav_ready=self._state.nav_ready, stopped=self._state.stopped)
            self.command = self.selected_goal = None
            self.effects.clear()
            self.local_c_observations.clear()
            self.local_motion = None
            self.local_target = None
            self.local_approach_succeeded = False
            self.goal_kind = ''
            self.peer_received = None
            self.task_started = now
            self.last_sequence = 0
            self.paused = False
        reply = validate_command(command, self.context)
        if not reply.accepted:
            return reply
        if sync:
            if self.task_started is None:
                self.task_started = now
            self.sync_received = now
            self.phase = command.phase
            return reply
        if command.action in ('PAUSE', 'CANCEL'):
            if command.action == 'CANCEL' and (not self.command or command.cancel_command_id != self.command.command_id):
                return CommandReply(False, 'cancel command mismatch')
            self.stop('coordinator pause/cancel')
            return reply
        if command.sequence <= self.last_sequence:
            return CommandReply(command.command_id == self._state.command_id, 'duplicate/stale command')
        if command.phase != self.phase:
            return CommandReply(False, 'phase must be synchronized before execution')
        if command.kind == 'APPROACH_PICKUP' and (self.phase != 'SEARCH_AB' or self._state.ready):
            return CommandReply(False, 'pickup phase/READY mismatch')
        if command.kind == 'APPROACH_DEPOSIT' and (self.phase != 'DEPOSIT' or not self._state.ready):
            return CommandReply(False, 'deposit requires READY and deposit phase')
        if self.paused or self._state.run_state == 'STOPPING':
            return CommandReply(False, 'paused: new task required')
        if self.sync_received is None or now-self.sync_received > self.lost_timeout or (
            self.peer_received is None or now-self.peer_received > self.lost_timeout):
            return CommandReply(False, 'fresh SYNC/peer state required')
        if self.command and self._state.reply not in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            return CommandReply(False, 'one active command')
        if not self._state.stopped or not self.health.motion_terminal:
            return CommandReply(False, 'previous motion not stopped')
        if command.kind == 'APPROACH_DEPOSIT' and not self.context.share_c_coordinates:
            observation = self.local_c_observations.get(command.observation_id)
            if not observation or not observation.navigable or observation.task_id != self.context.task_id:
                return CommandReply(False, 'own C observation missing for current task')
            if not self.local_execution_enabled:
                command = replace(command, object_position=observation.common_position, position_valid=True)
        self.command, self.last_sequence = command, command.sequence
        self.local_motion = None
        self.local_target = None
        self.local_approach_succeeded = False
        self.retry, self.excluded = 0, []
        self._state.command_id, self._state.sequence = command.command_id, command.sequence
        self._state.reply, self._state.reason = 'ACCEPTED', ''
        self._state.run_state = 'PLANNING'
        self._plan()
        return reply

    def on_approach(self, approach, now):
        if approach.goal is None:
            self.fail(approach.reason or 'no reachable approach')
            return
        self.selected_goal = approach.goal
        self.effects.append(Effect('PLAN', approach.goal))

    def on_path(self, result, goal, now):
        if self.paused or not self.command:
            return
        if not result.reachable:
            self.fail(result.reason or 'unreachable')
            return
        self.selected_goal = goal
        self.health.path = result.path
        self.health.path_valid = True
        error=motion_path_error(self.health,result.path)
        if error:
            self.stop(error)
            return
        kind=self.local_motion['kind'] if self.local_motion else NAV_KIND[self.command.kind]
        self.effects.append(Effect('NAVIGATE', (goal, kind)))

    def navigation_started(self, goal_id, goal_kind, now=0.0):
        self.navigation_started_at, self.path_acquired = now, False
        self._state.goal_id, self.goal_kind = goal_id, goal_kind
        self._state.run_state, self._state.reply = 'NAVIGATING', 'EXECUTING'
        self._state.stopped = False

    def on_goal_result(self, result, now):
        if self.paused or not self.command or (result.task_id, result.command_id, result.goal_id, result.goal_kind) != (
            self.context.task_id, self.command.command_id, self._state.goal_id, self.goal_kind):
            return []
        self._state.goal_id = ''
        local = self.local_motion
        if local is not None and (
            result.status == 'SUCCEEDED' or result.status != 'ABORTED' or self.retry >= 1
        ):
            if result.status == 'SUCCEEDED' and result.goal_kind in ('PICKUP_NAV','DEPOSIT_NAV'):
                self.local_approach_succeeded = True
            if not local['reply'].done():local['reply'].set_result(result.status)
            self._state.run_state='LOCAL_EXECUTION'
            return []
        # Terminal is not physical stop; tick/adapter must confirm it before replacement.
        if result.status != 'SUCCEEDED':
            if self.retry < 1 and result.status == 'ABORTED':
                self.retry += 1
                self.excluded.append(self.selected_goal)
                self._state.run_state = 'RETRY_STOP'
            else:
                self.fail(result.reason or result.status)
            return []
        if self.command.kind == 'SEARCH_VIEWPOINT':
            self._state.run_state = 'SCAN_STOP'
        else:
            if self.goal_kind == 'PICKUP_NAV' and self.command.kind == 'APPROACH_PICKUP':
                self._state.ready = True
            elif self.goal_kind == 'DEPOSIT_NAV' and self.command.kind == 'APPROACH_DEPOSIT':
                self._state.deposited = True
            self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'
        return []

    def on_observation(self, observation, now):
        if not self.paused and self.command and observation.scan_id == self.command.command_id:
            observation = replace(observation,
                task_id=self.context.task_id, robot_id=self.context.robot_id,
                local_map_id=self.context.local_map_id, map_version=self.context.map_version,
                common_frame_id=self.context.common_frame_id, alignment_id=self.context.alignment_id,
                viewpoint_id=self.command.viewpoint_id, simulation=self.context.simulation)
            if not self.context.share_c_coordinates:
                cached = observation.target_id == 'C' and observation.navigable and bool(observation.observation_id)
                if cached:
                    self.local_c_observations.setdefault(observation.observation_id, observation)
                observation = replace(observation, common_position=Pose2D(), position_valid=False,
                    status='CACHED' if cached else observation.status, reason='')
            self.effects.append(Effect('OBSERVATION', observation))
        return []

    def begin_local_navigation(self,goal,kind):
        from concurrent.futures import Future
        reply=Future()
        self.local_motion=dict(goal=goal,kind=kind,reply=reply)
        self._state.run_state='PLANNING'
        self._plan()
        return reply

    def complete_local_execution(self,identity,outcome):
        from vlm_nav.arrival import target_distance
        if self.paused or not self.command or identity!=(self.command.task_id,self.command.command_id):return
        state=self._state
        if self.command.kind=='SEARCH_VIEWPOINT':
            success=outcome=='TARGET_FOUND' and bool(self.local_c_observations)
        else:
            distance=target_distance((self.health.pose.x,self.health.pose.y),
                (self.local_target.x,self.local_target.y) if self.local_target else None)
            success=(outcome=='TARGET_APPROACHED' and self.local_approach_succeeded
                and self.health.localization_valid and self.health.stopped and self.health.motion_terminal
                and distance<=self.success_radius)
        self.local_motion=None
        if not success:
            self.fail(outcome or 'local target acceptance failed')
            return
        if self.command.kind=='APPROACH_PICKUP':state.ready=True
        elif self.command.kind=='APPROACH_DEPOSIT':state.deposited=True
        state.reply,state.run_state='SUCCEEDED','IDLE'

    def scan_completed(self, observations, now):
        if self.paused or not self.command:
            return
        for observation in observations:
            self.on_observation(observation, now)
        if {o.target_id for o in observations} != set(TARGETS) or any(o.status == 'API_ERROR' for o in observations):
            if self.retry < 1:
                self.retry += 1
                self.effects.append(Effect('SCAN', self.command.command_id))
            else:
                self.fail('scan failed after retry')
        else:
            self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'

    def fail(self, reason):
        self._state.reply, self._state.reason, self._state.run_state = 'FAILED', reason, 'IDLE'

    def stop(self, reason):
        if self._state.run_state == 'STOPPING' or self.paused:
            return
        self.paused = True
        self.effects.append(Effect('STOP', (self._state.goal_id, reason)))
        self._state.goal_id = ''
        self._state.run_state, self._state.reply, self._state.reason = 'STOPPING', 'CANCELED', reason

    def check_health(self, now, health):
        self.health = health
        self._state.pose_valid = self._state.localization_valid = health.localization_valid and health.pose.valid
        self._state.nav_ready, self._state.stopped = health.nav_ready, health.stopped
        if self._state.pose_valid:
            self._state.common_pose = self.frame.to_common(health.pose)
        # The first peer state gets one fixed handshake window; later loss gets no new grace.
        peer_since = self.peer_received if self.peer_received is not None else self.task_started
        if self.context.task_id and not self.paused and (
            self.sync_received is None or now-self.sync_received > self.lost_timeout or
            peer_since is None or now-peer_since > self.lost_timeout or
            not self._state.localization_valid or not health.nav_ready):
            self.stop('SYNC/peer/localization/navigation health lost')

    def tick(self, now, health):
        self.check_health(now, health)
        if self._state.run_state == 'STOPPING':
            if health.stopped and health.motion_terminal:
                self._state.run_state = 'PAUSED'
            return self.take_effects()
        if self.command and self._state.run_state=='NAVIGATING':
            self._monitor_path(health, now)
        if health.stopped and health.motion_terminal:
            if self._state.run_state == 'RETRY_STOP':
                self._state.run_state = 'PLANNING'
                self._plan()
            elif self._state.run_state == 'SCAN_STOP':
                self._state.run_state = 'SCANNING'
                self.effects.append(Effect('SCAN', self.command.command_id))
        return self.take_effects()

    def _monitor_path(self, health, now=0.0):
        if health.path_valid and health.path:
            self.path_acquired=True
        elif not self.path_acquired and health.stopped and now-self.navigation_started_at < self.path_start_timeout:
            error=motion_path_error(replace(health,path_valid=True),[health.pose])
            if error:self.stop(error)
            return
        error=motion_path_error(health,health.path)
        if error:self.stop(error)
