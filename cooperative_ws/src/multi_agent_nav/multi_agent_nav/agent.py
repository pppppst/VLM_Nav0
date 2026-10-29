from dataclasses import replace
from .models import *

NAV_KIND = {'SEARCH_VIEWPOINT':'VIEWPOINT_NAV', 'APPROACH_PICKUP':'PICKUP_NAV',
            'WAIT':'WAIT_NAV', 'APPROACH_DEPOSIT':'DEPOSIT_NAV'}

class AgentMachine:
    def __init__(self, context, frame, lost_timeout=3.0, path_start_timeout=1.0, peer_identity=None, task_timeout=600.0):
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
        self.peer_task_received = None
        self.peer_state = None
        self.peer_map_identity = peer_identity
        self.peer_problem = ''
        self.task_timeout = task_timeout
        self.committed = False
        self.executing = False
        self.task_synced = False
        self.last_sequence = 0
        self.paused = False
        self.recovery_state = None
        self.pause_sync_received = None
        self.awaiting_peer_resume = False
        self.task_completed = False
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
        if not context.share_c_coordinates:
            self._state.run_state = 'PREPARING'

    @property
    def single_robot_mode(self):
        return (self.context.robot_id == 'go2' and not self.context.simulation
                and not self.context.share_c_coordinates)

    def state(self):
        return replace(self._state) if self.context.share_c_coordinates else replace(
            self._state, common_pose=Pose2D(), pose_valid=False)

    def take_effects(self):
        effects, self.effects = self.effects, []
        return effects

    def receive_peer(self, state, now):
        if state.robot_id not in ROBOTS or state.robot_id == self.context.robot_id or (
            state.schema_version != SCHEMA_VERSION or state.simulation != self.context.simulation):
            self._reject_peer('peer schema/robot/mode mismatch')
            return
        if not state.local_map_id or not state.map_version:
            self._reject_peer('peer map identity mismatch')
            return
        if self.context.share_c_coordinates:
            if (state.common_frame_id, state.alignment_id) != (
                self.context.common_frame_id, self.context.alignment_id):
                self._reject_peer('peer frame/alignment mismatch');return
        elif state.common_frame_id or state.alignment_id:
            self._reject_peer('peer frame/alignment mismatch');return
        identity = (state.local_map_id, state.map_version)
        if self.peer_map_identity is not None and identity != self.peer_map_identity:
            self._reject_peer('peer map identity mismatch')
            return
        self.peer_received = now
        self.peer_map_identity = identity
        if self.context.task_id and state.task_id not in ('', self.context.task_id):
            return
        self.peer_state = replace(state)
        self.peer_problem = ''
        if state.task_id == self.context.task_id and self.context.task_id:
            self.peer_task_received = now
        if self.committed or self.executing:
            if state.task_id != self.context.task_id:self.stop('peer restarted: task identity lost')
            elif not state.localization_valid:
                self.stop('peer localization invalid')
            elif state.run_state in ('STOPPING', 'PAUSED') and not is_recoverable_pause(state.reason):
                self.stop('peer paused: '+state.reason)
            elif not state.nav_ready:
                self.stop('peer navigation not ready', recoverable=True)
            elif state.run_state in ('STOPPING', 'PAUSED'):
                if not (self.awaiting_peer_resume and state.run_state == 'PAUSED' and state.stopped):
                    self.stop('peer paused for recovery', recoverable=True)
            else:
                self.awaiting_peer_resume = False

    def _reject_peer(self, reason):
        self.peer_problem = reason
        self.peer_state = None
        if self.committed or self.executing:self.stop(reason)

    def _peer_healthy(self, now):
        state = self.peer_state
        return state is not None and self.peer_received is not None and now-self.peer_received <= self.lost_timeout and (
            state.localization_valid and state.nav_ready and (state.pose_valid or not self.context.share_c_coordinates))

    def _peer_acknowledges(self, now):
        if not self._peer_healthy(now) or self.peer_task_received is None or now-self.peer_task_received > self.lost_timeout:
            return False
        state = self.peer_state
        # A peer can receive EXECUTE before us after the clean handshake completed.
        return (state.stopped and task_sync_state(state, self.context.task_id)) or bool(
            self.task_synced and state.task_id == self.context.task_id and state.command_id and state.sequence > 0
            and state.run_state in ('PLANNING', 'NAVIGATING', 'LOCAL_EXECUTION', 'SCAN_STOP',
                'SCANNING', 'ARRIVAL_STOP', 'RETRY_STOP', 'IDLE'))

    def _prepare_state(self, now):
        if self.paused or self.executing or self.task_completed:return
        local_ready = (self._state.localization_valid and self.health.nav_ready and self.health.stopped
                       and self.health.motion_terminal)
        if self.context.task_id:
            acknowledged = bool(local_ready and self.sync_received is not None and
                now-self.sync_received <= self.lost_timeout and self._peer_acknowledges(now))
            if acknowledged:self.task_synced = True
            self._state.run_state = 'SYNCED' if acknowledged else 'SYNCING'
            self._state.reply = 'SYNCED' if acknowledged else ''
            self._state.reason = '' if acknowledged else self.peer_problem or 'task synchronization incomplete'
        elif not local_ready:
            self._state.run_state = 'PREPARING'
        else:
            standby = self._peer_healthy(now) and self.peer_state.stopped and (
                not self.peer_state.task_id and not self.peer_state.reply and clean_private_state(self.peer_state)
                and self.peer_state.run_state in PRETASK_READY_STATES)
            self._state.run_state = 'STANDBY' if standby else 'WAITING_PEER'
            self._state.reason = self.peer_problem

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
            if not self.context.share_c_coordinates and command.phase != 'SYNCING':
                return CommandReply(False, 'new task requires SYNCING phase')
            if command.task_id in self.retired_tasks:
                return CommandReply(False, 'retired task')
            if self._state.run_state not in ('IDLE', 'PAUSED', 'PREPARING', *PRETASK_READY_STATES) or not self._state.stopped or not self.health.motion_terminal:
                return CommandReply(False, 'not stopped for new task')
            if not self.context.share_c_coordinates and not (self._state.localization_valid and self.health.nav_ready):
                return CommandReply(False, 'local health not ready for new task')
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
            self.peer_task_received = None
            self.peer_state = None
            self.peer_problem = ''
            self.committed = self.executing = self.task_synced = False
            self.task_started = now
            self.last_sequence = 0
            self.paused = False
            self.recovery_state = None
            self.awaiting_peer_resume = False
            self.task_completed = False
        reply = validate_command(command, self.context)
        if not reply.accepted:
            return reply
        if self.task_completed:
            if sync and command.phase == 'DONE':
                self.sync_received = now
                return reply
            if command.action in ('PAUSE', 'CANCEL'):
                if command.action == 'CANCEL' and command.cancel_command_id != self._state.command_id:
                    return CommandReply(False, 'cancel command mismatch')
                self.stop('coordinator stop after task completion')
                return reply
            return CommandReply(False, 'task complete: new task required')
        if sync:
            if not self.context.share_c_coordinates:
                if command.phase == 'DONE':
                    # Global completion alone cannot certify an unfinished local action.
                    if (self.paused or not self._state.ready or not self._state.deposited
                            or self._state.run_state != 'IDLE' or self._state.reply != 'SUCCEEDED'
                            or not self.health.stopped or not self.health.motion_terminal
                            or self._state.goal_id or self.local_motion is not None
                            or not self.command or self.command.kind != 'APPROACH_DEPOSIT'):
                        return CommandReply(False, 'DONE requires local C success and confirmed stop')
                    self.task_completed = True
                    self.committed = self.executing = False
                    self.command = self.selected_goal = None
                    self.effects.clear()
                if self.committed and command.phase == 'SYNCING':
                    return CommandReply(False, 'task phase already committed')
                if command.phase == 'SEARCH_AB' and not self.committed:
                    if not self.task_synced:return CommandReply(False, 'task handshake not acknowledged')
                    self.committed = True
            if self.task_started is None:
                self.task_started = now
            self.sync_received = now
            self.phase = command.phase
            if not self.context.share_c_coordinates:
                self._prepare_state(now)
            return reply
        if command.action in ('PAUSE', 'CANCEL'):
            if command.action == 'CANCEL' and (not self.command or command.cancel_command_id != self.command.command_id):
                return CommandReply(False, 'cancel command mismatch')
            recoverable = command.action == 'PAUSE' and command.kind == 'NAV_READY_WAIT'
            self.stop('coordinator recoverable pause' if recoverable else 'coordinator pause/cancel',
                      recoverable=recoverable)
            return reply
        if command.sequence <= self.last_sequence:
            return CommandReply(command.command_id == self._state.command_id, 'duplicate/stale command')
        if command.phase != self.phase:
            return CommandReply(False, 'phase must be synchronized before execution')
        if not self.context.share_c_coordinates and not self.local_execution_enabled:
            return CommandReply(False, 'local task executor required')
        if command.kind == 'APPROACH_PICKUP' and (self.phase != 'SEARCH_AB' or self._state.ready):
            return CommandReply(False, 'pickup phase/READY mismatch')
        if command.kind == 'APPROACH_DEPOSIT' and (self.phase != 'DEPOSIT' or not self._state.ready):
            return CommandReply(False, 'deposit requires READY and deposit phase')
        if self.paused or self._state.run_state == 'STOPPING':
            return CommandReply(False, 'paused: new task required')
        if not self.context.share_c_coordinates and not self.executing and not (
            self.committed and self.task_synced and self._peer_acknowledges(now)):
            return CommandReply(False, 'current task peer handshake required')
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
        self.command, self.last_sequence = command, command.sequence
        self.executing = True
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
        if self.task_completed:return
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
            if not self.single_robot_mode and result.status == 'SUCCEEDED' and result.goal_kind in ('PICKUP_NAV','DEPOSIT_NAV'):
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
        elif self.command.kind.startswith('APPROACH'):
            self._state.run_state = 'ARRIVAL_STOP'
        else:
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
        from .local_approach_execution import local_target_accepted
        if self.paused or not self.command or identity!=(self.command.task_id,self.command.command_id):return
        if self.single_robot_mode and identity[0]!=self.context.task_id:return
        state=self._state
        if self.command.kind=='SEARCH_VIEWPOINT':
            success=outcome=='TARGET_FOUND' and bool(self.local_c_observations)
        else:
            distance=target_distance((self.health.pose.x,self.health.pose.y),
                (self.local_target.x,self.local_target.y) if self.local_target else None)
            if self.single_robot_mode and outcome=='TARGET_APPROACHED' and self.local_approach_succeeded:
                if not local_target_accepted(self.health,self.local_target,self.success_radius):
                    return False
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
        if self.task_completed:return
        self._state.reply, self._state.reason, self._state.run_state = 'FAILED', reason, 'IDLE'

    def stop(self, reason, *, recoverable=False):
        if self.task_completed:
            self.effects.append(Effect('STOP', ('', reason)))
            return
        recoverable = recoverable and not self.context.share_c_coordinates
        if self._state.run_state == 'STOPPING' or self.paused:
            if self.recovery_state is not None and not recoverable:
                self.recovery_state = None
                self._state.reason = reason
            return
        self.recovery_state = replace(self._state) if recoverable else None
        self.pause_sync_received = self.sync_received
        self.awaiting_peer_resume = False
        self.paused = True
        self.effects.clear()
        self.effects.append(Effect('STOP', (self._state.goal_id, reason)))
        self._state.goal_id = ''
        self._state.run_state, self._state.reply, self._state.reason = 'STOPPING', 'CANCELED', reason

    def _resume(self, now):
        if self.recovery_state is None or self._state.run_state != 'PAUSED':return
        peer = self.peer_state
        if (self.phase not in ('SEARCH_AB', 'SEARCH_C', 'DEPOSIT') or
                self.sync_received is None or self.sync_received == self.pause_sync_received or
                now-self.sync_received > self.lost_timeout or
                not self._peer_healthy(now) or peer.task_id != self.context.task_id or
                self.peer_task_received is None or now-self.peer_task_received > self.lost_timeout or
                not self._state.localization_valid or not self.health.nav_ready or
                not self.health.stopped or not self.health.motion_terminal or not peer.stopped or
                peer.run_state == 'STOPPING' or
                peer.run_state == 'PAUSED' and not is_recoverable_pause(peer.reason) or
                any(effect.kind == 'STOP' for effect in self.effects)):
            return
        saved, self.recovery_state = self.recovery_state, None
        self.paused = False
        self.awaiting_peer_resume = peer.run_state == 'PAUSED'
        self.local_motion = None
        self.local_target = None
        self.local_approach_succeeded = False
        self._state.run_state, self._state.reply, self._state.reason = saved.run_state, saved.reply, saved.reason
        if self.command and saved.reply not in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            self._state.run_state, self._state.reply, self._state.reason = 'PLANNING', 'ACCEPTED', ''
            self._plan()

    def check_health(self, now, health):
        self.health = health
        self._state.pose_valid = self._state.localization_valid = health.localization_valid and health.pose.valid
        self._state.nav_ready, self._state.stopped = health.nav_ready, health.stopped
        if self._state.pose_valid:
            self._state.common_pose = self.frame.to_common(health.pose)
        if self.task_completed:
            # Keep the task result; report current health independently of execution gates.
            issues = []
            if not self._state.localization_valid:issues.append('local localization invalid')
            if not health.nav_ready:issues.append('local navigation not ready')
            if not health.stopped:issues.append('stop unconfirmed')
            if not health.motion_terminal:issues.append('motion terminal unconfirmed')
            if self.sync_received is None or now-self.sync_received > self.lost_timeout:
                issues.append('coordinator heartbeat lost')
            if self.peer_task_received is None or now-self.peer_task_received > self.lost_timeout:
                issues.append('peer heartbeat lost')
            if self.peer_problem:issues.append(self.peer_problem)
            elif self.peer_state is not None and (not self._peer_healthy(now) or self.peer_state.run_state in ('STOPPING','PAUSED')):
                issues.append('peer unhealthy or paused')
            self._state.reason = 'post-completion health: '+ '; '.join(issues) if issues else ''
            return
        # The first peer state gets one fixed handshake window; later loss gets no new grace.
        peer_since = self.peer_received if self.peer_received is not None else self.task_started
        if not self.context.share_c_coordinates:
            if self.context.task_id and self.task_started is not None and now-self.task_started > self.task_timeout:
                self.stop('task timeout')
            if self.committed or self.executing:
                if not self._state.localization_valid:
                    self.stop('local localization invalid')
                elif self.sync_received is None or now-self.sync_received > self.lost_timeout:
                    self.stop('coordinator SYNC heartbeat lost', recoverable=True)
                elif self.peer_task_received is None or now-self.peer_task_received > self.lost_timeout:
                    self.stop('peer heartbeat lost for current task', recoverable=True)
                elif not health.nav_ready:
                    self.stop('local navigation not ready', recoverable=True)
            self._prepare_state(now)
        elif self.context.task_id and not self.paused and (
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
        if self.paused:
            self._resume(now)
            if self.paused:return self.take_effects()
        if self.command and self._state.run_state=='NAVIGATING':
            self._monitor_path(health, now)
        if health.stopped and health.motion_terminal:
            if self._state.run_state == 'RETRY_STOP':
                self._state.run_state = 'PLANNING'
                self._plan()
            elif self._state.run_state == 'SCAN_STOP':
                self._state.run_state = 'SCANNING'
                self.effects.append(Effect('SCAN', self.command.command_id))
            elif self._state.run_state == 'ARRIVAL_STOP':
                from vlm_nav.arrival import target_distance
                target = self.frame.to_local(self.command.object_position)
                if self.goal_kind != NAV_KIND[self.command.kind] or target_distance(
                    (health.pose.x, health.pose.y), (target.x, target.y)) > self.success_radius:
                    self.fail('current target arrival acceptance failed')
                else:
                    if self.command.kind == 'APPROACH_PICKUP':self._state.ready = True
                    else:self._state.deposited = True
                    self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'
        return self.take_effects()

    def _monitor_path(self, health, now=0.0):
        if self.single_robot_mode:
            error=motion_path_error(replace(health,path_valid=True),[health.pose])
            if error:self.stop(error)
            return
        if health.path_valid and health.path:
            self.path_acquired=True
        elif not self.path_acquired and health.stopped and now-self.navigation_started_at < self.path_start_timeout:
            error=motion_path_error(replace(health,path_valid=True),[health.pose])
            if error:self.stop(error)
            return
        error=motion_path_error(health,health.path)
        if error:self.stop(error)
