from dataclasses import replace
from .models import *

NAV_KIND = {'SEARCH_VIEWPOINT':'VIEWPOINT_NAV', 'APPROACH_PICKUP':'PICKUP_NAV',
            'WAIT':'WAIT_NAV', 'APPROACH_DEPOSIT':'DEPOSIT_NAV', 'EXIT':'EXIT_NAV'}

class AgentMachine:
    def __init__(self, context, frame, lost_timeout=3.0):
        self.context, self.frame, self.lost_timeout = context, frame, lost_timeout
        self._state = AgentState(robot_id=context.robot_id, task_id=context.task_id,
            local_map_id=context.local_map_id, map_version=context.map_version,
            common_frame_id=context.common_frame_id, alignment_id=context.alignment_id,
            simulation=context.simulation)
        self.command = None
        self.effects = []
        self.goal_kind = ''
        self.sync_received = None
        self.peer_received = None
        self.last_sequence = 0
        self.paused = False
        self.retry = 0
        self.excluded = []
        self.selected_goal = None
        self.health = Health()
        self.guard = None
        self.transit_goal = None

    def state(self):
        return replace(self._state)

    def take_effects(self):
        effects, self.effects = self.effects, []
        return effects

    def _plan(self):
        if self.command.kind.startswith('APPROACH'):
            self.effects.append(Effect('APPROACH', (self.frame.to_local(self.command.object_position), list(self.excluded))))
        else:
            self.selected_goal = self.frame.to_local(self.command.common_goal)
            self.effects.append(Effect('PLAN', self.selected_goal))

    def accept(self, command, now):
        # SYNC alone establishes a task; EXECUTE can never silently replace it.
        if command.action == 'SYNC' and command.task_id != self.context.task_id:
            if self._state.run_state not in ('IDLE', 'PAUSED') or not self._state.stopped or not self.health.motion_terminal:
                return CommandReply(False, 'not stopped for new task')
            candidate = replace(self.context, task_id=command.task_id)
            reply = validate_command(command, candidate)
            if not reply.accepted:
                return reply
            self.context = candidate
            self._state.task_id = command.task_id
            self._state.ready = self._state.deposited = self._state.exited = False
            self._state.reply = ''
            self.command = None
            self.last_sequence = 0
            self.paused = False
        reply = validate_command(command, self.context)
        if not reply.accepted:
            return reply
        if command.action == 'SYNC':
            self.sync_received = now
            return reply
        if command.action in ('PAUSE', 'CANCEL'):
            if command.action == 'CANCEL' and (not self.command or command.cancel_command_id != self.command.command_id):
                return CommandReply(False, 'cancel command mismatch')
            self.stop('coordinator pause/cancel')
            return reply
        if command.action == 'GRANT_RESOURCE':
            owner = self.command.parent_command_id if self.command and self.command.kind == 'EXIT' else (self.command.command_id if self.command else '')
            if not command.token or command.parent_command_id != owner or command.resource_id != self._state.requested_resource:
                # Repeated grants for an already-held token are harmless.
                return CommandReply(command.token == self._state.token and command.parent_command_id == self._state.permit_command_id, 'permit mismatch')
            self._state.resource_id, self._state.token = command.resource_id, command.token
            self._state.permit_command_id = owner
            self._state.resource_status = 'RESERVED'
            self._state.requested_resource = ''
            if self.selected_goal:
                self.on_path(PathResult(True, self.health.path), self.selected_goal, now)
            return reply
        if command.sequence <= self.last_sequence:
            return CommandReply(command.command_id == self._state.command_id, 'duplicate/stale command')
        if self.paused or self._state.run_state == 'STOPPING':
            return CommandReply(False, 'paused: new task required')
        if self.command and self._state.reply not in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            return CommandReply(False, 'one active command')
        if not self._state.stopped or not self.health.motion_terminal:
            return CommandReply(False, 'previous motion not stopped')
        if command.kind == 'EXIT' and (not self._state.deposited or command.token != self._state.token or
                command.parent_command_id != self._state.permit_command_id or command.resource_id != self._state.resource_id):
            return CommandReply(False, 'EXIT must preserve deposit permit identity')
        self.command, self.last_sequence = command, command.sequence
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
        if self.guard:
            decision = self.guard.check(self.health.pose, self.health.footprint, result.path,
                                       {self._state.resource_id: self._state.token} if self._state.token else {}, self.health)
            if decision.action == 'STOP':
                self.stop(decision.reason)
                return
            if decision.resource_id:
                self._state.requested_resource = decision.resource_id
                self._state.requested_command_id = self.command.command_id
                self._state.run_state = 'WAITING_RESOURCE'
                return
            if self._state.resource_id in self.guard.regions and self._state.resource_id != 'basket':
                self.transit_goal = self.guard.transit_exit(self._state.resource_id, self.context.robot_id, result.path)
                if self.transit_goal is None:
                    self.stop('missing door transit exit')
                    return
        nav_goal = self.transit_goal or goal
        kind = 'DOOR_TRANSIT' if self.transit_goal else NAV_KIND[self.command.kind]
        self.effects.append(Effect('NAVIGATE', (nav_goal, kind)))

    def navigation_started(self, goal_id, goal_kind):
        self._state.goal_id, self.goal_kind = goal_id, goal_kind
        self._state.run_state, self._state.reply = 'NAVIGATING', 'EXECUTING'
        self._state.stopped = False

    def on_goal_result(self, result, now):
        if self.paused or not self.command or (result.task_id, result.command_id, result.goal_id, result.goal_kind) != (
            self.context.task_id, self.command.command_id, self._state.goal_id, self.goal_kind):
            return []
        self._state.goal_id = ''
        # Terminal is not physical stop; tick/adapter must confirm it before replacement.
        if result.status != 'SUCCEEDED':
            if self.retry < 1 and result.status == 'ABORTED':
                self.retry += 1
                self.excluded.append(self.selected_goal)
                self._state.run_state = 'RETRY_STOP'
            else:
                self.fail(result.reason or result.status)
            return []
        if self.goal_kind == 'DOOR_TRANSIT':
            self._state.run_state = 'TRANSIT_STOP'
        elif self.command.kind == 'SEARCH_VIEWPOINT':
            self._state.run_state = 'SCAN_STOP'
        else:
            if self.goal_kind == 'PICKUP_NAV' and self.command.kind == 'APPROACH_PICKUP':
                self._state.ready = True
            elif self.goal_kind == 'DEPOSIT_NAV' and self.command.kind == 'APPROACH_DEPOSIT':
                self._state.deposited = True
            elif self.goal_kind == 'EXIT_NAV' and self.command.kind == 'EXIT':
                self._state.run_state = 'EXIT_STOP'
                return []
            self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'
        return []

    def on_observation(self, observation, now):
        if not self.paused and self.command and observation.scan_id == self.command.command_id:
            self.effects.append(Effect('OBSERVATION', replace(observation,
                task_id=self.context.task_id, robot_id=self.context.robot_id,
                local_map_id=self.context.local_map_id, map_version=self.context.map_version,
                common_frame_id=self.context.common_frame_id, alignment_id=self.context.alignment_id,
                viewpoint_id=self.command.viewpoint_id, simulation=self.context.simulation)))
        return []

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

    def tick(self, now, health):
        self.health = health
        self._state.pose_valid = self._state.localization_valid = health.localization_valid and health.pose.valid
        self._state.nav_ready, self._state.stopped = health.nav_ready, health.stopped
        if self._state.pose_valid:
            self._state.common_pose = self.frame.to_common(health.pose)
        if self.context.task_id and not self.paused and (
            self.sync_received is None or now-self.sync_received > self.lost_timeout or
            self.peer_received is None or now-self.peer_received > self.lost_timeout or
            not health.localization_valid or not health.nav_ready):
            self.stop('SYNC/peer/localization/navigation health lost')
        if self._state.run_state == 'STOPPING':
            if health.stopped and health.motion_terminal:
                self._state.run_state = 'PAUSED'
            return self.take_effects()
        if self.guard and self.command:
            self._monitor_region(health)
        if health.stopped and health.motion_terminal:
            if self._state.run_state == 'RETRY_STOP':
                self._state.run_state = 'PLANNING'
                self._plan()
            elif self._state.run_state == 'SCAN_STOP':
                self._state.run_state = 'SCANNING'
                self.effects.append(Effect('SCAN', self.command.command_id))
            elif self._state.run_state == 'TRANSIT_STOP' and not self._state.token:
                self.transit_goal = None
                self._plan()
            elif self._state.run_state == 'EXIT_STOP' and not self._state.token:
                self._state.exited = True
                self._state.run_state, self._state.reply = 'IDLE', 'SUCCEEDED'
        return self.take_effects()

    def _monitor_region(self, health):
        # Task 5 adds footprint/permit supervision here.
        pass
