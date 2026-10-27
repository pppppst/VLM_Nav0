"""Execute machine effects without blocking ROS callbacks; late futures are fenced."""
import time
from .models import Effect, Observation, TARGETS

class AgentRunner:
    def __init__(self,machine,adapter,descriptions,on_observation,clock=time.monotonic,config=None,success_radius=1.0,on_event=None):
        self.machine,self.adapter,self.descriptions=machine,adapter,descriptions
        self.on_observation=on_observation
        self.on_event=on_event
        self.clock=clock
        self.pending=[]
        self.waiting=[]
        self.adapter.motion_guard=self.motion_ready
        self.config=config
        self.execution=None
        self.execution_identity=None
        self.approach=None
        self.local_completion=None
        self.local_approach_result=None
        self.adapter.single_robot_mode=machine.single_robot_mode
        self.local_path_accept=None
        self.owners=lambda:[]
        machine.local_execution_enabled=config is not None and not machine.context.share_c_coordinates
        machine.success_radius=success_radius

    def local_approach(self, target, kind, record):
        from .single_robot_approach import load_parameters
        from .local_approach_execution import LocalApproachExecution
        identity=self.execution_identity
        command=self.machine.command
        if not command or identity!=(command.task_id,command.command_id):
            raise RuntimeError('approach task identity changed')
        self.machine.local_approach_succeeded=False
        self.machine.local_target=target
        parameters=load_parameters(self.config['robot_go2'].get('arrival_config_files'))
        execution=LocalApproachExecution(self.adapter,target,*identity,kind,parameters,record,self.clock)
        self.approach=execution.approach
        self.machine._state.run_state='LOCAL_EXECUTION'
        def check():
            command=self.machine.command
            if self.machine.paused or not command or identity!=(command.task_id,command.command_id):
                raise RuntimeError('approach interrupted')
        def on_tick():
            self.machine._state.goal_id=execution.approach.goal_id if self.adapter.motions else ''
        try:
            outcome=yield from execution.run(check,on_tick)
            self.machine.local_approach_succeeded=outcome=='TARGET_APPROACHED'
            if self.machine.local_approach_succeeded:self.local_approach_result=execution
            return outcome
        finally:
            self.approach=None

    def local_navigation(self,goal,kind,path_accept=None):
        self.local_path_accept=path_accept
        try:
            future=self.machine.begin_local_navigation(goal,kind)
            while not future.done():yield
            return future.result()
        finally:
            self.local_path_accept=None

    def local_stop(self):
        self.adapter.stop_motion('local execution stopping')
        until=self.clock()+self.config[f'robot_{self.machine.context.robot_id}'].get('cancel_timeout',5.)
        while True:
            health=self.adapter.get_health()
            if health.stopped and health.motion_terminal:return
            if self.clock()>=until:raise RuntimeError('physical stop unconfirmed')
            yield

    def start_local_execution(self):
        from copy import deepcopy
        from pathlib import Path
        import json
        import tempfile
        from .go2_exploration import explore_steps
        command=self.machine.command
        config=deepcopy(self.config)
        robot=self.machine.context.robot_id
        cfg=config[f'robot_{robot}']
        if not self.adapter.simulation:cfg['exploration_strategy']='generated_candidates_vlm'
        cfg['approach_on_detection']=command.kind!='SEARCH_VIEWPOINT'
        directory=Path(cfg.get('experiment_records_root') or tempfile.mkdtemp(prefix='cooperative-fake-'))/command.command_id.replace('/','_')
        directory.mkdir(parents=True,exist_ok=True)
        self.adapter.selection_evidence_dir=directory/'decisions'
        self.adapter.scan_evidence_dir=directory/'views'
        identity=(command.task_id,command.command_id)
        if self.on_event:self.on_event(dict(state='DISPLAY_RESET',task_id=command.task_id))
        def record(event):
            if event.get('state')=='APPROACH_PLANNING':
                from .models import Pose2D
                self.machine.local_target=Pose2D(**event['object_pose'])
            try:
                with (directory/'events.jsonl').open('a') as stream:stream.write(json.dumps(event)+'\n')
                if self.on_event:self.on_event(event)
            except Exception as error:
                if not self.machine.single_robot_mode:raise
                self.adapter.node.get_logger().warning('Task diagnostic write failed: '+str(error))
        def observations(values):
            for value in values:self.machine.on_observation(value,self.clock())
        self.execution_identity=identity
        if self.machine.single_robot_mode:self.adapter.record_event=record
        self.execution=explore_steps(self.adapter,config,self.machine.frame,
            self.machine.task_started+config['scene'].get('task_timeout',600.),
            self.descriptions[command.target_id],record,self.owners,lambda:True,lambda:None,
            clock=self.clock,robot_id=robot,target_id=command.target_id,success_radius=self.machine.success_radius,
            require_navigation_success=True,navigation=self.local_navigation,stopping=self.local_stop,
            on_observations=observations,task_id=command.task_id,parent_command_id=command.command_id,
            initial_camera_check=robot=='go2' and not self.adapter.simulation and command.kind!='APPROACH_DEPOSIT',
            approaching=self.local_approach if self.machine.single_robot_mode else None,
            initial_observation=self.machine.local_c_observations.get(command.observation_id)
                if command.kind=='APPROACH_DEPOSIT' else None)

    def motion_ready(self,task_id,command_id):
        command=self.machine.command
        if not command or command.command_id!=command_id or (task_id and command.task_id!=task_id):return False
        health=self.adapter.get_health()
        self.machine.check_health(self.clock(),health)
        if self.machine.single_robot_mode and self.owners():
            self.machine.stop('other navigation owner present')
        if self.machine.paused:
            self.dispatch(self.machine.take_effects(),self.clock())
            return False
        return self.machine.command is command and self.machine.peer_received is not None and health.stopped and health.motion_terminal

    def tick(self,now):
        if hasattr(self.adapter,'advance'):self.adapter.advance(now)
        # Stop before the adapter can send another scan Spin or consume a late result.
        health=self.adapter.get_health()
        self.machine.check_health(self.clock(),health)
        if (self.machine.single_robot_mode and (self.execution is not None or self.local_completion is not None)
                and self.owners()):
            self.machine.stop('other navigation owner present')
        if self.machine.paused:self.dispatch(self.machine.take_effects(),now)
        if hasattr(self.adapter,'tick'):self.adapter.tick()
        while self.adapter.results:
            self.machine.on_goal_result(self.adapter.results.popleft(),now)
        health=self.adapter.get_health()
        effects=self.machine.tick(self.clock(),health)
        for item in list(self.pending):
            future,identity,kind,goal=item
            if not future.done():continue
            self.pending.remove(item)
            command=self.machine.command
            if future.cancelled() or self.machine.paused or not command or identity!=(command.task_id,command.command_id):
                continue
            try:
                value=future.result()
                if kind=='APPROACH':self.machine.on_approach(value,now)
                elif kind=='PLAN':
                    if self.local_path_accept and not self.local_path_accept(goal,value):
                        self.machine.stop('local target replanned path rejected')
                    else:self.machine.on_path(value,goal,now)
                else:self.machine.scan_completed(value,now)
            except Exception as error:
                if kind=='SCAN':
                    self.machine.scan_completed([Observation(target_id=t,scan_id=command.command_id,
                        observation_id=f'{command.command_id}/error/{t}',status='API_ERROR',reason='scan operation failed') for t in TARGETS],now)
                elif kind=='PLAN' and self.local_path_accept:
                    self.machine.stop('local target path check failed: '+str(error))
                else:self.machine.fail(f'{kind.lower()} operation failed')
        command=self.machine.command
        waiting,self.waiting=self.waiting,[]
        effects += [effect for effect,identity in waiting if command and
                    identity==(command.task_id,command.command_id)]
        effects+=self.machine.take_effects()
        self.dispatch(effects,now)
        if self.execution is not None and not self.machine.paused:
            try:
                next(self.execution)
            except StopIteration as finished:
                self.execution=None
                self.machine.health=self.adapter.get_health()
                self.local_completion=(self.execution_identity,finished.value)
            except Exception as error:
                self.execution=None
                self.machine.stop('local execution failed: '+str(error))
            self.dispatch(self.machine.take_effects(),self.clock())
        if self.local_completion is not None:
            identity,outcome=self.local_completion
            if self.machine.complete_local_execution(identity,outcome) is not False:
                command=self.machine.command
                if (self.local_approach_result is not None and not self.machine.paused and command
                        and identity==(command.task_id,command.command_id)
                        and identity[0]==self.machine.context.task_id
                        and self.machine.state().reply=='SUCCEEDED'):
                    self.local_approach_result.accepted()
                self.local_approach_result=None
                self.local_completion=None
                self.execution_identity=None
                if self.machine.single_robot_mode:self.adapter.record_event=None

    def dispatch(self,effects,now):
        command=self.machine.command
        for effect in effects:
            if effect.kind=='STOP':
                self.local_approach_result=None
                self.local_completion=None
                goal_id,reason=effect.payload
                execution,self.execution=self.execution,None
                try:
                    if execution is not None:execution.close()
                except Exception:
                    reason+='; local execution cleanup failed'
                if self.machine.local_motion is not None:
                    self.machine.local_motion['reply'].cancel()
                    self.machine.local_motion=None
                for future,*_ in self.pending:future.cancel()
                self.pending.clear()
                self.waiting.clear()
                try:
                    if goal_id:self.adapter.cancel_navigation(goal_id)
                finally:
                    self.adapter.stop_motion(reason)
                    if self.on_event:self.on_event(dict(state='STOPPING'))
            elif command and self.machine.command is command and not self.machine.paused:
                if effect.kind=='OBSERVATION':
                    self.on_observation(effect.payload)
                    continue
                try:
                    if effect.kind=='LOCAL_EXECUTION':
                        if self.execution is None:self.start_local_execution()
                        continue
                    if not self.motion_ready(command.task_id,command.command_id):
                        # Replan after settling so a delayed goal cannot reuse an old path check.
                        if not self.machine.paused and self.machine.command is command:
                            retry=Effect('PLAN',effect.payload[0]) if effect.kind=='NAVIGATE' else effect
                            self.waiting.append((retry,(command.task_id,command.command_id)))
                        continue
                    if self.machine.command is not command:continue
                    if effect.kind=='APPROACH':future=self.adapter.select_approach(*effect.payload);goal=None
                    elif effect.kind=='PLAN':
                        goal=effect.payload;future=self.adapter.compute_path(goal)
                    elif effect.kind=='SCAN':
                        future=self.adapter.start_scan(self.descriptions,effect.payload);goal=None
                    elif effect.kind=='NAVIGATE':
                        goal,kind=effect.payload
                        goal_id=self.adapter.start_navigation(goal,command.task_id,command.command_id,kind)
                        if self.machine.paused or self.machine.command is not command:
                            if goal_id:self.adapter.cancel_navigation(goal_id)
                            self.dispatch(self.machine.take_effects(),now)
                            continue
                        if not goal_id:
                            self.waiting.append((Effect('PLAN',goal),(command.task_id,command.command_id)))
                            continue
                        self.machine.navigation_started(goal_id,kind,self.clock())
                        continue
                    else:continue
                    self.pending.append((future,(command.task_id,command.command_id),effect.kind,goal))
                except Exception:
                    self.machine.stop(f'{effect.kind.lower()} adapter failure')
                    self.dispatch(self.machine.take_effects(),now)
