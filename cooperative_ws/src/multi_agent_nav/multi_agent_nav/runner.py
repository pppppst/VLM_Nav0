"""Execute machine effects without blocking ROS callbacks; late futures are fenced."""
from .models import Effect, Observation, TARGETS

class AgentRunner:
    def __init__(self,machine,adapter,descriptions,on_observation):
        self.machine,self.adapter,self.descriptions=machine,adapter,descriptions
        self.on_observation=on_observation
        self.pending=[]

    def tick(self,now):
        if hasattr(self.adapter,'advance'):self.adapter.advance(now)
        if hasattr(self.adapter,'tick'):self.adapter.tick()
        while self.adapter.results:
            self.machine.on_goal_result(self.adapter.results.popleft(),now)
        effects=self.machine.tick(now,self.adapter.get_health())
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
                elif kind in ('PLAN','PLAN_TRANSIT'):self.machine.on_path(value,goal,now)
                else:self.machine.scan_completed(value,now)
            except Exception:
                if kind=='SCAN':
                    self.machine.scan_completed([Observation(target_id=t,scan_id=command.command_id,
                        observation_id=f'{command.command_id}/error/{t}',status='API_ERROR',reason='scan operation failed') for t in TARGETS],now)
                else:self.machine.fail(f'{kind.lower()} operation failed')
        effects+=self.machine.take_effects()
        self.dispatch(effects,now)

    def dispatch(self,effects,now):
        for effect in effects:
            command=self.machine.command
            if effect.kind=='STOP':
                for future,*_ in self.pending:future.cancel()
                self.pending.clear()
                goal_id,reason=effect.payload
                if goal_id:self.adapter.cancel_navigation(goal_id)
                self.adapter.stop_motion(reason)
            elif effect.kind=='OBSERVATION':self.on_observation(effect.payload)
            elif command and not self.machine.paused:
                try:
                    if effect.kind=='APPROACH':future=self.adapter.select_approach(*effect.payload);goal=None
                    elif effect.kind in ('PLAN','PLAN_TRANSIT'):
                        goal=effect.payload;future=self.adapter.compute_path(goal)
                    elif effect.kind=='SCAN':
                        future=self.adapter.start_scan(self.descriptions,effect.payload);goal=None
                    elif effect.kind=='NAVIGATE':
                        goal,kind=effect.payload
                        goal_id=self.adapter.start_navigation(goal,command.task_id,command.command_id,kind)
                        self.machine.navigation_started(goal_id,kind,now)
                        continue
                    else:continue
                    self.pending.append((future,(command.task_id,command.command_id),effect.kind,goal))
                except Exception:
                    self.machine.stop(f'{effect.kind.lower()} adapter failure')
                    self.dispatch(self.machine.take_effects(),now)
