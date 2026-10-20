from .models import PlannedAction, Pose2D, ROBOTS

class RulePlanner:
    def __init__(self):
        self.visited = set()
        self.waited = set()

    def decide(self, snapshot):
        actions = []
        local = not snapshot.scene.get('share_c_coordinates',True)
        for robot in ROBOTS:
            state = snapshot.states[robot]
            if robot in snapshot.commands or not state.stopped:
                continue
            if snapshot.phase == 'DEPOSIT':
                if state.deposited:
                    continue
                if local and robot not in snapshot.local_c:
                    actions.append(PlannedAction(robot,'SEARCH_VIEWPOINT',target_id='C'))
                else:
                    local_c = not snapshot.scene.get('share_c_coordinates', True)
                    obs_id = snapshot.local_c[robot] if local_c else snapshot.scene['bound_c']
                    obs = snapshot.observations[obs_id]
                    if (robot, obs_id) not in snapshot.rejected:
                        actions.append(PlannedAction(robot, 'APPROACH_DEPOSIT', target_id='C',
                            observation_id=obs_id, goal=None if local_c else obs.common_position))
                continue
            target = {'go2':'A','ranger':'B'}[robot] if snapshot.phase == 'SEARCH_AB' else 'C'
            if snapshot.phase == 'SEARCH_AB' and state.ready:
                if local:continue
                if robot not in self.waited and robot in snapshot.scene['wait_points']:
                    self.waited.add(robot)
                    actions.append(PlannedAction(robot, 'WAIT', goal=Pose2D(*snapshot.scene['wait_points'][robot])))
                continue
            if snapshot.phase == 'SEARCH_C' and robot in snapshot.local_c:
                continue
            if local:
                actions.append(PlannedAction(robot,'APPROACH_PICKUP' if target!='C' else 'SEARCH_VIEWPOINT',target_id=target))
                continue
            clue = next((o for o in snapshot.observations.values() if o.target_id == target and o.navigable
                         and (robot, o.observation_id) not in snapshot.rejected), None)
            if clue and target != 'C':
                actions.append(PlannedAction(robot, 'APPROACH_PICKUP', target_id=target,
                    observation_id=clue.observation_id, goal=clue.common_position))
                continue
            for point in snapshot.scene['viewpoints'][robot]:
                key = (robot, snapshot.phase, point['id'])
                if key in self.visited or point['id'] in snapshot.reservations:
                    continue
                actions.append(PlannedAction(robot, 'SEARCH_VIEWPOINT', viewpoint_id=point['id'],
                                             goal=Pose2D(*point['pose'])))
                break
        return actions
