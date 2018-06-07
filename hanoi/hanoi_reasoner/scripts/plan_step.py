class PlanStep():
    """
    Class representing a step in the planner. 
    Used for logging in experiments.
    """
    def __init__(self, step, best_action, time, total_reward, stop):
        self.step = step
        self.best_action = best_action
        self.time = time
        self.total_reward = total_reward
        self.stop = stop
