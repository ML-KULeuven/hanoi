import csv
import time
import json

CSV_FILE = "../experiments/runs.csv"

class Experiment():
    def __init__(self, model_file_name, timestamp):
        self.runs = []
        self.params = {}
        self.model_file_name = model_file_name
        self.timestamp = timestamp

    def add_runs_to_csv(self):
        with open(CSV_FILE, 'a') as f:
            w = csv.writer(f)
            for run in self.runs:
                run.add_to_csv(w, self.model_file_name, self.timestamp, self.params)

    def param(self, k, v):
        self.params[k] = v
        return self

    def add_run(self, run):
        self.runs.append(run)

    def runs_with_solution(self):
        return filter(lambda r: r.found_solution(), self.runs)

    def nb_runs(self, with_solution = False):
        if with_solution:
            return len(self.runs_with_solution())
        return len(self.runs)

    def nb_steps(self, with_solution = False):
        runs = self.runs_with_solution() if with_solution else self.runs
        return map(lambda run: run.nb_steps(), runs)

    def total_steps(self, with_solution=False):
        return sum(self.nb_steps(with_solution))

    def durations(self):
        return map(lambda run: run.duration(), self.runs)

    def total_duration(self):
        return sum(self.durations())

    def final_rewards(self):
        return map(lambda run: run.final_reward(), self.runs)

    def max_final_reward(self):
        if self.final_rewards():
            return max(self.final_rewards())
        else:
            return 0

    def min_final_reward(self):
        if self.final_rewards():
            return min(self.final_rewards())
        else:
            return 0

    def avg_final_reward(self):
        if self.nb_runs() == 0:
            return 0
        return 1.0 * sum(self.final_rewards()) / self.nb_runs()

    def avg_steps(self, with_solution = False):
        if self.nb_runs(with_solution) == 0:
            return 0
        return 1.0 * self.total_steps(with_solution) / self.nb_runs(with_solution)

    def best_steps(self, with_solution = False):
        if self.nb_runs(with_solution) == 0:
            return 0
        return min(self.nb_steps(with_solution))

    def best_actions(self, with_solution = False):
        best_steps = self.best_steps(with_solution)
        for run in self.runs:
            if run.nb_steps() == best_steps:
                return run.actions()

    def avg_duration(self):
        if self.nb_runs() == 0:
            return 0
        return 1.0 * self.total_duration() / self.nb_runs()

    def nb_found_solution(self):
        if self.nb_runs() == 0:
            return 0
        return len(self.runs_with_solution())

    def pct_found_solution(self):
        if self.nb_runs() == 0:
            return 0
        return 1.0 * len(filter(lambda r: r.found_solution(), self.runs)) / self.nb_runs()

    def nb_not_found_solution(self):
        if self.nb_runs() == 0:
            return 0
        return len(filter(lambda r: r.found_solution() == False, self.runs))

    def analysis(self):
        return "\t" + "\n\t".join([
            "runs: %d" % self.nb_runs(),
            "best steps: %d" % self.best_steps(),
            "avg steps: %d" % self.avg_steps(),
            "avg steps for runs with solution: %d" % self.avg_steps(with_solution = True),
            "total steps: %d" % self.total_steps(),
            "best final reward: %d" % self.max_final_reward(),
            "worst final reward: %d" % self.min_final_reward(),
            "avg final reward: %d" % self.avg_final_reward(),
            "avg duration: %d" % self.avg_duration(),
            "durations: %s" % self.durations(),
            "number of runs that found solution: %d" % self.nb_found_solution(),
            "percentage of runs that found solution: %3f" % self.pct_found_solution(),
            "number of runs that did not find solution: %d" % self.nb_not_found_solution()
            ]) + "\n============"

    def __str__(self):
        return "Experiment for model {} and analysis:\n{}".format(self.model_file_name, self.analysis())


class ExperimentRun():
    def __init__(self, id):
        self.id = id
        self.steps = []
        self.start_time = int(time.time())
        self.end_time = self.start_time

    def actions(self):
        return map(lambda s: s.best_action, self.steps)

    def add_to_csv(self, w, model_file_name, timestamp, params):
        w.writerow([model_file_name, timestamp, json.dumps(params), self.id, self.nb_steps(), self.duration(), self.found_solution(), self.final_reward()])

    def log_step(self, step):
        self.steps.append(step)
        self.end_time = int(time.time())

    def get_rewards(self):
        return map(lambda step: step.total_reward, self.steps)

    def get_times(self):
        return map(lambda step: step.time, self.steps)

    def nb_steps(self):
        # Counts number of actions
        return len(filter(lambda step: step.best_action is not None and step.best_action != 'null', self.steps))

    def final_reward(self):
        return self.steps[-1].total_reward

    def found_solution(self):
        return self.nb_steps() < 200

    def duration(self):
        try:
            return self.end_time - self.start_time
        except AttributeError:
            return 0
