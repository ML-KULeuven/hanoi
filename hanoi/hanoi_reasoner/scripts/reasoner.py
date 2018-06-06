#! /usr/bin/python

import rospkg
import rospy
import time
import pickle
from dc_pyutils import DCPlannerUtil
from hanoi_actuator.srv import Action
from hanoi_observer.msg import Observations

from parsed_action import ParsedAction
from plan_step import PlanStep
from experiment import Experiment, ExperimentRun

REFERENCE_TAG_ID = 1
LOG_DIR = rospkg.RosPack().get_path('hanoi_reasoner') + "/logs/"
NB_SAMPLES = 200


class Planner:
    """
    Used ROS params:
    * simulate: use simulated observer and actuator
    * initial_positions: provide a pickled list of initial positions
    * model_file: the path to the model file for the planner
    * used_horizon: the horizon to use for planning
    * runs: the number of runs for the experiment
    * experiment: whether or not to log this as an Experiment
    """
    def __init__(self, model_file_name, timestamp, experiment=None, run=1, used_horizon=10, *args, **kwargs):
        self.run = run
        self.used_horizon = used_horizon
        self.experiment = experiment
        if experiment is not None:
            print(experiment)
        self.use_simulation = rospy.get_param('simulate', False)
        self.model_file_name = model_file_name
        self.model_path = rospkg.RosPack().get_path('hanoi_reasoner') + \
            "/{}".format(model_file_name)
        with open(self.model_path, 'r+') as model:
            self.util = DCPlannerUtil(model.read(), NB_SAMPLES)
        self.initial_positions_path = rospy.get_param(
            'initial_positions', None)
        if self.initial_positions_path == "":
            self.initial_positions_path = None
        if not self.use_simulation:
            rospy.Subscriber('hanoi_observer', Observations,
                             self.callback_observations, queue_size=1)
        self.action_srv = rospy.ServiceProxy('hanoi_actuator_srv', Action)
        self.responses = []
        self.steps = []
        self.output_name = "{}_{}".format(self.model_file_name, timestamp)
        print(self)
        if self.use_simulation:
            self.experiment_run = ExperimentRun(run)
            self.simulate()

    def init_log(self):
        self.log_file = open(LOG_DIR + "{}.log".format(self.output_name), 'a')

    def format_observations(self, msg):
        return "[{}]".format(",".join(map(lambda o: self.format_observation(
            o), filter(lambda o: o.id != REFERENCE_TAG_ID, msg.observations))))

    def format_observation(self, observation):
        p = observation.pose.pose.position
        return "observation(pos({}))~=({},{},{})".format(observation.id, p.x, p.y, p.z)

    def parse_action(self, best_action, msg):
        return ParsedAction(best_action, msg)

    def format_positions(self, positions):
        return "[{}]".format(",".join(map(lambda (id, p): self.format_position(id, p), positions.iteritems())))

    def format_position(self, id, p):
        return "observation(pos({}))~=({},{},{})".format(id, p[0], p[1], p[2])

    def initial_positions(self):
        if self.initial_positions_path is not None:
            with open(self.initial_positions_path, 'rb') as f:
                return pickle.load(f)[self.run-1]

        return {
            2: (1.0, 0.0, 0.0),
            3: (2.0, 0.0, 0.0),
            4: (3.0, 0.0, 0.0),
            5: (4.0, 0.0, 0.0),
            6: (1.0, 4.0, 0.0),
            7: (1.0, 3.0, 0.0),
            8: (1.0, 2.0, 0.0),
            9: (1.0, 1.0, 0.0),
        }

    def simulate(self):
        positions = self.initial_positions()
        rospy.loginfo("Run {} starting with initial positions {}".format(
            self.run, positions))
        stop = False
        i = 0
        while not stop:
            i += 1
            rospy.loginfo("Run {} simulation step {}".format(self.run, i))
            obs_str = self.format_positions(positions)
            resp = self.do_step(obs_str)
            self.experiment_run.log_step(
                PlanStep(i, resp.best_action, resp.time, resp.total_reward, resp.stop))
            self.responses.append(resp)
            stop = resp.stop
            if stop:
                self.stop()
                return
            best_action = self.parse_action(resp.best_action, None)
            to_point = best_action.to_point()
            positions[best_action.disk] = (to_point.pose.position.x, to_point.pose.position.y, to_point.pose.position.z)

    def callback_observations(self, msg):
        # Format observations to string for input into HYPE
        obs_str = self.format_observations(msg)

        resp = self.do_step(obs_str)

        # Check stop condition
        if resp.stop:
            self.stop()
            return

        # Parse resulting action
        best_action = self.parse_action(resp.best_action, msg)

        # Create Action request
        print(best_action)
        req = best_action.create_request()

        # Send Action request to hanoi_actuator and wait
        rospy.loginfo('Waiting for hanoi_actuator_srv')
        rospy.wait_for_service('hanoi_actuator_srv')
        result = self.action_srv(req)
        rospy.loginfo(
            "ActionResponse: {} --> {}".format(result.success, result.message))

    def stop(self):
        if self.experiment is not None:
            self.experiment.add_run(self.experiment_run)
            self.log_experiment()

    def log_experiment(self):
        with open(LOG_DIR + "{}_experiment.pick".format(self.output_name), 'wb') as f:
            pickle.dump(self.experiment, f)

    def do_step(self, obs_str):
        # Call dc_bridge plan step
        try:
            resp = self.util.plan_step(
                # obs_str, False, 300, 50, self.used_horizon)
                obs_str, False, 1000, 200, self.used_horizon)
                # obs_str, False, 1000, 200, self.used_horizon)
            rospy.loginfo(resp)
        except Exception, e:
            rospy.logerr(e)
            rospy.signal_shutdown('Error occurred.')
        return resp

    def __str__(self):
        return "Planner {} [simulated: {}, experiment: {}]".format(self.output_name, self.use_simulation, self.experiment is not None)


def main():
    rospy.init_node('hanoi_reasoner', disable_signals=True)
    timestamp = int(time.time())
    model_file_name = rospy.get_param('model_file', 'hanoi.pl')
    used_horizon = rospy.get_param('used_horizon', 10)

    experiment = None
    runs = rospy.get_param("runs", 1)
    if rospy.get_param("experiment", False):
        experiment = Experiment(model_file_name, timestamp)
        for p in rospy.get_param_names():
            experiment.param(p, rospy.get_param(p, None))

    if rospy.get_param("simulate", True):
        for run in range(1, runs + 1):
            print("==== STARTING RUN {} OUT OF {} ====".format(run, runs))
            Planner(model_file_name, timestamp, experiment, run, used_horizon)
    else:
        Planner(model_file_name, timestamp,
                experiment, used_horizon=used_horizon)
        rospy.spin()

    rospy.signal_shutdown('Done!')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
