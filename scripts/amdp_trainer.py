#!/usr/bin/env python
# Node to learn the transition function from multiple training environments

from __future__ import print_function, division

# System imports
import os
import sys
import pickle
import datetime
import threading
import multiprocessing
import numpy as np

# ROS Imports
import rospy
import rospkg
from std_srvs.srv import Empty

# task_sim imports
from task_sim.msg import Action, State, Status
from task_sim.srv import Execute, QueryState, QueryStatus, SelectAction
from task_sim.str.modes import DemonstrationMode
from task_sim.str.amdp_value_iteration import AMDPValueIteration
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned
from learn_transition_function import LearnTransitionFunction
from amdp_node import AMDPNode

# Helper functions and classes

# Trainer node

class AMDPTrainer(object):
    """Trains the AMDP on a subset of environments and outputs the learned
    transition functions and values"""

    def __init__(self):
        root_path = rospkg.RosPack().get_path('task_sim')

        # TODO: come up with something more permanent
        self.report = []

        # Assume that there is an experiment data folder. #TODO: Get this from
        # an experiment config in the future
        self.experiment_data_folder = ""

        # First set the demonstration mode. TODO: This should be in an
        # experiment config object
        mode = 0
        if rospy.get_param('~demo_mode/random', True):
            mode |= DemonstrationMode.RANDOM
        if rospy.get_param('~demo_mode/shadow', False):
            mode |= DemonstrationMode.SHADOW
        if rospy.get_param('~demo_mode/classifier', False):
            mode |= DemonstrationMode.CLASSIFIER
        if rospy.get_param('~demo_mode/plan_network', True):
            mode |= DemonstrationMode.PLAN_NETWORK

        self.demo_mode = DemonstrationMode(mode)

        self.baseline_mode = rospy.get_param('~baseline_mode', False)

        # Create the different amdp_ids
        self.amdp_ids = (0,1,2,6,7,8,4,11,12,) # definition in amdp_node.py

        # Get the tasks and amdp_ids for the task. #TODO: When we have different
        # task envs, this should be more than just empty. Also the seeds should
        # be picked according to the experiment's configuration
        self.task_envs = [] # Format: seed, task folder
        for i in range(20):
            self.task_envs.append((i, ""))
        self.demo_envs = [ # Format: env_name, amdp_ids
            ("task4", (0,2)),
            ("task7", (6,8))
        ]
        self.test_envs = range(20,120) # test environment seeds

        # Instantiate the transition functions and the utility functions
        self.Ts = {}
        self.Us = {}
        for amdp_id in self.amdp_ids:
            if amdp_id not in (1,7,): # Don't repeat transition functions
                transition_filename = None if amdp_id in (4,11,12,) else "T{}.hdf5".format(amdp_id)
                self.Ts[amdp_id] = AMDPTransitionsLearned(amdp_id, transition_filename)
            else:
                self.Ts[amdp_id] = self.Ts[amdp_id-1]

            # Initialize the value functions
            self.Us[amdp_id] = AMDPValueIteration(amdp_id, self.Ts[amdp_id])
            if amdp_id in (4,11,12,): # Pre-calculated high-level value functions
                with open(
                    os.path.join(root_path, 'src/task_sim/str/U{}.pkl'.format(amdp_id)),
                    'rb'
                ) as fd:
                    self.Us[amdp_id].U = pickle.load(fd)

        # Instantiate interfaces to the environments. Always have 5 environments
        self.simulators = {} # Format: (amdp_id, simulator_name). None -> full task
        self.simulators[0] = rospy.get_param('~simulators/drawer_oc', 'drawer1')
        self.simulators[2] = rospy.get_param('~simulators/drawer_p', 'drawer2')
        self.simulators[6] = rospy.get_param('~simulators/box_oc', 'box1')
        self.simulators[8] = rospy.get_param('~simulators/box_p', 'box2')
        self.simulators[None] = rospy.get_param('~simulators/eval', 'eval')

        self.simulator_api = {
            name: {
                'reset_sim': rospy.ServiceProxy(name+'/reset_simulation', Empty),
                'seed_param_name': name+'/seed',
                'execute': rospy.ServiceProxy(name+'/execute_action', Execute),
                'query_state': rospy.ServiceProxy(name+'/query_state', QueryState),
                'query_status': rospy.ServiceProxy(name+'/query_status', QueryStatus),
                'select_action': rospy.ServiceProxy(name+'/select_action', SelectAction),
            }
            for (idx,name) in self.simulators.iteritems()
        }

        self.max_episode_length = rospy.get_param('~max_episode_length', 100)

        # Instantiate the transition function learners
        self.transition_learners = {}
        self.demo_configs = {}
        for container, amdp_ids in self.demo_envs:
            for amdp_id in amdp_ids:
                self.demo_configs[(container, amdp_id,)] = \
                    self.demo_mode.configuration(
                        container_env=container, amdp_id=amdp_id
                    )
                self.transition_learners[(container, amdp_id,)] = \
                    LearnTransitionFunction(
                        amdp_id, container,
                        self.simulators[amdp_id],
                        self.Ts[amdp_id],
                        self.demo_mode, self.demo_configs[(container, amdp_id,)],
                        self.max_episode_length
                    )

        # Instantiate the AMDP Node
        self.amdp_node = AMDPNode(self.simulators[None], self.Ts, self.Us, self.demo_mode, self.baseline_mode)

    def _run_transition_learner(self, learner, epoch):
        while learner.epoch == epoch:
            learner.run()

    def train(self, epochs=2502, test_every=10, save_every=100):
        """Trains the transition function, the value function, etc.
        TODO: Maybe some of the options here should be part of the experiment
        config"""
        epoch = 0
        num_envs = len(self.task_envs)
        start = datetime.datetime.now()

        # Keep track of the number of executions and successes of the test
        amdp_node_executions = amdp_node_successes = 0

        if self.baseline_mode:
            # Evaluation only
            self.amdp_node.reinit_U()

            eval_trials = 5
            print("Evaluating for", eval_trials, "trials over all training environments...")
            success_rate_demo = 0.0
            success_rate_train = 0.0
            for env in self.task_envs:
                eval_seed = env[0]
                for i in range(eval_trials):
                    if self.evaluate(eval_seed):
                        if eval_seed < 10:
                            success_rate_demo += 1
                        else:
                            success_rate_train += 1
                        amdp_node_successes += 1
                    amdp_node_executions += 1

            print("Evaluating over all", len(self.test_envs), "heldout test environments...")
            success_rate_test = 0.0
            for test_seed in self.test_envs:
                if self.evaluate(test_seed):
                    success_rate_test += 1

            print("**********************************************************************************")
            print(
                "\nTests:", amdp_node_executions,
                "Successes:", amdp_node_successes
            )

            ex_count = 0
            for key, transition_learner in self.transition_learners.iteritems():
                ex_count += transition_learner.action_executions
            rate_demo = success_rate_demo/(10*eval_trials)
            rate_train = success_rate_train/((len(self.task_envs) - 10)*eval_trials)
            rate_combined_train = (success_rate_demo + success_rate_train)/(len(self.task_envs)*eval_trials)
            rate_test = success_rate_test/(len(self.test_envs))
            print("Epoch:", epoch,
                  "\tSuccess (demo):", rate_demo,
                  "\tSuccess (train):", rate_train,
                  "\tSuccess (combined):", rate_combined_train,
                  "\tSuccess (test):", rate_test,
                  "\tAction executions:", ex_count)
            return

        while epoch < epochs:
            current_seed = self.task_envs[(epoch%num_envs)][0]

            learn_workers = []
            for key, transition_learner in self.transition_learners.iteritems():
                # print("Training:", key, end=' ')
                simulator_api = self.simulator_api[self.simulators[key[1]]]
                rospy.set_param(simulator_api['seed_param_name'], current_seed)
                simulator_api['reset_sim']()

                worker = threading.Thread(
                    target=self._run_transition_learner,
                    args=(transition_learner, epoch,)
                )
                learn_workers.append((key, transition_learner, worker,))
                worker.start()

            for key, transition_learner, worker in learn_workers:
                worker.join()
                print(
                    "Trained:", key,
                    "Epoch:", transition_learner.epoch,
                    "\tSuccesses:", transition_learner.successes,
                    "\tAction executions:", transition_learner.action_executions
                )

            # rospy.sleep(1)

            # If it is time to test
            if epoch % test_every == 0 and epoch > 0:
                # TODO: Need better state management for speedup with
                # multiprocessing here
                for amdp_id, value_iterator in self.Us.iteritems():
                    # Don't run value iteration for the top-level AMDPs
                    if amdp_id in [4,11,12]:
                        continue

                    print("Solving:", amdp_id)
                    value_iterator.init_utilities()
                    value_iterator.solve()

                self.amdp_node.reinit_U()

                eval_trials = 5
                print("Evaluating for", eval_trials, "trials over all training environments...")
                success_rate_demo = 0.0
                success_rate_train = 0.0
                self.actions_from_learned_policy = 0
                self.total_actions = 0
                for env in self.task_envs:
                    eval_seed = env[0]
                    for i in range(eval_trials):
                        if self.evaluate(eval_seed):
                            if eval_seed < 10:
                                success_rate_demo += 1
                            else:
                                success_rate_train += 1
                            amdp_node_successes += 1
                        amdp_node_executions += 1
                rate_action_from_utility = float(self.actions_from_learned_policy)/self.total_actions
                self.actions_from_learned_policy = 0
                self.total_actions = 0

                print("Evaluating over all", len(self.test_envs), "heldout test environments...")
                success_rate_test = 0.0
                for test_seed in self.test_envs:
                    if self.evaluate(test_seed):
                        success_rate_test += 1
                rate_action_from_utility_test = float(self.actions_from_learned_policy)/self.total_actions

                print("**********************************************************************************")
                print(
                    "\nTests:", amdp_node_executions,
                    "Successes:", amdp_node_successes
                )

                ex_count = 0
                for key, transition_learner in self.transition_learners.iteritems():
                    ex_count += transition_learner.action_executions
                rate_demo = success_rate_demo/(10*eval_trials)
                rate_train = success_rate_train/((len(self.task_envs) - 10)*eval_trials)
                rate_combined_train = (success_rate_demo + success_rate_train)/(len(self.task_envs)*eval_trials)
                rate_test = success_rate_test/(len(self.test_envs))

                print("Epoch:", epoch,
                      "\tSuccess (demo):", rate_demo,
                      "\tSuccess (train):", rate_train,
                      "\tSuccess (combined):", rate_combined_train,
                      "\tSuccess (test):", rate_test,
                      "\tAction executions:", ex_count,
                      "\tLearned action selection rate (train combined):", rate_action_from_utility,
                      "\tLearned action selection rate (test combined):", rate_action_from_utility_test)
                for key, transition_learner in self.transition_learners.iteritems():
                    print('\t', key, transition_learner.action_executions, 'training action executions')
                print("\n**********************************************************************************")
                self.report.append((epoch, rate_demo, rate_train, rate_combined_train, rate_test, ex_count,
                                    rate_action_from_utility, rate_action_from_utility_test))
                print("Cumulative results: ")
                print(self.report)

            # If it is time to save
            if epoch % save_every == 0 and epoch > 0:
                # TODO: Need to save the transition functions and the value tables
                pass

            epoch += 1

    def evaluate(self, eval_seed):
        simulator_api = self.simulator_api[self.simulators[None]]
        rospy.set_param(simulator_api['seed_param_name'], eval_seed)
        simulator_api['reset_sim']()
        num_steps = 0

        status = Status.IN_PROGRESS
        while status == Status.IN_PROGRESS:
            if num_steps > self.max_episode_length:
                status = Status.TIMEOUT
                break

            state = simulator_api['query_state']().state
            selected_action = simulator_api['select_action'](state, Action())
            action = selected_action.action
            next_state = simulator_api['execute'](action)
            status = simulator_api['query_status'](next_state.state).status.status_code

            self.total_actions += 1
            if selected_action.action_source == 1:
                self.actions_from_learned_policy += 1

            num_steps += 1
            # rospy.sleep(0.5)

        return status == Status.COMPLETED


# Main
if __name__ == '__main__':
    rospy.init_node('amdp_trainer')

    # TODO: Create an experiment config that provides the list of tasks and
    # seeds to the trainer. This config should also initialize the demo config
    # to use
    trainer = AMDPTrainer()
    trainer.train()
