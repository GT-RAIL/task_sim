#!/usr/bin/env python
# Node to train a specified RL learner

from __future__ import print_function, division

import os
import sys
import datetime
import random
import numpy as np

import rospy
import rospkg
from std_srvs.srv import Empty

from task_sim.msg import State, Action, Status
from task_sim.srv import Execute, QueryState
from task_sim.rl import tasks, learners
from task_sim.visdom_visualize import VisdomVisualize

# Helper functions

def get_path(path):
    """Takes a path relative to top level package and gets absolute path"""
    rospack = rospkg.RosPack()
    if not os.path.exists(path):
        path = os.path.join(rospack.get_path('task_sim'), path)
    return path

# Create a node to interface between the agent and the task simulator

class RLAgentTrainer(object):
    """Trains an RL Agent"""

    def __init__(self):
        # Get the params for training RL Agents and tasks
        self.num_episodes = rospy.get_param('~num_episodes', 10000)
        self.change_seeds = rospy.get_param('~change_seeds', True)
        self.rate = rospy.get_param('~rate', -1)
        self.execute_post_episode = rospy.get_param('~execute_post_episode', 1000)
        self.visdom_config = rospy.get_param(
            '~visdom_config',
            {
                'config_file': 'visdom_config.json',
                'plot_frequency': 1,
            }
        )
        self.visdom_config['config_file'] = get_path( # Get the full path
            self.visdom_config.get('config_file', '')
        )

        # If the visualize option is OFF, then make sure that we are not going
        # to try to visualize later in the code
        if self.visdom_config.get("visualize", True):
            self.viz = VisdomVisualize(**self.visdom_config)
        else:
            self.viz = None
            if self.visdom_config.has_key('plot_frequency'):
                del self.visdom_config['plot_frequency']

        # Set the parameters to save the agent
        self.save_path = rospy.get_param(
            '~save_path',
            os.path.join('data', 'task1', 'models')
        )
        self.save_prefix = rospy.get_param('~save_prefix', 'egreedy_q_table')
        self.save_suffix = rospy.get_param('~save_suffix', None)
        self.save_path = get_path(self.save_path)


        # Params and initialization for the Task
        self.task_params = rospy.get_param('~task', {})
        self.task_params['viz'] = self.viz
        self.task = tasks.Task1(**self.task_params)

        # Params for the agent
        self.agent_params = rospy.get_param('~agent', {})
        self.agent_params['task'] = self.task
        self.agent_params['viz'] = self.viz

        # Processing for epsilon-greedy agents.
        epsilon_start = self.agent_params.get('epsilon_start', 0.15)
        epsilon_decay_factor = self.agent_params.get('epsilon_decay_factor', 0.9995)
        # epsilon = lambda eps: epsilon_start * (epsilon_decay_factor**eps)
        epsilon = lambda eps: epsilon_start
        self.agent_params['epsilon'] = epsilon

        alpha_decay_factor = self.agent_params.get('alpha_decay_factor', 1000)
        alpha_start = self.agent_params.get('alpha_start', 0.1)
        # alpha = lambda eps: alpha_start * alpha_decay_factor / (alpha_decay_factor + eps)
        # alpha = lambda eps: alpha_start / np.ceil((eps+1)/alpha_decay_factor)
        # alpha = lambda eps: alpha_start / ((eps//alpha_decay_factor) + 1)
        alpha = lambda eps: alpha_start
        self.agent_params['alpha'] = alpha

        # Initialize the agent
        self.agent = learners.EpsilonGreedyQTableAgent(**self.agent_params)

        # Create services for communicating with table_sim
        self.execute = rospy.ServiceProxy('table_sim/execute_action', Execute)
        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.reset_simulation = rospy.ServiceProxy('table_sim/reset_simulation', Empty)

    def _episode(self, eps, train=True, print_actions=False):
        """Run an episode. If `train`, then update agent parameters"""
        action = action_msg = None
        cumulative_reward = 0.0
        state = self.query_state().state
        self.task.set_world_state(state, action)
        status = Status.IN_PROGRESS # We need to store the terminal Q value
        while status == Status.IN_PROGRESS:
            status = self.task.status()
            reward = self.task.reward()
            cumulative_reward += reward
            action = self.agent((state, reward), episode=eps, train=train)
            if print_actions:
                rospy.loginfo("\tExecuting {}".format(action))

            if action is not None:
                action_msg = self.task.create_action_msg(action)
                state = self.execute(action_msg).state
                self.task.increment_steps()
                self.task.set_world_state(state, action)

            if self.rate > 0:
                sleep_rate.sleep()

        # Completed an episode
        rospy.loginfo(
            "Episode {}: Status - {}, Reward - {}"
            .format(eps, status, cumulative_reward)
        )
        # If we should plot the learning params, send to visdom
        if (eps+1) % self.visdom_config.get('plot_frequency', eps+2) == 0:
            # self.viz.update_line(
            #     eps, status,
            #     'status' if train else 'test_status', 'status', 'Episode'
            # )
            self.viz.update_line(
                eps, self.task.num_steps,
                'steps' if train else 'test_steps', 'steps', 'Episode'
            )
            self.viz.update_line(
                eps, cumulative_reward,
                'reward' if train else 'test_reward', 'reward', 'Episode'
            )

        # Return the accummulated reward if anyone is interested
        return status, self.task.num_steps, cumulative_reward

    def train(self):
        # Set a rate if so desired
        if self.rate > 0:
            sleep_rate = rospy.Rate(self.rate)

        # Create a variable for the last seed
        should_update_seed = not (not self.change_seeds)
        seed_idx = (
            0 if should_update_seed and type(self.change_seeds) == list else None
        )

        # Get through all the training episodes and then save the agent
        for eps in xrange(self.num_episodes):

            # If we should plot the learning params, send to visdom
            # if (eps+1) % self.visdom_config.get('plot_frequency', eps+2) == 0:
            #     self.viz.update_line(
            #         eps, self.agent_params['epsilon'](eps),
            #         'epsilon', 'epsilon', 'Episode'
            #     )
            #     self.viz.update_line(
            #         eps, self.agent_params['alpha'](eps),
            #         'alpha', 'alpha', 'Episode'
            #     )

            # If this node must reset the world after every simulation, then
            if should_update_seed:
                if seed_idx is None:
                    rospy.set_param('table_sim/seed', random.random())
                else:
                    rospy.set_param('table_sim/seed', self.change_seeds[seed_idx])
                    seed_idx = (seed_idx+1) % len(self.change_seeds)
            self.reset_simulation()

            # Run the training episode
            self._episode(eps)
            # raw_input()

            # Update pi. Execute a policy if we want to test in the middle
            if self.execute_post_episode > 0 \
            and (eps+1) % self.execute_post_episode == 0:
                self.agent.update_pi()

                self.task.reset()
                self.agent.reset()
                self.reset_simulation()

                self._episode(eps, train=False, print_actions=False)
                # DEBUG Q Table
                # if eps > 0:
                #     d1 = np.array(self.agent.Q.values())
                #     d2 = np.unique(d1, return_counts=True)
                #     for d in zip(*d2):
                #         print(d)
                #     raw_input()
                # raw_input()

            self.task.reset()
            self.agent.reset()

        # Completed training. Save the agent
        agent_filename = os.path.join(
            self.save_path,
            "{}_{}.pkl".format(
                self.save_prefix,
                self.save_suffix or datetime.date.now().strftime("%Y-%m-%dT%H-%M-%S")
            )
        )
        self.agent.save(agent_filename)


if __name__ == '__main__':
    rospy.init_node('train_rl')
    trainer = RLAgentTrainer()
    rospy.sleep(3.0) # Allow the table_sim to setup
    trainer.train()
