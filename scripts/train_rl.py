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

# Create a node to interface between the agent and the task simulator

class RLAgentTrainer(object):
    """Trains an RL Agent"""

    def __init__(self):
        rospack = rospkg.RosPack()

        # Get the params for training RL Agents and tasks
        self.num_episodes = rospy.get_param('~num_episodes', 10000)
        self.alter_table_sim = rospy.get_param('~alter_table_sim', True)
        self.rate = rospy.get_param('~rate', -1)
        self.execute_post_episode = rospy.get_param('~execute_post_episode', 100)
        self.visdom_config = rospy.get_param(
            '~visdom_config',
            {
                'config_file': os.path.join(rospack.get_path('task_sim'), 'data', 'visdom_config.json'),
                'plot_frequency': 1,
            }
        )
        self.viz = VisdomVisualize(**self.visdom_config)

        # # Params for the DebugTask1
        # self.save_path = rospy.get_param(
        #     '~save_path',
        #     os.path.join(rospack.get_path('task_sim'), 'data', 'task3', 'models')
        # )
        # self.task = tasks.DebugTask1(
        #     rospy.get_param('~task/debug1/num_to_grab', 2),
        #     rospy.get_param('~task/debug1/state_vector_args', {}),
        #     rospy.get_param('~task/debug1/grab_reward', 1.0),
        #     rospy.get_param('~task/debug1/time_penalty', -0.4),
        #     rospy.get_param('~task/debug1/fail_penalty', -1.0),
        #     rospy.get_param('~task/debug1/timeout', 50)
        # )

        # Params for the Task1
        self.save_path = rospy.get_param(
            '~save_path',
            os.path.join(rospack.get_path('task_sim'), 'data', 'task1', 'models')
        )
        self.task = tasks.Task1(
            rospy.get_param('~task/1/state_vector_args', {}),
            rospy.get_param('~task/1/subgoal_reward', 500.0),
            rospy.get_param('~task/1/time_penalty', -0.5),
            rospy.get_param('~task/1/fail_penalty', -100.0),
            rospy.get_param('~task/1/timeout_penalty', -150.0),
            rospy.get_param('~task/1/timeout', 100)
        )

        # Params for epsilon-greedy agents.
        # Step decay of alpha.
        # Exponential decay of epsilon
        epsilon_start = rospy.get_param('~agent/epsilon_start', 0.15)
        epsilon_decay_factor = rospy.get_param('~agent/epsilon_decay_factor', 0.9995)
        # self.agent_epsilon = lambda eps: epsilon_start * (epsilon_decay_factor**eps)
        self.agent_epsilon = lambda eps: epsilon_start
        alpha_decay_factor = rospy.get_param('~agent/alpha_decay_factor', 1000)
        alpha_start = rospy.get_param('~agent/alpha_start', 0.1)
        # self.agent_alpha = lambda eps: alpha_start * alpha_decay_factor / (alpha_decay_factor + eps)
        # self.agent_alpha = lambda eps: alpha_start / np.ceil((eps+1)/alpha_decay_factor)
        # self.agent_alpha = lambda eps: alpha_start / ((eps//alpha_decay_factor) + 1)
        self.agent_alpha = lambda eps: alpha_start
        self.agent = learners.EpsilonGreedyQTableAgent(
            self.task,
            rospy.get_param('~agent/gamma', 0.99),
            self.agent_epsilon,
            self.agent_alpha,
            rospy.get_param('~agent/default_Q', 0.0)
        )
        self.save_prefix = rospy.get_param('~save_prefix', 'egreedy_q_table')
        self.save_suffix = rospy.get_param('~save_suffix', None)

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
            # self.viz.append_data(
            #     eps, status, 'status' if train else 'test_status', 'status', 'Episode'
            # )
            self.viz.append_data(
                eps, self.task.num_steps, 'steps' if train else 'test_steps', 'steps', 'Episode'
            )
            self.viz.append_data(
                eps, cumulative_reward, 'reward' if train else 'test_reward', 'reward', 'Episode'
            )

        # Return the accummulated reward if anyone is interested
        return status, self.task.num_steps, cumulative_reward

    def train(self):
        if self.rate > 0:
            sleep_rate = rospy.Rate(self.rate)

        # Get through all the training episodes and then save the agent
        for eps in xrange(self.num_episodes):

            # If we should plot the learning params, send to visdom
            # if (eps+1) % self.visdom_config.get('plot_frequency', eps+2) == 0:
            #     self.viz.append_data(
            #         eps, self.agent_epsilon(eps), 'epsilon', 'epsilon', 'Episode'
            #     )
            #     self.viz.append_data(
            #         eps, self.agent_alpha(eps), 'alpha', 'alpha', 'Episode'
            #     )

            # If this node must reset the world after every simulation, then
            if self.alter_table_sim:
                rospy.set_param('table_sim/seed', random.random())
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
                self.save_suffix or datetime.date.today().strftime("%Y-%m-%d")
            )
        )
        self.agent.save(agent_filename)


if __name__ == '__main__':
    rospy.init_node('train_rl')
    trainer = RLAgentTrainer()
    trainer.train()
