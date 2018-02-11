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

# Create a node to interface between the agent and the task simulator

class RLAgentTrainer(object):
    """Trains an RL Agent"""

    def __init__(self):
        rospack = rospkg.RosPack()

        # Get the params for training RL Agents and tasks
        self.num_episodes = rospy.get_param('~num_episodes', 500)
        self.alter_table_sim = rospy.get_param('~alter_table_sim', True)
        self.rate = rospy.get_param('~rate', -1)
        self.execute_post_episode = rospy.get_param('~execute_post_episode', 3)

        # Params for the DebugTask1
        self.save_path = rospy.get_param(
            '~save_path',
            os.path.join(rospack.get_path('task_sim'), 'data', 'task3', 'models')
        )
        self.task = tasks.DebugTask1(
            rospy.get_param('~task/debug1/num_to_grab', 2),
            rospy.get_param('~task/debug1/state_vector_args', {}),
            rospy.get_param('~task/debug1/grab_reward', 10.0),
            rospy.get_param('~task/debug1/time_penalty', -0.4),
            rospy.get_param('~task/debug1/fail_penalty', -10.0),
            rospy.get_param('~task/debug1/timeout', 30)
        )

        # Params for epsilon-greedy agents.
        # Step decay of alpha.
        # Exponential decay of epsilon
        epsilon_start = rospy.get_param('~agent/epsilon_start', 0.1)
        epsilon_decay_factor = rospy.get_param('~agent/epsilon_decay_factor', 0.99)
        alpha_decay_factor = rospy.get_param('~agent/alpha_decay_factor', 10)
        alpha_start = rospy.get_param('~agent/alpha_start', 0.25)
        self.agent = learners.EpsilonGreedyQTableAgent(
            self.task,
            rospy.get_param('~agent/gamma', 0.9),
            lambda eps: epsilon_start * (epsilon_decay_factor**eps),
            # lambda eps: alpha_start * alpha_decay_factor / (alpha_decay_factor + eps),
            lambda eps: alpha_start / np.ceil(eps/alpha_decay_factor),
            rospy.get_param('~agent/default_Q', -10.0)
        )
        self.save_prefix = rospy.get_param('~save_prefix', 'egreedy_q_table')
        self.save_suffix = rospy.get_param('~save_suffix', None)

        # Create services for communicating with table_sim
        self.execute = rospy.ServiceProxy('table_sim/execute_action', Execute)
        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.reset_simulation = rospy.ServiceProxy('table_sim/reset_simulation', Empty)

    def train(self):
        if self.rate > 0:
            sleep_rate = rospy.Rate(self.rate)

        # Get through all the training episodes and then save the agent
        for eps in xrange(self.num_episodes):

            # If this node must reset the world after every simulation, then
            if self.alter_table_sim:
                rospy.set_param('table_sim/seed', random.random())
            self.reset_simulation()

            # Start the process of training the agent
            status = Status.IN_PROGRESS
            action = action_msg = None
            state = self.query_state().state
            while status == Status.IN_PROGRESS:
                self.task.set_world_state(state, action)
                reward = self.task.reward()
                action = self.agent((state, reward), episode=eps, train=True)
                rospy.loginfo("Reward {}, Executing {}".format(reward, action))

                action_msg = self.task.create_action_msg(state, action)
                state = self.execute(action_msg).state
                self.task.increment_steps()

                status = self.task.status()

                if self.rate > 0:
                    sleep_rate.sleep()

                # raw_input()

            # Completed an episode
            rospy.loginfo("Episode {}, Status: {}".format(eps, status))

            # Update pi. Execute a policy and get the cumulative reward
            if self.execute_post_episode > 0 and eps % self.execute_post_episode == 0:
                self.agent.update_pi()

                # self.reset_simulation()
                # test_status = Status.IN_PROGRESS
                # test_state = self.query_state().state
                # test_action = None
                # cumulative_reward = 0.0

                # self.task.reset()
                # self.agent.reset()
                # while test_status


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
