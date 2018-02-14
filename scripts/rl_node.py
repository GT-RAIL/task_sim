#!/usr/bin/env python
# Node to load a saved RL agent and run it with the executor

from __future__ import print_function, division

import os
import sys
import numpy as np

from threading import Lock

import rospy
import rospkg

from task_sim.msg import State, Action, Status
from task_sim.srv import SelectAction, QueryStatus
from task_sim.rl import tasks, learners

# Create a node that loads a trained RL Agent and lets `executor.py` evaluate it

class RLAgentRunner(object):
    """Runs a trained RL Agent"""

    def __init__(self):
        rospack = rospkg.RosPack()

        # First load up the agent
        agent_filename = rospy.get_param(
            '~agent_filename',
            os.path.join(
                rospack.get_path('task_sim'),
                'data', 'task1', 'models',
                'egreedy_qtable_rspecific_2018-02-14T16-41-36.pkl'
            )
        )
        self.agent = learners.EpsilonGreedyQTableAgent(None, None)
        self.agent.load(agent_filename)
        self.task = self.agent.task

        # ROS Services
        self.action_service = rospy.Service(
            '/table_sim/select_action', SelectAction, self.select_action
        )
        self.status_service = rospy.Service(
            '/table_sim/query_status', QueryStatus, self.query_status
        )

        # Implement locking on accessing the task
        self._task_lock = Lock()


    def select_action(self, req):
        """Return the action from the agent"""
        state = req.state
        with self._task_lock:
            self.task.set_world_state(state, None) # TODO: Figure out action hist
            reward = self.task.reward() # Just in case
            action = self.agent((state, reward), episode=None, train=False)
            action_msg = self.task.create_action_msg(action)
            self.task.reset()
            self.agent.reset()

        return action_msg

    def query_status(self, req):
        state = req.state
        with self._task_lock:
            self.task.set_world_state(state, None)
            status = self.task.status()
            self.task.reset()

        return Status(status)

if __name__ == '__main__':
    rospy.init_node('rl_node')
    rl_node = RLAgentRunner()
    rospy.loginfo("Ready to work")
    rospy.spin()
