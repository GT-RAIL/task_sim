#!/usr/bin/env python
# Node to test a pre-trained AMDP on a physical system (Nimbus)

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

class AMDPTester(object):
    """Test a pre-trained AMDP on a physical system"""

    def __init__(self):
        root_path = rospkg.RosPack().get_path('task_sim')

        # First set the demonstration mode. TODO: This should be in an
        # experiment config object
        mode = 0
        if rospy.get_param('~demo_mode/random', False):
            mode |= DemonstrationMode.RANDOM
        if rospy.get_param('~demo_mode/shadow', False):
            mode |= DemonstrationMode.SHADOW
        if rospy.get_param('~demo_mode/classifier', True):
            mode |= DemonstrationMode.CLASSIFIER
        if rospy.get_param('~demo_mode/plan_network', True):
            mode |= DemonstrationMode.PLAN_NETWORK

        self.demo_mode = DemonstrationMode(mode)

        # Create the different amdp_ids
        self.amdp_ids = (0,1,2,6,7,8,4,11,12,) # definition in amdp_node.py

        # Instantiate the transition functions and the utility functions
        self.Ts = {}
        self.Us = {}
        for amdp_id in self.amdp_ids:
            if amdp_id not in (1,7,): # Don't repeat transition functions
                transition_filename = None if amdp_id in (4,11,12,) else "T{}.hdf5".format(amdp_id)
                self.Ts[amdp_id] = AMDPTransitionsLearned(amdp_id, transition_filename, reinit=False)
            else:
                self.Ts[amdp_id] = self.Ts[amdp_id-1]

            # Initialize the value functions
            self.Us[amdp_id] = AMDPValueIteration(amdp_id, self.Ts[amdp_id])
            # if amdp_id in (4,11,12,): # Pre-calculated high-level value functions
            #     with open(
            #         os.path.join(root_path, 'src/task_sim/str/U{}.pkl'.format(amdp_id)),
            #         'rb'
            #     ) as fd:
            #         self.Us[amdp_id].U = pickle.load(fd)
            with open(
                os.path.join(root_path, 'src/task_sim/str/U{}.pkl'.format(amdp_id)),
                'rb'
            ) as fd:
                self.Us[amdp_id].U = pickle.load(fd)

        # Instantiate the AMDP Node
        self.amdp_node = AMDPNode('amdp', self.Ts, self.Us, self.demo_mode, continuous=True)

        # Set up service clients for testing
        self.select_action = rospy.ServiceProxy('amdp/select_action', SelectAction)
        self.query_state = rospy.ServiceProxy('state_calculator_node/calculate_state', QueryState)
        self.execute = rospy.ServiceProxy('controller_node/execute', Execute)

    def run(self):

        for amdp_id, value_iterator in self.Us.iteritems():
            # Don't run value iteration for the top-level AMDPs
            if amdp_id in [4,11,12]:
                continue

            # print("Solving:", amdp_id)
            # value_iterator.init_utilities()
            # value_iterator.solve()

            # print("Saving:", amdp_id)
            # value_iterator.save()

        self.amdp_node.reinit_U()

        restart = 'y'
        while restart == 'y':
            state = self.query_state().state
            for i in range(100):
                selected_action = self.select_action(state, Action())

                action = selected_action.action

                print('Selected action: ' + str(action.action_type) + ', ' + str(action.object))
                print('(press e to execute, q to quit)')
                s = ''
                while (s != 'e' and s != 'q'):
                    s = raw_input('(e/q) >> ')
                if s == 'q':
                    break

                state = self.execute(action).state

            print('AMDP testing finished.  Restart?')
            restart = raw_input('(y/n) >> ')

        print('Exiting.')

# Main
if __name__ == '__main__':
    rospy.init_node('amdp_tester')

    tester = AMDPTester()
    tester.run()
