#!/usr/bin/env python
# Node to learn the transition function from multiple training environments

from __future__ import print_function, division

# System imports
import os
import sys
import numpy as np

# ROS Imports
import rospy

# task_sim imports
from task_sim.str.modes import DemonstrationMode
from task_sim.str.amdp_value_iteration import AMDPValueIteration
from learn_transition_function import LearnTransitionFunction
from amdp_node import AMDPNode

# Helper functions and classes

# Trainer node

class AMDPTrainer(object):
    """Trains the AMDP on a subset of environments and outputs the learned
    transition functions and values"""

    def __init__(self):
        # First set the demonstration mode. TODO: This should be in an
        # experiment config object
        mode = 0
        if rospy.get_param('~demo_mode/random', True):
            mode |= DemonstrationMode.RANDOM
        if rospy.get_param('~demo_mode/shadow', True):
            mode |= DemonstrationMode.SHADOW
        if rospy.get_param('~demo_mode/classifier', True):
            mode |= DemonstrationMode.CLASSIFIER
        if rospy.get_param('~demo_mode/plan_network', False):
            mode |= DemonstrationMode.PLAN_NETWORK

        demo_mode = DemonstrationMode(mode)

# Main

if __name__ == '__main__':
    rospy.init_node('amdp_trainer')

    # TODO: Create an experiment config that provides the list of tasks and
    # seeds to the trainer. This config should also initialize the demo config
    # to use
    trainer = AMDPTrainer()
