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
from task_sim.str.amdp_value_iteration import AMDPValueIteration
from learn_transition_function import LearnTransitionFunction
from amdp_node import AMDPNode

# Helper functions and classes

# Trainer node

class AMDPTrainer(object):
    """Trains the AMDP on a subset of environments and outputs the learned
    transition functions and values"""

    def __init__(self):
        pass


# Main

if __name__ == '__main__':
    pass
