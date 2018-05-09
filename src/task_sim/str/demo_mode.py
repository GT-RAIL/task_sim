#!/usr/bin/env python
# File with the definitions and configurations on how to use the demos

from __future__ import print_function, division

import os
import sys
import glob
import copy
import numpy as np
from sklearn.externals import joblib

import rospkg
import rosbag
from geometry_msgs.msg import Point

from task_sim.msg import Action
from task_sim import data_utils as DataUtils
from task_sim.oomdp.oo_state import OOState
from task_sim.str.amdp_state import AMDPState
from task_sim.str.stochastic_state_action import StochasticAction

class DemonstrationMode(object):
    """Enum definitions of the demo modes"""
    RANDOM = 1
    SHADOW = 1<<1
    CLASSIFIER = 1<<2
    PLAN_NETWORK = 1<<3

    def __init__(self, mode):
        """`mode` is the bitwise OR of the available modes"""
        self.mode = mode

    def configuration(self, **task_config):
        """Returns a dictionary of the pertinent objects given the selected
        mode and the configs for that mode. The configs are passed in as
        keyword arguments.

        TODO: Add configs like alpha and epsilon to the demo mode config?
        """
        demo_config = {}

        # Check each of the modes in turn. Start with Random
        if (self.mode & DemonstrationMode.RANDOM) > 0:
            # Nothing needed for random
            pass

        # Check if we should return a demonstration policy
        if (self.mode & DemonstrationMode.SHADOW) > 0:
            # Load the demonstration(s) and create a policy
            demo_task = task_config['demo_task'] # TODO: The following assertion should be updated
            assert (demo_task == 'task4' or demo_task == 'task7'), "Unknown task demo: {}".format(demo_task)
            amdp_id = task_config['amdp_id']
            assert (amdp_id in [0,1,2,6,7,8]), "Unknown AMDP ID: {}".format(amdp_id)

            print("Loading demonstrations for", demo_task, '...')
            demo_list = glob.glob(os.path.join(
                rospkg.RosPack().get_path('task_sim'),
                'data',
                demo_task,
                'demos/*.bag'
            ))
            print("Found", len(demo_list), 'demonstrations.')

            sa_pairs = []
            prev_state_msg = None
            for demo_file in demo_list:
                print("Reading", demo_file, "...")
                bag = rosbag.Bag(demo_file)
                for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
                    if prev_state_msg is None:
                        prev_state_msg = copy.deepcopy(msg.state)

                    elif msg.action.action_type != Action.NOOP:
                        pair = (copy.deepcopy(prev_state_msg), copy.deepcopy(msg.action))
                        sa_pairs.append(pair)
                        prev_state_msg = copy.deepcopy(msg.state)
                bag.close()

            pi = {}
            for pair in sa_pairs:
                state_msg = pair[0]
                s = AMDPState(amdp_id=amdp_id, state=OOState(state=state_msg))
                a = pair[1]

                # convert action into something that fits into the new action list
                if a.action_type == Action.PLACE:
                    a.object = DataUtils.get_task_frame(state_msg, a.position)
                    a.position = Point()
                elif a.action_type == Action.MOVE_ARM:
                    a.object = DataUtils.get_task_frame(state_msg, a.position)
                    if a.object != 'stack' and a.object != 'drawer' and a.object != 'box' and a.object != 'lid':
                        for o in state_msg.objects:
                            if o.name != 'apple' or o.name != 'banana' or o.name != 'carrot':
                                continue
                            if a.position == o.position:
                                a.object = o.name
                                break

                        if a.object != 'apple' or a.object != 'banana' or a.object != 'carrot':
                            x = state_msg.gripper_position.x
                            y = state_msg.gripper_position.y
                            px = a.position.x
                            py = a.position.y
                            if px == x and py > y:
                                a.object = 'b'
                            elif px < x and py > y:
                                a.object = 'bl'
                            elif px < x and py == y:
                                a.object = 'l'
                            elif px < x and py < y:
                                a.object = 'fl'
                            elif px == x and py < y:
                                a.object = 'f'
                            elif px > x and py < y:
                                a.object = 'fr'
                            elif px > x and py == y:
                                a.object = 'r'
                            else:
                                a.object = 'br'
                    a.position = Point()
                elif a.action_type == Action.GRASP:
                    a.position = Point()
                else:
                    a.position = Point()
                    a.object = ''

                # update the policy
                if s in pi:
                    pi[s].update(a)
                else:
                    pi[s] = StochasticAction(a)

            # insert the policy into the demo_config
            demo_config['demo_policy'] = pi

        # Check if there's a classifier that we need to return
        if (self.mode & DemonstrationMode.CLASSIFIER) > 0:
            demo_task = task_config['demo_task'] # TODO: The following assertion should be updated
            assert (demo_task == 'task4' or demo_task == 'task7'), "Unknown task demo: {}".format(demo_task)
            amdp_id = task_config['amdp_id']
            assert (amdp_id in [0,1,2,6,7,8]), "Unknown AMDP ID: {}".format(amdp_id)

            classifier_name = 'decision_tree_action_{}.pkl'.format(amdp_id)
            classifier_path = os.path.join(
                rospkg.RosPack().get_path('task_sim'),
                'data',
                demo_task,
                'models',
                classifier_name
            )
            print("Loading classifier at", classifier_path)
            demo_config['action_bias'] = joblib.load(classifier_path)

        # All configs are loaded. Return them
        return demo_config
