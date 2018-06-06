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
from amdp_plan_network import AMDPPlanNetwork

class DemonstrationMode(object):
    """Enum definitions of the demo modes"""
    RANDOM = 1
    SHADOW = 1<<1
    CLASSIFIER = 1<<2
    PLAN_NETWORK = 1<<3

    def __init__(self, mode):
        """`mode` is the bitwise OR of the available modes"""
        self.mode = mode

    @property
    def random(self):
        return (self.mode & DemonstrationMode.RANDOM) > 0

    @property
    def shadow(self):
        return (self.mode & DemonstrationMode.SHADOW) > 0

    @property
    def classifier(self):
        return (self.mode & DemonstrationMode.CLASSIFIER) > 0

    @property
    def plan_network(self):
        return (self.mode & DemonstrationMode.PLAN_NETWORK) > 0

    def configuration(self, **task_config):
        """Returns a dictionary of the pertinent objects given the selected
        mode and the configs for that mode. The configs are passed in as
        keyword arguments.

        TODO: Add configs like alpha and epsilon to the demo mode config?
        """
        demo_config = {}

        # Check each of the modes in turn. Start with Random
        if self.random:
            # Nothing needed for random
            pass

        # Check if we should return a demonstration policy
        if self.shadow:
            # Load the demonstration(s) and create a policy
            container_env = task_config['container_env'] # TODO: Sensible assert
            # assert all(
            #     [(x == 'task4' or x == 'task7') for x in container_env]
            # ), "Unknown task demo: {}".format(container_env)
            amdp_id = task_config['amdp_id']
            assert (amdp_id in [0,1,2,6,7,8]), "Unknown AMDP ID: {}".format(amdp_id)

            print("Loading demonstrations for", container_env, '...')
            demos_list = [] # TODO: In case there are multiple demo folders
            demo_list = glob.glob(os.path.join(
                rospkg.RosPack().get_path('task_sim'),
                'data',
                container_env,
                'demos/*.bag'
            ))
            print("Found", len(demo_list), 'demonstrations for container', container_env)
            demos_list.extend(demo_list)

            sa_pairs = []
            prev_state_msg = None
            for demo_file in demos_list:
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
        if self.classifier:
            # Load the demonstration(s) and create a policy
            container_env = task_config['container_env'] # TODO: Sensible assert
            # assert all(
            #     [(x == 'task4' or x == 'task7') for x in container_env]
            # ), "Unknown task demo: {}".format(container_env)
            amdp_id = task_config['amdp_id']
            assert (amdp_id in [0,1,2,6,7,8]), "Unknown AMDP ID: {}".format(amdp_id)

            if amdp_id == 1:
                classifier_id = 0
            elif amdp_id == 7:
                classifier_id = 6
            else:
                classifier_id = amdp_id
            classifier_name = 'decision_tree_action_{}.pkl'.format(classifier_id)
             #TODO: Perhaps use an experiment folder
            classifier_path = os.path.join(
                rospkg.RosPack().get_path('task_sim'),
                'data',
                container_env, #TODO: Update this to use multiple...
                'models',
                classifier_name
            )
            print("Loading classifier at", classifier_path)
            demo_config['action_bias'] = joblib.load(classifier_path)

            # knn: .20 .16 .18 .17
            # svm: .20 .18 .19 .15
            #
            classifier2_name = 'svm_action_{}.pkl'.format(classifier_id)
             #TODO: Perhaps use an experiment folder
            classifier2_path = os.path.join(
                rospkg.RosPack().get_path('task_sim'),
                'data',
                container_env, #TODO: Update this to use multiple...
                'models',
                classifier2_name
            )
            print("Loading alternate classifier at", classifier2_path)
            demo_config['action_bias_alternate'] = joblib.load(classifier2_path)

        # Check if there's a plan network that we need to return
        if self.plan_network:
            container_env = task_config['container_env']
            amdp_id = task_config['amdp_id']
            if amdp_id == 1:
                pn_id = 0
            elif amdp_id == 7:
                pn_id = 6
            else:
                pn_id = amdp_id
            demo_config['action_sequences'] = AMDPPlanNetwork(task=container_env, amdp_id=pn_id)
            demo_config['action_sequences'].read_graph(task=container_env, suffix='_'+str(pn_id))

        # All configs are loaded. Return them
        return demo_config
