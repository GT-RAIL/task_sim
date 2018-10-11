#!/usr/bin/env python

# Test code to learn transition functions for various AMDPs for the STR project

# Python
from copy import deepcopy
import datetime
import glob
import numpy as np
import pickle
from random import random, randint

# ROS
import rosbag
import rospkg
import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from task_sim import data_utils as DataUtils
from task_sim.msg import Action
from task_sim.srv import QueryState, Execute, QueryStatus, SelectAction

from task_sim.oomdp.oo_state import OOState
from task_sim.str.modes import DemonstrationMode
from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned
from task_sim.amdp_plan_action import AMDPPlanAction

# scikit-learn
from sklearn.externals import joblib

class LearnTransitionFunction:

    def __init__(
        self,
        amdp_id=2, # Correlates to the id's in amdp_state
        container_env='task4', # Container in the environment
        simulator_node='table_sim', # The name of the table_sim environment
        transition_function=None, # If the transitions are init elsewhere
        demo_mode=None, # DemonstrationMode object. If None, RANDOM+CLASSIFIER+SHADOW
        demo_config=None, # Config from demonstrations. If None, use the default mode
        max_episode_length=100, # Max. length of an episode
        exploit_policy=False # Exploit learned policy using AMDP node to tradeoff with exploration
    ):
        self.demo_mode = demo_mode or DemonstrationMode(
            DemonstrationMode.RANDOM | DemonstrationMode.CLASSIFIER | DemonstrationMode.SHADOW
        )

        # data_file = rospy.get_param('~data', 'state-action_2018-04-20.pkl')
        # sa_pairs = pickle.load(file(data_file))

        # parameters for controlling exploration. # TODO: fetch through the demo mode
        self.alpha = 0.5  # directly following demonstrations vs. random exploration
        self.epsilon = 0.5  # random exploration vs. general policy guided exploration
        self.exploit_epsilon = 1.0

        self.epoch = 0
        self.successes = 0
        self.action_executions = 0

        # Read demo data and config
        self.container_env = rospy.get_param('~container_env', container_env)
        self.amdp_id = rospy.get_param('~amdp_id', amdp_id)
        self.demo_config = demo_config or self.demo_mode.configuration(
            amdp_id=self.amdp_id,
            container_env=self.container_env
        )

        # Set the transition function
        self.transition_function = transition_function or AMDPTransitionsLearned(amdp_id=self.amdp_id)

        # read action list
        if self.amdp_id >= 0 and self.amdp_id <= 2:
            a_file = rospy.get_param('~actions', rospkg.RosPack().get_path('task_sim')
                                     + '/src/task_sim/str/A_drawer.pkl')
        else:
            a_file = rospy.get_param('~actions', rospkg.RosPack().get_path('task_sim')
                                     + '/src/task_sim/str/A_box.pkl')
        self.A = pickle.load(file(a_file))

        if self.amdp_id == -2:
            a = Action()
            a.action_type = 0
            a.object = 'banana'
            self.A.append(deepcopy(a))
            a.action_type = 4
            self.A.append(deepcopy(a))
        elif self.amdp_id == -3:
            a = Action()
            a.action_type = 0
            a.object = 'banana'
            self.A.append(deepcopy(a))
            a.action_type = 4
            self.A.append(deepcopy(a))
            a.object = 'carrot'
            self.A.append(deepcopy(a))
            a.action_type = 0
            self.A.append(deepcopy(a))

        if self.amdp_id >= 6 and self.amdp_id <= 8:
            a = Action()
            a.action_type = 0
            a.object = 'lid'
            self.A.append(deepcopy(a))
            a.action_type = 4
            self.A.append(deepcopy(a))
            a.object = 'box'
            self.A.append(deepcopy(a))
            a.action_type = 1
            self.A.append(deepcopy(a))
            a.object = 'lid'
            self.A.append(deepcopy(a))

        # fill in the policy directly from demonstrations (if demo_mode calls for it)
        if self.demo_mode.shadow:
            self.pi = self.demo_config.get('demo_policy')

        # load weak classifier to bias random exploration (if demo_mode calls for it)
        if self.demo_mode.classifier:
            self.action_bias = self.demo_config.get('action_bias')
            self.action_bias_alternate = self.demo_config.get('action_bias_alternate')

        # load plan network to bias random exploration (if demo_mode calls for it)
        if self.demo_mode.plan_network:
            self.action_sequences = self.demo_config.get('action_sequences')

        # Setup the services
        self.query_state = rospy.ServiceProxy(simulator_node + '/query_state', QueryState)
        self.execute_action = rospy.ServiceProxy(simulator_node + '/execute_action', Execute)
        self.reset_sim = rospy.ServiceProxy(simulator_node + '/reset_simulation', Empty)

        self.n = 0  # number of executions
        self.prev_state = None
        self.timeout = 0
        self.max_episode_length = max_episode_length

        if self.demo_mode.plan_network:
            self.current_node = 'start'
            self.prev_state_msg = None
            self.prev_action = None

        self.exploit_policy = exploit_policy
        if self.exploit_policy:
            self.query_status = rospy.ServiceProxy(simulator_node + '/query_status', QueryStatus),
            self.select_action = rospy.ServiceProxy(simulator_node + '/select_action', SelectAction)

    def run(self):
        state_msg = self.query_state().state
        s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state_msg))

        self.timeout += 1

        goal_reached = goal_check(state_msg, self.amdp_id)
        if self.timeout > self.max_episode_length or goal_reached:
            self.timeout = 0
            # self.reset_sim()
            self.epoch += 1
            if goal_reached:
                self.successes += 1
            if self.demo_mode.plan_network:
                self.current_node = 'start'
                self.prev_state_msg = None
                self.prev_action = None
            return

        exploit_check = random()
        if self.exploit_policy and exploit_check > self.exploit_epsilon:
            a = self.select_action(state_msg, Action()).action
        else:
            # plan network exploration, behavior implemented individually to stop conditionals from getting crazy
            if self.demo_mode.plan_network:
                # determine the current node in the plan network
                if self.prev_state_msg is None or self.prev_action is None:
                    self.current_node = 'start'
                else:
                    self.current_node = AMDPPlanAction(self.prev_state_msg, self.prev_action, state_msg, self.amdp_id)

                # select action
                a = Action()
                if self.demo_mode.classifier:
                    if random() < self.alpha:
                        action_list = []
                        if self.action_sequences.has_node(self.current_node):
                            action_list = self.action_sequences.get_successor_actions(self.current_node, state_msg)
                        else:
                            self.current_node = self.action_sequences.find_suitable_node(state_msg)
                            if self.current_node is not None:
                                action_list = self.action_sequences.get_successor_actions(self.current_node, state_msg)

                        # select action stochastically if we're in the network, select randomly otherwise
                        if len(action_list) == 0:
                            a = self.A[randint(0, len(self.A) - 1)]
                        else:
                            selection = random()
                            count = 0
                            selected_action = action_list[0]
                            for i in range(len(action_list)):
                                count += action_list[i][1]
                                if count >= selection:
                                    selected_action = action_list[i]
                                    break
                            a.action_type = selected_action[0].action_type
                            a.object = selected_action[0].action_object
                    else:
                        if self.demo_mode.classifier:
                            if self.demo_mode.random and random() <= self.epsilon:
                                a = self.A[randint(0, len(self.A) - 1)]
                            else:
                                features = s.to_vector()

                                # Classify action
                                probs = self.action_bias.predict_proba(np.asarray(features).reshape(1, -1)).flatten().tolist()
                                selection = random()
                                cprob = 0
                                action_label = '0:apple'
                                for i in range(0, len(probs)):
                                    cprob += probs[i]
                                    if cprob >= selection:
                                        action_label = self.action_bias.classes_[i]
                                        break
                                # Convert back to action
                                a = Action()
                                result = action_label.split(':')
                                a.action_type = int(result[0])
                                if len(result) > 1:
                                    a.object = result[1]
                        else:
                            a = self.A[randint(0, len(self.A) - 1)]
                else:
                    # select from the plan network, with a chance of random exploration, and use random exploration when
                    # off of the network
                    if random() < self.alpha:
                        action_list = []
                        if self.action_sequences.has_node(self.current_node):
                            action_list = self.action_sequences.get_successor_actions(self.current_node, state_msg)
                        else:
                            self.current_node = self.action_sequences.find_suitable_node(state_msg)
                            if self.current_node is not None:
                                action_list = self.action_sequences.get_successor_actions(self.current_node, state_msg)

                        # select action stochastically if we're in the network, select randomly otherwise
                        if len(action_list) == 0:
                            a = self.A[randint(0, len(self.A) - 1)]
                        else:
                            selection = random()
                            count = 0
                            selected_action = action_list[0]
                            for i in range(len(action_list)):
                                count += action_list[i][1]
                                if count >= selection:
                                    selected_action = action_list[i]
                                    break
                            a.action_type = selected_action[0].action_type
                            a.object = selected_action[0].action_object
                    else:
                        a = self.A[randint(0, len(self.A) - 1)]

                self.prev_state_msg = state_msg  # store state for the next iteration
                self.prev_action = action_to_sim(deepcopy(a), state_msg)

            else:
                if self.demo_mode.shadow and s in self.pi:
                    if random() < self.alpha:
                        a = self.pi[s].select_action()
                    else:
                        a = self.A[randint(0, len(self.A) - 1)]
                else:
                    if self.demo_mode.classifier:
                        # if random() < self.alpha:
                        if self.demo_mode.random and random() <= self.epsilon:
                            a = self.A[randint(0, len(self.A) - 1)]
                        else:
                            features = s.to_vector()

                            # Classify action
                            probs = self.action_bias.predict_proba(np.asarray(features).reshape(1, -1)).flatten().tolist()
                            selection = random()
                            cprob = 0
                            action_label = '0:apple'
                            for i in range(0, len(probs)):
                                cprob += probs[i]
                                if cprob >= selection:
                                    action_label = self.action_bias.classes_[i]
                                    break
                            # Convert back to action
                            a = Action()
                            result = action_label.split(':')
                            a.action_type = int(result[0])
                            if len(result) > 1:
                                a.object = result[1]
                        # else:
                        #     if self.demo_mode.random and random() <= self.epsilon:
                        #         a = self.A[randint(0, len(self.A) - 1)]
                        #     else:
                        #         features = s.to_vector()
                        #
                        #         # Classify action
                        #         probs = self.action_bias.predict_proba(np.asarray(features).reshape(1, -1)).flatten().tolist()
                        #         selection = random()
                        #         cprob = 0
                        #         action_label = '0:apple'
                        #         for i in range(0, len(probs)):
                        #             cprob += probs[i]
                        #             if cprob >= selection:
                        #                 action_label = self.action_bias.classes_[i]
                        #                 break
                        #         # Convert back to action
                        #         a = Action()
                        #         result = action_label.split(':')
                        #         a.action_type = int(result[0])
                        #         if len(result) > 1:
                        #             a.object = result[1]
                    else:
                        a = self.A[randint(0, len(self.A) - 1)]

        self.execute_action(action_to_sim(deepcopy(a), state_msg))
        s_prime = AMDPState(amdp_id=self.amdp_id, state=OOState(state=self.query_state().state))
        self.action_executions += 1

        self.transition_function.update_transition(s, a, s_prime)
        self.n += 1
        self.prev_state = deepcopy(s)


def goal_check(s, amdp_id=0):
    if amdp_id < 6: # Drawer / Flat task
        for o in s.objects:
            if o.name == 'apple':
                if not o.in_drawer:
                    return False
            if amdp_id == -2 or amdp_id == -3:
                if o.name == 'banana':
                    if not o.in_drawer:
                        return False
            if amdp_id == -3:
                if o.name == 'carrot':
                    if not o.in_drawer:
                        return False
        if s.drawer_opening > 1:
            return False
        if s.object_in_gripper == 'drawer':
            return False
    else: # Box task
        for o in s.objects:
            if o.name == 'apple':
                if not o.in_box:
                    return False
        if not (s.box_position.x == s.lid_position.x and s.box_position.y == s.lid_position.y
                and s.lid_position.z == 1):
            return False
        if s.object_in_gripper == 'lid':
            return False

    return True


def action_to_sim(action, state):
    if action.action_type == Action.PLACE:
        action.position = DataUtils.semantic_action_to_position(state, action.object)
        action.object = ''
    elif action.action_type == Action.MOVE_ARM:
        if action.object == 'l':
            action.position.x = state.gripper_position.x - 10
            action.position.y = state.gripper_position.y
        elif action.object == 'fl':
            action.position.x = state.gripper_position.x - 10
            action.position.y = state.gripper_position.y - 5
        elif action.object == 'f':
            action.position.x = state.gripper_position.x
            action.position.y = state.gripper_position.y - 5
        elif action.object == 'fr':
            action.position.x = state.gripper_position.x + 10
            action.position.y = state.gripper_position.y - 5
        elif action.object == 'r':
            action.position.x = state.gripper_position.x + 10
            action.position.y = state.gripper_position.y
        elif action.object == 'br':
            action.position.x = state.gripper_position.x + 10
            action.position.y = state.gripper_position.y + 5
        elif action.object == 'b':
            action.position.x = state.gripper_position.x
            action.position.y = state.gripper_position.y + 5
        elif action.object == 'bl':
            action.position.x = state.gripper_position.x - 10
            action.position.y = state.gripper_position.y + 5
        else:
            action.position = DataUtils.semantic_action_to_position(state, action.object)
        action.object = ''
    elif action.action_type != Action.GRASP:
        action.object = ''

    return action


if __name__ == '__main__':
    rospy.init_node('learn_transition_function_node')

    ltf = LearnTransitionFunction()

    start = datetime.datetime.now()

    last_epoch = ltf.epoch
    while ltf.epoch <= 500000:
        # rospy.sleep(0.01)
        ltf.run()
        if last_epoch != ltf.epoch:
            ltf.reset_sim()
            last_epoch = ltf.epoch
        if ltf.n % 10000 == 0:
            print 'Iteration: ' + str(ltf.n) + ' (epochs completed: ' + str(ltf.epoch) + ', ' + str(ltf.successes) + ' successes)'
            print '\tTransition function state-action count: ' + str(len(ltf.transition_function.transition.keys()))
            ltf.transition_function.save('_' + str(ltf.n))
            print 'Elapsed time: ' + str(datetime.datetime.now() - start)

    ltf.transition_function.save('_' + str(ltf.n))
    print 'Final runtime: ' + str(datetime.datetime.now() - start)
