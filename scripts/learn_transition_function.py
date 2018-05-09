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
from task_sim.srv import QueryState, Execute

from task_sim.oomdp.oo_state import OOState
from task_sim.str.stochastic_state_action import StochasticAction
from task_sim.str.demo_mode import DemonstrationMode
from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned

# scikit-learn
from sklearn.externals import joblib

class LearnTransitionFunction:

    def __init__(
        self,
        amdp_id=2, # Correlates to the id's in amdp_state
        demo_task='task4', # Demonstrations task(s). TODO: Update with multiple?
        simulator_node='table_sim', # The name of the table_sim environment
        transition_function=None, # If the transitions are init elsewhere
        demo_mode=None # DemonstrationMode object. If None, RANDOM+CLASSIFIER+SHADOW
    ):
        self.demo_mode = demo_mode or DemonstrationMode(
            DemonstrationMode.RANDOM | DemonstrationMode.CLASSIFIER | DemonstrationMode.SHADOW
        )

        # data_file = rospy.get_param('~data', 'state-action_2018-04-20.pkl')
        # sa_pairs = pickle.load(file(data_file))

        # parameters for controlling exploration. # TODO: fetch through the demo mode
        self.alpha = 0.8  # directly following demonstrations vs. random exploration
        self.epsilon = 0.5  # random exploration vs. general policy guided exploration

        self.epoch = 0
        self.successes = 0

        # Read demo data and config
        self.demo_task = rospy.get_param('~task', demo_task)
        self.amdp_id = rospy.get_param('~amdp_id', amdp_id)
        self.demo_config = self.demo_mode.configuration(
            amdp_id=self.amdp_id,
            demo_task=self.demo_task
        )

        # Set the transition function
        self.transition_function = transition_function or AMDPTransitionsLearned(amdp_id=self.amdp_id)

        # read action list
        a_file = rospy.get_param('~actions', rospkg.RosPack().get_path('task_sim') + '/src/task_sim/str/A.pkl')
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
        self.pi = self.demo_config.get('demo_policy')
        # print 'Loading demonstrations for ' + self.demo_task + '...'
        # demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.demo_task + "/demos/*.bag")

        # print 'Found ' + str(len(demo_list)) + ' demonstrations.'

        # sa_pairs = []
        # prev_state_msg = None

        # for demo_file in demo_list:
        #     print '\nReading ' + demo_file + '...'
        #     bag = rosbag.Bag(demo_file)
        #     for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
        #         if prev_state_msg is None:
        #             prev_state_msg = deepcopy(msg.state)

        #         elif msg.action.action_type != Action.NOOP:
        #             pair = (deepcopy(prev_state_msg), deepcopy(msg.action))
        #             sa_pairs.append(pair)

        #             # update stored data for next iteration
        #             prev_state_msg = deepcopy(msg.state)
        #     bag.close()

        # self.pi = {}
        # for pair in sa_pairs:
        #     state_msg = pair[0]
        #     s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state_msg))
        #     a = pair[1]

        #     # convert action into something that fits into the new action list
        #     if a.action_type == Action.PLACE:
        #         a.object = DataUtils.get_task_frame(state_msg, a.position)
        #         a.position = Point()
        #     elif a.action_type == Action.MOVE_ARM:
        #         a.object = DataUtils.get_task_frame(state_msg, a.position)
        #         if a.object != 'stack' and a.object != 'drawer' and a.object != 'box' and a.object != 'lid':
        #             for o in state_msg.objects:
        #                 if o.name != 'apple' or o.name != 'banana' or o.name != 'carrot':
        #                     continue
        #                 if a.position == o.position:
        #                     a.object = o.name
        #                     break

        #             if a.object != 'apple' or a.object != 'banana' or a.object != 'carrot':
        #                 x = state_msg.gripper_position.x
        #                 y = state_msg.gripper_position.y
        #                 px = a.position.x
        #                 py = a.position.y
        #                 if px == x and py > y:
        #                     a.object = 'b'
        #                 elif px < x and py > y:
        #                     a.object = 'bl'
        #                 elif px < x and py == y:
        #                     a.object = 'l'
        #                 elif px < x and py < y:
        #                     a.object = 'fl'
        #                 elif px == x and py < y:
        #                     a.object = 'f'
        #                 elif px > x and py < y:
        #                     a.object = 'fr'
        #                 elif px > x and py == y:
        #                     a.object = 'r'
        #                 else:
        #                     a.object = 'br'
        #         a.position = Point()
        #     elif a.action_type == Action.GRASP:
        #         a.position = Point()
        #     else:
        #         a.position = Point()
        #         a.object = ''

        #     if s in self.pi:
        #         self.pi[s].update(a)
        #     else:
        #         self.pi[s] = StochasticAction(a)

        # load weak classifier to bias random exploration (if demo_mode calls for it)
        self.action_bias = self.demo_config.get('action_bias')
        # classifier_path = rospy.get_param('~classifier_name', 'decision_tree_action_' + str(self.amdp_id) + '.pkl')
        # classifier_path = rospkg.RosPack().get_path('task_sim') + '/data/' + self.demo_task + '/models/' + classifier_path

        # self.action_bias = joblib.load(classifier_path)

        # Setup the services
        self.query_state = rospy.ServiceProxy(simulator_node + '/query_state', QueryState)
        self.execute_action = rospy.ServiceProxy(simulator_node + '/execute_action', Execute)
        self.reset_sim = rospy.ServiceProxy(simulator_node + '/reset_simulation', Empty)

        self.n = 0  # number of executions
        self.prev_state = None
        self.timeout = 0

    def run(self):
        state_msg = self.query_state().state
        s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state_msg))

        self.timeout += 1

        goal_reached = goal_check(state_msg, self.amdp_id)
        if self.timeout > 100 or goal_reached:
            self.timeout = 0
            self.reset_sim()
            self.epoch += 1
            if goal_reached:
                self.successes += 1
            return

        if s in self.pi:
            if random() < self.alpha:
                a = self.pi[s].select_action()
            else:
                a = self.A[randint(0, len(self.A) - 1)]
        else:
            if random() > self.epsilon:
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

        self.execute_action(action_to_sim(deepcopy(a), state_msg))
        s_prime = AMDPState(amdp_id=self.amdp_id, state=OOState(state=self.query_state().state))

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

    while ltf.epoch <= 500000:
        # rospy.sleep(0.01)
        ltf.run()
        if ltf.n % 10000 == 0:
            print 'Iteration: ' + str(ltf.n) + ' (epochs completed: ' + str(ltf.epoch) + ', ' + str(ltf.successes) + ' successes)'
            print '\tTransition function state-action count: ' + str(len(ltf.transition_function.transition.keys()))
            ltf.transition_function.save('_' + str(ltf.n))
            print 'Elapsed time: ' + str(datetime.datetime.now() - start)

    ltf.transition_function.save('_' + str(ltf.n))
    print 'Final runtime: ' + str(datetime.datetime.now() - start)
