#!/usr/bin/env python

# Test code to learn transition functions for various AMDPs for the STR project

# Python
from copy import deepcopy
import glob
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
from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned


class LearnTransitionFunction:

    class StochasticAction:

        def __init__(self, a):
            self.actions = [deepcopy(a)]
            self.frequency = [1]
            self.p = [1.0]

        def update(self, a):
            if a in self.actions:
                self.frequency[self.actions.index(a)] += 1
            else:
                self.actions.append(deepcopy(a))
                self.frequency.append(1)
                self.p.append(0.0)

            # Update probabilities
            total = float(sum(self.frequency))
            for i in range(len(self.frequency)):
                self.p[i] = self.frequency[i]/total

        def select_action(self):
            r = random()
            n = 0
            for i in range(len(self.actions)):
                n += self.p[i]
                if n > r:
                    return self.actions[i]
            return self.actions[len(self.actions) - 1]

    def __init__(self):
        # data_file = rospy.get_param('~data', 'state-action_2018-04-20.pkl')
        # self.sa_pairs = pickle.load(file(data_file))

        # Read demo data
        self.task = rospy.get_param('~task', 'task4')

        print 'Loading demonstrations for ' + self.task + '...'
        self.demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + "/demos/*.bag")

        print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

        self.sa_pairs = []
        prev_state_msg = None

        for demo_file in self.demo_list:
            print '\nReading ' + demo_file + '...'
            bag = rosbag.Bag(demo_file)
            for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
                if prev_state_msg is None:
                    prev_state_msg = deepcopy(msg.state)

                elif msg.action.action_type != Action.NOOP:
                    pair = (deepcopy(prev_state_msg), deepcopy(msg.action))
                    self.sa_pairs.append(pair)

                    # update stored data for next iteration
                    prev_state_msg = deepcopy(msg.state)
            bag.close()

        # read action list
        a_file = rospy.get_param('~actions', 'A.pkl')
        self.A = pickle.load(file(a_file))

        self.amdp_id = rospy.get_param('~amdp_id', 2)

        # fill in the policy directly from demonstrations
        self.pi = {}
        for pair in self.sa_pairs:
            state_msg = pair[0]
            s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state_msg))
            a = pair[1]

            # convert action into something that fits into the new action list
            if a.action_type == Action.PLACE:
                a.object = DataUtils.get_task_frame(state_msg, a.position)
                a.position = Point()
            elif a.action_type == Action.MOVE_ARM:
                a.object = DataUtils.get_task_frame(state_msg, a.position)
                if a.object != 'stack' and a.object != 'drawer':
                    for o in state_msg.objects:
                        if o.name != 'apple':
                            continue
                        if a.position == o.position:
                            a.object = 'apple'
                            break
                    if a.object != 'apple':
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
            else:
                a.position = Point()

            if s in self.pi:
                self.pi[s].update(a)
            else:
                self.pi[s] = self.StochasticAction(a)

        self.transition_function = AMDPTransitionsLearned(amdp_id=self.amdp_id)

        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.execute_action = rospy.ServiceProxy('table_sim/execute_action', Execute)
        self.reset_sim = rospy.ServiceProxy('table_sim/reset_simulation', Empty)

        self.n = 0  # number of executions
        self.consecutive_random_count = 0  # number of times a random action was chosen consecutively
        self.prev_state = None
        self.timeout = 0

    def run(self):
        state_msg = self.query_state().state
        s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state_msg))

        self.timeout += 1

        if self.consecutive_random_count > 10 or self.timeout > 100 or goal_check(state_msg):
            self.consecutive_random_count = 0
            self.timeout = 0
            self.reset_sim()
            return

        if s in self.pi:
            a = self.pi[s].select_action()
        else:
            self.consecutive_random_count += 1
            a = self.A[randint(0, len(self.A) - 1)]

        self.execute_action(action_to_sim(deepcopy(a), state_msg))
        s_prime = AMDPState(amdp_id=self.amdp_id, state=OOState(state=self.query_state().state))

        self.transition_function.update_transition(s, a, s_prime)
        self.n += 1
        self.prev_state = deepcopy(s)


def goal_check(s):
    for o in s.objects:
        if o.name == 'apple':
            if not o.in_drawer:
                return False
    if s.drawer_opening > 1:
        return False
    if s.object_in_gripper == 'drawer':
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

    while ltf.n <= 5000000:
        # rospy.sleep(0.01)
        ltf.run()
        if ltf.n % 250000 == 0:
            print 'Iteration: ' + str(ltf.n)
            print '\tTransition function state-action count: ' + str(len(ltf.transition_function.transition.keys()))
            ltf.transition_function.save('_' + str(ltf.n))
