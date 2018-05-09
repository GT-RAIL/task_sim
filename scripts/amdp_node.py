#!/usr/bin/env python

# Python
from copy import deepcopy
from math import sqrt
import pickle
from random import random, randint

# ROS
import rospy
import rospkg
from task_sim import data_utils as DataUtils
from task_sim.msg import Action, Status
from task_sim.srv import QueryStatus, SelectAction

from task_sim.oomdp.oo_state import OOState
from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned
from task_sim.str.amdp_reward import reward, is_terminal


t_id_map = {0:0, 1:0, 2:2, -1:2, 3:3, 4:4, 5:5, 6:6, 7:6, 8:8, 9:9, 10:10, 11:11, 12:12}

class AMDPNode:

    def __init__(
        self,
        simulator_name='table_sim',
        transition_functions=None,
        value_tables=None
    ):
        # NOTE: Hardcode the experiment
        self.experiment = 4
        # self.experiment = rospy.get_param('~experiment', 4)
        # experiment id, determines which hardcoded amdp hierarchy to use
        #   0 - 1 object 1 container flat relation mdp
        #   1 - 1 object 1 container amdp
        #   2 - 2 object 1 container amdp
        #   3 - 3 object 1 container amdp

        a_file = rospy.get_param('~actions', rospkg.RosPack().get_path('task_sim') + '/src/task_sim/str/A.pkl')

        self.A = {}
        self.U = (
            { amdp_id: v.U for (amdp_id, v) in value_tables.iteritems() }
            if value_tables is not None else {}
        )
        self.T = transition_functions or {}
        if self.experiment == 0:
            self.A[-1] = pickle.load(file(a_file))
            self.U[-1] = pickle.load(file('trained_U-1.pkl'))
            self.T[2] = AMDPTransitionsLearned(amdp_id=2, load=True)
        elif self.experiment >= 1 and self.experiment <= 4:
            self.A[0] = pickle.load(file(a_file))
            self.A[1] = self.A[0]
            self.A[2] = self.A[0]
            if self.experiment == 4:
                self.A[6] = self.A[0]
                self.A[7] = self.A[0]
                self.A[8] = self.A[0]

            if self.experiment == 1:
                self.A[3] = []
                a = Action()
                a.action_type = 0
                self.A[3].append(deepcopy(a))
                a.action_type = 1
                self.A[3].append(deepcopy(a))
                a.action_type = 2
                self.A[3].append(deepcopy(a))
            elif self.experiment == 2:
                self.A[4] = []
                a = Action()
                a.action_type = 0
                self.A[4].append(deepcopy(a))
                a.action_type = 1
                self.A[4].append(deepcopy(a))
                a.action_type = 2
                a.object = 'apple'
                self.A[4].append(deepcopy(a))
                a.object = 'banana'
                self.A[4].append(deepcopy(a))
            elif self.experiment == 3:
                self.A[5] = []
                a = Action()
                a.action_type = 0
                self.A[5].append(deepcopy(a))
                a.action_type = 1
                self.A[5].append(deepcopy(a))
                a.action_type = 2
                a.object = 'apple'
                self.A[5].append(deepcopy(a))
                a.object = 'banana'
                self.A[5].append(deepcopy(a))
                a.object = 'carrot'
                self.A[5].append(deepcopy(a))
            elif self.experiment == 4:
                self.A[4] = []
                a = Action()
                a.action_type = 0
                self.A[4].append(deepcopy(a))
                a.action_type = 1
                self.A[4].append(deepcopy(a))
                a.action_type = 2
                a.object = 'apple'
                self.A[4].append(deepcopy(a))
                a.object = 'banana'
                self.A[4].append(deepcopy(a))
                self.A[11] = []
                a = Action()
                a.action_type = 6
                self.A[11].append(deepcopy(a))
                a.action_type = 7
                self.A[11].append(deepcopy(a))
                a.action_type = 8
                a.object = 'carrot'
                self.A[11].append(deepcopy(a))
                a.object = 'daikon'
                self.A[11].append(deepcopy(a))
                a = Action()
                self.A[12] = []
                a.action_type = 4
                self.A[12].append(deepcopy(a))
                a.action_type = 11
                self.A[12].append(deepcopy(a))

            if value_tables is None:
                self.U[0] = pickle.load(file('trained_U0.pkl'))
                self.U[1] = pickle.load(file('trained_U1.pkl'))
                self.U[2] = pickle.load(file('trained_U2.pkl'))
                if self.experiment == 1:
                    self.U[3] = pickle.load(file('trained_U3.pkl'))
                elif self.experiment == 2:
                    self.U[4] = pickle.load(file('trained_U4.pkl'))
                elif self.experiment == 3:
                    self.U[5] = pickle.load(file('trained_U5.pkl'))
                elif self.experiment == 4:
                    self.U[4] = pickle.load(file('trained_U4.pkl'))
                    self.U[6] = pickle.load(file('trained_U6.pkl'))
                    self.U[7] = pickle.load(file('trained_U7.pkl'))
                    self.U[8] = pickle.load(file('trained_U8.pkl'))
                    self.U[11] = pickle.load(file('trained_U11.pkl'))
                    self.U[12] = pickle.load(file('trained_U12.pkl'))

            if transition_functions is None:
                self.T[0] = AMDPTransitionsLearned(amdp_id=0, load=True)
                self.T[2] = AMDPTransitionsLearned(amdp_id=2, load=True)
                if self.experiment == 1:
                    self.T[3] = AMDPTransitionsLearned(amdp_id=3, load=False)
                elif self.experiment == 2:
                    self.T[4] = AMDPTransitionsLearned(amdp_id=4, load=False)
                elif self.experiment == 3:
                    self.T[5] = AMDPTransitionsLearned(amdp_id=5, load=False)
                elif self.experiment == 4:
                    self.T[6] = AMDPTransitionsLearned(amdp_id=6, load=True)
                    self.T[8] = AMDPTransitionsLearned(amdp_id=8, load=True)
                    self.T[4] = AMDPTransitionsLearned(amdp_id=4, load=False)
                    self.T[11] = AMDPTransitionsLearned(amdp_id=11, load=False)
                    self.T[12] = AMDPTransitionsLearned(amdp_id=12, load=False)

        # self.T[t_id_map[self.amdp_id]].transition_function(s, a)

        self.service = rospy.Service(simulator_name + '/select_action', SelectAction, self.select_action)
        self.status_service = rospy.Service(simulator_name + '/query_status', QueryStatus, self.query_status)

        print 'AMDP node loaded and initialized.'


    def select_action(self, req):
        action = None

        action_list = []

        oo_state = OOState(state=req.state)
        if self.experiment == 0:
            s = AMDPState(amdp_id=-1, state=oo_state, ground_ites = ['apple'])

            utilities = {}
            for a in self.A[-1]:
                successors = self.T[t_id_map[-1]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[-1]:
                        u += p*self.U[-1][s_prime]
                    elif is_terminal(s_prime, amdp_id=-1):
                        u += p*reward(s_prime, amdp_id=-1)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

        elif self.experiment == 1:
            # start at the top level
            s = AMDPState(amdp_id=3, state=oo_state, ground_items=['apple'])

            utilities = {}
            for a in self.A[3]:
                successors = self.T[t_id_map[3]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[3]:
                        u += p*self.U[3][s_prime]
                    elif is_terminal(s_prime, amdp_id=3):
                        u += p*reward(s_prime, amdp_id=3)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

            # select action
            i = randint(0, len(action_list) - 1)
            id = action_list[i].action_type

            print 'High level action selection: ' + str(id)

            # solve lower level mdp for executable action
            action_list = []
            s = AMDPState(amdp_id=id, state=oo_state, ground_items=['apple'])

            utilities = {}
            for a in self.A[id]:
                successors = self.T[t_id_map[id]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[id]:
                        u += p*self.U[id][s_prime]
                    elif is_terminal(s_prime, amdp_id=id):
                        u += p*reward(s_prime, amdp_id=id)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

        elif self.experiment == 2:
            # start at the top level
            s = AMDPState(amdp_id=4, state=oo_state, ground_items=['apple', 'banana'])

            utilities = {}
            for a in self.A[4]:
                successors = self.T[t_id_map[4]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[4]:
                        u += p*self.U[4][s_prime]
                    elif is_terminal(s_prime, amdp_id=4):
                        u += p*reward(s_prime, amdp_id=4)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

            # select action
            i = randint(0, len(action_list) - 1)
            id = action_list[i].action_type
            obj = action_list[i].object

            print 'High level action selection: ' + str(id) + ', ' + str(obj)

            # solve lower level mdp for executable action
            action_list = []
            s = AMDPState(amdp_id=id, state=oo_state, ground_items=[obj])

            utilities = {}
            for a in self.A[id]:
                successors = self.T[t_id_map[id]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[id]:
                        u += p*self.U[id][s_prime]
                    elif is_terminal(s_prime, amdp_id=id):
                        u += p*reward(s_prime, amdp_id=id)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))
                elif utilities[a] == max_utility:
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))

        elif self.experiment == 3:
            # start at the top level
            s = AMDPState(amdp_id=5, state=oo_state, ground_items=['apple', 'banana', 'carrot'])

            utilities = {}
            for a in self.A[5]:
                successors = self.T[t_id_map[5]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[5]:
                        u += p*self.U[5][s_prime]
                    elif is_terminal(s_prime, amdp_id=5):
                        u += p*reward(s_prime, amdp_id=5)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

            # select action
            i = randint(0, len(action_list) - 1)
            id = action_list[i].action_type
            obj = action_list[i].object

            print 'High level action selection: ' + str(id) + ', ' + str(obj)

            # solve lower level mdp for executable action
            action_list = []
            s = AMDPState(amdp_id=id, state=oo_state, ground_items=[obj])

            utilities = {}
            for a in self.A[id]:
                successors = self.T[t_id_map[id]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[id]:
                        u += p*self.U[id][s_prime]
                    elif is_terminal(s_prime, amdp_id=id):
                        u += p*reward(s_prime, amdp_id=id)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))
                elif utilities[a] == max_utility:
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))

        elif self.experiment == 4:
            # start at the top level
            s = AMDPState(amdp_id=12, state=oo_state)
            utilities = {}
            for a in self.A[12]:
                successors = self.T[t_id_map[12]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[12]:
                        u += p*self.U[12][s_prime]
                    elif is_terminal(s_prime, amdp_id=12):
                        u += p*reward(s_prime, amdp_id=12)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

            # select action
            # i = randint(0, len(action_list) - 1)
            i = 0
            id = action_list[i].action_type
            #obj = action_list[i].object

            print 'Top level action selection: ' + str(id)


            s = AMDPState(amdp_id=id, state=oo_state)

            utilities = {}
            for a in self.A[id]:
                successors = self.T[t_id_map[id]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[id]:
                        u += p*self.U[id][s_prime]
                    elif is_terminal(s_prime, amdp_id=id):
                        u += p*reward(s_prime, amdp_id=id)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action_list.append(deepcopy(a))
                elif utilities[a] == max_utility:
                    action_list.append(deepcopy(a))

            # select action
            # i = randint(0, len(action_list) - 1)
            i = 0
            id = action_list[i].action_type
            obj = action_list[i].object

            print '\tMid level action selection: ' + str(id) + ', ' + str(obj)

            # solve lower level mdp for executable action
            action_list = []
            s = AMDPState(amdp_id=id, state=oo_state, ground_items=[obj])

            utilities = {}
            for a in self.A[id]:
                successors = self.T[t_id_map[id]].transition_function(s, a)
                u = 0
                for i in range(len(successors)):
                    p = successors[i][0]
                    s_prime = successors[i][1]
                    if s_prime in self.U[id]:
                        u += p*self.U[id][s_prime]
                    elif is_terminal(s_prime, amdp_id=id):
                        u += p*reward(s_prime, amdp_id=id)
                utilities[a] = u

            # print '\n---'
            # for key in utilities:
            #     print str(key)
            #     print 'utility: ' + str(utilities[key])

            # pick top action deterministically
            max_utility = -999999
            for a in utilities.keys():
                if utilities[a] > max_utility:
                    max_utility = utilities[a]
                    action_list = []
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))
                elif utilities[a] == max_utility:
                    action = deepcopy(a)
                    if action.object == 'apple':
                        action.object = obj
                    action_list.append(deepcopy(action))

        if len(action_list) > 0:
            # i = randint(0, len(action_list) - 1)
            i = 0
            action = action_list[i]
            print '\t\tLow level action selection: ' + str(action.action_type) + ', ' + str(action.object)
            if action.action_type == Action.PLACE:
                action.position = DataUtils.semantic_action_to_position(req.state, action.object)
                action.object = ''
            elif action.action_type == Action.MOVE_ARM:
                if action.object == 'l':
                    action.position.x = req.state.gripper_position.x - 10
                    action.position.y = req.state.gripper_position.y
                elif action.object == 'fl':
                    action.position.x = req.state.gripper_position.x - 10
                    action.position.y = req.state.gripper_position.y - 5
                elif action.object == 'f':
                    action.position.x = req.state.gripper_position.x
                    action.position.y = req.state.gripper_position.y - 5
                elif action.object == 'fr':
                    action.position.x = req.state.gripper_position.x + 10
                    action.position.y = req.state.gripper_position.y - 5
                elif action.object == 'r':
                    action.position.x = req.state.gripper_position.x + 10
                    action.position.y = req.state.gripper_position.y
                elif action.object == 'br':
                    action.position.x = req.state.gripper_position.x + 10
                    action.position.y = req.state.gripper_position.y + 5
                elif action.object == 'b':
                    action.position.x = req.state.gripper_position.x
                    action.position.y = req.state.gripper_position.y + 5
                elif action.object == 'bl':
                    action.position.x = req.state.gripper_position.x - 10
                    action.position.y = req.state.gripper_position.y + 5
                else:
                    action.position = DataUtils.semantic_action_to_position(req.state, action.object)
                action.object = ''
            elif action.action_type != Action.GRASP:
                action.object = ''
        else:
            action.action_type = Action.NOOP

        # print '\n\n-------------------'
        # print 'Selected action: '
        # print str(action)

        return action

    def query_status(self, req):
        # Check termination criteria
        failed = False
        status = Status()
        status.status_code = Status.IN_PROGRESS
        for object in req.state.objects:
            if object.name.lower() == 'apple':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    break
            if object.name.lower() == 'banana':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    break
            if object.name.lower() == 'carrot':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    break

        if req.state.drawer_opening > 1:
            completed = False
        if req.state.lid_position.x != req.state.box_position.x or req.state.lid_position.y != req.state.box_position.y:
            completed = False

        oo_state = OOState(state=req.state)
        amdp_id = -1
        if self.experiment == 0:
            amdp_id = -1
        elif self.experiment == 1:
            amdp_id = 3
        elif self.experiment == 2:
            amdp_id = 4
        elif self.experiment == 3:
            amdp_id = 5
        elif self.experiment == 4:
            amdp_id = 12
        s = AMDPState(amdp_id=amdp_id, state=oo_state)
        if is_terminal(s, amdp_id=amdp_id):
            status.status_code = Status.COMPLETED

        if failed:
            status.status_code = Status.FAILED
            return status

        return status


if __name__ == '__main__':
    rospy.init_node('amdp_node')

    amdp = AMDPNode()
    print 'Ready to generate actions.'

    rospy.spin()
