#!/usr/bin/env python

# Python
from copy import deepcopy
from math import sqrt
import pickle
from random import random, randint

# ROS
import rospy
from task_sim import data_utils as DataUtils
from task_sim.msg import Action, Status
from task_sim.srv import QueryStatus, SelectAction

from task_sim.oomdp.oo_state import OOState
from task_sim.str.relation_state import RelationState
from task_sim.str.relation_transitions import transition_function
from task_sim.str.relation_reward import reward, is_terminal

class RelationMDPNode:

    def __init__(self):
        u_file = rospy.get_param('~utilities', 'U_iter_13.pkl')
        a_file = rospy.get_param('~actions', 'A.pkl')

        self.U = pickle.load(file(u_file))
        self.A = pickle.load(file(a_file))

        self.intervention_requested = False
        self.handle_intervention_action = False

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.select_action)
        self.status_service = rospy.Service('/table_sim/query_status', QueryStatus, self.query_status)


    def select_action(self, req):
        action = None

        action_list = []

        oo_state = OOState(state=req.state)
        s = RelationState(state=oo_state)
        print str(s.relations)

        utilities = {}
        for a in self.A:
            successors = transition_function(s, a)
            u = 0
            for i in range(len(successors)):
                p = successors[i][0]
                s_prime = successors[i][1]
                if s_prime in self.U:
                    u += p*self.U[s_prime]
                elif is_terminal(s_prime):
                    u += p*reward(s_prime)
            utilities[a] = u

        print '\n---'
        for key in utilities:
            print str(key)
            print 'utility: ' + str(utilities[key])

        # pick top action deterministically
        max_utility = -999999
        for a in utilities.keys():
            if utilities[a] > max_utility:
                max_utility = utilities[a]
                action_list = []
                action_list.append(deepcopy(a))
            elif utilities[a] == max_utility:
                action_list.append(deepcopy(a))

        if len(action_list) > 0:
            i = randint(0, len(action_list) - 1)
            action = action_list[i]
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
            elif action.action_type != Action.GRASP:
                action.object = ''
        else:
            action.action_type = Action.NOOP

        print '\n\n-------------------'
        print 'Selected action: '
        print str(action)

        return action


    def query_status(self, req):
        # Check termination criteria
        completed = True
        failed = False
        status = Status()
        status.status_code = Status.IN_PROGRESS
        for object in req.state.objects:
            if object.name.lower() == 'apple':
                if not object.in_box:
                    completed = False
                else:
                    continue
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
            elif object.name.lower() == 'flashlight':
                if not object.in_drawer:
                    completed = False
                else:
                    continue
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
            elif object.name.lower() == 'batteries':
                if not object.in_drawer:
                    completed = False
                else:
                    continue
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
        if req.state.drawer_opening > 1:
            completed = False
        if req.state.lid_position.x != req.state.box_position.x or req.state.lid_position.y != req.state.box_position.y:
            completed = False

        if failed:
            status.status_code = Status.FAILED
            return status
        if completed:
            status.status_code = Status.COMPLETED
            return status

        # Check if intervention is required (state repeated 8 times in last 50 actions)
        if self.intervention_requested:
            status.status_code = Status.INTERVENTION_REQUESTED
            self.intervention_requested = False
            self.handle_intervention_action = True
            return status

        return status


if __name__ == '__main__':
    rospy.init_node('relation_mdp_node')

    rmn = RelationMDPNode()
    print 'Ready to generate actions.'

    rospy.spin()
