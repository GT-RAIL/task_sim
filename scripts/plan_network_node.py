#!/usr/bin/env python

# Python
import copy
from math import sqrt
from random import random, randint

# ROS
from task_sim.msg import Action, Status
from task_sim.srv import QueryStatus, SelectAction
import rospy

from data_utils import DataUtils
from plan_network import PlanNetwork
from plan_action import PlanAction

class PlanNetworkNode:

    def __init__(self):
        """Initialize action selection from plan networks as a service in a ROS node."""
        task = rospy.get_param('~task', 'task3')
        suffix = rospy.get_param('output_suffix', '_2018-01-30')

        self.network = PlanNetwork()
        self.network.read_graph(task=task, suffix=suffix)

        #self.network.test_output()

        self.prev_node = 'start'
        self.current_node = 'start'
        self.prev_state = None
        self.prev_action = None
        self.remaining_actions = []

        self.state_history = []

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.select_action)
        self.status_service = rospy.Service('/table_sim/query_status', QueryStatus, self.query_status)


    def select_action(self, req):
        """Return an action generated from the plan network."""

        action = Action()

        action_list = []

        # check if we are correctly at the current node based on action effects
        if self.current_node != 'start':
            actual_node = self.network.generalize_action(PlanAction(self.prev_state, self.prev_action, req.state))
            if actual_node != self.current_node:
                print 'Unexpected effects!  Updating current node... (Note: this node may not be in the graph!)'
                if self.network.has_node(actual_node):
                    self.current_node = actual_node
                else:
                    # see if there are remaining actions available
                    if len(self.remaining_actions) > 0:
                        # see if remaining actions have valid preconditions
                        valid_remaining_actions = []
                        norm = 0
                        for act in self.remaining_actions:
                            if act[0].check_preconditions(req.state, act[1], act[2], self.network.object_to_cluster):
                                valid_remaining_actions.append(act)
                                norm += act[3]
                        self.remaining_actions = []
                        if len(valid_remaining_actions) > 0:
                            for act in valid_remaining_actions:
                                act[3] /= float(norm)
                            action_list = valid_remaining_actions
                        else:
                            self.current_node = self.network.find_suitable_node(req.state)
                    self.current_node = self.network.find_suitable_node(req.state)
            else:
                print 'Expected effects match.'

        if self.current_node is None:
            action.action_type = Action.NOOP
            # TODO: intervention request
            return action
        elif len(action_list) == 0:
            action_list = self.network.get_successor_actions(self.current_node, req.state)

        # check if there were no successors
        if len(action_list) == 0:
            self.current_node = self.network.find_suitable_node(req.state)
            if self.current_node is None:
                action.action_type = Action.NOOP
                # TODO: intervention request
                return action
            action_list = self.network.get_successor_actions(self.current_node, req.state)

        #print '\n\nAction list: '
        #print str(action_list)

        if len(action_list) > 0:
            selection = random()
            count = 0
            selected_action = action_list[0]
            for i in range(len(action_list)):
                count += action_list[i][3]
                if count >= selection:
                    selected_action = action_list[i]
                    break
            action.action_type = selected_action[0].action
            if action.action_type == Action.GRASP:
                action.object = selected_action[1]
                if len(action.object) > 0:
                    action.object = action.object[0].upper() + action.object[1:]
            elif action.action_type == Action.PLACE or action.action_type == Action.MOVE_ARM:
                action.position = DataUtils.semantic_action_to_position(req.state, selected_action[2])
            self.prev_node = copy.deepcopy(self.current_node)
            self.current_node = copy.deepcopy(selected_action[0])
            self.prev_state = copy.deepcopy(req.state)
            self.prev_action = copy.deepcopy(action)
            self.remaining_actions = action_list
            norm = selected_action[3]
            self.remaining_actions.remove(selected_action)
        else:
            print 'Still no actions!'
            action.action_type = Action.NOOP
            # TODO: intervention request


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
        if req.state.drawer_opening > 0:
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
        self.state_history.append(copy.deepcopy(req.state))
        self.state_history = self.state_history[-50:]
        repeat = 0
        for state in self.state_history:
            if self.equivalent_state(state, req.state):
                repeat += 1
        if repeat >= 8:
            status.status_code = Status.INTERVENTION_REQUESTED
            # Clear state history for next intervention
            self.state_history = []

        return status

    def equivalent_state(self, s1, s2):
        return s1.objects == s2.objects and s1.drawer_position == s2.drawer_position \
               and s1.drawer_opening == s2.drawer_opening and s1.box_position == s2.box_position \
               and s1.lid_position == s2.lid_position and s1.gripper_position == s2.gripper_position \
               and s1.gripper_open == s2.gripper_open and s1.object_in_gripper == s2.object_in_gripper

if __name__ == '__main__':
    rospy.init_node('plan_network_node')

    plan_network_node = PlanNetworkNode()
    print 'Ready to generate actions.'

    rospy.spin()
