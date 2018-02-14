#!/usr/bin/env python

# Python
import copy
from math import sqrt
from random import randint

# ROS
import rospy
from geometry_msgs.msg import Point
from task_sim.msg import Action, Status
from task_sim.srv import QueryStatus, SelectAction
from task_sim import data_utils as DataUtils

class RandomAction:

    def __init__(self):
        self.semantic_place = rospy.get_param('~semantic_place', False)

        self.state_history = []

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.generate_action)

        self.status_service = rospy.Service('/table_sim/query_status', QueryStatus, self.query_status)

        print 'Ready to generate random actions.'

    def generate_action(self, req):
        """Return binary classification of an ordered grasp pair feature vector."""

        action = Action()

        action.action_type = randint(0, 7)

        if action.action_type == Action.GRASP:
            object = randint(5, 11)
            if object == 5:
                action.object = 'Drawer'
            else:
                action.object = DataUtils.int_to_name(object)

        if action.action_type == Action.PLACE:
            if self.semantic_place:
                action_modifier = randint(1,5)
                if action_modifier == 1:
                    action_modifier = 0
                elif action_modifier == 4:
                    action_modifier = 6
                if DataUtils.int_to_name(action_modifier) == 'Stack':
                    # Pick a random free point on top of the stack of drawers
                    points = []
                    if req.state.drawer_position.theta == 0 or req.state.drawer_position.theta == 180:
                        for x in range(int(req.state.drawer_position.x - 3), int(req.state.drawer_position.x + 4)):
                            for y in range(int(req.state.drawer_position.y - 2), int(req.state.drawer_position.y + 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 3))
                    else:
                        for x in range(int(req.state.drawer_position.x - 2), int(req.state.drawer_position.x + 3)):
                            for y in range(int(req.state.drawer_position.y - 3), int(req.state.drawer_position.y + 4)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 3))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Pick a random point on the table
                        action.position.x = randint(0, 40)
                        action.position.y = randint(0, 15)
                        action.position.z = 0
                elif DataUtils.int_to_name(action_modifier) == 'Drawer':
                    # Pick a random free point in the drawer that's also not in the drawer stack footprint
                    points = []
                    if req.state.drawer_position.theta == 0:
                        for x in range(int(req.state.drawer_position.x + 4), int(req.state.drawer_position.x + req.state.drawer_opening + 3)):
                            for y in range(int(req.state.drawer_position.y - 1), int(req.state.drawer_position.y + 2)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    elif req.state.drawer_position.theta == 180:
                        for x in range(int(req.state.drawer_position.x - req.state.drawer_opening - 2), int(req.state.drawer_position.x - 3)):
                            for y in range(int(req.state.drawer_position.y - 1), int(req.state.drawer_position.y + 2)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    elif req.state.drawer_position.theta == 90:
                        for x in range(int(req.state.drawer_position.x - 1), int(req.state.drawer_position.x + 2)):
                            for y in range(int(req.state.drawer_position.y + 4), int(req.state.drawer_position.y + req.state.drawer_opening + 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    else:
                        for x in range(int(req.state.drawer_position.x - 1), int(req.state.drawer_position.x + 2)):
                            for y in range(int(req.state.drawer_position.y - req.state.drawer_opening - 2), int(req.state.drawer_position.y - 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Pick a random point on the table
                        action.position.x = randint(0, 40)
                        action.position.y = randint(0, 15)
                        action.position.z = 0
                elif DataUtils.int_to_name(action_modifier) == 'Box':
                    # Special case: holding lid
                    if req.state.object_in_gripper.lower() == 'lid':
                        action.position = req.state.box_position
                    else:
                        # Pick a random free point in the box that's also not in the lid footprint
                        points = []
                        for x in range(int(req.state.box_position.x - 1), int(req.state.box_position.x + 2)):
                            for y in range(int(req.state.box_position.y - 1), int(req.state.box_position.y + 2)):
                                if (x >= req.state.lid_position.x - 2 and x <= req.state.lid_position.x + 2
                                    and y >= req.state.lid_position.y - 2 and y <= req.state.lid_position.y + 2):
                                    continue
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z <= 1:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                        if len(points) > 0:
                            action.position = points[randint(0, len(points) - 1)]
                        else:  # Pick a random point on the table
                            action.position.x = randint(0, 40)
                            action.position.y = randint(0, 15)
                            action.position.z = 0
                elif DataUtils.int_to_name(action_modifier) == 'Lid':
                    # Pick a random free point on the lid
                    points = []
                    for x in range(int(req.state.lid_position.x - 2), int(req.state.lid_position.x + 3)):
                        for y in range(int(req.state.lid_position.y - 2), int(req.state.lid_position.y + 3)):
                            clear = True
                            for obj in req.state.objects:
                                if obj.position.x == x and obj.position.y == y and obj.position.z == req.state.lid_position.z:
                                    clear = False
                                    break
                            if clear:
                                points.append(Point(x, y, 2))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Pick a random point on the table
                        action.position.x = randint(0, 40)
                        action.position.y = randint(0, 15)
                        action.position.z = 0
                else:  # Pick a random point on the table
                    action.position.x = randint(0, 40)
                    action.position.y = randint(0, 15)
                    action.position.z = 0
            else:  # Pick a random point on the table
                action.position.x = randint(0, 40)
                action.position.y = randint(0, 15)
                action.position.z = 0

        if action.action_type == Action.MOVE_ARM:
            action.position.x = randint(0, 40)
            action.position.y = randint(0, 15)
            action.position.z = 0

        return action

    def query_status(self, req):
        # Check termination criteria
        completed = True
        failed = False
        status = Status()
        status.status_code = Status.IN_PROGRESS
        for object in req.state.objects:
            if object.name.lower() == 'apple':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_box:
                    completed = False
            elif object.name.lower() == 'flashlight':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_drawer:
                    completed = False
            elif object.name.lower() == 'batteries':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_drawer:
                    completed = False
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
            # Clear history for next intervention
            self.state_history = []

        return status

    def equivalent_state(self, s1, s2):
        return s1.objects == s2.objects and s1.drawer_position == s2.drawer_position \
               and s1.drawer_opening == s2.drawer_opening and s1.box_position == s2.box_position \
               and s1.lid_position == s2.lid_position and s1.gripper_position == s2.gripper_position \
               and s1.gripper_open == s2.gripper_open and s1.object_in_gripper == s2.object_in_gripper


if __name__ == '__main__':
    rospy.init_node('random_action')

    random_action = RandomAction()

    rospy.spin()
