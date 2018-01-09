#!/usr/bin/env python

# Python
from math import floor
from random import randint

# ROS
from task_sim.msg import Action
from task_sim.srv import SelectAction
from geometry_msgs.msg import Point
import rospy

from data_utils import DataUtils

class RandomAction:

    def __init__(self):
        self.semantic_place = rospy.get_param('~semantic_place', False)

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.generate_action)

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
                print 'Placing in ' + DataUtils.int_to_name(action_modifier)
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

        print str(action)

        return action


if __name__ == '__main__':
    rospy.init_node('random_action')

    random_action = RandomAction()

    rospy.spin()
