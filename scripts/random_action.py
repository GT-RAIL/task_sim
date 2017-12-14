#!/usr/bin/env python

# Python
from random import randint

# ROS
from task_sim.msg import Action
from task_sim.srv import SelectAction
from geometry_msgs.msg import Point
import rospy

from data_utils import DataUtils

class RandomAction:

    def __init__(self):
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

        if action.action_type in [Action.PLACE, Action.MOVE_ARM]:
            action.position.x = randint(0, 40)
            action.position.y = randint(0, 15)
            action.position.z = 0

        print str(action)

        return action


if __name__ == '__main__':
    rospy.init_node('random_action')

    random_action = RandomAction()

    rospy.spin()
