#!/usr/bin/env python

# ROS
from task_sim.msg import Action, State
from task_sim.srv import Execute, QueryState, SelectAction
import rospy

class Executor:

    def __init__(self):
        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.select_action = rospy.ServiceProxy('table_sim/select_action', SelectAction)
        self.execute = rospy.ServiceProxy('table_sim/execute_action', Execute)

    def step(self):
        state = self.query_state().state
        action = self.select_action(state).action
        print str(action)
        self.execute(action)


if __name__ == '__main__':
    rospy.init_node('executor')

    executor = Executor()

    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        executor.step()
        loop_rate.sleep()
