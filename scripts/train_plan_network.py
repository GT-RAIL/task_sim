#!/usr/bin/env python

# ROS
import rospy

from plan_network import PlanNetwork

def construct_plan_network():
    """Construct a plan network and save it as a set of .pkl files for later use."""
    rospy.init_node('train_plan_network')

    task = rospy.get_param('~task', 'task2')
    output_suffix = rospy.get_param('output_suffix', None)
    affordance_threshold = rospy.get_param('affordance_threshold', 0.6)

    network = PlanNetwork()
    network.construct_network(task=task, output_suffix=output_suffix, affordance_threshold=affordance_threshold)
    network.test_output()

    print 'Plan network construction complete!'


if __name__ == '__main__':
    try:
        construct_plan_network()
    except rospy.ROSInterruptException:
        pass
